// SPDX-License-Identifier: GPL-2.0
/*
 * ISP tuning bin loader: the in-kernel equivalent of libmi_isp
 * MI_ISP_API_CmdLoadBinFile (OpenIPC /etc/sensors/imx415.bin etc.).
 *
 * Userspace normally pushes the file through libmi_isp -> libcus3a agent
 * -> MI_VPE_SetIspApiData -> mi_vpe ioctl -> MHalIspApiSet -> mhal
 * MS_CAM_IspApiSet. Without libcus3a (or with an agent that swallows the
 * calls) the kernel AE keeps the iqfile0.bin defaults (max shutter 14 ms,
 * max gain 8x). This module reads the bin with request_firmware() and calls
 * MHalIspApiSet() directly for every entry, like MI_ISP_ALGO_API_
 * CmdLoadBinFile + MI_ISP_API_EXCUTE_CMD do [V disasm, libmi_isp 0xe944].
 *
 * File format (little endian):
 *   header, 32 bytes: u32 magic 0x6d650321, u32 version (major << 16 |
 *     minor), u32 size of the entries, u32 checksum, u32 key (1234),
 *     u32 reserved, u32 checksum enabled (low byte), u32 reserved
 *   entries from offset 32, `size` bytes in total:
 *     u32 key (1234), u32 type (5 = ISP API set), u32 len, payload[len]
 *   payload: u16 api id, u16 argument count n, u32 offset[n], data; the
 *     argument i is at (payload + 4 + 4 * n + offset[i]). All entries use
 *     one contiguous struct starting at offset[0], whose size is the size
 *     of the API struct (table below).
 *   "CRC32" (MI_ISP_API_CRC32) is not a CRC: ~(0xffffffff - sum bytes),
 *   i.e. the plain 32-bit sum of the data bytes.
 *
 * Usage:
 *   insmod osm_isp.ko [isp_bin=sstar/imx415.bin isp_ch=0]
 *   echo "0 sstar/imx415.bin" > /sys/module/osm_isp/parameters/load
 * The bin has to be applied after the ISP channel exists (vendor: Divinus
 * calls it after the pipeline is running); from osm_v4l2 call
 * osm_isp_load_bin() once the channel is created.
 */
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "osm_compat.h"
#include "osm_isp.h"
#include "osm_sstar.h"

/* mhal exports, vendor ABI: return 0 or a 0xa007xxxx error code */
extern int MHalIspApiSet(u32 ch, u32 id, u32 size, void *buf);
extern int MHalIspApiGet(u32 ch, u32 id, u32 size, void *buf);

#define ISP_BIN_MAGIC		0x6d650321
#define ISP_BIN_KEY		1234
#define ISP_BIN_TYPE_API	5
#define ISP_API_VERSIONINFO	0x1001

struct isp_bin_hdr {
	u32 magic;
	u32 version;
	u32 size;
	u32 sum;
	u32 key;
	u32 reserved;
	u32 sum_en;
	u32 reserved2;
};

struct isp_bin_ent {
	u32 key;
	u32 type;
	u32 len;
};

/* API ids executed by the vendor (call_MI_ISP_* table) and the size of the
 * kernel struct; other entries (0x2803, 0x3000, 0x3001, ...) are skipped. */
static const struct {
	u16 id;
	u16 size;
} isp_api[] = {
#include "osm_isp_table.h"
};

static char isp_bin[128];
module_param_string(isp_bin, isp_bin, sizeof(isp_bin), 0444);
MODULE_PARM_DESC(isp_bin, "firmware name loaded at insmod (e.g. sstar/imx415.bin)");
static uint isp_ch;
module_param(isp_ch, uint, 0444);
MODULE_PARM_DESC(isp_ch, "ISP channel for isp_bin");

static DEFINE_MUTEX(isp_lock);

static int isp_api_size(u32 id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp_api); i++)
		if (isp_api[i].id == id)
			return isp_api[i].size;
	return -1;
}

static int isp_set(u32 ch, u32 id, u32 size, void *buf)
{
	struct osm_entry e;
	int ret;

	osm_enter(&e);
	ret = MHalIspApiSet(ch, id, size, buf);
	osm_leave(&e);
	return ret;
}

static int isp_get(u32 ch, u32 id, u32 size, void *buf)
{
	struct osm_entry e;
	int ret;

	osm_enter(&e);
	ret = MHalIspApiGet(ch, id, size, buf);
	osm_leave(&e);
	return ret;
}

static int isp_bin_apply(u32 ch, const u8 *d, size_t n)
{
	const struct isp_bin_hdr *h = (const void *)d;
	u32 sum = 0, ver[3] = {};
	size_t p, i;
	unsigned int done = 0, skipped = 0, failed = 0;
	int ret;

	if (n < sizeof(*h) || h->magic != ISP_BIN_MAGIC ||
	    h->size > n - sizeof(*h))
		return -EINVAL;
	if ((h->sum_en & 0xff)) {
		for (i = 0; i < h->size; i++)
			sum += d[sizeof(*h) + i];
		if (sum != h->sum) {
			pr_err("osm_isp: checksum 0x%x, expected 0x%x\n", sum, h->sum);
			return -EBADMSG;
		}
	}
	if (h->key != ISP_BIN_KEY)
		return -EINVAL;

	/* IQ API version of the kernel must match the file (major) */
	ret = isp_get(ch, ISP_API_VERSIONINFO, sizeof(ver), ver);
	if (ret) {
		pr_err("osm_isp: ch %u: version query failed 0x%x (no ISP channel?)\n", ch, ret);
		return -ENODEV;
	}
	pr_info("osm_isp: file version %u.%u, kernel %u.%u\n", h->version >> 16,
		h->version & 0xffff, ver[1], ver[2]);
	if (ver[1] != h->version >> 16) {
		pr_err("osm_isp: major version mismatch\n");
		return -EPROTO;
	}

	for (p = sizeof(*h); p + sizeof(struct isp_bin_ent) <= sizeof(*h) + h->size;) {
		const struct isp_bin_ent *e = (const void *)(d + p);
		const u8 *pl = d + p + sizeof(*e);
		u32 id, cnt, off0, len = e->len;
		int sz;

		if (e->key != ISP_BIN_KEY || len < 4 ||
		    len > sizeof(*h) + h->size - p - sizeof(*e)) {
			pr_err("osm_isp: bad entry at 0x%zx\n", p);
			return -EINVAL;
		}
		p += sizeof(*e) + len;
		if (e->type != ISP_BIN_TYPE_API)
			continue;
		id = *(const u16 *)pl;
		cnt = *(const u16 *)(pl + 2);
		if (!cnt || 4 + 4 * cnt > len) {
			skipped++;
			continue;
		}
		off0 = *(const u32 *)(pl + 4);
		sz = isp_api_size(id);
		if (sz < 0) {
			skipped++;
			continue;
		}
		if (4 + 4 * cnt + off0 + sz > len) {
			pr_warn("osm_isp: id 0x%x: data %d does not fit entry %u\n",
				id, sz, len);
			failed++;
			continue;
		}
		{
			void *buf = kmemdup(pl + 4 + 4 * cnt + off0, sz, GFP_KERNEL);

			if (!buf)
				return -ENOMEM;
			ret = isp_set(ch, id, sz, buf);
			kfree(buf);
		}
		if (ret) {
			pr_warn("osm_isp: id 0x%x size %d: error 0x%x\n", id, sz, ret);
			failed++;
		} else {
			done++;
		}
	}
	pr_info("osm_isp: ch %u: %u applied, %u skipped, %u failed\n", ch, done, skipped, failed);
	return failed ? -EIO : 0;
}

/**
 * osm_isp_load_bin - apply an ISP tuning bin to an ISP channel
 * @ch: ISP/VPE channel (0..2)
 * @name: firmware name relative to the firmware path
 *
 * Process context; the ISP channel must be open.
 */
int osm_isp_load_bin(u32 ch, const char *name)
{
	const struct firmware *fw;
	int ret;

	mutex_lock(&isp_lock);
	ret = request_firmware(&fw, name, osm_msys_device());
	if (ret) {
		pr_err("osm_isp: firmware %s: %d\n", name, ret);
		goto out;
	}
	ret = isp_bin_apply(ch, fw->data, fw->size);
	release_firmware(fw);
out:
	mutex_unlock(&isp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(osm_isp_load_bin);

static int load_set(const char *val, const struct kernel_param *kp)
{
	char buf[160], *name = buf;
	unsigned int ch = 0;
	char *sp;

	strscpy(buf, val, sizeof(buf));
	strim(buf);
	sp = strchr(buf, ' ');
	if (sp) {
		*sp++ = 0;
		if (kstrtouint(buf, 0, &ch))
			return -EINVAL;
		name = skip_spaces(sp);
	}
	if (!*name)
		return -EINVAL;
	return osm_isp_load_bin(ch, name);
}

/* debug: write "ch id size" (ids as in isp_api[]) to dump the kernel's current API data */
static int dump_set(const char *val, const struct kernel_param *kp)
{
	unsigned int ch, id, size;
	void *buf;
	int ret;

	if (sscanf(val, "%u %i %u", &ch, &id, &size) != 3 || !size || size > 0x6000)
		return -EINVAL;
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = isp_get(ch, id, size, buf);
	pr_info("osm_isp: get ch %u id 0x%x size %u: 0x%x\n", ch, id, size, ret);
	if (!ret)
		print_hex_dump(KERN_INFO, "osm_isp: ", DUMP_PREFIX_OFFSET, 16, 4, buf, size, false);
	kfree(buf);
	return ret ? -EIO : 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(dump, &dump_ops, NULL, 0200);
MODULE_PARM_DESC(dump, "debug: write \"ch id size\" to log the kernel's ISP API data");

static const struct kernel_param_ops load_ops = { .set = load_set };
module_param_cb(load, &load_ops, NULL, 0200);
MODULE_PARM_DESC(load, "write \"[ch] firmware-name\" to apply an ISP tuning bin");

static int __init osm_isp_init(void)
{
	if (isp_bin[0] && osm_isp_load_bin(isp_ch, isp_bin))
		pr_warn("osm_isp: isp_bin %s not applied at load\n", isp_bin);
	return 0;
}

module_init(osm_isp_init);
MODULE_DESCRIPTION("SigmaStar ISP tuning bin loader");
MODULE_LICENSE("GPL");
