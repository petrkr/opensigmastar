// SPDX-License-Identifier: GPL-2.0
/*
 * msys of the SigmaStar BSP (drivers/sstar/msys/ms_msys.c): named direct
 * memory (DMEM) buffers. The /dev/msys ioctls are not ported yet.
 *
 * Buffers come from dma_alloc_coherent() on the msys device, as in the
 * BSP; large buffers need CMA in the kernel.
 */
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "osm_riu.h"
#include "osm_sstar.h"

struct dmem_entry {
	struct list_head list;
	MSYS_DMEM_INFO info;
};

static LIST_HEAD(dmem_list);
static DEFINE_MUTEX(dmem_mutex);

static const struct file_operations msys_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice msys_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msys",
	.fops = &msys_fops,
};

static struct dmem_entry *dmem_find_name(const char *name)
{
	struct dmem_entry *e;

	list_for_each_entry(e, &dmem_list, list)
		if (!strncmp(e->info.name, name, strnlen(name, 15)))
			return e;
	return NULL;
}

static void dmem_free(struct dmem_entry *e)
{
	dma_free_coherent(msys_dev.this_device, PAGE_ALIGN(e->info.length),
			  (void *)(uintptr_t)e->info.kvirt, e->info.phys);
	list_del(&e->list);
	kfree(e);
}

int msys_request_dmem(MSYS_DMEM_INFO *mem_info)
{
	struct dmem_entry *e;
	dma_addr_t phys;
	void *virt;
	int err = 0;

	if (!mem_info->name[0] || strnlen(mem_info->name, 16) > 15) {
		pr_err("msys: invalid DMEM name\n");
		return -EINVAL;
	}

	mutex_lock(&dmem_mutex);
	e = dmem_find_name(mem_info->name);
	if (e) {
		/* the BSP hands out the kept buffer again */
		memcpy(mem_info, &e->info, sizeof(*mem_info));
		goto out;
	}

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e) {
		err = -ENOMEM;
		goto out;
	}
	virt = dma_alloc_coherent(msys_dev.this_device, PAGE_ALIGN(mem_info->length),
				  &phys, GFP_KERNEL);
	if (!virt) {
		pr_err("msys: DMEM [%s] 0x%x: allocation failed\n",
		       mem_info->name, mem_info->length);
		kfree(e);
		err = -ENOMEM;
		goto out;
	}
	mem_info->kvirt = (uintptr_t)virt;
	mem_info->phys = phys;
	memcpy(&e->info, mem_info, sizeof(*mem_info));
	list_add(&e->list, &dmem_list);
	pr_debug("msys: DMEM [%s] 0x%x at %pad\n", mem_info->name, mem_info->length, &phys);
out:
	mutex_unlock(&dmem_mutex);
	return err;
}
EXPORT_SYMBOL(msys_request_dmem);

int msys_release_dmem(MSYS_DMEM_INFO *mem_info)
{
	struct dmem_entry *e = NULL, *t;

	mutex_lock(&dmem_mutex);
	if (mem_info->name[0])
		e = dmem_find_name(mem_info->name);
	if (!e && mem_info->phys) {
		list_for_each_entry(t, &dmem_list, list) {
			if (t->info.phys == mem_info->phys) {
				e = t;
				break;
			}
		}
	}
	if (e)
		dmem_free(e);
	else
		pr_err("msys: DMEM [%s]@0x%llx not found\n", mem_info->name, mem_info->phys);
	mutex_unlock(&dmem_mutex);
	return 0;
}
EXPORT_SYMBOL(msys_release_dmem);

int msys_find_dmem_by_phys(unsigned long long phys, MSYS_DMEM_INFO *mem_info)
{
	struct dmem_entry *e;
	int ret = -EINVAL;

	if (!phys)
		return ret;
	mutex_lock(&dmem_mutex);
	list_for_each_entry(e, &dmem_list, list) {
		if (e->info.phys <= phys && phys < e->info.phys + e->info.length) {
			memcpy(mem_info, &e->info, sizeof(*mem_info));
			ret = 0;
			break;
		}
	}
	mutex_unlock(&dmem_mutex);
	return ret;
}
EXPORT_SYMBOL(msys_find_dmem_by_phys);

/* Chip unique ID from the OTP, as the BSP for Infinity6E */
int msys_read_uuid(unsigned long long *udid)
{
	*udid = (u64)INREG16(BASE_REG_OTP_PA + REG_ID_54) |
		((u64)INREG16(BASE_REG_OTP_PA + REG_ID_55) << 16) |
		((u64)INREG16(BASE_REG_OTP_PA + REG_ID_56) << 32);
	return 0;
}
EXPORT_SYMBOL(msys_read_uuid);

/* Boot time profiling records of the BSP (arch/arm/mach-sstar/prof.c): not kept */
void recode_timestamp(int mark, const char *name)
{
}
EXPORT_SYMBOL(recode_timestamp);

struct device *osm_msys_device(void)
{
	return msys_dev.this_device;
}
EXPORT_SYMBOL(osm_msys_device);

int osm_msys_init(void)
{
	int ret = misc_register(&msys_dev);

	if (ret)
		return ret;
	ret = dma_coerce_mask_and_coherent(msys_dev.this_device, DMA_BIT_MASK(32));
	if (ret)
		misc_deregister(&msys_dev);
	return ret;
}

void osm_msys_exit(void)
{
	struct dmem_entry *e, *t;

	mutex_lock(&dmem_mutex);
	list_for_each_entry_safe(e, t, &dmem_list, list)
		dmem_free(e);
	mutex_unlock(&dmem_mutex);
	misc_deregister(&msys_dev);
}
