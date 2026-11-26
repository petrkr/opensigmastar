// SPDX-License-Identifier: GPL-2.0
/*
 * JPE (JPEG encoder) shim, linked into mhal.ko with scripts/relink.py --jpe shim:
 * the MHAL_JPE_* entry points that mi_venc imports are undefined in the blob and
 * defined here. They forward to the blob (__vendor_<name>, aliases of the
 * original functions), count the calls and log the arguments of the first ones,
 * including the first bytes of every argument that points into the kernel linear
 * map, before and after the call. Nothing else changes.
 *
 * Part of the proprietary-tainted mhal module: no GPL-only kernel symbols.
 *
 * Parameters: mhal.jpetrace=N logs the first N calls of every entry point
 * (SetParam/GetParam: N * 4), mhal.jpemem=bytes dumped per pointer argument,
 * echo 1 > /sys/module/mhal/parameters/jpedump prints the call counts.
 *
 * The prototypes are the widest form (6 words in, 64 bit out): the callee
 * only looks at the registers/stack words it uses.
 */
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>
#include <linux/string.h>

static int jpetrace = 6;
module_param(jpetrace, int, 0644);
static int jpemem = 96;		/* bytes dumped per pointer argument */
module_param(jpemem, int, 0644);

#define JPE_FUNCS \
	X(MHAL_JPE_BarrierOneFrame) X(MHAL_JPE_CheckInstanceStatus) \
	X(MHAL_JPE_CreateDevice) X(MHAL_JPE_CreateInstance) \
	X(MHAL_JPE_DestroyDevice) X(MHAL_JPE_DestroyInstance) \
	X(MHAL_JPE_EncodeAddOutBuff) X(MHAL_JPE_EncodeCancel) \
	X(MHAL_JPE_EncodeFrameDone) X(MHAL_JPE_EncodeOneFrame) \
	X(MHAL_JPE_GetDevConfig) X(MHAL_JPE_GetEngStatus) X(MHAL_JPE_GetParam) \
	X(MHAL_JPE_IsrProc) X(MHAL_JPE_QueryBufSize) \
	X(MHAL_JPE_SetParam)

enum {
#define X(n) I_##n,
	JPE_FUNCS
#undef X
	NFUNC
};

static const char *const fname[NFUNC] = {
#define X(n) [I_##n] = #n,
	JPE_FUNCS
#undef X
};

static atomic_t cnt[NFUNC];

/* register snapshot of bank 0x1620 (the JPE): jperegs=N logs the register changes
 * around the first N calls of every entry point (including the polling ones) */
extern void __iomem *osm_riu;
static int jperegs;
module_param(jperegs, int, 0644);
static u16 snap[128];
static bool snap_valid;

static void snap_diff(const char *fn, int k)
{
	int n;

	for (n = 0; n < 128; n++) {
		u16 v = readw(osm_riu + 0x1620 * 0x200 + n * 4);

		if (snap_valid && v != snap[n])
			pr_info("jpe: reg %s#%d %02x: %04x -> %04x\n", fn, k, n * 2, snap[n], v);
		snap[n] = v;
	}
	snap_valid = true;
}

static void dump_arg(const char *fn, const char *tag, int i, unsigned long v)
{
	u32 buf[64];
	int n = min(jpemem, 256) / 4;

	if (jpemem <= 0 || v < PAGE_OFFSET || (v & 3) || n > 64)
		return;
	/* arguments are live objects: lowmem or vmalloc (kernel stacks) */
	if (!(v < (unsigned long)high_memory - 256 || is_vmalloc_addr((void *)v)))
		return;
	memcpy(buf, (void *)v, n * 4);
	pr_info("jpe: %s %s arg%d @%#lx:\n", fn, tag, i, v);
	print_hex_dump(KERN_INFO, "jpe:  ", DUMP_PREFIX_OFFSET, 32, 4, buf, n * 4, false);
}

#define X(n) \
	extern long long __vendor_##n(long, long, long, long, long, long); \
	long long n(long a, long b, long c, long d, long f, long g) \
	{ \
		long long r; \
		int k = atomic_inc_return(&cnt[I_##n]); \
		int lim = (I_##n == I_MHAL_JPE_SetParam || I_##n == I_MHAL_JPE_GetParam) ? \
			  jpetrace * 4 : jpetrace; \
		if (k <= lim) { \
			pr_info("jpe: %u >%s#%d(%#lx, %#lx, %#lx, %#lx, %#lx, %#lx)\n", \
				jiffies_to_msecs(jiffies), #n, k, a, b, c, d, f, g); \
			dump_arg(#n, "pre", 1, b); \
			dump_arg(#n, "pre", 2, c); dump_arg(#n, "pre", 3, d); \
		} \
		if (jperegs && k <= jperegs) snap_diff("pre-" #n, k); \
		r = __vendor_##n(a, b, c, d, f, g); \
		if (jperegs && k <= jperegs) snap_diff(#n, k); \
		if (k <= lim) { \
			pr_info("jpe: %u <%s#%d = %#llx\n", jiffies_to_msecs(jiffies), #n, k, r); \
			dump_arg(#n, "post", 1, b); \
			dump_arg(#n, "post", 2, c); dump_arg(#n, "post", 3, d); \
		} \
		return r; \
	}
JPE_FUNCS
#undef X

#define X(n) EXPORT_SYMBOL(n);
JPE_FUNCS
#undef X

static int dump_set(const char *val, const struct kernel_param *kp)
{
	int i;

	for (i = 0; i < NFUNC; i++)
		if (atomic_read(&cnt[i]))
			pr_info("jpe: %-34s %d\n", fname[i], atomic_read(&cnt[i]));
	return 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(jpedump, &dump_ops, NULL, 0200);
