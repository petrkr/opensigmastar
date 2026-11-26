// SPDX-License-Identifier: GPL-2.0
/*
 * SCL (scaler) shim, linked into mhal.ko with scripts/relink.py --scl shim: the
 * MHalVpe*Scl* entry points of SCL_CUT (the ones mi_vpe imports) are undefined
 * in the blob and defined here. They forward to the blob (__vendor_<name>,
 * aliases of the original functions), count the calls and log the arguments of
 * the first ones, including the first bytes of every argument that points into
 * the kernel linear map. Nothing else changes.
 *
 * Part of the proprietary-tainted mhal module: no GPL-only kernel symbols.
 *
 * Parameters: mhal.scltrace=N logs the first N calls of every entry point,
 * echo 1 > /sys/module/mhal/parameters/scldump prints the call counts.
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

static int scltrace = 4;
module_param(scltrace, int, 0644);
static int sclmem = 96;		/* bytes dumped per pointer argument */
module_param(sclmem, int, 0644);

#define SCL_FUNCS \
	X(MHalVpeCreateSclInstance) X(MHalVpeDestroySclInstance) X(MHalVpeSclCheckIrq) \
	X(MHalVpeSclClearIrq) X(MHalVpeSclDbgLevel) X(MHalVpeSclDmaPortConfig) \
	X(MHalVpeSclEnableIrq) X(MHalVpeSclGetActiveBuf) X(MHalVpeSclGetIrqNum) \
	X(MHalVpeSclGetQpHistogram) X(MHalVpeSclInputConfig) \
	X(MHalVpeSclPortMirrorFlipConfig) X(MHalVpeSclPortSizeConfig) \
	X(MHalVpeSclPreCropConfig) X(MHalVpeSclProcess) X(MHalVpeSclReset) \
	X(MHalVpeSclSetWaitDone)

enum {
#define X(n) I_##n,
	SCL_FUNCS
#undef X
	NFUNC
};

static const char *const fname[NFUNC] = {
#define X(n) [I_##n] = #n,
	SCL_FUNCS
#undef X
};

static atomic_t cnt[NFUNC];
static int port1log = 6;
module_param(port1log, int, 0644);

/* register snapshots of the banks 0x1200..0x123f: the calls that change registers are
 * logged with the changes (register diffs per call, scldiff=N logs the first N calls
 * of every entry point) */
extern void __iomem *osm_riu;
static int scldiff = 0;
module_param(scldiff, int, 0644);
#define SNAP_B0	0x1200
#define SNAP_NB	64
static u16 snap[SNAP_NB][128];
static bool snap_valid;

static void snap_diff(const char *fn, int k, bool log, bool pre)
{
	int b, n, nd = 0;

	for (b = 0; b < SNAP_NB; b++) {
		for (n = 0; n < 128; n++) {
			u16 v = readw(osm_riu + (SNAP_B0 + b) * 0x200 + n * 4);

			if (v != snap[b][n]) {
				if (log && snap_valid && nd++ < 3000)
					pr_info("scl: reg %s%s#%d %04x:%02x %04x -> %04x\n", pre ? "pre-" : "", fn, k,
						SNAP_B0 + b, n * 2, snap[b][n], v);
				snap[b][n] = v;
			}
		}
	}
	snap_valid = true;
}

static void dump_arg(const char *fn, int i, unsigned long v)
{
	if (sclmem > 0 && v >= PAGE_OFFSET && v < (unsigned long)high_memory - 256 && !(v & 3)) {
		pr_info("scl: %s arg%d @%#lx:\n", fn, i, v);
		print_hex_dump(KERN_INFO, "scl:  ", DUMP_PREFIX_OFFSET, 32, 4, (void *)v, sclmem, false);
	}
}

#define X(n) \
	extern long long __vendor_##n(long, long, long, long, long, long); \
	long long n(long a, long b, long c, long d, long f, long g) \
	{ \
		long long r; \
		int k = atomic_inc_return(&cnt[I_##n]); \
		if (k <= scltrace) { \
			pr_info("scl: %u >%s(%#lx, %#lx, %#lx, %#lx, %#lx)\n", \
				jiffies_to_msecs(jiffies), #n, a, b, c, d, f); \
			dump_arg(#n, 0, a); dump_arg(#n, 1, b); dump_arg(#n, 2, c); \
			dump_arg(#n, 3, d); \
		} \
		if (I_##n == I_MHalVpeSclProcess && c >= PAGE_OFFSET && c < (unsigned long)high_memory - 512 && \
		    ((u8 *)c)[0xd8] && port1log-- > 0) { \
			pr_info("scl: port1 Process %u buf:\n", jiffies_to_msecs(jiffies)); \
			print_hex_dump(KERN_INFO, "scl:  ", DUMP_PREFIX_OFFSET, 32, 4, (void *)c, 0xe0, false); \
		} \
		if (scldiff && k <= scldiff) snap_diff(#n, k, true, true); \
		r = __vendor_##n(a, b, c, d, f, g); \
		if (scldiff && k <= scldiff) snap_diff(#n, k, true, false); \
		if (k <= scltrace) { \
			pr_info("scl: %u <%s = %#llx\n", jiffies_to_msecs(jiffies), #n, r); \
			if (I_##n == I_MHalVpeCreateSclInstance && c >= PAGE_OFFSET && c < (unsigned long)high_memory) \
				pr_info("scl: ctx = %#lx\n", *(unsigned long *)c); \
		} \
		return r; \
	}
SCL_FUNCS
#undef X

#define X(n) EXPORT_SYMBOL(n);
SCL_FUNCS
#undef X

static int snap_set(const char *val, const struct kernel_param *kp)
{
	snap_diff("sclsnap", 0, false, false);
	snap_valid = true;
	return 0;
}

static const struct kernel_param_ops snap_ops = { .set = snap_set };
module_param_cb(sclsnap, &snap_ops, NULL, 0200);

static int dump_set(const char *val, const struct kernel_param *kp)
{
	int i;

	for (i = 0; i < NFUNC; i++)
		if (atomic_read(&cnt[i]))
			pr_info("scl: %-34s %d\n", fname[i], atomic_read(&cnt[i]));
	return 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(scldump, &dump_ops, NULL, 0200);
