// SPDX-License-Identifier: GPL-2.0
/*
 * VIF/CSI shim, linked into mhal.ko with scripts/relink.py --vif shim: the
 * entry points listed in VIF_CUT are undefined in the blob (so that the calls
 * inside mhal, from the sensor, ISP and ISRCB code, go through them too) and
 * defined here. They forward to the blob (__vendor_<name>, aliases of the
 * original functions), count the calls and log the arguments of the first
 * ones. Nothing else changes.
 *
 * Part of the proprietary-tainted mhal module: no GPL-only kernel symbols.
 *
 * Parameters: mhal.vtrace=N logs the first N calls of every entry point,
 * echo 1 > /sys/module/mhal/parameters/vifdump prints the call counts.
 *
 * The prototypes are the widest form (6 words in, 64 bit out): the callee
 * only looks at the registers/stack words it uses.
 */
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

static int vtrace = 6;
module_param(vtrace, int, 0644);

/* X(name, exported) */
#define VIF_FUNCS \
	X(MHal_VIF_ChnDisable, 1) X(MHal_VIF_ChnEnable, 1) X(MHal_VIF_ChnQuery, 1) \
	X(MHal_VIF_ChnReset, 1) X(MHal_VIF_ChnSetConfig, 1) X(MHal_VIF_Deinit, 1) \
	X(MHal_VIF_DequeueFrameBuffer, 1) X(MHal_VIF_DevDisable, 1) \
	X(MHal_VIF_DevEnable, 1) X(MHal_VIF_DevSetConfig, 1) \
	X(MHal_VIF_GetInputMask, 1) X(MHal_VIF_GetVIFDev2SensorPadMux, 1) \
	X(MHal_VIF_Init, 1) X(MHal_VIF_InputMask, 1) X(MHal_VIF_QueryFrames, 1) \
	X(MHal_VIF_QueueFrameBuffer, 1) X(MHal_VIF_RegisterEventCallback, 1) \
	X(MHal_VIF_RegisterGetPtsCallback, 1) X(MHal_VIF_SetVIFDev2SensorPadMux, 1) \
	X(MHal_VIF_StopMCU, 1) X(MHal_VIF_SubChnQuery, 1) \
	X(MHal_Vif_GetSensorFrmCnt64, 1) \
	X(DrvVif_ChnID2DevID, 0) X(DrvVif_Dev2ChnID, 0) X(DrvVif_GetDev2SNRPadID, 0) \
	X(DrvVif_GetSensorFrmCnt64, 0) X(DrvVif_Init, 0) X(DrvVif_InputMask, 0) \
	X(DrvVif_InputMaskFrameSync, 0) X(DrvVif_QueryMCLK, 0) \
	X(DrvVif_SNRPad2DevID, 0) X(DrvVif_SensorPdwn, 0) X(DrvVif_SensorReset, 0) \
	X(DrvVif_SetDev2SNRPadMap, 0) X(DrvVif_SetLineCntHit_MaskChange, 0) \
	X(DrvVif_SetMclk, 0) X(DrvVif_SetSelectSource, 0) X(DrvVif_SetIOPad, 0) \
	X(Drv_CSIQueryLaneNum, 0) X(Drv_CSISetClk, 0) X(Drv_CSISetHDRMode, 0) \
	X(Drv_CSISetLane, 0) X(Drv_CSISetClkDataSkip, 0) X(Drv_CSISetLongPacket, 0)

enum {
#define X(n, e) I_##n,
	VIF_FUNCS
#undef X
	NFUNC
};

static const char *const fname[NFUNC] = {
#define X(n, e) [I_##n] = #n,
	VIF_FUNCS
#undef X
};

static atomic_t cnt[NFUNC];

#define X(n, e) \
	extern long long __vendor_##n(long, long, long, long, long, long); \
	long long n(long a, long b, long c, long d, long f, long g) \
	{ \
		long long r; \
		int k = atomic_inc_return(&cnt[I_##n]); \
		r = __vendor_##n(a, b, c, d, f, g); \
		if (k <= vtrace) \
			pr_info("vif: %u %s(%#lx, %#lx, %#lx, %#lx, %#lx) = %#llx\n", \
				jiffies_to_msecs(jiffies), #n, a, b, c, d, f, r); \
		return r; \
	}
VIF_FUNCS
#undef X

#define X(n, e) XEXP_##e(n)
#define XEXP_0(n)
#define XEXP_1(n) EXPORT_SYMBOL(n);
VIF_FUNCS
#undef X

static int dump_set(const char *val, const struct kernel_param *kp)
{
	int i;

	for (i = 0; i < NFUNC; i++)
		if (atomic_read(&cnt[i]))
			pr_info("vif: %-34s %d\n", fname[i], atomic_read(&cnt[i]));
	return 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(vifdump, &dump_ops, NULL, 0200);
