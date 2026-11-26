// SPDX-License-Identifier: GPL-2.0
/*
 * CMDQ shim, linked into mhal.ko with scripts/relink.py --cmdq shim: the
 * MHAL_CMDQ_* entry points are undefined in the blob (the blob code itself
 * calls them too) and defined here. They forward to the blob implementation
 * (__vendor_MHAL_CMDQ_*, aliases of the original functions) and count and log
 * the calls. The command queue interface returned by
 * MHAL_CMDQ_GetSysCmdqService() is wrapped the same way: every member counts,
 * logs and forwards. All users are seen, the ISP and scaler code of mhal too.
 *
 * Part of the proprietary-tainted mhal module: no GPL-only kernel symbols.
 *
 * Statistics: echo 1 > /sys/module/mhal/parameters/dump (kernel log),
 * mhal.ntrace=N logs the first N calls of every member.
 */
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>

#define NID	4
#define IF_WORDS 25			/* 100 bytes */
#define IF_PRIV	18			/* word index of priv (+72) */
static int ntrace = 6;			/* argument logs per member and id */
module_param(ntrace, int, 0644);

typedef long (*fn6)(long, long, long, long, long, long);

static atomic_t api_cnt[16];
static const char *const api_name[16] = {
	"GetCmdqHWNumber", "GetFreeId", "GetSpecificId", "ReleaseId",
	"InitCmdqMmapInfo", "DeinitCmdqMmapInfo", "GetSysCmdqService",
	"ReleaseSysCmdqService", "DirectModeEarlyRun", "EarlyRunStop",
	"GetEarlyRunStatus", "DirectModeByteModeCmd",
};

static const char *const if_name[IF_WORDS] = {
	[3] = "CheckBufAvailable", [4] = "WriteDummyReg", [5] = "ReadDummyReg",
	[6] = "WriteRegMask", [7] = "WriteRegMaskMulti", [8] = "WriteReg",
	[9] = "WriteRegMulti", [10] = "PollRegBits", [11] = "AddWaitEvent",
	[12] = "AbortBuffer", [13] = "ResetEngine", [14] = "ReadStatus",
	[15] = "KickOff", [16] = "ClearTriggerEvent", [17] = "IsEmptyIdle",
	[19] = "KickOffByTag", [20] = "CmdDelay", [21] = "GetCurrentNumber",
	[22] = "PollRegBits_ByTime", [23] = "GetAppEventDummyRegAddr",
	[24] = "SetWeight",
};

static struct {
	u32 orig[IF_WORDS];		/* the blob's table */
	u32 wrap[IF_WORDS];		/* what the users get */
	bool valid;
} ifc[NID];
static atomic_t if_cnt[NID][IF_WORDS];
static atomic_t if_log[NID][IF_WORDS];
static atomic_t if_sum[NID][IF_WORDS];	/* sum of the count argument of the Multi members */

#define API(i, name) \
	extern long __vendor_MHAL_CMDQ_##name(long, long, long, long, long, long); \
	long MHAL_CMDQ_##name(long a, long b, long c, long d, long e, long f) \
	{ \
		long r; \
		atomic_inc(&api_cnt[i]); \
		r = __vendor_MHAL_CMDQ_##name(a, b, c, d, e, f); \
		if (atomic_read(&api_cnt[i]) < 12) \
			pr_info("cmdq: %s(%#lx, %#lx, %#lx) = %#lx\n", #name, a, b, c, r); \
		return r; \
	} \
	EXPORT_SYMBOL(MHAL_CMDQ_##name)

API(0, GetCmdqHWNumber);
API(1, GetFreeId);
API(2, GetSpecificId);
API(3, ReleaseId);
API(4, InitCmdqMmapInfo);
API(5, DeinitCmdqMmapInfo);
API(7, ReleaseSysCmdqService);
API(8, DirectModeEarlyRun);
API(9, DirectModeEarlyRunStop);
API(10, GetEarlyRunStatus);
API(11, DirectModeByteModeCmd);

extern long __vendor_MHAL_CMDQ_GetSysCmdqService(long, long, long);
static const u32 tramps[IF_WORDS];

long MHAL_CMDQ_GetSysCmdqService(long id, long desc, long force_riu)
{
	long r = __vendor_MHAL_CMDQ_GetSysCmdqService(id, desc, force_riu);

	atomic_inc(&api_cnt[6]);
	if (r && id >= 0 && id < NID) {
		u32 *o = (u32 *)r;
		int i;

		/* an iface that the blob refilled: wrap it again */
		memcpy(ifc[id].orig, o, sizeof(ifc[id].orig));
		memcpy(ifc[id].wrap, o, sizeof(ifc[id].wrap));
		for (i = 0; i < IF_WORDS; i++)
			if (tramps[i] && o[i])
				ifc[id].wrap[i] = tramps[i];
		ifc[id].valid = true;
		pr_info("cmdq: GetSysCmdqService(%ld, size %u align %u, riu %ld) = %p priv %#x\n",
			id, desc ? ((u32 *)desc)[0] : 0, desc ? ((u16 *)desc)[2] : 0,
			force_riu, o, o[IF_PRIV]);
		return (long)ifc[id].wrap;
	}
	return r;
}
EXPORT_SYMBOL(MHAL_CMDQ_GetSysCmdqService);

static long tramp(int w, long a, long b, long c, long d, long e, long f)
{
	int id;

	/* the iface may have been copied: identify the engine by priv */
	for (id = 0; id < NID; id++)
		if (ifc[id].valid && ((u32 *)a)[IF_PRIV] == ifc[id].orig[IF_PRIV])
			break;
	if (id == NID)
		return 0;
	atomic_inc(&if_cnt[id][w]);
	if (w == 7 || w == 9)
		atomic_add(b ? (int)c : 0, &if_sum[id][w]);
	if (atomic_inc_return(&if_log[id][w]) <= ntrace) {
		long r = ((fn6)ifc[id].orig[w])(a, b, c, d, e, f);

		pr_info("cmdq: %llu id %d %s(%#lx, %#lx, %#lx, %#lx, %#lx) = %#lx\n",
			(u64)jiffies_to_msecs(jiffies), id, if_name[w], b, c, d, e, f, r);
		return r;
	}
	return ((fn6)ifc[id].orig[w])(a, b, c, d, e, f);
}

#define T(n) static long t##n(long a, long b, long c, long d, long e, long f) \
	{ return tramp(n, a, b, c, d, e, f); }
T(3) T(4) T(5) T(6) T(7) T(8) T(9) T(10) T(11) T(12) T(13) T(14) T(15) T(16)
T(17) T(19) T(20) T(21) T(22) T(23) T(24)
#undef T

static const u32 tramps[IF_WORDS] = {
#define T(n) [n] = (u32)t##n,
	T(3) T(4) T(5) T(6) T(7) T(8) T(9) T(10) T(11) T(12) T(13) T(14) T(15)
	T(16) T(17) T(19) T(20) T(21) T(22) T(23) T(24)
#undef T
};

static void dump_stats(void)
{
	int i, id;

	for (i = 0; i < 12; i++)
		if (api_name[i])
			pr_info("cmdq: api %-24s %d\n", api_name[i], atomic_read(&api_cnt[i]));
	for (id = 0; id < NID; id++)
		for (i = 3; i < IF_WORDS; i++)
			if (atomic_read(&if_cnt[id][i]))
				pr_info("cmdq: id %d %-24s %d (sum n %d)\n", id, if_name[i],
					atomic_read(&if_cnt[id][i]), atomic_read(&if_sum[id][i]));
}

static int dump_set(const char *val, const struct kernel_param *kp)
{
	dump_stats();
	return 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(dump, &dump_ops, NULL, 0200);

