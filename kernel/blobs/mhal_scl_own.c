// SPDX-License-Identifier: GPL-2.0
/*
 * Own scaler (SCL) code for the mhal blob, linked into mhal.ko with
 * scripts/relink.py --scl own (make SCL=own). It provides the MHalVpe*Scl*
 * entry points that mi_vpe imports (SCL_CUT in relink.py). The blob's SCL
 * init (MHalVpeInit: clocks, default registers) is kept, everything that
 * mi_vpe calls per channel and per frame is done here.
 *
 * What the hardware needs (measured on the running stream, see KNOWHOW "SCL spec"):
 *  - per frame, queued into the CMDQ engine 1 of mi_sys (interface handed in by
 *    Process/SetWaitDone): the register program of the port (prog0..2), a wait for
 *    the SC_TRIG0 event (the ISP frame end), then the frame bookkeeping (prog3..5).
 *    Only the DMA addresses and the frame counter change from frame to frame.
 *  - the interrupt: the vendor mask 0x7feb of bank 0x1218 and the decode of the flag
 *    word into the types mi_vpe's ISR expects (1 = 0x10, 2 = 0x04, 8 = 0x8000)
 *  - the QP histogram counter read.
 *
 * Only the verified configuration is supported: one port, ISP output
 * 3840x2160, port 0 1920x1080 NV12. Everything else warns and returns 0
 * (MHAL boolean failure; success is 1). Mirror/flip/rotation are not used.
 *
 * Part of the proprietary-tainted mhal module: no GPL-only kernel symbols.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/types.h>

extern void __iomem *osm_riu;
/* blob helper: IRQ number of SCL instance (handle id) */
extern unsigned int DrvSclIfGetIrqNum(unsigned int id);

struct scl_reg { u32 addr; u16 val; u16 mask; };

static int sclown_log;
module_param(sclown_log, int, 0644);

/* register of the vendor notation: bank << 8 | byte offset (16 bit regs, 4 byte stride) */
static void __iomem *rp(u32 a)
{
	return osm_riu + (a >> 8) * 0x200 + ((a & 0xff) >> 1) * 4;
}

static u16 r16(u32 a) { return readw(rp(a)); }
static void w16m(u32 a, u16 v, u16 m) { writew((readw(rp(a)) & ~m) | (v & m), rp(a)); }

#define SCL_IRQ_MASK	0x121810
#define SCL_IRQ_CLEAR	0x121820
#define SCL_IRQ_FLAG	0x121826
#define SCL_IRQ_BITS	0x8014		/* types 0x10, 0x04, 0x8000 */
#define SCL_EV_TRIG0	0x18		/* logical CMDQ event SC_TRIG0 */
#define SCL_QP_HIST	0x1210a4

/* register program for 3840x2160 -> 1920x1080 NV12 (see KNOWHOW), the addresses
 * 0x121310/12 (Y) and 0x121320/22 (UV) are patched per frame, 0x1210b4/b6 hold the frame number */
static const struct scl_reg prog0[] = {
	{ 0x1210b0, 0x0000, 0x0001 }, { 0x1210b4, 0x0001, 0xffff }, { 0x1210b2, 0x0000, 0x0002 },
};

static const struct scl_reg prog1[] = {
	{ 0x121804, 0x0000, 0x4303 }, { 0x121302, 0x9f83, 0xd880 }, { 0x121308, 0x9f83, 0xd880 },
	{ 0x121402, 0x9f83, 0xd880 }, { 0x122fba, 0x0000, 0x0051 }, { 0x122f22, 0x0000, 0x0051 },
	{ 0x122fa2, 0x1000, 0x0007 }, { 0x121330, 0x0800, 0x0038 }, { 0x12133c, 0x000c, 0x000c },
	{ 0x122540, 0x7000, 0x0027 }, { 0x122fbe, 0x0000, 0x3303 }, { 0x122fc0, 0x0000, 0x3303 },
	{ 0x122fc2, 0x0000, 0x3303 }, { 0x122fc4, 0x0000, 0x00ff }, { 0x122fb8, 0x0000, 0x0051 },
	{ 0x122fbc, 0x0000, 0x0051 }, { 0x121900, 0x0003, 0x0001 }, { 0x121902, 0x0f00, 0x1fff },
	{ 0x121904, 0x0870, 0x1fff }, { 0x121906, 0x0000, 0x1fff }, { 0x121908, 0x0f00, 0x1fff },
	{ 0x12190a, 0x0000, 0x1fff }, { 0x12190c, 0x0870, 0x1fff }, { 0x121010, 0x0120, 0x01ff },
	{ 0x121016, 0x4343, 0xffff }, { 0x121018, 0xcfc3, 0xcfc3 }, { 0x12100e, 0x0000, 0xffff },
	{ 0x121002, 0x0000, 0xffff }, { 0x121004, 0x0008, 0x00ff }, { 0x121014, 0x0120, 0x01ff },
	{ 0x121012, 0x0000, 0xffff }, { 0x121006, 0x0000, 0xffff }, { 0x121008, 0x0008, 0x00ff },
	{ 0x121040, 0x0f00, 0x1fff }, { 0x121042, 0x0870, 0x0fff }, { 0x121044, 0x0780, 0x3fff },
	{ 0x121046, 0x0438, 0x1fff }, { 0x121334, 0x0780, 0xffff }, { 0x121336, 0x0438, 0xffff },
	{ 0x1213c0, 0x0800, 0x0038 }, { 0x1213cc, 0x000c, 0x000c }, { 0x122548, 0x7000, 0x0027 },
	{ 0x122fc8, 0x0000, 0x00ff }, { 0x121910, 0x0002, 0x0001 }, { 0x121912, 0x0f00, 0x1fff },
	{ 0x121914, 0x0870, 0x1fff }, { 0x121916, 0x0000, 0x1fff }, { 0x121918, 0x0f00, 0x1fff },
	{ 0x12191a, 0x0000, 0x1fff }, { 0x12191c, 0x0870, 0x1fff }, { 0x121110, 0x0000, 0x01ff },
	{ 0x121116, 0x0800, 0xffff }, { 0x121118, 0x8080, 0xcfc3 }, { 0x12110e, 0x0000, 0xffff },
	{ 0x121102, 0x0000, 0xffff }, { 0x121104, 0x0000, 0x00ff }, { 0x121114, 0x0000, 0x01ff },
	{ 0x121112, 0x0000, 0xffff }, { 0x121106, 0x0000, 0xffff }, { 0x121108, 0x0000, 0x00ff },
	{ 0x121140, 0x0f00, 0x1fff }, { 0x121142, 0x0870, 0x0fff }, { 0x121144, 0x0f00, 0x3fff },
	{ 0x121146, 0x0870, 0x1fff }, { 0x1213c4, 0x0f00, 0xffff }, { 0x1213c6, 0x0870, 0xffff },
	{ 0x121430, 0x0800, 0x0038 }, { 0x12143c, 0x000c, 0x000c }, { 0x122550, 0x7000, 0x0027 },
	{ 0x122fca, 0x0000, 0x00ff }, { 0x121930, 0x0002, 0x0001 }, { 0x121932, 0x0f00, 0x1fff },
	{ 0x121934, 0x0870, 0x1fff }, { 0x121936, 0x0000, 0x1fff }, { 0x121938, 0x0f00, 0x1fff },
	{ 0x12193a, 0x0000, 0x1fff }, { 0x12193c, 0x0870, 0x1fff }, { 0x121210, 0x0000, 0x01ff },
	{ 0x121216, 0x0800, 0xffff }, { 0x121218, 0x8080, 0xcfc3 }, { 0x12120e, 0x0000, 0xffff },
	{ 0x121202, 0x0000, 0xffff }, { 0x121204, 0x0000, 0x00ff }, { 0x121214, 0x0000, 0x01ff },
	{ 0x121212, 0x0000, 0xffff }, { 0x121206, 0x0000, 0xffff }, { 0x121208, 0x0000, 0x00ff },
	{ 0x121240, 0x0f00, 0x1fff }, { 0x121242, 0x0870, 0x0fff }, { 0x121244, 0x0f00, 0x3fff },
	{ 0x121246, 0x0870, 0x1fff }, { 0x121434, 0x0f00, 0xffff }, { 0x121436, 0x0870, 0xffff },
	{ 0x121310, 0x8e00, 0xffff }, { 0x121312, 0x00c5, 0xffff }, { 0x121320, 0x8a00, 0xffff },
	{ 0x121322, 0x00c9, 0xffff }, { 0x122570, 0x0000, 0xffff }, { 0x122572, 0x0000, 0xffff },
	{ 0x1215a6, 0x0000, 0x0001 }, { 0x1215a8, 0x0780, 0xffff }, { 0x1215b4, 0x0780, 0xffff },
	{ 0x1218ca, 0x0003, 0xffff }, { 0x1218cc, 0x0000, 0x000f }, { 0x1218d0, 0x0000, 0xffff },
	{ 0x1218d2, 0x0000, 0xffff }, { 0x1218d4, 0x0000, 0xffff }, { 0x1218d6, 0x0000, 0xffff },
	{ 0x1218d8, 0x0000, 0xffff }, { 0x1218da, 0x0000, 0xffff }, { 0x1218dc, 0x0000, 0xffff },
	{ 0x1218de, 0x0000, 0xffff }, { 0x121566, 0x0011, 0x0001 },
};

/* the same with the second port (JPEG, 1920x1080 YUYV 4:2:2) enabled */
static const struct scl_reg prog1b[] = {
	{ 0x121804, 0x0000, 0x4303 }, { 0x121302, 0x9f83, 0xd880 }, { 0x121308, 0xdf83, 0xd880 },
	{ 0x121402, 0x9f83, 0xd880 }, { 0x122fba, 0x0000, 0x0051 }, { 0x122f22, 0x0000, 0x0051 },
	{ 0x122fa2, 0x1000, 0x0007 }, { 0x121330, 0x0800, 0x0038 }, { 0x12133c, 0x000c, 0x000c },
	{ 0x122540, 0x7000, 0x0027 }, { 0x122fbe, 0x0000, 0x3303 }, { 0x122fc0, 0x0000, 0x3303 },
	{ 0x122fc2, 0x0000, 0x3303 }, { 0x122fc4, 0x0000, 0x00ff }, { 0x122fb8, 0x0000, 0x0051 },
	{ 0x122fbc, 0x0000, 0x0051 }, { 0x121900, 0x0003, 0x0001 }, { 0x121902, 0x0f00, 0x1fff },
	{ 0x121904, 0x0870, 0x1fff }, { 0x121906, 0x0000, 0x1fff }, { 0x121908, 0x0f00, 0x1fff },
	{ 0x12190a, 0x0000, 0x1fff }, { 0x12190c, 0x0870, 0x1fff }, { 0x121010, 0x0120, 0x01ff },
	{ 0x121016, 0x4343, 0xffff }, { 0x121018, 0xcfc3, 0xcfc3 }, { 0x12100e, 0x0000, 0xffff },
	{ 0x121002, 0x0000, 0xffff }, { 0x121004, 0x0008, 0x00ff }, { 0x121014, 0x0120, 0x01ff },
	{ 0x121012, 0x0000, 0xffff }, { 0x121006, 0x0000, 0xffff }, { 0x121008, 0x0008, 0x00ff },
	{ 0x121040, 0x0f00, 0x1fff }, { 0x121042, 0x0870, 0x0fff }, { 0x121044, 0x0780, 0x3fff },
	{ 0x121046, 0x0438, 0x1fff }, { 0x121334, 0x0780, 0xffff }, { 0x121336, 0x0438, 0xffff },
	{ 0x1213c0, 0x0800, 0x0038 }, { 0x1213cc, 0x0004, 0x000c }, { 0x122548, 0x7000, 0x0027 },
	{ 0x122fc8, 0x0000, 0x00ff }, { 0x121910, 0x0002, 0x0001 }, { 0x121912, 0x0f00, 0x1fff },
	{ 0x121914, 0x0870, 0x1fff }, { 0x121916, 0x0000, 0x1fff }, { 0x121918, 0x0f00, 0x1fff },
	{ 0x12191a, 0x0000, 0x1fff }, { 0x12191c, 0x0870, 0x1fff }, { 0x121110, 0x0120, 0x01ff },
	{ 0x121116, 0x4343, 0xffff }, { 0x121118, 0xcfc3, 0xcfc3 }, { 0x12110e, 0x0000, 0xffff },
	{ 0x121102, 0x0000, 0xffff }, { 0x121104, 0x0008, 0x00ff }, { 0x121114, 0x0120, 0x01ff },
	{ 0x121112, 0x0000, 0xffff }, { 0x121106, 0x0000, 0xffff }, { 0x121108, 0x0008, 0x00ff },
	{ 0x121140, 0x0f00, 0x1fff }, { 0x121142, 0x0870, 0x0fff }, { 0x121144, 0x0780, 0x3fff },
	{ 0x121146, 0x0438, 0x1fff }, { 0x1213c4, 0x0780, 0xffff }, { 0x1213c6, 0x0438, 0xffff },
	{ 0x121430, 0x0800, 0x0038 }, { 0x12143c, 0x000c, 0x000c }, { 0x122550, 0x7000, 0x0027 },
	{ 0x122fca, 0x0000, 0x00ff }, { 0x121930, 0x0002, 0x0001 }, { 0x121932, 0x0780, 0x1fff },
	{ 0x121934, 0x0438, 0x1fff }, { 0x121936, 0x0000, 0x1fff }, { 0x121938, 0x0f00, 0x1fff },
	{ 0x12193a, 0x0000, 0x1fff }, { 0x12193c, 0x0870, 0x1fff }, { 0x121210, 0x0108, 0x01ff },
	{ 0x121216, 0x4b43, 0xffff }, { 0x121218, 0x8f83, 0xcfc3 }, { 0x12120e, 0x0000, 0xffff },
	{ 0x121202, 0x0000, 0xffff }, { 0x121204, 0x0000, 0x00ff }, { 0x121214, 0x0108, 0x01ff },
	{ 0x121212, 0x0000, 0xffff }, { 0x121206, 0x0000, 0xffff }, { 0x121208, 0x0000, 0x00ff },
	{ 0x121240, 0x0780, 0x1fff }, { 0x121242, 0x0438, 0x0fff }, { 0x121244, 0x0f00, 0x3fff },
	{ 0x121246, 0x0870, 0x1fff }, { 0x121434, 0x0f00, 0xffff }, { 0x121436, 0x0870, 0xffff },
	{ 0x121310, 0x8e00, 0xffff }, { 0x121312, 0x00c5, 0xffff }, { 0x121320, 0x8a00, 0xffff },
	{ 0x121322, 0x00c9, 0xffff }, { 0x122570, 0x0000, 0xffff }, { 0x122572, 0x0000, 0xffff },
	{ 0x1215a6, 0x0000, 0x0011 }, { 0x1215a8, 0x0780, 0xffff }, { 0x1215b4, 0x0780, 0xffff },
	{ 0x1218ca, 0x0003, 0xffff }, { 0x1218cc, 0x0000, 0x00ff }, { 0x1218d0, 0x0000, 0xffff },
	{ 0x1218d2, 0x0000, 0xffff }, { 0x1218d4, 0x0000, 0xffff }, { 0x1218d6, 0x0000, 0xffff },
	{ 0x1218d8, 0x0000, 0xffff }, { 0x1218da, 0x0000, 0xffff }, { 0x1218dc, 0x0000, 0xffff },
	{ 0x1218de, 0x0000, 0xffff }, { 0x121566, 0x0013, 0x0003 }, { 0x1213a0, 0xaa00, 0xffff },
	{ 0x1213a2, 0x0106, 0xffff }, { 0x1213b0, 0xaa02, 0xffff }, { 0x1213b2, 0x0106, 0xffff },
	{ 0x122590, 0x0000, 0xffff }, { 0x122592, 0x0000, 0xffff }, { 0x1215b0, 0x0780, 0xffff },
	{ 0x1215bc, 0x0000, 0xffff },
};

static const struct scl_reg prog2[] = {
	{ 0x121302, 0x0040, 0x0040 }, { 0x1211b6, 0x0001, 0x0001 }, { 0x121308, 0x0000, 0x0040 },
	{ 0x121402, 0x0000, 0x0040 }, { 0x121838, 0xfffd, 0x0002 }, { 0x12183c, 0x0002, 0x0002 },
	{ 0x12183c, 0xfffd, 0x0002 },
};

static const struct scl_reg prog3[] = {
	{ 0x1210b6, 0x0001, 0xffff },
};

static const struct scl_reg prog4[] = {
	{ 0x12183c, 0x0002, 0x0002 }, { 0x1210b0, 0x0001, 0x0001 },
};

static const struct scl_reg prog5[] = {
	{ 0x1211b4, 0x0001, 0x000f }, { 0x1211b6, 0x0000, 0x0001 }, { 0x1210b2, 0x0002, 0x0002 },
};

static struct {
	bool created;
	u32 frame;
	bool in_ok, crop_ok, size_ok, dma_ok;
	bool p1_dma, p1_size;
	u16 in_w, in_h, out_w, out_h;
	bool warned;
} c;

typedef long (*cq_chk_t)(void *, long);
typedef long (*cq_multi_t)(void *, const void *, long);
typedef long (*cq_ev_t)(void *, long);

static long cq_chk(void *q, long n) { return ((cq_chk_t)((u32 *)q)[3])(q, n); }
static long cq_multi(void *q, const struct scl_reg *r, int n)
{
	long rc;

	if (cq_chk(q, n + 10) <= 0)
		return -1;
	rc = ((cq_multi_t)((u32 *)q)[7])(q, r, n);
	return rc == n ? 0 : -1;
}

static int unsupported(const char *what)
{
	pr_warn("scl: %s not supported by the own SCL code\n", what);
	return 0;
}

int MHalVpeCreateSclInstance(void *alloc, const u32 *cfg, void **ctx)
{
	if (!ctx)
		return 0;
	memset(&c, 0, sizeof(c));
	c.created = true;
	*ctx = &c;
	return 1;
}
EXPORT_SYMBOL(MHalVpeCreateSclInstance);

int MHalVpeDestroySclInstance(void *ctx)
{
	w16m(SCL_IRQ_MASK, 0xffff, 0xffff);
	c.created = false;
	return 1;
}
EXPORT_SYMBOL(MHalVpeDestroySclInstance);

int MHalVpeSclGetIrqNum(u32 *irq, const u32 *h)
{
	u32 id = h ? h[1] : 0;
	unsigned int n = DrvSclIfGetIrqNum(id);

	if (irq)
		*irq = n;
	return n ? 1 : 0;
}
EXPORT_SYMBOL(MHalVpeSclGetIrqNum);

int MHalVpeSclEnableIrq(const u32 *h)
{
	if (((const u8 *)h)[16]) {
		w16m(SCL_IRQ_MASK, (u16)~SCL_IRQ_BITS, 0xffff);
		w16m(SCL_IRQ_CLEAR, SCL_IRQ_BITS, SCL_IRQ_BITS);
		w16m(SCL_IRQ_CLEAR, 0, SCL_IRQ_BITS);
	} else {
		w16m(SCL_IRQ_MASK, 0xffff, 0xffff);
	}
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclEnableIrq);

/* returns the types that fired: 1 = frame start/trigger (flag 0x10), 2 = frame done
 * (0x04), 8 = flag 0x8000; the pending flags are acknowledged */
int MHalVpeSclCheckIrq(const u32 *h)
{
	static int lg;

	u16 pend = r16(SCL_IRQ_FLAG) & ~r16(SCL_IRQ_MASK) & SCL_IRQ_BITS;
	int ret = 0;

	if (pend & 0x10)
		ret |= 1;
	if (pend & 0x04) {
		ret |= 2;
		w16m(0x1211b4, 0, 0xf);	/* port 1 request flag, set by the frame program */
	}
	if (pend & 0x8000)
		ret |= 8;
	if (sclown_log && (r16(0x121828) & 0x180) && lg++ < 20)
		pr_info("scl: irq pend %#x hi %#x ret %d\n", pend, r16(0x121828), ret);
	if (pend) {
		w16m(SCL_IRQ_CLEAR, pend, pend);
		w16m(SCL_IRQ_CLEAR, 0, pend);
	}
	return ret;
}
EXPORT_SYMBOL(MHalVpeSclCheckIrq);

int MHalVpeSclClearIrq(long a, long b) { return 1; }
EXPORT_SYMBOL(MHalVpeSclClearIrq);

int MHalVpeSclDbgLevel(long l) { return 1; }
EXPORT_SYMBOL(MHalVpeSclDbgLevel);

int MHalVpeSclGetActiveBuf(void *p) { return 1; }
EXPORT_SYMBOL(MHalVpeSclGetActiveBuf);

int MHalVpeSclReset(void *p) { return 1; }
EXPORT_SYMBOL(MHalVpeSclReset);

int MHalVpeSclInputConfig(void *ctx, const u32 *cfg)
{
	c.in_ok = true;	/* the input is the ISP output, size from the pre-crop */
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclInputConfig);

/* cfg word 1 = height << 16 | width of the port output */
int MHalVpeSclPortSizeConfig(void *ctx, const u32 *cfg)
{
	if (cfg[1] != 0x04380780)
		return unsupported("port size other than 1920x1080");
	if (cfg[0] == 1)
		c.p1_size = true;
	else
		c.size_ok = true;
	c.out_w = cfg[1] & 0xffff;
	c.out_h = cfg[1] >> 16;
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclPortSizeConfig);

int MHalVpeSclDmaPortConfig(void *ctx, const u32 *cfg)
{
	if (cfg[0] == 0 && cfg[1] == 0xb)
		c.dma_ok = true;
	else if (cfg[0] == 1 && cfg[1] == 0)
		c.p1_dma = true;
	else
		return unsupported("DMA port other than port 0 NV12 / port 1 YUYV");
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclDmaPortConfig);

/* arg 2 = height << 16 | width of the input window */
int MHalVpeSclPreCropConfig(void *ctx, const u32 *cfg, u32 size)
{
	if (size == 0x08700f00) {
		c.crop_ok = true;
		c.in_w = size & 0xffff;
		c.in_h = size >> 16;
	}
	else if (!(size == 1 && cfg[9] == 0x780 && cfg[10] == 0x438))	/* port crop window */
		return unsupported("input other than 3840x2160");
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclPreCropConfig);

int MHalVpeSclPortMirrorFlipConfig(void *ctx, const u32 *cfg)
{
	return unsupported("mirror/flip");
}
EXPORT_SYMBOL(MHalVpeSclPortMirrorFlipConfig);


/* Fill the size dependent registers of a program from the configuration: the template holds
 * the verified 3840x2160 -> 1920x1080 values, the registers below carry the input window
 * (0xf00 x 0x870 in the template) or the port output size (0x780 x 0x438). Scale ratio =
 * in << 18 / out [A: only the 2:1 case is verified], 24 bit in two registers (+0x2 = low,
 * +0x4 = high byte), horizontal at 0x1210/1211 + 0x02, vertical at + 0x06. */
static void fix_sizes(struct scl_reg *p, int n, bool port1)
{
	u32 rh = ((u32)c.in_w << 18) / c.out_w, rv = ((u32)c.in_h << 18) / c.out_h;
	int i;

	for (i = 0; i < n; i++) {
		u32 a = p[i].addr;
		bool wreg = false, hreg = false;

		switch (a) {
		case 0x121902: case 0x121908: case 0x121040: case 0x121912: case 0x121918: case 0x121140:
		case 0x121240: case 0x121932: case 0x121938: case 0x121434: case 0x1213c4: case 0x121144:
		case 0x121044: case 0x121334: case 0x1215a8: case 0x1215b4: case 0x1215b0:
			wreg = true;
			break;
		case 0x121904: case 0x12190c: case 0x121042: case 0x121914: case 0x12191c: case 0x121142:
		case 0x121242: case 0x121934: case 0x12193c: case 0x121436: case 0x1213c6: case 0x121146:
		case 0x121046: case 0x121336:
			hreg = true;
			break;
		case 0x121002: p[i].val = rh; break;
		case 0x121004: p[i].val = rh >> 16; break;
		case 0x121006: p[i].val = rv; break;
		case 0x121008: p[i].val = rv >> 16; break;
		case 0x121102: case 0x121104: case 0x121106: case 0x121108:
			if (port1) {	/* same scale as port 0: both ports are 1920x1080 */
				u32 r = a == 0x121102 || a == 0x121104 ? rh : rv;

				p[i].val = (a == 0x121104 || a == 0x121108) ? r >> 16 : r;
			}
			break;
		}
		/* template sizes: input 0xf00 x 0x870, port output 0x780 x 0x438 */
		if (wreg)
			p[i].val = p[i].val == 0x0f00 ? c.in_w : c.out_w;
		if (hreg)
			p[i].val = p[i].val == 0x0870 ? c.in_h : c.out_h;
	}
}

/* buf: u64 Y address @0, u64 UV address @8 (MIU offsets, 8 byte units in the registers) */
int MHalVpeSclProcess(void *ctx, void *q, const u32 *buf, int n)
{
	struct scl_reg p0[ARRAY_SIZE(prog0)], p1[ARRAY_SIZE(prog1b)];
	const struct scl_reg *src = prog1;
	int n1 = ARRAY_SIZE(prog1);
	u32 y1 = buf[0x70 / 4] >> 3;
	u32 y = buf[0] >> 3, uv = buf[2] >> 3;
	int i;

	if (!(c.in_ok && c.crop_ok && c.size_ok && c.dma_ok))
		return unsupported("a configuration other than the verified one");
	c.frame++;
	w16m(0x1210b0, 0, 1);		/* direct writes of the frame bookkeeping, as the vendor Process */
	w16m(0x1210ba, c.frame, 0xffff);
	memcpy(p0, prog0, sizeof(p0));
	if (((const u8 *)buf)[0xd8]) {	/* second port enabled in this frame */
		if (!(c.p1_dma && c.p1_size))
			return unsupported("port 1 without its configuration");
		src = prog1b;
		n1 = ARRAY_SIZE(prog1b);
	}
	if (sclown_log && src == prog1b && sclown_log-- > 0)
		pr_info("scl: frame %u port1 y %#x uv1 %#x\n", c.frame, buf[0x70 / 4], buf[0x78 / 4]);
	memcpy(p1, src, n1 * sizeof(p1[0]));
	fix_sizes(p1, n1, src == prog1b);
	for (i = 0; i < ARRAY_SIZE(p0); i++)
		if (p0[i].addr == 0x1210b4)
			p0[i].val = c.frame;
	for (i = 0; i < n1; i++) {
		switch (p1[i].addr) {
		case 0x121310: p1[i].val = y; break;
		case 0x121312: p1[i].val = y >> 16; break;
		case 0x121320: p1[i].val = uv; break;
		case 0x121322: p1[i].val = uv >> 16; break;
		case 0x1213a0: p1[i].val = y1; break;
		case 0x1213a2: p1[i].val = y1 >> 16; break;
		case 0x1213b0: p1[i].val = y1 + 2; break;
		case 0x1213b2: p1[i].val = (y1 + 2) >> 16; break;
		}
	}
	if (cq_multi(q, p0, ARRAY_SIZE(p0)) || cq_multi(q, p1, n1) ||
	    cq_multi(q, prog2, ARRAY_SIZE(prog2)))
		return 0;
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclProcess);

int MHalVpeSclSetWaitDone(void *ctx, void *q, int flag)
{
	struct scl_reg p3[ARRAY_SIZE(prog3)];

	memcpy(p3, prog3, sizeof(p3));
	if (p3[0].addr == 0x1210b6)
		p3[0].val = c.frame;
	if (((cq_ev_t)((u32 *)q)[11])(q, SCL_EV_TRIG0) != 0)
		return 0;
	if (cq_multi(q, p3, ARRAY_SIZE(p3)) || cq_multi(q, prog4, ARRAY_SIZE(prog4)) ||
	    cq_multi(q, prog5, ARRAY_SIZE(prog5)))
		return 0;
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclSetWaitDone);

/* hit counter of the port, 32 bit at bank 0x1210/11/12 + 0xa4 */
int MHalVpeSclGetQpHistogram(void *ctx, int port, u32 *out)
{
	u32 a = SCL_QP_HIST + 0x100 * (port & 3);

	if (out)
		*out = r16(a) | (u32)r16(a + 2) << 16;
	return 1;
}
EXPORT_SYMBOL(MHalVpeSclGetQpHistogram);
