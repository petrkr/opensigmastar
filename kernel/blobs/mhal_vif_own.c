// SPDX-License-Identifier: GPL-2.0
/*
 * Own VIF and MIPI CSI RX code for the mhal blob, linked into mhal.ko with
 * scripts/relink.py --vif own (make VIF=own). It provides the entry points
 * that the relinker leaves undefined in the blob (VIF_CUT in relink.py): the
 * MHal_VIF_* interface that mi_vif and mi_vpe use, and the DrvVif_* and Drv_CSI*
 * functions that the sensor, ISP and ISRCB code inside mhal call. The blob's
 * own VIF/CSI drivers do not bind (their compatible strings are patched), so
 * what is below is the only code that touches the VIF channel, CSI and sensor
 * pad registers.
 *
 * What the pipeline needs, and what is done here:
 *  - sensor pad: pad mux, reset/power-down pins, MCLK, CSI clocks (Drv_CSI*,
 *    DrvVif_Set*)
 *  - CSI RX: D-PHY analog/digital and MAC setup of the 4 lane blocks
 *  - VIF channel: window crop, format, source pad, the line threshold for the
 *    ISP, enable and the interrupt of each channel. The interrupt calls the
 *    frame start/end callbacks that mi_vif registered, and ISRCB_Proc() with
 *    the ids the vendor ISP and AE code wait for: 8 + chn on frame start
 *    and 12 + chn on frame end
 *  - the real-time work mode only (RGB/RAW straight into the ISP): no DMA
 *    (frame mode), HDR or BT656/parallel input; those return an error
 *
 * Register layout: banks at 0x1f000000 + bank * 0x200, 16 bit registers at a
 * 4 byte stride, fields are accessed as bytes. DT: the sstar,vif and sstar,csi
 * nodes of the vendor DT (pad modes, lane select/swap, interrupts).
 *
 * Part of the proprietary-tainted mhal module: no GPL-only kernel symbols.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/string.h>

/* osm_sstar */
extern void __iomem *osm_riu;
extern int CamofPropertyReadU32(const struct device_node *np, const char *name, u32 *out);
extern int CamOfPropertyReadU32Array(const struct device_node *np, const char *name,
				     u32 *out, size_t n);
extern unsigned int CamIrqOfParseAndMap(struct device_node *np, int index);
/* mhal: ISR callback bus */
extern int ISRCB_Proc(int id);

#define NCHN		3	/* VIF channels = sensor pads */
#define BANK(b)		(osm_riu + (b) * 0x200)

/* register byte helpers */
static inline u8 rb(void __iomem *p, u32 off) { return readb(p + off); }
static inline void wb(void __iomem *p, u32 off, u8 mask, u8 val)
{
	writeb((readb(p + off) & ~mask) | (val & mask), p + off);
}
static inline void ww(void __iomem *p, u32 off, u16 val) { writew(val, p + off); }

/* ---- state ---- */

static struct {
	bool inited;
	u32 mode[NCHN], work[NCHN], hdr[NCHN];	/* MHal_VIF_DevSetConfig words 0..2 */
	u8 pad[NCHN], padc[NCHN];		/* dev -> sensor pad map (+ hdr byte) */
	/* sensor input mask: one bit per owner (0 = MI, 1 = skip-frame timer), set while masked */
	u8 imask[NCHN];
	u8 mflag[NCHN];			/* reads as masked until the first unmask */
	void *cb[NCHN][5];
	unsigned long cbx[NCHN][5];
	u64 frm[NCHN];
	u32 fs[NCHN], fe[NCHN], fifo[NCHN], ln[NCHN], int1[NCHN];
	u32 fs_dt[NCHN];		/* time between the last two frame starts, ns */
	unsigned long fs_last[NCHN];
	u64 (*get_pts)(void);
	/* DT */
	u32 mipi_mode, mipi_ctrl, mipi_rst, mipi_pdn;
	u32 mclk37[NCHN];
	u32 lane_num[3], lane_sel[3][5], lane_pn[3][5];
	int vif_irq, csi_irq;
	bool vif_irq_on, csi_irq_on;
} v = {
	.pad = { 0, 0, 1 }, .padc = { 0xff, 0xff, 0xff }, .mflag = { 1, 1, 1 },
};

/* base of a channel: ch0/ch1 in bank 0x1304 (ch1 = +0x100), ch2 in bank 0x1305 */
static void __iomem *cb_(int c)
{
	return c == 2 ? BANK(0x1305) : BANK(0x1304) + (c ? 0x100 : 0);
}

static int log_en;
module_param_named(vifown_log, log_en, int, 0644);
#define vlog(fmt, ...) do { if (log_en) pr_info("vif: " fmt "\n", ##__VA_ARGS__); } while (0)

/* ---- id maps (logical channel ids 0/4/8 <-> phys channel <-> dev) ---- */

static int chn2phys(u32 chn)
{
	return chn == 0 ? 0 : chn == 4 ? 1 : chn == 8 ? 2 : -1;
}

static const u8 phys2chn[3] = { 0, 4, 8 };

/* sensor pad -> phys channel: pad 1 -> 2, pad 2 -> 1 */
static int pad2phys(u32 pad)
{
	return pad == 0 ? 0 : pad == 1 ? 2 : pad == 2 ? 1 : -1;
}

/* ---- VIF channel registers ---- */

static u8 sensor_mask_set(int c, u8 val)
{
	void __iomem *b = cb_(c);
	u8 old = rb(b, 0x20) & 1;

	if (!val)
		v.mflag[c] = 0;
	wb(b, 0x20, 1, val);
	return old;
}

static u8 sensor_mask_get(int c)
{
	return v.mflag[c] ? 1 : rb(cb_(c), 0x20) & 1;
}

static u16 irq_get(int c, u32 off)
{
	void __iomem *b = cb_(c);

	return rb(b, off) | (rb(b, off + 1) & 1) << 8;
}

static void irq_mask(int c, u16 bits)
{
	void __iomem *b = cb_(c);
	u16 m = irq_get(c, 0x2c) | bits;

	writeb(m, b + 0x2c);
	wb(b, 0x2d, 1, m >> 8);
}

static void irq_unmask(int c, u16 bits)
{
	void __iomem *b = cb_(c);
	u16 m = irq_get(c, 0x2c) & ~bits;

	writeb(m, b + 0x2c);
	wb(b, 0x2d, 1, m >> 8);
}

static void irq_clr(int c, u16 bits)
{
	void __iomem *b = cb_(c);

	writeb(bits, b + 0x34);
	wb(b, 0x35, 1, bits >> 8);
}

static void field16(void __iomem *b, u32 off, u16 val)
{
	writeb(val, b + off);
	wb(b, off + 1, 0x1f, val >> 8);
}

/* ---- interrupt ---- */

static void vif_chn_isr(int c)
{
	u16 st = irq_get(c, 0x38) & ~irq_get(c, 0x2c);
	u16 done = 0;
	void (*cbf)(unsigned long, unsigned long, void *, unsigned long);

	if (!st)
		return;
	if (st & 0x01) {
		cbf = v.cb[c][0];
		if (cbf)
			cbf(c, 0, NULL, v.cbx[c][0]);
		ISRCB_Proc(8 + c);
		v.fs[c]++;
		v.fs_dt[c] = jiffies_to_usecs(jiffies - v.fs_last[c]) * 1000;
		v.fs_last[c] = jiffies;
		done |= 0x01;
	}
	if (st & 0x02)
		v.int1[c]++;
	done |= st & 0x1a;
	if (st & 0x20) {
		static u8 fe_state[0x58];	/* the callback does not use it in real-time mode */

		cbf = v.cb[c][1];
		if (cbf && v.work[c] == 3)
			cbf(c, 1, fe_state, v.cbx[c][1]);
		ISRCB_Proc(12 + c);
		v.fe[c]++;
		v.frm[c]++;
		done |= 0x20;
	}
	if (st & 0x40) {
		cbf = v.cb[c][2];
		if (cbf)
			cbf(c, 3, NULL, v.cbx[c][2]);
		v.ln[c]++;
		done |= 0x40;
	}
	if (st & 0x100) {
		v.fifo[c]++;
		done |= 0x100;
	}
	irq_clr(c, done);
}

static irqreturn_t vif_irq(int irq, void *dev)
{
	int c;

	for (c = 0; c < NCHN; c++)
		vif_chn_isr(c);
	return IRQ_HANDLED;
}

/* CSI error interrupt: masked at the source by the setup (never fires), if it
 * does, mask and clear what is pending like the vendor handler */
static irqreturn_t csi_irq(int irq, void *dev)
{
	void __iomem *mac = BANK(0x120a);
	u16 s = readw(mac + 0x30) & 0x7fff;

	if (s) {
		writew(readw(mac + 0x0c) | s, mac + 0x0c);
		writew(s, mac + 0x14);
	}
	return IRQ_HANDLED;
}

/* ---- init ---- */

static void dt_read(void)
{
	struct device_node *n = of_find_compatible_node(NULL, NULL, "sstar,vif");
	char name[40];
	int i;

	if (n) {
		CamofPropertyReadU32(n, "vif_sr0_mipi_mode", &v.mipi_mode);
		CamofPropertyReadU32(n, "vif_sr0_mipi_ctrl_mode", &v.mipi_ctrl);
		CamofPropertyReadU32(n, "vif_sr0_mipi_rst_mode", &v.mipi_rst);
		CamofPropertyReadU32(n, "vif_sr0_mipi_pdn_mode", &v.mipi_pdn);
		for (i = 0; i < NCHN; i++) {
			snprintf(name, sizeof(name), "vif_sr%d_mclk_37p125", i);
			CamofPropertyReadU32(n, name, &v.mclk37[i]);
		}
		v.vif_irq = CamIrqOfParseAndMap(n, 0);
	}
	n = of_find_compatible_node(NULL, NULL, "sstar,csi");
	if (n) {
		for (i = 0; i < 3; i++) {
			snprintf(name, sizeof(name), "csi_sr%d_lane_num", i);
			CamofPropertyReadU32(n, name, &v.lane_num[i]);
			snprintf(name, sizeof(name), "csi_sr%d_lane_select", i);
			CamOfPropertyReadU32Array(n, name, v.lane_sel[i], i == 2 ? 3 : 5);
			snprintf(name, sizeof(name), "csi_sr%d_lane_pn_swap", i);
			CamOfPropertyReadU32Array(n, name, v.lane_pn[i], i == 2 ? 3 : 5);
		}
		v.csi_irq = CamIrqOfParseAndMap(n, 0);
	}
}

/* DrvVif_Init: the former probe of the vif node, and of the csi node */
int DrvVif_Init(void)
{
	int c, ret;

	if (v.inited)
		return 0;
	dt_read();
	/* sensor inputs masked until MI unmasks them */
	for (c = 0; c < NCHN; c++) {
		v.imask[c] = 1;
		sensor_mask_set(c, 1);
	}
	for (c = 0; c < NCHN; c++) {
		irq_mask(c, 0x1ff);
		irq_unmask(c, 0x1ff & ~0x1fe);	/* default: frame start only */
	}
	if (v.vif_irq) {
		ret = request_irq(v.vif_irq, vif_irq, 0, "vif", &v);
		if (ret)
			pr_err("vif: request_irq(%d) failed %d\n", v.vif_irq, ret);
		else
			v.vif_irq_on = true;
	}
	if (v.csi_irq) {
		ret = request_irq(v.csi_irq, csi_irq, 0, "csi", &v);
		if (ret)
			pr_err("csi: request_irq(%d) failed %d\n", v.csi_irq, ret);
		else
			v.csi_irq_on = true;
	}
	v.inited = true;
	return v.vif_irq_on ? 0 : -1;
}

void osm_vif_exit(void)
{
	int c;

	if (v.vif_irq_on)
		free_irq(v.vif_irq, &v);
	if (v.csi_irq_on)
		free_irq(v.csi_irq, &v);
	v.vif_irq_on = v.csi_irq_on = v.inited = false;
	for (c = 0; c < NCHN; c++)
		irq_mask(c, 0x1ff);
}

/* ---- dev <-> pad maps ---- */

int DrvVif_SetDev2SNRPadMap(u32 dev, u32 pad, u32 c)
{
	if (dev > 2)
		return 0xfe;
	v.pad[dev] = pad;
	v.padc[dev] = c;
	return 0;
}

int DrvVif_GetDev2SNRPadID(u32 dev, u8 *pad, u8 *c)
{
	if (dev > 2)
		return -2;
	*pad = v.pad[dev];
	if (v.pad[dev] == 0xff) {
		*c = 0xff;
		return -2;
	}
	*c = v.padc[dev];
	return 0;
}

int DrvVif_SNRPad2DevID(u32 pad, u8 *dev, u8 *c)
{
	int d;

	for (d = 0; d < NCHN; d++) {
		if (v.pad[d] != pad)
			continue;
		*dev = d;
		if (c)
			*c = v.padc[d];
		return 0;
	}
	*dev = 0xff;
	if (c)
		*c = 0xff;
	return -1;
}

int DrvVif_Dev2ChnID(u32 dev, u8 *chn)
{
	if (dev > 2)
		return -1;
	*chn = phys2chn[dev];
	return 0;
}

int DrvVif_ChnID2DevID(u32 chn)
{
	int p = chn2phys(chn);

	return p < 0 ? 0 : p;
}

/* ---- pad: reset, power down, mux, MCLK ---- */

/* sensor-if reg: bank 0x1304 byte 0xfc, bit pair per phys channel, 1 = released */
static int pad_pin(u32 pad, int pin, u32 val)
{
	int p = pad2phys(pad);

	if (p < 0)
		return -1;
	wb(BANK(0x1304), 0xfc, 1 << (2 * p + pin), (val == 0) << (2 * p + pin));
	return 0;
}

int DrvVif_SensorReset(u32 pad, u32 val)
{
	return pad_pin(pad, 0, val);
}

int DrvVif_SensorPdwn(u32 pad, u32 val)
{
	return pad_pin(pad, 1, val);
}

/* MCLK sources (kHz -> select), the clkgen byte of each pad */
static int mclk_sel(u32 khz)
{
	switch (khz) {
	case 27000: return 0;
	case 72000: return 1;
	case 61700: return 2;
	case 54000: return 3;
	case 48000: return 4;
	case 43200: return 5;
	case 36000: return 6;
	case 24000: return 7;
	case 21600: return 8;
	case 12000: return 9;
	case 5400: return 10;
	case 37125: return 11;
	}
	return -1;
}

int DrvVif_QueryMCLK(u32 pad, u32 khz)
{
	if (khz == 37125)
		return pad < NCHN && v.mclk37[pad] ? 0 : -1;
	return mclk_sel(khz) < 0 ? -1 : 0;
}

int DrvVif_SetMclk(u32 pad, u32 on, u32 khz)
{
	static const u16 off[NCHN] = { 0x189, 0x195, 0x194 };
	int sel = on ? mclk_sel(khz) : -1;
	u8 val = sel < 0 ? 1 : sel << 2;

	if (pad >= NCHN)
		return -1;
	wb(BANK(0x1038), off[pad], 0x3f, val & 0x3d);
	return 0;
}

/* pad mux: 4 byte stride registers of bank 0x103c, per sensor pad fields */
int DrvVif_SetIOPad(u32 pad, u32 mode, u32 lanes)
{
	void __iomem *pt = BANK(0x103c);
	int i;

	if (pad > 2)
		return -2;
	wb(BANK(0x101e), 0x141, 0x80, 0);	/* all pad inputs off */
	if (pad != 0 || mode != 1) {
		pr_warn("vif: SetIOPad(%u, %u) not supported\n", pad, mode);
		return -1;
	}
	wb(pt, 0x1a4, 0x07, v.mipi_mode);
	wb(pt, 0x1a5, 0x03, v.mipi_pdn);
	wb(pt, 0x1a4, 0x40, v.mipi_rst << 6);
	wb(pt, 0x1a4, 0x10, v.mipi_ctrl << 4);
	for (i = 0; i < 10; i++)		/* data pad input enable off */
		wb(pt, 0xe0 + 4 * i, 0x50, 0);
	return 0;
}

int DrvVif_SetSelectSource(u32 src)
{
	u8 s = src == 1 ? 2 : src == 2 ? 1 : src;

	wb(cb_(0), 1, 0x1f, s);
	return 0;
}

/* ---- sensor input mask (frame skip) ---- */

int DrvVif_InputMask(u32 c, u32 mask, u32 owner)
{
	vlog("InputMask(%u, %u, %u)", c, mask, owner);
	if (c >= NCHN)
		return -1;
	if (mask)
		v.imask[c] |= 1 << (owner & 7);
	else
		v.imask[c] &= ~(1 << (owner & 7));
	return sensor_mask_set(c, !!(v.imask[c] & 7));
}

int DrvVif_InputMaskFrameSync(u32 c, u32 mask, u32 owner)
{
	DrvVif_InputMask(c, mask, owner);
	return 0;
}

void DrvVif_SetLineCntHit_MaskChange(void)
{
}

/* frame statistics of the monitor timer: { start interval, starts, irq 1, ends, line hits } */
int DrvVif_IntGet(u32 c, u32 *st)
{
	if (c >= NCHN)
		return -1;
	st[0] = v.fs_dt[c];
	st[1] = v.fs[c];
	st[2] = v.int1[c];
	st[3] = v.fe[c];
	st[4] = v.ln[c];
	return 0;
}

u64 DrvVif_GetSensorFrmCnt64(u32 chn)
{
	int p = chn2phys(chn);

	return p < 0 ? ~0ULL : v.frm[p];
}

u64 MHal_Vif_GetSensorFrmCnt64(u32 chn)
{
	return DrvVif_GetSensorFrmCnt64(chn);
}
EXPORT_SYMBOL(MHal_Vif_GetSensorFrmCnt64);

/* ---- CSI ---- */

/* clkgen bytes of the lptx, mac and ns clock of a csi */
static const u16 csi_clk[3][3] = {
	{ 0x160, 0x161, 0x164 }, { 0x16c, 0x16d, 0x170 }, { 0x165, 0x168, 0x169 },
};

int Drv_CSISetClk(u32 csi, u32 khz)
{
	int i;

	if (csi > 2)
		return -1;
	if (khz && khz != 216000)
		return -2;		/* the only frequency the vendor driver accepts */
	for (i = 0; i < 3; i++) {
		if (khz)
			wb(BANK(0x1038), csi_clk[csi][i], 0x1f, 0);	/* 216 MHz, running */
		else
			wb(BANK(0x1038), csi_clk[csi][i], 0x01, 1);	/* gated */
	}
	return 0;
}

int Drv_CSIQueryLaneNum(u32 csi, u8 *lanes)
{
	*lanes = csi < 3 ? v.lane_num[csi] : 0;
	return 0;
}

/* the 4 lane blocks: csi0 = banks 0x1208/9/a, csi1 = 0x1202/3/4 */
static bool csi_banks(u32 csi, void __iomem **ana, void __iomem **dig, void __iomem **mac)
{
	if (csi == 0) {
		*ana = BANK(0x1208); *dig = BANK(0x1209); *mac = BANK(0x120a);
	} else if (csi == 1) {
		*ana = BANK(0x1202); *dig = BANK(0x1203); *mac = BANK(0x1204);
	} else {
		return false;
	}
	return true;
}

int Drv_CSISetLane(u32 csi, u32 lanes, u32 unused)
{
	vlog("CSISetLane(%u, %u)", csi, lanes);
	void __iomem *ana, *dig, *mac;
	static const u16 slot[5] = { 0x18, 0x28, 0x38, 0x88, 0x94 };
	const u32 *sel, *pn;
	int i;

	if (!csi_banks(csi, &ana, &dig, &mac) || lanes != 4) {
		pr_warn("csi: SetLane(%u, %u) not supported\n", csi, lanes);
		return -1;
	}
	sel = v.lane_sel[csi];
	pn = v.lane_pn[csi];

	wb(dig, 0x100, 1, 0);				/* DIG soft reset */
	wb(dig, 0x100, 1, 1);
	wb(ana, 0x00, 0x43, 0x01);			/* soft reset off, VREG on, clock gating off */
	wb(ana, 0x04, 0x03, 0);				/* D-PHY powered */
	ww(ana, 0x60, 0);				/* HS/LP attributes */
	ww(ana, 0x6c, 0);
	ww(ana, 0x50, 0x4000);				/* test mode */
	/* MAC: clocks running, virtual channels in hsync mode, packet type filter */
	ww(mac, 0x60, 0);
	ww(mac, 0x64, 0);
	wb(mac, 0x80, 0x04, 0);				/* DontCareDt off */
	ww(mac, 0x04, 0x06db);
	ww(ana, 0x58, 0x1313);				/* HS skip (clock, data) */
	/* P/N swap and lane select of the 4 data lanes and the clock lane */
	for (i = 0; i < 5; i++)
		ww(ana, slot[i], (sel[i] & 7) << 12 | (pn[i] & 1));
	ww(ana, 0x64, (sel[1] << 2) << 8 | (sel[0] << 2));
	ww(ana, 0x68, sel[2] << 2);
	return 0;
}

int Drv_CSISetLongPacket(u32 csi, u32 a, u32 b, u32 c)
{
	void __iomem *ana, *dig, *mac;

	if (!csi_banks(csi, &ana, &dig, &mac))
		return -1;
	ww(mac, 0x74, a);
	ww(mac, 0x78, b);
	ww(mac, 0x7c, c);
	return 0;
}

int Drv_CSISetClkDataSkip(u32 csi, u32 clk, u32 data)
{
	void __iomem *ana, *dig, *mac;

	if (!csi_banks(csi, &ana, &dig, &mac))
		return -1;
	ww(ana, 0x58, (data & 0x1f) << 8 | (clk & 0x1f));
	return 0;
}

int Drv_CSISetHDRMode(u32 csi, u32 mode)
{
	if (mode)
		pr_warn("csi: HDR mode %u not supported\n", mode);
	return 0;
}

/* ---- MHal_VIF_* ---- */

static void vifisp_swreset(int c)
{
	void __iomem *isp = BANK(0x1302);

	wb(isp, 0x24, 1 << c, 0);	/* written as !x: reset on, off */
	wb(isp, 0x24, 1 << c, 1 << c);
}

int MHal_VIF_Init(void)
{
	void __iomem *isp = BANK(0x1302);
	int c, n = 0;

	for (c = 0; c < NCHN; c++) {
		DrvVif_InputMask(c, 1, 0);
		vifisp_swreset(c);
	}
	/* WDMA reset: idle the engine, reset pulse, arm */
	writeb(0, isp + 0x1e8);
	writeb(0, isp + 0x1e9);
	while ((readw(isp + 0x100) & 0x200) && n++ < 10000)
		udelay(1);
	wb(isp, 0x24, 0x08, 0);
	wb(isp, 0x24, 0x08, 0x08);
	writeb(0x5a, isp + 0x1e8);
	writeb(0x5a, isp + 0x1e9);
	for (c = 0; c < NCHN; c++) {
		irq_mask(c, 0x1ff);
		irq_unmask(c, 0x1ff & ~0x1fe);
	}
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_Init);

int MHal_VIF_Deinit(void)
{
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_Deinit);

int MHal_VIF_StopMCU(void)
{
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_StopMCU);

int MHal_VIF_RegisterGetPtsCallback(u64 (*fn)(void))
{
	v.get_pts = fn;
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_RegisterGetPtsCallback);

int MHal_VIF_DevSetConfig(u32 dev, const u32 *cfg, void *alloc)
{
	if (dev > 2 || !cfg)
		return -1;
	if (cfg[0] != 4) {		/* MIPI only */
		pr_warn("vif: interface mode %u not supported\n", cfg[0]);
		return -1;
	}
	v.mode[dev] = cfg[0];
	v.work[dev] = cfg[1];
	v.hdr[dev] = cfg[2];
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_DevSetConfig);

int MHal_VIF_DevEnable(u32 dev)
{
	return dev > 2 || v.mode[dev] != 4 ? -1 : 0;
}
EXPORT_SYMBOL(MHal_VIF_DevEnable);

int MHal_VIF_DevDisable(u32 dev)
{
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_DevDisable);

/* chn config: u16 x, y, w, h @0..6, u32 pixel format @0x10, bit depth enum @0x14 */
int MHal_VIF_ChnSetConfig(u32 chn, const u8 *cfg)
{
	int p = chn2phys(chn);
	void __iomem *b;
	u16 x, y, w, h;
	u32 cfg0 = cfg ? ((const u32 *)cfg)[0] : 0;
	u32 fmt, prec;
	u8 pad, hdrb, form, rgb;

	if (p < 0 || !cfg)
		return -1;
	vlog("ChnSetConfig(%u) cfg %08x", chn, cfg0);
	if (v.mode[p] != 4 || v.work[p] != 3) {
		pr_warn("vif: chn %u: work mode %u not supported\n", chn, v.work[p]);
		return -1;
	}
	if (DrvVif_GetDev2SNRPadID(p, &pad, &hdrb) < 0)
		return -1;
	x = *(u16 *)cfg;
	y = *(u16 *)(cfg + 2);
	w = *(u16 *)(cfg + 4);
	h = *(u16 *)(cfg + 6);
	fmt = *(u32 *)(cfg + 0x10);
	prec = *(u32 *)(cfg + 0x14);
	if (prec > 4)
		return -1;
	form = prec == 2 ? 3 : prec >= 3 ? 2 : prec;
	rgb = fmt && (fmt < 0xe || fmt > 0x10);
	b = cb_(p);

	wb(b, 0, 0x10, 0);			/* HDR off */
	wb(b, 0, 0x60, 0);
	wb(b, 0x19, 0x0c, form << 2);
	if (prec == 3)
		wb(b, 0x18, 0x0c, 1 << 2);
	wb(b, 0x19, 0x10, rgb << 4);
	field16(b, 0x40, x);			/* pixel crop */
	field16(b, 0x44, x + w - 1);
	field16(b, 0x48, y);			/* line crop */
	field16(b, 0x4c, y + h - 1);
	wb(b, 0x41, 0x80, 0x80);		/* crop on */
	wb(b, 1, 0x1f, pad == 1 ? 2 : pad == 2 ? 1 : pad);	/* source pad */
	/* pixel clock source (bank 0x1133): pad number mapping, no inversion */
	wb(BANK(0x1133), p == 0 ? 0xa0 : p == 1 ? 0xa1 : 0xa4, 0x1f,
	   (pad == 1 ? 2 : pad == 2 ? 1 : pad) << 2);
	field16(b, 0x50, h - h * 200 / 1000);	/* line threshold for the ISP */
	vlog("ChnSetConfig done hwmask %u", rb(b, 0x20) & 1);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_ChnSetConfig);

int MHal_VIF_ChnEnable(u32 chn)
{
	int p = chn2phys(chn);
	void __iomem *b;

	if (p < 0)
		return -1;
	b = cb_(p);
	vlog("ChnEnable(%u) hwmask %u", chn, rb(b, 0x20) & 1);
	irq_unmask(p, 0x20);			/* frame end */
	wb(b, 1, 0x80, 0x80);			/* channel enable */
	wb(b, 0, 1, 1);				/* sensor reset released */
	vlog("ChnEnable done hwmask %u", rb(b, 0x20) & 1);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_ChnEnable);

int MHal_VIF_ChnDisable(u32 chn)
{
	int p = chn2phys(chn);
	void __iomem *b;

	if (p < 0)
		return -1;
	b = cb_(p);
	vlog("ChnDisable(%u) hwmask %u", chn, rb(b, 0x20) & 1);
	irq_mask(p, 0x20);
	wb(b, 1, 0x80, 0);
	wb(b, 0, 1, 0);
	vlog("ChnDisable done hwmask %u", rb(b, 0x20) & 1);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_ChnDisable);

int MHal_VIF_ChnQuery(u32 chn, void *stat)
{
	if (chn2phys(chn) < 0)
		return -1;
	memset(stat, 0, 52);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_ChnQuery);

int MHal_VIF_SubChnQuery(u32 chn, u32 sub, void *stat)
{
	return -1;
}
EXPORT_SYMBOL(MHal_VIF_SubChnQuery);

int MHal_VIF_ChnReset(u32 chn, int what)
{
	int p = chn2phys(chn);

	if (p < 0 || what < 0 || what > 1)
		return -1;
	if (what == 0)
		vifisp_swreset(p);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_ChnReset);

/* type: 0 frame start, 1 frame end, 2 line count hit, 4 fifo full */
int MHal_VIF_RegisterEventCallback(u32 chn, u32 type, void *cb, void *unused, unsigned long x)
{
	int p = chn2phys(chn);

	if (p < 0 || type > 4 || type == 3)
		return -1;
	v.cb[p][type] = cb;
	v.cbx[p][type] = x;
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_RegisterEventCallback);

int MHal_VIF_QueueFrameBuffer(u32 chn, u32 sub, void *buf)
{
	return -1;
}
EXPORT_SYMBOL(MHal_VIF_QueueFrameBuffer);

int MHal_VIF_DequeueFrameBuffer(u32 chn, u32 sub, void *frame)
{
	return -1;
}
EXPORT_SYMBOL(MHal_VIF_DequeueFrameBuffer);

int MHal_VIF_QueryFrames(u32 chn, u32 sub, void *info)
{
	return -1;
}
EXPORT_SYMBOL(MHal_VIF_QueryFrames);

int MHal_VIF_GetInputMask(u32 chn, u8 *mask)
{
	int p = chn2phys(chn);

	if (p < 0)
		return -1;
	*mask = sensor_mask_get(p);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_GetInputMask);

int MHal_VIF_InputMask(u32 chn, u8 mask)
{
	int p = chn2phys(chn), i;

	if (p < 0)
		return -1;
	vlog("MHal_VIF_InputMask(%u, %u) cur %u", chn, mask, sensor_mask_get(p));
	if (sensor_mask_get(p) == (mask & 1))
		return 0;
	DrvVif_InputMask(p, mask, 0);
	/* the hardware takes the new mask at a frame boundary: wait until it reads back */
	for (i = 0; i < 2000; i++) {
		if (sensor_mask_get(p) == (mask & 1))
			return 0;
		usleep_range(500, 600);
	}
	vlog("InputMask(%u, %u) not taken", chn, mask);
	return -1;
}
EXPORT_SYMBOL(MHal_VIF_InputMask);

/* map: 3 entries of { u32 pad, u32 c } indexed by dev */
int MHal_VIF_SetVIFDev2SensorPadMux(const u32 *map, u32 n)
{
	u32 d;

	if (n != 3)
		return -1;
	for (d = 0; d < 3; d++)
		if (map[2 * d] != 0xff)
			DrvVif_SetDev2SNRPadMap(d, map[2 * d], map[2 * d + 1]);
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_SetVIFDev2SensorPadMux);

int MHal_VIF_GetVIFDev2SensorPadMux(u32 *map, u32 n)
{
	u8 pad, c;
	u32 d;

	if (n != 3)
		return -1;
	for (d = 0; d < 3; d++) {
		DrvVif_GetDev2SNRPadID(d, &pad, &c);
		map[2 * d] = pad;
		map[2 * d + 1] = c;
	}
	return 0;
}
EXPORT_SYMBOL(MHal_VIF_GetVIFDev2SensorPadMux);

static int dump_set(const char *val, const struct kernel_param *kp)
{
	int c;

	for (c = 0; c < NCHN; c++)
		pr_info("vif: chn %d: frame start %u end %u fifo full %u, mask %#x irq %#x\n",
			c, v.fs[c], v.fe[c], v.fifo[c], irq_get(c, 0x2c), irq_get(c, 0x38));
	return 0;
}

static const struct kernel_param_ops dump_ops = { .set = dump_set };
module_param_cb(vifdump, &dump_ops, NULL, 0200);
