// SPDX-License-Identifier: GPL-2.0
/*
 * Proxy sensor plug-in for the SigmaStar mhal sensor registry that drives
 * the mainline Linux IMX415 driver (drivers/media/i2c/imx415.c).
 *
 * The plug-in registers with mhal like the vendor plug-in (same object
 * layout and callbacks, see osm_snr.h) but contains no sensor register
 * tables: the imx415 V4L2 sub-device is looked up through the device
 * tree and
 *   - mhal power on/off   -> the sensor interface of the video input block
 *                            (reset and power-down pins, MIPI lanes, MCLK),
 *                            which is mhal's business, not a GPIO
 *   - mhal init           -> the sub-device s_stream(1) (full register setup)
 *   - mhal exposure       -> V4L2_CID_EXPOSURE (lines) and V4L2_CID_VBLANK
 *   - mhal gain           -> V4L2_CID_ANALOGUE_GAIN (first 30 dB) and
 *                            V4L2_CID_DIGITAL_GAIN (rest, up to 72 dB),
 *                            0.3 dB steps (digital gain needs the local
 *                            imx415 patch)
 *   - mhal flip / pattern -> V4L2_CID_HFLIP / VFLIP / TEST_PATTERN
 * The values are applied at frame start (ae_notify) inside the sensor
 * register hold, like the vendor plug-in does.
 *
 * Mode: the full 3864x2192 sensor output of the mainline driver, cropped by
 * the video input block to 3840x2160 at 20 fps (VBLANK stretches the frame).
 * The mainline driver offers 891 Mbps/lane with a 27 MHz INCK, not the
 * 594 Mbps of the vendor plug-in.
 *
 * Load order: this module first, then imx415. The reset and power-down pins
 * of the sensor are bits of the VIF sensor interface, not a GPIO, so imx415
 * cannot release them itself; the module init releases them (through mhal,
 * the same calls as the power-on callback) before imx415 probes. The
 * sub-device is bound lazily, at the first callback that needs it. mhal
 * resets the sensor again at power-on, which is fine: s_stream reprograms
 * all registers.
 *
 * Usage: insmod osm_snr_v4l2.ko chmap=1; insmod imx415.ko
 *   chmap: bit n = register the sensor on VIF pad n
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/int_log.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include "osm_compat.h"
#include "osm_snr.h"

#define R_REGHOLD	0x3001	/* IMX415 register hold */

/* output of the mainline driver (its whole pixel array) */
#define PIX_WIDTH	3864
#define PIX_HEIGHT	2192

#define MODE_FPS	20
#define MODE_WIDTH	3840
#define MODE_HEIGHT	2160

#define GAIN_MIN	1024
#define GAIN_MAX	0x3e3400	/* 72 dB */
#define US_MAX		333333

/* mhal VIF sensor interface (exported by mhal): v = 1 holds the pin */
extern int DrvVif_SensorPdwn(u32 pad, u32 v);
extern int DrvVif_SensorReset(u32 pad, u32 v);
extern int DrvVif_SetIOPad(u32 pad, u32 mode, u32 lanes);

struct osm_snr_priv {
	struct v4l2_subdev *sd;
	struct i2c_client *client;
	struct v4l2_ctrl *exposure, *again, *dgain, *vblank, *hblank;
	struct v4l2_ctrl *hflip, *vflip, *test;
	u32 width, height;	/* sensor output */
	u32 htotal;		/* width + hblank, pixels */
	u32 line_ns;
	u32 vmax_base;		/* VMAX at the mode's max_fps */
	u32 vmax;		/* VMAX wanted by the fps setting */
	u32 vmax_cur;		/* VMAX to program (raised for long exposures) */
	u32 fps;		/* last fps set, 1/1000 fps when >= 1000 */
	u32 gain;		/* mhal gain, 1024 = 1x */
	u32 us;
	u32 lines;		/* exposure in lines */
	u32 again_reg;		/* analogue part, 0.3 dB steps */
	u32 dgain_reg;		/* digital part, 0.3 dB steps */
	u32 orien;
	u32 pattern;
	bool ae_dirty;
	bool orien_dirty;
	bool pattern_dirty;
	bool streaming;
	bool bound;		/* sub-device and controls found */
};

/* the one imx415 sub-device, shared by all registered pads */
static struct v4l2_subdev *snr_sd;
static struct i2c_client *snr_client;

static int snr_bind(struct osm_snr_priv *p);

static void snr_set_vmax(struct osm_snr_priv *p, u32 vmax)
{
	p->vmax_cur = clamp(vmax, p->height + (u32)p->vblank->minimum,
			    p->height + (u32)p->vblank->maximum);
}

/* hold the sensor registers, so that exposure/gain/VMAX apply together */
static void snr_reghold(struct osm_snr_priv *p, u8 val)
{
	u8 buf[3] = { R_REGHOLD >> 8, R_REGHOLD & 0xff, val };

	i2c_master_send(p->client, buf, sizeof(buf));
}

static void snr_apply(struct osm_snr_priv *p)
{
	if (p->ae_dirty) {
		/* VBLANK first: it widens (or narrows) the exposure range */
		v4l2_ctrl_s_ctrl(p->vblank, p->vmax_cur - p->height);
		v4l2_ctrl_s_ctrl(p->exposure, p->lines);
		v4l2_ctrl_s_ctrl(p->again, p->again_reg);
		v4l2_ctrl_s_ctrl(p->dgain, p->dgain_reg);
		p->ae_dirty = false;
	}
	if (p->orien_dirty) {
		v4l2_ctrl_s_ctrl(p->hflip, p->orien & 1);
		v4l2_ctrl_s_ctrl(p->vflip, (p->orien >> 1) & 1);
		p->orien_dirty = false;
	}
	if (p->pattern_dirty) {
		v4l2_ctrl_s_ctrl(p->test, p->pattern ? 12 : 0);
		p->pattern_dirty = false;
	}
}

/* ---- callbacks ---- */

static int snr_poweron(struct osm_snr *h, u32 pad)
{
	struct osm_snr_if_ops *s = h->sif;

	s->pwdn(pad, h->pwdn_pol);
	s->reset(pad, h->reset_pol);
	s->io_pad(pad, h->if_mode, h->lanes);
	s->csi_clk(pad, 4);
	s->csi_lane(pad, (u16)h->lanes, 1);
	s->csi_pkt(pad, 0, 0x1c00, 0);
	if (h->hdr_mode == 1)
		s->hdr_mode(pad, 1);
	s->pwdn(pad, !h->pwdn_pol);
	msleep(31);
	s->reset(pad, !h->reset_pol);
	udelay(1);
	s->mclk(pad, 1, h->mclk);
	return 0;
}

static int snr_poweroff(struct osm_snr *h, u32 pad)
{
	struct osm_snr_priv *p = h->priv;
	struct osm_snr_if_ops *s = h->sif;

	if (p->streaming) {
		v4l2_subdev_call(p->sd, video, s_stream, 0);
		p->streaming = false;
	}
	s->pwdn(pad, h->pwdn_pol);
	s->reset(pad, h->reset_pol);
	s->mclk(pad, 0, h->mclk);
	s->csi_clk(pad, -1);
	if (h->hdr_mode == 1)
		s->hdr_mode(pad, 0);
	h->orien = 0;
	return 0;
}

/* The sensor was just reset: s_stream writes all registers and starts it */
static int snr_init(struct osm_snr *h)
{
	struct osm_snr_priv *p = h->priv;
	int ret;

	if (snr_bind(p))
		return -1;
	snr_apply(p);
	ret = v4l2_subdev_call(p->sd, video, s_stream, 1);
	if (ret) {
		pr_err("osm_snr_v4l2: s_stream failed: %d\n", ret);
		return ret;
	}
	p->streaming = true;
	return 0;
}

static int snr_release(struct osm_snr *h)
{
	return 0;
}

static int snr_get_id(struct osm_snr *h, u32 *id)
{
	/* the sub-device has probed, so the chip id was checked already */
	*id = 0x514;
	return 0;
}

static int snr_get_res_num(struct osm_snr *h, u32 *n)
{
	*n = h->res_cnt;
	return 0;
}

static int snr_get_res(struct osm_snr *h, u32 idx, struct osm_snr_res **r)
{
	if (idx >= h->res_cnt) {
		pr_err("osm_snr_v4l2: bad resolution index %u\n", idx);
		return -1;
	}
	*r = &h->res[idx];
	return 0;
}

static int snr_get_cur_res(struct osm_snr *h, u32 *idx, struct osm_snr_res **r)
{
	*idx = h->res_cur;
	if (h->res_cur >= h->res_cnt) {
		pr_err("osm_snr_v4l2: bad current resolution\n");
		return -1;
	}
	*r = &h->res[h->res_cur];
	return 0;
}

static int snr_set_res(struct osm_snr *h, u32 idx)
{
	struct osm_snr_priv *p = h->priv;

	if (idx >= h->res_cnt) {
		pr_err("osm_snr_v4l2: bad resolution index %u\n", idx);
		return -1;
	}
	if (snr_bind(p))
		return -1;
	h->res_cur = idx;
	h->init = snr_init;
	p->vmax = p->vmax_base;
	p->fps = MODE_FPS;
	snr_set_vmax(p, p->vmax);
	p->lines = p->vmax_cur - 8;	/* longest exposure until AE runs */
	p->us = 0;
	p->ae_dirty = true;
	h->f7c = 1;
	return 0;
}

static int snr_get_orien(struct osm_snr *h, u32 *o)
{
	struct osm_snr_priv *p = h->priv;

	*o = p->orien & 3;
	return 0;
}

static int snr_set_orien(struct osm_snr *h, u32 o)
{
	struct osm_snr_priv *p = h->priv;

	o = o <= 3 ? o : 0;
	h->orien = o;
	p->orien = o;
	p->orien_dirty = true;
	return 0;
}

static int snr_set_pattern(struct osm_snr *h, u32 mode)
{
	struct osm_snr_priv *p = h->priv;

	p->pattern = mode == 1;
	p->pattern_dirty = true;
	return 0;
}

static int snr_ae_notify(struct osm_snr *h, u32 status)
{
	struct osm_snr_priv *p = h->priv;

	if (status != 1 || !p->bound ||
	    (!p->ae_dirty && !p->orien_dirty && !p->pattern_dirty))
		return 0;

	snr_reghold(p, 1);
	snr_apply(p);
	snr_reghold(p, 0);
	return 0;
}

/*
 * The vendor plug-in reports the SHR0 register scaled by the line time
 * here (not the exposure); mhal tracks the exposure itself, keep it.
 */
static int snr_get_us(struct osm_snr *h, u32 *us)
{
	struct osm_snr_priv *p = h->priv;
	u32 shr = p->vmax_cur - p->lines;

	if (!p->bound)
		return -1;
	*us = div_u64((u64)shr * p->line_ns, 1000);
	return 0;
}

static int snr_set_us(struct osm_snr *h, u32 us)
{
	struct osm_snr_priv *p = h->priv;
	u32 lines;

	if (snr_bind(p))
		return -1;
	lines = 1000 * us / p->line_ns;
	p->us = us;
	lines = max(lines, (u32)p->exposure->minimum);
	/* the exposure may be at most VMAX - 8 lines: stretch the frame */
	snr_set_vmax(p, max(p->vmax, lines + 8));
	p->lines = min(lines, p->vmax_cur - 8);
	p->ae_dirty = true;
	return 0;
}

static int snr_get_gain(struct osm_snr *h, u32 *g)
{
	struct osm_snr_priv *p = h->priv;

	*g = p->gain;
	return 0;
}

/* gain (1024 = 1x) to 0.3 dB steps: 200 * log10(gain / 1024) / 3 */
static u32 gain_to_reg(u32 gain)
{
	u32 x;

	gain = clamp(gain, GAIN_MIN, GAIN_MAX);
	x = (u32)(((u64)(intlog10(gain) - intlog10(GAIN_MIN)) * 200) >> 24);
	return (x & 0xffff) / 3;
}

static int snr_set_gain(struct osm_snr *h, u32 gain)
{
	struct osm_snr_priv *p = h->priv;

	if (snr_bind(p))
		return -1;
	u32 reg = gain_to_reg(gain);

	p->gain = gain;
	/* the first 30 dB are analogue gain, the rest digital gain */
	p->again_reg = min_t(u32, reg, p->again->maximum);
	p->dgain_reg = min_t(u32, reg - p->again_reg, p->dgain->maximum);
	p->ae_dirty = true;
	return 0;
}

static int snr_minmax_us(struct osm_snr *h, u32 *min, u32 *max)
{
	*min = 1;
	*max = US_MAX;
	return 0;
}

static int snr_minmax_gain(struct osm_snr *h, u32 *min, u32 *max)
{
	*min = GAIN_MIN;
	*max = GAIN_MAX;
	return 0;
}

static int snr_shutter_info(struct osm_snr *h, struct osm_snr_shutter_info *i)
{
	struct osm_snr_priv *p = h->priv;

	if (snr_bind(p))
		return -1;
	i->max_ns = 333333333;
	i->step_ns = p->line_ns;
	i->min_ns = p->line_ns;
	return 0;
}

static u32 snr_get_fps(struct osm_snr *h)
{
	struct osm_snr_priv *p = h->priv;
	struct osm_snr_res *r = &h->res[h->res_cur];
	u64 n = (u64)r->max_fps * p->vmax_base;

	if (!p->bound)
		return 0;
	if (p->fps >= 1000)
		n *= 1000;
	return p->vmax_cur ? div_u64(n, p->vmax_cur) : 0;
}

static int snr_set_fps(struct osm_snr *h, u32 fps)
{
	struct osm_snr_priv *p = h->priv;
	struct osm_snr_res *r = &h->res[h->res_cur];
	u64 n;

	if (snr_bind(p))
		return -1;
	n = (u64)r->max_fps * p->vmax_base * 1000;
	if (fps >= r->min_fps && fps <= r->max_fps) {
		p->vmax = div_u64(n + 500 * fps, 1000 * fps);
	} else if (fps >= r->min_fps * 1000 && fps <= r->max_fps * 1000) {
		p->vmax = div_u64(n + (fps >> 1), fps);
	} else {
		return -1;
	}
	p->fps = fps;
	if (p->us)
		return snr_set_us(h, p->us);	/* VMAX changed: redo exposure */
	snr_set_vmax(p, p->vmax);
	p->lines = min(p->lines, p->vmax_cur - 8);
	p->ae_dirty = true;
	return 0;
}

/* ---- init handle: mhal calls this once per pad ---- */

/* sensor geometry and timing from the sub-device */
static int snr_read_mode(struct osm_snr_priv *p)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	ret = v4l2_subdev_call_state_active(p->sd, pad, get_fmt, &fmt);
	if (ret)
		return ret;
	p->width = fmt.format.width;
	p->height = fmt.format.height;
	if (p->width != PIX_WIDTH || p->height != PIX_HEIGHT)
		return -EINVAL;

	p->htotal = p->width + v4l2_ctrl_g_ctrl(p->hblank);
	p->line_ns = div_u64(1000000000ULL * p->htotal,
			     v4l2_ctrl_g_ctrl_int64(v4l2_ctrl_find(
					p->sd->ctrl_handler,
					V4L2_CID_PIXEL_RATE)));
	/* VMAX of the 20 fps frame: 1 s / (line time * fps) */
	p->vmax_base = div_u64(1000000000ULL, (u64)p->line_ns * MODE_FPS);
	return 0;
}

/* the sub-device of the imx415 node, which must have probed */
static int snr_find_subdev(void)
{
	struct device_node *np;

	if (snr_sd)
		return 0;
	np = of_find_compatible_node(NULL, NULL, "sony,imx415");
	if (!np)
		return -ENODEV;
	snr_client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!snr_client)
		return -ENODEV;
	if (!snr_client->dev.driver ||
	    !try_module_get(snr_client->dev.driver->owner)) {
		pr_err("osm_snr_v4l2: imx415 has not probed (load imx415)\n");
		kobject_put(&snr_client->dev.kobj);
		snr_client = NULL;
		return -ENODEV;
	}
	snr_sd = i2c_get_clientdata(snr_client);
	return 0;
}

static void snr_put_subdev(void)
{
	if (!snr_client)
		return;
	module_put(snr_client->dev.driver->owner);
	kobject_put(&snr_client->dev.kobj);
	snr_client = NULL;
	snr_sd = NULL;
}

/* find the sub-device and its controls, once */
static int snr_bind(struct osm_snr_priv *p)
{
	struct v4l2_ctrl_handler *hdl;

	if (p->bound)
		return 0;
	if (snr_find_subdev())
		return -ENODEV;

	p->sd = snr_sd;
	p->client = snr_client;
	hdl = p->sd->ctrl_handler;
	p->exposure = v4l2_ctrl_find(hdl, V4L2_CID_EXPOSURE);
	p->again = v4l2_ctrl_find(hdl, V4L2_CID_ANALOGUE_GAIN);
	p->dgain = v4l2_ctrl_find(hdl, V4L2_CID_DIGITAL_GAIN);
	p->vblank = v4l2_ctrl_find(hdl, V4L2_CID_VBLANK);
	p->hblank = v4l2_ctrl_find(hdl, V4L2_CID_HBLANK);
	p->hflip = v4l2_ctrl_find(hdl, V4L2_CID_HFLIP);
	p->vflip = v4l2_ctrl_find(hdl, V4L2_CID_VFLIP);
	p->test = v4l2_ctrl_find(hdl, V4L2_CID_TEST_PATTERN);
	if (!p->exposure || !p->again || !p->dgain || !p->vblank || !p->hblank ||
	    !p->hflip || !p->vflip || !p->test ||
	    !v4l2_ctrl_find(hdl, V4L2_CID_PIXEL_RATE) || snr_read_mode(p)) {
		pr_err("osm_snr_v4l2: unusable imx415 sub-device\n");
		return -EINVAL;
	}
	p->vmax = p->vmax_base;
	p->fps = MODE_FPS;
	snr_set_vmax(p, p->vmax);
	p->lines = p->vmax_cur - 8;
	p->ae_dirty = true;
	p->bound = true;
	return 0;
}

static int snr_init_handle(struct osm_snr *h)
{
	struct osm_snr_priv *p;
	struct osm_snr_res *r;

	if (!h)
		return -1;
	p = h->priv;
	if (!p) {
		pr_err("osm_snr_v4l2: private data is empty\n");
		return -1;
	}

	strcpy(h->name, "IMX415_MIPI");
	h->i2c_cfg.mode = 1;
	h->i2c_cfg.fmt = 1;
	h->i2c_cfg.speed = 300000;
	h->i2c_cfg.addr = 0x34;
	h->mclk = 0;
	h->f6c = 1;
	h->f70 = 3;
	h->f74 = 0;
	h->if_mode = 1;
	h->f7c = 1;
	h->f80 = 4;
	h->orien = 0;
	h->lanes = 4;
	h->f3fc = 1;
	h->f400 = 0;
	h->f404 = 0;
	h->hdr_mode = 0;
	h->f410 = 0;
	h->res_cur = 0;
	h->res_cnt = 1;
	h->f48 = 2;
	h->f4c = 2;
	h->f50 = 1;
	h->f54 = 1;
	h->reset_pol = 1;
	h->pwdn_pol = 1;
	h->gain_unit = 1024;

	r = &h->res[0];
	r->width = MODE_WIDTH;
	r->height = MODE_HEIGHT;
	r->max_fps = MODE_FPS;
	r->min_fps = 3;
	r->crop_x = (PIX_WIDTH - MODE_WIDTH) / 2;
	r->crop_y = (PIX_HEIGHT - MODE_HEIGHT) / 2;
	r->sns_width = 3860;
	r->sns_height = 2250;
	strcpy(r->name, "3840x2160@20fps");

	h->poweron = snr_poweron;
	h->poweroff = snr_poweroff;
	h->init = snr_init;
	h->release = snr_release;
	h->set_pattern = snr_set_pattern;
	h->get_id = snr_get_id;
	h->get_res = snr_get_res;
	h->get_cur_res = snr_get_cur_res;
	h->set_res = snr_set_res;
	h->get_orien = snr_get_orien;
	h->set_orien = snr_set_orien;
	h->ae_notify = snr_ae_notify;
	h->get_us = snr_get_us;
	h->set_us = snr_set_us;
	h->get_gain = snr_get_gain;
	h->set_gain = snr_set_gain;
	h->minmax_us = snr_minmax_us;
	h->minmax_gain = snr_minmax_gain;
	h->get_fps = snr_get_fps;
	h->set_fps = snr_set_fps;
	h->shutter_info = snr_shutter_info;
	h->get_res_num = snr_get_res_num;
	return 0;
}

/* ---- module ---- */

static int chmap;
module_param(chmap, int, 0444);
MODULE_PARM_DESC(chmap, "VIF channel mapping (bit n = pad n)");

static struct osm_snr_priv *priv[OSM_SNR_PADS];

/*
 * Release the sensor from power-down and reset like the power-on callback
 * does, so that imx415 (loaded next) can talk to it in its probe. MCLK is
 * enabled by imx415 itself, after the release, as in the vendor order.
 */
static void snr_release_pins(u32 pad)
{
	int ret;

	DrvVif_SensorPdwn(pad, 1);
	DrvVif_SensorReset(pad, 1);
	DrvVif_SetIOPad(pad, 1, 4);
	DrvVif_SensorPdwn(pad, 0);
	msleep(31);
	ret = DrvVif_SensorReset(pad, 0);
	udelay(1);
	if (ret)
		pr_warn("osm_snr_v4l2: releasing the sensor reset failed: %d\n",
			ret);
}

static int __init osm_snr_v4l2_init(void)
{
	struct osm_entry e;
	int pad, ret = 0;

	osm_enter(&e);
	if (DrvSensorHandleVer(2, 2) == -1 || DrvSensorIFVer(1, 2) == -1 ||
	    DrvSensorI2CVer(1, 1) == -1) {
		ret = -EINVAL;
		goto out;
	}
	for (pad = 0; pad < OSM_SNR_PADS; pad++) {
		if (!((chmap >> pad) & 1))
			continue;
		priv[pad] = kzalloc(sizeof(*priv[pad]), GFP_KERNEL);
		if (!priv[pad]) {
			ret = -ENOMEM;
			goto out;
		}
		snr_release_pins(pad);
		DrvRegisterSensorDriverEx(pad, snr_init_handle, priv[pad]);
		DrvRegisterSensorI2CSlaveID(pad, 0);
		pr_info("osm_snr_v4l2: registered on pad %d\n", pad);
	}
out:
	osm_leave(&e);
	if (ret) {
		for (pad = 0; pad < OSM_SNR_PADS; pad++) {
			kfree(priv[pad]);
			priv[pad] = NULL;
		}
	}
	return ret;
}

static void __exit osm_snr_v4l2_exit(void)
{
	struct osm_entry e;
	int pad;

	osm_enter(&e);
	for (pad = 0; pad < OSM_SNR_PADS; pad++) {
		if (!priv[pad])
			continue;
		DrvSensorRelease(pad);
		kfree(priv[pad]);
		priv[pad] = NULL;
	}
	osm_leave(&e);
	snr_put_subdev();
}

module_init(osm_snr_v4l2_init);
module_exit(osm_snr_v4l2_exit);
MODULE_DESCRIPTION("mhal sensor plug-in driving the mainline IMX415 V4L2 sub-device");
MODULE_AUTHOR("opensigmastar");
MODULE_LICENSE("GPL");
