// SPDX-License-Identifier: GPL-2.0
/*
 * osm_v4l2 backend: sensor -> VIF -> VPE (ISP + SCL) over the vendor MI
 * modules, called in the kernel like the MI userspace libraries call them
 * through the ioctls (the kernel entry points take the same arguments
 * and structures; struct layouts below were checked against what the mi_*
 * modules read, offsets are noted where they matter). Frames are taken from
 * the VPE output port with mi_sys and copied into the V4L2 buffer.
 *
 * Needs the vendor stack loaded (scripts/osm-load.sh, no userspace
 * libraries): the kernel 3A in mhal runs the exposure, the ISP tuning bin
 * is applied with osm_isp_load_bin() once the pipeline runs.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include "osm_compat.h"
#include "osm_isp.h"
#include "osm_sstar.h"
#include "osm_v4l2.h"

static uint sensor;
module_param(sensor, uint, 0444);
MODULE_PARM_DESC(sensor, "sensor pad (default 0)");
static char isp_bin[128];
module_param_string(isp_bin, isp_bin, sizeof(isp_bin), 0444);
MODULE_PARM_DESC(isp_bin, "ISP tuning bin (firmware name) applied at stream start, e.g. sstar/imx415.bin");
static uint dump_frames = 2;
module_param(dump_frames, uint, 0644);
MODULE_PARM_DESC(dump_frames, "log the raw mi_sys buffer info of the first N frames");

/* vendor entry points: MI kernel API (exported) and MI_*_IMPL (osm exports) */
extern int MI_VIF_SetDevAttr(u32 dev, const void *attr);
extern int MI_VIF_EnableDev(u32 dev);
extern int MI_VIF_DisableDev(u32 dev);
extern int MI_VIF_SetChnPortAttr(u32 chn, u32 port, const void *attr);
extern int MI_VIF_EnableChnPort(u32 chn, u32 port);
extern int MI_VIF_DisableChnPort(u32 chn, u32 port);
extern int MI_VPE_CreateChannel(u32 chn, const void *attr);
extern int MI_VPE_DestroyChannel(u32 chn);
extern int MI_VPE_SetChannelParam(u32 chn, const void *param);
extern int MI_VPE_StartChannel(u32 chn);
extern int MI_VPE_StopChannel(u32 chn);
extern int MI_VPE_SetPortMode(u32 chn, u32 port, const void *mode);
extern int MI_VPE_EnablePort(u32 chn, u32 port);
extern int MI_VPE_DisablePort(u32 chn, u32 port);
extern int MI_SYS_IMPL_Init(void);
extern int MI_SYS_IMPL_Exit(void);
extern int MI_SNR_IMPL_Init(void);
extern int MI_SNR_IMPL_DeInit(void);
extern int MI_VIF_IMPL_Init(void);
extern int MI_VIF_IMPL_DeInit(void);
extern int MI_VPE_IMPL_Init(void);
extern int MI_VPE_IMPL_DeInit(void);
extern int MI_SNR_IMPL_SetPlaneMode(u32 snr, u8 on);
extern int MI_SNR_IMPL_QueryResCount(u32 snr, u32 *count);
extern int MI_SNR_IMPL_GetRes(u32 snr, u8 idx, void *res);
extern int MI_SNR_IMPL_SetRes(u32 snr, u8 idx);
extern int MI_SNR_IMPL_SetFps(u32 snr, u32 fps);
extern int MI_SNR_IMPL_SetOrien(u32 snr, u8 mirror, u8 flip);
extern int MI_SNR_IMPL_GetPadInfo(u32 snr, void *pad);
extern int MI_SNR_IMPL_GetPlaneInfo(u32 snr, u32 plane, void *info);
extern int MI_SNR_IMPL_Enable(u32 snr);
extern int MI_SNR_IMPL_Disable(u32 snr);
extern int MI_SYS_IMPL_BindChnPort2(const void *src, const void *dst, u32 sfps,
				    u32 dfps, u32 link, u32 param);
extern int MI_SYS_IMPL_UnBindChnPort(const void *src, const void *dst);
extern int MI_SYS_IMPL_SetChnOutputPortDepth(const void *port, u32 user, u32 buf);
extern int mi_sys_ChnOutputPortGetBufPa(const void *port, void *info, int *handle);
extern int mi_sys_ChnOutputPortPutBuf(int handle);

extern int MI_VENC_IMPL_Init(void);
extern int MI_VENC_IMPL_DeInit(void);
extern int MI_VENC_IMPL_CreateChn(u32 chn, const void *attr);
extern int MI_VENC_IMPL_DestroyChn(u32 chn);
extern int MI_VENC_IMPL_StartRecvPic(u32 chn);
extern int MI_VENC_IMPL_StopRecvPic(u32 chn);
extern int MI_VENC_IMPL_GetChnDevid(u32 chn, u32 *dev);
extern int MI_VENC_IMPL_GetRingPoolHandle(u32 chn, u32 *handle);
extern int MI_VENC_IMPL_GetChnAttr(u32 chn, void *attr);
extern int MI_VENC_IMPL_SetChnAttr(u32 chn, const void *attr);
extern int MI_VENC_IMPL_SetJpegParam(u32 chn, const void *param);
extern int MI_VENC_IMPL_GetJpegParam(u32 chn, void *param);
extern int MI_VENC_IMPL_RequestIdr(u32 chn, u8 instant);
extern int MI_VENC_IMPL_Query(u32 chn, void *stat);
extern int mi_sys_RingPool_GetHeadRecordTimeout(u32 handle, void *rec, u32 flag, u32 ms);
extern int mi_sys_RingPool_ReleaseHeadRecord(u32 handle, u32 flag);
extern int mi_sys_RingPool_GetRingPoolTotalSpace(u32 handle, void *info);

#define MI_ERR_NOBUF		0xa009200d	/* no buffer on the port */

/* enums of the MI API used here */
#define MOD_VIF			6
#define MOD_VPE			7
#define MOD_VENC		2
#define INTF_MIPI		4
#define VIF_WORK_RGB_REALTIME	3
#define EDGE_DOUBLE		2
#define PIXFMT_YUV420SP		11
#define PIXFMT_YUV422_YUYV	0
#define PIXFMT_RGB_BAYER	20
#define BAYER_END		12
#define VPE_MODE_REALTIME	0x18
#define LINK_REALTIME		4
#define LINK_FRAMEBASE		1

/* MI_SYS_ChnPort: module, device, channel, port */
struct chn_port {
	u32 module, dev, chn, port;
};

/* VIF device attribute (52 bytes read by the vendor code, 13 words) */
struct vif_dev_attr {
	u32 intf, work, hdr, edge, input, bitswap;
	u32 sync[6];
	u32 ext;
};

/* VIF port attribute (32 bytes) */
struct vif_port_attr {
	u16 x, y, w, h;
	u16 dest_w, dest_h;
	u32 field, interlace, pixfmt, frate, line_cnt;
};

/* VPE channel attribute for this chip (192 bytes) */
struct vpe_chn_attr {
	u16 w, h;
	u32 pixfmt, hdr, sensor;
	u8 nr, edge, edge_smooth, contrast, invert, rotate;
	u8 pad0[2];
	u32 mode;
	u8 rest[192 - 28];
};

/* VPE channel parameter (100 bytes); hdr at 88, 3DNR level at 92 */
struct vpe_chn_param {
	u8 raw[88];
	u32 hdr;
	u32 level_3dnr;
	u8 mirror, flip, reserved, lens_adj;
};

/* VPE port mode (16 bytes) */
struct vpe_port_mode {
	u16 w, h;
	u8 mirror, flip;
	u8 pad[2];
	u32 pixfmt, compress;
};

#define VPE_PORT_NV12		0
#define VPE_PORT_H264		1
#define VPE_PORT_MJPEG		2

struct pipe {
	struct mutex lock;
	bool up;		/* sensor, VIF and the VPE channel run */
	bool bound;		/* VIF -> VPE bound */
	u32 ports;		/* VPE ports in use (bit per port) */
	u32 snr;
	struct chn_port vif, vpe;
	struct mutex first_lock;
	u32 frames;
	struct mutex ctl_lock;	/* ctl_* below and ISP API access */
	bool ctl_ready;		/* the 3A accepts API calls (after the bin) */
	u32 ctl_mask;		/* values waiting for ctl_ready */
	s32 ctl_val[OSM_CTRL_NUM];
	int inits;		/* IMPL_Init calls done: 0..5 */
	/* encoder (one channel, bound to a VPE port of the same number) */
	bool enc_on;
	u32 enc_chn;		/* = VPE port */
	u32 enc_dev;
	u32 enc_ring;		/* ring pool handle of the channel */
	u64 ring_base;		/* start of the ring pool memory (MIU address) */
	u32 ring_size;
	struct osm_enc_params enc;
	u32 enc_fps_den;
	bool enc_idr_wait;
};

static struct pipe pipe_state;

#define MI(ret_var, call) do {			\
	struct osm_entry _e;			\
	osm_enter(&_e);				\
	ret_var = (call);			\
	osm_leave(&_e);				\
} while (0)

static int mi_check(const char *what, int ret)
{
	if (ret)
		pr_err("osm_pipe: %s failed: 0x%x\n", what, ret);
	return ret ? -EIO : 0;
}

#define STEP(ret, what, call) do {		\
	MI(ret, call);				\
	ret = mi_check(what, ret);		\
	if (ret)				\
		goto fail;			\
} while (0)

#define DOWN(what, call) do {			\
	int _r;					\
	MI(_r, call);				\
	mi_check(what, _r);			\
} while (0)

/* the "open" of the MI devices stays for the life of the module */
static void pipe_deinit(struct pipe *p)
{
	int r;

	if (p->inits > 4)
		MI(r, MI_VENC_IMPL_DeInit());
	if (p->inits > 3)
		MI(r, MI_VPE_IMPL_DeInit());
	if (p->inits > 2)
		MI(r, MI_VIF_IMPL_DeInit());
	if (p->inits > 1)
		MI(r, MI_SNR_IMPL_DeInit());
	if (p->inits > 0)
		MI(r, MI_SYS_IMPL_Exit());
	p->inits = 0;
}

/* p->lock held. Sensor, VIF and the VPE channel; the first port follows. */
static int base_up(struct pipe *p, u32 width, u32 height, u32 fps)
{
	struct scratch {
		u8 pad[256], plane[256], res[256];
		struct vif_dev_attr dev;
		struct vif_port_attr port;
		struct vpe_chn_attr chn;
		struct vpe_chn_param param;
	} *sc;
	u8 *pad, *plane, *res;
	struct vif_dev_attr *dev;
	struct vif_port_attr *port;
	struct vpe_chn_attr *chn;
	struct vpe_chn_param *param;
	u32 count = 0, i, pixfmt;
	int idx = -1, ret;

	sc = kzalloc(sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	pad = sc->pad;
	plane = sc->plane;
	res = sc->res;
	dev = &sc->dev;
	port = &sc->port;
	chn = &sc->chn;
	param = &sc->param;

	p->snr = sensor;
	p->vif = (struct chn_port){ MOD_VIF, 0, 0, 0 };
	p->vpe = (struct chn_port){ MOD_VPE, 0, 0, 0 };
	p->frames = 0;
	mutex_lock(&p->ctl_lock);
	p->ctl_ready = false;
	p->ctl_mask = 0;
	mutex_unlock(&p->ctl_lock);

	/* what the MI device layer does when a process opens the devices */
	if (!p->inits) {
		STEP(ret, "SYS Init", MI_SYS_IMPL_Init());
		p->inits = 1;
		STEP(ret, "SNR Init", MI_SNR_IMPL_Init());
		p->inits = 2;
		STEP(ret, "VIF Init", MI_VIF_IMPL_Init());
		p->inits = 3;
		STEP(ret, "VPE Init", MI_VPE_IMPL_Init());
		p->inits = 4;
		STEP(ret, "VENC Init", MI_VENC_IMPL_Init());
		p->inits = 5;
	}
	STEP(ret, "SNR SetPlaneMode", MI_SNR_IMPL_SetPlaneMode(p->snr, 0));
	STEP(ret, "SNR QueryResCount", MI_SNR_IMPL_QueryResCount(p->snr, &count));
	/* first sensor mode that covers the output size at this frame rate */
	for (i = 0; i < count; i++) {
		u16 *crop = (u16 *)res;	/* x, y, w, h */
		u32 *fp = (u32 *)res;	/* fp[3] = max fps */

		STEP(ret, "SNR GetRes", MI_SNR_IMPL_GetRes(p->snr, i, res));
		if (crop[2] >= width && crop[3] >= height && fp[3] >= fps) {
			idx = i;
			break;
		}
	}
	if (idx < 0) {
		pr_err("osm_pipe: no sensor mode for %ux%u@%u\n", width, height, fps);
		ret = -EINVAL;
		goto out;
	}
	STEP(ret, "SNR SetRes", MI_SNR_IMPL_SetRes(p->snr, idx));
	STEP(ret, "SNR SetFps", MI_SNR_IMPL_SetFps(p->snr, fps));
	STEP(ret, "SNR SetOrien", MI_SNR_IMPL_SetOrien(p->snr, 0, 0));
	STEP(ret, "SNR GetPadInfo", MI_SNR_IMPL_GetPadInfo(p->snr, pad));
	STEP(ret, "SNR GetPlaneInfo", MI_SNR_IMPL_GetPlaneInfo(p->snr, 0, plane));
	STEP(ret, "SNR Enable", MI_SNR_IMPL_Enable(p->snr));

	/* plane info: capt rect u16 x/y/w/h at 36, bayer 44, precision 48 */
	{
		u16 *capt = (u16 *)(plane + 36);
		u32 bayer = *(u32 *)(plane + 44), prec = *(u32 *)(plane + 48);

		pixfmt = PIXFMT_RGB_BAYER + prec * BAYER_END + bayer;
		port->x = capt[0];
		port->y = capt[1];
		port->w = capt[2];
		port->h = capt[3];
		port->dest_w = capt[2];
		port->dest_h = capt[3];
		chn->w = capt[2];
		chn->h = capt[3];
		pr_info("osm_pipe: sensor %.32s %ux%u bayer %u prec %u\n", plane + 4,
			capt[2], capt[3], bayer, prec);
	}
	/* pad info: interface at 4, MIPI input format at 20 */
	dev->intf = INTF_MIPI;
	dev->work = VIF_WORK_RGB_REALTIME;
	dev->edge = EDGE_DOUBLE;
	dev->input = *(u32 *)(pad + 20);
	STEP(ret, "VIF SetDevAttr", MI_VIF_SetDevAttr(0, dev));
	STEP(ret, "VIF EnableDev", MI_VIF_EnableDev(0));
	port->pixfmt = pixfmt;
	STEP(ret, "VIF SetChnPortAttr", MI_VIF_SetChnPortAttr(0, 0, port));
	STEP(ret, "VIF EnableChnPort", MI_VIF_EnableChnPort(0, 0));

	chn->pixfmt = pixfmt;
	chn->sensor = p->snr + 1;
	chn->mode = VPE_MODE_REALTIME;
	STEP(ret, "VPE CreateChannel", MI_VPE_CreateChannel(0, chn));
	param->level_3dnr = 1;
	STEP(ret, "VPE SetChannelParam", MI_VPE_SetChannelParam(0, param));
	STEP(ret, "VPE StartChannel", MI_VPE_StartChannel(0));
	p->up = true;
	ret = 0;
	goto out;
fail:
	/* the steps above run in the order of the teardown below */
	DOWN("VPE StopChannel", MI_VPE_StopChannel(0));
	DOWN("VPE DestroyChannel", MI_VPE_DestroyChannel(0));
	DOWN("VIF DisableChnPort", MI_VIF_DisableChnPort(0, 0));
	DOWN("VIF DisableDev", MI_VIF_DisableDev(0));
	DOWN("SNR Disable", MI_SNR_IMPL_Disable(p->snr));
out:
	kfree(sc);
	return ret;
}

/* p->lock held, no VPE port in use any more */
static void base_down(struct pipe *p)
{
	if (p->bound)
		DOWN("SYS UnBind", MI_SYS_IMPL_UnBindChnPort(&p->vif, &p->vpe));
	p->bound = false;
	DOWN("VPE StopChannel", MI_VPE_StopChannel(0));
	DOWN("VPE DestroyChannel", MI_VPE_DestroyChannel(0));
	DOWN("VIF DisableChnPort", MI_VIF_DisableChnPort(0, 0));
	DOWN("VIF DisableDev", MI_VIF_DisableDev(0));
	DOWN("SNR Disable", MI_SNR_IMPL_Disable(p->snr));
	p->up = false;
}

/* p->lock held, base up: enable one VPE output port */
static int vpe_port_on(struct pipe *p, u32 port, u32 w, u32 h, u32 pixfmt, u32 fps)
{
	struct vpe_port_mode pm = { .w = w, .h = h, .pixfmt = pixfmt };
	int ret;

	STEP(ret, "VPE SetPortMode", MI_VPE_SetPortMode(0, port, &pm));
	STEP(ret, "VPE EnablePort", MI_VPE_EnablePort(0, port));
	p->ports |= BIT(port);
	ret = 0;
fail:
	return ret;
}

static void vpe_port_off(struct pipe *p, u32 port)
{
	DOWN("VPE DisablePort", MI_VPE_DisablePort(0, port));
	p->ports &= ~BIT(port);
}

/* p->lock held: connect the sensor to the VPE (after the first port) */
static int vif_bind(struct pipe *p, u32 fps)
{
	int ret;

	if (p->bound)
		return 0;
	STEP(ret, "SYS Bind", MI_SYS_IMPL_BindChnPort2(&p->vif, &p->vpe, fps, fps,
						       LINK_REALTIME, 0));
	p->bound = true;
	ret = 0;
fail:
	return ret;
}

static int pipe_start(void *priv, u32 width, u32 height, u32 fps)
{
	struct pipe *p = priv;
	bool was_up;
	int ret;

	mutex_lock(&p->lock);
	if (p->ports & BIT(VPE_PORT_NV12)) {
		ret = -EBUSY;
		goto unlock;
	}
	was_up = p->up;
	if (!was_up) {
		ret = base_up(p, width, height, fps);
		if (ret)
			goto unlock;
	}
	ret = vpe_port_on(p, VPE_PORT_NV12, width, height, PIXFMT_YUV420SP, fps);
	if (ret)
		goto fail;
	STEP(ret, "SYS SetOutputDepth", MI_SYS_IMPL_SetChnOutputPortDepth(&p->vpe, 2, 4));
	ret = vif_bind(p, fps);
	if (ret)
		goto fail;
	goto unlock;
fail:
	if (p->ports & BIT(VPE_PORT_NV12))
		vpe_port_off(p, VPE_PORT_NV12);
	if (!was_up && !p->ports)
		base_down(p);
unlock:
	mutex_unlock(&p->lock);
	return ret;
}

static void pipe_stop(void *priv)
{
	struct pipe *p = priv;

	mutex_lock(&p->lock);
	if (p->ports & BIT(VPE_PORT_NV12)) {
		vpe_port_off(p, VPE_PORT_NV12);
		if (!p->ports)
			base_down(p);
	}
	mutex_unlock(&p->lock);
}

/*
 * MI_SYS_BufInfo (272 bytes) of a frame buffer, offsets from a live dump of
 * a 1920x1080 NV12 VPE buffer: type (1 = frame) at 0x10, width/height u16
 * at 0x38, plane addresses (u64, MIU offsets without the DRAM base) at
 * 0x48 (Y) and 0x50 (UV), strides u32 at 0x60 / 0x64, size at 0x6c.
 */
#define BI_TYPE			0x10
#define BI_W			0x38
#define BI_H			0x3a
#define BI_PHYS(p)		(0x48 + 8 * (p))
#define BI_STRIDE(p)		(0x60 + 4 * (p))
#define BI_SIZE			0x6c
#define BI_TYPE_FRAME		1
#define MIU0_BASE		0x20000000UL	/* DRAM base, MIU offset 0 */

static u32 bi32(const u8 *bi, u32 off)
{
	return *(const u32 *)(bi + off);
}

static int copy_plane(const u8 *bi, int plane, void *dst, u32 w, u32 rows)
{
	unsigned long pa = MIU0_BASE + bi32(bi, BI_PHYS(plane));
	u32 stride = bi32(bi, BI_STRIDE(plane)), r;
	const u8 *src;

	if (!bi32(bi, BI_PHYS(plane) + 4) && stride >= w) {
		src = __va(pa);
		/* the ISP/SCL wrote the frame by DMA: drop stale cache lines */
		dma_sync_single_for_cpu(osm_msys_device(), pa, (size_t)stride * rows,
					DMA_FROM_DEVICE);
		for (r = 0; r < rows; r++)
			memcpy(dst + (size_t)r * w, src + (size_t)r * stride, w);
		return 0;
	}
	return -EINVAL;
}

static void ctl_replay(struct pipe *p);

/* the ISP channel exists once frames flow: load the tuning bin, replay controls */
static void pipe_first_frame(struct pipe *p)
{
	if (p->ctl_ready)
		return;
	mutex_lock(&p->first_lock);
	if (!p->ctl_ready) {
		if (isp_bin[0]) {
			int r = osm_isp_load_bin(0, isp_bin);

			if (r)
				pr_warn("osm_pipe: ISP bin %s: %d\n", isp_bin, r);
		}
		ctl_replay(p);
	}
	mutex_unlock(&p->first_lock);
}

static int pipe_read_frame(void *priv, void *dst, u32 size, u64 *ts_ns)
{
	struct pipe *p = priv;
	u8 *info;
	int handle = -1, ret, tries;
	u32 w, h;

	info = kzalloc(272, GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	for (tries = 0; tries < 50; tries++) {
		MI(ret, mi_sys_ChnOutputPortGetBufPa(&p->vpe, info, &handle));
		if (!ret)
			break;
		if (ret != MI_ERR_NOBUF) {
			ret = mi_check("GetBuf", ret);
			goto out;
		}
		msleep(4);
	}
	if (ret) {
		ret = -EAGAIN;
		goto out;
	}

	if (p->frames < dump_frames)
		print_hex_dump(KERN_INFO, "osm_pipe buf: ", DUMP_PREFIX_OFFSET, 16, 4,
			       info, 272, false);
	w = *(u16 *)(info + BI_W);
	h = *(u16 *)(info + BI_H);
	if (bi32(info, BI_TYPE) != BI_TYPE_FRAME || w * h * 3 / 2 > size) {
		pr_warn_ratelimited("osm_pipe: unexpected buffer type %u %ux%u\n",
				    bi32(info, BI_TYPE), w, h);
		ret = -EIO;
	} else {
		ret = copy_plane(info, 0, dst, w, h);
		if (!ret)
			ret = copy_plane(info, 1, dst + w * h, w, h / 2);
	}
	{
		int r;

		MI(r, mi_sys_ChnOutputPortPutBuf(handle));
	}
	*ts_ns = 0;
	if (!ret) {
		p->frames++;
		pipe_first_frame(p);
	}
out:
	kfree(info);
	return ret;
}


/*
 * Image controls: the 3A of mhal (kernel AE) is driven over the ISP API
 * (MHalIspApiGet/Set, structs of libmi_isp's MI_ISP_AE_*, ids 0x14xx).
 * Layouts and enums decoded from the vendor code and checked on the stream:
 *   0x1404 ExpoMode  u32: 0 auto, 2 gain fixed (shutter auto), 3 shutter
 *                    fixed (gain auto), 4 manual (1 behaves like 0)
 *   0x1405 ManualExpo {fn_x10, sensor gain, isp gain, shutter us}; used by
 *                    the fixed modes, gains 1024 = 1.0x
 *   0x1409 ExposureLimit {min us, max us, min fn, max fn, min sensor gain,
 *                    min isp gain, max sensor gain, max isp gain}
 *   0x1403 EVComp    {ev, grid 1..20}: bias ev / grid in vendor units
 *   0x140e Flicker   u32: 0 off, 1 60 Hz, 2 50 Hz, 3 auto
 *   0x1402 ExpoInfo  (read only, 0x23c bytes): u32 stable, at-limit, then
 *                    {fn_x10, sensor gain, isp gain, us} of the long frame
 */
#define API_EXPO_INFO		0x1402
#define API_EV_COMP		0x1403
#define API_EXPO_MODE		0x1404
#define API_MANUAL_EXPO		0x1405
#define API_EXPO_LIMIT		0x1409
#define API_FLICKER		0x140e
#define EXPO_INFO_SIZE		0x23c
#define BIAS_GRID		2
#define BIAS_MEV_PER_EV		(1000 / BIAS_GRID)

static const u32 expo_mode_hw[] = {
	[OSM_EXPO_AUTO] = 0, [OSM_EXPO_GAIN] = 2,
	[OSM_EXPO_SHUTTER] = 3, [OSM_EXPO_MANUAL] = 4,
};
static const u32 flicker_hw[] = {
	[OSM_FLICKER_OFF] = 0, [OSM_FLICKER_50HZ] = 2,
	[OSM_FLICKER_60HZ] = 1, [OSM_FLICKER_AUTO] = 3,
};

static int api_get(u32 id, u32 size, void *buf)
{
	int r = osm_isp_api_get(0, id, size, buf);

	return r ? -EIO : 0;
}

static int api_set(u32 id, u32 size, void *buf)
{
	int r = osm_isp_api_set(0, id, size, buf);

	return r ? -EIO : 0;
}

static int manual_field(u32 field, u32 lo_idx, u32 hi_idx, u32 val)
{
	u32 lim[8], man[4];
	int ret = api_get(API_EXPO_LIMIT, sizeof(lim), lim);

	if (!ret)
		ret = api_get(API_MANUAL_EXPO, sizeof(man), man);
	if (ret)
		return ret;
	man[field] = clamp(val, lim[lo_idx], lim[hi_idx]);
	return api_set(API_MANUAL_EXPO, sizeof(man), man);
}

/* ctl_lock held, 3A up */
static int ctl_write(u32 id, s32 val)
{
	u32 w[2];

	switch (id) {
	case OSM_CTRL_EXPO_MODE:
		if (val < 0 || val >= ARRAY_SIZE(expo_mode_hw))
			return -EINVAL;
		w[0] = expo_mode_hw[val];
		return api_set(API_EXPO_MODE, 4, w);
	case OSM_CTRL_EXPO_US:
		return manual_field(3, 0, 1, max(val, 0));
	case OSM_CTRL_GAIN:
		return manual_field(1, 4, 6, max(val, 0));
	case OSM_CTRL_BIAS:
		w[0] = DIV_ROUND_CLOSEST(val, BIAS_MEV_PER_EV);
		w[1] = BIAS_GRID;
		return api_set(API_EV_COMP, 8, w);
	case OSM_CTRL_FLICKER:
		if (val < 0 || val >= ARRAY_SIZE(flicker_hw))
			return -EINVAL;
		w[0] = flicker_hw[val];
		return api_set(API_FLICKER, 4, w);
	}
	return -EINVAL;
}

/* the first frame is out and the bin is applied: apply what was set before */
static void ctl_replay(struct pipe *p)
{
	/* values first, the mode last */
	static const u8 order[] = { OSM_CTRL_EXPO_US, OSM_CTRL_GAIN, OSM_CTRL_BIAS,
				    OSM_CTRL_FLICKER, OSM_CTRL_EXPO_MODE };
	int i;

	mutex_lock(&p->ctl_lock);
	p->ctl_ready = true;
	for (i = 0; i < ARRAY_SIZE(order); i++) {
		if (!(p->ctl_mask & BIT(order[i])))
			continue;
		if (ctl_write(order[i], p->ctl_val[order[i]]))
			pr_warn("osm_pipe: control %u not applied\n", order[i]);
	}
	p->ctl_mask = 0;
	mutex_unlock(&p->ctl_lock);
}

static int pipe_set_ctrl(void *priv, enum osm_ctrl id, s32 val)
{
	struct pipe *p = priv;
	int ret;

	if (id >= OSM_CTRL_NUM)
		return -EINVAL;
	mutex_lock(&p->ctl_lock);
	if (p->ctl_ready) {
		ret = ctl_write(id, val);
	} else {
		p->ctl_val[id] = val;
		p->ctl_mask |= BIT(id);
		ret = 0;
	}
	mutex_unlock(&p->ctl_lock);
	return ret;
}

static int pipe_get_ctrl(void *priv, enum osm_ctrl id, s32 *val)
{
	struct pipe *p = priv;
	u32 w[EXPO_INFO_SIZE / 4];
	int ret, i;

	mutex_lock(&p->ctl_lock);
	if (!p->ctl_ready) {
		ret = -EAGAIN;
		goto out;
	}
	switch (id) {
	case OSM_CTRL_EXPO_MODE:
		ret = api_get(API_EXPO_MODE, 4, w);
		if (ret)
			break;
		for (i = 0; i < ARRAY_SIZE(expo_mode_hw); i++)
			if (expo_mode_hw[i] == w[0])
				break;
		/* 1 is an alias of 0 */
		*val = i < ARRAY_SIZE(expo_mode_hw) ? i : OSM_EXPO_AUTO;
		break;
	case OSM_CTRL_EXPO_US:
	case OSM_CTRL_GAIN:
		ret = api_get(API_EXPO_INFO, sizeof(w), w);
		if (!ret)
			*val = id == OSM_CTRL_GAIN ? w[3] : w[5];
		break;
	case OSM_CTRL_BIAS:
		ret = api_get(API_EV_COMP, 8, w);
		if (!ret)
			*val = (s32)w[0] * 1000 / (w[1] ? (s32)w[1] : 1);
		break;
	case OSM_CTRL_FLICKER:
		ret = api_get(API_FLICKER, 4, w);
		if (ret)
			break;
		for (i = 0; i < ARRAY_SIZE(flicker_hw); i++)
			if (flicker_hw[i] == w[0])
				break;
		*val = i < ARRAY_SIZE(flicker_hw) ? i : OSM_FLICKER_OFF;
		break;
	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&p->ctl_lock);
	return ret;
}

/*
 * Encoder: a VPE port feeds an MI_VENC channel of the same number (SYS
 * bind, frame based), the channel writes access units into a ring pool of
 * mi_sys. libmi_venc reads the pool through the ioctls; the kernel
 * callers use the same mi_sys ring pool functions: the handle comes from
 * MI_VENC_IMPL_GetRingPoolHandle(), the head record (flags u64 at 0,
 * timestamp u64 at 8, address u64 at 0x20, length u32 at 0x2c, all in a
 * 0x48 byte structure) is the next piece of data, and is released after
 * the copy.
 */
#define ENC_CHN_ATTR_SIZE	76
#define ENC_CODEC_H264		2
#define ENC_CODEC_MJPEG		4
#define ENC_RC_H264_CBR		1
#define ENC_RC_MJPEG_QP		7
#define ENC_PROFILE_HIGH	2
#define RING_REC_SIZE		0x48
#define RING_INFO_SIZE		0x48
#define MIU0_BASE_PHYS		0x20000000UL

static uint dump_recs;
module_param(dump_recs, uint, 0644);
MODULE_PARM_DESC(dump_recs, "log the ring pool record and first bytes of the first N encoder units");

static void enc_build_attr(const struct osm_enc_params *e, u32 *w)
{
	memset(w, 0, ENC_CHN_ATTR_SIZE);
	/* attribute union at word 1, rate control mode at word 10 */
	if (e->codec == OSM_CODEC_H264) {
		w[0] = ENC_CODEC_H264;
		w[1] = e->width;		/* max size */
		w[2] = e->height;
		w[3] = e->width * e->height;	/* stream buffer */
		w[4] = ENC_PROFILE_HIGH;
		w[5] = 1;			/* by frame */
		w[6] = e->width;
		w[7] = e->height;
		w[8] = 0;			/* B frames */
		w[9] = 1;			/* reference frames */
		w[10] = ENC_RC_H264_CBR;
		w[11] = e->gop;
		w[12] = 1;			/* stat time (s) */
		w[13] = e->fps;
		w[14] = 1;
		w[15] = e->bitrate;
		w[16] = 1;			/* average level */
	} else {
		w[0] = ENC_CODEC_MJPEG;
		w[1] = e->width;
		w[2] = e->height;
		w[3] = e->width * e->height;
		w[4] = 1;			/* by frame */
		w[5] = e->width;
		w[6] = e->height;
		w[10] = ENC_RC_MJPEG_QP;
		w[11] = e->fps;
		w[12] = 1;
		w[13] = e->quality;
	}
}

/* p->lock held */
static void enc_teardown(struct pipe *p)
{
	struct chn_port vpe_port = { MOD_VPE, 0, 0, p->enc_chn };
	struct chn_port venc = { MOD_VENC, p->enc_dev, p->enc_chn, 0 };

	DOWN("VENC StopRecvPic", MI_VENC_IMPL_StopRecvPic(p->enc_chn));
	DOWN("SYS UnBind (VENC)", MI_SYS_IMPL_UnBindChnPort(&vpe_port, &venc));
	DOWN("VENC DestroyChn", MI_VENC_IMPL_DestroyChn(p->enc_chn));
	vpe_port_off(p, p->enc_chn);
	p->enc_on = false;
}

static int enc_start(void *priv, const struct osm_enc_params *e)
{
	struct pipe *p = priv;
	struct chn_port vpe_port, venc;
	bool was_up;
	u32 attr[ENC_CHN_ATTR_SIZE / 4 + 4];
	u32 port = e->codec == OSM_CODEC_H264 ? VPE_PORT_H264 : VPE_PORT_MJPEG;
	int ret;

	mutex_lock(&p->lock);
	if (p->enc_on) {
		ret = -EBUSY;
		goto unlock;
	}
	was_up = p->up;
	if (!was_up) {
		ret = base_up(p, e->width, e->height, e->fps);
		if (ret)
			goto unlock;
	}
	ret = vpe_port_on(p, port, e->width, e->height,
			  e->codec == OSM_CODEC_H264 ? PIXFMT_YUV420SP : PIXFMT_YUV422_YUYV,
			  e->fps);
	if (ret)
		goto fail;
	p->enc_chn = port;
	p->enc = *e;
	enc_build_attr(e, attr);
	STEP(ret, "VENC CreateChn", MI_VENC_IMPL_CreateChn(p->enc_chn, attr));
	p->enc_on = true;	/* teardown on failure covers the channel */
	STEP(ret, "VENC GetChnDevid", MI_VENC_IMPL_GetChnDevid(p->enc_chn, &p->enc_dev));
	vpe_port = (struct chn_port){ MOD_VPE, 0, 0, p->enc_chn };
	venc = (struct chn_port){ MOD_VENC, p->enc_dev, p->enc_chn, 0 };
	STEP(ret, "SYS Bind (VENC)", MI_SYS_IMPL_BindChnPort2(&vpe_port, &venc, e->fps,
							       e->fps, LINK_FRAMEBASE, 0));
	STEP(ret, "VENC StartRecvPic", MI_VENC_IMPL_StartRecvPic(p->enc_chn));
	p->enc_ring = 0;	/* the pool exists once the first frame arrived */
	p->enc_idr_wait = true;
	ret = vif_bind(p, e->fps);
	if (ret)
		goto fail;
	goto unlock;
fail:
	if (p->enc_on)
		enc_teardown(p);
	else if (p->ports & BIT(port))
		vpe_port_off(p, port);
	if (!was_up && !p->ports)
		base_down(p);
unlock:
	mutex_unlock(&p->lock);
	return ret;
}

static void enc_stop(void *priv)
{
	struct pipe *p = priv;

	mutex_lock(&p->lock);
	if (p->enc_on) {
		enc_teardown(p);
		if (!p->ports)
			base_down(p);
	}
	mutex_unlock(&p->lock);
}

static u8 *ring_va(struct pipe *p, u64 addr)
{
	unsigned long pa = addr >= MIU0_BASE_PHYS ? addr : addr + MIU0_BASE_PHYS;

	return __va(pa);
}

/* Annex B access unit starts an IDR picture: SPS or IDR slice before any other slice */
static bool h264_is_idr(const u8 *d, u32 n)
{
	u32 i, end = min(n, 128u);

	for (i = 0; i + 3 < end; i++) {
		u8 nal;

		if (d[i] || d[i + 1] || d[i + 2] != 1)
			continue;
		nal = d[i + 3] & 0x1f;
		if (nal == 5 || nal == 7)
			return true;
		if (nal == 1)
			return false;
	}
	return false;
}

static int enc_read(void *priv, void *dst, u32 size, u32 *len, u32 *flags, u64 *ts_ns)
{
	struct pipe *p = priv;
	u8 *rec;
	u64 addr;
	u32 n, h;
	int ret;

	rec = kzalloc(RING_REC_SIZE, GFP_KERNEL);
	if (!rec)
		return -ENOMEM;
	if (!p->enc_ring) {
		/*
		 * The channel creates its ring pool with the first frame; asking
		 * earlier makes the vendor code log an error. Query: word 4 =
		 * packs ready.
		 */
		u32 st[16] = {};
		int tries;

		for (tries = 0; tries < 10; tries++) {
			MI(ret, MI_VENC_IMPL_Query(p->enc_chn, st));
			if (!ret && st[4])
				break;
			msleep(10);
		}
		if (ret || !st[4]) {
			ret = -EAGAIN;
			goto out;
		}
		MI(ret, MI_VENC_IMPL_GetRingPoolHandle(p->enc_chn, &p->enc_ring));
		if (ret || !p->enc_ring) {
			p->enc_ring = 0;
			ret = -EAGAIN;
			goto out;
		}
		MI(ret, mi_sys_RingPool_GetRingPoolTotalSpace(p->enc_ring, rec));
		if (!ret) {
			p->ring_base = *(u64 *)(rec + 0x20);
			p->ring_size = *(u32 *)(rec + 0x2c);
		}
		pr_info("osm_pipe: venc chn %u dev %u ring %u base 0x%llx size %u (%d)\n",
			p->enc_chn, p->enc_dev, p->enc_ring, p->ring_base, p->ring_size, ret);
		memset(rec, 0, RING_REC_SIZE);
	}
	h = p->enc_ring;
	MI(ret, mi_sys_RingPool_GetHeadRecordTimeout(h, rec, 1, 100));
	if (ret) {
		static int once;

		if (!once++)
			pr_info("osm_pipe: ring GetHeadRecord 0x%x\n", ret);
		ret = -EAGAIN;
		goto out;
	}
	addr = *(u64 *)(rec + 0x20);
	n = *(u32 *)(rec + 0x2c);
	if (dump_recs) {
		dump_recs--;
		print_hex_dump(KERN_INFO, "osm_pipe rec: ", DUMP_PREFIX_OFFSET, 16, 4, rec,
			       RING_REC_SIZE, false);
		pr_info("osm_pipe: unit addr 0x%llx len %u\n", addr, n);
		print_hex_dump(KERN_INFO, "osm_pipe data: ", DUMP_PREFIX_OFFSET, 16, 1,
			       ring_va(p, addr), min(n, 64u), true);
	}
	*ts_ns = 0;
	*flags = 0;
	if (n > size) {
		ret = -ENOSPC;
	} else {
		unsigned long pa = (unsigned long)ring_va(p, addr);

		dma_sync_single_for_cpu(osm_msys_device(), __pa(pa), n, DMA_FROM_DEVICE);
		memcpy(dst, (void *)pa, n);
		*len = n;
		if (p->enc.codec == OSM_CODEC_MJPEG || h264_is_idr(dst, n))
			*flags = OSM_AU_KEY;
		ret = 0;
	}
	{
		int r;

		MI(r, mi_sys_RingPool_ReleaseHeadRecord(h, 0));
	}
	if (!ret)
		pipe_first_frame(p);
out:
	kfree(rec);
	return ret;
}

static int enc_set(void *priv, enum osm_enc_ctrl id, s32 val)
{
	struct pipe *p = priv;
	int ret = 0;

	mutex_lock(&p->lock);
	if (!p->enc_on) {
		ret = -ENODEV;
		goto out;
	}
	if (id == OSM_ENC_IDR) {
		MI(ret, MI_VENC_IMPL_RequestIdr(p->enc_chn, 1));
		ret = mi_check("VENC RequestIdr", ret);
		goto out;
	}
	if (id == OSM_ENC_BITRATE)
		p->enc.bitrate = val;
	else if (id == OSM_ENC_GOP)
		p->enc.gop = val;
	else if (id == OSM_ENC_QUALITY)
		p->enc.quality = val;
	else {
		ret = -EINVAL;
		goto out;
	}
	{
		u32 attr[ENC_CHN_ATTR_SIZE / 4 + 4];

		enc_build_attr(&p->enc, attr);
		MI(ret, MI_VENC_IMPL_SetChnAttr(p->enc_chn, attr));
		ret = mi_check("VENC SetChnAttr", ret);
	}
out:
	mutex_unlock(&p->lock);
	return ret;
}

static const struct osm_v4l2_backend pipe_be = {
	.name = "osm_pipe (vendor MI)",
	.priv = &pipe_state,
	.start = pipe_start,
	.stop = pipe_stop,
	.read_frame = pipe_read_frame,
	.set_ctrl = pipe_set_ctrl,
	.get_ctrl = pipe_get_ctrl,
	.enc_start = enc_start,
	.enc_stop = enc_stop,
	.enc_read = enc_read,
	.enc_set = enc_set,
};

static struct osm_v4l2_backend be;

static int __init osm_pipe_init(void)
{
	mutex_init(&pipe_state.lock);
	mutex_init(&pipe_state.ctl_lock);
	mutex_init(&pipe_state.first_lock);
	be = pipe_be;
	be.owner = THIS_MODULE;
	return osm_v4l2_register(&be);
}

static void __exit osm_pipe_exit(void)
{
	osm_v4l2_unregister(&be);
	pipe_deinit(&pipe_state);
}

module_init(osm_pipe_init);
module_exit(osm_pipe_exit);
MODULE_DESCRIPTION("osm_v4l2 backend over the SigmaStar MI modules");
MODULE_LICENSE("GPL");
