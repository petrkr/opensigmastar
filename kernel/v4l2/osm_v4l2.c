// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 capture device for the SSC338Q camera pipeline.
 *
 * This module is only the V4L2/videobuf2 front: one video node,
 * NV12 1920x1080, vb2 vmalloc buffers filled by a kernel thread from the
 * backend (osm_pipe: sensor -> VIF -> VPE ISP + SCL over the vendor MI
 * modules). A second node offers the hardware encoder like a UVC camera:
 * one access unit per buffer, H.264 (Annex B) or MJPEG selected with
 * VIDIOC_S_FMT, both at 1920x1080@20. The front uses GPL-only symbols and therefore must not link to
 * any module that uses the proprietary vendor modules; the backend
 * registers itself through osm_v4l2_register() (see osm_v4l2.h).
 */
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include "osm_v4l2.h"

#define OSM_WIDTH	1920
#define OSM_HEIGHT	1080
#define OSM_FPS		20
#define OSM_SIZE	(OSM_WIDTH * OSM_HEIGHT * 3 / 2)
#define OSM_ENC_SIZE	(1024 * 1024)	/* largest access unit */

static int video_nr = -1;
module_param(video_nr, int, 0444);
MODULE_PARM_DESC(video_nr, "video device number (-1 = first free)");

struct osm_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct osm_dev;

/* one video node: NV12 (enc false) or compressed (enc true) */
struct osm_node {
	struct osm_dev *d;
	bool enc;
	struct video_device vdev;
	struct vb2_queue q;
	struct mutex lock;		/* ioctls, queue */
	spinlock_t qlock;		/* bufs */
	struct list_head bufs;
	wait_queue_head_t wq;
	struct task_struct *thread;
	u32 sequence;
};

struct osm_dev {
	struct platform_device *pdev;
	struct v4l2_device v4l2;
	struct osm_node node[2];	/* 0: NV12, 1: compressed */
	struct v4l2_ctrl_handler hdl;	/* image controls (node 0) */
	struct v4l2_ctrl *c_auto, *c_exp, *c_gain, *c_bias, *c_plf;
	s32 exp_set, gain_set;	/* values for the fixed modes, 100 us / 1024 = 1x */
	struct v4l2_ctrl_handler ehdl;	/* encoder controls (node 1) */
	struct v4l2_ctrl *c_bitrate, *c_gop, *c_quality, *c_idr;
	u32 codec_fmt;			/* V4L2_PIX_FMT_H264 or _MJPEG */
	struct mutex be_lock;		/* backend (un)registration, start/stop */
	const struct osm_v4l2_backend *be;
	bool registered;
};

static struct osm_dev *odev;

/* the backend pipeline runs (either node streams); be_lock held */
static bool osm_streaming(struct osm_dev *d)
{
	return d->node[0].thread || d->node[1].thread;
}

static struct osm_buf *osm_take_buf(struct osm_node *n)
{
	struct osm_buf *b;

	wait_event_interruptible(n->wq, kthread_should_stop() || !list_empty(&n->bufs));
	if (kthread_should_stop())
		return NULL;
	spin_lock_irq(&n->qlock);
	b = list_first_entry_or_null(&n->bufs, struct osm_buf, list);
	if (b)
		list_del(&b->list);
	spin_unlock_irq(&n->qlock);
	return b;
}

static void osm_put_back(struct osm_node *n, struct osm_buf *b, int ret)
{
	/* nothing yet: keep the buffer for the next try */
	spin_lock_irq(&n->qlock);
	list_add(&b->list, &n->bufs);
	spin_unlock_irq(&n->qlock);
	if (ret != -EAGAIN) {
		pr_warn_ratelimited("osm_v4l2: read: %d\n", ret);
		msleep(20);
	}
}

static int osm_thread(void *data)
{
	struct osm_node *n = data;
	struct osm_dev *d = n->d;
	const struct osm_v4l2_backend *be = d->be;

	while (!kthread_should_stop()) {
		struct osm_buf *b = osm_take_buf(n);
		u64 ts = 0;
		int ret;

		if (!b)
			continue;
		ret = be->read_frame(be->priv, vb2_plane_vaddr(&b->vb.vb2_buf, 0),
				     OSM_SIZE, &ts);
		if (ret) {
			osm_put_back(n, b, ret);
			continue;
		}
		vb2_set_plane_payload(&b->vb.vb2_buf, 0, OSM_SIZE);
		b->vb.vb2_buf.timestamp = ts ? ts : ktime_get_ns();
		b->vb.sequence = n->sequence++;
		b->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
	return 0;
}

/* encoder node: one access unit per buffer */
static int osm_enc_thread(void *data)
{
	struct osm_node *n = data;
	struct osm_dev *d = n->d;
	const struct osm_v4l2_backend *be = d->be;

	while (!kthread_should_stop()) {
		struct osm_buf *b = osm_take_buf(n);
		u32 len = 0, flags = 0;
		u64 ts = 0;
		int ret;

		if (!b)
			continue;
		ret = be->enc_read(be->priv, vb2_plane_vaddr(&b->vb.vb2_buf, 0),
				   vb2_plane_size(&b->vb.vb2_buf, 0), &len, &flags, &ts);
		if (ret == -ENOSPC) {
			/* the unit was dropped by the backend: report nothing, retry */
			pr_warn_ratelimited("osm_v4l2: access unit does not fit the buffer\n");
			osm_put_back(n, b, -EAGAIN);
			continue;
		}
		if (ret) {
			osm_put_back(n, b, ret);
			continue;
		}
		vb2_set_plane_payload(&b->vb.vb2_buf, 0, len);
		b->vb.vb2_buf.timestamp = ts ? ts : ktime_get_ns();
		b->vb.sequence = n->sequence++;
		b->vb.field = V4L2_FIELD_NONE;
		b->vb.flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME);
		b->vb.flags |= flags & OSM_AU_KEY ? V4L2_BUF_FLAG_KEYFRAME : V4L2_BUF_FLAG_PFRAME;
		vb2_buffer_done(&b->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
	return 0;
}

static u32 osm_size(struct osm_node *n)
{
	return n->enc ? OSM_ENC_SIZE : OSM_SIZE;
}

static int osm_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct osm_node *n = vb2_get_drv_priv(q);

	if (*nplanes)
		return sizes[0] < osm_size(n) ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = osm_size(n);
	return 0;
}

static int osm_buf_prepare(struct vb2_buffer *vb)
{
	struct osm_node *n = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < osm_size(n))
		return -EINVAL;
	/* the encoder sets the payload itself */
	if (!n->enc)
		vb2_set_plane_payload(vb, 0, OSM_SIZE);
	return 0;
}

static void osm_buf_queue(struct vb2_buffer *vb)
{
	struct osm_node *n = vb2_get_drv_priv(vb->vb2_queue);
	struct osm_buf *b = container_of(to_vb2_v4l2_buffer(vb), struct osm_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&n->qlock, flags);
	list_add_tail(&b->list, &n->bufs);
	spin_unlock_irqrestore(&n->qlock, flags);
	wake_up(&n->wq);
}

static void osm_return_bufs(struct osm_node *n, enum vb2_buffer_state st)
{
	struct osm_buf *b, *t;
	unsigned long flags;

	spin_lock_irqsave(&n->qlock, flags);
	list_for_each_entry_safe(b, t, &n->bufs, list) {
		list_del(&b->list);
		vb2_buffer_done(&b->vb.vb2_buf, st);
	}
	spin_unlock_irqrestore(&n->qlock, flags);
}


/*
 * Image controls. Exposure: V4L2 exposure time is in 100 us, the gain is
 * the sensor gain with 1024 = 1.0x (V4L2 leaves the unit open). The
 * auto cluster (mode, time, gain) is volatile: time and gain read back what
 * the AE chose. Bias is an integer menu in nominal milli-EV (the vendor
 * scale is not calibrated in stops, see osm_pipe.c).
 */
#define OSM_EXP_UNIT_US		100
#define OSM_GAIN_MIN		1024
#define OSM_GAIN_MAX		(128 * 1024)
#define OSM_EXP_MIN		1
#define OSM_EXP_MAX		1000

static const s64 osm_bias_menu[] = {
	-3000, -2500, -2000, -1500, -1000, -500, 0,
	500, 1000, 1500, 2000, 2500, 3000,
};
#define OSM_BIAS_DEF		6

static const u8 expo_to_osm[] = {
	[V4L2_EXPOSURE_AUTO] = OSM_EXPO_AUTO,
	[V4L2_EXPOSURE_MANUAL] = OSM_EXPO_MANUAL,
	[V4L2_EXPOSURE_SHUTTER_PRIORITY] = OSM_EXPO_SHUTTER,
	[V4L2_EXPOSURE_APERTURE_PRIORITY] = OSM_EXPO_GAIN,
};

static const u8 plf_to_osm[] = {
	[V4L2_CID_POWER_LINE_FREQUENCY_DISABLED] = OSM_FLICKER_OFF,
	[V4L2_CID_POWER_LINE_FREQUENCY_50HZ] = OSM_FLICKER_50HZ,
	[V4L2_CID_POWER_LINE_FREQUENCY_60HZ] = OSM_FLICKER_60HZ,
	[V4L2_CID_POWER_LINE_FREQUENCY_AUTO] = OSM_FLICKER_AUTO,
};

/*
 * Apply the exposure cluster. The values are only used by the fixed modes
 * (manual: both, shutter priority: the time, aperture priority: the
 * gain), the AE keeps the rest. Switching to manual freezes what the AE
 * chose unless the same request carries a value; the mode goes last.
 */
static int osm_set_expo(struct osm_dev *d, s32 mode, bool mode_new,
			bool exp_new, bool gain_new)
{
	const struct osm_v4l2_backend *be = d->be;
	bool use_exp = mode == V4L2_EXPOSURE_MANUAL || mode == V4L2_EXPOSURE_SHUTTER_PRIORITY;
	bool use_gain = mode == V4L2_EXPOSURE_MANUAL || mode == V4L2_EXPOSURE_APERTURE_PRIORITY;
	s32 v;
	int ret = 0;

	if (exp_new)
		d->exp_set = d->c_exp->val;
	if (gain_new)
		d->gain_set = d->c_gain->val;
	if (mode == V4L2_EXPOSURE_MANUAL && mode_new) {
		if (!exp_new && !be->get_ctrl(be->priv, OSM_CTRL_EXPO_US, &v))
			d->exp_set = clamp(DIV_ROUND_CLOSEST(v, OSM_EXP_UNIT_US),
					   OSM_EXP_MIN, OSM_EXP_MAX);
		if (!gain_new && !be->get_ctrl(be->priv, OSM_CTRL_GAIN, &v))
			d->gain_set = clamp(v, OSM_GAIN_MIN, OSM_GAIN_MAX);
	}
	if (use_exp && (exp_new || mode_new))
		ret = be->set_ctrl(be->priv, OSM_CTRL_EXPO_US, d->exp_set * OSM_EXP_UNIT_US);
	if (!ret && use_gain && (gain_new || mode_new))
		ret = be->set_ctrl(be->priv, OSM_CTRL_GAIN, d->gain_set);
	if (!ret && mode_new)
		ret = be->set_ctrl(be->priv, OSM_CTRL_EXPO_MODE, expo_to_osm[mode]);
	return ret;
}

static int osm_s_ctrl(struct v4l2_ctrl *c)
{
	struct osm_dev *d = container_of(c->handler, struct osm_dev, hdl);
	const struct osm_v4l2_backend *be = d->be;
	int ret = -EINVAL;

	/* the handler has its own lock; be_lock orders this against start/stop */
	mutex_lock(&d->be_lock);
	if (!osm_streaming(d)) {
		/* not streaming: osm_replay_ctrls() applies the values at the start */
		if (c->id == V4L2_CID_EXPOSURE_AUTO) {
			if (d->c_exp->is_new)
				d->exp_set = d->c_exp->val;
			if (d->c_gain->is_new)
				d->gain_set = d->c_gain->val;
		}
		mutex_unlock(&d->be_lock);
		return 0;
	}
	switch (c->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		ret = osm_set_expo(d, d->c_auto->val, d->c_auto->is_new,
				   d->c_exp->is_new, d->c_gain->is_new);
		break;
	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		ret = be->set_ctrl(be->priv, OSM_CTRL_BIAS, osm_bias_menu[c->val]);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = be->set_ctrl(be->priv, OSM_CTRL_FLICKER, plf_to_osm[c->val]);
		break;
	}
	mutex_unlock(&d->be_lock);
	return ret;
}

static int osm_g_volatile_ctrl(struct v4l2_ctrl *c)
{
	struct osm_dev *d = container_of(c->handler, struct osm_dev, hdl);
	const struct osm_v4l2_backend *be = d->be;
	s32 us, gain;
	int ret;

	mutex_lock(&d->be_lock);
	if (!osm_streaming(d)) {
		mutex_unlock(&d->be_lock);
		return 0;	/* keep the stored values */
	}
	ret = be->get_ctrl(be->priv, OSM_CTRL_EXPO_US, &us);
	if (!ret)
		ret = be->get_ctrl(be->priv, OSM_CTRL_GAIN, &gain);
	mutex_unlock(&d->be_lock);
	if (ret)
		return 0;
	d->c_exp->val = clamp(DIV_ROUND_CLOSEST(us, OSM_EXP_UNIT_US),
			      OSM_EXP_MIN, OSM_EXP_MAX);
	d->c_gain->val = clamp(gain, OSM_GAIN_MIN, OSM_GAIN_MAX);
	return 0;
}

static const struct v4l2_ctrl_ops osm_ctrl_ops = {
	.s_ctrl = osm_s_ctrl,
	.g_volatile_ctrl = osm_g_volatile_ctrl,
};

/* streaming just started: send what differs from the defaults */
static void osm_replay_ctrls(struct osm_dev *d)
{
	const struct osm_v4l2_backend *be = d->be;

	if (d->c_auto->cur.val != V4L2_EXPOSURE_AUTO)
		osm_set_expo(d, d->c_auto->cur.val, true, false, false);
	if (d->c_bias->cur.val != OSM_BIAS_DEF)
		be->set_ctrl(be->priv, OSM_CTRL_BIAS, osm_bias_menu[d->c_bias->cur.val]);
	if (d->c_plf->cur.val != V4L2_CID_POWER_LINE_FREQUENCY_50HZ)
		be->set_ctrl(be->priv, OSM_CTRL_FLICKER, plf_to_osm[d->c_plf->cur.val]);
}

static int osm_init_ctrls(struct osm_dev *d)
{
	struct v4l2_ctrl_handler *h = &d->hdl;

	v4l2_ctrl_handler_init(h, 5);
	d->c_auto = v4l2_ctrl_new_std_menu(h, &osm_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
					   V4L2_EXPOSURE_APERTURE_PRIORITY, 0,
					   V4L2_EXPOSURE_AUTO);
	d->c_exp = v4l2_ctrl_new_std(h, &osm_ctrl_ops, V4L2_CID_EXPOSURE_ABSOLUTE,
				     OSM_EXP_MIN, OSM_EXP_MAX, 1, 100);
	d->c_gain = v4l2_ctrl_new_std(h, &osm_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
				      OSM_GAIN_MIN, OSM_GAIN_MAX, 1, OSM_GAIN_MIN);
	d->c_bias = v4l2_ctrl_new_int_menu(h, &osm_ctrl_ops, V4L2_CID_AUTO_EXPOSURE_BIAS,
					   ARRAY_SIZE(osm_bias_menu) - 1, OSM_BIAS_DEF,
					   osm_bias_menu);
	d->c_plf = v4l2_ctrl_new_std_menu(h, &osm_ctrl_ops, V4L2_CID_POWER_LINE_FREQUENCY,
					  V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
					  V4L2_CID_POWER_LINE_FREQUENCY_50HZ);
	if (h->error)
		return h->error;
	/* volatile controls only count as changed with execute-on-write */
	d->c_exp->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	d->c_gain->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	/*
	 * One cluster, but not an auto cluster: the core would refuse the
	 * values outside manual mode, and the priority modes need them.
	 */
	v4l2_ctrl_cluster(3, &d->c_auto);
	d->exp_set = d->c_exp->default_value;
	d->gain_set = d->c_gain->default_value;
	return 0;
}

static int osm_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct osm_node *n = vb2_get_drv_priv(q);
	struct osm_dev *d = n->d;
	int ret;

	mutex_lock(&d->be_lock);
	if (!d->be || !try_module_get(d->be->owner)) {
		ret = -ENODEV;
		goto err;
	}
	if (n->enc) {
		struct osm_enc_params par = {
			.codec = d->codec_fmt == V4L2_PIX_FMT_MJPEG ? OSM_CODEC_MJPEG : OSM_CODEC_H264,
			.width = OSM_WIDTH, .height = OSM_HEIGHT, .fps = OSM_FPS,
			.bitrate = d->c_bitrate->val, .gop = d->c_gop->val,
			.quality = d->c_quality->val,
		};

		ret = d->be->enc_start(d->be->priv, &par);
	} else {
		ret = d->be->start(d->be->priv, OSM_WIDTH, OSM_HEIGHT, OSM_FPS);
	}
	if (ret)
		goto put;
	osm_replay_ctrls(d);
	n->sequence = 0;
	n->thread = kthread_run(n->enc ? osm_enc_thread : osm_thread, n,
				n->enc ? "osm_v4l2_enc" : "osm_v4l2");
	if (IS_ERR(n->thread)) {
		ret = PTR_ERR(n->thread);
		n->thread = NULL;
		if (n->enc)
			d->be->enc_stop(d->be->priv);
		else
			d->be->stop(d->be->priv);
		goto put;
	}
	mutex_unlock(&d->be_lock);
	return 0;
put:
	module_put(d->be->owner);
err:
	mutex_unlock(&d->be_lock);
	osm_return_bufs(n, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void osm_stop_streaming(struct vb2_queue *q)
{
	struct osm_node *n = vb2_get_drv_priv(q);
	struct osm_dev *d = n->d;

	mutex_lock(&d->be_lock);
	if (n->thread) {
		kthread_stop(n->thread);
		n->thread = NULL;
		if (n->enc)
			d->be->enc_stop(d->be->priv);
		else
			d->be->stop(d->be->priv);
		module_put(d->be->owner);
	}
	mutex_unlock(&d->be_lock);
	osm_return_bufs(n, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops osm_vb2_ops = {
	.queue_setup = osm_queue_setup,
	.buf_prepare = osm_buf_prepare,
	.buf_queue = osm_buf_queue,
	.start_streaming = osm_start_streaming,
	.stop_streaming = osm_stop_streaming,
};

static void osm_fill_fmt(struct v4l2_pix_format *p)
{
	p->width = OSM_WIDTH;
	p->height = OSM_HEIGHT;
	p->pixelformat = V4L2_PIX_FMT_NV12;
	p->field = V4L2_FIELD_NONE;
	p->bytesperline = OSM_WIDTH;
	p->sizeimage = OSM_SIZE;
	p->colorspace = V4L2_COLORSPACE_REC709;
}

static int osm_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "osm_v4l2", sizeof(cap->driver));
	strscpy(cap->card, "SSC338Q camera", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:osm_v4l2", sizeof(cap->bus_info));
	return 0;
}

static int osm_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int osm_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	osm_fill_fmt(&f->fmt.pix);
	return 0;
}

/* encoder node: the format selects the codec */
static int osm_enc_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	static const u32 fmts[] = { V4L2_PIX_FMT_H264, V4L2_PIX_FMT_MJPEG };

	if (f->index >= ARRAY_SIZE(fmts))
		return -EINVAL;
	f->pixelformat = fmts[f->index];
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static void osm_enc_fill_fmt(struct v4l2_pix_format *p, u32 fmt)
{
	p->width = OSM_WIDTH;
	p->height = OSM_HEIGHT;
	p->pixelformat = fmt;
	p->field = V4L2_FIELD_NONE;
	p->bytesperline = 0;
	p->sizeimage = OSM_ENC_SIZE;
	p->colorspace = V4L2_COLORSPACE_REC709;
}

static int osm_enc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct osm_node *n = video_drvdata(file);

	osm_enc_fill_fmt(&f->fmt.pix, n->d->codec_fmt);
	return 0;
}

static int osm_enc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	u32 fmt = f->fmt.pix.pixelformat;

	osm_enc_fill_fmt(&f->fmt.pix, fmt == V4L2_PIX_FMT_MJPEG ? fmt : V4L2_PIX_FMT_H264);
	return 0;
}

static int osm_enc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct osm_node *n = video_drvdata(file);

	osm_enc_try_fmt(file, priv, f);
	if (f->fmt.pix.pixelformat != n->d->codec_fmt && vb2_is_busy(&n->q))
		return -EBUSY;
	n->d->codec_fmt = f->fmt.pix.pixelformat;
	return 0;
}

static int osm_enum_input(struct file *file, void *priv, struct v4l2_input *i)
{
	if (i->index)
		return -EINVAL;
	i->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(i->name, "IMX415", sizeof(i->name));
	return 0;
}

static int osm_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int osm_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static int osm_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fs)
{
	struct osm_node *n = video_drvdata(file);
	bool ok = n->enc ? (fs->pixel_format == V4L2_PIX_FMT_H264 ||
			    fs->pixel_format == V4L2_PIX_FMT_MJPEG)
			 : fs->pixel_format == V4L2_PIX_FMT_NV12;

	if (fs->index || !ok)
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fs->discrete.width = OSM_WIDTH;
	fs->discrete.height = OSM_HEIGHT;
	return 0;
}

static int osm_enum_frameintervals(struct file *file, void *priv,
				   struct v4l2_frmivalenum *fi)
{
	struct osm_node *n = video_drvdata(file);
	bool ok = n->enc ? (fi->pixel_format == V4L2_PIX_FMT_H264 ||
			    fi->pixel_format == V4L2_PIX_FMT_MJPEG)
			 : fi->pixel_format == V4L2_PIX_FMT_NV12;

	if (fi->index || !ok || fi->width != OSM_WIDTH || fi->height != OSM_HEIGHT)
		return -EINVAL;
	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	fi->discrete.denominator = OSM_FPS;
	return 0;
}

static int osm_g_parm(struct file *file, void *priv, struct v4l2_streamparm *p)
{
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	p->parm.capture.capability = 0;
	p->parm.capture.timeperframe.numerator = 1;
	p->parm.capture.timeperframe.denominator = OSM_FPS;
	p->parm.capture.readbuffers = 2;
	return 0;
}

#define OSM_COMMON_IOCTLS						\
	.vidioc_querycap = osm_querycap,				\
	.vidioc_enum_input = osm_enum_input,				\
	.vidioc_g_input = osm_g_input,					\
	.vidioc_s_input = osm_s_input,					\
	.vidioc_enum_framesizes = osm_enum_framesizes,			\
	.vidioc_enum_frameintervals = osm_enum_frameintervals,		\
	.vidioc_g_parm = osm_g_parm,					\
	.vidioc_s_parm = osm_g_parm,					\
	.vidioc_reqbufs = vb2_ioctl_reqbufs,				\
	.vidioc_create_bufs = vb2_ioctl_create_bufs,			\
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,			\
	.vidioc_querybuf = vb2_ioctl_querybuf,				\
	.vidioc_qbuf = vb2_ioctl_qbuf,					\
	.vidioc_dqbuf = vb2_ioctl_dqbuf,				\
	.vidioc_expbuf = vb2_ioctl_expbuf,				\
	.vidioc_streamon = vb2_ioctl_streamon,				\
	.vidioc_streamoff = vb2_ioctl_streamoff

static const struct v4l2_ioctl_ops osm_ioctl_ops = {
	OSM_COMMON_IOCTLS,
	.vidioc_enum_fmt_vid_cap = osm_enum_fmt,
	.vidioc_g_fmt_vid_cap = osm_g_fmt,
	.vidioc_s_fmt_vid_cap = osm_g_fmt,
	.vidioc_try_fmt_vid_cap = osm_g_fmt,
};

static const struct v4l2_ioctl_ops osm_enc_ioctl_ops = {
	OSM_COMMON_IOCTLS,
	.vidioc_enum_fmt_vid_cap = osm_enc_enum_fmt,
	.vidioc_g_fmt_vid_cap = osm_enc_g_fmt,
	.vidioc_s_fmt_vid_cap = osm_enc_s_fmt,
	.vidioc_try_fmt_vid_cap = osm_enc_try_fmt,
};

static const struct v4l2_file_operations osm_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

/*
 * Encoder controls. Set while streaming they go to the backend at once,
 * otherwise the values are read when the stream starts.
 */
#define OSM_BITRATE_MIN		100000
#define OSM_BITRATE_MAX		16000000
#define OSM_BITRATE_DEF		4000000
#define OSM_GOP_MAX		250
#define OSM_GOP_DEF		40

static int osm_enc_s_ctrl(struct v4l2_ctrl *c)
{
	struct osm_dev *d = container_of(c->handler, struct osm_dev, ehdl);
	const struct osm_v4l2_backend *be = d->be;
	int ret = 0;

	mutex_lock(&d->be_lock);
	if (!d->node[1].thread) {
		mutex_unlock(&d->be_lock);
		return 0;
	}
	switch (c->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ret = be->enc_set(be->priv, OSM_ENC_BITRATE, c->val);
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ret = be->enc_set(be->priv, OSM_ENC_GOP, c->val);
		break;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ret = be->enc_set(be->priv, OSM_ENC_QUALITY, c->val);
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ret = be->enc_set(be->priv, OSM_ENC_IDR, 0);
		break;
	}
	mutex_unlock(&d->be_lock);
	return ret;
}

static const struct v4l2_ctrl_ops osm_enc_ctrl_ops = {
	.s_ctrl = osm_enc_s_ctrl,
};

static int osm_init_enc_ctrls(struct osm_dev *d)
{
	struct v4l2_ctrl_handler *h = &d->ehdl;

	v4l2_ctrl_handler_init(h, 4);
	d->c_bitrate = v4l2_ctrl_new_std(h, &osm_enc_ctrl_ops, V4L2_CID_MPEG_VIDEO_BITRATE,
					 OSM_BITRATE_MIN, OSM_BITRATE_MAX, 1000, OSM_BITRATE_DEF);
	d->c_gop = v4l2_ctrl_new_std(h, &osm_enc_ctrl_ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
				     1, OSM_GOP_MAX, 1, OSM_GOP_DEF);
	d->c_quality = v4l2_ctrl_new_std(h, &osm_enc_ctrl_ops, V4L2_CID_JPEG_COMPRESSION_QUALITY,
					 1, 100, 1, 80);
	d->c_idr = v4l2_ctrl_new_std(h, &osm_enc_ctrl_ops, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
				     0, 0, 0, 0);
	return h->error;
}

int osm_v4l2_register(const struct osm_v4l2_backend *be)
{
	struct osm_dev *d = odev;
	int ret = 0;

	if (!d)
		return -ENODEV;
	mutex_lock(&d->be_lock);
	if (d->be)
		ret = -EBUSY;
	else
		d->be = be;
	mutex_unlock(&d->be_lock);
	if (!ret)
		pr_info("osm_v4l2: backend %s\n", be->name);
	return ret;
}
EXPORT_SYMBOL(osm_v4l2_register);

void osm_v4l2_unregister(const struct osm_v4l2_backend *be)
{
	struct osm_dev *d = odev;

	if (!d)
		return;
	/* streaming holds a module reference on the backend: not running here */
	mutex_lock(&d->be_lock);
	if (d->be == be)
		d->be = NULL;
	mutex_unlock(&d->be_lock);
}
EXPORT_SYMBOL(osm_v4l2_unregister);

static int osm_init_node(struct osm_dev *d, int idx, int nr)
{
	struct osm_node *n = &d->node[idx];
	struct vb2_queue *q = &n->q;
	int ret;

	n->d = d;
	n->enc = idx == 1;
	mutex_init(&n->lock);
	spin_lock_init(&n->qlock);
	INIT_LIST_HEAD(&n->bufs);
	init_waitqueue_head(&n->wq);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_READ | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = n;
	q->buf_struct_size = sizeof(struct osm_buf);
	q->ops = &osm_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &n->lock;
	q->dev = &d->pdev->dev;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	strscpy(n->vdev.name, n->enc ? "osm_v4l2 compressed" : "osm_v4l2", sizeof(n->vdev.name));
	n->vdev.fops = &osm_fops;
	n->vdev.ioctl_ops = n->enc ? &osm_enc_ioctl_ops : &osm_ioctl_ops;
	n->vdev.release = video_device_release_empty;
	n->vdev.v4l2_dev = &d->v4l2;
	n->vdev.ctrl_handler = n->enc ? &d->ehdl : &d->hdl;
	n->vdev.queue = q;
	n->vdev.lock = &n->lock;
	n->vdev.vfl_dir = VFL_DIR_RX;
	n->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	video_set_drvdata(&n->vdev, n);
	ret = video_register_device(&n->vdev, VFL_TYPE_VIDEO, nr);
	if (ret)
		return ret;
	pr_info("osm_v4l2: /dev/video%d (%s)\n", n->vdev.num,
		n->enc ? "H.264/MJPEG" : "NV12");
	return 0;
}

static int __init osm_v4l2_init(void)
{
	struct osm_dev *d;
	int ret;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	mutex_init(&d->be_lock);
	d->codec_fmt = V4L2_PIX_FMT_H264;

	d->pdev = platform_device_register_simple("osm_v4l2", -1, NULL, 0);
	if (IS_ERR(d->pdev)) {
		ret = PTR_ERR(d->pdev);
		goto err_free;
	}
	strscpy(d->v4l2.name, "osm_v4l2", sizeof(d->v4l2.name));
	ret = v4l2_device_register(&d->pdev->dev, &d->v4l2);
	if (ret)
		goto err_pdev;

	ret = osm_init_ctrls(d);
	if (ret)
		goto err_ctrls;
	ret = osm_init_enc_ctrls(d);
	if (ret)
		goto err_ctrls;

	ret = osm_init_node(d, 0, video_nr);
	if (ret)
		goto err_ctrls;
	d->registered = true;
	ret = osm_init_node(d, 1, video_nr < 0 ? -1 : video_nr + 1);
	if (ret)
		goto err_node0;
	odev = d;
	return 0;
err_node0:
	video_unregister_device(&d->node[0].vdev);
err_ctrls:
	v4l2_ctrl_handler_free(&d->hdl);
	v4l2_ctrl_handler_free(&d->ehdl);
	v4l2_device_unregister(&d->v4l2);
err_pdev:
	platform_device_unregister(d->pdev);
err_free:
	kfree(d);
	return ret;
}

static void __exit osm_v4l2_exit(void)
{
	struct osm_dev *d = odev;

	odev = NULL;
	video_unregister_device(&d->node[1].vdev);
	video_unregister_device(&d->node[0].vdev);
	v4l2_ctrl_handler_free(&d->hdl);
	v4l2_ctrl_handler_free(&d->ehdl);
	v4l2_device_unregister(&d->v4l2);
	platform_device_unregister(d->pdev);
	kfree(d);
}

module_init(osm_v4l2_init);
module_exit(osm_v4l2_exit);
MODULE_DESCRIPTION("V4L2 capture device front for the SSC338Q camera pipeline");
MODULE_LICENSE("GPL");
