/* SPDX-License-Identifier: GPL-2.0 */
#ifndef OSM_V4L2_H
#define OSM_V4L2_H

#include <linux/types.h>

/*
 * Image controls of the backend (3A of the kernel ISP). Units are the
 * backend's own, not V4L2's: exposure in us, gain as a fixed-point
 * multiplier with 1024 = 1.0x, bias in nominal milli-EV (see osm_pipe.c),
 * flicker as a mode. Values are accepted only while the pipeline runs.
 */
enum osm_ctrl {
	OSM_CTRL_EXPO_MODE,	/* enum osm_expo_mode */
	OSM_CTRL_EXPO_US,	/* shutter time used by the fixed-shutter modes */
	OSM_CTRL_GAIN,		/* sensor gain used by the fixed-gain modes */
	OSM_CTRL_BIAS,		/* auto exposure bias, milli-EV */
	OSM_CTRL_FLICKER,	/* enum osm_flicker */
	OSM_CTRL_NUM
};

enum osm_expo_mode {
	OSM_EXPO_AUTO,		/* shutter and gain automatic */
	OSM_EXPO_SHUTTER,	/* shutter fixed, gain automatic */
	OSM_EXPO_GAIN,		/* gain fixed, shutter automatic */
	OSM_EXPO_MANUAL,	/* both fixed */
};

enum osm_flicker {
	OSM_FLICKER_OFF,	/* no avoidance, shutter not quantized */
	OSM_FLICKER_50HZ,
	OSM_FLICKER_60HZ,
	OSM_FLICKER_AUTO,
};

/*
 * Backend of the osm_v4l2 capture device (osm_pipe.ko registers one).
 * Split in two modules because a module that uses symbols of the
 * proprietary vendor modules cannot use GPL-only symbols (V4L2, vb2), and
 * a module using symbols of such a module is tainted the same way: the
 * front only knows this callback table, it never links to the backend.
 * All callbacks run in process context.
 */
struct osm_v4l2_backend {
	struct module *owner;
	const char *name;
	void *priv;
	/* bring up the pipeline for an NV12 frame width x height at fps */
	int (*start)(void *priv, u32 width, u32 height, u32 fps);
	void (*stop)(void *priv);
	/*
	 * Copy one NV12 frame (Y plane, then interleaved UV, stride = width)
	 * into dst (size bytes). -EAGAIN when no frame arrived within about
	 * 200 ms, other errors are fatal for that buffer. *ts_ns is the
	 * capture time (ktime_get_ns() timebase) or 0 for "now" (the front stamps it;
	 * a backend using vendor symbols cannot call ktime_get()).
	 */
	int (*read_frame)(void *priv, void *dst, u32 size, u64 *ts_ns);
	/*
	 * Image controls. set_ctrl may be called right after start(): the
	 * backend keeps the value until the 3A accepts it. get_ctrl returns
	 * the value in effect (EXPO_US/GAIN: the current exposure) or
	 * -EAGAIN while the 3A is not up yet.
	 */
	int (*set_ctrl)(void *priv, enum osm_ctrl id, s32 val);
	int (*get_ctrl)(void *priv, enum osm_ctrl id, s32 *val);
};

int osm_v4l2_register(const struct osm_v4l2_backend *be);
void osm_v4l2_unregister(const struct osm_v4l2_backend *be);

#endif
