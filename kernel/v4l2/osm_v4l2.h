/* SPDX-License-Identifier: GPL-2.0 */
#ifndef OSM_V4L2_H
#define OSM_V4L2_H

#include <linux/types.h>

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
};

int osm_v4l2_register(const struct osm_v4l2_backend *be);
void osm_v4l2_unregister(const struct osm_v4l2_backend *be);

#endif
