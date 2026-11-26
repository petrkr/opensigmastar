/* SPDX-License-Identifier: GPL-2.0 */
#ifndef OSM_ISP_H
#define OSM_ISP_H

#include <linux/types.h>

/* apply an ISP tuning bin (firmware name) to an open ISP channel, see osm_isp.c */
int osm_isp_load_bin(u32 ch, const char *name);

#endif
