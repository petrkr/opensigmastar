/* SPDX-License-Identifier: GPL-2.0 */
#ifndef OSM_ISP_H
#define OSM_ISP_H

#include <linux/types.h>

/* apply an ISP tuning bin (firmware name) to an open ISP channel, see osm_isp.c */
int osm_isp_load_bin(u32 ch, const char *name);

/* raw ISP API get/set of the kernel's 3A (MHalIspApi*), id/size as in osm_isp_table.h */
int osm_isp_api_set(u32 ch, u32 id, u32 size, void *buf);
int osm_isp_api_get(u32 ch, u32 id, u32 size, void *buf);

#endif
