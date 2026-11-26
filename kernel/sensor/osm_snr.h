/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ABI of the SigmaStar mhal sensor registry: the sensor object that mhal
 * allocates and the plug-in fills in (recovered from the vendor binary).
 * Offsets are checked at build time.
 */
#ifndef OSM_SNR_H
#define OSM_SNR_H

#include <linux/build_bug.h>
#include <linux/stddef.h>
#include <linux/types.h>

/* mhal exports (vendor ABI) */
struct osm_snr;
typedef int (*osm_snr_init_fn)(struct osm_snr *h);

extern int DrvRegisterSensorDriverEx(u32 pad, osm_snr_init_fn init, void *priv);
extern int DrvRegisterSensorI2CSlaveID(u32 pad, u32 id);
extern int DrvRegisterPlaneDriverEx(u32 pad, u32 plane, osm_snr_init_fn init,
				    void *priv);
extern int DrvRegisterPlaneI2CSlaveID(u32 pad, u32 plane, u32 id);
extern int DrvSensorHandleVer(u32 a, u32 b);
extern int DrvSensorIFVer(u32 a, u32 b);
extern int DrvSensorI2CVer(u32 a, u32 b);
extern int DrvSensorRelease(u32 pad);

#define OSM_SNR_PADS		4

/* I2C configuration object, passed to the I2C ops */
struct osm_snr_i2c_cfg {
	u32 mode;		/* 1 */
	u32 fmt;		/* 1 = 16 bit register, 8 bit data */
	u32 speed;		/* Hz */
	u16 addr;		/* 8 bit address (7 bit << 1) */
	u16 pad;
};

struct osm_snr_reg {
	u16 reg;
	u16 val;
};

/* I2C ops provided by mhal (handle->i2c) */
struct osm_snr_i2c_ops {
	void *rsvd0[5];
	int (*write)(void *bus, struct osm_snr_i2c_cfg *c, u16 reg, u16 val);
	int (*read)(void *bus, struct osm_snr_i2c_cfg *c, u16 reg, u16 *val);
	int (*write_array)(void *bus, struct osm_snr_i2c_cfg *c,
			   struct osm_snr_reg *r, u32 n);
	void *rsvd20[2];
	int (*read_array)(void *bus, struct osm_snr_i2c_cfg *c,
			  struct osm_snr_reg *r, u32 n);
};

/* sensor interface ops provided by mhal (handle->sif) */
struct osm_snr_if_ops {
	void *rsvd0[2];
	int (*pwdn)(u32 pad, u32 val);		/* power-down pin */
	int (*reset)(u32 pad, u32 val);		/* reset pin */
	int (*mclk)(u32 pad, u32 on, u32 mclk);
	void *rsvd14[2];
	int (*io_pad)(u32 pad, u32 mode, u32 lanes);
	int (*csi_clk)(u32 pad, u32 clk);	/* -1 = off */
	int (*csi_lane)(u32 pad, u32 lanes, u32 on);
	int (*csi_pkt)(u32 pad, u32 v0, u32 v1, u32 v2); /* DtEnable masks */
	void *rsvd2c[2];
	int (*hdr_mode)(u32 pad, u32 on);
};

struct osm_snr_res {
	u32 width;		/* output size */
	u32 height;
	u32 max_fps;
	u32 min_fps;
	u32 crop_x;
	u32 crop_y;
	u32 sns_width;		/* sensor output incl. margins */
	u32 sns_height;
	char name[40];
};

struct osm_snr_shutter_info {
	u32 rsvd;
	u32 max_ns;
	u32 step_ns;
	u32 min_ns;
};

/*
 * mhal's sensor object. Only the members the plug-in sets or reads are
 * named, the rest belongs to mhal.
 */
struct osm_snr {
	u8 mhal0[4];
	char name[36];				/* 0x004 */
	void *priv;				/* 0x028 */
	u8 mhal2c[4];
	struct osm_snr_i2c_cfg i2c_cfg;		/* 0x030 */
	struct osm_snr_i2c_ops *i2c;		/* 0x040 */
	struct osm_snr_if_ops *sif;		/* 0x044 */
	u32 f48, f4c, f50, f54;			/* 0x048 */
	u32 reset_pol;				/* 0x058 */
	u32 pwdn_pol;				/* 0x05c */
	u8 mhal60[12];
	u32 f6c;				/* 0x06c */
	u32 f70, f74;
	u32 if_mode;				/* 0x078, 1 = MIPI */
	u32 f7c;
	u32 f80;
	u32 orien;				/* 0x084 current orientation */
	u8 mhal88[8];
	u32 res_cnt;				/* 0x090 */
	u32 res_cur;				/* 0x094 */
	struct osm_snr_res res[11];		/* 0x098 */
	u8 mhal3b0[0x3f8 - 0x3b0];
	u32 lanes;				/* 0x3f8 */
	u32 f3fc, f400, f404;
	u8 mhal408[4];
	u32 hdr_mode;				/* 0x40c */
	u32 f410;
	u8 mhal414[0x424 - 0x414];
	u32 mclk;				/* 0x424 */
	u32 gain_unit;				/* 0x428, 1024 = 1x */
	u8 mhal42c[8];
	int (*poweron)(struct osm_snr *h, u32 pad);	/* 0x434 */
	int (*poweroff)(struct osm_snr *h, u32 pad);	/* 0x438 */
	int (*init)(struct osm_snr *h);			/* 0x43c */
	u8 mhal440[4];
	int (*release)(struct osm_snr *h);		/* 0x444 */
	u8 mhal448[8];
	int (*set_pattern)(struct osm_snr *h, u32 mode); /* 0x450 */
	int (*get_id)(struct osm_snr *h, u32 *id);	/* 0x454 */
	int (*get_res)(struct osm_snr *h, u32 idx,
		       struct osm_snr_res **r);		/* 0x458 */
	int (*get_cur_res)(struct osm_snr *h, u32 *idx,
			   struct osm_snr_res **r);	/* 0x45c */
	int (*set_res)(struct osm_snr *h, u32 idx);	/* 0x460 */
	int (*get_orien)(struct osm_snr *h, u32 *o);	/* 0x464 */
	int (*set_orien)(struct osm_snr *h, u32 o);	/* 0x468 */
	u8 mhal46c[4];
	int (*ae_notify)(struct osm_snr *h, u32 status); /* 0x470 */
	int (*get_us)(struct osm_snr *h, u32 *us);	/* 0x474 */
	int (*set_us)(struct osm_snr *h, u32 us);	/* 0x478 */
	int (*get_gain)(struct osm_snr *h, u32 *g);	/* 0x47c */
	int (*set_gain)(struct osm_snr *h, u32 g);	/* 0x480 */
	int (*minmax_us)(struct osm_snr *h, u32 *min, u32 *max); /* 0x484 */
	int (*minmax_gain)(struct osm_snr *h, u32 *min, u32 *max); /* 0x488 */
	u32 (*get_fps)(struct osm_snr *h);		/* 0x48c */
	int (*set_fps)(struct osm_snr *h, u32 fps);	/* 0x490 */
	u8 mhal494[8];
	int (*shutter_info)(struct osm_snr *h,
			    struct osm_snr_shutter_info *i); /* 0x49c */
	int (*get_res_num)(struct osm_snr *h, u32 *n);	/* 0x4a0 */
};

#define OSM_SNR_CHECK(m, off) \
	static_assert(offsetof(struct osm_snr, m) == (off))
OSM_SNR_CHECK(name, 0x004);
OSM_SNR_CHECK(priv, 0x028);
OSM_SNR_CHECK(i2c_cfg, 0x030);
OSM_SNR_CHECK(i2c, 0x040);
OSM_SNR_CHECK(sif, 0x044);
OSM_SNR_CHECK(reset_pol, 0x058);
OSM_SNR_CHECK(f6c, 0x06c);
OSM_SNR_CHECK(if_mode, 0x078);
OSM_SNR_CHECK(orien, 0x084);
OSM_SNR_CHECK(res_cnt, 0x090);
OSM_SNR_CHECK(res, 0x098);
OSM_SNR_CHECK(lanes, 0x3f8);
OSM_SNR_CHECK(hdr_mode, 0x40c);
OSM_SNR_CHECK(mclk, 0x424);
OSM_SNR_CHECK(poweron, 0x434);
OSM_SNR_CHECK(release, 0x444);
OSM_SNR_CHECK(set_pattern, 0x450);
OSM_SNR_CHECK(ae_notify, 0x470);
OSM_SNR_CHECK(get_fps, 0x48c);
OSM_SNR_CHECK(get_res_num, 0x4a0);
static_assert(sizeof(struct osm_snr_res) == 0x48);

#endif
