/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SigmaStar vendor support API used by the ported code (msys, MIU, chip).
 * Layouts as in the SigmaStar BSP (OpenIPC/linux, sigmastar-infinity6e).
 */
#ifndef OSM_SSTAR_H
#define OSM_SSTAR_H

#include <linux/types.h>

/* drivers/sstar/include/mdrv_msys_io_st.h, also the /dev/msys ioctl ABI */
typedef struct {
	unsigned int VerChk_Version;
	char name[16];
	unsigned int length;
	unsigned long long phys;
	unsigned long long kvirt;
	unsigned int option;
	unsigned int VerChk_Size;
} __attribute__((__packed__)) MSYS_DMEM_INFO;

int msys_request_dmem(MSYS_DMEM_INFO *mem_info);
int msys_release_dmem(MSYS_DMEM_INFO *mem_info);
int msys_find_dmem_by_phys(unsigned long long phys, MSYS_DMEM_INFO *pdmem);

/* MIU_DramInfo, MDrv_MIU_* */
#include "mdrv_miu.h"

/* drivers/sstar/include/ms_platform.h */
void Chip_Flush_MIU_Pipe(void);
void Chip_Flush_Memory(void);
void Chip_Flush_Cache_Range_VA_PA(unsigned long u32VAddr, unsigned long u32PAddr,
				  unsigned long u32Size);
void Chip_Flush_Cache_Range(unsigned long u32Addr, unsigned long u32Size);
void Chip_Inv_Cache_Range(unsigned long u32Addr, unsigned long u32Size);
u64 Chip_Phys_to_MIU(u64 phys);
u64 Chip_MIU_to_Phys(u64 miu);
int Chip_Get_Device_ID(void);
int Chip_Get_Revision(void);
void Chip_Flush_CacheAll(void);
char *Chip_Get_Platform_Name(void);
int Chip_Get_Package_Type(void);

/* opensigmastar module init/exit of the ports */
int osm_chip_init(void);
void osm_chip_exit(void);
int osm_msys_init(void);
void osm_msys_exit(void);
void osm_miu_exit(void);
struct device;
struct device *osm_msys_device(void);
int osm_mma_init(struct device *dev);
void osm_mma_exit(struct device *dev);
void osm_msys_dma_init(void);
void osm_msys_dma_exit(void);
void osm_cam_clk_exit(void);
void osm_board_exit(void);
void osm_cam_sysfs_init(void);
void osm_cam_sysfs_exit(void);

#endif
