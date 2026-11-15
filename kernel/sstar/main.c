// SPDX-License-Identifier: GPL-2.0
/* SigmaStar vendor support code ported from the BSP */
#include <linux/module.h>
#include "osm_riu.h"
#include "osm_sstar.h"

void __iomem *osm_riu;
EXPORT_SYMBOL(osm_riu);
static unsigned long osm_riu_base;
module_param_named(riu_base, osm_riu_base, ulong, 0444);
MODULE_PARM_DESC(riu_base, "Mapped RIU virtual base for pre-load vendor module fixups");

static int __init osm_sstar_init(void)
{
	int ret;

	osm_riu = ioremap(BASE_REG_RIU_PA, OSM_RIU_SIZE);
	if (!osm_riu)
		return -ENOMEM;
	osm_riu_base = (unsigned long)osm_riu;
	ret = osm_chip_init();
	if (ret)
		goto err_riu;
	ret = osm_msys_init();
	if (ret)
		goto err_chip;
	ret = osm_mma_init(osm_msys_device());
	if (ret)
		goto err_msys;
	osm_msys_dma_init();
	osm_cam_sysfs_init();
	return 0;

err_msys:
	osm_msys_exit();
err_chip:
	osm_chip_exit();
err_riu:
	osm_riu_base = 0;
	iounmap(osm_riu);
	return ret;
}
module_init(osm_sstar_init);

static void __exit osm_sstar_exit(void)
{
	osm_cam_sysfs_exit();
	osm_board_exit();
	osm_cam_clk_exit();
	osm_msys_dma_exit();
	osm_miu_exit();
	osm_mma_exit(osm_msys_device());
	osm_msys_exit();
	osm_chip_exit();
	iounmap(osm_riu);
	osm_riu_base = 0;
}
module_exit(osm_sstar_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SigmaStar vendor support code (cam_os_wrapper, msys, MIU, ...)");
