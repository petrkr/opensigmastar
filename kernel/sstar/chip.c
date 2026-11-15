// SPDX-License-Identifier: GPL-2.0
/*
 * Chip_* API of the SigmaStar BSP (arch/arm/mach-sstar/ms_chip.c and
 * infinity6e/soc.c) for Infinity6E.
 *
 * Cortex-A7 has no outer cache, so the vendor cache helpers are L1 D-cache
 * maintenance by VA to PoC followed by a MIU write pipe flush. The by-VA
 * operations work for any mapped address (linear map, vmap, ioremap of
 * DRAM), as the vendor versions did.
 */
#include <linux/clk.h>
#include <linux/clk/clk-conf.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/barrier.h>
#include <asm/cacheflush.h>
#include "osm_sstar.h"

#define MIU0_BASE		0x20000000ULL

/* PMTOP bank: chip ID in reg 0x00, revision in the high byte of reg 0x67 */
#define PMTOP_PA		0x1f003c00
#define PMTOP_ID		0x00
#define PMTOP_REV		(0x67 * 4)

/* MIU write pipe flush: fire in 0x1f204414, done bit 12 of 0x1f204440 */
#define MIUFLUSH_PA		0x1f204400
#define MIUFLUSH_CTRL		0x14
#define MIUFLUSH_STATUS		0x40
#define MIUFLUSH_DONE		BIT(12)

/* MS_I6_PACKAGE_TYPE of the BSP */
#define MS_I6E_PACKAGE_QFN_DDR3	0x01

static void __iomem *pmtop;
static void __iomem *miuflush;
static DEFINE_SPINLOCK(miuflush_lock);

static int package_type = MS_I6E_PACKAGE_QFN_DDR3;
module_param(package_type, int, 0444);
MODULE_PARM_DESC(package_type, "Chip_Get_Package_Type() value (MS_I6_PACKAGE_TYPE)");

static void chip_flush_miu_pipe_nodsb(void)
{
	unsigned long flags;

	spin_lock_irqsave(&miuflush_lock, flags);
	writew(0x10, miuflush + MIUFLUSH_CTRL);
	writew(0x11, miuflush + MIUFLUSH_CTRL);
	while (!(readw(miuflush + MIUFLUSH_STATUS) & MIUFLUSH_DONE))
		cpu_relax();
	spin_unlock_irqrestore(&miuflush_lock, flags);
}

void Chip_Flush_MIU_Pipe(void)
{
	dsb();
	chip_flush_miu_pipe_nodsb();
}
EXPORT_SYMBOL(Chip_Flush_MIU_Pipe);

void Chip_Flush_Memory(void)
{
	chip_flush_miu_pipe_nodsb();
}
EXPORT_SYMBOL(Chip_Flush_Memory);

static inline unsigned long dcache_line(void)
{
	unsigned long ctr;

	asm("mrc p15, 0, %0, c0, c0, 1" : "=r" (ctr));
	return 4 << ((ctr >> 16) & 0xf);
}

#define DCACHE_OP(name, crm)						\
static void name(unsigned long start, unsigned long end)		\
{									\
	unsigned long line = dcache_line();				\
									\
	for (start &= ~(line - 1); start < end; start += line)		\
		asm volatile("mcr p15, 0, %0, c7, " #crm ", 1"		\
			     : : "r" (start) : "memory");		\
	dsb();								\
}

DCACHE_OP(dcache_clean_inv, c14)	/* DCCIMVAC */
DCACHE_OP(dcache_inv, c6)		/* DCIMVAC */

void Chip_Flush_Cache_Range(unsigned long u32Addr, unsigned long u32Size)
{
	if (!u32Addr) {
		pr_err("%s: u32Addr is invalid\n", __func__);
		return;
	}
	dcache_clean_inv(u32Addr, u32Addr + u32Size);
	chip_flush_miu_pipe_nodsb();
}
EXPORT_SYMBOL(Chip_Flush_Cache_Range);

void Chip_Flush_Cache_Range_VA_PA(unsigned long u32VAddr, unsigned long u32PAddr,
				  unsigned long u32Size)
{
	Chip_Flush_Cache_Range(u32VAddr, u32Size);
}
EXPORT_SYMBOL(Chip_Flush_Cache_Range_VA_PA);

void Chip_Inv_Cache_Range(unsigned long u32Addr, unsigned long u32Size)
{
	unsigned long line = dcache_line();
	unsigned long end = u32Addr + u32Size;

	if (!u32Addr) {
		pr_err("%s: u32Addr is invalid\n", __func__);
		return;
	}
	/* Partial lines at the ends hold other data: clean them first */
	if (u32Addr & (line - 1))
		dcache_clean_inv(u32Addr, u32Addr + 1);
	if (end & (line - 1))
		dcache_clean_inv(end, end + 1);
	dcache_inv(u32Addr, end);
}
EXPORT_SYMBOL(Chip_Inv_Cache_Range);

void Chip_Flush_CacheAll(void)
{
	flush_cache_all();
	chip_flush_miu_pipe_nodsb();
}
EXPORT_SYMBOL(Chip_Flush_CacheAll);

u64 Chip_Phys_to_MIU(u64 phys)
{
	return phys - MIU0_BASE;
}
EXPORT_SYMBOL(Chip_Phys_to_MIU);

u64 Chip_MIU_to_Phys(u64 miu)
{
	return miu + MIU0_BASE;
}
EXPORT_SYMBOL(Chip_MIU_to_Phys);

int Chip_Get_Device_ID(void)
{
	return readw(pmtop + PMTOP_ID) & 0xff;
}
EXPORT_SYMBOL(Chip_Get_Device_ID);

int Chip_Get_Revision(void)
{
	return ((readw(pmtop + PMTOP_REV) >> 8) & 0xff) + 1;
}
EXPORT_SYMBOL(Chip_Get_Revision);

char *Chip_Get_Platform_Name(void)
{
	static char name[] = "I6E";

	return name;
}
EXPORT_SYMBOL(Chip_Get_Platform_Name);

int Chip_Get_Package_Type(void)
{
	return package_type;
}
EXPORT_SYMBOL(Chip_Get_Package_Type);

/*
 * Video clocks: the vendor nodes carry clocks= for ISP and FCLK1, but the
 * vendor drivers never enable them through the clk API, and they are gated
 * at boot (clk_ignore_unused only keeps them as they are). Gated, the first
 * register access of the vendor ISP/SCL code stalls the CPU. Enable them
 * here from the DT clocks (assigned-clock-parents on the vpe node selects
 * mpll 345 MHz for FCLK1).
 */
static struct clk *video_clks[2];

static void osm_video_clk_get(const char *compat, int idx, bool set_defaults)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, compat);
	struct clk *clk;

	if (!np)
		return;
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		of_node_put(np);
		return;
	}
	if (set_defaults)
		of_clk_set_defaults(np, false);
	of_node_put(np);
	if (clk_prepare_enable(clk)) {
		clk_put(clk);
		return;
	}
	video_clks[idx] = clk;
}

static void osm_video_clk_init(void)
{
	osm_video_clk_get("isp", 0, false);
	osm_video_clk_get("sigmastar,vpe", 1, true);
}

static void osm_video_clk_exit(void)
{
	int i;

	for (i = 0; i < 2; i++)
		if (video_clks[i]) {
			clk_disable_unprepare(video_clks[i]);
			clk_put(video_clks[i]);
			video_clks[i] = NULL;
		}
}

int osm_chip_init(void)
{
	osm_video_clk_init();
	pmtop = ioremap(PMTOP_PA, 0x200);
	miuflush = ioremap(MIUFLUSH_PA, 0x100);
	if (!pmtop || !miuflush) {
		osm_chip_exit();
		return -ENOMEM;
	}
	return 0;
}

void osm_chip_exit(void)
{
	osm_video_clk_exit();
	if (miuflush)
		iounmap(miuflush);
	if (pmtop)
		iounmap(pmtop);
}
