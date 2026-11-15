// SPDX-License-Identifier: GPL-2.0
/*
 * MMA heaps of the SigmaStar BSP (drivers/sstar/mma_heap) and the Linux
 * memory layout variables (drivers/sstar/cpu/memory.c).
 *
 * The BSP reserves the heaps at boot with memblock from the mma_heap=
 * kernel parameter. Here they are allocated from CMA when the module is
 * loaded (osm_sstar mma_heap=<name>,miu=0,sz=<size>), so the memory stays
 * usable by Linux without the video stack. The heaps keep struct pages
 * and a cached linear mapping as in the BSP: mi_sys checks pfn_valid() and
 * vmaps them, and MI_SYS_FreeMmaHeap gives them back with
 * free_reserved_area().
 */
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include "osm_sstar.h"

#define MAX_MMA_AREAS		30
#define MMA_HEAP_NAME_LENG	32

/* struct MMA_BootArgs_Config of the BSP, 52 bytes on 32-bit ARM */
struct MMA_BootArgs_Config {
	int miu;
	u32 size;
	char name[MMA_HEAP_NAME_LENG];
	u32 max_start_offset_to_curr_bus_base;
	u32 max_end_offset_to_curr_bus_base;
	u32 reserved_start;
};
static_assert(sizeof(struct MMA_BootArgs_Config) == 52);

struct MMA_BootArgs_Config mma_config[MAX_MMA_AREAS];
EXPORT_SYMBOL(mma_config);
int mstar_driver_boot_mma_buffer_num;
EXPORT_SYMBOL(mstar_driver_boot_mma_buffer_num);

/* Linux memory of the BSP: one region, mem2/mem3 unused */
u32 lx_mem_addr, lx_mem_size;
u32 lx_mem2_addr = 0xffffffff, lx_mem2_size = 0xffffffff;
u32 lx_mem3_addr = 0xffffffff, lx_mem3_size = 0xffffffff;
EXPORT_SYMBOL(lx_mem_addr);
EXPORT_SYMBOL(lx_mem_size);
EXPORT_SYMBOL(lx_mem2_addr);
EXPORT_SYMBOL(lx_mem2_size);
EXPORT_SYMBOL(lx_mem3_addr);
EXPORT_SYMBOL(lx_mem3_size);

static char *mma_heap;
module_param(mma_heap, charp, 0444);
MODULE_PARM_DESC(mma_heap, "MMA heap as in the BSP: <name>,miu=0,sz=<bytes> (from CMA)");

static struct page *mma_pages[MAX_MMA_AREAS];
static dma_addr_t mma_dma[MAX_MMA_AREAS];

static int mma_parse(char *arg, struct MMA_BootArgs_Config *c)
{
	char *opt;

	memset(c, 0, sizeof(*c));
	c->max_start_offset_to_curr_bus_base = ~0U;
	c->max_end_offset_to_curr_bus_base = ~0U;
	c->reserved_start = ~0U;
	opt = strsep(&arg, ",");
	if (!opt || !*opt)
		return -EINVAL;
	strscpy(c->name, opt, sizeof(c->name));
	while ((opt = strsep(&arg, ","))) {
		if (!strncmp(opt, "miu=", 4))
			c->miu = simple_strtol(opt + 4, NULL, 0);
		else if (!strncmp(opt, "sz=", 3))
			c->size = memparse(opt + 3, NULL);
	}
	return c->size && !c->miu ? 0 : -EINVAL;
}

int osm_mma_init(struct device *dev)
{
	struct MMA_BootArgs_Config *c = &mma_config[0];
	char *arg;
	int ret;

	lx_mem_addr = PHYS_OFFSET;
	/* the whole DRAM, as LX_MEM= of the BSP: mi_sys removes the heaps from it */
	lx_mem_size = PFN_PHYS(max_mapnr);

	if (!mma_heap)
		return 0;
	arg = kstrdup(mma_heap, GFP_KERNEL);
	if (!arg)
		return -ENOMEM;
	ret = mma_parse(arg, c);
	kfree(arg);
	if (ret) {
		pr_err("osm_sstar: bad mma_heap=%s\n", mma_heap);
		return ret;
	}

	c->size = PAGE_ALIGN(c->size);
	mma_pages[0] = dma_alloc_pages(dev, c->size, &mma_dma[0], DMA_BIDIRECTIONAL,
				       GFP_KERNEL | __GFP_NOWARN);
	if (!mma_pages[0]) {
		pr_err("osm_sstar: no CMA for mma heap %s (0x%x bytes)\n", c->name, c->size);
		return -ENOMEM;
	}
	c->reserved_start = page_to_phys(mma_pages[0]);
	mstar_driver_boot_mma_buffer_num = 1;
	pr_info("osm_sstar: mma heap %s at 0x%08x, 0x%x bytes\n", c->name,
		c->reserved_start, c->size);
	return 0;
}

static void mma_release(struct device *dev, int i)
{
	dma_free_pages(dev, mma_config[i].size, mma_pages[i], mma_dma[i], DMA_BIDIRECTIONAL);
	mma_pages[i] = NULL;
}

void osm_mma_exit(struct device *dev)
{
	int i;

	for (i = 0; i < mstar_driver_boot_mma_buffer_num; i++)
		if (mma_pages[i])
			mma_release(dev, i);
}

/*
 * free_reserved_area() of mi_sys (MI_SYS_FreeMmaHeap) on a heap: give the
 * heap back to CMA. Returns the number of pages, as the kernel function.
 */
unsigned long osm_free_reserved_area(void *start, void *end, int poison, const char *s)
{
	phys_addr_t pa = virt_to_phys(start);
	int i;

	for (i = 0; i < mstar_driver_boot_mma_buffer_num; i++) {
		if (mma_pages[i] && pa == page_to_phys(mma_pages[i]) &&
		    end - start == mma_config[i].size) {
			mma_release(osm_msys_device(), i);
			return mma_config[i].size >> PAGE_SHIFT;
		}
	}
	pr_err("osm_sstar: free_reserved_area(%px, %px) is not an mma heap\n", start, end);
	return 0;
}
EXPORT_SYMBOL(osm_free_reserved_area);
