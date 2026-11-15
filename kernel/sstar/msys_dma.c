// SPDX-License-Identifier: GPL-2.0
/*
 * msys_dma_* of the SigmaStar BSP (drivers/sstar/msys/ms_msys_dma_wrapper.c)
 * on the BDMA dmaengine driver (sstar-bdma).
 *
 * Addresses are MIU addresses, as the BSP API takes them; the BDMA driver
 * programs them unchanged. Fills with a pattern that is not one repeated
 * byte, and line copies with different source and destination widths, are
 * done by the CPU (the dmaengine memset takes a byte, the BDMA line mode
 * one width). The callers do the cache maintenance, as with the BSP.
 */
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include "osm_sstar.h"

typedef struct {
	unsigned long long phyaddr;
	unsigned int length;
	unsigned int pattern;
} MSYS_DMA_FILL;

typedef struct {
	unsigned long long phyaddr_src;
	unsigned long long phyaddr_dst;
	unsigned int length;
} MSYS_DMA_COPY;

typedef struct {
	unsigned long long phyaddr_src;
	unsigned long long phyaddr_dst;
	unsigned int lineofst_src;
	unsigned int lineofst_dst;
	unsigned int width_src;
	unsigned int width_dst;
	unsigned int length;
} MSYS_DMA_BLIT;

typedef struct {
	unsigned long long phyaddr;
	unsigned int length;
	unsigned int lineofst_dst;
	unsigned int width_dst;
	unsigned int pattern;
} MSYS_DMA_FILL_BILT;

static struct dma_chan *msys_chan;
static DEFINE_MUTEX(msys_dma_lock);

static int msys_dma_run(struct dma_async_tx_descriptor *tx)
{
	dma_cookie_t cookie;

	if (!tx)
		return -EINVAL;
	cookie = dmaengine_submit(tx);
	if (dma_submit_error(cookie))
		return -EIO;
	dma_async_issue_pending(msys_chan);
	return dma_sync_wait(msys_chan, cookie) == DMA_COMPLETE ? 0 : -ETIMEDOUT;
}

/* Kernel mapping of DRAM given by its MIU address */
static void *msys_va(unsigned long long miu, size_t len)
{
	phys_addr_t pa = Chip_MIU_to_Phys(miu);

	if (pfn_valid(PHYS_PFN(pa)) && !PageHighMem(pfn_to_page(PHYS_PFN(pa))))
		return phys_to_virt(pa);
	return memremap(pa, len, MEMREMAP_WB);
}

static void msys_va_put(void *va, unsigned long long miu)
{
	if (va != phys_to_virt(Chip_MIU_to_Phys(miu)))
		memunmap(va);
}

static void cpu_fill(void *dst, unsigned int len, u32 pattern)
{
	u32 *p = dst;
	unsigned int i;

	for (i = 0; i + 4 <= len; i += 4)
		*p++ = pattern;
	memcpy(p, &pattern, len - i);
}

static bool pattern_is_byte(u32 p)
{
	return p == (p & 0xff) * 0x01010101U;
}

int msys_dma_fill(MSYS_DMA_FILL *cfg)
{
	int ret;

	if (msys_chan && pattern_is_byte(cfg->pattern)) {
		mutex_lock(&msys_dma_lock);
		ret = msys_dma_run(dmaengine_prep_dma_memset(msys_chan, cfg->phyaddr,
							     cfg->pattern & 0xff,
							     cfg->length, DMA_PREP_INTERRUPT));
		mutex_unlock(&msys_dma_lock);
		return ret ? -1 : 0;
	} else {
		void *va = msys_va(cfg->phyaddr, cfg->length);

		if (!va)
			return -1;
		cpu_fill(va, cfg->length, cfg->pattern);
		Chip_Flush_Cache_Range((unsigned long)va, cfg->length);
		msys_va_put(va, cfg->phyaddr);
		return 0;
	}
}
EXPORT_SYMBOL(msys_dma_fill);

int msys_dma_copy(MSYS_DMA_COPY *cfg)
{
	int ret = -ENODEV;

	if (msys_chan) {
		mutex_lock(&msys_dma_lock);
		ret = msys_dma_run(dmaengine_prep_dma_memcpy(msys_chan, cfg->phyaddr_dst,
							     cfg->phyaddr_src, cfg->length,
							     DMA_PREP_INTERRUPT));
		mutex_unlock(&msys_dma_lock);
	} else {
		void *src = msys_va(cfg->phyaddr_src, cfg->length);
		void *dst = msys_va(cfg->phyaddr_dst, cfg->length);

		if (src && dst) {
			Chip_Inv_Cache_Range((unsigned long)src, cfg->length);
			memcpy(dst, src, cfg->length);
			Chip_Flush_Cache_Range((unsigned long)dst, cfg->length);
			ret = 0;
		}
		if (src)
			msys_va_put(src, cfg->phyaddr_src);
		if (dst)
			msys_va_put(dst, cfg->phyaddr_dst);
	}
	return ret ? -1 : 0;
}
EXPORT_SYMBOL(msys_dma_copy);

int msys_dma_fill_lineoffset(MSYS_DMA_FILL_BILT *cfg)
{
	unsigned int lines, i;
	void *va;

	if (!cfg->lineofst_dst) {
		MSYS_DMA_FILL f = { cfg->phyaddr, cfg->length, cfg->pattern };

		return msys_dma_fill(&f);
	}
	if (cfg->lineofst_dst < cfg->width_dst || !cfg->width_dst)
		return -1;

	/* the BDMA has no line mode for fills */
	lines = DIV_ROUND_UP(cfg->length, cfg->width_dst);
	va = msys_va(cfg->phyaddr, (size_t)cfg->lineofst_dst * lines);
	if (!va)
		return -1;
	for (i = 0; i < lines; i++)
		cpu_fill(va + i * cfg->lineofst_dst,
			 min(cfg->width_dst, cfg->length - i * cfg->width_dst), cfg->pattern);
	Chip_Flush_Cache_Range((unsigned long)va, (size_t)cfg->lineofst_dst * lines);
	msys_va_put(va, cfg->phyaddr);
	return 0;
}
EXPORT_SYMBOL(msys_dma_fill_lineoffset);

int msys_dma_copy_lineoffset(MSYS_DMA_BLIT *cfg)
{
	struct dma_interleaved_template *xt;
	unsigned int lines, i;
	void *src, *dst;
	int ret;

	if (!cfg->lineofst_src || !cfg->lineofst_dst) {
		MSYS_DMA_COPY c = { cfg->phyaddr_src, cfg->phyaddr_dst, cfg->length };

		return msys_dma_copy(&c);
	}
	if (cfg->lineofst_src < cfg->width_src || cfg->lineofst_dst < cfg->width_dst ||
	    !cfg->width_src || !cfg->width_dst)
		return -1;

	if (msys_chan && cfg->width_src == cfg->width_dst && !(cfg->length % cfg->width_src)) {
		xt = kzalloc(struct_size(xt, sgl, 1), GFP_KERNEL);
		if (!xt)
			return -1;
		xt->src_start = cfg->phyaddr_src;
		xt->dst_start = cfg->phyaddr_dst;
		xt->dir = DMA_MEM_TO_MEM;
		xt->src_inc = true;
		xt->dst_inc = true;
		xt->src_sgl = true;
		xt->dst_sgl = true;
		xt->numf = cfg->length / cfg->width_src;
		xt->frame_size = 1;
		xt->sgl[0].size = cfg->width_src;
		xt->sgl[0].src_icg = cfg->lineofst_src - cfg->width_src;
		xt->sgl[0].dst_icg = cfg->lineofst_dst - cfg->width_dst;
		mutex_lock(&msys_dma_lock);
		ret = msys_dma_run(dmaengine_prep_interleaved_dma(msys_chan, xt,
								  DMA_PREP_INTERRUPT));
		mutex_unlock(&msys_dma_lock);
		kfree(xt);
		return ret ? -1 : 0;
	}

	/* different widths: copy line by line with the CPU */
	lines = DIV_ROUND_UP(cfg->length, cfg->width_src);
	src = msys_va(cfg->phyaddr_src, (size_t)cfg->lineofst_src * lines);
	dst = msys_va(cfg->phyaddr_dst, (size_t)cfg->lineofst_dst * lines);
	if (src && dst) {
		Chip_Inv_Cache_Range((unsigned long)src, (size_t)cfg->lineofst_src * lines);
		for (i = 0; i < lines; i++)
			memcpy(dst + i * cfg->lineofst_dst, src + i * cfg->lineofst_src,
			       min3(cfg->width_src, cfg->width_dst,
				    cfg->length - i * cfg->width_src));
		Chip_Flush_Cache_Range((unsigned long)dst, (size_t)cfg->lineofst_dst * lines);
	}
	if (src)
		msys_va_put(src, cfg->phyaddr_src);
	if (dst)
		msys_va_put(dst, cfg->phyaddr_dst);
	return src && dst ? 0 : -1;
}
EXPORT_SYMBOL(msys_dma_copy_lineoffset);

void osm_msys_dma_init(void)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	msys_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(msys_chan)) {
		pr_warn("osm_sstar: no BDMA channel, msys DMA copies fail (%ld)\n",
			PTR_ERR(msys_chan));
		msys_chan = NULL;
	}
}

void osm_msys_dma_exit(void)
{
	if (msys_chan)
		dma_release_channel(msys_chan);
}
