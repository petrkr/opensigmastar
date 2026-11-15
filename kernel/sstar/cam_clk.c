// SPDX-License-Identifier: GPL-2.0
/*
 * Cam clock wrappers of the SigmaStar BSP (drivers/sstar/cam_clkgen/
 * cam_clkgen.c).
 *
 * All functions are thin pass-throughs to the common clock API. The vendor
 * mhal.ko reads hw->clk (offset 4 of struct clk_hw, layout identical to 4.9)
 * of the CamClkGetParentByIndex() result inline and without a NULL check.
 * In 4.9 that field was always set; in 7.4 it is NULL unless a consumer was
 * created, so such hws are replaced by a cached shadow carrying a consumer
 * clk. A failed lookup returns a shadow holding ERR_PTR(), which the vendor
 * checks with IS_ERR().
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>

struct osm_clk_shadow {
	struct clk_hw hw;		/* handed to the vendor, must stay first */
	const struct clk_hw *real;
	struct list_head node;
};

static LIST_HEAD(shadows);
static DEFINE_MUTEX(shadows_lock);

/*
 * hw handed out when there is no parent: a NULL clk, which the clk API
 * accepts (rate 0). mhal calls clk_get_rate() on parents without checks.
 */
static struct clk_hw err_hw;

unsigned long CamClkGetRate(struct clk *clk)
{
	return clk_get_rate(clk);
}
EXPORT_SYMBOL(CamClkGetRate);

int CamClkPrepare(struct clk *clk)
{
	return clk_prepare(clk);
}
EXPORT_SYMBOL(CamClkPrepare);

int CamClkEnable(struct clk *clk)
{
	return clk_enable(clk);
}
EXPORT_SYMBOL(CamClkEnable);

void CamClkUnprepare(struct clk *clk)
{
	clk_unprepare(clk);
}
EXPORT_SYMBOL(CamClkUnprepare);

void CamClkDisable(struct clk *clk)
{
	clk_disable(clk);
}
EXPORT_SYMBOL(CamClkDisable);

int CamClkSetRate(struct clk *clk, unsigned long rate)
{
	return clk_set_rate(clk, rate);
}
EXPORT_SYMBOL(CamClkSetRate);

int CamClkSetParent(struct clk *clk, struct clk *parent)
{
	return clk_set_parent(clk, parent);
}
EXPORT_SYMBOL(CamClkSetParent);

long CamClkRoundRate(struct clk *clk, unsigned long rate)
{
	return clk_round_rate(clk, rate);
}
EXPORT_SYMBOL(CamClkRoundRate);

struct clk_hw *__CamClkGetHw(struct clk *clk)
{
	return __clk_get_hw(clk);
}
EXPORT_SYMBOL(__CamClkGetHw);

struct clk_hw *CamClkGetParentByIndex(const struct clk_hw *hw,
				      unsigned int index)
{
	struct clk_hw *parent = clk_hw_get_parent_by_index(hw, index);
	struct osm_clk_shadow *s;
	struct clk *clk;

	if (!parent)
		return &err_hw;
	if (parent->clk)
		return parent;

	mutex_lock(&shadows_lock);
	list_for_each_entry(s, &shadows, node) {
		if (s->real == parent)
			goto out;
	}

	clk = clk_hw_get_clk(parent, "osm");
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (IS_ERR(clk) || !s) {
		if (!IS_ERR(clk))
			clk_put(clk);
		kfree(s);
		mutex_unlock(&shadows_lock);
		return &err_hw;
	}
	s->hw = *parent;
	s->hw.clk = clk;
	s->real = parent;
	list_add(&s->node, &shadows);
out:
	mutex_unlock(&shadows_lock);
	return &s->hw;
}
EXPORT_SYMBOL(CamClkGetParentByIndex);

unsigned int CamOfClkGetParentCount(struct device_node *np)
{
	int count = of_count_phandle_with_args(np, "clocks", "#clock-cells");

	return count < 0 ? 0 : count;
}
EXPORT_SYMBOL(CamOfClkGetParentCount);

const char *CamOfClkGetParentName(struct device_node *np, int index)
{
	return of_clk_get_parent_name(np, index);
}
EXPORT_SYMBOL(CamOfClkGetParentName);

/* To be called from module exit */
void osm_cam_clk_exit(void)
{
	struct osm_clk_shadow *s, *tmp;

	list_for_each_entry_safe(s, tmp, &shadows, node) {
		clk_put(s->hw.clk);
		list_del(&s->node);
		kfree(s);
	}
}
