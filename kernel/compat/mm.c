// SPDX-License-Identifier: GPL-2.0
/*
 * 4.9 scatterlists, file I/O, delayed work and shrinkers of the vendor
 * modules.
 */
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/shrinker.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include "osm_compat.h"

/* struct scatterlist and sg_table have the 4.9 layout (16 and 12 bytes) */
static_assert(sizeof(struct scatterlist) == 16);
static_assert(sizeof(struct sg_table) == 12);

int osm_sg_alloc_table(struct sg_table *table, unsigned int nents, unsigned int flags)
{
	return sg_alloc_table(table, nents, osm_gfp(flags));
}
EXPORT_SYMBOL(osm_sg_alloc_table);

struct scatterlist *osm_sg_next(struct scatterlist *sg)
{
	return sg_next(sg);
}
EXPORT_SYMBOL(osm_sg_next);

/*
 * struct delayed_work of 4.9 is 48 bytes: work (16), timer_list (24: entry,
 * expires, function, data, flags), wq, cpu. The current one (44 bytes) fits
 * at the same address; work and timer.function are at the same offsets,
 * timer.data of the inline 4.9 INIT_DELAYED_WORK lands on timer.flags.
 * A delayed work set up by vendor code is initialised again on first use.
 */
static_assert(sizeof(struct delayed_work) <= 48);

static bool osm_dwork_fresh(struct delayed_work *dw)
{
	return dw->timer.function == delayed_work_timer_fn &&
	       dw->timer.flags == (u32)(unsigned long)dw;
}

/* The vendor work function, kept in the unused last word of the 4.9 struct */
#define OSM_DWORK_FUNC(dw)	(*(work_func_t *)((u8 *)(dw) + 44))
static_assert(sizeof(struct delayed_work) <= 44);

static void osm_dwork_fn(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct osm_entry e;

	osm_enter(&e);
	OSM_DWORK_FUNC(dw)(work);
	osm_leave(&e);
}

static void osm_dwork_init(struct delayed_work *dw)
{
	if (osm_dwork_fresh(dw)) {
		OSM_DWORK_FUNC(dw) = dw->work.func;
		INIT_DELAYED_WORK(dw, osm_dwork_fn);
	}
}

/* 4.9 init_timer_key(): only used by the vendor for INIT_DELAYED_WORK */
void osm_init_timer_key(void *timer49, unsigned int flags, const char *name, void *key)
{
	memset(timer49, 0, 24);
}
EXPORT_SYMBOL(osm_init_timer_key);

bool osm_queue_delayed_work_on(int cpu, struct workqueue_struct *wq,
			       struct delayed_work *dw, unsigned long delay)
{
	osm_dwork_init(dw);
	return queue_delayed_work_on(cpu, wq, dw, delay);
}
EXPORT_SYMBOL(osm_queue_delayed_work_on);

bool osm_cancel_delayed_work(struct delayed_work *dw)
{
	if (osm_dwork_fresh(dw))
		return false;
	return cancel_delayed_work(dw);
}
EXPORT_SYMBOL(osm_cancel_delayed_work);

/*
 * Shrinkers: 4.9 struct shrinker { count_objects, scan_objects, seeks,
 * batch, flags, list, nr_deferred } and shrink_control { gfp_mask,
 * nr_to_scan, nid, memcg }. The vendor callbacks get a 4.9 shrink_control.
 */
struct shrinker49 {
	unsigned long (*count_objects)(struct shrinker49 *, void *sc);
	unsigned long (*scan_objects)(struct shrinker49 *, void *sc);
	int seeks;
	long batch;
	unsigned long flags;
};

struct shrink_control49 {
	unsigned int gfp_mask;
	unsigned long nr_to_scan;
	int nid;
	void *memcg;
};

struct osm_shrinker {
	struct list_head list;
	struct shrinker49 *s49;
	struct shrinker *real;
};

static LIST_HEAD(osm_shrinkers);
static DEFINE_MUTEX(osm_shrinkers_lock);

static unsigned long osm_shrink(struct shrinker *s, struct shrink_control *sc, bool scan)
{
	struct osm_shrinker *o = s->private_data;
	struct shrink_control49 sc49 = {
		.gfp_mask = osm_gfp49(sc->gfp_mask),
		.nr_to_scan = sc->nr_to_scan,
		.nid = sc->nid,
	};
	struct osm_entry e;
	unsigned long r;

	osm_enter(&e);
	r = scan ? o->s49->scan_objects(o->s49, &sc49) : o->s49->count_objects(o->s49, &sc49);
	osm_leave(&e);
	return r;
}

static unsigned long osm_shrink_count(struct shrinker *s, struct shrink_control *sc)
{
	return osm_shrink(s, sc, false);
}

static unsigned long osm_shrink_scan(struct shrinker *s, struct shrink_control *sc)
{
	return osm_shrink(s, sc, true);
}

int osm_register_shrinker(struct shrinker49 *s49)
{
	struct osm_shrinker *o;

	o = kzalloc(sizeof(*o), GFP_KERNEL);
	if (!o)
		return -ENOMEM;
	o->real = shrinker_alloc(0, "osm-%ps", s49->count_objects);
	if (!o->real) {
		kfree(o);
		return -ENOMEM;
	}
	o->s49 = s49;
	o->real->count_objects = osm_shrink_count;
	o->real->scan_objects = osm_shrink_scan;
	o->real->seeks = s49->seeks;
	o->real->batch = s49->batch;
	o->real->private_data = o;
	mutex_lock(&osm_shrinkers_lock);
	list_add(&o->list, &osm_shrinkers);
	mutex_unlock(&osm_shrinkers_lock);
	shrinker_register(o->real);
	return 0;
}
EXPORT_SYMBOL(osm_register_shrinker);

/* The vendor modules never unregister their shrinkers: drop them on unload */
static int osm_mm_module_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct module *mod = data;
	struct osm_shrinker *o, *t;

	if (action != MODULE_STATE_GOING)
		return NOTIFY_DONE;
	mutex_lock(&osm_shrinkers_lock);
	list_for_each_entry_safe(o, t, &osm_shrinkers, list) {
		if (within_module((unsigned long)o->s49->count_objects, mod)) {
			list_del(&o->list);
			shrinker_free(o->real);
			kfree(o);
		}
	}
	mutex_unlock(&osm_shrinkers_lock);
	return NOTIFY_OK;
}

static struct notifier_block osm_mm_module_nb = {
	.notifier_call = osm_mm_module_notify,
};

int osm_mm_init(void)
{
	return register_module_notifier(&osm_mm_module_nb);
}

void osm_mm_exit(void)
{
	unregister_module_notifier(&osm_mm_module_nb);
}
