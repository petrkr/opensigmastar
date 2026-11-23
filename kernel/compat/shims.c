// SPDX-License-Identifier: GPL-2.0
/*
 * Linux 4.9 kernel API on top of the current kernel. The vendor modules
 * are relinked with redefine.map, which renames the 4.9 symbols to the
 * osm_* functions here.
 */
#include <linux/delay.h>
#include <linux/gfp.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/sched/debug.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include "osm_compat.h"

void osm___aeabi_uldivmod(void);
void osm___aeabi_ldivmod(void);
void osm___aeabi_uidiv(void);

#ifndef CONFIG_STACKPROTECTOR
/* The vendor modules are built with -fstack-protector */
unsigned long __stack_chk_guard = 0x5a7c4b3e;
EXPORT_SYMBOL(__stack_chk_guard);

void __stack_chk_fail(void)
{
	panic("stack-protector: kernel stack corrupted in: %pB",
	      __builtin_return_address(0));
}
EXPORT_SYMBOL(__stack_chk_fail);
#endif

/* gfp_t bits of 4.9 */
#define G49_DMA			0x01u
#define G49_HIGHMEM		0x02u
#define G49_MOVABLE		0x08u
#define G49_RECLAIMABLE		0x10u
#define G49_HIGH		0x20u
#define G49_IO			0x40u
#define G49_FS			0x80u
#define G49_NOWARN		0x200u
#define G49_REPEAT		0x400u
#define G49_NOFAIL		0x800u
#define G49_NORETRY		0x1000u
#define G49_MEMALLOC		0x2000u
#define G49_COMP		0x4000u
#define G49_ZERO		0x8000u
#define G49_NOMEMALLOC		0x10000u
#define G49_HARDWALL		0x20000u
#define G49_THISNODE		0x40000u
#define G49_ATOMIC		0x80000u
#define G49_ACCOUNT		0x100000u
#define G49_DIRECT_RECLAIM	0x400000u
#define G49_WRITE		0x1000000u
#define G49_KSWAPD_RECLAIM	0x2000000u

static const struct { unsigned int from; gfp_t to; } osm_gfp_map[] = {
		{ G49_DMA, __GFP_DMA },
		{ G49_HIGHMEM, __GFP_HIGHMEM },
		{ G49_MOVABLE, __GFP_MOVABLE },
		{ G49_RECLAIMABLE, __GFP_RECLAIMABLE },
		{ G49_HIGH, __GFP_HIGH },
		{ G49_IO, __GFP_IO },
		{ G49_FS, __GFP_FS },
		{ G49_NOWARN, __GFP_NOWARN },
		{ G49_REPEAT, __GFP_RETRY_MAYFAIL },
		{ G49_NOFAIL, __GFP_NOFAIL },
		{ G49_NORETRY, __GFP_NORETRY },
		{ G49_MEMALLOC, __GFP_MEMALLOC },
		{ G49_COMP, __GFP_COMP },
		{ G49_ZERO, __GFP_ZERO },
		{ G49_NOMEMALLOC, __GFP_NOMEMALLOC },
		{ G49_HARDWALL, __GFP_HARDWALL },
		{ G49_THISNODE, __GFP_THISNODE },
		/* __GFP_ATOMIC is gone, its users are __GFP_HIGH now */
		{ G49_ATOMIC, __GFP_HIGH },
		{ G49_ACCOUNT, __GFP_ACCOUNT },
		{ G49_DIRECT_RECLAIM, __GFP_DIRECT_RECLAIM },
		{ G49_WRITE, __GFP_WRITE },
		{ G49_KSWAPD_RECLAIM, __GFP_KSWAPD_RECLAIM },
};

gfp_t osm_gfp(unsigned int g)
{
	gfp_t r = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(osm_gfp_map); i++)
		if (g & osm_gfp_map[i].from)
			r |= osm_gfp_map[i].to;
	return r;
}
EXPORT_SYMBOL(osm_gfp);

/* current gfp_t to 4.9, for callbacks into vendor code */
unsigned int osm_gfp49(gfp_t g)
{
	unsigned int r = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(osm_gfp_map); i++)
		if (osm_gfp_map[i].from != G49_ATOMIC && (g & osm_gfp_map[i].to))
			r |= osm_gfp_map[i].from;
	return r;
}
EXPORT_SYMBOL(osm_gfp49);

void *osm___kmalloc(size_t size, unsigned int flags)
{
	return kmalloc(size, osm_gfp(flags));
}
EXPORT_SYMBOL(osm___kmalloc);

/*
 * 4.9 kmalloc() with a constant size is inlined as
 * kmem_cache_alloc(kmalloc_caches[index], flags). The vendor code gets
 * tokens instead of caches, osm_kmem_cache_alloc() turns them back into
 * kmalloc() of the size of that index (SLUB, KMALLOC_SHIFT_HIGH = 13).
 */
#define OSM_KMALLOC_CACHES	14
static u8 osm_kmalloc_tokens[OSM_KMALLOC_CACHES];
void *osm_kmalloc_caches[OSM_KMALLOC_CACHES] = {
#define T(i) [i] = &osm_kmalloc_tokens[i]
	T(0), T(1), T(2), T(3), T(4), T(5), T(6), T(7),
	T(8), T(9), T(10), T(11), T(12), T(13),
#undef T
};
EXPORT_SYMBOL(osm_kmalloc_caches);

static int osm_kmalloc_index(const void *cachep)
{
	const u8 *p = cachep;

	if (p < osm_kmalloc_tokens || p >= osm_kmalloc_tokens + OSM_KMALLOC_CACHES)
		return -1;
	return p - osm_kmalloc_tokens;
}

void *osm_kmem_cache_alloc(struct kmem_cache *cachep, unsigned int flags)
{
	int i = osm_kmalloc_index(cachep);

	if (i < 0)
		return kmem_cache_alloc(cachep, osm_gfp(flags));
	return kmalloc(i == 1 ? 96 : i == 2 ? 192 : 1U << i, osm_gfp(flags));
}
EXPORT_SYMBOL(osm_kmem_cache_alloc);

void osm_kmem_cache_free(struct kmem_cache *cachep, void *obj)
{
	if (osm_kmalloc_index(cachep) < 0)
		kmem_cache_free(cachep, obj);
	else
		kfree(obj);
}
EXPORT_SYMBOL(osm_kmem_cache_free);

void *osm_kmemdup(const void *src, size_t len, unsigned int flags)
{
	return kmemdup(src, len, osm_gfp(flags));
}
EXPORT_SYMBOL(osm_kmemdup);

char *osm_kstrdup(const char *s, unsigned int flags)
{
	return kstrdup(s, osm_gfp(flags));
}
EXPORT_SYMBOL(osm_kstrdup);

struct page *osm___alloc_pages_nodemask(unsigned int flags, unsigned int order,
					 void *zonelist, void *nodemask)
{
	return alloc_pages(osm_gfp(flags), order);
}
EXPORT_SYMBOL(osm___alloc_pages_nodemask);

/*
 * struct mutex of 4.9 (DEBUG_MUTEXES, DEBUG_SPINLOCK) is 36 bytes starting
 * with the count (1 when unlocked, also in DEFINE_MUTEX). A current mutex
 * is kept at offset 4 and set up on first use, the first word marks that.
 */
#define OSM_MUTEX_MAGIC	0x4d757478
struct osm_mutex49 {
	u32 state;
	struct mutex m;
};
static_assert(sizeof(struct osm_mutex49) <= 36);
static DEFINE_SPINLOCK(osm_mutex_init_lock);

static struct mutex *osm_mutex(struct osm_mutex49 *l)
{
	if (smp_load_acquire(&l->state) != OSM_MUTEX_MAGIC) {
		spin_lock(&osm_mutex_init_lock);
		if (l->state != OSM_MUTEX_MAGIC) {
			mutex_init(&l->m);
			smp_store_release(&l->state, OSM_MUTEX_MAGIC);
		}
		spin_unlock(&osm_mutex_init_lock);
	}
	return &l->m;
}

void osm___mutex_init(struct osm_mutex49 *l, const char *name, void *key)
{
	mutex_init(&l->m);
	smp_store_release(&l->state, OSM_MUTEX_MAGIC);
}
EXPORT_SYMBOL(osm___mutex_init);

void osm_mutex_lock(struct osm_mutex49 *l)
{
	mutex_lock(osm_mutex(l));
}
EXPORT_SYMBOL(osm_mutex_lock);

void osm_mutex_unlock(struct osm_mutex49 *l)
{
	mutex_unlock(osm_mutex(l));
}
EXPORT_SYMBOL(osm_mutex_unlock);

/* Inlined by the current kernel; 4.9 arch_spinlock_t has the same layout */
void osm__raw_spin_unlock(raw_spinlock_t *lock)
{
	raw_spin_unlock(lock);
}
EXPORT_SYMBOL(osm__raw_spin_unlock);

char *osm_strncpy(char *dest, const char *src, size_t count)
{
	char *tmp = dest;

	while (count) {
		if ((*tmp = *src) != 0)
			src++;
		tmp++;
		count--;
	}
	return dest;
}
EXPORT_SYMBOL(osm_strncpy);

void osm_usleep_range(unsigned long min, unsigned long max)
{
	usleep_range(min, max);
}
EXPORT_SYMBOL(osm_usleep_range);

unsigned int osm_jiffies_to_msecs(unsigned long j)
{
	return jiffies_to_msecs(j);
}
EXPORT_SYMBOL(osm_jiffies_to_msecs);

void osm_getnstimeofday64(struct timespec64 *ts)
{
	ktime_get_real_ts64(ts);
}
EXPORT_SYMBOL(osm_getnstimeofday64);

void osm_show_stack(struct task_struct *task, unsigned long *sp)
{
	dump_stack();
}
EXPORT_SYMBOL(osm_show_stack);

void osm_warn_slowpath_null(const char *file, int line)
{
	pr_warn("WARNING: at %s:%d %pS\n", file, line,
		__builtin_return_address(0));
	dump_stack();
}
EXPORT_SYMBOL(osm_warn_slowpath_null);

/* Helper of osm___aeabi_ldivmod (ldivmod.S): remainder has the sign of n */
s64 osm_div64_s64_rem(s64 n, s64 d, s64 *rem)
{
	s64 q = div64_s64(n, d);

	*rem = n - q * d;
	return q;
}

EXPORT_SYMBOL(osm___aeabi_uldivmod);
EXPORT_SYMBOL(osm___aeabi_ldivmod);
EXPORT_SYMBOL(osm___aeabi_uidiv);

int osm_fops_init(void);
void osm_fops_exit(void);
int osm_mm_init(void);
void osm_mm_exit(void);
void osm_file_init(void);

static int __init osm_compat_init(void)
{
	int ret = osm_fops_init();

	osm_file_init();

	if (ret)
		return ret;
	ret = osm_mm_init();
	if (ret) {
		osm_fops_exit();
		return ret;
	}
	return 0;
}
module_init(osm_compat_init);

static void __exit osm_compat_exit(void)
{
	osm_mm_exit();
	osm_fops_exit();
}
module_exit(osm_compat_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Linux 4.9 kernel API for the SigmaStar vendor modules");
