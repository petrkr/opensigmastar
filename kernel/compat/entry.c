// SPDX-License-Identifier: GPL-2.0
/*
 * 4.9 thread_info/task_struct/mm_struct emulation for vendor code entries,
 * see osm_compat.h.
 */
#include <linux/build_bug.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/smp.h>
#include <linux/string.h>
#include <asm/domain.h>
#include "osm_compat.h"

static inline u8 *osm_ti49(void)
{
	BUILD_BUG_ON(THREAD_SIZE != 8192);
	return (u8 *)(current_stack_pointer & ~(THREAD_SIZE - 1));
}

static inline void osm_put(u8 *base, unsigned int off, u32 val)
{
	*(u32 *)(base + off) = val;
}

void osm_enter(struct osm_entry *e)
{
	u8 *ti = osm_ti49();
	struct mm_struct *mm = current->mm;

	e->ti = ti;
	INIT_LIST_HEAD(&e->vmas);
	memcpy(e->saved, ti + OSM_TI49_PREEMPT, sizeof(e->saved));

	memset(e->task, 0, sizeof(e->task));
	osm_put(e->task, OSM_TASK49_FLAGS, current->flags);
	osm_put(e->task, OSM_TASK49_PID, current->pid);
	osm_put(e->task, OSM_TASK49_TGID, current->tgid);
	if (mm) {
		memset(e->mm.raw, 0, sizeof(e->mm.raw));
		e->mm.mm = mm;
		osm_put(e->task, OSM_TASK49_MM, (u32)&e->mm);
	}

	osm_put(ti, OSM_TI49_PREEMPT, 0);
	osm_put(ti, OSM_TI49_ADDR_LIMIT, OSM_USER_DS);
	osm_put(ti, OSM_TI49_TASK, (u32)e->task);
	osm_put(ti, OSM_TI49_CPU, raw_smp_processor_id());
	osm_put(ti, OSM_TI49_CPU_DOMAIN, get_domain());
}
EXPORT_SYMBOL(osm_enter);

void osm_leave(struct osm_entry *e)
{
	osm_vma_release(e);
	WARN_ON_ONCE(*(u32 *)(e->ti + OSM_TI49_PREEMPT));
	memcpy(e->ti + OSM_TI49_PREEMPT, e->saved, sizeof(e->saved));
}
EXPORT_SYMBOL(osm_leave);

struct osm_entry *osm_current_entry(void)
{
	u8 *task = (u8 *)*(u32 *)(osm_ti49() + OSM_TI49_TASK);

	return container_of((u8 (*)[OSM_TASK49_SIZE])task, struct osm_entry, task);
}
EXPORT_SYMBOL(osm_current_entry);

bool osm_kernel_ds(void)
{
	return *(u32 *)(osm_ti49() + OSM_TI49_ADDR_LIMIT) == OSM_KERNEL_DS;
}
EXPORT_SYMBOL(osm_kernel_ds);
