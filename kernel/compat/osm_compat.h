/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Linux 4.9 ABI emulation for the SigmaStar vendor modules.
 *
 * The vendor modules were built for a Thumb-2 SMP PREEMPT 4.9 kernel and
 * inline current_thread_info() as (sp & ~(THREAD_SIZE - 1)). They read
 * addr_limit (set_fs), task (current) and preempt_count from there and
 * follow task->mm with the 4.9 layouts. The current kernel keeps
 * thread_info in task_struct, so every entry from the kernel into vendor
 * code (module init/exit, file operations, threads, timers, interrupts,
 * ...) must go through osm_enter()/osm_leave(), which put a 4.9
 * thread_info at the bottom of the stack, pointing to a 4.9 task_struct
 * and mm_struct with the fields the vendor code uses.
 */
#ifndef OSM_COMPAT_H
#define OSM_COMPAT_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/gfp_types.h>

/* Offsets in the 4.9 structures, taken from the vendor code */
#define OSM_TI49_PREEMPT	4
#define OSM_TI49_ADDR_LIMIT	8
#define OSM_TI49_TASK		12
#define OSM_TI49_CPU		16
#define OSM_TI49_CPU_DOMAIN	20
#define OSM_TI49_END		24	/* offset 0 holds STACK_END_MAGIC */

#define OSM_TASK49_FLAGS	12
#define OSM_TASK49_MM		552
#define OSM_TASK49_PID		680
#define OSM_TASK49_TGID		684
#define OSM_TASK49_SIZE		704

#define OSM_MM49_MMAP_SEM	72
#define OSM_MM49_SIZE		128

/* 4.9 USER_DS and KERNEL_DS with VMSPLIT_3G */
#define OSM_USER_DS		0xbf000000UL
#define OSM_KERNEL_DS		0x00000000UL

struct osm_mm49 {
	u8 raw[OSM_MM49_SIZE];
	struct mm_struct *mm;
};

struct osm_entry {
	u32 saved[(OSM_TI49_END - OSM_TI49_PREEMPT) / 4];
	u8 *ti;
	u8 task[OSM_TASK49_SIZE] __aligned(8);
	struct osm_mm49 mm;
	struct list_head vmas;	/* 4.9 vm_area_structs handed out, see vma.c */
};

void osm_enter(struct osm_entry *e);
void osm_leave(struct osm_entry *e);

/* entry of the vendor code running on this stack */
struct osm_entry *osm_current_entry(void);
void osm_vma_release(struct osm_entry *e);

/* current wait queue head inside a 4.9 one, see locks.c */
struct wait_queue_head *osm_wq(void *wq49);

struct file_operations;
struct kobject;
const struct file_operations *osm_fops49(const void *f49);
extern struct kobject *(*osm_kobj_real)(void *kobj49);

/* 4.9 struct completion (done + 4.9 wait queue head), see locks.c */
void osm_completion49_init(void *c49);
void osm_completion49_complete(void *c49, bool all);
long osm_completion49_wait(void *c49, long timeout, bool interruptible);
bool osm_completion49_done(void *c49);

/* gfp_t of 4.9 to current and back */
gfp_t osm_gfp(unsigned int g);
unsigned int osm_gfp49(gfp_t g);

/* addr_limit of the running vendor code, for the vfs_read/vfs_write shims */
bool osm_kernel_ds(void);

#endif
