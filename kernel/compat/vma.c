// SPDX-License-Identifier: GPL-2.0
/*
 * 4.9 address space API of the vendor modules (mi_sys user mappings).
 *
 * The vendor code takes current->mm->mmap_sem, creates an anonymous shared
 * mapping with do_mmap(), looks it up with find_vma() and fills it with
 * remap_pfn_range(), reading and writing vm_area_struct fields inline at
 * the 4.9 offsets. find_vma() hands out a 4.9 vm_area_struct shadowing the
 * real one; the vendor changes to it (vm_page_prot, vm_flags, vm_ops,
 * vm_private_data) go to the real vma. The 4.9 mmap_sem is not taken: the
 * current kernel functions used here take the mmap lock themselves (do_mmap
 * is not exported, vm_mmap locks), so each step is atomic on its own.
 */
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "osm_compat.h"

/* mem_map users in the vendor code assume 32-byte pages */
static_assert(sizeof(struct page) == 32);

/* struct vm_area_struct of 4.9 */
#define VMA49_START		0
#define VMA49_END		4
#define VMA49_MM		32
#define VMA49_PAGE_PROT		36
#define VMA49_FLAGS		40
#define VMA49_OPS		72
#define VMA49_PGOFF		76
#define VMA49_FILE		80
#define VMA49_PRIVATE		84
#define VMA49_SIZE		88

/* struct vm_operations_struct of 4.9 (slots) and struct vm_fault */
#define VMOPS49_OPEN		0
#define VMOPS49_CLOSE		1
#define VMOPS49_FAULT		4
#define VMF49_ADDRESS		12
#define VMF49_SIZE		64

#define OSM_MAGIC_VMA		0x6f736d76

struct osm_vma49 {
	struct list_head list;
	u8 raw[VMA49_SIZE];
	u32 magic;
	struct vm_area_struct *real;
};

struct osm_vmops {
	struct list_head list;
	const void *const *t49;
	struct vm_operations_struct ops;
};

static LIST_HEAD(osm_vmops_list);
static DEFINE_MUTEX(osm_vmops_lock);

static inline u32 get32(const u8 *b, unsigned int off) { return *(const u32 *)(b + off); }
static inline void put32(u8 *b, unsigned int off, u32 v) { *(u32 *)(b + off) = v; }

static struct osm_vma49 *osm_vma(void *vma49)
{
	struct osm_vma49 *v = container_of((u8 (*)[VMA49_SIZE])vma49, struct osm_vma49, raw);

	if (WARN_ON_ONCE(!vma49 || v->magic != OSM_MAGIC_VMA))
		return NULL;
	return v;
}

static struct mm_struct *osm_mm(void *mm49)
{
	return container_of((u8 (*)[OSM_MM49_SIZE])mm49, struct osm_mm49, raw)->mm;
}

static const void *osm_vmops49(const struct vm_operations_struct *ops);

static void osm_vma_fill(u8 *raw, struct vm_area_struct *vma, void *mm49)
{
	memset(raw, 0, VMA49_SIZE);
	put32(raw, VMA49_START, vma->vm_start);
	put32(raw, VMA49_END, vma->vm_end);
	put32(raw, VMA49_MM, (u32)mm49);
	put32(raw, VMA49_PAGE_PROT, pgprot_val(vma->vm_page_prot));
	put32(raw, VMA49_FLAGS, vma->vm_flags);
	put32(raw, VMA49_OPS, (u32)osm_vmops49(vma->vm_ops));
	put32(raw, VMA49_PGOFF, vma->vm_pgoff);
	put32(raw, VMA49_FILE, (u32)vma->vm_file);
	put32(raw, VMA49_PRIVATE, (u32)vma->vm_private_data);
}

/* Vendor callbacks of a translated vm_operations_struct */
static struct osm_vmops *osm_vmops_of(const struct vm_operations_struct *ops)
{
	return container_of(ops, struct osm_vmops, ops);
}

static void osm_vm_call(struct vm_area_struct *vma, int slot)
{
	void (*fn)(void *) = osm_vmops_of(vma->vm_ops)->t49[slot];
	struct osm_vma49 v = { .magic = OSM_MAGIC_VMA, .real = vma };
	struct osm_entry e;

	osm_enter(&e);
	osm_vma_fill(v.raw, vma, &e.mm.raw);
	fn(v.raw);
	osm_leave(&e);
}

static void osm_vm_open(struct vm_area_struct *vma)
{
	osm_vm_call(vma, VMOPS49_OPEN);
}

static void osm_vm_close(struct vm_area_struct *vma)
{
	osm_vm_call(vma, VMOPS49_CLOSE);
}

static vm_fault_t osm_vm_fault(struct vm_fault *vmf)
{
	int (*fn)(void *, void *) = osm_vmops_of(vmf->vma->vm_ops)->t49[VMOPS49_FAULT];
	struct osm_vma49 v = { .magic = OSM_MAGIC_VMA, .real = vmf->vma };
	u8 vmf49[VMF49_SIZE] = {};
	struct osm_entry e;
	int r;

	/* only the vendor SIGBUS handler exists: the address is all it reads */
	put32(vmf49, VMF49_ADDRESS, vmf->address);
	osm_enter(&e);
	osm_vma_fill(v.raw, vmf->vma, &e.mm.raw);
	r = fn(v.raw, vmf49);
	osm_leave(&e);
	return r;
}

static const struct vm_operations_struct *osm_vmops_get(const void *const *t49)
{
	struct osm_vmops *o;

	if (!t49)
		return NULL;
	mutex_lock(&osm_vmops_lock);
	list_for_each_entry(o, &osm_vmops_list, list)
		if (o->t49 == t49)
			goto out;
	o = kzalloc(sizeof(*o), GFP_KERNEL);
	if (!o)
		goto out;
	o->t49 = t49;
	if (t49[VMOPS49_OPEN])
		o->ops.open = osm_vm_open;
	if (t49[VMOPS49_CLOSE])
		o->ops.close = osm_vm_close;
	if (t49[VMOPS49_FAULT])
		o->ops.fault = osm_vm_fault;
	list_add(&o->list, &osm_vmops_list);
out:
	mutex_unlock(&osm_vmops_lock);
	return o ? &o->ops : NULL;
}

/* real vm_ops back to the vendor table, other ones stay opaque */
static const void *osm_vmops49(const struct vm_operations_struct *ops)
{
	struct osm_vmops *o;

	if (!ops)
		return NULL;
	list_for_each_entry(o, &osm_vmops_list, list)
		if (&o->ops == ops)
			return o->t49;
	return ops;
}

/* Vendor changes to the shadow go to the real vma (mmap lock held) */
static void osm_vma_sync(struct osm_vma49 *v)
{
	struct vm_area_struct *vma = v->real;
	const void *ops49 = (const void *)get32(v->raw, VMA49_OPS);
	vm_flags_t set = get32(v->raw, VMA49_FLAGS) & ~vma->vm_flags;

	vma->vm_page_prot = __pgprot(get32(v->raw, VMA49_PAGE_PROT));
	vma->vm_pgoff = get32(v->raw, VMA49_PGOFF);
	vma->vm_private_data = (void *)get32(v->raw, VMA49_PRIVATE);
	if (ops49 != osm_vmops49(vma->vm_ops))
		vma->vm_ops = osm_vmops_get(ops49);
	if (set)
		vm_flags_set(vma, set);
}

void osm_vma_release(struct osm_entry *e)
{
	struct osm_vma49 *v, *t;

	list_for_each_entry_safe(v, t, &e->vmas, list) {
		list_del(&v->list);
		kfree(v);
	}
}
EXPORT_SYMBOL(osm_vma_release);

void *osm_find_vma(void *mm49, unsigned long addr)
{
	struct mm_struct *mm = osm_mm(mm49);
	struct osm_entry *e = osm_current_entry();
	struct vm_area_struct *vma;
	struct osm_vma49 *v;

	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return NULL;
	mmap_read_lock(mm);
	vma = find_vma(mm, addr);
	if (vma)
		osm_vma_fill(v->raw, vma, mm49);
	mmap_read_unlock(mm);
	if (!vma) {
		kfree(v);
		return NULL;
	}
	v->magic = OSM_MAGIC_VMA;
	v->real = vma;
	list_add(&v->list, &e->vmas);
	return v->raw;
}
EXPORT_SYMBOL(osm_find_vma);

int osm_remap_pfn_range(void *vma49, unsigned long addr, unsigned long pfn,
			unsigned long size, pgprot_t prot)
{
	struct osm_vma49 *v = osm_vma(vma49);
	struct mm_struct *mm;
	int ret;

	if (!v)
		return -EINVAL;
	mm = v->real->vm_mm;
	mmap_write_lock(mm);
	osm_vma_sync(v);
	ret = remap_pfn_range(v->real, addr, pfn, size, prot);
	/* the vendor reads vm_flags back and ORs its own bits */
	put32(v->raw, VMA49_FLAGS, v->real->vm_flags);
	mmap_write_unlock(mm);
	return ret;
}
EXPORT_SYMBOL(osm_remap_pfn_range);

int osm_follow_pfn(void *vma49, unsigned long addr, unsigned long *pfn)
{
	struct osm_vma49 *v = osm_vma(vma49);
	struct follow_pfnmap_args args = { .address = addr };
	struct mm_struct *mm;
	int ret;

	if (!v)
		return -EINVAL;
	mm = v->real->vm_mm;
	args.vma = v->real;
	mmap_read_lock(mm);
	ret = follow_pfnmap_start(&args);
	if (!ret) {
		*pfn = args.pfn;
		follow_pfnmap_end(&args);
	}
	mmap_read_unlock(mm);
	return ret;
}
EXPORT_SYMBOL(osm_follow_pfn);

/* 4.9 do_mmap(file, addr, len, prot, flags, vm_flags, pgoff, *populate) */
unsigned long osm_do_mmap(struct file *file, unsigned long addr, unsigned long len,
			  unsigned long prot, unsigned long flags, unsigned long vm_flags,
			  unsigned long pgoff, unsigned long *populate)
{
	WARN_ON_ONCE(vm_flags);
	*populate = 0;
	return vm_mmap(file, addr, len, prot, flags, pgoff << PAGE_SHIFT);
}
EXPORT_SYMBOL(osm_do_mmap);

int osm___mm_populate(unsigned long start, unsigned long len, int ignore_errors)
{
	return 0;
}
EXPORT_SYMBOL(osm___mm_populate);

int osm_do_munmap(void *mm49, unsigned long start, size_t len)
{
	return vm_munmap(start, len);
}
EXPORT_SYMBOL(osm_do_munmap);

/*
 * 4.9 rw_semaphore: only current->mm->mmap_sem is used by the vendor code.
 * Before it is released, vendor changes to the vmas found meanwhile are
 * written back.
 */
static bool osm_is_mmap_sem(void *sem49)
{
	struct osm_entry *e = osm_current_entry();

	return sem49 == e->mm.raw + OSM_MM49_MMAP_SEM;
}

void osm_down_write(void *sem49)
{
	WARN_ONCE(!osm_is_mmap_sem(sem49), "osm: 4.9 rw_semaphore %px is not supported\n", sem49);
}
EXPORT_SYMBOL(osm_down_write);

void osm_up_write(void *sem49)
{
	struct osm_entry *e = osm_current_entry();
	struct osm_vma49 *v;

	if (!osm_is_mmap_sem(sem49) || !e->mm.mm)
		return;
	mmap_write_lock(e->mm.mm);
	list_for_each_entry(v, &e->vmas, list) {
		/* skip vmas the vendor unmapped meanwhile */
		if (find_vma(e->mm.mm, v->real->vm_start) == v->real)
			osm_vma_sync(v);
	}
	mmap_write_unlock(e->mm.mm);
}
EXPORT_SYMBOL(osm_up_write);
