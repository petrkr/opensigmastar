// SPDX-License-Identifier: GPL-2.0
/*
 * 4.9 file_operations of the vendor modules on the current kernel.
 *
 * For every vendor file_operations table a current file_operations and
 * proc_ops table of trampolines is built. On open, a 4.9 struct file and
 * inode are set up with the fields the vendor code reads, and the vendor
 * callbacks get those. Kernel helpers in the vendor table (seq_read,
 * seq_lseek, single_release, ...) are called with the real objects.
 *
 * seq_file: single_open() from vendor code gets a 4.9 seq_file for the
 * vendor show callback; seq_printf() on it goes to the real seq_file.
 */
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include "osm_compat.h"

/* struct file_operations of 4.9, slot numbers */
enum {
	F49_OWNER, F49_LLSEEK, F49_READ, F49_WRITE, F49_READ_ITER, F49_WRITE_ITER,
	F49_ITERATE, F49_ITERATE_SHARED, F49_POLL, F49_UNLOCKED_IOCTL,
	F49_COMPAT_IOCTL, F49_MMAP, F49_OPEN, F49_FLUSH, F49_RELEASE, F49_FSYNC,
	F49_FASYNC, F49_NR = 30,
};

/* 4.9 struct file (DEBUG_SPINLOCK, DEBUG_MUTEXES), offsets from the vendor code */
#define FILE49_INODE		16
#define FILE49_OP		20
#define FILE49_FLAGS		44
#define FILE49_MODE		48
#define FILE49_POS		88
#define FILE49_PRIVATE		176
#define FILE49_SIZE		256
/* 4.9 struct inode */
#define INODE49_RDEV		44
#define INODE49_SIZE		512
/* 4.9 struct seq_file */
#define SEQ49_PRIVATE		96
#define SEQ49_SIZE		128

#define OSM_MAGIC_FILE		0x6f736d66
#define OSM_MAGIC_SEQ		0x6f736d73

typedef void *slot_t;

struct osm_fops {
	struct list_head list;
	const slot_t *f49;
	struct module *owner;
	struct file_operations fops;
	struct proc_ops pops;
};

struct osm_seq49 {
	u8 raw[SEQ49_SIZE];
	u32 magic;
	struct seq_file *real;
	int (*show)(void *m, void *v);
	void *data;
};

struct osm_file49 {
	u8 file[FILE49_SIZE];
	u8 inode[INODE49_SIZE];
	u32 magic;
	struct hlist_node node;
	struct file *real;
	struct inode *real_inode;
	struct osm_fops *of;
	struct osm_seq49 *seq;
};

/* proc entries: pde data is this, the vendor data is in it */
struct osm_pde {
	struct osm_fops *of;
	void *data;
};

static LIST_HEAD(osm_fops_list);
static DEFINE_MUTEX(osm_fops_lock);
static DEFINE_HASHTABLE(osm_files, 8);
static DEFINE_SPINLOCK(osm_files_lock);

static inline void put32(u8 *base, unsigned int off, u32 v)
{
	*(u32 *)(base + off) = v;
}

static inline u32 get32(const u8 *base, unsigned int off)
{
	return *(const u32 *)(base + off);
}

static bool vendor_fn(struct osm_fops *of, const void *fn)
{
	return of->owner && within_module((unsigned long)fn, of->owner);
}

static struct osm_file49 *osm_file_lookup(struct file *file)
{
	struct osm_file49 *f;

	spin_lock(&osm_files_lock);
	hash_for_each_possible(osm_files, f, node, (unsigned long)file)
		if (f->real == file)
			goto out;
	f = NULL;
out:
	spin_unlock(&osm_files_lock);
	return f;
}

/* 4.9 struct file of the vendor code back to the real one */
static struct osm_file49 *osm_file49(const void *file49)
{
	struct osm_file49 *f = (struct osm_file49 *)file49;

	if (WARN_ON_ONCE(!f || f->magic != OSM_MAGIC_FILE))
		return NULL;
	return f;
}

static void osm_file_sync_in(struct osm_file49 *f)
{
	put32(f->file, FILE49_FLAGS, f->real->f_flags);
	*(loff_t *)(f->file + FILE49_POS) = f->real->f_pos;
}

static void osm_file_sync_out(struct osm_file49 *f)
{
	f->real->f_pos = *(loff_t *)(f->file + FILE49_POS);
}

#define SLOT(f, n)	((f)->of->f49[n])

/* Calls a vendor callback (4.9 objects) or a kernel helper (real objects) */
#define OSM_CALL(f, n, type, vargs, rargs) ({				\
	typeof(type) fn = SLOT(f, n);					\
	typeof(fn vargs) r;						\
	if (vendor_fn((f)->of, fn)) {					\
		struct osm_entry e;					\
		osm_file_sync_in(f);					\
		osm_enter(&e);						\
		r = fn vargs;						\
		osm_leave(&e);						\
		osm_file_sync_out(f);					\
	} else {							\
		r = fn rargs;						\
	}								\
	r; })

static int osm_open_common(struct osm_fops *of, struct inode *inode, struct file *file)
{
	struct osm_file49 *f;
	int ret = 0;

	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	f->magic = OSM_MAGIC_FILE;
	f->real = file;
	f->real_inode = inode;
	f->of = of;
	put32(f->file, FILE49_INODE, (u32)f->inode);
	put32(f->file, FILE49_OP, (u32)of->f49);
	put32(f->file, FILE49_MODE, file->f_mode);
	put32(f->inode, INODE49_RDEV, inode->i_rdev);

	spin_lock(&osm_files_lock);
	hash_add(osm_files, &f->node, (unsigned long)file);
	spin_unlock(&osm_files_lock);

	if (SLOT(f, F49_OPEN))
		ret = OSM_CALL(f, F49_OPEN, int (*)(void *, void *),
			       (f->inode, f->file), (inode, file));
	if (ret) {
		spin_lock(&osm_files_lock);
		hash_del(&f->node);
		spin_unlock(&osm_files_lock);
		kfree(f->seq);
		kfree(f);
	}
	return ret;
}

static int osm_chr_open(struct inode *inode, struct file *file)
{
	return osm_open_common(container_of(file->f_op, struct osm_fops, fops), inode, file);
}

static int osm_proc_open(struct inode *inode, struct file *file)
{
	struct osm_pde *p = pde_data(inode);

	return osm_open_common(p->of, inode, file);
}

static int osm_release(struct inode *inode, struct file *file)
{
	struct osm_file49 *f = osm_file_lookup(file);
	int ret = 0;

	if (!f)
		return 0;
	if (SLOT(f, F49_RELEASE))
		ret = OSM_CALL(f, F49_RELEASE, int (*)(void *, void *),
			       (f->inode, f->file), (inode, file));
	spin_lock(&osm_files_lock);
	hash_del(&f->node);
	spin_unlock(&osm_files_lock);
	kfree(f->seq);
	kfree(f);
	return ret;
}

static ssize_t osm_read(struct file *file, char __user *buf, size_t n, loff_t *pos)
{
	struct osm_file49 *f = osm_file_lookup(file);

	if (!f || !SLOT(f, F49_READ))
		return -EINVAL;
	return OSM_CALL(f, F49_READ, ssize_t (*)(void *, char __user *, size_t, loff_t *),
			(f->file, buf, n, pos), (file, buf, n, pos));
}

static ssize_t osm_write(struct file *file, const char __user *buf, size_t n, loff_t *pos)
{
	struct osm_file49 *f = osm_file_lookup(file);

	if (!f || !SLOT(f, F49_WRITE))
		return -EINVAL;
	return OSM_CALL(f, F49_WRITE, ssize_t (*)(void *, const char __user *, size_t, loff_t *),
			(f->file, buf, n, pos), (file, buf, n, pos));
}

static loff_t osm_llseek(struct file *file, loff_t off, int whence)
{
	struct osm_file49 *f = osm_file_lookup(file);

	if (!f || !SLOT(f, F49_LLSEEK))
		return noop_llseek(file, off, whence);
	return OSM_CALL(f, F49_LLSEEK, loff_t (*)(void *, loff_t, int),
			(f->file, off, whence), (file, off, whence));
}

static long osm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct osm_file49 *f = osm_file_lookup(file);

	if (!f || !SLOT(f, F49_UNLOCKED_IOCTL))
		return -ENOTTY;
	return OSM_CALL(f, F49_UNLOCKED_IOCTL, long (*)(void *, unsigned int, unsigned long),
			(f->file, cmd, arg), (file, cmd, arg));
}

/* poll_wait() of the vendor code passes its 4.9 file and wait queue head */
struct osm_poll {
	poll_table pt;
	poll_table *orig;
	struct file *real;
};

void osm_wq_dump_inited(void);

static void osm_qproc(struct file *file49, wait_queue_head_t *wq49, poll_table *pt)
{
	struct osm_poll *p = container_of(pt, struct osm_poll, pt);
	wait_queue_head_t *wq = osm_wq(wq49);

	/* an uninitialised, freed or corrupted 4.9 head: skip it, do not oops */
	if ((unsigned long)wq49 < PAGE_SIZE || !wq->head.next ||
	    (unsigned long)wq->head.next < PAGE_OFFSET ||
	    wq->head.next->prev != &wq->head) {
		if (WARN_ONCE(1, "osm: poll on uninitialised wait queue %px\n", wq49))
			osm_wq_dump_inited();
		return;
	}
	p->orig->_qproc(p->real, wq, p->orig);
}

static __poll_t osm_poll(struct file *file, poll_table *pt)
{
	struct osm_file49 *f = osm_file_lookup(file);
	struct osm_poll p = { .orig = pt, .real = file };
	poll_table *vpt = NULL;

	if (!f || !SLOT(f, F49_POLL))
		return DEFAULT_POLLMASK;
	if (pt && pt->_qproc) {
		init_poll_funcptr(&p.pt, osm_qproc);
		p.pt._key = pt->_key;
		vpt = &p.pt;
	}
	return OSM_CALL(f, F49_POLL, __poll_t (*)(void *, poll_table *),
			(f->file, vpt), (file, pt));
}

static int osm_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct osm_file49 *f = osm_file_lookup(file);

	if (!f || !SLOT(f, F49_MMAP))
		return -ENODEV;
	/* TODO: 4.9 vm_area_struct */
	pr_warn_once("osm: mmap of vendor files is not supported yet\n");
	return -ENODEV;
}

static_assert(sizeof(struct cdev) == 60, "struct cdev differs from 4.9");

static struct osm_fops *osm_fops_get(const void *f49p)
{
	const slot_t *f49 = f49p;
	struct osm_fops *of;

	mutex_lock(&osm_fops_lock);
	list_for_each_entry(of, &osm_fops_list, list)
		if (of->f49 == f49)
			goto out;
	of = kzalloc(sizeof(*of), GFP_KERNEL);
	if (!of)
		goto out;
	of->f49 = f49;
	of->owner = f49[F49_OWNER];
	of->fops.owner = of->owner;
	of->fops.open = osm_chr_open;
	of->fops.release = osm_release;
	of->fops.llseek = osm_llseek;
	if (f49[F49_READ])
		of->fops.read = osm_read;
	if (f49[F49_WRITE])
		of->fops.write = osm_write;
	if (f49[F49_UNLOCKED_IOCTL])
		of->fops.unlocked_ioctl = osm_ioctl;
	if (f49[F49_POLL])
		of->fops.poll = osm_poll;
	if (f49[F49_MMAP])
		of->fops.mmap = osm_mmap;

	of->pops.proc_open = osm_proc_open;
	of->pops.proc_release = osm_release;
	of->pops.proc_lseek = osm_llseek;
	if (f49[F49_READ])
		of->pops.proc_read = osm_read;
	if (f49[F49_WRITE])
		of->pops.proc_write = osm_write;
	if (f49[F49_UNLOCKED_IOCTL])
		of->pops.proc_ioctl = osm_ioctl;
	if (f49[F49_POLL])
		of->pops.proc_poll = osm_poll;
	if (f49[F49_MMAP])
		of->pops.proc_mmap = osm_mmap;
	list_add(&of->list, &osm_fops_list);
out:
	mutex_unlock(&osm_fops_lock);
	return of;
}

/* Drop the tables of a vendor module that goes away */
static int osm_module_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct module *mod = data;
	struct osm_fops *of, *t;

	if (action != MODULE_STATE_GOING)
		return NOTIFY_DONE;
	mutex_lock(&osm_fops_lock);
	list_for_each_entry_safe(of, t, &osm_fops_list, list) {
		if (of->owner == mod) {
			list_del(&of->list);
			kfree(of);
		}
	}
	mutex_unlock(&osm_fops_lock);
	return NOTIFY_OK;
}

static struct notifier_block osm_module_nb = {
	.notifier_call = osm_module_notify,
};

int osm_fops_init(void)
{
	return register_module_notifier(&osm_module_nb);
}

void osm_fops_exit(void)
{
	unregister_module_notifier(&osm_module_nb);
}

int osm___register_chrdev(unsigned int major, unsigned int baseminor, unsigned int count,
			  const char *name, const void *f49)
{
	struct osm_fops *of = osm_fops_get(f49);

	if (!of)
		return -ENOMEM;
	return __register_chrdev(major, baseminor, count, name, &of->fops);
}
EXPORT_SYMBOL(osm___register_chrdev);

void osm_cdev_init(struct cdev *cdev, const void *f49)
{
	struct osm_fops *of = osm_fops_get(f49);

	cdev_init(cdev, of ? &of->fops : NULL);
}
EXPORT_SYMBOL(osm_cdev_init);

/* current file_operations for a vendor table, for misc_register() in osm_sstar */
const struct file_operations *osm_fops49(const void *f49)
{
	struct osm_fops *of = f49 ? osm_fops_get(f49) : NULL;

	return of ? &of->fops : NULL;
}
EXPORT_SYMBOL(osm_fops49);

/* 4.9 device kobject (set by osm_sstar) to the real one */
struct kobject *(*osm_kobj_real)(void *kobj49);
EXPORT_SYMBOL(osm_kobj_real);

/*
 * cdev_alloc() users set cdev->ops to their 4.9 table themselves, and
 * kobj.parent to the kobject of a 4.9 device.
 */
int osm_cdev_add(struct cdev *cdev, dev_t dev, unsigned int count)
{
	struct osm_fops *of;

	if (cdev->kobj.parent && osm_kobj_real)
		cdev->kobj.parent = osm_kobj_real(cdev->kobj.parent);

	mutex_lock(&osm_fops_lock);
	list_for_each_entry(of, &osm_fops_list, list) {
		if (cdev->ops == &of->fops) {
			mutex_unlock(&osm_fops_lock);
			return cdev_add(cdev, dev, count);
		}
	}
	mutex_unlock(&osm_fops_lock);
	of = osm_fops_get(cdev->ops);
	if (!of)
		return -ENOMEM;
	cdev->ops = &of->fops;
	return cdev_add(cdev, dev, count);
}
EXPORT_SYMBOL(osm_cdev_add);

struct proc_dir_entry *osm_proc_create_data(const char *name, umode_t mode,
					    struct proc_dir_entry *parent,
					    const void *f49, void *data)
{
	struct osm_fops *of = osm_fops_get(f49);
	struct proc_dir_entry *pde;
	struct osm_pde *p;

	if (!of)
		return NULL;
	/* freed with the entry: TODO, a few bytes per entry until then */
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;
	p->of = of;
	p->data = data;
	pde = proc_create_data(name, mode, parent, &of->pops, p);
	if (!pde)
		kfree(p);
	return pde;
}
EXPORT_SYMBOL(osm_proc_create_data);

/* PDE_DATA() on the 4.9 inode of a vendor proc file */
void *osm_pde_data(const void *inode49)
{
	struct osm_file49 *f = container_of((u8 (*)[INODE49_SIZE])inode49,
					    struct osm_file49, inode);
	struct osm_pde *p;

	if (WARN_ON_ONCE(f->magic != OSM_MAGIC_FILE))
		return NULL;
	p = pde_data(f->real_inode);
	return p->data;
}
EXPORT_SYMBOL(osm_pde_data);

static int osm_seq_show(struct seq_file *m, void *v)
{
	struct osm_seq49 *s = m->private;
	struct osm_entry e;
	int ret;

	s->real = m;
	osm_enter(&e);
	ret = s->show(s, v);
	osm_leave(&e);
	return ret;
}

int osm_single_open(void *file49, int (*show)(void *, void *), void *data)
{
	struct osm_file49 *f = osm_file49(file49);
	struct osm_seq49 *s;
	int ret;

	if (!f)
		return -EINVAL;
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->magic = OSM_MAGIC_SEQ;
	s->show = show;
	s->data = data;
	put32(s->raw, SEQ49_PRIVATE, (u32)data);
	ret = single_open(f->real, osm_seq_show, s);
	if (ret) {
		kfree(s);
		return ret;
	}
	f->seq = s;
	put32(f->file, FILE49_PRIVATE, (u32)s);
	return 0;
}
EXPORT_SYMBOL(osm_single_open);

/*
 * Kernel seq helpers renamed by redefine.map: the vendor code calls them
 * with its 4.9 file, or they sit in a vendor fops table and get the real
 * file from the trampolines.
 */
static struct file *osm_real_file(void *file)
{
	struct osm_file49 *f = osm_file_lookup(file);

	if (f)
		return file;
	f = osm_file49(file);
	return f ? f->real : NULL;
}

ssize_t osm_seq_read(void *file, char __user *buf, size_t n, loff_t *pos)
{
	return seq_read(osm_real_file(file), buf, n, pos);
}
EXPORT_SYMBOL(osm_seq_read);

loff_t osm_seq_lseek(void *file, loff_t off, int whence)
{
	return seq_lseek(osm_real_file(file), off, whence);
}
EXPORT_SYMBOL(osm_seq_lseek);

int osm_single_release(void *inode, void *file)
{
	struct file *real = osm_real_file(file);

	return single_release(file_inode(real), real);
}
EXPORT_SYMBOL(osm_single_release);

static struct seq_file *osm_seq(void *m49)
{
	struct osm_seq49 *s = m49;

	if (WARN_ON_ONCE(!s || s->magic != OSM_MAGIC_SEQ || !s->real))
		return NULL;
	return s->real;
}

void osm_seq_vprintf(void *m49, const char *fmt, va_list args)
{
	struct seq_file *m = osm_seq(m49);

	if (m)
		seq_vprintf(m, fmt, args);
}
EXPORT_SYMBOL(osm_seq_vprintf);

void osm_seq_printf(void *m49, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	osm_seq_vprintf(m49, fmt, args);
	va_end(args);
}
EXPORT_SYMBOL(osm_seq_printf);
