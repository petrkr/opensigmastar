// SPDX-License-Identifier: GPL-2.0
/*
 * File access shims for the vendor modules (filp_open and friends).
 *
 * The vendor code computes &file->f_pos at offset 88 of a 4.9 struct file
 * and reads f_path at offset 8. filp_open() therefore returns a shadow
 * with the 4.9 offsets, which points to the real struct file. All the vfs
 * shims accept a shadow or (for files not opened here) a real struct file.
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/namei.h>
#include <linux/stat.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include "osm_compat.h"

#define OSM_FILE_MAGIC		0x4f534d46	/* "OSMF" */

#define OSM_FILE49_PATH		8	/* struct path: mnt, dentry */
#define OSM_FILE49_F_OP		20
#define OSM_FILE49_F_FLAGS	44
#define OSM_FILE49_F_POS	88
#define OSM_FILE49_AREA		128	/* zeroed, laid out as the 4.9 file */

struct osm_file {
	u8 pad0[OSM_FILE49_PATH];
	void *mnt;
	void *dentry;
	void *f_inode;
	const void *f_op;	/* 4.9 file_operations of this shadow, below */
	u8 pad1[OSM_FILE49_F_FLAGS - OSM_FILE49_F_OP - sizeof(void *)];
	u32 f_flags;
	u32 f_mode;
	u8 pad2[OSM_FILE49_F_POS - OSM_FILE49_F_FLAGS - 2 * sizeof(u32)];
	loff_t f_pos;
	u8 pad3[OSM_FILE49_AREA - OSM_FILE49_F_POS - sizeof(loff_t)];
	u32 magic;
	struct file *real;
	struct list_head node;
};

static_assert(sizeof(void *) == 4);
static_assert(offsetof(struct osm_file, mnt) == OSM_FILE49_PATH);
static_assert(offsetof(struct osm_file, dentry) == OSM_FILE49_PATH + 4);
static_assert(offsetof(struct osm_file, f_op) == OSM_FILE49_F_OP);
static_assert(offsetof(struct osm_file, f_flags) == OSM_FILE49_F_FLAGS);
static_assert(offsetof(struct osm_file, f_pos) == OSM_FILE49_F_POS);

/*
 * mi_sys calls file->f_op->llseek/read/write itself: the shadow gets a 4.9
 * file_operations (slots: owner 0, llseek 1, read 2, write 3) of the
 * osm_vfs_* functions below, which take the shadow.
 */
static const void *osm_file49_fops[30];
void osm_file_init(void);

static LIST_HEAD(osm_files);
static DEFINE_SPINLOCK(osm_files_lock);

/* shadow containing the address p (anywhere inside it), or NULL */
static struct osm_file *osm_file_find(const void *p)
{
	struct osm_file *sh, *found = NULL;
	unsigned long a = (unsigned long)p;

	spin_lock(&osm_files_lock);
	list_for_each_entry(sh, &osm_files, node) {
		if (a >= (unsigned long)sh && a < (unsigned long)(sh + 1)) {
			found = sh;
			break;
		}
	}
	spin_unlock(&osm_files_lock);
	return found;
}

static struct file *osm_file_real(void *file49, struct osm_file **shp)
{
	struct osm_file *sh = osm_file_find(file49);

	*shp = sh;
	return sh ? sh->real : file49;
}

struct file *osm_filp_open(const char *name, int flags, umode_t mode)
{
	struct file *real;
	struct osm_file *sh;

	real = filp_open(name, flags, mode);
	if (IS_ERR(real))
		return real;

	sh = kzalloc(sizeof(*sh), GFP_KERNEL);
	if (!sh) {
		filp_close(real, NULL);
		return ERR_PTR(-ENOMEM);
	}
	sh->magic = OSM_FILE_MAGIC;
	sh->real = real;
	sh->mnt = real->f_path.mnt;
	sh->dentry = real->f_path.dentry;
	sh->f_pos = real->f_pos;
	sh->f_op = osm_file49_fops;
	sh->f_flags = real->f_flags;
	sh->f_mode = real->f_mode;

	spin_lock(&osm_files_lock);
	list_add(&sh->node, &osm_files);
	spin_unlock(&osm_files_lock);
	return (struct file *)sh;
}
EXPORT_SYMBOL(osm_filp_open);

int osm_filp_close(struct file *file49, fl_owner_t id)
{
	struct osm_file *sh = osm_file_find(file49);
	struct file *real = file49;

	if (sh) {
		spin_lock(&osm_files_lock);
		list_del(&sh->node);
		spin_unlock(&osm_files_lock);
		real = sh->real;
		kfree(sh);
	}
	return filp_close(real, id);
}
EXPORT_SYMBOL(osm_filp_close);

loff_t osm_vfs_llseek(struct file *file49, loff_t off, int whence)
{
	struct osm_file *sh;
	struct file *real = osm_file_real(file49, &sh);
	loff_t ret;

	if (sh)
		real->f_pos = sh->f_pos;
	ret = vfs_llseek(real, off, whence);
	if (sh && ret >= 0)
		sh->f_pos = real->f_pos;
	return ret;
}
EXPORT_SYMBOL(osm_vfs_llseek);

/*
 * vfs_read/vfs_write: with the 4.9 KERNEL_DS set by the vendor code the
 * buffer is a kernel one, otherwise it is bounced to/from the user buffer.
 * When pos is the f_pos of a shadow, the real file's f_pos follows it.
 */
ssize_t osm_vfs_read(struct file *file49, char *buf, size_t count, loff_t *pos)
{
	struct osm_file *sh;
	struct file *real = osm_file_real(file49, &sh);
	ssize_t ret;
	void *kbuf;

	if (osm_kernel_ds()) {
		ret = kernel_read(real, buf, count, pos);
	} else {
		kbuf = kvmalloc(count, GFP_KERNEL);
		if (!kbuf)
			return -ENOMEM;
		ret = kernel_read(real, kbuf, count, pos);
		if (ret > 0 && copy_to_user((void __user *)buf, kbuf, ret))
			ret = -EFAULT;
		kvfree(kbuf);
	}
	if (sh && pos == &sh->f_pos)
		real->f_pos = sh->f_pos;
	return ret;
}
EXPORT_SYMBOL(osm_vfs_read);

ssize_t osm_vfs_write(struct file *file49, const char *buf, size_t count, loff_t *pos)
{
	struct osm_file *sh;
	struct file *real = osm_file_real(file49, &sh);
	ssize_t ret;
	void *kbuf;

	if (osm_kernel_ds()) {
		ret = kernel_write(real, buf, count, pos);
	} else {
		kbuf = memdup_user((const void __user *)buf, count);
		if (IS_ERR(kbuf))
			return PTR_ERR(kbuf);
		ret = kernel_write(real, kbuf, count, pos);
		kfree(kbuf);
	}
	if (sh && pos == &sh->f_pos)
		real->f_pos = sh->f_pos;
	return ret;
}
EXPORT_SYMBOL(osm_vfs_write);

/* 4.9 struct kstat, 32-bit ARM */
#define KSTAT49_INO		0	/* u64 */
#define KSTAT49_DEV		8
#define KSTAT49_MODE		12	/* u16 */
#define KSTAT49_NLINK		16
#define KSTAT49_UID		20
#define KSTAT49_GID		24
#define KSTAT49_RDEV		28
#define KSTAT49_SIZE		32	/* u64 */
#define KSTAT49_ATIME		40	/* struct timespec: s32 sec, s32 nsec */
#define KSTAT49_MTIME		48
#define KSTAT49_CTIME		56
#define KSTAT49_BLKSIZE		64
#define KSTAT49_BLOCKS		72	/* u64 */
#define KSTAT49_SIZEOF		80

static void kstat49_put(u8 *k, unsigned int off, const void *v, size_t len)
{
	memcpy(k + off, v, len);
}

static void kstat49_time(u8 *k, unsigned int off, const struct timespec64 *ts)
{
	s32 t[2] = { (s32)ts->tv_sec, (s32)ts->tv_nsec };

	kstat49_put(k, off, t, sizeof(t));
}

int osm_vfs_getattr(const struct path *path49, void *kstat49)
{
	struct osm_file *sh = osm_file_find(path49);
	const struct path *path = path49;
	struct path tmp;
	struct kstat st;
	u8 *k = kstat49;
	int ret;

	if (sh) {
		tmp = sh->real->f_path;
		path = &tmp;
	}
	ret = vfs_getattr(path, &st, STATX_BASIC_STATS, AT_STATX_SYNC_AS_STAT);
	if (ret)
		return ret;

	memset(k, 0, KSTAT49_SIZEOF);
	kstat49_put(k, KSTAT49_INO, &st.ino, 8);
	kstat49_put(k, KSTAT49_DEV, &st.dev, 4);
	kstat49_put(k, KSTAT49_MODE, &st.mode, 2);
	kstat49_put(k, KSTAT49_NLINK, &st.nlink, 4);
	{
		u32 uid = __kuid_val(st.uid), gid = __kgid_val(st.gid);

		kstat49_put(k, KSTAT49_UID, &uid, 4);
		kstat49_put(k, KSTAT49_GID, &gid, 4);
	}
	kstat49_put(k, KSTAT49_RDEV, &st.rdev, 4);
	kstat49_put(k, KSTAT49_SIZE, &st.size, 8);
	kstat49_time(k, KSTAT49_ATIME, &st.atime);
	kstat49_time(k, KSTAT49_MTIME, &st.mtime);
	kstat49_time(k, KSTAT49_CTIME, &st.ctime);
	{
		unsigned long blksize = st.blksize;

		kstat49_put(k, KSTAT49_BLKSIZE, &blksize, 4);
	}
	kstat49_put(k, KSTAT49_BLOCKS, &st.blocks, 8);
	return 0;
}
EXPORT_SYMBOL(osm_vfs_getattr);

void osm_file_init(void)
{
	osm_file49_fops[1] = osm_vfs_llseek;
	osm_file49_fops[2] = osm_vfs_read;
	osm_file49_fops[3] = osm_vfs_write;
}
