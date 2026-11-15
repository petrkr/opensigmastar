// SPDX-License-Identifier: GPL-2.0
/*
 * CamFs* of the SigmaStar BSP cam_fs_wrapper (Linux kernel part), on top of
 * the current file API. The handle is the real struct file, opaque to the
 * callers. Buffers are kernel ones, as with the BSP set_fs(get_ds()).
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/err.h>

typedef void *CamFsFd;

enum {
	CAM_FS_OK = 0,
	CAM_FS_FAIL = -1,
};

int CamFsOpen(CamFsFd *ptFd, const char *szPath, u32 nFlag, u32 nMode)
{
	struct file *fp = filp_open(szPath, nFlag, nMode);

	if (IS_ERR(fp)) {
		*ptFd = NULL;
		return CAM_FS_FAIL;
	}
	*ptFd = fp;
	return CAM_FS_OK;
}
EXPORT_SYMBOL(CamFsOpen);

int CamFsClose(CamFsFd tFd)
{
	struct file *fp = tFd;

	if (!fp)
		return CAM_FS_FAIL;
	return filp_close(fp, NULL) ? CAM_FS_FAIL : CAM_FS_OK;
}
EXPORT_SYMBOL(CamFsClose);

s32 CamFsRead(CamFsFd tFd, void *pBuf, u32 nCount)
{
	struct file *fp = tFd;
	loff_t pos;
	ssize_t ret;

	if (!fp)
		return -1;
	pos = fp->f_pos;
	ret = kernel_read(fp, pBuf, nCount, &pos);
	fp->f_pos = pos;
	return ret;
}
EXPORT_SYMBOL(CamFsRead);

s32 CamFsWrite(CamFsFd tFd, const void *pBuf, u32 nCount)
{
	struct file *fp = tFd;
	loff_t pos;
	ssize_t ret;

	if (!fp)
		return -1;
	pos = fp->f_pos;
	ret = kernel_write(fp, pBuf, nCount, &pos);
	fp->f_pos = pos;
	return ret;
}
EXPORT_SYMBOL(CamFsWrite);

s32 CamFsSeek(CamFsFd tFd, u32 nOffset, u32 nWhence)
{
	struct file *fp = tFd;

	if (!fp)
		return -1;
	return vfs_llseek(fp, nOffset, nWhence);
}
EXPORT_SYMBOL(CamFsSeek);
