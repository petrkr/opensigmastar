/*
* cam_os_wrapper.c - Sigmastar
*
* Copyright (C) 2018 Sigmastar Technology Corp.
*
* Author: giggs.huang <giggs.huang@sigmastar.com.tw>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/


///////////////////////////////////////////////////////////////////////////////
/// @file      cam_os_wrapper.c
/// @brief     Cam OS Wrapper Source File for Linux Kernel Space
///
/// opensigmastar: Linux kernel part of the SigmaStar BSP cam_os_wrapper
/// v1.0.35 (OpenIPC/linux, sigmastar-infinity6e), ported to the current
/// kernel. Callbacks into vendor code (threads, timers, interrupts) go
/// through osm_enter()/osm_leave().
///////////////////////////////////////////////////////////////////////////////

#define CAM_OS_LINUX_KERNEL

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/scatterlist.h>
#include <linux/idr.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <linux/uaccess.h>
#include <linux/timekeeping.h>
#include <asm/page.h>
#include "osm_compat.h"
#include "osm_sstar.h"
#include "osm_riu.h"
#include "cam_os_wrapper.h"
#include "cam_os_util.h"
#include "cam_os_util_list.h"
#include "cam_os_util_bitmap.h"

#define OS_NAME "LINUX KERNEL"

#define CAM_OS_THREAD_STACKSIZE_DEFAULT         8192
#define KMALLOC_THRESHOLD_SIZE                  (PAGE_SIZE >> 1)
#define LOG_MAX_TRACE_LEN                       256
#define LINUX_KERNEL_MAX_IRQ                    256

typedef struct
{
    u32 nInited;
    struct mutex tMutex;
} CamOsMutexLK_t, *pCamOsMutexLK;

typedef struct
{
    u32 nInited;
    struct semaphore tSem;
} CamOsTsemLK_t, *pCamOsTsemLK;

typedef struct
{
    u32 nInited;
    struct rw_semaphore tRwsem;
} CamOsRwsemLK_t, *pCamOsRwsemLK;

typedef struct
{
    u32 nInited;
    u8 tCompletion[28];     // 4.9 struct completion: mhal polls on its wait queue
} CamOsTcondLK_t, *pCamOsTcondLK;

typedef struct
{
    u32 nInited;
    spinlock_t tLock;
    unsigned long nFlags;
} CamOsSpinlockLK_t, *pCamOsSpinlockLK;

typedef struct
{
    struct timer_list tTimerID;
    void (*pfnFunc)(unsigned long nDataAddr);
    unsigned long nData;
} CamOsTimerLK_t, *pCamOsTimerLK;

typedef struct
{
    struct kmem_cache *ptKmemCache;
} CamOsMemCacheLK_t, *pCamOsMemCacheLK;

typedef struct
{
    struct idr tIdr;
} CamOsIdrLK_t, *pCamOsIdrLK;

typedef s32 CamOsThreadEntry_t(void *);

static CamOsIrqHandler CamOsIrqHandlerList[LINUX_KERNEL_MAX_IRQ] = {0};

static irqreturn_t CamOsIrqCommonHandler(int nIrq, void *pDevId)
{
    struct osm_entry e;

    if (nIrq < LINUX_KERNEL_MAX_IRQ && CamOsIrqHandlerList[nIrq] != NULL)
    {
        osm_enter(&e);
        (CamOsIrqHandlerList[nIrq])(nIrq, pDevId);
        osm_leave(&e);
    }

    return IRQ_HANDLED;
}

static DEFINE_MUTEX(_gtSelfInitLock);
static DEFINE_MUTEX(_gtMemLock);

_Static_assert(sizeof(CamOsMutex_t) >= sizeof(struct mutex) + 4, "CamOsMutex_t size define not enough! %d");
_Static_assert(sizeof(CamOsTsem_t) >= sizeof(CamOsTsemLK_t), "CamOsTsem_t size define not enough!");
_Static_assert(sizeof(CamOsRwsem_t) >= sizeof(CamOsRwsemLK_t), "CamOsRwsem_t size define not enough!");
_Static_assert(sizeof(CamOsTcond_t) >= sizeof(CamOsTcondLK_t), "CamOsTcond_t size define not enough!");
_Static_assert(sizeof(CamOsSpinlock_t) >= sizeof(CamOsSpinlockLK_t), "CamOsSpinlock_t size define not enough!");
_Static_assert(sizeof(CamOsTimespec_t) == 8, "CamOsTimespec_t size define error!");
_Static_assert(sizeof(CamOsTimer_t) >= sizeof(CamOsTimerLK_t), "CamOsTimer_t size define not enough!");
_Static_assert(sizeof(CamOsMemCache_t) >= sizeof(CamOsMemCacheLK_t), "CamOsMemCache_t size define not enough!");
_Static_assert(sizeof(CamOsAtomic_t) == sizeof(atomic_t), "CamOsAtomic_t size define not enough!");
_Static_assert(sizeof(CamOsIdr_t) >= sizeof(CamOsIdrLK_t), "CamOsIdr_t size define not enough!");


#define ASSIGN_POINTER_VALUE(a, b) if((a))*(a)=(b)

#define INIT_MAGIC_NUM          0x55AA5AA5

#define CAM_OS_MAX_LIST_LENGTH_BITS 20

#define CAM_OS_WARN_TRACE_LR
#ifdef CAM_OS_WARN_TRACE_LR
#define CAM_OS_WARN(x)      CamOsPrintf("%s "x", LR:0x%08X\n", __FUNCTION__, __builtin_return_address(0))
#else
#define CAM_OS_WARN(x)      CamOsPrintf("%s "x"\n", __FUNCTION__)
#endif

typedef struct MemoryList_t
{
    struct CamOsListHead_t tList;
    void *pPhysPtr;
    void *pVirtPtr;
    void *pMemifoPtr;
    char *szName;
    u32  nSize;
} MemoryList_t;

static MemoryList_t _gtMemList;

static s32 _gnDmemDbgListInited = 0;

char *CamOsVersion(void)
{
    return CAM_OS_WRAPPER_VERSION;
}

void CamOsPrintf(const char *szFmt, ...)
{
#if defined(CONFIG_MS_MSYS_LOG)
    va_list tArgs;
    unsigned int u32MsgLen = 0;
    char szLogStr[LOG_MAX_TRACE_LEN]={'a'};
    va_start(tArgs, szFmt);
    u32MsgLen = vsnprintf(szLogStr, LOG_MAX_TRACE_LEN, szFmt, tArgs);
    va_end(tArgs);
    if (u32MsgLen >= LOG_MAX_TRACE_LEN)
    {
        szLogStr[LOG_MAX_TRACE_LEN-1] = '\0';  /* even the 'vsnprintf' commond will do it */
        szLogStr[LOG_MAX_TRACE_LEN-2] = '\n';
        szLogStr[LOG_MAX_TRACE_LEN-3] = '.';
        szLogStr[LOG_MAX_TRACE_LEN-4] = '.';
        szLogStr[LOG_MAX_TRACE_LEN-5] = '.';
    }
    msys_prints(szLogStr, u32MsgLen);
#else
    va_list tArgs;
    va_start(tArgs, szFmt);
    vprintk(szFmt, tArgs);
    va_end(tArgs);
#endif
}

void CamOsPrintString(const char *szStr)
{
#if defined(CONFIG_MS_MSYS_LOG)
    msys_prints(szStr, strlen(szStr));
#else
    printk(szStr);
#endif
}


s32 CamOsScanf(const char *szFmt, ...)
{
    return 0;
}

s32 CamOsGetChar(void)
{
    return 0;
}

s32 CamOsSnprintf(char *szBuf, u32 nSize, const char *szFmt, ...)
{
    va_list tArgs;
    s32 i;

    va_start(tArgs, szFmt);
    i = vsnprintf(szBuf, nSize, szFmt, tArgs);
    va_end(tArgs);

    return i;
}

void CamOsHexdump(char *szBuf, u32 nSize)
{
    int i, j;
    int cx = 0;
    char szLine[80] = {0};

    CamOsPrintf("\nOffset(h)  00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F\n"
                "-----------------------------------------------------------\n");

    if ((u32)szBuf % 16)
    {
        cx = 0;
        cx += snprintf(szLine + cx, sizeof(szLine) - cx, "%08X  ", (u32)szBuf & 0xFFFFFFF0);

        for (i = 0; i < (u32)szBuf % 16; i++)
        {
            cx += snprintf(szLine + cx, sizeof(szLine) - cx, "   ");
            szLine[i + 62] = ' ';
            if (i % 8 == 0)
            {
                cx += snprintf(szLine + cx, sizeof(szLine) - cx, " ");
            }
        }
    }

    for (i = 0; i < nSize; i++)
    {
        if ((i + (u32)szBuf) % 16 == 0)
        {
            cx = 0;
            cx += snprintf(szLine + cx, sizeof(szLine) - cx, "%08X  ", (u32)szBuf + i);
        }
        if ((i + (u32)szBuf) % 8 == 0)
        {
            cx += snprintf(szLine + cx, sizeof(szLine) - cx, " ");
        }

        cx += snprintf(szLine + cx, sizeof(szLine) - cx, "%02X ", szBuf[i]);

        if (((unsigned char*)szBuf)[i] >= ' ' && ((unsigned char*)szBuf)[i] <= '~')
        {
            szLine[(i + (u32)szBuf) % 16 + 62] = ((unsigned char*)szBuf)[i];
        }
        else
        {
            szLine[(i + (u32)szBuf) % 16 + 62] = '.';
        }

        if ((i + (u32)szBuf) % 16 == 15)
        {
            szLine[59] = ' ';
            szLine[60] = ' ';
            szLine[61] = '|';
            szLine[78] = '|';
            szLine[79] = 0;
            CamOsPrintf("%s\n", szLine);
        }
        else if (i == nSize-1)
        {
            for (j = ((i + (u32)szBuf) + 1) % 16; j < 16; j++)
            {
                cx += snprintf(szLine + cx, sizeof(szLine) - cx, "   ");
                szLine[j + 62] = ' ';
            }
            if (((i + (u32)szBuf) + 1) % 16 <= 8)
            {
                cx += snprintf(szLine + cx, sizeof(szLine) - cx, " ");
            }
            szLine[59] = ' ';
            szLine[60] = ' ';
            szLine[61] = '|';
            szLine[78] = '|';
            szLine[79] = 0;
            CamOsPrintf("%s\n", szLine);
        }
    }
    CamOsPrintf("\n");
}

void CamOsMsSleep(u32 nMsec)
{
    msleep(nMsec);
}

void CamOsUsSleep(u32 nUsec)
{
    usleep_range(nUsec, nUsec + 10);
}

void CamOsMsDelay(u32 nMsec)
{
    mdelay(nMsec);
}

void CamOsUsDelay(u32 nUsec)
{
    udelay(nUsec);
}


void CamOsGetTimeOfDay(CamOsTimespec_t *ptRes)
{
    struct timespec64 tTs;
    if(ptRes)
    {
        ktime_get_real_ts64(&tTs);
        ptRes->nSec = tTs.tv_sec;
        ptRes->nNanoSec = tTs.tv_nsec / 1000 * 1000;
    }
}

void CamOsSetTimeOfDay(const CamOsTimespec_t *ptRes)
{
    struct timespec64 tTs;
    if(ptRes)
    {
        tTs.tv_sec = ptRes->nSec;
        tTs.tv_nsec = ptRes->nNanoSec;
        do_settimeofday64(&tTs);
    }
}

void CamOsGetMonotonicTime(CamOsTimespec_t *ptRes)
{
    struct timespec64 tTs;

    ktime_get_raw_ts64(&tTs);
    ptRes->nSec = tTs.tv_sec;
    ptRes->nNanoSec = tTs.tv_nsec;
}

s64 CamOsTimeDiff(CamOsTimespec_t *ptStart, CamOsTimespec_t *ptEnd, CamOsTimeDiffUnit_e eUnit)
{
    if (ptStart && ptEnd)
    {
        switch (eUnit)
        {
        case CAM_OS_TIME_DIFF_SEC:
            return (s64)ptEnd->nSec - ptStart->nSec;
        case CAM_OS_TIME_DIFF_MS:
            return ((s64)ptEnd->nSec - ptStart->nSec)*1000 + div_s64((s64)ptEnd->nNanoSec - ptStart->nNanoSec, 1000000);
        case CAM_OS_TIME_DIFF_US:
            return ((s64)ptEnd->nSec - ptStart->nSec)*1000000 + div_s64((s64)ptEnd->nNanoSec - ptStart->nNanoSec, 1000);
        case CAM_OS_TIME_DIFF_NS:
            return ((s64)ptEnd->nSec - ptStart->nSec)*1000000000 + ((s64)ptEnd->nNanoSec - ptStart->nNanoSec);
        default:
            return 0;
        }
    }
    else
        return 0;
}


typedef struct
{
    void *(*pfnStartRoutine)(void *);
    void *pArg;
} CamOsThreadStart_t;

static int CamOsThreadEntry(void *pData)
{
    CamOsThreadStart_t tStart = *(CamOsThreadStart_t *)pData;
    struct osm_entry e;
    int nRet;

    kfree(pData);
    osm_enter(&e);
    nRet = (int)(uintptr_t)tStart.pfnStartRoutine(tStart.pArg);
    osm_leave(&e);
    return nRet;
}

// sched_setscheduler() is not exported any more: RT threads get the
// default SCHED_FIFO priority
static void CamOsThreadSetRt(struct task_struct *tpThreadHandle)
{
    sched_set_fifo(tpThreadHandle);
}

CamOsRet_e CamOsThreadCreate(CamOsThread *ptThread,
                             CamOsThreadAttrb_t *ptAttrb,
                             void *(*pfnStartRoutine)(void *),
                             void *pArg)
{
    CamOsRet_e eRet = CAM_OS_OK;
    struct task_struct *tpThreadHandle;
    CamOsThreadStart_t *ptStart;
    s32 nNiceVal = 0;

    do
    {
        ptStart = kmalloc(sizeof(*ptStart), GFP_KERNEL);
        if (!ptStart)
        {
            eRet = CAM_OS_FAIL;
            *ptThread = NULL;
            break;
        }
        ptStart->pfnStartRoutine = pfnStartRoutine;
        ptStart->pArg = pArg;
        tpThreadHandle = kthread_run(CamOsThreadEntry, ptStart, "%s", (ptAttrb && ptAttrb->szName)? ptAttrb->szName : "CAMOS");
        if (IS_ERR(tpThreadHandle))
        {
            kfree(ptStart);
            eRet = CAM_OS_FAIL;
            *ptThread = NULL;
            break;
        }
        if(ptAttrb != NULL)
        {
            if((ptAttrb->nPriority > 0) && (ptAttrb->nPriority <= 70))
            {
                if (ptAttrb->nPriority <= 50)
                {
                    nNiceVal = 19 - ptAttrb->nPriority * 19 / 50;
                }
                else
                {
                    nNiceVal = 50 - ptAttrb->nPriority;
                }

                set_user_nice(tpThreadHandle, nNiceVal);
            }
            else if((ptAttrb->nPriority > 70) && (ptAttrb->nPriority <= 100))
            {
                CamOsThreadSetRt(tpThreadHandle);
            }
        }

        *ptThread = (CamOsThread *)tpThreadHandle;
    }
    while(0);

    // coverity[leaked_storage]
    return eRet;
}

CamOsRet_e CamOsThreadChangePriority(CamOsThread tThread, u32 nPriority)
{
    CamOsRet_e eRet = CAM_OS_OK;
    struct task_struct *tpThreadHandle = (struct task_struct *)tThread;
    s32 nNiceVal = 0;

    if(tpThreadHandle && (nPriority >= 0) && (nPriority <= 70))
    {
        nPriority = nPriority? nPriority : 50;
        if (nPriority <= 50)
        {
            nNiceVal = 19 - nPriority * 19 / 50;
        }
        else
        {
            nNiceVal = 50 - nPriority;
        }

        sched_set_normal(tpThreadHandle, nNiceVal);
    }
    else if(tpThreadHandle && (nPriority > 70) && (nPriority <= 100))
    {
        CamOsThreadSetRt(tpThreadHandle);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsThreadSchedule(u8 bInterruptible, u32 nMsec)
{
    CamOsRet_e eRet = CAM_OS_OK;
    if(nMsec == CAM_OS_MAX_TIMEOUT)
    {
        if(bInterruptible)
            set_current_state(TASK_INTERRUPTIBLE);
        else
            set_current_state(TASK_UNINTERRUPTIBLE);

        schedule();
    }
    else if(nMsec > 0)
    {
        if(bInterruptible)
        {
            if(0 == schedule_timeout(msecs_to_jiffies(nMsec) + 1))
                eRet = CAM_OS_TIMEOUT;
        }
        else
        {
            schedule_timeout_uninterruptible(msecs_to_jiffies(nMsec) + 1);
        }
    }
    return eRet;
}

CamOsRet_e CamOsThreadWakeUp(CamOsThread tThread)
{
    CamOsRet_e eRet = CAM_OS_OK;
    struct task_struct *tpThreadHandle = (struct task_struct *)tThread;
    if(tpThreadHandle)
    {
        wake_up_process(tpThreadHandle);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsThreadJoin(CamOsThread tThread)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CAM_OS_WARN("not support in "OS_NAME);
    eRet = CAM_OS_FAIL;
    return eRet;
}

CamOsRet_e CamOsThreadStop(CamOsThread tThread)
{
    CamOsRet_e eRet = CAM_OS_OK;
    struct task_struct *tpThreadHandle = (struct task_struct *)tThread;
    s32 nErr;
    if(tpThreadHandle)
    {
        if(0 != (nErr = kthread_stop(tpThreadHandle)))
        {
            CAM_OS_WARN("stop fail");
            CamOsPrintf("%s Err=%d\n", __FUNCTION__, nErr);
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsThreadShouldStop(void)
{
    CamOsRet_e eRet = CAM_OS_OK;
    if(kthread_should_stop())
    {
        eRet = CAM_OS_OK;
    }
    else
    {
        eRet = CAM_OS_FAIL;
    }
    return eRet;
}

CamOsRet_e CamOsThreadSetName(CamOsThread tThread, const char *szName)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CAM_OS_WARN("not support in "OS_NAME);
    eRet = CAM_OS_FAIL;

    return eRet;
}

CamOsRet_e CamOsThreadGetName(CamOsThread tThread, char *szName, u32 nLen)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CAM_OS_WARN("not support in "OS_NAME);
    eRet = CAM_OS_FAIL;
    return eRet;
}

u32 CamOsThreadGetID()
{
    return current->tgid;
}

CamOsRet_e CamOsMutexInit(CamOsMutex_t *ptMutex)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsMutexLK_t *ptHandle = (CamOsMutexLK_t *)ptMutex;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            mutex_lock(&_gtSelfInitLock);
            if(ptHandle->nInited != INIT_MAGIC_NUM)
            {
                mutex_init(&ptHandle->tMutex);
                ptHandle->nInited = INIT_MAGIC_NUM;
            }
            mutex_unlock(&_gtSelfInitLock);
        }
        else
        {
            CAM_OS_WARN("already inited");
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsMutexDestroy(CamOsMutex_t *ptMutex)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsMutexLK_t *ptHandle = (CamOsMutexLK_t *)ptMutex;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
            ptHandle->nInited = 0;
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsMutexLock(CamOsMutex_t *ptMutex)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsMutexLK_t *ptHandle = (CamOsMutexLK_t *)ptMutex;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsMutexInit(ptMutex);
        }

        mutex_lock(&ptHandle->tMutex);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsMutexTryLock(CamOsMutex_t *ptMutex)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsMutexLK_t *ptHandle = (CamOsMutexLK_t *)ptMutex;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsMutexInit(ptMutex);
        }

        if(0 != mutex_trylock(&ptHandle->tMutex))
            eRet = CAM_OS_RESOURCE_BUSY;
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsMutexUnlock(CamOsMutex_t *ptMutex)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsMutexLK_t *ptHandle = (CamOsMutexLK_t *)ptMutex;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsMutexInit(ptMutex);
        }

        mutex_unlock(&ptHandle->tMutex);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTsemInit(CamOsTsem_t *ptTsem, u32 nVal)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            mutex_lock(&_gtSelfInitLock);

            sema_init(&ptHandle->tSem, nVal);
            ptHandle->nInited = INIT_MAGIC_NUM;

            mutex_unlock(&_gtSelfInitLock);
        }
        else
        {
            CAM_OS_WARN("already inited");
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTsemDeinit(CamOsTsem_t *ptTsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
            ptHandle->nInited = 0;
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

void CamOsTsemUp(CamOsTsem_t *ptTsem)
{
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            up(&ptHandle->tSem);
    }
}

void CamOsTsemDown(CamOsTsem_t *ptTsem)
{
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            down(&ptHandle->tSem);
    }
}

CamOsRet_e CamOsTsemDownInterruptible(CamOsTsem_t *ptTsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
        {
            if(-EINTR == down_interruptible(&ptHandle->tSem))
            {
                eRet = CAM_OS_INTERRUPTED;
            }
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTsemTimedDown(CamOsTsem_t *ptTsem, u32 nMsec)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else if(0 != down_timeout(&ptHandle->tSem, msecs_to_jiffies(nMsec)))
            eRet = CAM_OS_TIMEOUT;
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTsemTryDown(CamOsTsem_t *ptTsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTsemLK_t *ptHandle = (CamOsTsemLK_t *)ptTsem;
    s32 nErr;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else if(0 != (nErr = down_trylock(&ptHandle->tSem)))
        {
            CAM_OS_WARN("down fail");
            eRet = CAM_OS_RESOURCE_BUSY;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsRwsemInit(CamOsRwsem_t *ptRwsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            mutex_lock(&_gtSelfInitLock);

            init_rwsem(&ptHandle->tRwsem);
            ptHandle->nInited = INIT_MAGIC_NUM;

            mutex_unlock(&_gtSelfInitLock);
        }
        else
        {
            CAM_OS_WARN("already inited");
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsRwsemDeinit(CamOsRwsem_t *ptRwsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
            ptHandle->nInited = 0;
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

void CamOsRwsemUpRead(CamOsRwsem_t *ptRwsem)
{
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            up_read(&ptHandle->tRwsem);
    }
}

void CamOsRwsemUpWrite(CamOsRwsem_t *ptRwsem)
{
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            up_write(&ptHandle->tRwsem);
    }
}

void CamOsRwsemDownRead(CamOsRwsem_t *ptRwsem)
{
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            down_read(&ptHandle->tRwsem);
    }
}

void CamOsRwsemDownWrite(CamOsRwsem_t *ptRwsem)
{
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            down_write(&ptHandle->tRwsem);
    }
}

CamOsRet_e CamOsRwsemTryDownRead(CamOsRwsem_t *ptRwsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    s32 nErr;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else if(1 != (nErr = down_read_trylock(&ptHandle->tRwsem)))
        {
            eRet = CAM_OS_RESOURCE_BUSY;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsRwsemTryDownWrite(CamOsRwsem_t *ptRwsem)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsRwsemLK_t *ptHandle = (CamOsRwsemLK_t *)ptRwsem;
    s32 nErr;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else if(1 != (nErr = down_write_trylock(&ptHandle->tRwsem)))
        {
            eRet = CAM_OS_RESOURCE_BUSY;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTcondInit(CamOsTcond_t *ptTcond)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            mutex_lock(&_gtSelfInitLock);

            osm_completion49_init(ptHandle->tCompletion);
            ptHandle->nInited = INIT_MAGIC_NUM;

            mutex_unlock(&_gtSelfInitLock);
        }
        else
        {
            CAM_OS_WARN("already inited");
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTcondDeinit(CamOsTcond_t *ptTcond)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
            ptHandle->nInited = 0;
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

void CamOsTcondSignal(CamOsTcond_t *ptTcond)
{
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
            osm_completion49_complete(ptHandle->tCompletion, false);
    }
}

void CamOsTcondSignalAll(CamOsTcond_t *ptTcond)
{
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
        {
            osm_completion49_complete(ptHandle->tCompletion, true);
        }
    }
}

void CamOsTcondWait(CamOsTcond_t *ptTcond)
{
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;

    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
            CAM_OS_WARN("not inited");
        else
        {
            *(unsigned int *)ptHandle->tCompletion = 0;
            osm_completion49_wait(ptHandle->tCompletion, MAX_SCHEDULE_TIMEOUT, false);
        }
    }
}

CamOsRet_e CamOsTcondTimedWait(CamOsTcond_t *ptTcond, u32 nMsec)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
        {
            *(unsigned int *)ptHandle->tCompletion = 0;
            if(!osm_completion49_wait(ptHandle->tCompletion, msecs_to_jiffies(nMsec), false))
            {
                eRet = CAM_OS_TIMEOUT;
            }
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTcondWaitInterruptible(CamOsTcond_t *ptTcond)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
        {
            *(unsigned int *)ptHandle->tCompletion = 0;
            if (-ERESTARTSYS == osm_completion49_wait(ptHandle->tCompletion, MAX_SCHEDULE_TIMEOUT, true))
                eRet = CAM_OS_INTERRUPTED;
        }
    }
    return eRet;
}

CamOsRet_e CamOsTcondTimedWaitInterruptible(CamOsTcond_t *ptTcond, u32 nMsec)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    s32 nWaitRet;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
        {
            *(unsigned int *)ptHandle->tCompletion = 0;
            nWaitRet = osm_completion49_wait(ptHandle->tCompletion, msecs_to_jiffies(nMsec), true);
            if (nWaitRet == -ERESTARTSYS)
            {
                eRet = CAM_OS_INTERRUPTED;
            }
            else if (nWaitRet == 0)
            {
                eRet = CAM_OS_TIMEOUT;
            }
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsTcondWaitActive(CamOsTcond_t *ptTcond)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTcondLK_t *ptHandle = (CamOsTcondLK_t *)ptTcond;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CAM_OS_WARN("not inited");
            eRet = CAM_OS_FAIL;
        }
        else
        {
            if(!osm_completion49_done(ptHandle->tCompletion))
            {
                eRet = CAM_OS_FAIL;
            }
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsSpinInit(CamOsSpinlock_t *ptSpinlock)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsSpinlockLK_t *ptHandle = (CamOsSpinlockLK_t *)ptSpinlock;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            if (CamOsInInterrupt() != CAM_OS_OK)
            {
                mutex_lock(&_gtSelfInitLock);
                if(ptHandle->nInited != INIT_MAGIC_NUM)
                {
                    spin_lock_init(&ptHandle->tLock);
                    ptHandle->nInited = INIT_MAGIC_NUM;
                }
                mutex_unlock(&_gtSelfInitLock);
            }
            else
            {
                if(ptHandle->nInited != INIT_MAGIC_NUM)
                {
                    spin_lock_init(&ptHandle->tLock);
                    ptHandle->nInited = INIT_MAGIC_NUM;
                }
            }
        }
        else
        {
            CAM_OS_WARN("already inited");
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsSpinLock(CamOsSpinlock_t *ptSpinlock)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsSpinlockLK_t *ptHandle = (CamOsSpinlockLK_t *)ptSpinlock;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsSpinInit(ptSpinlock);
        }

        spin_lock(&ptHandle->tLock);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsSpinUnlock(CamOsSpinlock_t *ptSpinlock)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsSpinlockLK_t *ptHandle = (CamOsSpinlockLK_t *)ptSpinlock;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsSpinInit(ptSpinlock);
        }

        spin_unlock(&ptHandle->tLock);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsSpinLockIrqSave(CamOsSpinlock_t *ptSpinlock)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsSpinlockLK_t *ptHandle = (CamOsSpinlockLK_t *)ptSpinlock;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsSpinInit(ptSpinlock);
        }

        spin_lock_irqsave(&ptHandle->tLock, ptHandle->nFlags);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

CamOsRet_e CamOsSpinUnlockIrqRestore(CamOsSpinlock_t *ptSpinlock)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsSpinlockLK_t *ptHandle = (CamOsSpinlockLK_t *)ptSpinlock;
    if(ptHandle)
    {
        if(ptHandle->nInited != INIT_MAGIC_NUM)
        {
            CamOsSpinInit(ptSpinlock);
        }

        spin_unlock_irqrestore(&ptHandle->tLock, ptHandle->nFlags);
    }
    else
    {
        eRet = CAM_OS_PARAM_ERR;
    }
    return eRet;
}

void* CamOsMemAlloc(u32 nSize)
{
    if (nSize > KMALLOC_THRESHOLD_SIZE)
        return vzalloc(nSize);
    else
        return kzalloc(nSize, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
}

void* CamOsMemAllocAtomic(u32 nSize)
{
    if (nSize > KMALLOC_THRESHOLD_SIZE)
        return NULL;
    else
        return kzalloc(nSize, GFP_ATOMIC);
}

void* CamOsMemCalloc(u32 nNum, u32 nSize)
{
    if ((nNum * nSize) > KMALLOC_THRESHOLD_SIZE)
        return vzalloc(nNum * nSize);
    else
        return kzalloc(nNum * nSize, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
}

void* CamOsMemCallocAtomic(u32 nNum, u32 nSize)
{
    if ((nNum * nSize) > KMALLOC_THRESHOLD_SIZE)
        return NULL;
    else
        return kzalloc(nNum * nSize, GFP_ATOMIC);
}

void* CamOsMemRealloc(void* pPtr, u32 nSize)
{
    void *pAddr;

    if (nSize > KMALLOC_THRESHOLD_SIZE)
        pAddr = vzalloc(nSize);
    else
        pAddr = kzalloc(nSize, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

    if(pPtr && pAddr)
    {
        memcpy(pAddr, pPtr, nSize);
        kvfree(pPtr);
    }
    return pAddr;
}

void* CamOsMemReallocAtomic(void* pPtr, u32 nSize)
{
    void *pAddr;

    if (nSize > KMALLOC_THRESHOLD_SIZE)
        pAddr = NULL;
    else
        pAddr = kzalloc(nSize, GFP_ATOMIC);

    if(pPtr && pAddr)
    {
        memcpy(pAddr, pPtr, nSize);
        kvfree(pPtr);
    }
    return pAddr;
}

void CamOsMemRelease(void* pPtr)
{
    if(pPtr)
    {
        kvfree(pPtr);
    }
}

void CamOsMemFlush(void* pPtr, u32 nSize)
{
    Chip_Flush_Cache_Range((unsigned long)pPtr, nSize);
}

void CamOsMemFlushExt(void* pVa, void* pPA, u32 nSize)
{
    Chip_Flush_Cache_Range_VA_PA((unsigned long)pVa, (unsigned long)pPA, nSize);
}

void CamOsMemInvalidate(void* pPtr, u32 nSize)
{
    Chip_Inv_Cache_Range((unsigned long)pPtr, nSize);
}

void CamOsMiuPipeFlush(void)
{
    Chip_Flush_MIU_Pipe();
}

static s32 _CheckDmemInfoListInited(void)
{
    mutex_lock(&_gtMemLock);
    if(!_gnDmemDbgListInited)
    {
        memset(&_gtMemList, 0, sizeof(MemoryList_t));
        CAM_OS_INIT_LIST_HEAD(&_gtMemList.tList);

        _gnDmemDbgListInited = 1;
    }
    mutex_unlock(&_gtMemLock);

    return 0;
}

// Porting from stdint.h for Linux user space

CamOsRet_e CamOsDirectMemAlloc(const char* szName,
                               u32 nSize,
                               void** ppVirtPtr,
                               void** ppPhysPtr,
                               void** ppMiuPtr)
{
    CamOsRet_e eRet = CAM_OS_OK;
    MSYS_DMEM_INFO *ptDmem = NULL;
    MemoryList_t* ptNewEntry = NULL;
    struct CamOsListHead_t *ptPos, *ptQ;
    MemoryList_t* ptTmp;

    ASSIGN_POINTER_VALUE(ppVirtPtr, NULL);
    ASSIGN_POINTER_VALUE(ppPhysPtr, NULL);
    ASSIGN_POINTER_VALUE(ppMiuPtr, NULL);

    do
    {
        //Check request name to avoid allocate same dmem address.
        _CheckDmemInfoListInited();

        mutex_lock(&_gtMemLock);
        CAM_OS_LIST_FOR_EACH_SAFE(ptPos, ptQ, &_gtMemList.tList)
        {
            ptTmp = CAM_OS_LIST_ENTRY(ptPos, MemoryList_t, tList);

            if(ptTmp->pPhysPtr && ptTmp->pVirtPtr && ptTmp->szName && 0 == strcmp(szName, ptTmp->szName))
            {
                printk(KERN_WARNING "%s name conflict: %s\n", __FUNCTION__, szName);
                eRet = CAM_OS_PARAM_ERR;
            }
        }
        mutex_unlock(&_gtMemLock);
        if(eRet == CAM_OS_PARAM_ERR)
        {
            ASSIGN_POINTER_VALUE(ppVirtPtr, NULL);
            ASSIGN_POINTER_VALUE(ppPhysPtr, NULL);
            ASSIGN_POINTER_VALUE(ppMiuPtr, NULL);
            break;
        }

        if(0 == (ptDmem = (MSYS_DMEM_INFO *)kzalloc(sizeof(MSYS_DMEM_INFO), in_interrupt() ? GFP_ATOMIC : GFP_KERNEL)))
        {
            printk(KERN_WARNING "%s alloc DMEM INFO fail\n", __FUNCTION__);
            eRet = CAM_OS_FAIL;
            break;
        }

        snprintf(ptDmem->name, 15, "%s", szName);
        ptDmem->length = nSize;

        if(0 != msys_request_dmem(ptDmem))
        {
            printk(KERN_WARNING "%s request dmem fail\n", __FUNCTION__);
            eRet = CAM_OS_FAIL;
            break;
        }

        ASSIGN_POINTER_VALUE(ppVirtPtr, (void *)(uintptr_t)ptDmem->kvirt);
        ASSIGN_POINTER_VALUE(ppMiuPtr, (void *)(uintptr_t)Chip_Phys_to_MIU(ptDmem->phys));
        ASSIGN_POINTER_VALUE(ppPhysPtr, (void *)(uintptr_t)ptDmem->phys);

        printk(KERN_INFO "%s <%s> physAddr<0x%08X> size<%d>\n",
               __FUNCTION__,
               szName,
               (u32)ptDmem->phys,
               (u32)ptDmem->length);

        mutex_lock(&_gtMemLock);
        if(0 == (ptNewEntry = (MemoryList_t*) kzalloc(sizeof(MemoryList_t), in_interrupt() ? GFP_ATOMIC : GFP_KERNEL)))
        {
            printk(KERN_WARNING "%s alloc entry fail\n", __FUNCTION__);
            eRet = CAM_OS_FAIL;
            break;
        }
        ptNewEntry->pPhysPtr = *ppPhysPtr;
        ptNewEntry->pVirtPtr = *ppVirtPtr;
        ptNewEntry->pMemifoPtr = (void *) ptDmem;
        if(0 == (ptNewEntry->szName = (char *)kzalloc(strlen(szName) + 1, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL)))
        {
            printk(KERN_WARNING "%s alloc entry name fail\n", __FUNCTION__);
            eRet = CAM_OS_FAIL;
            break;
        }
        memcpy(ptNewEntry->szName, szName, strlen(szName));
        ptNewEntry->nSize = ptDmem->length;
        CAM_OS_LIST_ADD_TAIL(&(ptNewEntry->tList), &_gtMemList.tList);
        mutex_unlock(&_gtMemLock);
    }
    while(0);

    if(eRet == CAM_OS_FAIL)
    {
        if(ptDmem)
        {
            if(ptDmem->phys)
            {
                msys_release_dmem(ptDmem);
            }
            kfree(ptDmem);
        }

        if(ptNewEntry)
        {
            if(ptNewEntry->szName)
            {
                kfree(ptNewEntry->szName);
            }
            kfree(ptNewEntry);
        }
    }
    return eRet;
}

CamOsRet_e CamOsDirectMemRelease(void* pPtr, u32 nSize)
{
    CamOsRet_e eRet = CAM_OS_OK;
    MSYS_DMEM_INFO *tpDmem = NULL;
    struct CamOsListHead_t *ptPos, *ptQ;
    MemoryList_t* ptTmp;

    if(pPtr)
    {
        do
        {
            _CheckDmemInfoListInited();

            mutex_lock(&_gtMemLock);
            CAM_OS_LIST_FOR_EACH_SAFE(ptPos, ptQ, &_gtMemList.tList)
            {
                ptTmp = CAM_OS_LIST_ENTRY(ptPos, MemoryList_t, tList);

                //printk("search tmp->ptr: %08X  %s\n", (u32)ptTmp->pVirtPtr, ptTmp->szName);

                if(ptTmp->pPhysPtr == pPtr || ptTmp->pVirtPtr == pPtr)
                {
                    tpDmem = ptTmp->pMemifoPtr;
                    //printk("search(2) pdmem->name: %s\n", tpDmem->name);
                    break;
                }
            }
            mutex_unlock(&_gtMemLock);
            if(tpDmem == NULL)
            {
                printk(KERN_WARNING "%s find node fail <0x%08X>\n", __FUNCTION__, (u32)ptTmp->pVirtPtr);
                eRet = CAM_OS_FAIL;
                break;
            }

            msys_release_dmem(tpDmem);

            if(tpDmem)
            {
                kfree(tpDmem);
                tpDmem = NULL;
            }
            mutex_lock(&_gtMemLock);
            CAM_OS_LIST_FOR_EACH_SAFE(ptPos, ptQ, &_gtMemList.tList)
            {
                ptTmp = CAM_OS_LIST_ENTRY(ptPos, MemoryList_t, tList);

                if(ptTmp->pPhysPtr == pPtr || ptTmp->pVirtPtr == pPtr)
                {
                    if(ptTmp->szName)
                        kfree(ptTmp->szName);
                    CAM_OS_LIST_DEL(ptPos);
                    kfree(ptTmp);
                }
            }
            mutex_unlock(&_gtMemLock);
        }
        while(0);
    }
    return eRet;
}

CamOsRet_e CamOsDirectMemFlush(void* pPtr)
{
    CamOsRet_e eRet = CAM_OS_OK;
    flush_cache_all();
    Chip_Flush_Memory();
    return eRet;
}

CamOsRet_e CamOsDirectMemStat(void)
{
    CamOsRet_e eRet = CAM_OS_OK;
    struct CamOsListHead_t *ptPos, *ptQ;
    MemoryList_t* ptTmp;

    _CheckDmemInfoListInited();

    mutex_lock(&_gtMemLock);
    CAM_OS_LIST_FOR_EACH_SAFE(ptPos, ptQ, &_gtMemList.tList)
    {
        ptTmp = CAM_OS_LIST_ENTRY(ptPos, MemoryList_t, tList);

        if(ptTmp->pVirtPtr)
        {
            printk(KERN_WARNING "%s allocated 0x%08X %s\n", __FUNCTION__, (u32)ptTmp->pVirtPtr, ptTmp->szName);
        }
    }
    mutex_unlock(&_gtMemLock);
    return eRet;
}

void* CamOsDirectMemPhysToMiu(void* pPtr)
{
    return (void *)(uintptr_t)Chip_Phys_to_MIU((uintptr_t)pPtr);
}

void* CamOsDirectMemMiuToPhys(void* pPtr)
{
    return (void *)(uintptr_t)Chip_MIU_to_Phys((uintptr_t)pPtr);
}

void* CamOsDirectMemPhysToVirt(void* pPtr)
{
    MSYS_DMEM_INFO dmem;
    u64 kptr = 0;
    if(0 == msys_find_dmem_by_phys((u64)(u32)pPtr, &dmem))
    {
        kptr = dmem.kvirt;
        kptr += ((u64)(u32)pPtr - dmem.phys);
    }
    else
    {
        CamOsPrintf("CamOs WARNING: PhysToVirt not found 0x%08X in msys\n", (u32)pPtr);
    }

    return (void*)(u32)kptr;
}

void* CamOsDirectMemVirtToPhys(void* pPtr)
{
    return (void *)virt_to_phys(pPtr);
}

void* CamOsPhyMemMap(void* pPhyPtr, u32 nSize, u8 bNonCache)
{
    unsigned long long nCpuBusAddr;
    void *pVirtPtr = NULL;
    int nRet, i, j, k;
    struct sg_table *pSgTable;
    struct scatterlist *pScatterList;
    int nPageCount = 0;
    struct page **ppPages;
    pgprot_t tPgProt;

    if(!nSize)
        return NULL;

    if(bNonCache)
        tPgProt = pgprot_writecombine(PAGE_KERNEL);
    else
        tPgProt = PAGE_KERNEL;

    pSgTable = kmalloc(sizeof(struct sg_table), in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
    if(!pSgTable)
    {
        CAM_OS_WARN("kmalloc fail");
        return NULL;
    }

    nRet = sg_alloc_table(pSgTable, 1, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

    if(unlikely(nRet))
    {
        CAM_OS_WARN("sg_alloc_table fail");
        kfree(pSgTable);
        return NULL;
    }

    if((u32)pPhyPtr & 0xF0000000)
    {
        nCpuBusAddr = (uintptr_t)Chip_Phys_to_MIU((uintptr_t)pPhyPtr);
    }
    else
    {
        CAM_OS_WARN("wrong addr");
        CamOsPrintf("%s input MIU addr 0x%08X\n", __FUNCTION__, (u32)pPhyPtr);
    }

    sg_set_page(pSgTable->sgl, pfn_to_page(__phys_to_pfn(nCpuBusAddr)), PAGE_ALIGN(nSize), 0);

    for_each_sg(pSgTable->sgl, pScatterList, pSgTable->nents, i)
    {
        nPageCount += pScatterList->length / PAGE_SIZE;
    }

    ppPages = vmalloc(sizeof(struct page*)*nPageCount);

    if(ppPages == NULL)
    {
        CAM_OS_WARN("vmalloc fail");
        sg_free_table(pSgTable);
        kfree(pSgTable);
        return NULL;
    }

    for_each_sg(pSgTable->sgl, pScatterList, pSgTable->nents, k)
    {
        nPageCount = PAGE_ALIGN(pScatterList->length) / PAGE_SIZE;
        for(j = 0; j < nPageCount; j++)
            ppPages[i + j] = sg_page(pScatterList) + j;
        i += nPageCount;
    }
    pVirtPtr = vmap(ppPages, i, VM_MAP, tPgProt);

    vfree(ppPages);
    sg_free_table(pSgTable);
    kfree(pSgTable);

    return pVirtPtr;
}

void CamOsPhyMemUnMap(void* pVirtPtr, u32 nSize)
{
    vunmap(pVirtPtr);
}

void* CamOsMemMap(void* pPhyPtr, u32 nSize, u8 bCache)
{
    unsigned long long nCpuBusAddr = 0;
    void *pVirtPtr = NULL;
    int nRet, i, j;
    struct sg_table *pSgTable;
    struct scatterlist *pScatterList;
    int nPageCount = 0;
    int nPageTotal = 0;
    struct page **ppPages;
    pgprot_t tPgProt;

    if(!nSize)
        return NULL;

    if(bCache)
        tPgProt = PAGE_KERNEL;
    else
        tPgProt = pgprot_writecombine(PAGE_KERNEL);

    pSgTable = kmalloc(sizeof(struct sg_table), in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
    if(!pSgTable)
    {
        CAM_OS_WARN("kmalloc fail");
        return NULL;
    }

    nRet = sg_alloc_table(pSgTable, 1, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

    if(unlikely(nRet))
    {
        CAM_OS_WARN("sg_alloc_table fail");
        kfree(pSgTable);
        return NULL;
    }

    nCpuBusAddr = (uintptr_t)pPhyPtr;
    // opensigmastar: callers pass MIU addresses too; the 4.9 vmap() did not
    // check pfn_valid() and mapped whatever page that was
    if (!pfn_valid(__phys_to_pfn(nCpuBusAddr)))
    {
        CamOsPrintf("%s: 0x%08llX is not RAM, using it as a MIU address\n", __FUNCTION__, nCpuBusAddr);
        nCpuBusAddr = Chip_MIU_to_Phys(nCpuBusAddr);
    }
    nSize       = CAM_OS_ALIGN_UP(nSize + offset_in_page(nCpuBusAddr), PAGE_SIZE);

    sg_set_page(pSgTable->sgl, pfn_to_page(__phys_to_pfn(nCpuBusAddr)), PAGE_ALIGN(nSize), 0);

    for_each_sg(pSgTable->sgl, pScatterList, pSgTable->nents, i)
    {
        nPageCount += pScatterList->length / PAGE_SIZE;
    }

    ppPages = vmalloc(sizeof(struct page*)*nPageCount);

    if(ppPages == NULL)
    {
        CAM_OS_WARN("vmalloc fail");
        sg_free_table(pSgTable);
        kfree(pSgTable);
        return NULL;
    }

    for_each_sg(pSgTable->sgl, pScatterList, pSgTable->nents, i)
    {
        nPageCount = PAGE_ALIGN(pScatterList->length) / PAGE_SIZE;
        for(j = 0; j < nPageCount; j++)
            ppPages[nPageTotal + j] = sg_page(pScatterList) + j;
        nPageTotal += nPageCount;
    }
    pVirtPtr = vmap(ppPages, nPageTotal, VM_MAP, tPgProt);

    vfree(ppPages);
    sg_free_table(pSgTable);
    kfree(pSgTable);

    return (pVirtPtr) ? pVirtPtr + offset_in_page(pPhyPtr) : pVirtPtr;
}

void CamOsMemUnmap(void* pVirtPtr, u32 nSize)
{
    vunmap(pVirtPtr - offset_in_page(pVirtPtr));
}

CamOsRet_e CamOsMemCacheCreate(CamOsMemCache_t *ptMemCache, char *szName, u32 nSize, u8 bHwCacheAlign)
{
    CamOsRet_e eRet = CAM_OS_OK;
    struct kmem_cache *ptKmemCache;
    CamOsMemCacheLK_t *ptHandle = (CamOsMemCacheLK_t *)ptMemCache;

    if(ptHandle)
    {
        ptKmemCache = kmem_cache_create(szName, nSize, 0, bHwCacheAlign ? SLAB_HWCACHE_ALIGN : 0, NULL);

        if(ptKmemCache)
        {
            ptHandle->ptKmemCache = ptKmemCache;
        }
        else
        {
            ptHandle->ptKmemCache = NULL;
            eRet = CAM_OS_FAIL;
        }
    }
    else
        eRet = CAM_OS_PARAM_ERR;
    return eRet;
}

void CamOsMemCacheDestroy(CamOsMemCache_t *ptMemCache)
{
    CamOsMemCacheLK_t *ptHandle = (CamOsMemCacheLK_t *)ptMemCache;

    if(ptHandle && ptHandle->ptKmemCache)
    {
        kmem_cache_destroy(ptHandle->ptKmemCache);
        ptHandle->ptKmemCache = NULL;
    }
}

void *CamOsMemCacheAlloc(CamOsMemCache_t *ptMemCache)
{
    CamOsMemCacheLK_t *ptHandle = (CamOsMemCacheLK_t *)ptMemCache;

    if(ptHandle && ptHandle->ptKmemCache)
        return kmem_cache_alloc(ptHandle->ptKmemCache, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
    else
        return NULL;
}

void *CamOsMemCacheAllocAtomic(CamOsMemCache_t *ptMemCache)
{
    CamOsMemCacheLK_t *ptHandle = (CamOsMemCacheLK_t *)ptMemCache;

    if(ptHandle && ptHandle->ptKmemCache)
        return kmem_cache_alloc(ptHandle->ptKmemCache, GFP_ATOMIC);
    else
        return NULL;
}

void CamOsMemCacheFree(CamOsMemCache_t *ptMemCache, void *pObjPtr)
{
    CamOsMemCacheLK_t *ptHandle = (CamOsMemCacheLK_t *)ptMemCache;

    if(ptHandle && ptHandle->ptKmemCache)
        kmem_cache_free(ptHandle->ptKmemCache, pObjPtr);
}

CamOsRet_e CamOsPropertySet(const char *szKey, const char *szValue)
{
    CamOsRet_e eRet = CAM_OS_OK;

    return eRet;
}

CamOsRet_e CamOsPropertyGet(const char *szKey, char *szValue, const char *szDefaultValue)
{
    CamOsRet_e eRet = CAM_OS_OK;

    return eRet;
}

u64 CamOsMathDivU64(u64 nDividend, u64 nDivisor, u64 *pRemainder)
{
    return (pRemainder)? div64_u64_rem(nDividend, nDivisor, pRemainder) : div64_u64(nDividend, nDivisor);
}

s64 CamOsMathDivS64(s64 nDividend, s64 nDivisor, s64 *pRemainder)
{
    s64 nQuotient = div64_s64(nDividend, nDivisor);
    if (pRemainder)
        *pRemainder = nDividend - nDivisor * nQuotient;
    return nQuotient;
}

u32 CamOsCopyFromUpperLayer(void *pTo, const void *pFrom, u32 nLen)
{
    if (osm_kernel_ds())
    {
        memcpy(pTo, pFrom, nLen);
        return 0;
    }
    return copy_from_user(pTo, (const void __user *)pFrom, nLen);
}

u32 CamOsCopyToUpperLayer(void *pTo, const void * pFrom, u32 nLen)
{
    if (osm_kernel_ds())
    {
        memcpy(pTo, pFrom, nLen);
        return 0;
    }
    return copy_to_user((void __user *)pTo, pFrom, nLen);
}


static void CamOsTimerEntry(struct timer_list *ptTimer)
{
    CamOsTimerLK_t *ptHandle = container_of(ptTimer, CamOsTimerLK_t, tTimerID);
    struct osm_entry e;

    osm_enter(&e);
    ptHandle->pfnFunc(ptHandle->nData);
    osm_leave(&e);
}

CamOsRet_e CamOsTimerInit(CamOsTimer_t *ptTimer)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTimerLK_t *ptHandle = (CamOsTimerLK_t *)ptTimer;
    if(ptHandle)
    {
        timer_setup(&ptHandle->tTimerID, CamOsTimerEntry, 0);
    }
    else
        eRet = CAM_OS_PARAM_ERR;
    return eRet;
}

u32 CamOsTimerDelete(CamOsTimer_t *ptTimer)
{
    CamOsTimerLK_t *ptHandle = (CamOsTimerLK_t *)ptTimer;
    if(ptHandle)
    {
        return timer_delete(&ptHandle->tTimerID);
    }
    return 0;
}

u32 CamOsTimerDeleteSync(CamOsTimer_t *ptTimer)
{
    CamOsTimerLK_t *ptHandle = (CamOsTimerLK_t *)ptTimer;
    if(ptHandle)
    {
        return timer_delete_sync(&ptHandle->tTimerID);
    }
    return 0;
}

CamOsRet_e CamOsTimerAdd(CamOsTimer_t *ptTimer, u32 nMsec, void *pDataPtr,
                         void (*pfnFunc)(unsigned long nDataAddr))
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTimerLK_t *ptHandle = (CamOsTimerLK_t *)ptTimer;
    if(ptHandle)
    {
        ptHandle->tTimerID.expires = jiffies + msecs_to_jiffies(nMsec);
        ptHandle->pfnFunc = pfnFunc;
        ptHandle->nData = (unsigned long)pDataPtr;
        add_timer(&ptHandle->tTimerID);
    }
    else
        eRet = CAM_OS_PARAM_ERR;
    return eRet;
}

CamOsRet_e CamOsTimerModify(CamOsTimer_t *ptTimer, u32 nMsec)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsTimerLK_t *ptHandle = (CamOsTimerLK_t *)ptTimer;
    if(ptHandle)
    {
        mod_timer(&ptHandle->tTimerID, jiffies + msecs_to_jiffies(nMsec));
    }
    else
        eRet = CAM_OS_PARAM_ERR;
    return eRet;
}

s32 CamOsAtomicRead(CamOsAtomic_t *ptAtomic)
{
    if(ptAtomic)
    {
        return atomic_read((atomic_t *)ptAtomic);
    }
    return 0;
}

void CamOsAtomicSet(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        atomic_set((atomic_t *)ptAtomic, nValue);
    }
}

s32 CamOsAtomicAddReturn(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return atomic_add_return(nValue, (atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicSubReturn(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return atomic_sub_return(nValue, (atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicSubAndTest(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return atomic_sub_and_test(nValue, (atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicIncReturn(CamOsAtomic_t *ptAtomic)
{
    if(ptAtomic)
    {
        return atomic_inc_return((atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicDecReturn(CamOsAtomic_t *ptAtomic)
{
    if(ptAtomic)
    {
        return atomic_dec_return((atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicIncAndTest(CamOsAtomic_t *ptAtomic)
{
    if(ptAtomic)
    {
        return atomic_inc_and_test((atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicDecAndTest(CamOsAtomic_t *ptAtomic)
{
    if(ptAtomic)
    {
        return atomic_dec_and_test((atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicAddNegative(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return atomic_add_negative(nValue, (atomic_t *)ptAtomic);
    }
    return 0;
}

s32 CamOsAtomicCompareAndSwap(CamOsAtomic_t *ptAtomic, s32 nOldValue, s32 nNewValue)
{
    if(ptAtomic)
    {
        return atomic_cmpxchg( (atomic_t *)ptAtomic, nOldValue,  nNewValue);    // return TRUE: CAS doing,
    }
    return 0;
}

s32 CamOsAtomicAndFetch(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_and_and_fetch(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicFetchAnd(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_fetch_and_and(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicNandFetch(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_nand_and_fetch(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicFetchNand(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_fetch_and_nand(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicOrFetch(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_or_and_fetch(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicFetchOr(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_fetch_and_or(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicXorFetch(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_xor_and_fetch(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}

s32 CamOsAtomicFetchXor(CamOsAtomic_t *ptAtomic, s32 nValue)
{
    if(ptAtomic)
    {
        return __sync_fetch_and_xor(&ptAtomic->nCounter, nValue);
    }
    else
    {
        CAM_OS_WARN("null atomic handle");
        return 0;
    }
}


CamOsRet_e CamOsIdrInit(CamOsIdr_t *ptIdr)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsIdrLK_t *ptHandle = (CamOsIdrLK_t *)ptIdr;

    if(ptHandle)
    {
        idr_init(&ptHandle->tIdr);
    }
    else
        eRet = CAM_OS_PARAM_ERR;
    return eRet;
}

CamOsRet_e CamOsIdrInitEx(CamOsIdr_t *ptIdr, u32 nEntryNum)
{
    CamOsRet_e eRet = CAM_OS_OK;
    CamOsIdrLK_t *ptHandle = (CamOsIdrLK_t *)ptIdr;

    if(ptHandle)
    {
        idr_init(&ptHandle->tIdr);
    }
    else
        eRet = CAM_OS_PARAM_ERR;
    return eRet;
}

void CamOsIdrDestroy(CamOsIdr_t *ptIdr)
{
    CamOsIdrLK_t *ptHandle = (CamOsIdrLK_t *)ptIdr;

    if(ptHandle)
    {
        idr_destroy(&ptHandle->tIdr);
    }
}

s32 CamOsIdrAlloc(CamOsIdr_t *ptIdr, void *pPtr, s32 nStart, s32 nEnd)
{
    CamOsIdrLK_t *ptHandle = (CamOsIdrLK_t *)ptIdr;

    if(ptHandle)
    {
        return idr_alloc(&ptHandle->tIdr, pPtr, nStart, nEnd, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
    }
    else
        return -1;
}

void CamOsIdrRemove(CamOsIdr_t *ptIdr, s32 nId)
{
    CamOsIdrLK_t *ptHandle = (CamOsIdrLK_t *)ptIdr;

    if(ptHandle)
    {
        idr_remove(&ptHandle->tIdr, nId);
    }
}

void *CamOsIdrFind(CamOsIdr_t *ptIdr, s32 nId)
{
    CamOsIdrLK_t *ptHandle = (CamOsIdrLK_t *)ptIdr;

    if(ptHandle)
    {
        return idr_find(&ptHandle->tIdr, nId);
    }
    else
        return NULL;
}


CamOsMemSize_e CamOsPhysMemSize(void)
{
    CamOsDramInfo_t tInfo = {0};
    u32 i = 0;
    u32 nMegaBytes = 0;

    CamOsDramInfo(&tInfo);
    nMegaBytes = tInfo.nBytes >> 20;
    while (nMegaBytes >>= 1) ++i;

    return i;
}

CamOsRet_e CamOsDramInfo(CamOsDramInfo_t *ptInfo)
{
    CamOsRet_e eRet = CAM_OS_OK;
    MIU_DramInfo stDramInfo;
    MDrv_MIU_Info(&stDramInfo);
    ptInfo->nBytes = stDramInfo.size;
    ptInfo->nType = stDramInfo.type;
    ptInfo->nBusWidth = stDramInfo.bus_width;
    return eRet;
}

u32 CamOsChipId(void)
{
    return Chip_Get_Device_ID();
}

u32 CamOsChipRevision(void)
{
    return Chip_Get_Revision();;
}

CamOsRet_e CamOsIrqRequest(u32 nIrq, CamOsIrqHandler pfnHandler, const char *szName, void *pDevId)
{
    CamOsRet_e eRet = CAM_OS_OK;
    s32 nResult = 0;

    if (nIrq < LINUX_KERNEL_MAX_IRQ)
    {
        CamOsIrqHandlerList[nIrq] = pfnHandler;
        if ((nResult = request_irq(nIrq, CamOsIrqCommonHandler, IRQF_SHARED, szName, pDevId)))
        {
            CAM_OS_WARN("request_irq fail");
            CamOsPrintf("%s irq(%d) err(%d)\n", __FUNCTION__ , nIrq, nResult);
            eRet = CAM_OS_FAIL;
        }
    }
    else
    {
        eRet = CAM_OS_FAIL;
    }
    return eRet;
}

void CamOsIrqFree(u32 nIrq, void *pDevId)
{
    free_irq(nIrq, pDevId);
}

void CamOsIrqEnable(u32 nIrq)
{
    enable_irq(nIrq);
}

void CamOsIrqDisable(u32 nIrq)
{
    disable_irq(nIrq);
}

CamOsRet_e FORCE_INLINE CamOsInInterrupt(void)
{
    CamOsRet_e eRet = CAM_OS_OK;
    if(!in_interrupt())
        eRet = CAM_OS_FAIL;
    return eRet;
}

void FORCE_INLINE CamOsMemoryBarrier(void)
{
    asm volatile("": : :"memory");
}

void FORCE_INLINE CamOsSmpMemoryBarrier(void)
{
    smp_mb();
}

char _szErrStrBuf[128];

char *CamOsStrError(s32 nErrNo)
{
    sprintf(_szErrStrBuf, "errno: %d", nErrNo);
    return _szErrStrBuf;
}

void CamOsPanic(const char *szMessage)
{
    panic(szMessage);
}

long CamOsStrtol(const char *szStr, char** szEndptr, s32 nBase)
{
    return simple_strtol(szStr, szEndptr, nBase);
}

unsigned long CamOsStrtoul(const char *szStr, char** szEndptr, s32 nBase)
{
    return simple_strtoul(szStr, szEndptr, nBase);
}

unsigned long long CamOsStrtoull(const char *szStr, char** szEndptr, s32 nBase)
{
    return simple_strtoull(szStr, szEndptr, nBase);
}

unsigned long _CamOsFindFirstZeroBit(unsigned long *pAddr, unsigned long nSize,
                                     unsigned long nOffset)
{
    unsigned long *pLongBitmap = pAddr + (nOffset / CAM_OS_BITS_PER_LONG);
    unsigned long nResult = nOffset & ~(CAM_OS_BITS_PER_LONG - 1), nTemp;

    if(nOffset >= nSize)
        return nSize;

    nSize -= nResult;
    nOffset %= CAM_OS_BITS_PER_LONG;
    if(nOffset)
    {
        nTemp = *(pLongBitmap++);
        nTemp |= ~0UL >> (CAM_OS_BITS_PER_LONG - nOffset);
        if(nSize < CAM_OS_BITS_PER_LONG)
            goto IN_FIRST_BIT;

        if(~nTemp)
            goto IN_OTHER_BIT;

        nSize -= CAM_OS_BITS_PER_LONG;
        nResult += CAM_OS_BITS_PER_LONG;
    }

    //while (nSize > CAM_OS_BITS_PER_LONG)
    while(nSize & ~(CAM_OS_BITS_PER_LONG - 1))
    {
        if(~(nTemp = *pLongBitmap))
            goto IN_OTHER_BIT;

        nResult += CAM_OS_BITS_PER_LONG;
        nSize -= CAM_OS_BITS_PER_LONG;
        pLongBitmap++;
    }

    if(!nSize)
        return nResult;

    nTemp = *pLongBitmap;

IN_FIRST_BIT:
    nTemp |= ~0UL << nSize;
    if(nTemp == ~0UL)
        return nResult + nSize;

IN_OTHER_BIT:
    return nResult + CAM_OS_FFZ(nTemp);
}

static struct CamOsListHead_t *_CamOsListMerge(void *priv,
        int (*cmp)(void *priv, struct CamOsListHead_t *a,
                   struct CamOsListHead_t *b),
        struct CamOsListHead_t *a, struct CamOsListHead_t *b)
{
    struct CamOsListHead_t head, *tail = &head;

    while(a && b)
    {
        /* if equal, take 'a' -- important for sort stability */
        if((*cmp)(priv, a, b) <= 0)
        {
            tail->pNext = a;
            a = a->pNext;
        }
        else
        {
            tail->pNext = b;
            b = b->pNext;
        }
        tail = tail->pNext;
    }
    tail->pNext = a ? : b;
    return head.pNext;
}

static void _CamOsListMergeAndRestoreBackLinks(void *priv,
        int (*cmp)(void *priv, struct CamOsListHead_t *a,
                   struct CamOsListHead_t *b),
        struct CamOsListHead_t *head,
        struct CamOsListHead_t *a, struct CamOsListHead_t *b)
{
    struct CamOsListHead_t *tail = head;
    u8 count = 0;

    while(a && b)
    {
        /* if equal, take 'a' -- important for sort stability */
        if((*cmp)(priv, a, b) <= 0)
        {
            tail->pNext = a;
            a->pPrev = tail;
            a = a->pNext;
        }
        else
        {
            tail->pNext = b;
            b->pPrev = tail;
            b = b->pNext;
        }
        tail = tail->pNext;
    }
    tail->pNext = a ? : b;

    do
    {
        /*
         * In worst cases this loop may run many iterations.
         * Continue callbacks to the client even though no
         * element comparison is needed, so the client's cmp()
         * routine can invoke cond_resched() periodically.
         */
        if(!(++count))
            (*cmp)(priv, tail->pNext, tail->pNext);

        tail->pNext->pPrev = tail;
        tail = tail->pNext;
    }
    while(tail->pNext);

    tail->pNext = head;
    head->pPrev = tail;
}

void CamOsListSort(void *priv, struct CamOsListHead_t *head,
                   int (*cmp)(void *priv, struct CamOsListHead_t *a,
                              struct CamOsListHead_t *b))
{
    struct CamOsListHead_t *part[CAM_OS_MAX_LIST_LENGTH_BITS + 1]; /* sorted partial lists
						-- last slot is a sentinel */
    int lev;  /* index into part[] */
    int max_lev = 0;
    struct CamOsListHead_t *list;

    if(CAM_OS_LIST_EMPTY(head))
        return;

    memset(part, 0, sizeof(part));

    head->pPrev->pNext = NULL;
    list = head->pNext;

    while(list)
    {
        struct CamOsListHead_t *cur = list;
        list = list->pNext;
        cur->pNext = NULL;

        for(lev = 0; part[lev]; lev++)
        {
            cur = _CamOsListMerge(priv, cmp, part[lev], cur);
            part[lev] = NULL;
        }
        if(lev > max_lev)
        {
            if(lev >= CAM_OS_ARRAY_SIZE(part) - 1)
            {
                CAM_OS_WARN("list too long");
                lev--;
            }
            max_lev = lev;
        }
        part[lev] = cur;
    }

    for(lev = 0; lev < max_lev; lev++)
        if(part[lev])
            list = _CamOsListMerge(priv, cmp, part[lev], list);

    _CamOsListMergeAndRestoreBackLinks(priv, cmp, head, part[max_lev], list);
}
