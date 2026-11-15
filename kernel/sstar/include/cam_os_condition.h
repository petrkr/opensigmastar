/*
* cam_os_condition.h- Sigmastar
*
* Copyright (c) [2019~2020] SigmaStar Technology.
*
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License version 2 for more details.
*
*/


#ifndef __CAM_OS_CONDITION_H__
#define __CAM_OS_CONDITION_H__

#include "cam_os_wrapper.h"

#define CAM_OS_LINUX_KERNEL

#include "linux/wait.h"
typedef wait_queue_head_t   CamOsCondition_t;

#define CamOsConditionInit(ptCondition)                                     \
    init_waitqueue_head(ptCondition);

#define CamOsConditionDeinit(ptCondition)

#define CamOsConditionWakeUpAll(ptCondition)                                \
    wake_up_all(ptCondition);

#define CamOsConditionWait(ptCondition, condition)                          \
({                                                                          \
    CamOsRet_e __eRet = CAM_OS_OK;                                          \
    wait_event((*(ptCondition)), condition);                                \
    __eRet;                                                                 \
})

#define CamOsConditionTimedWait(ptCondition, condition, nMsec)              \
({                                                                          \
    CamOsRet_e __eRet = CAM_OS_OK;                                          \
    if (!wait_event_timeout((*(ptCondition)), condition,                    \
                               msecs_to_jiffies(nMsec)))                    \
    {                                                                       \
        __eRet = CAM_OS_TIMEOUT;                                            \
    }                                                                       \
    __eRet;                                                                 \
})

#endif //__CAM_OS_CONDITION_H__
