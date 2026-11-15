/*
* mdrv_miu.c- Sigmastar
*
* Copyright (C) 2018 Sigmastar Technology Corp.
*
* Author: karl.xiao <karl.xiao@sigmastar.com.tw>
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
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include "MsTypes.h"
#include "mdrv_miu.h"
#include "mhal_miu.h"
#include "regMIU.h"
#include <asm/io.h>
#include "osm_compat.h"
#include "osm_sstar.h"
#include <linux/of.h>
#include <linux/of_irq.h>

//-------------------------------------------------------------------------------------------------
//  Function prototype with weak symbol
//-------------------------------------------------------------------------------------------------

MS_BOOL HAL_MIU_Protect(MS_U8 u8Blockx, MS_U16 *pu8ProtectId, MS_U32 u32BusStart, MS_U32 u32BusEnd, MS_BOOL bSetFlag) __attribute__((weak));
MS_BOOL HAL_MIU_GetHitProtectInfo(MS_U8 u8MiuSel, MIU_PortectInfo *pInfo) __attribute__((weak));
MS_BOOL HAL_MIU_ParseOccupiedResource(void) __attribute__((weak));
MS_U16* HAL_MIU_GetDefaultKernelProtectClientID(void) __attribute__((weak));
MS_U16* HAL_MIU_GetKernelProtectClientID(MS_U8 u8MiuSel) __attribute__((weak));

MS_BOOL HAL_MIU_SlitInit(void) __attribute__((weak));
MS_BOOL HAL_MIU_SetSlitRange(MS_U8 u8Blockx, MS_U16 *pu8ProtectId, MS_PHY u64BusStart, MS_PHY u64BusEnd, MS_BOOL  bSetFlag) __attribute__((weak));
MS_BOOL HAL_MIU_Slits(MS_U8 u8Blockx, MS_PHY u64SlitsStart, MS_PHY u64SlitsEnd, MS_BOOL bSetFlag) __attribute__((weak));

//--------------------------------------------------------------------------------------------------
//  Local variable
//--------------------------------------------------------------------------------------------------

static int bMiuProtect_is_initialized = 0;
static int MiuSelId[MIU_MAX_DEVICE] = {0};

static DEFINE_SPINLOCK(miu_lock);
MS_U8 u8_MiuWhiteListNum = 0;


//-------------------------------------------------------------------------------------------------
//  Local functions
//-------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function  \b Name: MDrv_MIU_Init
/// @brief \b Function  \b Description: parse occupied resource to software structure
/// @param None         \b IN :
/// @param None         \b OUT :
/// @param MS_BOOL      \b RET
/// @param None         \b GLOBAL :
////////////////////////////////////////////////////////////////////////////////
MS_BOOL MDrv_MIU_Init(void)
{
    MS_BOOL ret = FALSE;

    /* Parse the used client ID in hardware into software data structure */
    if (HAL_MIU_ParseOccupiedResource) {
        ret = HAL_MIU_ParseOccupiedResource();
    }
    else {
        ret = FALSE;
    }

    /* Initialize MIU slit setting */
    if (HAL_MIU_SlitInit) {
        ret = HAL_MIU_SlitInit();
    }

    u8_MiuWhiteListNum = IDNUM_KERNELPROTECT;

    return ret;
}

MS_BOOL MDrv_MIU_GetProtectInfo(MS_U8 u8MiuSel, MIU_PortectInfo *pInfo)
{
    return HAL_MIU_GetHitProtectInfo(u8MiuSel, pInfo);
}

MS_BOOL MDrv_MIU_Get_IDEnables_Value(MS_U8 u8MiuSel, MS_U8 u8Blockx, MS_U8 u8ClientIndex)
{
   return HAL_MIU_GetProtectIdEnVal(u8MiuSel, u8Blockx, u8ClientIndex);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: MDrv_MIU_GetDefaultClientID_KernelProtect()
/// @brief \b Function \b Description:  Get default client id array pointer for protect kernel
/// @param <RET>           \b     : The pointer of Array of client IDs
////////////////////////////////////////////////////////////////////////////////
MS_U16* MDrv_MIU_GetDefaultClientID_KernelProtect(void)
{
    if (HAL_MIU_GetDefaultKernelProtectClientID) {
        return HAL_MIU_GetDefaultKernelProtectClientID();
    }
    else {
        return NULL;
    }
}

MS_U16* MDrv_MIU_GetClientID_KernelProtect(MS_U8 u8MiuSel)
{
    if (HAL_MIU_GetKernelProtectClientID) {
        return HAL_MIU_GetKernelProtectClientID(u8MiuSel);
    }
    else {
        return NULL;
    }
}

static irqreturn_t MDrv_MIU_Protect_interrupt(s32 irq, void *dev_id)
{
    MS_U8 u8MiuSel = *(MS_U8 *)dev_id;
    MIU_PortectInfo pInfo;

    memset(&pInfo, 0, sizeof(pInfo));

    MDrv_MIU_GetProtectInfo(u8MiuSel, &pInfo);
    BUG();
    return IRQ_HANDLED;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief \b Function \b Name: MDrv_MIU_Protect()
/// @brief \b Function \b Description:  Enable/Disable MIU Protection mode
/// @param u8Blockx        \b IN     : MIU Block to protect (0 ~ 3)
/// @param *pu8ProtectId   \b IN     : Allow specified client IDs to write
/// @param u32Start        \b IN     : Starting address(bus address)
/// @param u32End          \b IN     : End address(bus address)
/// @param bSetFlag        \b IN     : Disable or Enable MIU protection
///                                      - -Disable(0)
///                                      - -Enable(1)
/// @param <OUT>           \b None    :
/// @param <RET>           \b None    :
/// @param <GLOBAL>        \b None    :
////////////////////////////////////////////////////////////////////////////////
/*
 * opensigmastar: MIU write protection is a debug feature of the BSP. mi_sys
 * protects the whole Linux memory except its heaps for the clients in the
 * kernel whitelist; a hit calls BUG() from the protect interrupt. It needs
 * the "sstar,miu" DT node and a complete whitelist, so it is off by default
 * and MDrv_MIU_Protect() then succeeds without touching the hardware.
 */
static bool miu_protect;
module_param(miu_protect, bool, 0444);
MODULE_PARM_DESC(miu_protect, "Enable the MIU kernel write protection (debug)");

MS_BOOL MDrv_MIU_Protect(   MS_U8   u8Blockx,
                            MS_U16  *pu8ProtectId,
                            MS_PHY  u64BusStart,
                            MS_PHY  u64BusEnd,
                            MS_BOOL bSetFlag)
{
    MS_BOOL Result = FALSE;

    if (!miu_protect)
        return TRUE;

    /* Case of former MIU protect */
    if ((u8Blockx >= E_PROTECT_0) && (u8Blockx < E_SLIT_0))
    {
        if (HAL_MIU_Protect) {
            if (TRUE != (Result = HAL_MIU_Protect(u8Blockx, pu8ProtectId, u64BusStart, u64BusEnd, bSetFlag)))
            {
                goto MDrv_MIU_Protect_Exit;
            }
        }
    }
    /* Case of MIU slits */
    else if (u8Blockx == E_SLIT_0)
    {
        if (HAL_MIU_SetSlitRange) {
            if (TRUE != (Result = HAL_MIU_SetSlitRange(u8Blockx, pu8ProtectId, u64BusStart, u64BusEnd, bSetFlag)))
            {
                goto MDrv_MIU_Protect_Exit;
            }
        }
    }

    if(!bMiuProtect_is_initialized)
    {
        struct device_node  *dev_node = NULL;
        MS_U8 u8MiuSel = 0;
        int rc = 0;
        int iIrqNum = 0;
        char compat_str[32] = {0};

        for (u8MiuSel = 0; u8MiuSel < MIU_MAX_DEVICE; u8MiuSel++)
        {
            if (u8MiuSel == 0)
            {
                snprintf(compat_str, sizeof(compat_str)-1, "sstar,miu");
            }
            else
            {
                snprintf(compat_str, sizeof(compat_str)-1, "sstar,miu%d", u8MiuSel);
            }

            dev_node = of_find_compatible_node(NULL, NULL, compat_str);
            if (!dev_node)
            {
                printk("[MIU Protecrt] of_find_compatible_node Fail\r\n");
                Result = FALSE;
                goto MDrv_MIU_Protect_Exit;
            }

            iIrqNum = irq_of_parse_and_map(dev_node, 0);

            MiuSelId[u8MiuSel] = u8MiuSel;

            if(0 != (rc = request_irq(iIrqNum, MDrv_MIU_Protect_interrupt, IRQF_TRIGGER_HIGH, "MIU_Protect", (void *)&MiuSelId[u8MiuSel])))
            {
                printk("[MIU Protecrt] request_irq [%d] Fail, ErrCode: %d\r\n", iIrqNum, rc);
                Result = FALSE;
                goto MDrv_MIU_Protect_Exit;
            }
        }
        bMiuProtect_is_initialized++;
    }


MDrv_MIU_Protect_Exit:
    return Result;
}

unsigned int MDrv_MIU_ProtectDramSize(void)
{
    return HAL_MIU_ProtectDramSize();
}

int MDrv_MIU_ClientIdToName(MS_U16 clientId, char *clientName)
{
    return HAL_MIU_ClientIdToName(clientId, clientName);
}

MS_BOOL MDrv_MIU_Slits(MS_U8 u8Blockx, MS_PHY u64SlitsStart, MS_PHY u64SlitsEnd, MS_BOOL bSetFlag)
{
    unsigned long flags;
    MS_BOOL Result = FALSE;

    spin_lock_irqsave(&miu_lock, flags);

    if (HAL_MIU_Slits) {
        Result = HAL_MIU_Slits(u8Blockx, u64SlitsStart, u64SlitsEnd, bSetFlag);
    }

    spin_unlock_irqrestore(&miu_lock, flags);

    return Result;
}

int MDrv_MIU_Info(MIU_DramInfo *pDramInfo)
{
    return HAL_MIU_Info((MIU_DramInfo_Hal *)pDramInfo);
}

EXPORT_SYMBOL(MDrv_MIU_Init);
EXPORT_SYMBOL(u8_MiuWhiteListNum);
EXPORT_SYMBOL(MDrv_MIU_GetDefaultClientID_KernelProtect);
EXPORT_SYMBOL(MDrv_MIU_GetClientID_KernelProtect);
EXPORT_SYMBOL(MDrv_MIU_Protect);
EXPORT_SYMBOL(MDrv_MIU_Get_IDEnables_Value);
EXPORT_SYMBOL(MDrv_MIU_Info);
static DEFINE_SPINLOCK(mmu_lock);

static int mmu_isr_init=0;
MDrv_MMU_Callback MDrv_MMU_Notify=NULL;

int MDrv_MMU_SetPageSize(unsigned char u8PgSz256En)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&mmu_lock, flags);
    ret = HAL_MMU_SetPageSize(u8PgSz256En);
    spin_unlock_irqrestore(&mmu_lock, flags);

    return ret;
}

int MDrv_MMU_SetRegion(unsigned short u16Region, unsigned short u16ReplaceRegion)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&mmu_lock, flags);
    ret = HAL_MMU_SetRegion(u16Region,u16ReplaceRegion);
    spin_unlock_irqrestore(&mmu_lock, flags);

    return ret;
}

int MDrv_MMU_Map(unsigned short u16PhyAddrEntry, unsigned short u16VirtAddrEntry)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&mmu_lock, flags);
    ret = HAL_MMU_Map(u16PhyAddrEntry, u16VirtAddrEntry);
    spin_unlock_irqrestore(&mmu_lock, flags);

    return ret;
}

unsigned short MDrv_MMU_MapQuery(unsigned short u16PhyAddrEntry)
{
    unsigned long flags;
    unsigned short entry;

    spin_lock_irqsave(&mmu_lock, flags);
    entry = HAL_MMU_MapQuery(u16PhyAddrEntry);
    spin_unlock_irqrestore(&mmu_lock, flags);

    return entry;
}

int MDrv_MMU_UnMap(unsigned short u16PhyAddrEntry)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&mmu_lock, flags);
    ret = HAL_MMU_UnMap(u16PhyAddrEntry);
    spin_unlock_irqrestore(&mmu_lock, flags);

    return ret;
}

static irqreturn_t MDrv_MMU_interrupt(s32 irq, void *dev_id)
{
    unsigned long flags;
    unsigned int status;
    unsigned short u16PhyAddrEntry, u16ClientId;
    unsigned char u8IsWriteCmd;

    spin_lock_irqsave(&mmu_lock, flags);
    status = HAL_MMU_Status(&u16PhyAddrEntry, &u16ClientId, &u8IsWriteCmd);

    if (MDrv_MMU_Notify != NULL)
    {
        struct osm_entry e;

        osm_enter(&e);
        MDrv_MMU_Notify(status, u16PhyAddrEntry, u16ClientId, u8IsWriteCmd);
        osm_leave(&e);
    }
    else
    {
        printk("[%s] Status=0x%x, PhyAddrEntry=0x%x, ClientId=0x%x, IsWrite=%d\n", __FUNCTION__,
                                                                                   status,
                                                                                   u16PhyAddrEntry,
                                                                                   u16ClientId,
                                                                                   u8IsWriteCmd);
    }

    spin_unlock_irqrestore(&mmu_lock, flags);

    return IRQ_HANDLED;
}

void MDrv_MMU_CallbackFunc(MDrv_MMU_Callback pFuncPtr)
{
    unsigned long flags;

    spin_lock_irqsave(&mmu_lock, flags);
    MDrv_MMU_Notify = pFuncPtr;
    spin_unlock_irqrestore(&mmu_lock, flags);
}
EXPORT_SYMBOL(MDrv_MMU_CallbackFunc);
int MDrv_MMU_Enable(unsigned char u8Enable)
{
    unsigned long flags;
    int ret;

    if (!mmu_isr_init)
    {
        struct device_node  *dev_node = NULL;
        int rc = 0;
        int iIrqNum = 0;
        dev_node = of_find_compatible_node(NULL, NULL, "sstar,mmu");
        if (!dev_node)
        {
            printk("[MMU] of_find_compatible_node Fail\r\n");
        }

        iIrqNum = irq_of_parse_and_map(dev_node, 0);

        if(0 != (rc = request_irq(iIrqNum, MDrv_MMU_interrupt, IRQF_TRIGGER_HIGH, "MMU", NULL)))
        {
            printk("[MMU] request_irq [%d] Fail, ErrCode: %d\r\n", iIrqNum, rc);
        }
        mmu_isr_init = rc ? -1 : iIrqNum;
    }

    spin_lock_irqsave(&mmu_lock, flags);
    ret = HAL_MMU_Enable(u8Enable);
    spin_unlock_irqrestore(&mmu_lock, flags);

    return ret;
}

int MDrv_MMU_Reset(void)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&mmu_lock, flags);
    ret = HAL_MMU_Reset();
    spin_unlock_irqrestore(&mmu_lock, flags);

    return ret;
}

EXPORT_SYMBOL(MDrv_MMU_SetPageSize);
EXPORT_SYMBOL(MDrv_MMU_SetRegion);
EXPORT_SYMBOL(MDrv_MMU_Map);
EXPORT_SYMBOL(MDrv_MMU_MapQuery);
EXPORT_SYMBOL(MDrv_MMU_UnMap);
EXPORT_SYMBOL(MDrv_MMU_Enable);
EXPORT_SYMBOL(MDrv_MMU_Reset);

/* opensigmastar: no platform driver, set up from osm_sstar */
void osm_miu_exit(void)
{
    if (mmu_isr_init > 0)
        free_irq(mmu_isr_init, NULL);
}
