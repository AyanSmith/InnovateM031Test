/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 18/04/24 3:05p $
 * @brief    M031 series FMC driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"



/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/


/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

int32_t  g_FMC_i32ErrCode;

/**
  * @brief Disable FMC ISP function.
  * @return None
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 4096 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 4096 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out 
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    if (u32PageAddr == FMC_SPROM_BASE)
    {
        return FMC_Erase_SPROM();
    }
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}


/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase SPROM. The page size is 4096 bytes.
  * @return   SPROM page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out 
  */
int32_t FMC_Erase_SPROM(void)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = FMC_SPROM_BASE;
    FMC->ISPDAT = 0x0055AA03UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;

    }
    return 0;
}


/**
 * @brief      Execute Flash Bank erase
 *
 * @param[in]  u32BankAddr Base address of the flash bank to be erased.
 *
 * @return     ISP bank erase success or not.
 * @retval     0  Success
 * @retval     -1  Erase failed
 *
 * @details  Execute FMC_ISPCMD_BANK_ERASE command to erase a flash bank.
 *
 * @note     Global error code g_FMC_i32ErrCode
 *           -1  Erase failed or erase time-out  
 */
int32_t FMC_Erase_Bank(uint32_t u32BankAddr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_BANK_ERASE;
    FMC->ISPADDR = u32BankAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_ERASE;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
 * @brief Execute FMC_ISPCMD_BANK_REMAP command to remap bank.
 * @return   Bank remap success or not.
 * @retval   0  Success
 * @retval   -1  Erase failed
 */
int32_t FMC_RemapBank(uint32_t u32BankIdx)
{
    uint32_t  tout = FMC_TIMEOUT_WRITE;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_BANK_REMAP;
    FMC->ISPADDR = u32BankIdx;
    FMC->ISPDAT = 0x5AA55AA5UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t FMC_GetBootSource (void)
{
    int32_t  ret = 0;

    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
    {
        ret = 1;
    }

    return ret;
}


/**
  * @brief Enable FMC ISP function
  * @return None
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute FMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *          Return 0xFFFFFFFF if read failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out 
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_READ;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return FMC->ISPDAT;
}


/**
  * @brief    Get the base address of Data Flash if enabled.
  * @retval   The base address of Data Flash
  */
uint32_t FMC_ReadDataFlashBaseAddr(void)
{
    return FMC->DFBA;
}

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  * @param[in]  i32BootSrc
  *                1: Boot from LDROM
  *                0: Boot from APROM
  * @return    None
  * @details   This function is used to switch APROM boot or LDROM boot. User need to call
  *            FMC_SetBootSource to select boot source first, then use CPU reset or
  *            System Reset Request to reset system.
  */
void FMC_SetBootSource(int32_t i32BootSrc)
{
    if(i32BootSrc)
    {
        FMC->ISPCTL |= FMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    }
    else
    {
        FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk;/* Boot from APROM */
    }
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   0   Success
  * @return   -1  Program Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  
  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    
    tout = FMC_TIMEOUT_WRITE;
    
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    /* Invalidation Cache */
    FMC->FTCTL |= FMC_FTCTL_CACHEINV_Msk;
    
    tout = FMC_TIMEOUT_WRITE;
    
    while ((--tout > 0) && (FMC->FTCTL & FMC_FTCTL_CACHEINV_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM_64 to program a double-word to flash.
  * @param[in]  u32addr Address of the flash location to be programmed.
  *             It must be a double-word aligned address.
  * @param[in]  u32data0   The word data to be programmed to flash address u32addr.
  * @param[in]  u32data1   The word data to be programmed to flash address u32addr+4.
  * @return   0   Success
  * @return   -1  Program Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_64;
    FMC->ISPADDR = u32addr;
    FMC->MPDAT0  = u32data0;
    FMC->MPDAT1  = u32data1;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_WRITE;
    while ((--tout > 0) && (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    /* Invalidation Cache */
    FMC->FTCTL |= FMC_FTCTL_CACHEINV_Msk;
    
    tout = FMC_TIMEOUT_WRITE;
    
    while ((--tout > 0) && (FMC->FTCTL & FMC_FTCTL_CACHEINV_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    return 0;
}

/**
  * @brief Execute FMC_ISPCMD_READ command to read User Configuration.
  * @param[out]  u32Config A three-word array.
  *              u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Available word count in u32Config.
  * @return Success or not.
  * @retval   0  Success.
  * @retval   -1  Read failed
  * @retval   -2  Invalid parameter.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read failed
  *           -2  Invalid parameter
  */
int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int32_t   ret = 0;

    u32Config[0] = FMC_Read(FMC_CONFIG_BASE);

    if (g_FMC_i32ErrCode != 0)
        return g_FMC_i32ErrCode;

    if (u32Count > 3UL)
    {
        ret = -2;
    }
    else
    {
        if(u32Count > 1UL)
        {
            u32Config[1] = FMC_Read(FMC_CONFIG_BASE+4UL);
            
            if (g_FMC_i32ErrCode != 0)
                return g_FMC_i32ErrCode;
            
        }
        if(u32Count > 2UL)
        {
            u32Config[2] = FMC_Read(FMC_CONFIG_BASE+8UL);
            if (g_FMC_i32ErrCode != 0)
                return g_FMC_i32ErrCode;
            
        }
    }
    return ret;
}

/**
 * @brief Execute ISP commands to erase then write User Configuration.
 * @param[in] u32Config   A two-word array.
 *            u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
 * @param[in] u32Count  Always be 2 in this BSP.
 * @return Success or not.
 * @retval   0  Success.
 * @retval   -1  Erase/program/read/verify failed
 *
 * @note     Global error code g_FMC_i32ErrCode
 *           < 0  Errors caused by erase/program/read failed or time-out
 */
int32_t FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count)
{
    uint32_t i;
    
    FMC_ENABLE_CFG_UPDATE();

    if (FMC_Erase(FMC_CONFIG_BASE) != 0)
        return -1;
        
    if ((FMC_Read(FMC_CONFIG_BASE) != 0xFFFFFFFF) || (FMC_Read(FMC_CONFIG_BASE+4) != 0xFFFFFFFF) ||
            (FMC_Read(FMC_CONFIG_BASE+8) != 0xFFFFFF5A))
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (g_FMC_i32ErrCode != 0)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    for (i = 0u; i < u32Count; i++)
    {
        if (FMC_Write(FMC_CONFIG_BASE+i*4UL, u32Config[i]) != 0)
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }

        if (FMC_Read(FMC_CONFIG_BASE+i*4UL) != u32Config[i])
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }
        if (g_FMC_i32ErrCode != 0)
        {
            FMC_DISABLE_CFG_UPDATE();
            return -1;
        }  
    }
    
    FMC_DISABLE_CFG_UPDATE();

    return 0;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  * @return Success or not.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Run/Read check sum time-out failed
  *           -2  u32addr or u32count must be aligned with 512
  */
uint32_t  FMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  tout;
    
    g_FMC_i32ErrCode = 0;

    if ((u32addr % 512UL) || (u32count % 512UL))
    {
        g_FMC_i32ErrCode = -2;
        return 0xFFFFFFFF;

    }
        FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
        FMC->ISPADDR = u32addr;
        FMC->ISPDAT  = u32count;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        tout = FMC_TIMEOUT_CHKSUM;
        while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}
        if (tout == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }

        FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
        FMC->ISPADDR    = u32addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

        tout = FMC_TIMEOUT_CHKSUM;
        while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}
        if (tout == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }

        return FMC->ISPDAT;
}

/**
 * @brief Run flash all one verification and get result.
 *
 * @param[in] u32addr   Starting flash address. It must be a page aligned address.
 * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
 *
 * @retval   READ_ALLONE_YES      The contents of verified flash area are 0xFFFFFFFF.
 * @retval   READ_ALLONE_NOT  Some contents of verified flash area are not 0xFFFFFFFF.
 * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
 *
 * @details  Run ISP check all one command to check specify area is all one or not.
 *
 * @note     Global error code g_FMC_i32ErrCode
 *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out
 */
#define FMC_APROM_BANK1_BASE    (0x40000)
#define FMC_CHECKALLONE_UNIT    (512)
uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  tout;

    g_FMC_i32ErrCode = 0;
    /** Workaround solution for M031 with 512KB Flash uses FMC Read command instead of FMC All-One-Verification command to 
      * check the Flash content from 0x40000 to 0x401FF.
      */    
    if(u32addr == FMC_APROM_BANK1_BASE)
    {
        uint32_t i;
        u32count = u32count - FMC_CHECKALLONE_UNIT;
        for(i = FMC_APROM_BANK1_BASE; i < (FMC_APROM_BANK1_BASE + FMC_CHECKALLONE_UNIT); i = i+4)
        {
            if( FMC_Read(i) != 0xFFFFFFFF)
                return READ_ALLONE_NOT;
        }

        if(u32count == 0)
            return READ_ALLONE_YES;
        else
            u32addr = u32addr + FMC_CHECKALLONE_UNIT;
    }

    FMC->ISPSTS = 0x80UL; /* clear check all one bit */

    FMC->ISPCMD = FMC_ISPCMD_RUN_ALL1;
    FMC->ISPADDR = u32addr;
    FMC->ISPDAT = u32count;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKALLONE;
    while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return READ_ALLONE_CMD_FAIL;
    }

    tout = FMC_TIMEOUT_CHKALLONE;

    do
    {
        FMC->ISPCMD = FMC_ISPCMD_READ_ALL1;
        FMC->ISPADDR = u32addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

        while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) {}
        if (tout == 0)
        {
            g_FMC_i32ErrCode = -1;
            return READ_ALLONE_CMD_FAIL;
        }
    }
    while (FMC->ISPDAT == 0UL);
    
    if ((FMC->ISPDAT == READ_ALLONE_YES) || (FMC->ISPDAT == READ_ALLONE_NOT))
        return FMC->ISPDAT;
    else
    {
        g_FMC_i32ErrCode = -1;
        return READ_ALLONE_CMD_FAIL;
    }
}

/**
 * @brief      Write Multi-Word bytes to flash
 *
 * @param[in]  u32Addr    Start flash address in APROM where the data chunk to be programmed into.
 *                        This address must be 8-bytes aligned to flash address.
 * @param[in]  pu32Buf    Buffer that carry the data chunk.
 * @param[in]  u32Len     Length of the data chunk in bytes.
 *
 * @retval     >=0  Number of data bytes were programmed.
 * @return   -1   Program failed.
 * @return   -2   Invalid address.
 *
 * @detail     Program Multi-Word data into specified address of flash.
 * @note     Global error code g_FMC_i32ErrCode
 *           -1  Program failed or time-out
 *           -2  Invalid address 
 */
#if defined ( __CC_ARM )
#pragma arm section code="fastcode"
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)

#elif defined ( __ICCARM__ )
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len) @ "fastcode"

#elif defined ( __GNUC__ )
#pragma GCC push_options
#pragma GCC optimize ("O0")
__attribute__ ((used,  section(".fastcode"))) int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)

#else
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)
#endif
{

    uint32_t i, idx, u32OnProg, retval = 0;
    uint32_t  tout;
    int32_t err;

    g_FMC_i32ErrCode = 0;
    if ((u32Addr % 8) != 0)
    {
        g_FMC_i32ErrCode = -2;
        return -2;
    }

    idx = 0u;
    FMC->ISPCMD = FMC_ISPCMD_MULTI_PROG;
    FMC->ISPADDR = u32Addr;
    retval += 16;
    do
    {
        err = 0;
        u32OnProg = 1u;
        FMC->MPDAT0 = pu32Buf[idx + 0u];
        FMC->MPDAT1 = pu32Buf[idx + 1u];
        FMC->MPDAT2 = pu32Buf[idx + 2u];
        FMC->MPDAT3 = pu32Buf[idx + 3u];
        FMC->ISPTRG = 0x1u;
        idx += 4u;

        for (i = idx; i < (FMC_MULTI_WORD_PROG_LEN / 4u); i += 4u) /* Max data length is 256 bytes (512/4 words)*/
        {
            __set_PRIMASK(1u); /* Mask interrupt to avoid status check coherence error*/
            tout = FMC_TIMEOUT_WRITE;
            do
            {
                if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0u)
                {
                    __set_PRIMASK(0u);

                    FMC->ISPADDR = FMC->MPADDR & (~0xful);
                    idx = (FMC->ISPADDR - u32Addr) / 4u;
                    err = -1;
                }
            }
            while ((--tout > 0) && (FMC->MPSTS & (3u << FMC_MPSTS_D0_Pos)) && (err == 0));

            if (tout == 0)
            {
                g_FMC_i32ErrCode = -1;
                return -1;
            }
            if (err == 0)
            {
                retval += 8;

                /* Update new data for D0 */
                FMC->MPDAT0 = pu32Buf[i];
                FMC->MPDAT1 = pu32Buf[i + 1u];
                tout = FMC_TIMEOUT_WRITE;
                do
                {
                    if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0u)
                    {
                        __set_PRIMASK(0u);
                        FMC->ISPADDR = FMC->MPADDR & (~0xful);
                        idx = (FMC->ISPADDR - u32Addr) / 4u;
                        err = -1;
                    }
                }
                while ((--tout > 0) && (FMC->MPSTS & (3u << FMC_MPSTS_D2_Pos)) && (err == 0));

                if (tout == 0)
                {
                    g_FMC_i32ErrCode = -1;
                    return -1;
                }

                if (err == 0)
                {
                    retval += 8;

                    /* Update new data for D2*/
                    FMC->MPDAT2 = pu32Buf[i + 2u];
                    FMC->MPDAT3 = pu32Buf[i + 3u];
                    __set_PRIMASK(0u);
                }
            }

            if (err < 0)
            {
                break;
            }
        }
        if (err == 0)
        {
            u32OnProg = 0u;
            
            tout = FMC_TIMEOUT_WRITE;
            
            while ((--tout > 0) && (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)) { }

            if (tout == 0)
            {
                g_FMC_i32ErrCode = -1;
                return -1;
            }            
        }
    }
    while (u32OnProg);

    /* Invalidation Cache */
    FMC->FTCTL |= FMC_FTCTL_CACHEINV_Msk;
    
    tout = FMC_TIMEOUT_WRITE;
    
    while ((--tout > 0) && (FMC->FTCTL & FMC_FTCTL_CACHEINV_Msk)) {}
    
    if (tout == 0)
    {
        g_FMC_i32ErrCode = -1;
        return -1;
    }

    return retval;
}
#if defined ( __CC_ARM )
#pragma arm section

#elif defined ( __GNUC__ )
#pragma GCC pop_options

#endif

/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group FMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/


