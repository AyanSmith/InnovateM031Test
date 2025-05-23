/****************************************************************************
 * @file     clk_conf.c
 * @version  V0.42
 * @Date     2023/01/18-00:22:54 
 * @brief    NuMicro generated code file
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2016-2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/********************
MCU:M031LE3AE(LQFP48)
Base Clocks:
LIRC:38.4kHz
HIRC:48MHz
HCLK:48MHz
PCLK0:48MHz
PCLK1:48MHz
Enabled-Module Frequencies:
ADC=Bus Clock(PCLK1):48MHz/Engine Clock:48MHz
I2C0=Bus Clock(PCLK0):48MHz
ISP=Bus Clock(HCLK):48MHz/Engine Clock:48MHz
SPI0=Bus Clock(PCLK1):48MHz/Engine Clock:48MHz
UART0=Bus Clock(PCLK0):48MHz/Engine Clock:48MHz
UART2=Bus Clock(PCLK0):48MHz/Engine Clock:48MHz
WDT=Bus Clock(PCLK0):48MHz/Engine Clock:38.4kHz
WWDT=Bus Clock(PCLK0):48MHz/Engine Clock:23.4375kHz
********************/

#include "NuCodeGenProj.h"
#include "clk_conf.h"

void NuCodeGenProj_init_adc(void)
{
    CLK_EnableModuleClock(ADC_MODULE);
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));

    return;
}

void NuCodeGenProj_deinit_adc(void)
{
    CLK_DisableModuleClock(ADC_MODULE);

    return;
}

void NuCodeGenProj_init_i2c0(void)
{
    CLK_EnableModuleClock(I2C0_MODULE);

    return;
}

void NuCodeGenProj_deinit_i2c0(void)
{
    CLK_DisableModuleClock(I2C0_MODULE);

    return;
}

void NuCodeGenProj_init_isp(void)
{
    CLK_EnableModuleClock(ISP_MODULE);

    return;
}

void NuCodeGenProj_deinit_isp(void)
{
    CLK_DisableModuleClock(ISP_MODULE);

    return;
}

void NuCodeGenProj_init_spi0(void)
{
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    return;
}

void NuCodeGenProj_deinit_spi0(void)
{
    CLK_DisableModuleClock(SPI0_MODULE);

    return;
}

void NuCodeGenProj_init_uart0(void)
{
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    return;
}

void NuCodeGenProj_deinit_uart0(void)
{
    CLK_DisableModuleClock(UART0_MODULE);

    return;
}

void NuCodeGenProj_init_uart2(void)
{
    CLK_EnableModuleClock(UART2_MODULE);
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_PCLK0, CLK_CLKDIV4_UART2(1));

    return;
}

void NuCodeGenProj_deinit_uart2(void)
{
    CLK_DisableModuleClock(UART2_MODULE);

    return;
}

void NuCodeGenProj_init_wdt(void)
{
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);

    return;
}

void NuCodeGenProj_deinit_wdt(void)
{
    CLK_DisableModuleClock(WDT_MODULE);

    return;
}

void NuCodeGenProj_init_wwdt(void)
{
    CLK_EnableModuleClock(WWDT_MODULE);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);

    return;
}

void NuCodeGenProj_deinit_wwdt(void)
{
    CLK_DisableModuleClock(WWDT_MODULE);

    return;
}

void NuCodeGenProj_init_base(void)
{
    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk);

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    return;
}

void Clock_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    //CLK->PWRCTL   = (CLK->PWRCTL   & ~(0x0000000FUL)) | 0x0231001CUL;
    //CLK->PLLCTL   = (CLK->PLLCTL   & ~(0x000FFFFFUL)) | 0x0005C25EUL;
    //CLK->CLKDIV0  = (CLK->CLKDIV0  & ~(0x00FFFFFFUL)) | 0x00000000UL;
    //CLK->CLKDIV4  = (CLK->CLKDIV4  & ~(0x00FFFFFFUL)) | 0x00000000UL;
    //CLK->PCLKDIV  = (CLK->PCLKDIV  & ~(0x00000077UL)) | 0x00000000UL;
    //CLK->CLKSEL0  = (CLK->CLKSEL0  & ~(0x0000013FUL)) | 0x0000003FUL;
    //CLK->CLKSEL1  = (CLK->CLKSEL1  & ~(0x7777777FUL)) | 0x4477773BUL;
    //CLK->CLKSEL2  = (CLK->CLKSEL2  & ~(0x0030033FUL)) | 0x0020032BUL;
    //CLK->CLKSEL3  = (CLK->CLKSEL3  & ~(0x77777700UL)) | 0x44444400UL;
    //CLK->AHBCLK   = (CLK->AHBCLK   & ~(0x0000009EUL)) | 0x00000004UL;
    //CLK->APBCLK0  = (CLK->APBCLK0  & ~(0x18FF33FFUL)) | 0x10052101UL;
    //CLK->APBCLK1  = (CLK->APBCLK1  & ~(0x000F0300UL)) | 0x00000000UL;
    //CLK->CLKOCTL  = (CLK->CLKOCTL  & ~(0x0000007FUL)) | 0x00000000UL;
    //SysTick->CTRL = (SysTick->CTRL & ~(0x00000005UL)) | 0x00000000UL;
    //RTC->LXTCTL   = (RTC->LXTCTL   & ~(0x00000080UL)) | 0x00000000UL;

    /* Enable base clock */
    NuCodeGenProj_init_base();

    /* Enable module clock and set clock source */
    NuCodeGenProj_init_adc();
    NuCodeGenProj_init_i2c0();
    NuCodeGenProj_init_isp();
    NuCodeGenProj_init_spi0();
    NuCodeGenProj_init_uart0();
    NuCodeGenProj_init_uart2();
    NuCodeGenProj_init_wdt();
    NuCodeGenProj_init_wwdt();

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    return;
}

/*** (C) COPYRIGHT 2016-2023 Nuvoton Technology Corp. ***/
