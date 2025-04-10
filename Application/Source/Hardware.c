//*************************************************************************
// 												Hardware declaration
// 								Author : ARMAN
//								Dated  : 1st Jan 2022 
//*************************************************************************

/********************
MCU:M031LE3AE(LQFP48)
Base Clocks:
LIRC:38.4kHz
HIRC:48MHz
PLL:96MHz
HCLK:48MHz
PCLK0:48MHz
PCLK1:48MHz
Enabled-Module Frequencies:
I2C0=Bus Clock(PCLK0):48MHz
ISP=Bus Clock(HCLK):48MHz/Engine Clock:48MHz
SPI0=Bus Clock(PCLK1):48MHz/Engine Clock:48MHz
SYSTICK=Bus Clock(HCLK):48MHz/Engine Clock:24MHz
TMR0=Bus Clock(PCLK0):48MHz/Engine Clock:48MHz
UART0=Bus Clock(PCLK0):48MHz/Engine Clock:48MHz
UART1=Bus Clock(PCLK1):48MHz/Engine Clock:48MHz
UART2=Bus Clock(PCLK0):48MHz/Engine Clock:48MHz
WDT=Bus Clock(PCLK0):48MHz/Engine Clock:38.4kHz
WWDT=Bus Clock(PCLK0):48MHz/Engine Clock:23.4375kHz
********************/


#include "Hardware.h"
#include "DS1307.h"


PepheralTypedef PeriPheralVal;
uint8_t IsRTCSet;
uint16_t ADCVal[4];


void SYS_Init(void)
{
    /* Unlock protected registers */
  SYS_UnlockReg();

    /* Enable HIRC */
   // CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
	CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk);
	CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
    /* Waiting for HIRC clock ready */
    //CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
	CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk);
	
	/* Enable LIRC clock */
//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

//    /* Waiting for 38.4 kHz clock ready */
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
	
	
	// PLL Setting --------
	CLK_DisablePLL();
	/* Set PLL frequency */
  CLK->PLLCTL = (CLK->PLLCTL & ~(0x000FFFFFUL)) | 0x0008C03EUL;
	/* Waiting for PLL ready */
  CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
	// ---------------------
	
    /* Switch HCLK clock source to HIRC */
    //CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    //CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);
	CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);
		
//    /* Switch UART0 clock source to HIRC */
//    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

//    /* Enable UART peripheral clock */
//    CLK_EnableModuleClock(UART0_MODULE);
	CLK_EnableModuleClock(SPI0_MODULE);
  CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
	
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HCLK, CLK_GetHCLKFreq() / 1000);

//	CLK_EnableModuleClock(UART0_MODULE);
//  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
	CLK_EnableModuleClock(UART1_MODULE);
  CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_PCLK1, CLK_CLKDIV0_UART1(1));
	
	CLK_EnableModuleClock(UART2_MODULE);
  CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_PCLK0, CLK_CLKDIV4_UART2(1));
//	
	CLK_EnableModuleClock(I2C0_MODULE);

CLK_EnableModuleClock(ISP_MODULE);
//	CLK_EnableModuleClock(TMR0_MODULE);
//	
//	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_LIRC, 576);
	
//	CLK_EnableModuleClock(ADC_MODULE);
//	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
  SystemCoreClockUpdate();

  SYS->GPA_MFPH = SYS_GPA_MFPH_PA11MFP_GPIO | SYS_GPA_MFPH_PA9MFP_GPIO | SYS_GPA_MFPH_PA8MFP_GPIO;
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA3MFP_GPIO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA0MFP_SPI0_MOSI;
    SYS->GPB_MFPH = SYS_GPB_MFPH_PB14MFP_ADC0_CH14 | SYS_GPB_MFPH_PB13MFP_ADC0_CH13;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB7MFP_GPIO | SYS_GPB_MFPL_PB5MFP_UART2_TXD | SYS_GPB_MFPL_PB4MFP_UART2_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB0MFP_ADC0_CH0;
    SYS->GPC_MFPH = 0x00000000;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC4MFP_GPIO;
    SYS->GPF_MFPH = SYS_GPF_MFPH_PF15MFP_GPIO;
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF5MFP_GPIO | SYS_GPF_MFPL_PF4MFP_GPIO | SYS_GPF_MFPL_PF3MFP_UART0_TXD | SYS_GPF_MFPL_PF2MFP_UART0_RXD | SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT;

		GPIO_DISABLE_DIGITAL_PATH(ADC1_PORT,ADC1_PIN);
		GPIO_DISABLE_DIGITAL_PATH(ADC2_PORT,ADC2_PIN);
		GPIO_DISABLE_DIGITAL_PATH(VMAIN_PORT,VMAIN_PIN);
		GPIO_DISABLE_DIGITAL_PATH(VSEN_PORT,VSEN_PIN);
    /* Lock protected registers */
    SYS_LockReg();
}

static void IOInit(void)
{
	GPIO_SetMode(IP1_PORT, IP1_PIN, GPIO_MODE_INPUT);
	GPIO_SetMode(IP2_PORT, IP2_PIN, GPIO_MODE_INPUT);
}


static void ADCInit(void)
{
	ADC_POWER_ON(ADC);

	ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, ADC1_PIN|ADC2_PIN|VMAIN_PIN|VSEN_PIN);
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
	
}

void UpdateADC(void)
{
	uint16_t i;
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
  ADC_START_CONV(ADC);
	while (ADC_GET_INT_FLAG(ADC, ADC_ADF_INT)==0);
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); 

	ADCVal[0]=ADC_GET_CONVERSION_DATA(ADC, 1);
	ADCVal[1]=ADC_GET_CONVERSION_DATA(ADC, 0);
	ADCVal[2]=ADC_GET_CONVERSION_DATA(ADC, 13);
	ADCVal[3]=ADC_GET_CONVERSION_DATA(ADC, 14);

	ADC_STOP_CONV(ADC);
}

#define METER_DEF_TIMEOUT				2000
void PeripheralInit(void)
{
	IOInit();
//	ADCInit();
	
}





void ProcessPeripheral(void)
{
	//UpdateADC();
	PeriPheralVal.IP1=IP1_VAL;
	PeriPheralVal.IP2=IP2_VAL;
	PeriPheralVal.ADCVal1=ADCVal[0];
	PeriPheralVal.ADCVal2=ADCVal[1];
	PeriPheralVal.MainVolt=ADCVal[2];
	PeriPheralVal.BattVolt=ADCVal[3];

}



