# 1 "../Source/MComm.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 349 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "../Source/MComm.c" 2
# 1 "../Header\\MComm.h" 1



# 1 "../../Library/Device/Nuvoton/M031/Include\\NuMicro.h" 1
# 12 "../../Library/Device/Nuvoton/M031/Include\\NuMicro.h"
# 1 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 1
# 74 "../../Library/Device/Nuvoton/M031/Include/M031Series.h"
typedef enum IRQn
{

    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    SVCall_IRQn = -5,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,


    BOD_IRQn = 0,
    WDT_IRQn = 1,
    EINT024_IRQn = 2,
    EINT135_IRQn = 3,
    GPIO_PAPB_IRQn = 4,
    GPIO_PAPBPGPH_IRQn = 4,
    GPIO_PCPDPEPF_IRQn = 5,
    PWM0_IRQn = 6,
    PWM1_IRQn = 7,
    TMR0_IRQn = 8,
    TMR1_IRQn = 9,
    TMR2_IRQn = 10,
    TMR3_IRQn = 11,
    UART02_IRQn = 12,
    UART1_IRQn = 13,
    UART13_IRQn = 13,
    SPI0_IRQn = 14,
    QSPI0_IRQn = 15,
    ISP_IRQn = 16,
    UART57_IRQn = 17,
    I2C0_IRQn = 18,
    I2C1_IRQn = 19,
    BPWM0_IRQn = 20,
    BPWM1_IRQn = 21,
    USCI_IRQn = 22,
    USCI01_IRQn = 22,
    USBD_IRQn = 23,
    ACMP01_IRQn = 25,
    PDMA_IRQn = 26,
    UART46_IRQn = 27,
    PWRWU_IRQn = 28,
    ADC_IRQn = 29,
    CKFAIL_IRQn = 30,
    RTC_IRQn = 31,
} IRQn_Type;
# 134 "../../Library/Device/Nuvoton/M031/Include/M031Series.h"
# 1 "../../Library/CMSIS/Include\\core_cm0.h" 1
# 39 "../../Library/CMSIS/Include\\core_cm0.h" 3





# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 1 3
# 56 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 3
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int int64_t;


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef signed short int int_least16_t;
typedef signed int int_least32_t;
typedef signed long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
typedef unsigned long long int uint_least64_t;




typedef signed int int_fast8_t;
typedef signed int int_fast16_t;
typedef signed int int_fast32_t;
typedef signed long long int int_fast64_t;


typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef unsigned long long int uint_fast64_t;






typedef signed int intptr_t;
typedef unsigned int uintptr_t;



typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
# 45 "../../Library/CMSIS/Include\\core_cm0.h" 2 3
# 163 "../../Library/CMSIS/Include\\core_cm0.h" 3
# 1 "../../Library/CMSIS/Include/core_cmInstr.h" 1 3
# 39 "../../Library/CMSIS/Include/core_cmInstr.h" 3
# 57 "../../Library/CMSIS/Include/core_cmInstr.h" 3
# 1 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 1 3
# 50 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__((always_inline)) static __inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}







__attribute__((always_inline)) static __inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 102 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 126 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 156 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 186 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 216 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return(result);
}
# 246 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : "sp");
}
# 270 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
# 300 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : "sp");
}
# 324 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 354 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 836 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline int32_t __REVSH(int32_t value)
{
  int32_t result;

  __asm volatile ("revsh %0, %1" : "=l" (result) : "l" (value) );
  return(result);
}
# 852 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  return (op1 >> op2) | (op1 << (32U - op2));
}
# 875 "../../Library/CMSIS/Include/cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;




  int32_t s = 4 * 8 - 1;

  result = value;
  for (value >>= 1U; value; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;

  return(result);
}
# 58 "../../Library/CMSIS/Include/core_cmInstr.h" 2 3
# 164 "../../Library/CMSIS/Include\\core_cm0.h" 2 3
# 1 "../../Library/CMSIS/Include/core_cmFunc.h" 1 3
# 39 "../../Library/CMSIS/Include/core_cmFunc.h" 3
# 165 "../../Library/CMSIS/Include\\core_cm0.h" 2 3
# 247 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t _reserved0:28;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 277 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 295 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:15;
    uint32_t T:1;
    uint32_t _reserved1:3;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 334 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t _reserved0:1;
    uint32_t SPSEL:1;
    uint32_t _reserved1:30;
  } b;
  uint32_t w;
} CONTROL_Type;
# 362 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef struct
{
  volatile uint32_t ISER[1U];
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];
} NVIC_Type;
# 389 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];
  volatile uint32_t SHCSR;
} SCB_Type;
# 496 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 629 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}







static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}
# 653 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}







static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}







static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}
# 688 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}
# 712 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}






static __inline void NVIC_SystemReset(void)
{
  __builtin_arm_dsb(0xF);;

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  __builtin_arm_dsb(0xF);;

  for(;;)
  {
    __builtin_arm_nop();
  }
}
# 769 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 135 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/system_M031Series.h" 1
# 50 "../../Library/Device/Nuvoton/M031/Include/system_M031Series.h"
extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
extern uint32_t PllClock;
# 84 "../../Library/Device/Nuvoton/M031/Include/system_M031Series.h"
extern void SystemInit(void);
# 97 "../../Library/Device/Nuvoton/M031/Include/system_M031Series.h"
extern void SystemCoreClockUpdate(void);
# 136 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 152 "../../Library/Device/Nuvoton/M031/Include/M031Series.h"
extern void SystemInit(void);








# 1 "../../Library/Device/Nuvoton/M031/Include/acmp_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/acmp_reg.h"
typedef struct
{
# 129 "../../Library/Device/Nuvoton/M031/Include/acmp_reg.h"
    volatile uint32_t CTL[2];
    volatile uint32_t STATUS;
    volatile uint32_t VREF;
    volatile uint32_t CALCTL;
    volatile const uint32_t CALSR;

} ACMP_T;
# 161 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/adc_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/adc_reg.h"
typedef struct
{
# 251 "../../Library/Device/Nuvoton/M031/Include/adc_reg.h"
    volatile const uint32_t ADDR[30];
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t ADCR;
    volatile uint32_t ADCHER;
    volatile uint32_t ADCMPR[2];
    volatile uint32_t ADSR0;
    volatile const uint32_t ADSR1;
    volatile const uint32_t ADSR2;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t ESMPCTL;
    volatile uint32_t CFDCTL;
    volatile const uint32_t RESERVE3[22];
    volatile const uint32_t ADPDMA;
    volatile const uint32_t RESERVE4[31];
    volatile uint32_t ADCALR;
    volatile uint32_t ADCALSTSR;
    volatile uint32_t ADCALDBR;
} ADC_T;
# 162 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/clk_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/clk_reg.h"
typedef struct
{
# 520 "../../Library/Device/Nuvoton/M031/Include/clk_reg.h"
    volatile uint32_t PWRCTL;
    volatile uint32_t AHBCLK;
    volatile uint32_t APBCLK0;
    volatile uint32_t APBCLK1;
    volatile uint32_t CLKSEL0;
    volatile uint32_t CLKSEL1;
    volatile uint32_t CLKSEL2;
    volatile uint32_t CLKSEL3;
    volatile uint32_t CLKDIV0;
    volatile const uint32_t RESERVE0[3];
    volatile uint32_t CLKDIV4;
    volatile uint32_t PCLKDIV;
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t PLLCTL;
    volatile const uint32_t RESERVE2[3];
    volatile const uint32_t STATUS;
    volatile const uint32_t RESERVE3[3];
    volatile uint32_t CLKOCTL;
    volatile const uint32_t RESERVE4[3];
    volatile uint32_t CLKDCTL;
    volatile uint32_t CLKDSTS;
    volatile uint32_t CDUPB;
    volatile uint32_t CDLOWB;
    volatile uint32_t LDOCTL;
    volatile const uint32_t RESERVE5[12];
    volatile uint32_t HXTFSEL;
    volatile const uint32_t RESERVE9[14];
    volatile uint32_t TESTCLK;

} CLK_T;
# 163 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/crc_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/crc_reg.h"
typedef struct
{
# 97 "../../Library/Device/Nuvoton/M031/Include/crc_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t DAT;
    volatile uint32_t SEED;
    volatile const uint32_t CHECKSUM;

} CRC_T;
# 164 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/ebi_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/ebi_reg.h"
typedef struct
{
# 164 "../../Library/Device/Nuvoton/M031/Include/ebi_reg.h"
    volatile uint32_t CTL0;
    volatile uint32_t TCTL0;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t CTL1;
    volatile uint32_t TCTL1;

} EBI_T;
# 165 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/fmc_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/fmc_reg.h"
typedef struct
{
# 180 "../../Library/Device/Nuvoton/M031/Include/fmc_reg.h"
    volatile uint32_t ISPCTL;
    volatile uint32_t ISPADDR;
    volatile uint32_t ISPDAT;
    volatile uint32_t ISPCMD;
    volatile uint32_t ISPTRG;
    volatile const uint32_t DFBA;
    volatile uint32_t FTCTL;
    volatile uint32_t ICPCTL;
    volatile const uint32_t RESERVE0[8];
    volatile uint32_t ISPSTS;
    volatile const uint32_t RESERVE1[15];
    volatile uint32_t MPDAT0;
    volatile uint32_t MPDAT1;
    volatile uint32_t MPDAT2;
    volatile uint32_t MPDAT3;
    volatile const uint32_t RESERVE2[12];
    volatile const uint32_t MPSTS;
    volatile const uint32_t MPADDR;
    volatile const uint32_t RESERVE3[0x3CD];
    volatile const uint32_t VERSION;
} FMC_T;
# 166 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/gpio_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/gpio_reg.h"
typedef struct
{
# 154 "../../Library/Device/Nuvoton/M031/Include/gpio_reg.h"
    volatile uint32_t MODE;
    volatile uint32_t DINOFF;
    volatile uint32_t DOUT;
    volatile uint32_t DATMSK;
    volatile const uint32_t PIN;
    volatile uint32_t DBEN;
    volatile uint32_t INTTYPE;
    volatile uint32_t INTEN;
    volatile uint32_t INTSRC;
} GPIO_T;

typedef struct
{
# 201 "../../Library/Device/Nuvoton/M031/Include/gpio_reg.h"
    volatile uint32_t DBCTL;
} GPIO_DBCTL_T;
# 167 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h"
typedef struct
{
# 40 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h"
    volatile uint32_t DIVIDEND;
# 53 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h"
    volatile uint32_t DIVISOR;
# 65 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h"
    volatile uint32_t QUOTIENT;
# 78 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h"
    volatile uint32_t REM;
# 94 "../../Library/Device/Nuvoton/M031/Include/hdiv_reg.h"
    volatile const uint32_t STATUS;

} HDIV_T;
# 168 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/i2c_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/i2c_reg.h"
typedef struct
{
# 481 "../../Library/Device/Nuvoton/M031/Include/i2c_reg.h"
    volatile uint32_t CTL0;
    volatile uint32_t ADDR0;
    volatile uint32_t DAT;
    volatile const uint32_t STATUS0;
    volatile uint32_t CLKDIV;
    volatile uint32_t TOCTL;
    volatile uint32_t ADDR1;
    volatile uint32_t ADDR2;
    volatile uint32_t ADDR3;
    volatile uint32_t ADDRMSK0;
    volatile uint32_t ADDRMSK1;
    volatile uint32_t ADDRMSK2;
    volatile uint32_t ADDRMSK3;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t CTL1;
    volatile uint32_t STATUS1;
    volatile uint32_t TMCTL;
    volatile uint32_t BUSCTL;
    volatile uint32_t BUSTCTL;
    volatile uint32_t BUSSTS;
    volatile uint32_t PKTSIZE;
    volatile const uint32_t PKTCRC;
    volatile uint32_t BUSTOUT;
    volatile uint32_t CLKTOUT;
} I2C_T;
# 169 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/pdma_reg.h" 1
# 27 "../../Library/Device/Nuvoton/M031/Include/pdma_reg.h"
typedef struct
{
# 113 "../../Library/Device/Nuvoton/M031/Include/pdma_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t SA;
    volatile uint32_t DA;
    volatile uint32_t NEXT;

} DSCT_T;


typedef struct
{
# 448 "../../Library/Device/Nuvoton/M031/Include/pdma_reg.h"
    DSCT_T DSCT[9];
    volatile const uint32_t RESERVE0[28];
    volatile const uint32_t CURSCAT[9];
    volatile const uint32_t RESERVE1[183];
    volatile uint32_t CHCTL;
    volatile uint32_t PAUSE;
    volatile uint32_t SWREQ;
    volatile const uint32_t TRGSTS;
    volatile uint32_t PRISET;
    volatile uint32_t PRICLR;
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile uint32_t ABTSTS;
    volatile uint32_t TDSTS;
    volatile uint32_t ALIGN;
    volatile const uint32_t TACTSTS;
    volatile uint32_t TOUTPSC;
    volatile uint32_t TOUTEN;
    volatile uint32_t TOUTIEN;
    volatile uint32_t SCATBA;
    volatile uint32_t TOC0_1;
    volatile const uint32_t RESERVE2[7];
    volatile uint32_t CHRST;
    volatile const uint32_t RESERVE3[7];
    volatile uint32_t REQSEL0_3;
    volatile uint32_t REQSEL4_7;
    volatile uint32_t REQSEL8;
} PDMA_T;
# 170 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/pwm_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/pwm_reg.h"
typedef struct
{
# 1165 "../../Library/Device/Nuvoton/M031/Include/pwm_reg.h"
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t CLKSRC;
    volatile uint32_t CLKPSC[3];
    volatile uint32_t CNTEN;
    volatile uint32_t CNTCLR;
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t PERIOD[6];
    volatile const uint32_t RESERVE2[2];
    volatile uint32_t CMPDAT[6];
    volatile const uint32_t RESERVE3[2];
    volatile uint32_t DTCTL[3];
    volatile const uint32_t RESERVE4[5];
    volatile const uint32_t CNT[6];
    volatile const uint32_t RESERVE5[2];
    volatile uint32_t WGCTL0;
    volatile uint32_t WGCTL1;
    volatile uint32_t MSKEN;
    volatile uint32_t MSK;
    volatile uint32_t BNF;
    volatile uint32_t FAILBRK;
    volatile uint32_t BRKCTL[3];
    volatile uint32_t POLCTL;
    volatile uint32_t POEN;
    volatile uint32_t SWBRK;
    volatile uint32_t INTEN0;
    volatile uint32_t INTEN1;
    volatile uint32_t INTSTS0;
    volatile uint32_t INTSTS1;
    volatile const uint32_t RESERVE6[2];
    volatile uint32_t ADCTS0;
    volatile uint32_t ADCTS1;
    volatile const uint32_t RESERVE7[4];
    volatile uint32_t SSCTL;
    volatile uint32_t SSTRG;
    volatile const uint32_t RESERVE8[2];
    volatile uint32_t STATUS;
    volatile const uint32_t RESERVE9[55];
    volatile uint32_t CAPINEN;
    volatile uint32_t CAPCTL;
    volatile const uint32_t CAPSTS;
    volatile const uint32_t RCAPDAT0;
    volatile const uint32_t FCAPDAT0;
    volatile const uint32_t RCAPDAT1;
    volatile const uint32_t FCAPDAT1;
    volatile const uint32_t RCAPDAT2;
    volatile const uint32_t FCAPDAT2;
    volatile const uint32_t RCAPDAT3;
    volatile const uint32_t FCAPDAT3;
    volatile const uint32_t RCAPDAT4;
    volatile const uint32_t FCAPDAT4;
    volatile const uint32_t RCAPDAT5;
    volatile const uint32_t FCAPDAT5;
    volatile uint32_t PDMACTL;
    volatile const uint32_t PDMACAP0_1;
    volatile const uint32_t PDMACAP2_3;
    volatile const uint32_t PDMACAP4_5;
    volatile const uint32_t RESERVE10[1];
    volatile uint32_t CAPIEN;
    volatile uint32_t CAPIF;
    volatile const uint32_t RESERVE11[43];
    volatile const uint32_t PBUF[6];
    volatile const uint32_t CMPBUF[6];
} PWM_T;
# 171 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/bpwm_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/bpwm_reg.h"
typedef struct
{
# 44 "../../Library/Device/Nuvoton/M031/Include/bpwm_reg.h"
    volatile uint32_t RCAPDAT;
    volatile uint32_t FCAPDAT;
} BCAPDAT_T;

typedef struct
{
# 1084 "../../Library/Device/Nuvoton/M031/Include/bpwm_reg.h"
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t CLKSRC;
    volatile uint32_t CLKPSC;
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t CNTEN;
    volatile uint32_t CNTCLR;
    volatile const uint32_t RESERVE2[2];
    volatile uint32_t PERIOD;
    volatile const uint32_t RESERVE3[7];
    volatile uint32_t CMPDAT[6];
    volatile const uint32_t RESERVE4[10];
    volatile const uint32_t CNT;
    volatile const uint32_t RESERVE5[7];
    volatile uint32_t WGCTL0;
    volatile uint32_t WGCTL1;
    volatile uint32_t MSKEN;
    volatile uint32_t MSK;
    volatile const uint32_t RESERVE6[5];
    volatile uint32_t POLCTL;
    volatile uint32_t POEN;
    volatile const uint32_t RESERVE7[1];
    volatile uint32_t INTEN;
    volatile const uint32_t RESERVE8[1];
    volatile uint32_t INTSTS;
    volatile const uint32_t RESERVE9[3];
    volatile uint32_t EADCTS0;
    volatile uint32_t EADCTS1;
    volatile const uint32_t RESERVE10[4];
    volatile uint32_t SSCTL;
    volatile uint32_t SSTRG;
    volatile const uint32_t RESERVE11[2];
    volatile uint32_t STATUS;
    volatile const uint32_t RESERVE12[55];
    volatile uint32_t CAPINEN;
    volatile uint32_t CAPCTL;
    volatile const uint32_t CAPSTS;
    BCAPDAT_T CAPDAT[6];
    volatile const uint32_t RESERVE13[5];
    volatile uint32_t CAPIEN;
    volatile uint32_t CAPIF;
    volatile const uint32_t RESERVE14[43];
    volatile const uint32_t PBUF;
    volatile const uint32_t RESERVE15[5];
    volatile const uint32_t CMPBUF[6];

} BPWM_T;
# 172 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/qspi_reg.h" 1
# 25 "../../Library/Device/Nuvoton/M031/Include/qspi_reg.h"
typedef struct
{
# 360 "../../Library/Device/Nuvoton/M031/Include/qspi_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t CLKDIV;
    volatile uint32_t SSCTL;
    volatile uint32_t PDMACTL;
    volatile uint32_t FIFOCTL;
    volatile uint32_t STATUS;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t TX;
    volatile const uint32_t RESERVE1[3];
    volatile const uint32_t RX;

} QSPI_T;
# 173 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/spi_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/spi_reg.h"
typedef struct
{
# 481 "../../Library/Device/Nuvoton/M031/Include/spi_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t CLKDIV;
    volatile uint32_t SSCTL;
    volatile uint32_t PDMACTL;
    volatile uint32_t FIFOCTL;
    volatile uint32_t STATUS;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t TX;
    volatile const uint32_t RESERVE1[3];
    volatile const uint32_t RX;
    volatile const uint32_t RESERVE2[11];
    volatile uint32_t I2SCTL;
    volatile uint32_t I2SCLK;
    volatile uint32_t I2SSTS;

} SPI_T;
# 174 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/sys_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/sys_reg.h"
typedef struct
{
# 579 "../../Library/Device/Nuvoton/M031/Include/sys_reg.h"
    volatile const uint32_t PDID;
    volatile uint32_t RSTSTS;
    volatile uint32_t IPRST0;
    volatile uint32_t IPRST1;
    volatile uint32_t IPRST2;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t BODCTL;
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t PORCTL;
    volatile const uint32_t RESERVE2[2];
    volatile uint32_t GPA_MFPL;
    volatile uint32_t GPA_MFPH;
    volatile uint32_t GPB_MFPL;
    volatile uint32_t GPB_MFPH;
    volatile uint32_t GPC_MFPL;
    volatile uint32_t GPC_MFPH;
    volatile uint32_t GPD_MFPL;
    volatile uint32_t GPD_MFPH;
    volatile uint32_t GPE_MFPL;
    volatile uint32_t GPE_MFPH;
    volatile uint32_t GPF_MFPL;
    volatile uint32_t GPF_MFPH;
    volatile uint32_t GPG_MFPL;
    volatile uint32_t GPG_MFPH;
    volatile uint32_t GPH_MFPL;
    volatile uint32_t GPH_MFPH;
    volatile const uint32_t RESERVE3[2];
    volatile uint32_t LPLDOCTL;
    volatile const uint32_t RESERVE4[17];
    volatile uint32_t MODCTL;
    volatile const uint32_t RESERVE5[3];
    volatile uint32_t SRAM_BISTCTL;
    volatile const uint32_t SRAM_BISTSTS;
    volatile uint32_t SRAM_PARITY;
    volatile uint32_t SRAM_INTCTL;
    volatile uint32_t SRAM_STATUS;
    volatile const uint32_t SRAM_ERRADDR;
    volatile const uint32_t RESERVE6[2];
    volatile uint32_t HIRCTRIMCTL;
    volatile uint32_t HIRCTRIMIEN;
    volatile uint32_t HIRCTRIMSTS;
    volatile const uint32_t RESERVE7[1];
    volatile uint32_t REGLCTL;
    volatile const uint32_t RESERVE8[5];
    volatile uint32_t HIRCADJ;
    volatile const uint32_t RESERVE9[1];
    volatile const uint32_t LDOTRIM;
    volatile const uint32_t LVR16TRIM;
    volatile const uint32_t RESERVE10[4];
    volatile const uint32_t LIRCT;
    volatile const uint32_t RESERVE11[5];
    volatile const uint32_t LVR17TRIM;
    volatile const uint32_t LVR20TRIM;
    volatile const uint32_t LVR25TRIM;
    volatile const uint32_t uLDOVITRIM;
    volatile uint32_t LVRITRIMSEL;
    volatile const uint32_t RESERVE12[9];
    volatile uint32_t HIRCTCTL;
    volatile uint32_t ADCCHIP;
    volatile uint32_t HXTTCTL;
    volatile const uint32_t RESERVE13[22];
    volatile uint32_t PORDISAN;
} SYS_T;

typedef struct
{
# 743 "../../Library/Device/Nuvoton/M031/Include/sys_reg.h"
    volatile uint32_t NMIEN;
    volatile const uint32_t NMISTS;

} NMI_T;
# 175 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/rtc_reg.h" 1
# 24 "../../Library/Device/Nuvoton/M031/Include/rtc_reg.h"
typedef struct
{
# 234 "../../Library/Device/Nuvoton/M031/Include/rtc_reg.h"
    volatile uint32_t INIT;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t FREQADJ;
    volatile uint32_t TIME;
    volatile uint32_t CAL;
    volatile uint32_t CLKFMT;
    volatile uint32_t WEEKDAY;
    volatile uint32_t TALM;
    volatile uint32_t CALM;
    volatile const uint32_t LEAPYEAR;
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile uint32_t TICK;
    volatile uint32_t TAMSK;
    volatile uint32_t CAMSK;
    volatile const uint32_t RESERVE1[49];
    volatile uint32_t LXTCTL;

} RTC_T;
# 176 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/timer_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/timer_reg.h"
typedef struct
{
# 218 "../../Library/Device/Nuvoton/M031/Include/timer_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t CMP;
    volatile uint32_t INTSTS;
    volatile const uint32_t CNT;
    volatile const uint32_t CAP;
    volatile uint32_t EXTCTL;
    volatile uint32_t EINTSTS;
} TIMER_T;
# 177 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/uart_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/uart_reg.h"
typedef struct
{
# 702 "../../Library/Device/Nuvoton/M031/Include/uart_reg.h"
    volatile uint32_t DAT;
    volatile uint32_t INTEN;
    volatile uint32_t FIFO;
    volatile uint32_t LINE;
    volatile uint32_t MODEM;
    volatile uint32_t MODEMSTS;
    volatile uint32_t FIFOSTS;
    volatile uint32_t INTSTS;
    volatile uint32_t TOUT;
    volatile uint32_t BAUD;
    volatile uint32_t IRDA;
    volatile uint32_t ALTCTL;
    volatile uint32_t FUNCSEL;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t BRCOMP;
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t DWKCOMP;

} UART_T;
# 178 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/ui2c_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/ui2c_reg.h"
typedef struct
{
# 367 "../../Library/Device/Nuvoton/M031/Include/ui2c_reg.h"
    volatile uint32_t CTL;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE1[8];
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile const uint32_t RESERVE2[3];
    volatile uint32_t DEVADDR0;
    volatile uint32_t DEVADDR1;
    volatile uint32_t ADDRMSK0;
    volatile uint32_t ADDRMSK1;
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;
    volatile const uint32_t RESERVE3[8];
    volatile uint32_t ADMAT;
    volatile uint32_t TMCTL;

} UI2C_T;
# 179 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/usbd_reg.h" 1
# 28 "../../Library/Device/Nuvoton/M031/Include/usbd_reg.h"
typedef struct
{
# 96 "../../Library/Device/Nuvoton/M031/Include/usbd_reg.h"
    volatile uint32_t BUFSEG;
    volatile uint32_t MXPLD;
    volatile uint32_t CFG;
    volatile uint32_t CFGP;

} USBD_EP_T;

typedef struct
{
# 356 "../../Library/Device/Nuvoton/M031/Include/usbd_reg.h"
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile uint32_t FADDR;
    volatile const uint32_t EPSTS;
    volatile uint32_t ATTR;
    volatile const uint32_t VBUSDET;
    volatile uint32_t STBUFSEG;

    volatile const uint32_t RESERVE0[1];

    volatile const uint32_t EPSTS0;

    volatile const uint32_t RESERVE1[25];

    volatile const uint32_t LPMATTR;
    volatile const uint32_t FN;
    volatile uint32_t SE0;

    volatile const uint32_t RESERVE2[283];

    USBD_EP_T EP[8];

} USBD_T;
# 180 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/uspi_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/uspi_reg.h"
typedef struct
{
# 433 "../../Library/Device/Nuvoton/M031/Include/uspi_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t INTEN;
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
    volatile uint32_t PDMACTL;
    volatile const uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;

} USPI_T;
# 181 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/uuart_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/uuart_reg.h"
typedef struct
{
# 417 "../../Library/Device/Nuvoton/M031/Include/uuart_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t INTEN;
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
    volatile uint32_t PDMACTL;
    volatile const uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;

} UUART_T;
# 182 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/wdt_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/wdt_reg.h"
typedef struct
{
# 118 "../../Library/Device/Nuvoton/M031/Include/wdt_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t ALTCTL;
    volatile uint32_t RSTCNT;

} WDT_T;
# 183 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/Device/Nuvoton/M031/Include/wwdt_reg.h" 1
# 26 "../../Library/Device/Nuvoton/M031/Include/wwdt_reg.h"
typedef struct
{
# 101 "../../Library/Device/Nuvoton/M031/Include/wwdt_reg.h"
    volatile uint32_t RLDCNT;
    volatile uint32_t CTL;
    volatile uint32_t STATUS;
    volatile const uint32_t CNT;

} WWDT_T;
# 184 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 355 "../../Library/Device/Nuvoton/M031/Include/M031Series.h"
typedef volatile unsigned char vu8;
typedef volatile unsigned long vu32;
typedef volatile unsigned short vu16;
# 563 "../../Library/Device/Nuvoton/M031/Include/M031Series.h"
# 1 "../../Library/StdDriver/inc\\sys.h" 1
# 1343 "../../Library/StdDriver/inc\\sys.h"
static __inline void SYS_UnlockReg(void)
{
    do {
        ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0x59;
        ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0x16;
        ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0x88;
    } while (((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL == 0);
}
# 1360 "../../Library/StdDriver/inc\\sys.h"
static __inline void SYS_LockReg(void)
{
    ((SYS_T *) ((( uint32_t)0x40000000) + 0x00000))->REGLCTL = 0;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
# 564 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\clk.h" 1
# 444 "../../Library/StdDriver/inc\\clk.h"
extern int32_t g_CLK_i32ErrCode;
# 461 "../../Library/StdDriver/inc\\clk.h"
static __inline uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};
    uint32_t u32Reg;

    u32PllFreq = 0;
    u32Reg = ((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->PLLCTL;

    if ((u32Reg & ((0x1ul << (16)) | (0x1ul << (18)))) == 0)
    {

        if (u32Reg & (0x1ul << (19)))
        {
            u32FIN = ((48000000UL) >> 2);
        } else
            u32FIN = (32000000UL);

        if (u32Reg & (0x1ul << (17)))
        {

            u32PllFreq = u32FIN;
        }
        else
        {

            u32NO = au8NoTbl[((u32Reg & (0x3ul << (14))) >> (14))];
            u32NF = ((u32Reg & (0x1fful << (0))) >> (0)) + 2;
            u32NR = ((u32Reg & (0x1ful << (9))) >> (9)) + 2;

            u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);
        }
    }

    return u32PllFreq;
}
# 511 "../../Library/StdDriver/inc\\clk.h"
static __inline int32_t CLK_SysTickDelay(uint32_t us)
{

    uint32_t u32TimeOutCnt = SystemCoreClock * 2;

    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = (0x00);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) | (1UL );


    while ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            break;
        }
    }


    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;

    if(u32TimeOutCnt == 0)
        return (-1);
    else
        return 0;
}
# 545 "../../Library/StdDriver/inc\\clk.h"
static __inline uint32_t CLK_GetUARTFreq(void)
{
    uint32_t u32Freqout, u32AHBDivider, u32ClkSel, PCLK0Div;

    u32Freqout = 0;
    u32ClkSel = ((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->CLKSEL1 & (0x7ul << (24)) ;

    if (u32ClkSel == (0x0UL<<(24)))
    {
        u32Freqout = (32000000UL);
    }
    else if(u32ClkSel == (0x1UL<<(24)))
    {
        u32Freqout = CLK_GetPLLClockFreq();
    }
    else if(u32ClkSel == (0x2UL<<(24)))
    {
        u32Freqout = (32768UL);
    }
    else if(u32ClkSel == (0x3UL<<(24)))
    {
        u32Freqout = (48000000UL);
    }
    else if(u32ClkSel == (0x4UL<<(24)))
    {
        PCLK0Div = (((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->PCLKDIV & (0x7ul << (0))) >> (0);
        u32Freqout = (SystemCoreClock >> PCLK0Div);
    }
    else if(u32ClkSel == (0x5UL<<(24)))
    {
        u32Freqout = (38400UL);
    }

    u32AHBDivider = (((CLK_T *) ((( uint32_t)0x40000000) + 0x00200))->CLKDIV0 & (0xful << (8))) + 1 ;

    return (u32Freqout/u32AHBDivider);
}


uint32_t CLK_WaitClockReady(uint32_t);
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHXTFreq(void);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_DisablePLL(void);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_DisableSysTick(void);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_PowerDown(void);
void CLK_Idle(void);
# 565 "../../Library/Device/Nuvoton/M031/Include/M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\acmp.h" 1
# 18 "../../Library/StdDriver/inc\\acmp.h"
# 1 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 1
# 19 "../../Library/StdDriver/inc\\acmp.h" 2
# 383 "../../Library/StdDriver/inc\\acmp.h"
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);
# 566 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\adc.h" 1
# 109 "../../Library/StdDriver/inc\\adc.h"
extern int32_t g_ADC_i32ErrCode;
# 395 "../../Library/StdDriver/inc\\adc.h"
void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_SetExtendSampleTime(ADC_T *adc,
                             uint32_t u32ModuleNum,
                             uint32_t u32ExtendSampleTime);
# 567 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\crc.h" 1
# 102 "../../Library/StdDriver/inc\\crc.h"
void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
uint32_t CRC_GetChecksum(void);
# 568 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\ebi.h" 1
# 244 "../../Library/StdDriver/inc\\ebi.h"
void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);
# 569 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\fmc.h" 1
# 129 "../../Library/StdDriver/inc\\fmc.h"
extern int32_t g_FMC_i32ErrCode;







static __inline uint32_t FMC_ReadCID(void);
static __inline uint32_t FMC_ReadPID(void);
static __inline uint32_t FMC_ReadUID(uint8_t u8Index);
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index);
static __inline int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr);
static __inline uint32_t FMC_GetVECMAP(void);
# 152 "../../Library/StdDriver/inc\\fmc.h"
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPSTS & (0x1ffffful << (9)));
}
# 166 "../../Library/StdDriver/inc\\fmc.h"
static __inline uint32_t FMC_ReadCID(void)
{
    uint32_t tout = ((SystemCoreClock/10)/4);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x0BUL;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = 0x0u;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = (0x1ul << (0));



    while (tout-- > 0)
    {
        if (!(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))))
        {
            if (((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT != 0xDA)
                g_FMC_i32ErrCode = -1;
            return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
        }
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;

}
# 201 "../../Library/StdDriver/inc\\fmc.h"
static __inline uint32_t FMC_ReadPID(void)
{
    uint32_t tout = ((SystemCoreClock/10)/4);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x0CUL;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = 0x04u;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = (0x1ul << (0));



    while (tout-- > 0)
    {
        if (!(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))))
            return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}
# 231 "../../Library/StdDriver/inc\\fmc.h"
static __inline uint32_t FMC_ReadUID(uint8_t u8Index)
{
    uint32_t tout = ((SystemCoreClock/10)/4);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x04UL;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = ((uint32_t)u8Index << 2u);
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT = 0u;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = 0x1u;



    while (tout-- > 0)
    {
        if (!(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))))
            return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}
# 262 "../../Library/StdDriver/inc\\fmc.h"
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    uint32_t tout = ((SystemCoreClock/10)/4);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x04UL;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = (0x04u * u32Index) + 0x10u;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = (0x1ul << (0));



    while (tout-- > 0)
    {
        if (!(((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG & (0x1ul << (0))))
            return ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPDAT;
    }
    g_FMC_i32ErrCode = -1;
    return 0xFFFFFFFF;
}
# 297 "../../Library/StdDriver/inc\\fmc.h"
static __inline int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    uint32_t tout = ((SystemCoreClock/10)/4);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPCMD = 0x2EUL;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPADDR = u32PageAddr;
    ((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG = 0x1u;



    while (tout-- > 0)
    {
        if (!((FMC_T *) ((( uint32_t)0x40000000) + 0x0C000))->ISPTRG)
            return 0;
    }
    g_FMC_i32ErrCode = -1;
    return -1;
}






extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_Erase_SPROM(void);
extern int32_t FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetBootSource(int32_t i32BootSrc);
extern int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1);
extern int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count);
extern uint32_t FMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
extern int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len);
extern int32_t FMC_RemapBank(uint32_t u32BankIdx);
# 570 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\gpio.h" 1
# 15 "../../Library/StdDriver/inc\\gpio.h"
# 1 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 1
# 16 "../../Library/StdDriver/inc\\gpio.h" 2
# 464 "../../Library/StdDriver/inc\\gpio.h"
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);
# 571 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\i2c.h" 1
# 61 "../../Library/StdDriver/inc\\i2c.h"
extern int32_t g_I2C_i32ErrCode;
# 471 "../../Library/StdDriver/inc\\i2c.h"
static __inline void I2C_STOP(I2C_T *i2c);
# 482 "../../Library/StdDriver/inc\\i2c.h"
static __inline void I2C_STOP(I2C_T *i2c)
{
    uint32_t u32TimeOutCount = SystemCoreClock;

    (i2c)->CTL0 |= ((0x1ul << (3)) | (0x1ul << (4)));
    while(i2c->CTL0 & (0x1ul << (4)))
    {
        u32TimeOutCount--;
        if(u32TimeOutCount == 0) break;
    }
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
uint8_t I2C_WriteByte(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t I2C_WriteMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_WriteByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_ReadByte(I2C_T *i2c, uint8_t u8SlaveAddr);
uint32_t I2C_ReadMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t I2C_ReadMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t I2C_ReadByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint32_t I2C_SMBusGetStatus(I2C_T *i2c);
void I2C_SMBusClearInterruptFlag(I2C_T *i2c, uint8_t u8SMBusIntFlag);
void I2C_SMBusSetPacketByteCount(I2C_T *i2c, uint32_t u32PktSize);
void I2C_SMBusOpen(I2C_T *i2c, uint8_t u8HostDevice);
void I2C_SMBusClose(I2C_T *i2c);
void I2C_SMBusPECTxEnable(I2C_T *i2c, uint8_t u8PECTxEn);
uint8_t I2C_SMBusGetPECValue(I2C_T *i2c);
void I2C_SMBusIdleTimeout(I2C_T *i2c, uint32_t us, uint32_t u32Hclk);
void I2C_SMBusTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);
void I2C_SMBusClockLoTimeout(I2C_T *i2c, uint32_t ms, uint32_t u32Pclk);
# 572 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\pdma.h" 1
# 332 "../../Library/StdDriver/inc\\pdma.h"
void PDMA_Open(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_Close(PDMA_T *pdma);
void PDMA_SetTransferCnt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetBurstType(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void PDMA_EnableTimeout(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_DisableTimeout(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_SetTimeOut(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(PDMA_T *pdma, uint32_t u32Ch);
void PDMA_EnableInt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Mask);
# 573 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\pwm.h" 1
# 453 "../../Library/StdDriver/inc\\pwm.h"
uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrake(PWM_T *pwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnablePDMA(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32RisingFirst, uint32_t u32Mode);
void PWM_DisablePDMA(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_DisableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
uint32_t PWM_GetFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_DisableZeroInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetZeroIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableLoadMode(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void PWM_DisableLoadMode(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void PWM_SetClockSource(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
void PWM_EnableBrakeNoiseFilter(PWM_T *pwm, uint32_t u32BrakePinNum, uint32_t u32ClkCnt, uint32_t u32ClkDivSel);
void PWM_DisableBrakeNoiseFilter(PWM_T *pwm, uint32_t u32BrakePinNum);
void PWM_EnableBrakePinInverse(PWM_T *pwm, uint32_t u32BrakePinNum);
void PWM_DisableBrakePinInverse(PWM_T *pwm, uint32_t u32BrakePinNum);
void PWM_SetBrakePinSource(PWM_T *pwm, uint32_t u32BrakePinNum, uint32_t u32SelAnotherModule);
uint32_t PWM_GetWrapAroundFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearWrapAroundFlag(PWM_T *pwm, uint32_t u32ChannelNum);
# 574 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\bpwm.h" 1
# 331 "../../Library/StdDriver/inc\\bpwm.h"
uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void BPWM_DisableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t BPWM_GetADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_DisableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_DisableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_SetClockSource(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
uint32_t BPWM_GetWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
# 575 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\qspi.h" 1
# 384 "../../Library/StdDriver/inc\\qspi.h"
uint32_t QSPI_Open(QSPI_T *qspi, uint32_t u32MasterSlave, uint32_t u32QSPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void QSPI_Close(QSPI_T *qspi);
void QSPI_ClearRxFIFO(QSPI_T *qspi);
void QSPI_ClearTxFIFO(QSPI_T *qspi);
void QSPI_DisableAutoSS(QSPI_T *qspi);
void QSPI_EnableAutoSS(QSPI_T *qspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t QSPI_SetBusClock(QSPI_T *qspi, uint32_t u32BusClock);
void QSPI_SetFIFO(QSPI_T *qspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t QSPI_GetBusClock(QSPI_T *qspi);
void QSPI_EnableInt(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_DisableInt(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetIntFlag(QSPI_T *qspi, uint32_t u32Mask);
void QSPI_ClearIntFlag(QSPI_T *qspi, uint32_t u32Mask);
uint32_t QSPI_GetStatus(QSPI_T *qspi, uint32_t u32Mask);
# 576 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\spi.h" 1
# 314 "../../Library/StdDriver/inc\\spi.h"
static __inline void SPII2S_ENABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0ul))
        i2s->I2SCTL |= (0x1ul << (16));
    else
        i2s->I2SCTL |= (0x1ul << (17));
}
# 331 "../../Library/StdDriver/inc\\spi.h"
static __inline void SPII2S_DISABLE_TX_ZCD(SPI_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == (0ul))
        i2s->I2SCTL &= ~(0x1ul << (16));
    else
        i2s->I2SCTL &= ~(0x1ul << (17));
}
# 444 "../../Library/StdDriver/inc\\spi.h"
static __inline void SPII2S_SET_MONO_RX_CHANNEL(SPI_T *i2s, uint32_t u32Ch)
{
    u32Ch == (0x1ul << (23)) ?
    (i2s->I2SCTL |= (0x1ul << (23))) :
    (i2s->I2SCTL &= ~(0x1ul << (23)));
}
# 506 "../../Library/StdDriver/inc\\spi.h"
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

uint32_t SPII2S_Open(SPI_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat);
void SPII2S_Close(SPI_T *i2s);
void SPII2S_EnableInt(SPI_T *i2s, uint32_t u32Mask);
void SPII2S_DisableInt(SPI_T *i2s, uint32_t u32Mask);
uint32_t SPII2S_EnableMCLK(SPI_T *i2s, uint32_t u32BusClock);
void SPII2S_DisableMCLK(SPI_T *i2s);
void SPII2S_SetFIFO(SPI_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
# 577 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\rtc.h" 1
# 126 "../../Library/StdDriver/inc\\rtc.h"
typedef struct
{
    uint32_t u32Year;
    uint32_t u32Month;
    uint32_t u32Day;
    uint32_t u32DayOfWeek;
    uint32_t u32Hour;
    uint32_t u32Minute;
    uint32_t u32Second;
    uint32_t u32TimeScale;
    uint32_t u32AmPm;
} S_RTC_TIME_DATA_T;
# 270 "../../Library/StdDriver/inc\\rtc.h"
void RTC_Open(S_RTC_TIME_DATA_T *psPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX10000);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDateMask(uint8_t u8IsTenYMsk, uint8_t u8IsYMsk, uint8_t u8IsTenMMsk, uint8_t u8IsMMsk, uint8_t u8IsTenDMsk, uint8_t u8IsDMsk);
void RTC_SetAlarmTimeMask(uint8_t u8IsTenHMsk, uint8_t u8IsHMsk, uint8_t u8IsTenMMsk, uint8_t u8IsMMsk, uint8_t u8IsTenSMsk, uint8_t u8IsSMsk);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
# 578 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\hdiv.h" 1
# 43 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int32_t HDIV_Div(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)((( uint32_t)0x40000000) + 0x14000);
    *p32++ = x;
    *p32++ = y;
    return *p32;
}
# 64 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int16_t HDIV_Mod(int32_t x, int16_t y)
{
    uint32_t *p32;

    p32 = (uint32_t *)((( uint32_t)0x40000000) + 0x14000);
    *p32++ = x;
    *p32++ = y;
    return p32[1];
}
# 579 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\timer.h" 1
# 175 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}
# 191 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}
# 209 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (23));
}
# 225 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (23));
}
# 241 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_StartCapture(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (3));
}
# 257 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_StopCapture(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (3));
}
# 273 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (6));
}
# 289 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (6));
}
# 305 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (7));
}
# 321 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (7));
}
# 337 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}
# 353 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}
# 369 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (5));
}
# 385 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (5));
}
# 402 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return ((timer->INTSTS & (0x1ul << (0))) ? 1 : 0);
}
# 418 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}
# 435 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}
# 451 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = (0x1ul << (0));
}
# 468 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & (0x1ul << (1)) ? 1 : 0);
}
# 484 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (1));
}
# 500 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}
# 516 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}
# 532 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_ResetCounter(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (26));
    while((timer->CTL & (0x1ul << (26))) == (0x1ul << (26)));
}


uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
int32_t TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);
void TIMER_CaptureSelect(TIMER_T *timer, uint32_t u32Src);
# 580 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\uart.h" 1
# 431 "../../Library/StdDriver/inc\\uart.h"
static __inline void UART_CLEAR_RTS(UART_T *uart);
static __inline void UART_SET_RTS(UART_T *uart);
# 444 "../../Library/StdDriver/inc\\uart.h"
static __inline void UART_CLEAR_RTS(UART_T *uart)
{
    uart->MODEM |= (0x1ul << (9));
    uart->MODEM &= ~(0x1ul << (1));
}
# 460 "../../Library/StdDriver/inc\\uart.h"
static __inline void UART_SET_RTS(UART_T *uart)
{
    uart->MODEM |= (0x1ul << (9)) | (0x1ul << (1));
}


void UART_ClearIntFlag(UART_T *uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T *uart);
void UART_DisableFlowCtrl(UART_T *uart);
void UART_DisableInt(UART_T *uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T *uart);
void UART_EnableInt(UART_T *uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T *uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T *uart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T *uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
void UART_SetTimeoutCnt(UART_T *uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T *uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T *uart, uint32_t u32Mode, uint32_t u32Addr);
uint32_t UART_Write(UART_T *uart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UART_SelectSingleWireMode(UART_T *uart);
# 581 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\usbd.h" 1
# 33 "../../Library/StdDriver/inc\\usbd.h"
typedef struct s_usbd_info
{
    uint8_t *gu8DevDesc;
    uint8_t *gu8ConfigDesc;
    uint8_t **gu8StringDesc;
    uint8_t **gu8HidReportDesc;
    uint8_t *gu8BosDesc;
    uint32_t *gu32HidReportSize;
    uint32_t *gu32ConfigHidDescIdx;

} S_USBD_INFO_T;

extern const S_USBD_INFO_T gsInfo;
# 556 "../../Library/StdDriver/inc\\usbd.h"
static __inline void USBD_MemCopy(uint8_t dest[], uint8_t src[], uint32_t size)
{
    uint32_t volatile i=0ul;

    while(size--)
    {
        dest[i] = src[i];
        i++;
    }
}
# 577 "../../Library/StdDriver/inc\\usbd.h"
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 8ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFG;
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFGP;
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg | (0x1ul << (1)));
            break;
        }
    }
}
# 608 "../../Library/StdDriver/inc\\usbd.h"
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 8ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFG;
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFGP;
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg & ~(0x1ul << (1)));
            break;
        }
    }
}
# 641 "../../Library/StdDriver/inc\\usbd.h"
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    uint32_t i;

    for(i = 0ul; i < 8ul; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFG;
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xful) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0xC0000))->EP[0].CFGP;
            break;
        }
    }

    return ((*((volatile uint32_t *)(u32CfgAddr))) & (0x1ul << (1)));
}


extern volatile uint8_t g_usbd_RemoteWakeupEn;


typedef void (*VENDOR_REQ)(void);
typedef void (*CLASS_REQ)(void);
typedef void (*SET_INTERFACE_REQ)(uint32_t u32AltInterface);
typedef void (*SET_CONFIG_CB)(void);



void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t pu8Buf[], uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);
# 582 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\usci_i2c.h" 1
# 33 "../../Library/StdDriver/inc\\usci_i2c.h"
enum UI2C_MASTER_EVENT
{
    MASTER_SEND_ADDRESS = 10u,
    MASTER_SEND_H_WR_ADDRESS,
    MASTER_SEND_H_RD_ADDRESS,
    MASTER_SEND_L_ADDRESS,
    MASTER_SEND_DATA,
    MASTER_SEND_REPEAT_START,
    MASTER_READ_DATA,
    MASTER_STOP,
    MASTER_SEND_START
};




enum UI2C_SLAVE_EVENT
{
    SLAVE_ADDRESS_ACK = 100u,
    SLAVE_H_WR_ADDRESS_ACK,
    SLAVE_L_WR_ADDRESS_ACK,
    SLAVE_GET_DATA,
    SLAVE_SEND_DATA,
    SLAVE_H_RD_ADDRESS_ACK,
    SLAVE_L_RD_ADDRESS_ACK
};
# 289 "../../Library/StdDriver/inc\\usci_i2c.h"
uint32_t UI2C_Open(UI2C_T *ui2c, uint32_t u32BusClock);
void UI2C_Close(UI2C_T *ui2c);
void UI2C_ClearTimeoutFlag(UI2C_T *ui2c);
void UI2C_Trigger(UI2C_T *ui2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Ptrg, uint8_t u8Ack);
void UI2C_DisableInt(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_EnableInt(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetBusClockFreq(UI2C_T *ui2c);
uint32_t UI2C_SetBusClockFreq(UI2C_T *ui2c, uint32_t u32BusClock);
uint32_t UI2C_GetIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_ClearIntFlag(UI2C_T *ui2c , uint32_t u32Mask);
uint32_t UI2C_GetData(UI2C_T *ui2c);
void UI2C_SetData(UI2C_T *ui2c, uint8_t u8Data);
void UI2C_SetSlaveAddr(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void UI2C_SetSlaveAddrMask(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
void UI2C_EnableTimeout(UI2C_T *ui2c, uint32_t u32TimeoutCnt);
void UI2C_DisableTimeout(UI2C_T *ui2c);
void UI2C_EnableWakeup(UI2C_T *ui2c, uint8_t u8WakeupMode);
void UI2C_DisableWakeup(UI2C_T *ui2c);
uint8_t UI2C_WriteByte(UI2C_T *ui2c, uint8_t u8SlaveAddr, const uint8_t data);
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, const uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, const uint8_t data);
uint32_t UI2C_WriteMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, const uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t data);
uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_ReadByte(UI2C_T *ui2c, uint8_t u8SlaveAddr);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t UI2C_ReadMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *rdata, uint32_t u32rLen);
# 583 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\usci_spi.h" 1
# 399 "../../Library/StdDriver/inc\\usci_spi.h"
uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void USPI_Close(USPI_T *uspi);
void USPI_ClearRxBuf(USPI_T *uspi);
void USPI_ClearTxBuf(USPI_T *uspi);
void USPI_DisableAutoSS(USPI_T *uspi);
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock);
uint32_t USPI_GetBusClock(USPI_T *uspi);
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask);
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetIntFlag(USPI_T *uspi, uint32_t u32Mask);
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetStatus(USPI_T *uspi, uint32_t u32Mask);
void USPI_EnableWakeup(USPI_T *uspi);
void USPI_DisableWakeup(USPI_T *uspi);
# 584 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\usci_uart.h" 1
# 496 "../../Library/StdDriver/inc\\usci_uart.h"
void UUART_ClearIntFlag(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_GetIntFlag(UUART_T* uuart, uint32_t u32Mask);
void UUART_Close(UUART_T* uuart);
void UUART_DisableInt(UUART_T* uuart, uint32_t u32Mask);
void UUART_EnableInt(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_Open(UUART_T* uuart, uint32_t u32baudrate);
uint32_t UUART_Read(UUART_T* uuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t UUART_SetLine_Config(UUART_T* uuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
uint32_t UUART_Write(UUART_T* uuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UUART_EnableWakeup(UUART_T* uuart, uint32_t u32WakeupMode);
void UUART_DisableWakeup(UUART_T* uuart);
void UUART_EnableFlowCtrl(UUART_T* uuart);
void UUART_DisableFlowCtrl(UUART_T* uuart);
# 585 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\wdt.h" 1
# 155 "../../Library/StdDriver/inc\\wdt.h"
static __inline void WDT_Close(void);
static __inline void WDT_EnableInt(void);
static __inline void WDT_DisableInt(void);
# 167 "../../Library/StdDriver/inc\\wdt.h"
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL = 0UL;
    while(((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL & (0x1ul << (30)));
    return;
}
# 183 "../../Library/StdDriver/inc\\wdt.h"
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL |= (0x1ul << (6));
    while(((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL & (0x1ul << (30)));
    return;
}
# 199 "../../Library/StdDriver/inc\\wdt.h"
static __inline void WDT_DisableInt(void)
{

    ((WDT_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL &= ~((0x1ul << (6)) | (0x1ul << (2)) | (0x1ul << (3)) | (0x1ul << (5)));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);
# 586 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 1 "../../Library/StdDriver/inc\\wwdt.h" 1
# 141 "../../Library/StdDriver/inc\\wwdt.h"
void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);
# 587 "../../Library/Device/Nuvoton/M031/Include\\M031Series.h" 2
# 13 "../../Library/Device/Nuvoton/M031/Include\\NuMicro.h" 2
# 5 "../Header\\MComm.h" 2
# 1 "../Header/Hardware.h" 1




# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 1 3
# 53 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    typedef unsigned int size_t;
# 68 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    typedef __builtin_va_list __va_list;
# 87 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
typedef struct __fpos_t_struct {
    unsigned long long int __pos;





    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
# 108 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
typedef struct __FILE FILE;
# 119 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
struct __FILE {
    union {
        long __FILE_alignment;



        char __FILE_size[84];

    } __FILE_opaque;
};
# 138 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;
# 224 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int remove(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int rename(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 243 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *tmpfile(void);






extern __attribute__((__nothrow__)) char *tmpnam(char * );
# 265 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fclose(FILE * ) __attribute__((__nonnull__(1)));
# 275 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fflush(FILE * );
# 285 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *fopen(const char * __restrict ,
                           const char * __restrict ) __attribute__((__nonnull__(1,2)));
# 329 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *freopen(const char * __restrict ,
                    const char * __restrict ,
                    FILE * __restrict ) __attribute__((__nonnull__(2,3)));
# 342 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void setbuf(FILE * __restrict ,
                    char * __restrict ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) int setvbuf(FILE * __restrict ,
                   char * __restrict ,
                   int , size_t ) __attribute__((__nonnull__(1)));
# 370 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int fprintf(FILE * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 393 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int _fprintf(FILE * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));





#pragma __printf_args
extern __attribute__((__nothrow__)) int printf(const char * __restrict , ...) __attribute__((__nonnull__(1)));






#pragma __printf_args
extern __attribute__((__nothrow__)) int _printf(const char * __restrict , ...) __attribute__((__nonnull__(1)));





#pragma __printf_args
extern __attribute__((__nothrow__)) int sprintf(char * __restrict , const char * __restrict , ...) __attribute__((__nonnull__(1,2)));








#pragma __printf_args
extern __attribute__((__nothrow__)) int _sprintf(char * __restrict , const char * __restrict , ...) __attribute__((__nonnull__(1,2)));






#pragma __printf_args
extern __attribute__((__nothrow__)) int __ARM_snprintf(char * __restrict , size_t ,
                     const char * __restrict , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __attribute__((__nothrow__)) int snprintf(char * __restrict , size_t ,
                     const char * __restrict , ...) __attribute__((__nonnull__(3)));
# 460 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int _snprintf(char * __restrict , size_t ,
                      const char * __restrict , ...) __attribute__((__nonnull__(3)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int fscanf(FILE * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 503 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __scanf_args
extern __attribute__((__nothrow__)) int _fscanf(FILE * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int scanf(const char * __restrict , ...) __attribute__((__nonnull__(1)));








#pragma __scanf_args
extern __attribute__((__nothrow__)) int _scanf(const char * __restrict , ...) __attribute__((__nonnull__(1)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int sscanf(const char * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 541 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __scanf_args
extern __attribute__((__nothrow__)) int _sscanf(const char * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int vfscanf(FILE * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int vscanf(const char * __restrict , __va_list) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) int vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));

extern __attribute__((__nothrow__)) int _vfscanf(FILE * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int _vscanf(const char * __restrict , __va_list) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) int _vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int __ARM_vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));

extern __attribute__((__nothrow__)) int vprintf(const char * __restrict , __va_list ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int _vprintf(const char * __restrict , __va_list ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) int vfprintf(FILE * __restrict ,
                    const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));
# 584 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int vsprintf(char * __restrict ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));
# 594 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int __ARM_vsnprintf(char * __restrict , size_t ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));

extern __attribute__((__nothrow__)) int vsnprintf(char * __restrict , size_t ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));
# 609 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int _vsprintf(char * __restrict ,
                      const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));





extern __attribute__((__nothrow__)) int _vfprintf(FILE * __restrict ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));





extern __attribute__((__nothrow__)) int _vsnprintf(char * __restrict , size_t ,
                      const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));
# 635 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int __ARM_asprintf(char ** , const char * __restrict , ...) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) int __ARM_vasprintf(char ** , const char * __restrict , __va_list ) __attribute__((__nonnull__(2)));
# 649 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fgetc(FILE * ) __attribute__((__nonnull__(1)));
# 659 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) char *fgets(char * __restrict , int ,
                    FILE * __restrict ) __attribute__((__nonnull__(1,3)));
# 673 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fputc(int , FILE * ) __attribute__((__nonnull__(2)));
# 683 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fputs(const char * __restrict , FILE * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) int getc(FILE * ) __attribute__((__nonnull__(1)));
# 704 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    extern __attribute__((__nothrow__)) int (getchar)(void);
# 713 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) char *gets(char * ) __attribute__((__nonnull__(1)));
# 725 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int putc(int , FILE * ) __attribute__((__nonnull__(2)));
# 737 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    extern __attribute__((__nothrow__)) int (putchar)(int );






extern __attribute__((__nothrow__)) int puts(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int ungetc(int , FILE * ) __attribute__((__nonnull__(2)));
# 778 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t fread(void * __restrict ,
                    size_t , size_t , FILE * __restrict ) __attribute__((__nonnull__(1,4)));
# 794 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t __fread_bytes_avail(void * __restrict ,
                    size_t , FILE * __restrict ) __attribute__((__nonnull__(1,3)));
# 810 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t fwrite(const void * __restrict ,
                    size_t , size_t , FILE * __restrict ) __attribute__((__nonnull__(1,4)));
# 822 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fgetpos(FILE * __restrict , fpos_t * __restrict ) __attribute__((__nonnull__(1,2)));
# 833 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fseek(FILE * , long int , int ) __attribute__((__nonnull__(1)));
# 850 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fsetpos(FILE * __restrict , const fpos_t * __restrict ) __attribute__((__nonnull__(1,2)));
# 863 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) long int ftell(FILE * ) __attribute__((__nonnull__(1)));
# 877 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void rewind(FILE * ) __attribute__((__nonnull__(1)));
# 886 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void clearerr(FILE * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int feof(FILE * ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) int ferror(FILE * ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) void perror(const char * );
# 917 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int _fisatty(FILE * ) __attribute__((__nonnull__(1)));



extern __attribute__((__nothrow__)) void __use_no_semihosting_swi(void);
extern __attribute__((__nothrow__)) void __use_no_semihosting(void);
# 6 "../Header/Hardware.h" 2
# 58 "../Header/Hardware.h"
typedef struct
{
 uint8_t IP1;
 uint8_t IP2;
 uint8_t IP3;
 uint8_t RL1;
 uint8_t OP1;
 uint8_t OP2;
 uint8_t OP3;
 uint8_t IsMain;
 uint8_t IsCoverOpen;
 uint8_t LED0;
 uint8_t LED1;
 double BattVolt;
 uint8_t BattPerc;
 double ADCVal1;
 double ADCVal2;
 uint32_t MeterTimeout;
}PepheralTypedef;

typedef struct
{
  uint16_t Year;
  uint8_t Month;
  uint8_t Date;
  uint8_t DaysOfWeek;
  uint8_t Hour;
  uint8_t Min;
  uint8_t Sec;
} _RTC;

typedef struct
{
    _RTC CurrentTime;
    double Lat;
    double Long;
}AstroReqtypedef;

typedef struct
{
    double Sunset;
    double Sunrise;
}AstroReplytypedef;

extern PepheralTypedef PeriPheralVal;




extern _RTC CurrentDateTime;
extern uint8_t IsTimeSet, IsRTC;
void Delay(volatile uint32_t nTime);
void ProcessRTC(void);
void SYS_Init(void);
void PeripheralInit(void);
void ProcessPeripheral(void);
# 6 "../Header\\MComm.h" 2
# 1 "../Header/EMeter.h" 1
# 19 "../Header/EMeter.h"
typedef struct
{
 uint16_t Volt;
 uint16_t Amp;
 uint16_t mPF;

 uint16_t kW;
 uint16_t kVA;
 uint16_t kVAr;
 uint16_t kVAH;
}ENERGY_PARAM;

typedef struct
{
 ENERGY_PARAM RPhase;
 ENERGY_PARAM YPhase;
 ENERGY_PARAM BPhase;
 uint32_t TotalKW;
 uint32_t TotalKVA;
 uint32_t TotalKWH;
 uint32_t TotalKVAH;
 uint16_t Freq;
 uint8_t IsMeter;
 uint8_t MeterType;
}METER_PARAM;


typedef union _INTCHAR
{
    char chVal[4];
    unsigned int uVal;
    int iVal;
}INTCHAR;

extern METER_PARAM MeterParam;
# 68 "../Header/EMeter.h"
void EMeterInit(void);
void UpdateMeterMaxwell3P(void);
void GetEMeterData(void);
void ProcessMeter(void);
# 7 "../Header\\MComm.h" 2
# 1 "../Header/DS1307.h" 1








# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 1 3
# 91 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
    typedef unsigned short wchar_t;




typedef struct div_t { int quot, rem; } div_t;

typedef struct ldiv_t { long int quot, rem; } ldiv_t;


typedef struct lldiv_t { long long quot, rem; } lldiv_t;
# 139 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int __aeabi_MB_CUR_MAX(void);
# 158 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) double atof(const char * ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) int atoi(const char * ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) long int atol(const char * ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) long long atoll(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) double strtod(const char * __restrict , char ** __restrict ) __attribute__((__nonnull__(1)));
# 206 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) float strtof(const char * __restrict , char ** __restrict ) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) long double strtold(const char * __restrict , char ** __restrict ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) long int strtol(const char * __restrict ,
                        char ** __restrict , int ) __attribute__((__nonnull__(1)));
# 243 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) unsigned long int strtoul(const char * __restrict ,
                                       char ** __restrict , int ) __attribute__((__nonnull__(1)));
# 275 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) long long strtoll(const char * __restrict ,
                                  char ** __restrict , int )
                          __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) unsigned long long strtoull(const char * __restrict ,
                                            char ** __restrict , int )
                                   __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) int rand(void);
# 303 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) void srand(unsigned int );
# 313 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
struct _rand_state { int __x[57]; };
extern __attribute__((__nothrow__)) int _rand_r(struct _rand_state *);
extern __attribute__((__nothrow__)) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __attribute__((__nothrow__)) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __attribute__((__nothrow__)) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);





extern __attribute__((__nothrow__)) void *calloc(size_t , size_t );





extern __attribute__((__nothrow__)) void free(void * );







extern __attribute__((__nothrow__)) void *malloc(size_t );





extern __attribute__((__nothrow__)) void *realloc(void * , size_t );
# 374 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
typedef int (*__heapprt)(void *, char const *, ...);
extern __attribute__((__nothrow__)) void __heapstats(int (* )(void * ,
                                           char const * , ...),
                        void * ) __attribute__((__nonnull__(1)));
# 390 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int __heapvalid(int (* )(void * ,
                                           char const * , ...),
                       void * , int ) __attribute__((__nonnull__(1)));
# 411 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__noreturn__)) void abort(void);
# 422 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int atexit(void (* )(void)) __attribute__((__nonnull__(1)));
# 444 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__noreturn__)) void exit(int );
# 460 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__noreturn__)) void _Exit(int );
# 471 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) char *getenv(const char * ) __attribute__((__nonnull__(1)));
# 484 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int system(const char * );
# 497 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern void *bsearch(const void * , const void * ,
              size_t , size_t ,
              int (* )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
# 532 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern void qsort(void * , size_t , size_t ,
           int (* )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
# 560 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) int abs(int );






extern __attribute__((__nothrow__)) __attribute__((__const__)) div_t div(int , int );
# 579 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) long int labs(long int );
# 589 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) ldiv_t ldiv(long int , long int );
# 610 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) long long llabs(long long );
# 620 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) lldiv_t lldiv(long long , long long );
# 644 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
typedef struct __sdiv32by16 { long quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned long quot, rem; } __udiv32by16;

typedef struct __sdiv64by32 { long rem, quot; } __sdiv64by32;

__attribute__((__value_in_regs__)) extern __attribute__((__nothrow__)) __attribute__((__const__)) __sdiv32by16 __rt_sdiv32by16(
     int ,
     short int );



__attribute__((__value_in_regs__)) extern __attribute__((__nothrow__)) __attribute__((__const__)) __udiv32by16 __rt_udiv32by16(
     unsigned int ,
     unsigned short );



__attribute__((__value_in_regs__)) extern __attribute__((__nothrow__)) __attribute__((__const__)) __sdiv64by32 __rt_sdiv64by32(
     int , unsigned int ,
     int );







extern __attribute__((__nothrow__)) unsigned int __fp_status(unsigned int , unsigned int );
# 705 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int mblen(const char * , size_t );
# 720 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int mbtowc(wchar_t * __restrict ,
                   const char * __restrict , size_t );
# 739 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int wctomb(char * , wchar_t );
# 761 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) size_t mbstowcs(wchar_t * __restrict ,
                      const char * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 779 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) size_t wcstombs(char * __restrict ,
                      const wchar_t * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 798 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) void __use_realtime_heap(void);
extern __attribute__((__nothrow__)) void __use_realtime_division(void);
extern __attribute__((__nothrow__)) void __use_two_region_memory(void);
extern __attribute__((__nothrow__)) void __use_no_heap(void);
extern __attribute__((__nothrow__)) void __use_no_heap_region(void);

extern __attribute__((__nothrow__)) char const *__C_library_version_string(void);
extern __attribute__((__nothrow__)) int __C_library_version_number(void);
# 10 "../Header/DS1307.h" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdbool.h" 1 3
# 11 "../Header/DS1307.h" 2
# 28 "../Header/DS1307.h"
typedef enum
{
  SUNDAY = 1,
  MONDAY,
  TUESDAY,
  WEDNESDAY,
  THURSDAY,
  FRIDAY,
  SATURDAY
} DaysOfWeek;
# 50 "../Header/DS1307.h"
extern _RTC RTCDateTime;

void DS1307_Init(void);
uint8_t CheckRTC(void);
_Bool DS1307_GetTime(_RTC *rtc);
_Bool DS1307_SetTime(_RTC *rtc);
_Bool DS1307_ReadTemperature(float *temp);



_Bool ReadRegister(uint8_t regAddr, uint8_t *value);
_Bool WriteRegister(uint8_t regAddr, uint8_t value);
# 8 "../Header\\MComm.h" 2
# 1 "../Header/SunPos.h" 1







# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 1 3
# 157 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double , double );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float , float );







extern __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float );
extern __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double );



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);
}





static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
# 266 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
  typedef float float_t;
  typedef double double_t;
# 282 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern const int math_errhandling;







extern __attribute__((__nothrow__)) double acos(double );



extern __attribute__((__nothrow__)) double asin(double );





extern __attribute__((__nothrow__)) __attribute__((__const__)) double atan(double );



extern __attribute__((__nothrow__)) double atan2(double , double );





extern __attribute__((__nothrow__)) double cos(double );




extern __attribute__((__nothrow__)) double sin(double );





extern void __use_accurate_range_reduction(void);



extern __attribute__((__nothrow__)) double tan(double );





extern __attribute__((__nothrow__)) double cosh(double );




extern __attribute__((__nothrow__)) double sinh(double );






extern __attribute__((__nothrow__)) __attribute__((__const__)) double tanh(double );



extern __attribute__((__nothrow__)) double exp(double );






extern __attribute__((__nothrow__)) double frexp(double , int * ) __attribute__((__nonnull__(2)));







extern __attribute__((__nothrow__)) double ldexp(double , int );




extern __attribute__((__nothrow__)) double log(double );





extern __attribute__((__nothrow__)) double log10(double );



extern __attribute__((__nothrow__)) double modf(double , double * ) __attribute__((__nonnull__(2)));





extern __attribute__((__nothrow__)) double pow(double , double );






extern __attribute__((__nothrow__)) double sqrt(double );
# 410 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
    static __inline double _sqrt(double __x) { return sqrt(__x); }
# 427 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
    static __inline float _sqrtf(float __x) { return (float)sqrt(__x); }







extern __attribute__((__nothrow__)) __attribute__((__const__)) double ceil(double );


extern __attribute__((__nothrow__)) __attribute__((__const__)) double fabs(double );



extern __attribute__((__nothrow__)) __attribute__((__const__)) double floor(double );



extern __attribute__((__nothrow__)) double fmod(double , double );
# 467 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double acosh(double );



extern __attribute__((__nothrow__)) double asinh(double );



extern __attribute__((__nothrow__)) double atanh(double );



extern __attribute__((__nothrow__)) double cbrt(double );



static __inline __attribute__((__nothrow__)) __attribute__((__const__)) double copysign(double __x, double __y)



{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
static __inline __attribute__((__nothrow__)) __attribute__((__const__)) float copysignf(float __x, float __y)



{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __attribute__((__nothrow__)) double erf(double );



extern __attribute__((__nothrow__)) double erfc(double );



extern __attribute__((__nothrow__)) double expm1(double );
# 533 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double hypot(double , double );






extern __attribute__((__nothrow__)) int ilogb(double );



extern __attribute__((__nothrow__)) int ilogbf(float );



extern __attribute__((__nothrow__)) int ilogbl(long double );
# 646 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double lgamma (double );




extern __attribute__((__nothrow__)) double log1p(double );



extern __attribute__((__nothrow__)) double logb(double );



extern __attribute__((__nothrow__)) float logbf(float );



extern __attribute__((__nothrow__)) long double logbl(long double );



extern __attribute__((__nothrow__)) double nextafter(double , double );




extern __attribute__((__nothrow__)) float nextafterf(float , float );




extern __attribute__((__nothrow__)) long double nextafterl(long double , long double );




extern __attribute__((__nothrow__)) double nexttoward(double , long double );




extern __attribute__((__nothrow__)) float nexttowardf(float , long double );




extern __attribute__((__nothrow__)) long double nexttowardl(long double , long double );




extern __attribute__((__nothrow__)) double remainder(double , double );



extern __attribute__((__nothrow__)) __attribute__((__const__)) double rint(double );



extern __attribute__((__nothrow__)) double scalbln(double , long int );



extern __attribute__((__nothrow__)) float scalblnf(float , long int );



extern __attribute__((__nothrow__)) long double scalblnl(long double , long int );



extern __attribute__((__nothrow__)) double scalbn(double , int );



extern __attribute__((__nothrow__)) float scalbnf(float , int );



extern __attribute__((__nothrow__)) long double scalbnl(long double , int );
# 740 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) float fabsf(float);
static __inline __attribute__((__nothrow__)) __attribute__((__const__)) float _fabsf(float __f) { return fabsf(__f); }
extern __attribute__((__nothrow__)) float sinf(float );
extern __attribute__((__nothrow__)) float cosf(float );
extern __attribute__((__nothrow__)) float tanf(float );
extern __attribute__((__nothrow__)) float acosf(float );
extern __attribute__((__nothrow__)) float asinf(float );
extern __attribute__((__nothrow__)) float atanf(float );
extern __attribute__((__nothrow__)) float atan2f(float , float );
extern __attribute__((__nothrow__)) float sinhf(float );
extern __attribute__((__nothrow__)) float coshf(float );
extern __attribute__((__nothrow__)) float tanhf(float );
extern __attribute__((__nothrow__)) float expf(float );
extern __attribute__((__nothrow__)) float logf(float );
extern __attribute__((__nothrow__)) float log10f(float );
extern __attribute__((__nothrow__)) float powf(float , float );
extern __attribute__((__nothrow__)) float sqrtf(float );
extern __attribute__((__nothrow__)) float ldexpf(float , int );
extern __attribute__((__nothrow__)) float frexpf(float , int * ) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) __attribute__((__const__)) float ceilf(float );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float floorf(float );
extern __attribute__((__nothrow__)) float fmodf(float , float );
extern __attribute__((__nothrow__)) float modff(float , float * ) __attribute__((__nonnull__(2)));
# 780 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
__attribute__((__nothrow__)) long double acosl(long double );
__attribute__((__nothrow__)) long double asinl(long double );
__attribute__((__nothrow__)) long double atanl(long double );
__attribute__((__nothrow__)) long double atan2l(long double , long double );
__attribute__((__nothrow__)) long double ceill(long double );
__attribute__((__nothrow__)) long double cosl(long double );
__attribute__((__nothrow__)) long double coshl(long double );
__attribute__((__nothrow__)) long double expl(long double );
__attribute__((__nothrow__)) long double fabsl(long double );
__attribute__((__nothrow__)) long double floorl(long double );
__attribute__((__nothrow__)) long double fmodl(long double , long double );
__attribute__((__nothrow__)) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__attribute__((__nothrow__)) long double ldexpl(long double , int );
__attribute__((__nothrow__)) long double logl(long double );
__attribute__((__nothrow__)) long double log10l(long double );
__attribute__((__nothrow__)) long double modfl(long double , long double * ) __attribute__((__nonnull__(2)));
__attribute__((__nothrow__)) long double powl(long double , long double );
__attribute__((__nothrow__)) long double sinl(long double );
__attribute__((__nothrow__)) long double sinhl(long double );
__attribute__((__nothrow__)) long double sqrtl(long double );
__attribute__((__nothrow__)) long double tanl(long double );
__attribute__((__nothrow__)) long double tanhl(long double );






extern __attribute__((__nothrow__)) float acoshf(float );
__attribute__((__nothrow__)) long double acoshl(long double );
extern __attribute__((__nothrow__)) float asinhf(float );
__attribute__((__nothrow__)) long double asinhl(long double );
extern __attribute__((__nothrow__)) float atanhf(float );
__attribute__((__nothrow__)) long double atanhl(long double );
__attribute__((__nothrow__)) long double copysignl(long double , long double );
extern __attribute__((__nothrow__)) float cbrtf(float );
__attribute__((__nothrow__)) long double cbrtl(long double );
extern __attribute__((__nothrow__)) float erff(float );
__attribute__((__nothrow__)) long double erfl(long double );
extern __attribute__((__nothrow__)) float erfcf(float );
__attribute__((__nothrow__)) long double erfcl(long double );
extern __attribute__((__nothrow__)) float expm1f(float );
__attribute__((__nothrow__)) long double expm1l(long double );
extern __attribute__((__nothrow__)) float log1pf(float );
__attribute__((__nothrow__)) long double log1pl(long double );
extern __attribute__((__nothrow__)) float hypotf(float , float );
__attribute__((__nothrow__)) long double hypotl(long double , long double );
extern __attribute__((__nothrow__)) float lgammaf(float );
__attribute__((__nothrow__)) long double lgammal(long double );
extern __attribute__((__nothrow__)) float remainderf(float , float );
__attribute__((__nothrow__)) long double remainderl(long double , long double );
extern __attribute__((__nothrow__)) float rintf(float );
__attribute__((__nothrow__)) long double rintl(long double );







extern __attribute__((__nothrow__)) double exp2(double );
extern __attribute__((__nothrow__)) float exp2f(float );
__attribute__((__nothrow__)) long double exp2l(long double );
extern __attribute__((__nothrow__)) double fdim(double , double );
extern __attribute__((__nothrow__)) float fdimf(float , float );
__attribute__((__nothrow__)) long double fdiml(long double , long double );
# 855 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double fma(double , double , double );
extern __attribute__((__nothrow__)) float fmaf(float , float , float );

static __inline __attribute__((__nothrow__)) long double fmal(long double __x, long double __y, long double __z) { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __attribute__((__nothrow__)) __attribute__((__const__)) double fmax(double , double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float fmaxf(float , float );
__attribute__((__nothrow__)) long double fmaxl(long double , long double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) double fmin(double , double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float fminf(float , float );
__attribute__((__nothrow__)) long double fminl(long double , long double );
extern __attribute__((__nothrow__)) double log2(double );
extern __attribute__((__nothrow__)) float log2f(float );
__attribute__((__nothrow__)) long double log2l(long double );
extern __attribute__((__nothrow__)) long lrint(double );
extern __attribute__((__nothrow__)) long lrintf(float );

static __inline __attribute__((__nothrow__)) long lrintl(long double __x) { return lrint((double)__x); }


extern __attribute__((__nothrow__)) long long llrint(double );
extern __attribute__((__nothrow__)) long long llrintf(float );

static __inline __attribute__((__nothrow__)) long long llrintl(long double __x) { return llrint((double)__x); }


extern __attribute__((__nothrow__)) long lround(double );
extern __attribute__((__nothrow__)) long lroundf(float );

static __inline __attribute__((__nothrow__)) long lroundl(long double __x) { return lround((double)__x); }


extern __attribute__((__nothrow__)) long long llround(double );
extern __attribute__((__nothrow__)) long long llroundf(float );

static __inline __attribute__((__nothrow__)) long long llroundl(long double __x) { return llround((double)__x); }


extern __attribute__((__nothrow__)) __attribute__((__const__)) double nan(const char * );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float nanf(const char * );

static __inline __attribute__((__nothrow__)) __attribute__((__const__)) long double nanl(const char *__t) { return (long double)nan(__t); }
# 908 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) double nearbyint(double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float nearbyintf(float );
__attribute__((__nothrow__)) long double nearbyintl(long double );
extern __attribute__((__nothrow__)) double remquo(double , double , int * );
extern __attribute__((__nothrow__)) float remquof(float , float , int * );

static __inline long double remquol(long double __x, long double __y, int *__q) { return (long double)remquo((double)__x, (double)__y, __q); }


extern __attribute__((__nothrow__)) __attribute__((__const__)) double round(double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float roundf(float );
__attribute__((__nothrow__)) long double roundl(long double );
extern __attribute__((__nothrow__)) double tgamma(double );
extern __attribute__((__nothrow__)) float tgammaf(float );
__attribute__((__nothrow__)) long double tgammal(long double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) double trunc(double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float truncf(float );
__attribute__((__nothrow__)) long double truncl(long double );
# 9 "../Header/SunPos.h" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 1 3
# 82 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 3
typedef unsigned int clock_t;
typedef unsigned int time_t;






struct tm {
    int tm_sec;

    int tm_min;
    int tm_hour;
    int tm_mday;
    int tm_mon;
    int tm_year;
    int tm_wday;
    int tm_yday;
    int tm_isdst;
    union {
        struct {
            int __extra_1, __extra_2;
        };
        struct {
            long __extra_1_long, __extra_2_long;
        };
        struct {
            char *__extra_1_cptr, *__extra_2_cptr;
        };
        struct {
            void *__extra_1_vptr, *__extra_2_vptr;
        };
    };
};
# 127 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 3
extern __attribute__((__nothrow__)) clock_t clock(void);







extern __attribute__((__nothrow__)) double difftime(time_t , time_t );




extern __attribute__((__nothrow__)) time_t mktime(struct tm * ) __attribute__((__nonnull__(1)));
# 156 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 3
extern __attribute__((__nothrow__)) time_t time(time_t * );
# 166 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 3
extern __attribute__((__nothrow__)) char *asctime(const struct tm * ) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) char *_asctime_r(const struct tm * ,
                                char * __restrict ) __attribute__((__nonnull__(1,2)));
# 178 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 3
extern __attribute__((__nothrow__)) char *ctime(const time_t * ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) struct tm *gmtime(const time_t * ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) struct tm *localtime(const time_t * ) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) struct tm *_localtime_r(const time_t * __restrict ,
                                       struct tm * __restrict ) __attribute__((__nonnull__(1,2)));
# 203 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\time.h" 3
extern __attribute__((__nothrow__)) size_t strftime(char * __restrict , size_t ,
                       const char * __restrict ,
                       const struct tm * __restrict ) __attribute__((__nonnull__(1,3,4)));
# 10 "../Header/SunPos.h" 2






void SetPosition(double lat, double lon, double tz);
double setCurrentDate(int y, int m, int d);
double calcSunrise(void);
double calcSunset(void);
double calcJD(int year,int month,int day);
double calcSunriseUTC(double JD, double latitude, double longitude);
double calcSunsetUTC(double JD, double latitude, double longitude);
# 9 "../Header\\MComm.h" 2

# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 1 3
# 58 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memcpy(void * __restrict ,
                    const void * __restrict , size_t ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) void *memmove(void * ,
                    const void * , size_t ) __attribute__((__nonnull__(1,2)));
# 77 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strcpy(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) char *strncpy(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 93 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strcat(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) char *strncat(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 117 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int memcmp(const void * , const void * , size_t ) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int strcmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) int strncmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 141 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int strcasecmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int strncasecmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 158 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int strcoll(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 169 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strxfrm(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 193 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memchr(const void * , int , size_t ) __attribute__((__nonnull__(1)));
# 209 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strchr(const char * , int ) __attribute__((__nonnull__(1)));
# 218 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strcspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 232 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strpbrk(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 247 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strrchr(const char * , int ) __attribute__((__nonnull__(1)));
# 257 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 270 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strstr(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 280 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strtok(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) char *_strtok_r(char * , const char * , char ** ) __attribute__((__nonnull__(2,3)));
# 321 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memset(void * , int , size_t ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) char *strerror(int );







extern __attribute__((__nothrow__)) size_t strlen(const char * ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) size_t strlcpy(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 362 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strlcat(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 388 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void _membitcpybl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpybb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpyhl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpyhb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpywl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpywb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovebl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovebb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovehl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovehb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovewl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovewb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
# 11 "../Header\\MComm.h" 2
# 23 "../Header\\MComm.h"
extern uint8_t MCOMMTxBuff[];
extern uint8_t MCOMMRxBuff[];
extern uint8_t MCOMMRxFlags[];

static inline void ClearMCOMMRXBuffer(void)
{
    memset(MCOMMRxBuff,0x00,300);
}

static inline void ClearMCOMMTXBuffer(void)
{
    memset(MCOMMTxBuff,0x00,300);
}

extern int MCOMMUartHandle;
extern uint8_t IsMCOMM;
# 47 "../Header\\MComm.h"
typedef enum {MCOMM_COM_FUNCTION_ERROR=0,MCOMM_COM_FUNCTION_SUCCESS,MCOMM_COM_FUNCTION_GETPH,MCOMM_COM_FUNCTION_SETPH,MCOMM_COM_FUNCTION_METER,
                MCOMM_COM_FUNCTION_URT1,MCOMM_COM_FUNCTION_URT2,MCOMM_COM_FUNCTION_GETRTC,MCOMM_COM_FUNCTION_SETRTC,MCOMM_COM_FUNCTION_ASTREQ,MCOMM_COM_FUNCTION_ASTREP}MCOMMComFunctionTypedef;



typedef enum {SRCH_FOR_ST=0,SRCH_FOR_FUNC,SRCH_FOR_ED,WAIT_FOR_PARS}RXSTATETypedef;



typedef struct
{
    uint8_t datalen;
    uint8_t data[300];
}UartExchangetypedef;
# 80 "../Header\\MComm.h"
static const uint16_t LENTABLE[11] =
{
 3,
 3,
 3,
 3 +sizeof(PepheralTypedef),
 3,
 3 +sizeof(UartExchangetypedef),
 3 +sizeof(UartExchangetypedef),
 3,
 3 +sizeof(_RTC),
 3 +sizeof(AstroReqtypedef),
 3 +sizeof(AstroReplytypedef)
};

void InitModemComm(void);
void ProcessModemComm(void);
# 2 "../Source/MComm.c" 2

uint8_t MCOMMTxBuff[300];
uint8_t MCOMMRxBuff[300];

AstroReqtypedef AstroReq;
AstroReplytypedef AstroRep;
PepheralTypedef RcvPhData;
volatile uint16_t RxCount;
uint16_t ExpectedLEN;
RXSTATETypedef RxState;
extern uint8_t IsRTCSet;

void InitModemComm(void)
{
 SYS_ResetModule(((0x4<<24)|(18)));
 UART_Open(((UART_T *) ((( uint32_t)0x40000000) + 0x72000)), 230400);
 NVIC_EnableIRQ(UART02_IRQn);
  UART_EnableInt(((UART_T *) ((( uint32_t)0x40000000) + 0x72000)), ((0x1ul << (0)) ));
}

void SendDatatoModem(uint8_t *src, size_t len)
{
 while(len)
 {
  while((((((UART_T *) ((( uint32_t)0x40000000) + 0x72000)))->FIFOSTS & (0x1ul << (23)))>>(23)));
  ((((UART_T *) ((( uint32_t)0x40000000) + 0x72000)))->DAT = (*src));
  src++;
  len--;
 }
}

void UART02_IRQHandler(void)
{
 uint8_t tmp = 0xFF;
 if ((((((UART_T *) ((( uint32_t)0x40000000) + 0x72000)))->INTSTS & ((0x1ul << (8))))?1:0))
 {
  while((((((UART_T *) ((( uint32_t)0x40000000) + 0x72000)))->INTSTS & (0x1ul << (0)))>>(0)))
    {
   tmp = ((((UART_T *) ((( uint32_t)0x40000000) + 0x72000)))->DAT);
   switch(RxState)
   {
    case WAIT_FOR_PARS:
      return;
    case SRCH_FOR_ST:
     if(tmp!=0x26)
      return;
     MCOMMRxBuff[0]=tmp;
     RxCount=1;
     RxState=SRCH_FOR_FUNC;
     return;
    case SRCH_FOR_FUNC:
     if(tmp>=11)
     {
      RxState=SRCH_FOR_ST;
      return;
     }
     ExpectedLEN=LENTABLE[tmp];
     MCOMMRxBuff[RxCount++] = tmp;
     RxState=SRCH_FOR_ED;
     return;
    case SRCH_FOR_ED:
     MCOMMRxBuff[RxCount++] = tmp;
     if(RxCount==ExpectedLEN)
     {
      if(tmp!=0x7E)
      {
       RxState=SRCH_FOR_ST;
       return;
      }
      RxState=WAIT_FOR_PARS;
      return;
     }
   }

  }
 }
}

void MCOMM_SendSuccess(void)
{
 ClearMCOMMTXBuffer();
 MCOMMTxBuff[0] = 0x26;
 MCOMMTxBuff[1] = MCOMM_COM_FUNCTION_SUCCESS;
 MCOMMTxBuff[3 -1]=0x7E;
 SendDatatoModem(MCOMMTxBuff,3);
}

void MCOMM_SendError(void)
{
 ClearMCOMMTXBuffer();
 MCOMMTxBuff[0] = 0x26;
 MCOMMTxBuff[1] = MCOMM_COM_FUNCTION_ERROR;
 MCOMMTxBuff[3 -1]=0x7E;
 SendDatatoModem(MCOMMTxBuff,3);
}

void MCOMM_SendPhpr(void)
{
 ClearMCOMMTXBuffer();
 MCOMMTxBuff[0] = 0x26;
 MCOMMTxBuff[1] = MCOMM_COM_FUNCTION_GETPH;
 memcpy((void*)&MCOMMTxBuff[2],&PeriPheralVal,sizeof(PepheralTypedef));
 MCOMMTxBuff[3 +sizeof(PepheralTypedef)-1]=0x7E;
 SendDatatoModem(MCOMMTxBuff,3 +sizeof(PepheralTypedef));
}

void MCOMM_SendMeter(void)
{
 ClearMCOMMTXBuffer();
 MCOMMTxBuff[0] = 0x26;
 MCOMMTxBuff[1] = MCOMM_COM_FUNCTION_METER;
 memcpy((void*)&MCOMMTxBuff[2],&MeterParam,sizeof(METER_PARAM));
 MCOMMTxBuff[3 +sizeof(METER_PARAM)-1]=0x7E;
 SendDatatoModem(MCOMMTxBuff,3 +sizeof(METER_PARAM));
}

void MCOMM_SendRTC(void)
{
 uint8_t y;
 ClearMCOMMTXBuffer();
 MCOMMTxBuff[0] = 0x26;
 MCOMMTxBuff[1] = MCOMM_COM_FUNCTION_GETRTC;
 y=RTCDateTime.Year;
 if(!IsRTC)
  RTCDateTime.Year=0xFE;
 memcpy((void*)&MCOMMTxBuff[2],&RTCDateTime,sizeof(_RTC));
 RTCDateTime.Year=y;
 MCOMMTxBuff[3 +sizeof(_RTC)-1]=0x7E;
 SendDatatoModem(MCOMMTxBuff,3 +sizeof(_RTC));
}

void MCOMM_SendAstroData(void)
{
 double JD;
 JD = calcJD(AstroReq.CurrentTime.Year,AstroReq.CurrentTime.Month,AstroReq.CurrentTime.Date);
 AstroRep.Sunrise= calcSunriseUTC(JD,AstroReq.Lat,AstroReq.Long);
 AstroRep.Sunset = calcSunsetUTC(JD,AstroReq.Lat,AstroReq.Long);
 ClearMCOMMTXBuffer();
 MCOMMTxBuff[0] = 0x26;
 MCOMMTxBuff[1] = MCOMM_COM_FUNCTION_ASTREP;
 memcpy((void*)&MCOMMTxBuff[2],&AstroRep,sizeof(AstroReplytypedef));
 MCOMMTxBuff[3 +sizeof(AstroReplytypedef)-1]=0x7E;
 SendDatatoModem(MCOMMTxBuff,3 +sizeof(AstroReplytypedef));
}



void ParseModemData(void)
{
 if(MCOMMRxBuff[1]== MCOMM_COM_FUNCTION_GETPH)
 {
  MCOMM_SendPhpr();
  return;
 }
 if(MCOMMRxBuff[1]== MCOMM_COM_FUNCTION_SETPH)
 {
  memcpy(&RcvPhData,(void*)&MCOMMRxBuff[2],sizeof(PepheralTypedef));
  PeriPheralVal.OP1= RcvPhData.OP1;
  PeriPheralVal.OP2= RcvPhData.OP2;
  PeriPheralVal.OP3= RcvPhData.OP3;
  PeriPheralVal.RL1= RcvPhData.RL1;
  PeriPheralVal.LED0= RcvPhData.LED0;
  PeriPheralVal.LED1= RcvPhData.LED1;
  PeriPheralVal.MeterTimeout = RcvPhData.MeterTimeout;
  MCOMM_SendSuccess();
  return;
 }
 if(MCOMMRxBuff[1]== MCOMM_COM_FUNCTION_METER)
 {
  MCOMM_SendMeter();
  return;
 }
 if(MCOMMRxBuff[1]== MCOMM_COM_FUNCTION_GETRTC)
 {
  MCOMM_SendRTC();
  return;
 }
 if(MCOMMRxBuff[1]== MCOMM_COM_FUNCTION_SETRTC)
 {
  IsRTCSet=1;
  memcpy(&RTCDateTime,(void*)&MCOMMRxBuff[2],sizeof(_RTC));
  MCOMM_SendSuccess();
  return;
 }
 if(MCOMMRxBuff[1]== MCOMM_COM_FUNCTION_ASTREQ)
 {
  memcpy(&AstroReq,(void*)&MCOMMRxBuff[2],sizeof(AstroReqtypedef));
  MCOMM_SendAstroData();
  return;
 }
}

void ProcessModemComm(void)
{
 if(RxState!=WAIT_FOR_PARS)
  return;

 ParseModemData();
 ClearMCOMMRXBuffer();
 RxState=SRCH_FOR_ST;
}
