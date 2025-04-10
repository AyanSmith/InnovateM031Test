
#ifndef					_HARDWARE_H
#define					_HARDWARE_H

#include <stdio.h>
#include "NuMicro.h"




#define							IP1_PIN						BIT13
#define							IP1_PORT					PA
#define							IP1_VAL						PA13

#define							IP2_PIN						BIT14
#define							IP2_PORT					PA
#define							IP2_VAL						PA14

#define 						ADC1_PORT					PB
#define							ADC1_PIN					BIT1

#define 						ADC2_PORT					PB
#define							ADC2_PIN					BIT0

#define 						VMAIN_PORT				PB
#define							VMAIN_PIN					BIT13

#define 						VSEN_PORT					PB
#define							VSEN_PIN					BIT14




typedef struct
{
	uint8_t IP1;
	uint8_t IP2;
	double MainVolt;
	double BattVolt;
	double ADCVal1;
	double ADCVal2;
}PepheralTypedef;





extern PepheralTypedef PeriPheralVal;

//  ---------------------------------------------------------
// Prototype Declaration
//

void Delay(__IO uint32_t nTime);
void SYS_Init(void);
void PeripheralInit(void);
void ProcessPeripheral(void);

#endif

