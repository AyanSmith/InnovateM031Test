#ifndef MCOMM_H_
#define MCOMM_H_

#include "NuMicro.h"
#include "Hardware.h"
#include <stdio.h>
#include <string.h>

#define MCOMM_VERION					10

#define MCOMM_UART_SYS_RST		UART0_RST
#define MCOMM_UART						UART0
#define MCOMM_UART_BAUD				115200
#define MCOMM_UART_IRQ				UART02_IRQn

#define MCOMM_FET_PORT				PC
#define MCOMM_FET_PIN					BIT4
#define MCOMM_PWK_PORT				PC
#define MCOMM_PWK_PIN





#define MCOMM_RX_BUFF_SIZE 300
#define MCOMM_TX_BUFF_SIZE 300


extern uint8_t MCOMMTxBuff[];
extern uint8_t MCOMMRxBuff[];
extern uint8_t MCOMMRxFlags[];

static inline void ClearMCOMMRXBuffer(void)
{
    memset(MCOMMRxBuff,0x00,MCOMM_RX_BUFF_SIZE);
}

static inline void ClearMCOMMTXBuffer(void)
{
    memset(MCOMMTxBuff,0x00,MCOMM_TX_BUFF_SIZE);
}

extern int MCOMMUartHandle;
extern uint8_t IsMCOMM;

#define MCOMM_COM_HEADER  0x26
#define MCOMM_COM_FOOTER  0x7E

#define MCOMM_COM_HEADER_INDEX    0
#define MCOMM_COM_FUNCTION_INDEX  1
#define MCOMM_COM_DATA_INDEX      2

typedef enum {MCOMM_COM_FUNCTION_ERROR=0,MCOMM_COM_FUNCTION_SUCCESS,MCOMM_COM_FUNCTION_GETPH,MCOMM_COM_FUNCTION_SETPH,
                MCOMM_COM_FUNCTION_485TX,MCOMM_COM_FUNCTION_485RX,MCOMM_COM_FUNCTION_232TX,MCOMM_COM_FUNCTION_232RX}MCOMMComFunctionTypedef;

#define MCOMM_RX_FLAG_COUNT  8
								
typedef enum {SRCH_FOR_ST=0,SRCH_FOR_FUNC,SRCH_FOR_ED,WAIT_FOR_PARS}RXSTATETypedef;


#define MCOMM_COM_URT_EXG_BUFF_SIZE   250
typedef struct 
{
    uint8_t datalen;
    uint8_t data[MCOMM_COM_URT_EXG_BUFF_SIZE];
}UartExchangetypedef;

#define MCOMM_COM_LEN_EXTRAS 3
// FUNTION TOTAL LENGTHS

#define MCOMM_COM_LEN_ERROR_MCU         MCOMM_COM_LEN_EXTRAS
#define MCOMM_COM_LEN_SUCCESS_MCU       MCOMM_COM_LEN_EXTRAS
#define MCOMM_COM_LEN_GETPH_MDM         MCOMM_COM_LEN_EXTRAS
#define MCOMM_COM_LEN_GETPH_MCU         MCOMM_COM_LEN_EXTRAS+sizeof(PepheralTypedef)
#define MCOMM_COM_LEN_SETPH_MDM         MCOMM_COM_LEN_EXTRAS+sizeof(PepheralTypedef)
#define MCOMM_COM_LEN_485_MDM           MCOMM_COM_LEN_EXTRAS+sizeof(UartExchangetypedef)
#define MCOMM_COM_LEN_485_MCU           MCOMM_COM_LEN_EXTRAS+sizeof(UartExchangetypedef)
#define MCOMM_COM_LEN_232_MDM           MCOMM_COM_LEN_EXTRAS+sizeof(UartExchangetypedef)
#define MCOMM_COM_LEN_232_MCU           MCOMM_COM_LEN_EXTRAS+sizeof(UartExchangetypedef)


static const uint16_t LENTABLE[MCOMM_RX_FLAG_COUNT] = 
{
	MCOMM_COM_LEN_ERROR_MCU,
	MCOMM_COM_LEN_SUCCESS_MCU,
	MCOMM_COM_LEN_GETPH_MDM,
	MCOMM_COM_LEN_SETPH_MDM,
	MCOMM_COM_LEN_485_MDM,
	MCOMM_COM_LEN_485_MCU,
	MCOMM_COM_LEN_232_MDM,
	MCOMM_COM_LEN_232_MCU
};

void InitModemComm(void);
void ProcessModemComm(void);


#endif