#include "MComm.h"

uint8_t MCOMMTxBuff[MCOMM_TX_BUFF_SIZE];
uint8_t MCOMMRxBuff[MCOMM_RX_BUFF_SIZE];


PepheralTypedef RcvPhData;
__IO uint16_t RxCount;
uint16_t ExpectedLEN;
RXSTATETypedef RxState;
extern uint8_t IsRTCSet;

void InitModemComm(void)
{
	SYS_ResetModule(MCOMM_UART_SYS_RST);
	UART_Open(MCOMM_UART, MCOMM_UART_BAUD);
	NVIC_EnableIRQ(MCOMM_UART_IRQ);
  UART_EnableInt(MCOMM_UART, (UART_INTEN_RDAIEN_Msk ));
}

void SendDatatoModem(uint8_t *src, size_t len)
{
	while(len)
	{
		while(UART_IS_TX_FULL(MCOMM_UART));
		UART_WRITE(MCOMM_UART,*src);
		src++;
		len--;
	}
}

void UART02_IRQHandler(void)
{
	uint8_t tmp = 0xFF;
	if (UART_GET_INT_FLAG(MCOMM_UART,UART_INTSTS_RDAINT_Msk))
	{
		while(UART_IS_RX_READY(MCOMM_UART))
    {
			tmp = UART_READ(MCOMM_UART);
			switch(RxState)
			{
				case WAIT_FOR_PARS:
						return;
				case SRCH_FOR_ST: 
					if(tmp!=MCOMM_COM_HEADER)
						return;
					MCOMMRxBuff[0]=tmp;
					RxCount=1;
					RxState=SRCH_FOR_FUNC;
					return;
				case SRCH_FOR_FUNC:
					if(tmp>=MCOMM_RX_FLAG_COUNT)
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
						if(tmp!=MCOMM_COM_FOOTER)
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
	MCOMMTxBuff[MCOMM_COM_HEADER_INDEX] = MCOMM_COM_HEADER;
	MCOMMTxBuff[MCOMM_COM_FUNCTION_INDEX] = MCOMM_COM_FUNCTION_SUCCESS;
	MCOMMTxBuff[MCOMM_COM_LEN_SUCCESS_MCU-1]=MCOMM_COM_FOOTER;
	SendDatatoModem(MCOMMTxBuff,MCOMM_COM_LEN_SUCCESS_MCU);
}

void MCOMM_SendError(void)
{
	ClearMCOMMTXBuffer();
	MCOMMTxBuff[MCOMM_COM_HEADER_INDEX] = MCOMM_COM_HEADER;
	MCOMMTxBuff[MCOMM_COM_FUNCTION_INDEX] = MCOMM_COM_FUNCTION_ERROR;
	MCOMMTxBuff[MCOMM_COM_LEN_ERROR_MCU-1]=MCOMM_COM_FOOTER;
	SendDatatoModem(MCOMMTxBuff,MCOMM_COM_LEN_ERROR_MCU);
}

void MCOMM_SendPhpr(void)
{
	ClearMCOMMTXBuffer();
	MCOMMTxBuff[MCOMM_COM_HEADER_INDEX] = MCOMM_COM_HEADER;
	MCOMMTxBuff[MCOMM_COM_FUNCTION_INDEX] = MCOMM_COM_FUNCTION_GETPH;
	memcpy((void*)&MCOMMTxBuff[MCOMM_COM_DATA_INDEX],&PeriPheralVal,sizeof(PepheralTypedef));
	MCOMMTxBuff[MCOMM_COM_LEN_GETPH_MCU-1]=MCOMM_COM_FOOTER;
	SendDatatoModem(MCOMMTxBuff,MCOMM_COM_LEN_GETPH_MCU);
}



void ParseModemData(void)
{
	if(MCOMMRxBuff[MCOMM_COM_FUNCTION_INDEX]== MCOMM_COM_FUNCTION_GETPH)
	{
		MCOMM_SendPhpr();
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