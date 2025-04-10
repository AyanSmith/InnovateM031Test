#include "Boot.h"
#include "FMCUser.h"

BootConfigTypedef BootConfig;
uint8_t 	FDataBuff[FMC_FLASH_PAGE_SIZE], RemainingPackets, tries;
uint16_t  NoOfPackets, CurrentPacket, Appsize;
uint32_t 	ADDR,  word, Rword, Cword;
uint16_t 	WordIndex;

void FlashNewApp(void);

void LoadBootConfig(void)
{		
	uint16_t Fsize;
	Fsize = sizeof(BootConfig);
	ReadFMCDataBuffer((uint8_t*)&BootConfig, FMC_LDROM_BASE+FLASH_BOOT_CONFIG_ADDR, Fsize);
		
	if(BootConfig.DefValue != FLASH_DEFAULT_VALUE)
	{
		BootConfig.DefValue = FLASH_DEFAULT_VALUE;
		goto savereturn;
	}
	
	BootConfig.BootCode = BCODE_NORMAL_OPERATION;
	
	if(BootConfig.UpdateREQCode == FLASH_UPDATE_REQ_CODE)
	{
		FlashNewApp();
	}
	
	
	
	savereturn:
	Fsize = sizeof(BootConfig);
	FMC_ENABLE_LD_UPDATE();
	EraseFMCData(FMC_LDROM_BASE+FLASH_BOOT_CONFIG_ADDR,Fsize);
	WriteFMCData((uint8_t*)&BootConfig, FMC_LDROM_BASE+FLASH_BOOT_CONFIG_ADDR, Fsize );
	FMC_DISABLE_LD_UPDATE();
	return;
}


void FlashNewApp(void)
{
	//FLASHING New

	Appsize = BootConfig.NewAppSize;
	if(Appsize == 0 || Appsize > 60000)
		return;
	
	NoOfPackets = Appsize / 512;
	RemainingPackets = Appsize % 512;
	

	
	FMC_ENABLE_AP_UPDATE();
	//FMC_Open(); // Open FMC for Erase/Write

	for(CurrentPacket=0;CurrentPacket < Appsize; CurrentPacket+= FMC_FLASH_PAGE_SIZE)
	{
		ADDR = FLASH_NEWAPP_ADDR;
		ADDR += (CurrentPacket);
		
		ReadFMCDataBuffer(FDataBuff,ADDR,512);
		
		ADDR = ROM_APP_BASE_ADDR;
		ADDR += (CurrentPacket);
		FMC_Erase(ADDR);
		for(WordIndex = 0; WordIndex < 512; WordIndex += 4)
		{
			word = (FDataBuff[WordIndex+3] << 24) | (FDataBuff[WordIndex+2] << 16) | (FDataBuff[WordIndex+1] << 8) | FDataBuff[WordIndex];
			tries=0;
			FMC_Write(ADDR,word);
			ADDR+=4;
		}
	}
	//Verify
	for(CurrentPacket=0;CurrentPacket < Appsize; CurrentPacket+= FMC_FLASH_PAGE_SIZE)
	{
		ADDR = FLASH_NEWAPP_ADDR;
		ADDR += (CurrentPacket);
		
		ReadFMCDataBuffer(FDataBuff,ADDR,512);
		
		ADDR = ROM_APP_BASE_ADDR;
		ADDR += (CurrentPacket);
		for(WordIndex = 0; WordIndex < 512; WordIndex += 4)
		{
			word = (FDataBuff[WordIndex+3] << 24) | (FDataBuff[WordIndex+2] << 16) | (FDataBuff[WordIndex+1] << 8) | FDataBuff[WordIndex];
			tries=0;
			Rword = FMC_Read(ADDR);
			if(Rword != word)
				DelayMS(1);	
			ADDR+=4;
		}
	}
	

	FMC_DISABLE_AP_UPDATE();
	BootConfig.UpdateREQCode =0;
	BootConfig.CurrentAppSize = BootConfig.NewAppSize;
	BootConfig.BootCode = BCODE_FLASHED_NEW;
}