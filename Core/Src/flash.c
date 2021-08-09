
#include "defines.h"
#include "global.h"

#define FLASH_KEY1				0x45670123U
#define FLASH_KEY2				0xCDEF89ABU
#define FLASH_TYPEPROGRAM_BYTE	0x00000000U	// 1byteずつの書き込み

void Flash_Lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

void Flash_Unlock(void)
{
	FLASH->KEYR =  FLASH_KEY1;
	FLASH->KEYR =  FLASH_KEY2;
}

void Flash_WaitBusy(void)
{
	while(((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) == 1);
}

void Flash_EraseSector(uint32_t sector)
{
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= (sector << 3) & FLASH_CR_SNB_Msk;
	FLASH->CR |= FLASH_CR_STRT;
	Flash_WaitBusy();
}

void Flash_WriteByte(uint32_t address, uint8_t data)
{
	FLASH->CR &= ~(FLASH_CR_PSIZE);
	FLASH->CR |= FLASH_TYPEPROGRAM_BYTE;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO uint8_t*)address = data;

	Flash_WaitBusy();

	FLASH->CR &= ~(FLASH_CR_PG);
}

void Flash_WriteData(uint32_t address, uint8_t* data, uint32_t size)
{
	do {
		Flash_WriteByte(address, *data);
	} while(++address, ++data, --size);
}

void Flash_ReadData(uint32_t address, uint8_t* data, uint32_t size)
{
	memcpy(data, (uint8_t*)address, size);
}
