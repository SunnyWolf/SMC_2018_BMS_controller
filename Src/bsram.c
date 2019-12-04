#include "bsram.h"

void BSRAM_Init(void){
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_BKP_CLK_ENABLE();
	
	HAL_PWR_EnableBkUpAccess();
}

uint32_t GetAddressFromBlockNum(int block){
	uint32_t addr;
	if (block < 0 || block > 42)
		return 0;
	if (block <= 10){
		addr = BKP_BASE + 0x04 + block * 0x04;
	}
	if (block > 10){
		addr = BKP_BASE + 0x40 + (block - 11) * 0x04;
	}
	return addr;
}

uint16_t BSRAM_Read(int block){
	uint32_t addr = GetAddressFromBlockNum(block);
	uint16_t value;
	if (addr){
		value = *(__IO uint32_t *) (addr);
	}
	return value;
}

void BSRAM_Write(int block, void * p){
	uint32_t addr = GetAddressFromBlockNum(block);
	if (addr){
		*(__IO uint32_t *) (addr) = *(uint16_t *)p;
	}
}
