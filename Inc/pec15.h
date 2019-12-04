#ifndef _PEC15_H_
#define _PEC15_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
	 
#define PEC15_POLY 0x4599
	 	 
	 void PEC15_Init(void);
	 uint16_t PEC15(uint8_t* data, int len);

#ifdef __cplusplus
}
#endif
#endif

