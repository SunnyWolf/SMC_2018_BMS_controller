#ifndef __BUTTONS_H__
#define __BUTTONS_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
	 
void buttons_task(void* params);

#ifdef __cplusplus
}
#endif

#endif // __BUTTONS_H__
