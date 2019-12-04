#ifndef __NTC_H_
#define __NTC_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "cmsis_os.h"

int NTC_B57500M_GetTemperatureC(uint32_t Urt_mV, float *ret_temp);	
	
	
#ifdef __cplusplus
}
#endif

#endif
