#include "buttons.h"
#include "LTC68042.h"

extern LTC_DriverStruct ltc;

void buttons_task(void* params){
	GPIO_PinState state = GPIO_PIN_SET;
	TickType_t t_start, t_cur;
	uint8_t st = 0;
	
	t_start = xTaskGetTickCount();
	
	for (;;){
		state = HAL_GPIO_ReadPin(KEY_DETECT_GPIO_Port, KEY_DETECT_Pin);
		
		
		t_cur = xTaskGetTickCount();
		
		if (state == GPIO_PIN_RESET){
			if (t_cur - t_start > 200 && !st){
				if (ltc.state == LTC_STATE_STANDBY ||
						ltc.state == LTC_STATE_POWERON){
					ltc.flag_PowerOn = ltc.flag_PowerOn > 0 ? 0 : 1;
				}
				st = 1;
			}
		} else {
			t_start = t_cur;
			st = 0;
		}
		
		vTaskDelay(100);
	}
}

