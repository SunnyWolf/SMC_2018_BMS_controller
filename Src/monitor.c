#include "monitor.h"
#include "usart.h"
#include "LTC68042.h"
#include "adc.h"

extern LTC_DriverStruct ltc;

void monitor_task(void * params){
	for(;;){
		vTaskDelay(200);
		UART_SendMessageUU(UART_MSG_BAT_STATUS, ltc.state, 0); 
		UART_SendMessageFF(UART_MSG_BAT_MINMAX, ltc.CellVoltageMax, ltc.CellVoltageMin);
		UART_SendMessageFF(UART_MSG_BAT_PERCENT, ADC_GetPercent(), ltc.VoltageTotal); 
		UART_SendMessageFF(UART_MSG_BCB_TEMP, ltc.Boards[0].TemperatureChip, ltc.Boards[1].TemperatureChip);
		UART_SendMessageFF(UART_MSG_BAT_CURRENT, 0, ADC_GetVal());
		for (int b=0; b<LTC_BOARDS; b++){
			for (int c=0; c<LTC_CELLS/2; c++){
				UART_SendMessageFF(UART_MSG_BAT_CELL + b*6 + c, ltc.Boards[b].CellVoltage[2*c], ltc.Boards[b].CellVoltage[2*c+1]);
			}				
		}
	}
}
