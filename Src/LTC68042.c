#include "LTC68042.h"
#include "cmsis_os.h"
#include "pec15.h"
#include "CMSIS_OS.h"
#include "math.h"
#include "usart.h"

#define HAL_SPI_SS_LOW() HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET)
#define HAL_SPI_SS_HIGH() HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET)
#define RELAY_CHARGER_ON() HAL_GPIO_WritePin(RELAY_CH_GPIO_Port, RELAY_CH_Pin, GPIO_PIN_SET)
#define RELAY_CHARGER_OFF() HAL_GPIO_WritePin(RELAY_CH_GPIO_Port, RELAY_CH_Pin, GPIO_PIN_RESET)
#define RELAY_CONTROLLERS_ON() HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET)
#define RELAY_CONTROLLERS_OFF() HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_RESET)
#define RELAY_DCDC_ON() HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET)
#define RELAY_DCDC_OFF() HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET)

LTC_DriverStruct ltc;

void LTC_SendBroadcast(uint16_t cmd);
void LTC_WakeUp(void);
void LTC_DischargeCells(uint8_t boardid, uint16_t DischargeCells, uint8_t DischargeTime);
void LTC_BalanceCells(void);
uint8_t LTC_CheckPEC(LTC_RegisterPacketStruct *packet);
void LTC_DetectCellsToDischarge(void);
void LTC_CheckCharger(void);
uint16_t LTC_GetADCMode(void);
void LTC_BoardsSelfTest(void);
void LTC_ReadTemperature(void);
void LTC_CleanRegister(LTC_RegisterPacketStruct* reg);
void LTC_FailHandler(void);
void LTC_ClearErrorFlags(void);
uint8_t LTC_CheckConnectionErrorFlag(void);
uint8_t LTC_CheckSelftestErrorFlag(void);
uint8_t LTC_CheckTemperatureIsNotValid(void);
void LTC_StateMachine(void);
void LTC_StateMachineWork(void);

void LTC_Init(SPI_HandleTypeDef* spi){
	ltc.Spi = spi;
	ltc.cfg_CellVoltageMax = LTC_BATTERY_VOLTAGE_MAX;
	ltc.cfg_CellVoltageMin = LTC_BATTERY_VOLTAGE_MIN;
	
	ltc.cfg_ADCMode = LTC_ADC_MODE_14KHZ;
	ltc.cfg_DCP = 1;
	ltc.cfg_GPIO = 0x18;
	
	ltc.state = LTC_STATE_STANDBY;
	
	ltc.ErrorCounter_CON = 0;
	ltc.ErrorCounter_ST = 0;
	
	PEC15_Init();
}

void LTC_Task(void* params){
	LTC_WriteBasicConfig();
	LTC_BoardsSelfTest();
	
	volatile int b = 0;
	volatile int p = 0;
	volatile uint16_t id = 0;
	
	for(;;){
		if (ltc.ErrorCounter_CON > 10){
			LTC_FailHandler();
			ltc.ErrorCounter_CON = 0;
		}

		if (LTC_CheckConnectionErrorFlag()) {
			LTC_WriteBasicConfig();
			ltc.ErrorCounter_ST++;
			LTC_ClearErrorFlags();
		}
		/*
		if (LTC_CheckSelftestErrorFlag()){
			LTC_BoardsSelfTest();
		}

		if (LTC_CheckSelftestErrorFlag()){
			ltc.ErrorCounter_ST++;
			continue;
		}
		*/
		LTC_ReadTemperature();
		if (LTC_CheckConnectionErrorFlag()) continue;
		LTC_StartADCConverting();
		LTC_ReadVoltages();
		if (LTC_CheckConnectionErrorFlag()) continue;
		LTC_ReadVoltageTotal();
		if (LTC_CheckConnectionErrorFlag()) continue;
		LTC_CheckCharger();
		
		LTC_StateMachine();
		LTC_StateMachineWork();
		
		vTaskDelay(200);
	}
}

void LTC_FailHandler(){
	RELAY_CHARGER_OFF();
	RELAY_CONTROLLERS_OFF();
	RELAY_DCDC_OFF();
	//ltc.state = LTC_STATE_ERROR;
}

void LTC_StateMachine(){
	if (ltc.state == LTC_STATE_STANDBY){
		if (ltc.CellVoltageMin - ltc.cfg_CellVoltageMin < LTC_ACCURACY_DISCHARGE){
			ltc.state = LTC_STATE_TURNOFF;
		} else if (ltc.flag_ChargerOn){
			ltc.state = LTC_STATE_CHARGING;
		} else if (ltc.flag_PowerOn){
			ltc.state = LTC_STATE_POWERON;
		}
		return;
	}
	
	if (ltc.state == LTC_STATE_POWERON){
		if (ltc.flag_ChargerOn){
			ltc.state = LTC_STATE_CHARGING;
		} else if (ltc.CellVoltageMin <= ltc.cfg_CellVoltageMin) {
			ltc.state = LTC_STATE_TURNOFF;
		} else if (!ltc.flag_PowerOn){
			ltc.state = LTC_STATE_STANDBY;
		}
		return;
	}
	
	if (ltc.state == LTC_STATE_TURNOFF){
		if (ltc.flag_ChargerOn)
			ltc.state = LTC_STATE_CHARGING;
		return;
	}
	
	if (ltc.state == LTC_STATE_CHARGING){
		if (!ltc.flag_ChargerOn)
			ltc.state = LTC_STATE_STANDBY;
		else if (ltc.cfg_CellVoltageMax - ltc.CellVoltageMax < LTC_ACCURACY_CHARGING)
			ltc.state = LTC_STATE_BALANSING;
		return;
	}
	
	if (ltc.state == LTC_STATE_BALANSING){
		if (!ltc.flag_ChargerOn)
			ltc.state = LTC_STATE_STANDBY;
		else if (ltc.CellVoltageMax - ltc.CellVoltageMin < LTC_ACCURACY_BALANSING){
			if (ltc.cfg_CellVoltageMax - ltc.CellVoltageMax < LTC_ACCURACY_CHARGING)
				ltc.state = LTC_STATE_CHARGED;
			else
				ltc.state = LTC_STATE_CHARGING;
		}
		return;
	}
	
	if (ltc.state == LTC_STATE_CHARGED)
		if (!ltc.flag_ChargerOn)
			ltc.state = LTC_STATE_STANDBY;
		return;
}

void LTC_StateMachineWork(){
	if (ltc.state == LTC_STATE_STANDBY){
		RELAY_DCDC_ON();
		RELAY_CONTROLLERS_OFF();
		RELAY_CHARGER_OFF();
		LTC_WriteBasicConfig();
		return;
	}
	
	if (ltc.state == LTC_STATE_POWERON){
		RELAY_DCDC_ON();
		RELAY_CONTROLLERS_ON();
		RELAY_CHARGER_OFF();
		LTC_WriteBasicConfig();
		return;
	}
	if (ltc.state == LTC_STATE_CHARGING){
		RELAY_CONTROLLERS_OFF();
		RELAY_DCDC_OFF();
		RELAY_CHARGER_ON();
		ltc.flag_PowerOn = 0;
		LTC_BalanceCells();
		return;
	}
	if (ltc.state == LTC_STATE_BALANSING){
		RELAY_CHARGER_OFF();
		LTC_BalanceCells();
		return;
	}
	if (ltc.state == LTC_STATE_CHARGED){
		RELAY_CHARGER_OFF();
		LTC_WriteBasicConfig();
		return;
	}
	if (ltc.state == LTC_STATE_TURNOFF){
		RELAY_DCDC_OFF();
		RELAY_CONTROLLERS_OFF();
		RELAY_CHARGER_OFF();
		LTC_WriteBasicConfig();
	}
}

void LTC_ClearErrorFlags(){
	for (int i=0; i<LTC_BOARDS; i++){
		ltc.Boards[i].flag_Error_CON = 0;
		ltc.Boards[i].flag_Error_ST1 = 0;
		ltc.Boards[i].flag_Error_ST2 = 0;
	}
}

uint8_t LTC_CheckConnectionErrorFlag(){
	for (int i=0; i<LTC_BOARDS; i++){
		if (ltc.Boards[i].flag_Error_CON) return 1;
	}
	return 0;
}

uint8_t LTC_CheckSelftestErrorFlag(){
	for (int i=0; i<LTC_BOARDS; i++){
		if (ltc.Boards[i].flag_Error_ST1) return 1;
		if (ltc.Boards[i].flag_Error_ST2) return 1;
	}
	return 0;
}

uint8_t LTC_CheckTemperatureIsNotValid(){
	for (int i=0; i<LTC_BOARDS; i++){
		if (ltc.Boards[i].TemperatureChip > LTC_CRIT_TEMPERATURE) return 1;
		if (ltc.Boards[i].TemperatureSensors[0] > LTC_CRIT_TEMPERATURE) return 1;
		if (ltc.Boards[i].TemperatureSensors[1] > LTC_CRIT_TEMPERATURE) return 1;
	}
	
	return 0;
}

void LTC_CheckCharger(){
	GPIO_PinState charger = HAL_GPIO_ReadPin(CH_DETECT_GPIO_Port, CH_DETECT_Pin);
	if (charger == GPIO_PIN_RESET){
		ltc.flag_ChargerOn = 1;
	} else {
		ltc.flag_ChargerOn = 0;
	}
}

void LTC_ReadTemperature(){
	LTC_RegisterPacketStruct reg;
	volatile float temp;
	volatile float R_t;
	
	LTC_SendBroadcast(LTC_CMD_ADSTAT | LTC_GetADCMode() | LTC_CHST_ITMP);
	vTaskDelay(3);
	for (int i=0; i<LTC_BOARDS; i++){
		LTC_ReadRegister(i, LTC_CMD_RDSTATA, &reg);
		if(!LTC_CheckPEC(&reg)){
			ltc.Boards[i].flag_Error_CON = 1;
			ltc.Boards[i].TemperatureChip = 0.0f;
			continue;
		}
		temp = (((uint16_t)reg.REG3 << 8) | (uint16_t)(reg.REG2));
		ltc.Boards[i].TemperatureChip = temp / 10.0f / 7.5f - 273.0f;	
	}
	
	LTC_CleanRegister(&reg);
	LTC_SendBroadcast(LTC_CMD_ADAX | LTC_GetADCMode() | LTC_CHG_GPIO4);
	LTC_SendBroadcast(LTC_CMD_ADAX | LTC_GetADCMode() | LTC_CHG_GPIO5);
	for (int i=0; i<LTC_BOARDS; i++){
		LTC_ReadRegister(i, LTC_CMD_RDAUXB, &reg);
		if(!LTC_CheckPEC(&reg)) {
			ltc.Boards[i].flag_Error_CON = 1;
			ltc.Boards[i].TemperatureSensors[0] = 0.0f;
			ltc.Boards[i].TemperatureSensors[1] = 0.0f;
		} else {		
			temp = (uint16_t)((reg.REG1 << 8) | reg.REG0);
			R_t = 10000.0f * (30000.0f / temp - 1);
			temp = log(R_t);
			ltc.Boards[i].TemperatureSensors[0] = 1 / (LTC_NTC_A0 + LTC_NTC_A1 * temp + LTC_NTC_A2 * pow(temp, 3));
			temp = (uint16_t)((reg.REG3 << 8) | reg.REG2);
			R_t = 10000.0f * (30000.0f / temp - 1);
			temp = log(R_t);
			ltc.Boards[i].TemperatureSensors[1] = 1 / (LTC_NTC_A0 + LTC_NTC_A1 * temp + LTC_NTC_A2 * pow(temp, 3));
		}
	}
}

void LTC_BoardsSelfTest(){
	volatile uint16_t value;
	LTC_RegisterPacketStruct reg;
	
	// self-test 1
	LTC_SendBroadcast(LTC_CMD_CVST | LTC_GetADCMode() | LTC_ST1);
	vTaskDelay(5);
	for (int i=0; i<LTC_BOARDS; i++){
		LTC_ReadRegister(i, LTC_CMD_RDCV, &reg);
		if (!LTC_CheckPEC(&reg)){
			ltc.Boards[i].flag_Error_CON = 1;
			continue;
		}
		value = (reg.REG1 << 8) | reg.REG0;
		switch (ltc.cfg_ADCMode){
			case LTC_ADC_MODE_27KHZ:
				ltc.Boards[i].flag_Error_ST1 = value == LTC_ST1_RESULT_27 ? 0 : 1;
				break;
			case LTC_ADC_MODE_14KHZ:
				ltc.Boards[i].flag_Error_ST1 = value == LTC_ST1_RESULT_14 ? 0 : 1;
				break;
			default:
				ltc.Boards[i].flag_Error_ST1 = value == LTC_ST1_RESULT_7_3_2_26 ? 0 : 1;
				break;
		}
	}
	// self-test 2
	LTC_SendBroadcast(LTC_CMD_CVST | LTC_GetADCMode() | LTC_ST2);
	vTaskDelay(5);
	for (int i=0; i<LTC_BOARDS; i++){
		LTC_ReadRegister(i, LTC_CMD_RDCV, &reg);
		if (!LTC_CheckPEC(&reg)){
			ltc.Boards[i].flag_Error_CON = 1;
			continue;
		}
		value = (reg.REG1 << 8) | reg.REG0;
		switch (ltc.cfg_ADCMode){
			case LTC_ADC_MODE_27KHZ:
				ltc.Boards[i].flag_Error_ST2 = value == LTC_ST2_RESULT_27 ? 0 : 1;
				break;
			case LTC_ADC_MODE_14KHZ:
				ltc.Boards[i].flag_Error_ST2 = value == LTC_ST2_RESULT_14 ? 0 : 1;
				break;
			default:
				ltc.Boards[i].flag_Error_ST2 = value == LTC_ST2_RESULT_7_3_2_26 ? 0 : 1;
				break;
		}
	}
}

uint16_t LTC_GetADCMode(){
	return (((uint16_t)ltc.cfg_ADCMode & 0x06) << 6);
}


void LTC_SendBroadcast(uint16_t cmd){
	uint16_t pec;
	LTC_CommandPacketStruct packet;
	
	packet.CMD1 = cmd >> 8;
	packet.CMD2 = cmd;
	pec = PEC15((uint8_t*)&packet, 2);
	packet.PEC1 = pec >> 8;
	packet.PEC2 = pec;
	
	LTC_WakeUp();
	
	HAL_SPI_SS_LOW();
	HAL_SPI_Transmit(ltc.Spi, (uint8_t*)&packet, 4, 100);
	HAL_SPI_SS_HIGH();
}

void LTC_ReadRegister(uint8_t boardid, uint16_t reg, LTC_RegisterPacketStruct* data){
	uint16_t temp;
	LTC_CommandPacketStruct packet;
	
	temp = LTC_BASE_ADDR | (boardid << 11) | reg;
	
	packet.CMD1 = temp >> 8;
	packet.CMD2 = temp;
	temp = PEC15((uint8_t*)&packet, 2);
	packet.PEC1 = temp >> 8;
	packet.PEC2 = temp;
	
	
	LTC_WakeUp();
	HAL_SPI_SS_LOW();
	HAL_SPI_Transmit(ltc.Spi, (uint8_t*)&packet, 4, 100);
	HAL_SPI_Receive(ltc.Spi, (uint8_t*)data, 8, 100);
	HAL_SPI_SS_HIGH();
}

void LTC_WriteRegister(uint16_t cmd, LTC_RegisterPacketStruct* data){
	uint16_t pec;
	uint8_t packet[12];
	
	packet[0] = cmd >> 8;
	packet[1] = cmd;
	pec = PEC15(packet, 2);
	packet[2] = pec >> 8;
	packet[3] = pec;
	
	pec = PEC15((uint8_t*)data, 6);
	data->PEC0 = pec >> 8;
	data->PEC1 = pec;
	
	for (int i=4; i<12; i++){
		packet[i] = ((uint8_t*)data)[i-4];
	}
	
	LTC_WakeUp();
	HAL_SPI_SS_LOW();
	volatile HAL_StatusTypeDef stat = HAL_SPI_Transmit(ltc.Spi, packet, 12, 100);
	HAL_SPI_SS_HIGH();
}

void LTC_ReadVoltages(){
	volatile uint16_t pec, pec_calc;
	LTC_RegisterPacketStruct packet;
	
	for(int i=0; i<LTC_BOARDS; i++){	
		for(int j=0; j<4; j++){
			LTC_ReadRegister(i, LTC_CMD_RDCV + j*2, &packet);
			if (!LTC_CheckPEC(&packet)){
				ltc.Boards[i].flag_Error_CON = 1;
				
				ltc.Boards[i].CellVoltage[3*j + 0] = 0.0f;
				ltc.Boards[i].CellVoltage[3*j + 1] = 0.0f;
				ltc.Boards[i].CellVoltage[3*j + 2] = 0.0f;
			} else {
				ltc.Boards[i].CellVoltage[3*j + 0] = ((float)((packet.REG1 << 8) | packet.REG0))/10000;
				ltc.Boards[i].CellVoltage[3*j + 1] = ((float)((packet.REG3 << 8) | packet.REG2))/10000;
				ltc.Boards[i].CellVoltage[3*j + 2] = ((float)((packet.REG5 << 8) | packet.REG4))/10000;
				if(j==3) 
					ltc.Boards[i].CellVoltage[3*j + 2] += 0.065f;
			}
		}
	}
	ltc.CellVoltageMin = ltc.Boards[0].CellVoltage[0];
	ltc.CellVoltageMax = ltc.Boards[0].CellVoltage[0];
	for(int i=0; i<LTC_BOARDS; i++){
		for(int j=0; j<LTC_CELLS; j++){
			if (ltc.Boards[i].CellVoltage[j] < ltc.CellVoltageMin)
				ltc.CellVoltageMin = ltc.Boards[i].CellVoltage[j];
			if (ltc.Boards[i].CellVoltage[j] > ltc.CellVoltageMax)
				ltc.CellVoltageMax = ltc.Boards[i].CellVoltage[j];
		}
	}
}

void LTC_StartADCConverting(){
	uint16_t cmd = LTC_CMD_ADCV | LTC_GetADCMode() | (ltc.cfg_DCP << 4);
	LTC_SendBroadcast(cmd);
}

void LTC_WriteBasicConfig(){
	LTC_RegisterPacketStruct packet;
	uint16_t temp;
	
	packet.REG0 = (ltc.cfg_ADCMode & 0x01) | (ltc.cfg_REFON << 2) | (ltc.cfg_GPIO << 3);

	temp = ((ltc.cfg_CellVoltageMin * 10000) / 16) - 1;
	packet.REG1 = temp;
	packet.REG2 = 0x0F & (temp >> 8);
	temp = (ltc.cfg_CellVoltageMax * 10000) / 16;
	packet.REG2 |= temp << 4;  
	packet.REG3 = temp >> 4;
	packet.REG4 = 0;
	packet.REG5 = 0;	
	
	LTC_WriteRegister(LTC_CMD_WRCFG, &packet);
}

void LTC_DischargeCells(uint8_t boardid, uint16_t DischargeCells, uint8_t DischargeTime){
	LTC_RegisterPacketStruct packet;
	volatile uint16_t temp;
	
	packet.REG0 = (ltc.cfg_ADCMode & 0x01) | (ltc.cfg_REFON << 2) | (ltc.cfg_GPIO << 3);

	temp = (((uint16_t)(ltc.cfg_CellVoltageMin * 10000)) >> 4) - 1;
	packet.REG1 = temp;
	packet.REG2 = 0x0F & (temp >> 8);
	temp = (((uint16_t)(ltc.cfg_CellVoltageMax * 10000)) >> 4) - 1;
	packet.REG2 |= temp << 4;  
	packet.REG3 = temp >> 4;
	packet.REG4 = DischargeCells;
	packet.REG5 = ((DischargeCells >> 8) & 0x0F) | DischargeTime << 4;

	LTC_WriteRegister(LTC_BASE_ADDR | (boardid << 11) | LTC_CMD_WRCFG, &packet);
}

void LTC_BalanceCells(){
	//uint16_t cells_to_balance;
	
	LTC_DetectCellsToDischarge();
	for (int b=0; b<LTC_BOARDS; b++){
		/*
		cells_to_balance = 0;
		for(int c=0; c<LTC_CELLS; c++){
			if (ltc.Boards[b].CellVoltage[c] > ltc.CellVoltageMin + LTC_ACCURACY_BALANSING){
				cells_to_balance |= (1 << c);
			}
			LTC_DischargeCells(b, cells_to_balance, 0);
		}
		*/
		LTC_DischargeCells(b, ltc.Boards[b].CellsToDischarge, 0);
	}
}

void LTC_ReadVoltageTotal(){
	LTC_RegisterPacketStruct packet;
	float v_total;
	uint16_t cmd;

	ltc.VoltageTotal = 0.0f;
	
	cmd = LTC_CMD_ADSTAT | (((uint16_t)ltc.cfg_ADCMode & 0x0006) << 6) | LTC_CHST_SOC;
	LTC_SendBroadcast(cmd);
	for (int i=0; i < LTC_BOARDS; i++){
		LTC_ReadRegister(i, LTC_CMD_RDSTATA, &packet);
		if (!LTC_CheckPEC(&packet)){
			ltc.Boards[i].flag_Error_CON = 1;
			continue;
		}
		v_total = ((uint16_t)(packet.REG1 << 8) | packet.REG0) * 0.0001f * 20.0f;
		ltc.VoltageTotal += v_total;
	}
}

void LTC_WakeUp(){
	HAL_SPI_SS_LOW();
	HAL_Delay(1);
	HAL_SPI_SS_HIGH();
	HAL_Delay(1);
}

uint8_t LTC_CheckPEC(LTC_RegisterPacketStruct* packet){
	volatile uint16_t pec, pec_calc;
	pec = (packet->PEC0 << 8) | packet->PEC1;
	pec_calc = PEC15((uint8_t*)packet, 6);
	
	if (pec == pec_calc){
		return 1;
	} else {
		return 0;
	}
}

void LTC_DetectCellsToDischarge(){
	for(int i=0; i < LTC_BOARDS; i++){
		ltc.Boards[i].CellsToDischarge = 0;
		for (int j=0; j < LTC_CELLS; j++){
			if (ltc.Boards[i].CellVoltage[j] > ltc.CellVoltageMin + LTC_ACCURACY_BALANSING){
				ltc.Boards[i].CellsToDischarge |= (1 << j);
			}
		}
	}
}

void LTC_CleanRegister(LTC_RegisterPacketStruct* reg){
	for (int i=0; i<8; i++) ((uint8_t*)reg)[i] = 0;
}
