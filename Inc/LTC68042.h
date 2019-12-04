#ifndef __LTC6804_H__
#define __LTC6804_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
	 
#define LTC_BASE_ADDR						0x8000

#define LTC_ADC_MODE_27KHZ	2
#define LTC_ADC_MODE_7KHZ		4
#define LTC_ADC_MODE_26KHZ	6
#define LTC_ADC_MODE_14KHZ	3
#define LTC_ADC_MODE_3KHZ		5
#define LTC_ADC_MODE_2KHZ 	7
	 
#define LTC_DISCHARGE_PERMITED		1
#define LTC_DISCHARGE_NOTPERMITED	0
	 
#define LTC_BATTERY_VOLTAGE_MAX 3.65f
#define LTC_BATTERY_VOLTAGE_MIN 2.0f
	 
#define LTC_ACCURACY_DISCHARGE 0.003f	
#define LTC_ACCURACY_CHARGING 0.003f
#define LTC_ACCURACY_BALANSING 0.003f

#define LTC_CRIT_TEMPERATURE 80.0f
	 
#define LTC_BOARDS 2
#define LTC_CELLS 12

#define LTC_BASE_ADDR 0x8000

#define LTC_CMD_WRCFG		0x0001 // Write Configuration Registers
#define LTC_CMD_RDCFG		0x0002 // Read Configuration
#define LTC_CMD_RDCV		0x0004 // Read Cell Voltage Register
#define LTC_CMD_RDAUXB	0x000E // Read Auxiliary Register Group B
#define LTC_CMD_RDSTATA	0x0010 // Read Status Register A
#define LTC_CMD_RDSTATB	0x0012 // Read Status Register A
#define LTC_CMD_ADCV    0x0260 // Start Cell Voltage ADC Conversion and Poll Status
#define LTC_CMD_ADAX		0x0460 // Start GPIOs ADC Conversion and Poll Status
#define LTC_CMD_ADSTAT	0x0468 // Start Status group ADC Conversion and Poll Status
#define LTC_CMD_ADCVAX	0x046f // Start Combined Cell Voltage and GPIO1/2 conversion and poll status
#define LTC_CMD_CLRSTAT 0x0713 // Clear Status Register Group

#define LTC_CMD_CVST	 0x0207 // Start ADC Self-test
#define LTC_ST1 (1 << 5)
#define LTC_ST2 (2 << 5)

#define LTC_ST1_RESULT_27 0x9565
#define LTC_ST2_RESULT_27 0x6a9a
#define LTC_ST1_RESULT_14 0x9553
#define LTC_ST2_RESULT_14 0x6aac
#define LTC_ST1_RESULT_7_3_2_26 0x9555
#define LTC_ST2_RESULT_7_3_2_26 0x6aaa

#define LTC_NTC_A0 1.0f
#define LTC_NTC_A1 0.0f
#define LTC_NTC_A2 0.0f


enum {
	LTC_DISCHARGE_DISABLED = 0,
	LTC_DISCHARGE_30S,
	LTC_DISCHARGE_1M,
	LTC_DISCHARGE_2M,
	LTC_DISCHARGE_3M,
	LTC_DISCHARGE_4M,
	LTC_DISCHARGE_5M,
	LTC_DISCHARGE_10M,
	LTC_DISCHARGE_15M,
	LTC_DISCHARGE_30M,
	LTC_DISCHARGE_40M,
	LTC_DISCHARGE_60M,
	LTC_DISCHARGE_75M,
	LTC_DISCHARGE_90M,
	LTC_DISCHARGE_120M
};

enum {
	LTC_CHST_ALL = 0,
	LTC_CHST_SOC,
	LTC_CHST_ITMP,
	LTC_CHST_VA,
	LTC_CHST_VD
};

enum {
	LTC_CHG_ALL = 0,
	LTC_CHG_GPIO1,
	LTC_CHG_GPIO2,
	LTC_CHG_GPIO3,
	LTC_CHG_GPIO4,
	LTC_CHG_GPIO5,
	LTC_CHG_2ND_REF
};

typedef enum {
	LTC_STATE_STANDBY,
	LTC_STATE_POWERON,
	LTC_STATE_CHARGING,
	LTC_STATE_CHARGED,
	LTC_STATE_BALANSING,
	LTC_STATE_TURNOFF,
	LTC_STATE_ERROR
} LTC_StateEnum;

#pragma pack(1)
typedef struct {
	uint8_t REG0;
	uint8_t REG1;	
	uint8_t REG2;
	uint8_t REG3;
	uint8_t REG4;
	uint8_t REG5;
	uint8_t	PEC0;
	uint8_t PEC1;
}LTC_RegisterPacketStruct;

typedef struct {
	uint8_t CMD1;
	uint8_t CMD2;
	uint8_t PEC1;
	uint8_t PEC2;
}LTC_CommandPacketStruct;

#pragma pack(4)

typedef struct {
	uint8_t flag_Error_ST1 : 1;
	uint8_t flag_Error_ST2 : 1;
	uint8_t flag_Error_CON : 1;
	
	uint16_t CellsToDischarge;
	float CellVoltage[LTC_CELLS];
	float TemperatureChip;
	float TemperatureSensors[2];
} LTC_BoardStruct;

typedef struct {
	uint8_t flag_ChargerOn : 1;
	uint8_t flag_PowerOn : 1;
	uint8_t cfg_REFON : 1;
	uint8_t cfg_DCP : 1;
	uint8_t cfg_GPIO;
	uint8_t cfg_ADCMode;
	uint8_t ErrorCounter_CON;
	uint8_t ErrorCounter_ST;
	LTC_StateEnum state;
	float cfg_CellVoltageMin;
	float cfg_CellVoltageMax;
	float VoltageTotal;
	float CellVoltageMin;
	float CellVoltageMax;
	SPI_HandleTypeDef* Spi;
	LTC_BoardStruct Boards[LTC_BOARDS];
}LTC_DriverStruct;



void LTC_Task(void* params);

void LTC_Init(SPI_HandleTypeDef* spi);
void LTC_SetADCMode(uint8_t mode);

void LTC_ReadVoltages(void);
void LTC_StartADCConverting(void);
void LTC_WriteBasicConfig(void);
void LTC_DischargeCells(uint8_t boardid, uint16_t DischargeCells, uint8_t DischargeTime);
void LTC_ReadRegister(uint8_t boardid, uint16_t reg, LTC_RegisterPacketStruct* data);
void LTC_WriteRegister(uint16_t cmd, LTC_RegisterPacketStruct* data);
void LTC_ReadVoltageTotal(void);

void LTC_FailHandler(void);

#define LTC_GPIO_MODE_PULL_DOWN 0
#define LTC_GPIO_MODE_PULL_DOWN_OFF 1


#ifdef __cplusplus
}
#endif

#endif // __LTC6804_H__
