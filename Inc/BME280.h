#ifndef __BME280_H
#define __BME280_H

#include "stdbool.h"
#include "stm32f4xx_hal.h"

#define	BME280_REGISTER_DIG_T1               0x88
#define	BME280_REGISTER_DIG_T2               0x8A
#define BME280_REGISTER_DIG_T3               0x8C

#define	BME280_REGISTER_DIG_P1               0x8E
#define	BME280_REGISTER_DIG_P2               0x90
#define	BME280_REGISTER_DIG_P3               0x92
#define	BME280_REGISTER_DIG_P4               0x94
#define	BME280_REGISTER_DIG_P5               0x96
#define	BME280_REGISTER_DIG_P6               0x98
#define	BME280_REGISTER_DIG_P7               0x9A
#define	BME280_REGISTER_DIG_P8               0x9C
#define	BME280_REGISTER_DIG_P9               0x9E

#define	BME280_REGISTER_DIG_H1               0xA1
#define	BME280_REGISTER_DIG_H2               0xE1
#define	BME280_REGISTER_DIG_H3               0xE3
#define	BME280_REGISTER_DIG_H4               0xE4
#define	BME280_REGISTER_DIG_H5               0xE5
#define	BME280_REGISTER_DIG_H6               0xE7

#define	BME280_REGISTER_CHIPID              0xD0
#define BME280_REGISTER_VERSION             0xD1
#define	BME280_REGISTER_SOFTRESET           0xE0

#define	BME280_REGISTER_CAL26               0xE1  // R calibration stored in 0xE1-0xF0

#define	BME280_REGISTER_CONTROLHUMID        0xF2
#define	BME280_REGISTER_STATUS              0XF3
#define	BME280_REGISTER_CONTROL             0xF4
#define	BME280_REGISTER_CONFIG              0xF5
#define	BME280_REGISTER_PRESSUREDATA        0xF7
#define	BME280_REGISTER_TEMPDATA            0xFA
#define	BME280_REGISTER_HUMIDDATA           0xFD

#define BME280_ADDR 	0x77

#define	MODE_SLEEP  	0b00
#define	MODE_FORCED  	0b01
#define MODE_NORMAL		0b11

#define	SAMPLING_NONE	0b000
#define	SAMPLING_X1   0b001
#define	SAMPLING_X2   0b010
#define	SAMPLING_X4   0b011
#define	SAMPLING_X8   0b100
#define	SAMPLING_X16  0b101

		
extern I2C_HandleTypeDef hi2c1;
extern bool verbose;

uint8_t BME_RREG(uint8_t addr);
void BME_WREG(uint8_t addr, uint8_t data);

void BME_readData(void);

void BME_getDIG_T(void);
void BME_getDIG_P(void);
void BME_getDIG_H(void);

void BME_ReadPreassure(void);
void BME_ReadTemp(void);
void BME_ReadHumid(void);
void BME_SendData(void);
	
void BME_Init(void);

int32_t BME280_compensate_T_int32(int32_t adc_T);
uint32_t BME280_compensate_P_int32(int32_t adc_P);
uint32_t BME280_CalcH(int32_t UH);		
		
void BME_printRegisterName(uint8_t _address);
					
extern uint8_t transferSPI(uint8_t send);
extern void USB_Print(unsigned char* string);
extern void USB_SendBits(uint8_t b);
extern void USB_SendNumber(int32_t num);

extern void USB_Send2Byte(uint8_t* fb);
extern void USB_Send3Byte(uint8_t* fb);
extern void USB_Send4Byte(uint8_t* fb);
#endif //__BME280_H

