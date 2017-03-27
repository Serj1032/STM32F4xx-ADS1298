#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18


#define WHOAMI 					0x75
#define CONFIG 					0x1A
#define GYRO_CONFIG 		0x1B
#define ACCEL_CONFIG 		0x1C
#define ACCEL_CONFIG2 	0x1D
#define FIFO_EN 				0x23
#define INT_PIN_CFG 		0x37
#define INT_ENABLE 			0x38
#define INT_STATUS 			0x3A
#define ACCEL_XOUT_H 		0x3B
#define ACCEL_XOUT_L 		0x3C
#define ACCEL_YOUT_H 		0x3D
#define ACCEL_YOUT_L 		0x3E
#define ACCEL_ZOUT_H 		0x3F
#define ACCEL_ZOUT_L 		0x40


extern bool verbose;

void MPU_Init(void);

uint8_t MPU_RREG(uint8_t _address);
void MPU_WREG(uint8_t _address, uint8_t _value);
void MPU_printRegisterName(uint8_t _address);

void MPU_ReadAcc(void);
void MPU_SendData(void);

extern uint8_t transferMPU(uint8_t send);

extern void USB_Print(unsigned char* string);
extern void USB_SendBits(uint8_t b);
extern void USB_SendNumber(int32_t num);

extern void USB_Send2Byte(uint8_t* fb);
extern void USB_Send3Byte(uint8_t* fb);
#endif // __MPU9250_H
