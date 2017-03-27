/*==============================================================================
* Reference: https://github.com/OpenBCI/OpenBCI-V2hardware-DEPRECATED/tree/master/Arduino/Libraries/ADS1299
* Datasheet: http://www.ti.com/lit/ds/symlink/ads1298.pdf
* Author: Kravchenko S.
*
*
*==============================================================================*/

#ifndef __ADS1298_H
#define __ADS1298_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define _WAKEUP 	0x02 // Wake-up from standby mode
#define _STANDBY 	0x04 // Enter Standby mode
#define _RESET 		0x06 // Reset the device registers to default
#define _START 		0x08 // Start and restart (synchronize) conversions
#define _STOP 		0x0A // Stop conversion
#define _RDATAC 	0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC 	0x11 // Stop Read Data Continuous mode
#define _RDATA 		0x12 // Read data by command; supports multiple read back

//Register Addresses
#define ID 0x00
#define CONFIG1 0x01
#define CONFIG2 0x02
#define CONFIG3 0x03
#define LOFF 0x04
#define CH1SET 0x05
#define CH2SET 0x06
#define CH3SET 0x07
#define CH4SET 0x08
#define CH5SET 0x09
#define CH6SET 0x0A
#define CH7SET 0x0B
#define CH8SET 0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP 0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO 0x14
#define MISC1 0x15
#define MISC2 0x16
#define CONFIG4 0x17

#define CONFIG1_HR 							0x80
#define CONFIG1_LP 							0x00
#define CONFIG1_DAISY_EN				0x00
#define CONFIG1_DAIST_MUL				0x40
#define CONFIG1_CLK_EN_OUT			0x00
#define CONFIG1_CLK_DIS_OUT			0x20
#define CONFIG1_DR_250SPS				0x06 

extern bool verbose;

extern int32_t channelData [16];

void ADS_Init(void);

void ADS_RDATAC(void);
void ADS_SDATAC(void);
void ADS_RESET(void);
void ADS_START(void);
void ADS_STOP(void);
void ADS_WAKEUP(void);

uint8_t ADS_getDeviceID(void);
uint8_t ADS_RREG(uint8_t _address);

void ADS_RREGS(uint8_t _address, uint8_t _numRegistersMinusOne);
void ADS_WREG(uint8_t _address, uint8_t _value);
void ADS_WREGS(uint8_t _address, uint8_t _numRegistersMinusOne);
void ADS_updateChannelData(void);
void ADS_RDATA(void);
void ADS_printRegisterName(uint8_t _address);

void ADS_sendUSBData(void);

extern uint8_t transferSPI(uint8_t send);

extern void USB_Print(unsigned char* string);
extern void USB_SendBits(uint8_t b);
extern void USB_SendNumber(int32_t num);
extern void USB_Send4Byte(uint8_t* fb);

#endif //__ADS1298_H
