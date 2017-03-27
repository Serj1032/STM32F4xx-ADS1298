#ifndef __MICROSD_H
#define __MICROSD_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "fatfs.h"

void testSDCard(void);

extern void USB_Print(unsigned char* string);
extern void USB_SendBits(uint8_t b);
extern void USB_SendNumber(int32_t num);


#endif //__MICROSD_H

