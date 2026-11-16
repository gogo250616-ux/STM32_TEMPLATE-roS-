#ifndef __PS2_H__
#define __PS2_H__

#include "stm32f4xx_hal.h"

/* USB OTG FS: PA11=D-, PA12=D+ (AF10). */
void PS2_Init(void);
void PS2_Task(void);
uint8_t PS2_IsManualMode(void);

#endif
