#ifndef __SERVICE_H
#define __SERVICE_H

#include "stm32f4xx_hal.h"
#include "tim.h"

void Service_Init(void);
void Service_SetAngle(uint8_t angle1, uint8_t angle2);

#endif
