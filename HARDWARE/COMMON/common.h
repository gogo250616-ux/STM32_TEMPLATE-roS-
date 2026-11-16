#ifndef INC_COMMON_H_
#define INC_COMMON_H_
#include "stm32f4xx_hal.h"

typedef union 
{
	int16_t d;
	uint8_t data[2];
} UnionData;

#endif
