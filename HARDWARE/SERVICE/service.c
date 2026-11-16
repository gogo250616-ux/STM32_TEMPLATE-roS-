#include "service.h"

void Service_Init(void)
{
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Service_SetAngle(uint8_t angle1, uint8_t angle2)
{
	uint32_t pulse1 = 25 + (angle1 * 5 / 9);
  if (pulse1 < 25) pulse1 = 25;
  if (pulse1 > 125) pulse1 = 125;
	
	uint32_t pulse2 = 25 + (angle2 * 5 / 9);
  if (pulse2 < 25) pulse2 = 25;
  if (pulse2 > 125) pulse2 = 125;

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse1);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulse2);
}
