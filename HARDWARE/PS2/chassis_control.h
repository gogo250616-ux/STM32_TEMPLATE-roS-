#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include "stm32f4xx_hal.h"

/*
 * 1: continuous CAN transmit test; ROS and PS2 commands are ignored.
 *    After a 3 s safety delay, continuously send the measured forward frame
 *    0x603: 2B 00 00 00 F6 0A 00 00 at the TIM3 rate (20 Hz).
 *    Disconnect the motor controller when this is only a CANgaroo test.
 * 0: normal ROS/PS2 dual-control operation.
 */
#define CHASSIS_FORWARD_TEST_MODE  1U

void ChassisControl_Init(void);
void ChassisControl_SetRosCommand(int8_t left, int8_t right);
void ChassisControl_SetPs2Command(int8_t left, int8_t right);
void ChassisControl_SetManualMode(uint8_t enabled);
void ChassisControl_EmergencyStop(void);
void ChassisControl_Tick(void);

#endif
