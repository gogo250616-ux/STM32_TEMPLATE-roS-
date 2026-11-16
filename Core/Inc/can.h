#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "common.h"

extern CAN_HandleTypeDef hcan1;

/* 兼容旧工程里可能还会引用到的变量 */
extern uint8_t control_cmmd[8];
extern uint8_t status_cmmd[8];
extern uint8_t left_rev_cmmd[8];
extern uint8_t right_rev_cmmd[8];
extern uint8_t max_rev_cmmd[8];
extern uint8_t min_rev_cmmd[8];
extern UnionData leftVelNow;
extern UnionData rightVelNow;
extern uint16_t leftVel_Data;
extern uint16_t rightVel_Data;
extern uint8_t left_status;
extern uint8_t right_status;
extern uint8_t status;
extern const float wheel_perimeter;

extern volatile uint32_t g_last_rx_id;
extern volatile uint8_t  g_last_rx_data[8];
extern volatile uint8_t  g_last_rx_dlc;
extern volatile uint32_t g_rx_count;

void MX_CAN1_Init(void);
void CAN_FeedbackTask(void);
void CAN_GetWheelSpeed(int16_t *left, int16_t *right);
void can_filterconfig_and_init(void);
void CAN1_Send(uint32_t id, uint8_t dlc, uint8_t* data);
void CAN_Send_Move_Node3(void);
void CAN_Send_Stop_Node3(void);

#ifdef __cplusplus
}
#endif

#endif
