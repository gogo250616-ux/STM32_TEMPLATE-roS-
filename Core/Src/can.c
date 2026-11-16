/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */
uint8_t control_cmmd[8]={0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t status_cmmd[8]={0x40, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t left_rev_cmmd[8]={0x40, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t right_rev_cmmd[8]={0x40, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t max_rev_cmmd[8]={0x2B, 0x05, 0x01, 0x00, 0xB8, 0x0B, 0x00, 0x00};
uint8_t min_rev_cmmd[8]={0x2B, 0x06, 0x01, 0x00, 0x1E, 0x00, 0x00, 0x00};

//float leftVel=0, rightVel=0;
uint16_t leftVel_Data=0, rightVel_Data=0;
uint8_t left_status = 0xFF, right_status = 0xFF;
uint8_t status = 0;

UnionData leftVelNow={.d=0},rightVelNow={.d=0};

const float wheel_perimeter = 3.1415926535f * 0.20f;

static volatile uint8_t feedback_pending, feedback_step;
static volatile uint32_t feedback_stamp, feedback_count;
typedef struct {
    uint16_t rpm_a, rpm_b;
    uint8_t state_a, state_b;
    int16_t left_mm_s, right_mm_s;
    uint32_t sample_ms, span_ms, sequence;
} FeedbackSnapshot;
static volatile FeedbackSnapshot feedback_snapshot;
static uint16_t pending_rpm_a, pending_rpm_b;

static uint8_t GetFeedbackSnapshot(FeedbackSnapshot *sample, uint32_t *age)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    *sample = feedback_snapshot;
    *age = sample->sequence ? (uint32_t)(HAL_GetTick() - sample->sample_ms) : 0U;
    __set_PRIMASK(irq);
    return sample->sequence != 0U && *age <= 250U;
}
static uint32_t feedback_request_ms, feedback_cycle_ms, feedback_print_ms;
static volatile uint32_t can_tx_queued_count;
static volatile uint32_t can_tx_fail_count;
static volatile uint32_t can_tx_last_error;
static volatile uint32_t can_tx_last_esr;
static volatile uint32_t can_tx_last_fail_ms;
static uint32_t can_recovery_count;
static uint32_t can_last_recovery_ms;

void CAN_GetWheelSpeed(int16_t *left, int16_t *right)
{
    FeedbackSnapshot sample;
    uint32_t age;
    uint8_t valid = GetFeedbackSnapshot(&sample, &age);
    *left = valid ? sample.left_mm_s : 0;
    *right = valid ? sample.right_mm_s : 0;
}

void CAN_FeedbackTask(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t esr = hcan1.Instance->ESR;

    /* A missing ACK or a wiring/bit-rate fault can leave all three mailboxes
       occupied. Abort stale requests and restart CAN at a limited rate so the
       controller can recover after the bus is corrected or reconnected. */
    if (((esr & CAN_ESR_BOFF) != 0U ||
         HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) &&
        (uint32_t)(now - can_last_recovery_ms) >= 1000U)
    {
        can_last_recovery_ms = now;
        (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 |
                                             CAN_TX_MAILBOX1 |
                                             CAN_TX_MAILBOX2);
        (void)HAL_CAN_Stop(&hcan1);
        if (HAL_CAN_Start(&hcan1) == HAL_OK)
        {
            (void)HAL_CAN_ActivateNotification(&hcan1,
                                                CAN_IT_RX_FIFO0_MSG_PENDING);
            can_recovery_count++;
        }
        feedback_pending = 0U;
        feedback_step = 0U;
    }
    if (feedback_pending && (uint32_t)(now - feedback_request_ms) >= 50U) {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        if (feedback_pending && (uint32_t)(HAL_GetTick() - feedback_request_ms) >= 50U) {
            feedback_pending = 0U;
            feedback_step = 0U;
        }
        __set_PRIMASK(irq);
    }
    if (!feedback_pending &&
        (feedback_step != 0U || (uint32_t)(now - feedback_cycle_ms) >= 50U)) {
        CAN_TxHeaderTypeDef tx = {0};
        uint32_t mailbox, irq;
        uint8_t request[8] = {0x40, 0, 0x02, 0, 0, 0, 0, 0};
        tx.StdId = 0x603U;
        tx.IDE = CAN_ID_STD;
        tx.RTR = CAN_RTR_DATA;
        tx.DLC = 8U;
        request[1] = (uint8_t)(3U + feedback_step);
        irq = __get_PRIMASK();
        __disable_irq();
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
            feedback_pending = request[1];
            if (HAL_CAN_AddTxMessage(&hcan1, &tx, request, &mailbox) == HAL_OK) {
                feedback_request_ms = now;
                if (feedback_step == 0U) feedback_cycle_ms = now;
            } else feedback_pending = 0U;
        }
        __set_PRIMASK(irq);
    }
    if (!feedback_count || (uint32_t)(now - feedback_stamp) > 250U) {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        if (!feedback_count || (uint32_t)(HAL_GetTick() - feedback_stamp) > 250U) {
            leftVelNow.d = 0;
            rightVelNow.d = 0;
        }
        __set_PRIMASK(irq);
    }
    if ((uint32_t)(now - feedback_print_ms) >= 1000U) {
        FeedbackSnapshot sample;
        uint32_t age;
        uint8_t valid = GetFeedbackSnapshot(&sample, &age);
        feedback_print_ms = now;
        printf("SPEED valid=%u count=%lu sample_ms=%lu span_ms=%lu age_ms=%lu A_rpm=%u B_rpm=%u A_state=%u B_state=%u L_mm_s=%d R_mm_s=%d\r\n",
            (unsigned)valid, (unsigned long)sample.sequence,
            (unsigned long)sample.sample_ms, (unsigned long)sample.span_ms,
            (unsigned long)age, (unsigned)sample.rpm_a, (unsigned)sample.rpm_b,
            (unsigned)sample.state_a, (unsigned)sample.state_b,
            (int)sample.left_mm_s, (int)sample.right_mm_s);
        esr = hcan1.Instance->ESR;
        printf("CAN TX queued=%lu fail=%lu recover=%lu last_fail_ms=%lu state=%u error=0x%08lX ESR=0x%08lX TEC=%lu REC=%lu LEC=%lu BOFF=%u free=%lu PCLK1=%lu BTR=0x%08lX\r\n",
            (unsigned long)can_tx_queued_count,
            (unsigned long)can_tx_fail_count,
            (unsigned long)can_recovery_count,
            (unsigned long)can_tx_last_fail_ms,
            (unsigned)HAL_CAN_GetState(&hcan1),
            (unsigned long)can_tx_last_error,
            (unsigned long)esr,
            (unsigned long)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos),
            (unsigned long)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos),
            (unsigned long)((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos),
            (unsigned)((esr & CAN_ESR_BOFF) != 0U),
            (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
            (unsigned long)HAL_RCC_GetPCLK1Freq(),
            (unsigned long)hcan1.Instance->BTR);
    }
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  /* Keep retrying when an ACK is temporarily missing and recover from
     bus-off automatically. This is important when CANgaroo or the motor
     controller is connected after the STM32 has already started. */
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*void CANFilter_Config(void)
{
		HAL_StatusTypeDef HAL_Status;
    CAN_FilterTypeDef Filter0;
    Filter0.FilterBank = 1;
    Filter0.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter0.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter0.FilterIdHigh = 0x00;
    Filter0.FilterIdLow = 0x00;
    Filter0.FilterMaskIdHigh = 0x00;
    Filter0.FilterMaskIdLow = 0x00;
    Filter0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    Filter0.FilterActivation = CAN_FILTER_ENABLE;

    HAL_Status = HAL_CAN_ConfigFilter(&hcan1,&Filter0);
    if(HAL_Status != HAL_OK)
    {
        Error_Handler();
    }
}*/

/*void CAN_Start_Init(void)
{
		if (HAL_CAN_Start(&hcan1) != HAL_OK) 
		{
				Error_Handler();
				 
		}
		else
		{
				printf("HAL_CAN_Start is HAL_OK!\r\n");
		}
			
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) !=  HAL_OK)
		{
        Error_Handler();
    }
		else
		{ 
				printf("HAL_CAN_ActivateNotification is HAL_OK!\r\n"); 
		}
}*/

void CAN1_Send(uint32_t id, uint8_t dlc, uint8_t* data)
{
#if PS2_USB_ONLY_TEST
	(void)id;
	(void)dlc;
	(void)data;
	return;
#else
		CAN_TxHeaderTypeDef TxMessage = {0};
	  uint32_t TxMailbox;
		
    TxMessage.IDE = CAN_ID_STD; 
		TxMessage.StdId = id; 
    TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = dlc;
		if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, data, &TxMailbox) == HAL_OK)
		{
				can_tx_queued_count++;
		}
		else
		{
				/* CAN1_Send is also called from TIM3 ISR: never print or block here. */
				can_tx_fail_count++;
				can_tx_last_error = HAL_CAN_GetError(&hcan1);
				can_tx_last_esr = hcan1.Instance->ESR;
				can_tx_last_fail_ms = HAL_GetTick();
		}		
#endif
}

void can_filterconfig_and_init(void)
{
		CAN_FilterTypeDef Filter0;
    Filter0.FilterBank = 0;
    Filter0.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter0.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter0.FilterIdHigh = 0x0000;
    Filter0.FilterIdLow = 0x0000;
    Filter0.FilterMaskIdHigh = 0x0000;
    Filter0.FilterMaskIdLow = 0x0000;
    Filter0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    Filter0.FilterActivation = CAN_FILTER_ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&Filter0) != HAL_OK)
	{
		Error_Handler();
		printf("HAL_CAN_ConfigFilter is HAL_ERROR!\r\n");
	}	
	else
	{
		printf("HAL_CAN_ConfigFilter is HAL_OK!\r\n");
	}
		
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
		printf("HAL_CAN_Start is HAL_ERROR!\r\n");
	}	
	else
	{
		printf("HAL_CAN_Start is HAL_OK!\r\n");
	}
	
	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
		printf("HAL_CAN_ActivateNotification is HAL_ERROR!\r\n");
	}	
	else
	{
		printf("HAL_CAN_ActivateNotification is HAL_OK!\r\n");
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    if (hcan->Instance != CAN1) return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) break;
        if (rx.IDE != CAN_ID_STD || rx.RTR != CAN_RTR_DATA ||
            rx.StdId != 0x583U || rx.DLC != 8U || data[0] != 0x4BU ||
            data[2] != 0x02U || data[3] != 0U || !feedback_pending ||
            data[1] != feedback_pending) continue;
        if (data[1] == 3U) {
            pending_rpm_a = (uint16_t)((uint16_t)data[5] << 8 | data[4]);
            feedback_step = 1U;
        } else if (data[1] == 4U) {
            pending_rpm_b = (uint16_t)((uint16_t)data[5] << 8 | data[4]);
            feedback_step = 2U;
        } else {
            feedback_step = 0U;
            if (data[5] <= 3U && data[4] <= 3U &&
                pending_rpm_a <= 3600U && pending_rpm_b <= 3600U) {
                FeedbackSnapshot sample;
                leftVel_Data = pending_rpm_a;
                rightVel_Data = pending_rpm_b;
                left_status = data[5];
                right_status = data[4];
                int32_t left = (int32_t)(1000.0f * leftVel_Data * wheel_perimeter / 1800.0f + 0.5f);
                int32_t right = (int32_t)(1000.0f * rightVel_Data * wheel_perimeter / 1800.0f + 0.5f);
                leftVelNow.d = left_status == 1U ? left : (left_status == 2U ? -left : 0);
                rightVelNow.d = right_status == 1U ? -right : (right_status == 2U ? right : 0);
                feedback_stamp = HAL_GetTick();
                feedback_count++;
                sample.rpm_a = leftVel_Data;
                sample.rpm_b = rightVel_Data;
                sample.state_a = left_status;
                sample.state_b = right_status;
                sample.left_mm_s = (int16_t)leftVelNow.d;
                sample.right_mm_s = (int16_t)rightVelNow.d;
                sample.sample_ms = feedback_stamp;
                sample.span_ms = (uint32_t)(feedback_stamp - feedback_cycle_ms);
                sample.sequence = feedback_count;
                /* ISR publishes only after all three responses are complete.
                   Readers copy under a short interrupt mask; printing is outside. */
                feedback_snapshot = sample;
            }
        }
        feedback_pending = 0U;
    }
}

#if 0 /* Superseded by validated node-3 polling above. */
void Legacy_CAN_RxCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxMessage;
	uint8_t data[8];
	
	if(hcan->Instance==CAN1)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data) == HAL_OK)
		 {
				/*if((data[0] == 0x60)&&(data[1] == 0x05))
				{
					 printf("max_rev is ok!");
				}
				
				if((data[0] == 0x60)&&(data[1] == 0x06))
				{
					 printf("min_rev is ok!");
				}*/
				
			  if((data[0] == 0x60)&&(data[1] == 0x00))
				{
					CAN1_Send(0x600, 8, left_rev_cmmd);
				}
			 
			  if((data[0] == 0x4B)&&(data[1]==0x03))
				{
					leftVel_Data = (uint16_t)((data[5] << 8)| data[4]);
					CAN1_Send(0x600, 8, right_rev_cmmd);
				}
				
				if((data[0] == 0x4B)&&(data[1]==0x04))
				{
					rightVel_Data = (uint16_t)((data[5] << 8)| data[4]);
					CAN1_Send(0x600, 8, status_cmmd);//0x4B
				}
				
				if((data[0] == 0x4B)&&(data[1]==0x05))
				{
					//left_status = data[5];
					//right_status = data[4];
					//printf("Lvel=%d,Rvel=%d\r\n", data[5], data[4]);
					//status = 1;
					if(data[5]==0x02)
					{
						leftVelNow.d = (int32_t)(-1000.0f*(leftVel_Data* wheel_perimeter) / 1800.0f);
					}
					else
					{
						leftVelNow.d  = (int32_t)(1000.0f*(leftVel_Data* wheel_perimeter) / 1800.0f );
					}
						
						
					if(data[4]==0x02)
					{
						rightVelNow.d = (int32_t)((-1000.0f*rightVel_Data* wheel_perimeter) / 1800.0f );
					}
					else
					{
						rightVelNow.d = (int32_t)((1000.0f*rightVel_Data* wheel_perimeter) / 1800.0f );
					}
					
					//printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
					
				}
			 
			 
			  /*if(data[0] == 0x4B)
				{
					
					if(data[1]==0x03)
					{
						leftVel_Data = (uint16_t)((data[5] << 8)| data[4]);
						//printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
					}
					else if(data[1]==0x04)
					{
						rightVel_Data = (uint16_t)((data[5] << 8)| data[4]);
						//printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
					}
					else if(data[1]==0x05)
					{
						printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
						left_status = data[5];
						right_status = data[4];
						//printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
						
						status = 1;
					}*/
					
					/*switch(data[1])
					{
						case 0x03:
							//leftVel = (float)((data[5] << 8)| data[4]) * wheel_perimeter / 60.0f;
						  leftVel_Data = (uint16_t)((data[5] << 8)| data[4]);
							break;
						case 0x04:
							//rightVel = (float)((data[5] << 8)| data[4]) * wheel_perimeter / 60.0f;
							rightVel_Data = (uint16_t)((data[5] << 8)| data[4]);
							break;
						case 0x05:
							left_status = data[5];
							right_status = data[4];
						  status = 1;
							break;
					}
				}*/
			 
			 
				/*
				if((data[0] == 0x4B) && (data[1] == 0x05)) //normal:0x4B,loopback:0x40
				{
					//printf("left_rev:\r\n");
					//printf("data[5] is 0x%02x\r\n", data[5]);
					//printf("data[4] is 0x%02x\r\n", data[4]);
					//leftVel = (float)((data[5] << 8)| data[4]) * wheel_perimeter / 60.0f;
					//printf("leftVel: %f\r\n", leftVel);
					left_status = data[5];
					right_status = data[4];
					status = 1;
				}
				
				if(status)
				{
					if((data[0] == 0x4B) && (data[1] == 0x03)) //normal:0x4B,loopback:0x40
				{
					//printf("left_rev:\r\n");
					//printf("data[5] is 0x%02x\r\n", data[5]);
					//printf("data[4] is 0x%02x\r\n", data[4]);
					leftVel = (float)((data[5] << 8)| data[4]) * wheel_perimeter / 60.0f;
					//printf("leftVel: %f\r\n", leftVel);
					if(left_status==0x02) leftVel *= -1.0f;
				}
				
				if((data[0] == 0x4B) && (data[1] == 0x04)) //normal:0x4B,loopback:0x40
				{
					//printf("right_rev:\r\n");
					//printf("data[5] is 0x%02x\r\n", data[5]);
					//printf("data[4] is 0x%02x\r\n", data[4]);
					rightVel = (float)((data[5] << 8)| data[4]) * wheel_perimeter / 60.0f;
					//printf("rightVel: %f\r\n", rightVel);
					if(right_status==0x02) rightVel *= -1.0f;
				}
				
				status = 0;
				
				}*/
				
					
		 }
		 
	}
	
}

#endif
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
