#ifndef INC_ROSNFC_H_
#define INC_ROSNFC_H_
#include "stm32f4xx_hal.h"
#include <string.h>
#include "can.h"
#include "im948.h"
#include "adc.h"


extern int16_t leftSpeedSet, rightSpeedSet;
extern uint8_t service1, service2;
extern uint8_t sendCtrlFlag, receCtrlFlag;

extern uint8_t USARTROS_Receiver;
extern uint8_t receiveBuffROS[13];

extern uint8_t USARTNFC_Receiver;
extern uint8_t receiveBuffNFC[4];
extern float leftVel, rightVel, pitch, roll, yaw;

uint8_t getCrc8(uint8_t *ptr, uint16_t len);
void usartSendData(UART_HandleTypeDef* USARTx, uint8_t sendCtrlFlag);
void usartRosReceiveOneData(UART_HandleTypeDef* USARTx, uint8_t *receCtrlFlag);
void usartNfcReceiveOneData(UART_HandleTypeDef* USARTx, UART_HandleTypeDef* USARTROS);

#endif /* INC_ROSNFC_H_ */
