#ifndef INC_IM948_H_
#define INC_IM948_H_
#include "stm32f4xx_hal.h"
#include <string.h>
#include "common.h"

extern UnionData pitchNow,rollNow,yawNow;
extern uint16_t ctl;

extern uint8_t USARTIM948_Receiver;
extern uint8_t receiveBuffIM948[250];
extern uint8_t sendcmdBuffIM948[86];
extern uint8_t USARTIM948BufferIndex;

void usartim948ReceiveOneData(UART_HandleTypeDef* USARTx);
void usartim948SendCmd(UART_HandleTypeDef* USARTx, uint8_t *pDat, uint8_t DLen);
uint8_t CalcSum1(uint8_t *Buf, uint8_t Len);
uint8_t getCrc8(uint8_t *ptr, uint16_t len);
void im948Cmd03(UART_HandleTypeDef* USARTx);
void im948Cmd05(UART_HandleTypeDef* USARTx);
void im948Cmd06(UART_HandleTypeDef* USARTx);
void im948Cmd12(UART_HandleTypeDef* USARTx);
void im948Cmd13(UART_HandleTypeDef* USARTx);
void im948Cmd19(UART_HandleTypeDef* USARTx);
void im948Init(UART_HandleTypeDef* USARTx);
#endif /* INC_IM948_H_ */

