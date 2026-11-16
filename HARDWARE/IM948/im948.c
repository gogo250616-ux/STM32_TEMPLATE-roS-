#include "im948.h"
#include "usart.h"

UnionData pitchNow={.d=0}, rollNow={.d=0}, yawNow={.d=0};
uint16_t ctl = 0;

uint8_t USARTIM948_Receiver = 0;
uint8_t receiveBuffIM948[250] = {0};
uint8_t USARTIM948BufferIndex = 0;

void usartim948ReceiveOneData(UART_HandleTypeDef* USARTx)
{
	receiveBuffIM948[USARTIM948BufferIndex++] = USARTIM948_Receiver;
	if(receiveBuffIM948[0] != 0x49)
	 {
		 USARTIM948BufferIndex = 0;
	 }
	 
	 if(receiveBuffIM948[USARTIM948BufferIndex-1] == 0x4D)
	 {
		 //printf("%d\r\n",receiveBuffIM948[2]);
		 
		 if(receiveBuffIM948[3] == 0x11)
		 {
				ctl = ((uint16_t)receiveBuffIM948[5] << 8) | receiveBuffIM948[4];
			 
			 if ((ctl & 0x0040) != 0)
        {
          rollNow.d = (int16_t)((receiveBuffIM948[11]<<8) | receiveBuffIM948[10]);
          pitchNow.d = (int16_t)((receiveBuffIM948[13]<<8) | receiveBuffIM948[12]);
          yawNow.d = (int16_t)((receiveBuffIM948[15]<<8) | receiveBuffIM948[14]);
					//printf("%d\r\n",yawNow.d );
        }
		 }
		
			USARTIM948BufferIndex = 0;
			memset(receiveBuffIM948,0x00,sizeof(receiveBuffIM948));
	 }
	
	 HAL_UART_Receive_IT(USARTx, &USARTIM948_Receiver, 1);
}

void usart6SendCmd(UART_HandleTypeDef* USARTx, uint8_t *pDat, uint8_t DLen)
{
		uint8_t buf[86] =
					{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff};
		
		buf[50] = 0x49;
		buf[51] = 255; 
		buf[52] = DLen;
		memcpy(&buf[53], pDat, DLen);
		buf[53+DLen] = CalcSum1(&buf[51], DLen+2);
		buf[54+DLen] = 0x4D;
		HAL_UART_Transmit(USARTx, buf,sizeof(buf), 100);
}

uint8_t CalcSum1(uint8_t *Buf, uint8_t Len)
{
		uint8_t Sum = 0;
    while (Len-- > 0)
    {
        Sum += Buf[Len];
    }
    return Sum;
}

void im948Cmd03(UART_HandleTypeDef* USARTx)
{
	uint8_t data = 0x03;
	usart6SendCmd(USARTx, &data, 1);
}

void im948Cmd05(UART_HandleTypeDef* USARTx)
{
	uint8_t data = 0x05;
	usart6SendCmd(USARTx, &data, 1);
}

void im948Cmd06(UART_HandleTypeDef* USARTx)
{
	uint8_t data = 0x06;
	usart6SendCmd(USARTx, &data, 1);
}

void im948Cmd13(UART_HandleTypeDef* USARTx)
{
	uint8_t data = 0x13;
	usart6SendCmd(USARTx, &data, 1);
}

void im948Cmd12(UART_HandleTypeDef* USARTx)
{
	uint8_t buf[11] = {0x12};
  buf[1] = 5;
  buf[2] = 255;
  buf[3] = 0;
  buf[4] = 6; //cmpass on 0ff
  buf[5] = 200; //200Hz
  buf[6] = 1;
  buf[7] = 3;
  buf[8] = 2;
  buf[9] = 0x40;
  buf[10] =0x00;
	
	usart6SendCmd(USARTx, buf, 11);
}

void im948Cmd19(UART_HandleTypeDef* USARTx)
{
	uint8_t data = 0x19;
	usart6SendCmd(USARTx, &data, 1);
}

void im948Init(UART_HandleTypeDef* USARTx)
{
	int i = 60000000; while (i--);
	
	im948Cmd03(USARTx);
	im948Cmd05(USARTx);
	im948Cmd06(USARTx);
	im948Cmd13(USARTx);
	im948Cmd12(USARTx);
	im948Cmd19(USARTx);
}
