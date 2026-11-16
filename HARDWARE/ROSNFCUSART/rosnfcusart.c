#include "rosnfcusart.h"
#include "chassis_control.h"

uint8_t sendCtrlFlag = 0;
uint8_t receCtrlFlag = 0;

int16_t leftSpeedSet = 0, rightSpeedSet = 0;
uint8_t service1=0, service2=0;
float leftVel = 0, rightVel = 0, pitch=0, roll=0, yaw=0;
const uint8_t header[2] = {0x55, 0xaa};
const uint8_t ender[2]  = {0x0d, 0x0a};

uint8_t USARTROS_Receiver = 0;
uint8_t receiveBuffROS[13] = {0};
uint8_t USARTROSBufferIndex = 0;

uint8_t USARTNFC_Receiver = 0;
uint8_t receiveBuffNFC[4] = {0};
uint8_t USARTNFCBufferIndex = 0;

uint8_t getCrc8(uint8_t *ptr, uint16_t len)
{
		uint8_t crc;
		uint8_t i;
		crc = 0;
		while(len--)
		{
			crc ^= *ptr++;
			for(i = 0; i < 8; i++)
			{
				if(crc&0x01)
									crc=(crc>>1)^0x8C;
				else 
									crc >>= 1;
			}
		}
		
		return crc;
}

void usartSendData(UART_HandleTypeDef* USARTx, uint8_t sendCtrlFlag)
{
	
		uint8_t buf[19] = {0};
		int16_t feedback_left, feedback_right;
		int i, length = 0;
		CAN_GetWheelSpeed(&feedback_left, &feedback_right);

		for(i = 0; i < 2; i++)
			buf[i] = header[i]; 
		
		length = 13;
		buf[2] = length;
		
		/*for(i = 0; i < 4; i++)
		{
			buf[i + 3] = leftVelNow.data[i];         // buf[3]lowbit buf[4] buf[5] buf[6]highbit 
			buf[i + 7] = rightVelNow.data[i];        // buf[7] buf[8] buf[9] buf[10]
		}*/
		
			
		for(i=0; i < 2; i++)
		{
			buf[i + 3] = (uint8_t)((uint16_t)feedback_left >> (8 * i));
			buf[i + 5] = (uint8_t)((uint16_t)feedback_right >> (8 * i));
			buf[i + 7] = pitchNow.data[i];           // buf[7] buf[8]
			buf[i + 9] = rollNow.data[i];           // buf[9] buf[10]
			buf[i + 11] = yawNow.data[i];           // buf[11] buf[12]
			buf[i + 13] = voltageNow.data[i];       // buf[13] buf[14]
		}
		
		buf[3 + length - 1] = sendCtrlFlag;              // buf[15]
		
		buf[3 + length] = getCrc8(buf, 3 + length);  // buf[16]
		buf[3 + length + 1] = ender[0];              // buf[17]
		buf[3 + length + 2] = ender[1];              // buf[18]
		
		//HAL_UART_Transmit_IT(USARTx, buf,sizeof(buf));
		HAL_UART_Transmit(USARTx, buf,sizeof(buf),100);
		/*if(HAL_UART_GetState(USARTx)==HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit_IT(USARTx, buf,sizeof(buf));
		}*/
}

void usartNfcReceiveOneData(UART_HandleTypeDef* USARTx, UART_HandleTypeDef* USARTROS)
{
		receiveBuffNFC[USARTNFCBufferIndex++] = USARTNFC_Receiver;
	  if(receiveBuffNFC[0] != 0x0A)
	 {
		 USARTNFCBufferIndex = 0;
	 }
	  
	  if(receiveBuffNFC[USARTNFCBufferIndex-1] == 0x0D)
		{
			uint8_t sum = 0;
			for(int i = 0; i < USARTNFCBufferIndex-2; i++)
			{
				sum += receiveBuffNFC[i];
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x41))
			{
				  control_cmmd[4] = 0x32;
				  control_cmmd[5] = 0x32;
				  CAN1_Send(0x603, 8, control_cmmd);
				  HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x42))
			{
				  control_cmmd[4] = 0x00;
				  control_cmmd[5] = 0x00;
				  CAN1_Send(0x603, 8, control_cmmd);
				  HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x43))
			{
				 control_cmmd[4] = 0xCE;
				 control_cmmd[5] = 0xCE;
				 CAN1_Send(0x603, 8, control_cmmd);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x44))
			{
				 control_cmmd[4] = 0x32;
				 control_cmmd[5] = 0xCE;
				 CAN1_Send(0x603, 8, control_cmmd);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x45))
			{
				 control_cmmd[4] = 0xCE;
				 control_cmmd[5] = 0x32;
				 CAN1_Send(0x603, 8, control_cmmd);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x46))
			{
				 control_cmmd[4] = 0x19;
				 control_cmmd[5] = 0x32;
				 CAN1_Send(0x603, 8, control_cmmd);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x47))
			{
				 control_cmmd[4] = 0x32;
				 control_cmmd[5] = 0x19;
				 CAN1_Send(0x603, 8, control_cmmd);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x48))
			{
				 //ros_control = 0;
				 __HAL_UART_DISABLE_IT(USARTROS, UART_IT_RXNE);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			if((sum == receiveBuffNFC[USARTNFCBufferIndex-2])&&(receiveBuffNFC[1] == 0x49))
			{
				 //ros_control = 1;
				 __HAL_UART_ENABLE_IT(USARTROS, UART_IT_RXNE);
				 HAL_UART_Transmit(USARTx, receiveBuffNFC,4, 100);
			}
			
			USARTNFCBufferIndex = 0;
			memset(receiveBuffNFC,0x00,sizeof(receiveBuffNFC));
		}
		
	 HAL_UART_Receive_IT(USARTx, &USARTNFC_Receiver, 1);
}

void usartRosReceiveOneData(UART_HandleTypeDef* USARTx, uint8_t *receCtrlFlag)
{
	/* Fixed 13-byte binary frame. Payload may itself contain 0x0A. */
	if (USARTROSBufferIndex >= sizeof(receiveBuffROS)) USARTROSBufferIndex = 0;
	receiveBuffROS[USARTROSBufferIndex++] = USARTROS_Receiver;
	if (USARTROSBufferIndex == 1U && receiveBuffROS[0] != header[0])
		USARTROSBufferIndex = 0;
	else if (USARTROSBufferIndex == 2U && receiveBuffROS[1] != header[1])
		USARTROSBufferIndex = 0;
	else if (USARTROSBufferIndex == 3U && receiveBuffROS[2] != 7U)
		USARTROSBufferIndex = 0;
	else if (USARTROSBufferIndex == sizeof(receiveBuffROS))
	{
		if (receiveBuffROS[11] == ender[0] && receiveBuffROS[12] == ender[1] &&
			getCrc8(receiveBuffROS, 10U) == receiveBuffROS[10])
		{
			int16_t left = (int16_t)((receiveBuffROS[4] << 8) | receiveBuffROS[3]);
			int16_t right = (int16_t)((receiveBuffROS[6] << 8) | receiveBuffROS[5]);
			service1 = receiveBuffROS[7];
			service2 = receiveBuffROS[8];
			*receCtrlFlag = receiveBuffROS[9];
			left /= 30;
			right /= 30;
			if (left > 100) left = 100;
			if (left < -100) left = -100;
			if (right > 100) right = 100;
			if (right < -100) right = -100;
			leftSpeedSet = left;
			rightSpeedSet = right;
			ChassisControl_SetRosCommand((int8_t)left, (int8_t)right);
		}
		USARTROSBufferIndex = 0;
		memset(receiveBuffROS, 0, sizeof(receiveBuffROS));
	}
	
	 HAL_UART_Receive_IT(USARTx, &USARTROS_Receiver, 1);
}
