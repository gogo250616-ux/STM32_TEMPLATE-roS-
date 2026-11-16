/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "adc.h"
#include "led.h"
#include "common.h"
#include "service.h"
#include "ps2.h"
#include "chassis_control.h"
//#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t receiveData[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//HAL_NVIC_SetPriority(SysTick_IRQn,0,0);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
#if !PS2_USB_ONLY_TEST
  MX_CAN1_Init();
#endif
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	
#if !PS2_USB_ONLY_TEST
	can_filterconfig_and_init();
#endif
	Adc_POWER_Init();
	OLED_Init();
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
	
	HAL_UART_Receive_IT(&huart6, &USARTIM948_Receiver, 1);
	HAL_UART_Receive_IT(&huart3, &USARTROS_Receiver, 1);
	HAL_UART_Receive_IT(&huart2, &USARTNFC_Receiver, 1);
#if PS2_USB_ONLY_TEST
	printf("\r\n========== USB ONLY TEST BUILD ==========\r\n");
	printf("CAN1 initialization and all CAN traffic are disabled.\r\n");
#else
	printf("\r\n========== NORMAL PS2/ROS + CAN BUILD ==========\r\n");
#endif
	//HAL_TIM_Base_Start_IT(&htim3);
	im948Init(&huart6);
	//uint8_t sendCount = 0;
	
	//CAN1_Send(0x600, 8, max_rev_cmmd);
	//CAN1_Send(0x600, 8, min_rev_cmmd);
	control_cmmd[5] = 0;
	control_cmmd[4] = 0;
#if !PS2_USB_ONLY_TEST
	CAN1_Send(0x600, 8, control_cmmd);
#endif
	ChassisControl_Init();
	
	HAL_GPIO_WritePin(BUZZER_CTR_GPIO_Port, BUZZER_CTR_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(BUZZER_CTR_GPIO_Port, BUZZER_CTR_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim12);
	Service_Init();
	PS2_Init();
	
	//Service_SetAngle(0, 90);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		PS2_Task();
#if !PS2_USB_ONLY_TEST
		CAN_FeedbackTask();
#endif
		//printf("Lvel=%d,Rvel=%d\r\n",leftSpeedSet, rightSpeedSet);
		//usartSendData(&huart3, sendCtrlFlag);
		//usartSendData(&huart2, sendCtrlFlag);
		
		//printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
		//printf("Lvel=%d,Rvel=%d\r\n",leftVel_Data, rightVel_Data);
		
		//printf("ok\r\n");
		
		/*if(status)
		{
			if(left_status==0x02)
			{
				leftVelNow.d = (int32_t)(-1000.0f*(leftVel_Data* wheel_perimeter) / 60.0f);
			}
			else
			{
				leftVelNow.d  = (int32_t)(1000.0f*(leftVel_Data* wheel_perimeter) / 60.0f );
			}
				
				
			if(right_status==0x02)
			{
				rightVelNow.d = (int32_t)((-1000.0f*rightVel_Data* wheel_perimeter) / 60.0f );
			}
			else
			{
				rightVelNow.d = (int32_t)((1000.0f*rightVel_Data* wheel_perimeter) / 60.0f );
			}
			printf("Lvel=%d,Rvel=%d\r\n",leftVelNow.d, rightVelNow.d);
			status = 0;
		}*/
		//Service_SetAngle(90, 90);
		//Service_SetAngle(service1, service2);
		voltageNow.d = (int16_t)(100.f*Get_battery_volt());
		usartSendData(&huart3, sendCtrlFlag);
		usartSendData(&huart2, sendCtrlFlag);
		
		leftVel = (float)leftVelNow.d / 1000.0f;
		rightVel = (float)rightVelNow.d / 1000.0f;
		//rightVel = (float)(((rightVelNow.data[1])<<8|rightVelNow.data[0])) / 1000.0f;
		
		//printf("Lvel=%f,Rvel=%f\r\n",leftVel, rightVel);
		
		//printf("%xd,%xd,%xd\r\n",rightVelNow.data[2],rightVelNow.data[1],rightVelNow.data[0]);
		
		//UnionData32 temp={.d=-5000};
		
		
		//printf("%d\r\n",temp.d);
		
		//printf("%xd,%xd,%xd\r\n",temp.data[2],temp.data[1],temp.data[0]);
		//printf("%d\r\n",(temp.data[3]<<24|temp.data[2]<<16|temp.data[1]<<8|temp.data[0]));
		
		pitch = (float)(pitchNow.d * 0.0054931640625f);
		roll = (float)(rollNow.d * 0.0054931640625f);
		yaw = (float)(yawNow.d * 0.0054931640625f);
		
		OLED_ShowString(0, 0, (uint8_t* )"Picking Robot");
		OLED_Show(0,10, (uint8_t* )"L:", 2, leftVel);
		OLED_ShowNumber(84,10,leftSpeedSet);
		OLED_Show(0,20, (uint8_t* )"R:",2, rightVel);
		OLED_ShowNumber(84,20,rightSpeedSet);	
		OLED_Show(0,30, (uint8_t* )"Pitch:", 7, pitch);
		OLED_Show(0,40, (uint8_t* )"Roll:", 7, roll);
		OLED_Show(0,50, (uint8_t* )"Yaw:", 7, yaw);
		OLED_Refresh_Gram();
		
		  Led_Flash(200);
		  
		  //HAL_Delay(50);
				
		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  /* USB OTG FS requires 48 MHz: 336 MHz / 7 = 48 MHz. */
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
