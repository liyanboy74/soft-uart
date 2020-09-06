/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define 							Number_Of_SoftUart		6

#define								SoftUartTxBufferSize	32
#define								SoftUartRxBufferSize	32

typedef enum {
	SoftUart_OK,
	SoftUart_Error
}SoftUartState_E;

typedef struct{
	uint8_t					TxBuffer[SoftUartTxBufferSize];
	uint8_t					RxBuffer[SoftUartRxBufferSize];
}SoftUartBuffer_S;

typedef struct {
	__IO uint8_t 			TxNComplated;
	//__IO uint8_t 			RxNComplated;
	
	uint8_t						TxEnable;
	uint8_t						RxEnable;
	
	uint8_t 					TxBitShift,TxBitConter;
	uint8_t 					RxBitShift,RxBitConter;
	
	uint8_t						TxIndex,TxSize;
	uint8_t						RxIndex;//,RxSize;
	
	SoftUartBuffer_S	Buffer;
	
	GPIO_TypeDef  		*TxPort;
	uint16_t 					TxPin;
	
	GPIO_TypeDef  		*RxPort;
	uint16_t 					RxPin;
	
} SoftUart_S;
//

void SoftUartInit(SoftUart_S *SU,SoftUartBuffer_S Buffer,GPIO_TypeDef *TxPort,uint16_t TxPin,GPIO_TypeDef *RxPort,uint16_t RxPin)
{
	SU->TxNComplated=0;
	
	SU->RxBitConter=0;
	SU->RxBitShift=0;
	SU->RxIndex=0;

	SU->TxEnable=0;
	SU->RxEnable=0;
	
	SU->TxBitConter=0;
	SU->TxBitShift=0;
	SU->TxIndex=0;
	
	SU->TxSize=0;
	
	SU->Buffer=Buffer;
	
	SU->RxPort=RxPort;
	SU->RxPin=RxPin;
	
	SU->TxPort=TxPort;
	SU->TxPin=TxPin;
}
//

uint8_t 							ScanPortBuffer;

SoftUart_S 						SUart[Number_Of_SoftUart];
SoftUartBuffer_S 			SUBuffer[Number_Of_SoftUart];

void SoftUartTransmitBit(SoftUart_S *SU,uint8_t Bit0_1)
{
	HAL_GPIO_WritePin(SU->TxPort,SU->TxPin,(GPIO_PinState)Bit0_1);
}
//

void SoftUartEnableRx(SoftUart_S *SU)
{
	SU->RxEnable=1;
}
//

void SoftUartDisableRx(SoftUart_S *SU)
{
	SU->RxEnable=0;
}
//

void SoftUartTxProcess(SoftUart_S *SU)
{
	if(SU->TxEnable)
	{
		if(SU->TxBitConter==0)
		{
			SU->TxNComplated=1;
			SU->TxBitShift=0;
			SoftUartTransmitBit(SU,0);
			SU->TxBitConter++;
		}
		else if(SU->TxBitConter<9)
		{
			SoftUartTransmitBit(SU,((SU->Buffer.TxBuffer[SU->TxIndex])>>(SU->TxBitShift))&0x01);
			SU->TxBitConter++;
			SU->TxBitShift++;
		}
		else if(SU->TxBitConter==9)
		{
			SoftUartTransmitBit(SU,1);
			SU->TxBitConter++;
		}
		else if(SU->TxBitConter==10)
		{
			//Complate
			SU->TxBitConter=0;
			
			SU->TxIndex++;
			if(SU->TxSize > SU->TxIndex)
			{
				SU->TxNComplated=1;
				SU->TxEnable=1;
			}
			else
			{
				SU->TxNComplated=0;
				SU->TxEnable=0;
			}
		}
	}
}
//

SoftUartState_E SoftUart_Puts(SoftUart_S *SU,uint8_t *Str,uint8_t Len)
{
	int i;
	
	if(SU->TxNComplated) return SoftUart_Error;
	
	SU->TxIndex=0;
	SU->TxSize=Len;
	
	for(i=0;i<Len;i++)
	{
		SU->Buffer.TxBuffer[i]= Str[i];
	}
	
	SU->TxNComplated=1;
	SU->TxEnable=1;
	
	//while(SU->TxNComplated);
	
	return SoftUart_OK;
}
//

uint8_t SoftUartScanRxPorts(void)
{
	int i;
	uint8_t Buffer=0x00;
	for(i=0;i<Number_Of_SoftUart;i++) 
	{
		Buffer|=((HAL_GPIO_ReadPin(SUart[i].RxPort,SUart[i].RxPin)&0x01)<<i);
	}
	return Buffer;
}
//

uint8_t SoftUartRxGetBit(uint8_t InputChannel)
{
	return ((ScanPortBuffer>>InputChannel)&0x01);
}
//

void SoftUartRxDataBitProcess(SoftUart_S *SU,uint8_t B0_1)
{
	if(SU->RxEnable)
	{
		if(SU->RxBitConter==0)//Start
		{
			if(B0_1)return;
			SU->RxBitShift=0;
			SU->RxBitConter++;
			SU->Buffer.RxBuffer[SU->RxIndex]=0;
		}
		else if(SU->RxBitConter<9)//Data
		{
			SU->Buffer.RxBuffer[SU->RxIndex]|=((B0_1&0x01)<<SU->RxBitShift);
			SU->RxBitConter++;
			SU->RxBitShift++;
		}
		else if(SU->RxBitConter==9)
		{
			SU->RxBitConter=0;
			if(B0_1)//Stop Bit
			{
				//OK
				if((SU->RxIndex)<SoftUartRxBufferSize)(SU->RxIndex)++;
			}
		}
	}
}
//

void SoftUartProcessRxBuffer(void)
{
	int i;
	for(i=0;i<Number_Of_SoftUart;i++) SoftUartRxDataBitProcess(&SUart[i],SoftUartRxGetBit(i));
}
//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		int i;
		
		//RX
		ScanPortBuffer=SoftUartScanRxPorts();//Sampeling
		SoftUartProcessRxBuffer();
		
		//TX
		for(i=0;i<Number_Of_SoftUart;i++)//Transfer Data
		{
			SoftUartTxProcess(&SUart[i]);
		}
	}

}
//
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
		
	SoftUartInit(&SUart[0],SUBuffer[0],GPIOB,GPIO_PIN_3,GPIOA,GPIO_PIN_4);
	SoftUartInit(&SUart[1],SUBuffer[1],GPIOB,GPIO_PIN_4,GPIOA,GPIO_PIN_5);
	SoftUartInit(&SUart[2],SUBuffer[2],GPIOB,GPIO_PIN_5,GPIOA,GPIO_PIN_6);
	SoftUartInit(&SUart[3],SUBuffer[3],GPIOB,GPIO_PIN_6,GPIOA,GPIO_PIN_7);
	SoftUartInit(&SUart[4],SUBuffer[4],GPIOB,GPIO_PIN_7,GPIOB,GPIO_PIN_0);
	SoftUartInit(&SUart[5],SUBuffer[5],GPIOB,GPIO_PIN_8,GPIOB,GPIO_PIN_1);
	
	SoftUartEnableRx(&SUart[0]);
	SoftUartEnableRx(&SUart[1]);
	SoftUartEnableRx(&SUart[2]);
	SoftUartEnableRx(&SUart[3]);
	SoftUartEnableRx(&SUart[4]);
	SoftUartEnableRx(&SUart[5]);
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_Delay(10);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 103;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
