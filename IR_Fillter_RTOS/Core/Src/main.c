/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "handler_adc.h"
#include "handler_delay.h"
#include "handler_button.h"
#include "handler_flash.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId Task_ButtonHandle;
osThreadId Task_Test_IRHandle;
osThreadId Task_ADCHandle;
osMessageQId button_mesHandle;
osMutexId led_lockHandle;
osMutexId myMutex02Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask_Button(void const * argument);
void StartTask_Test_IR(void const * argument);
void StartTask_ADC(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t timer1_counter = 0;
volatile uint16_t adc_val[ADC_CH_NUM] = {0};
uint16_t adc_tag[ADC_CH_NUM] = {0};

uint16_t adc_read_ok;
uint16_t adc_read_ng;
adc_caculate_t adc_caculate = {0};


Button_name_t Button_ok;
Button_name_t Button_ng;

static uint16_t sample_count = 0;
REAL_VALUE product_value;

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Button_Init(&Button_ok, I2_GPIO_Port, I2_Pin);
  Button_Init(&Button_ng, I1_GPIO_Port, I1_Pin);
  HAL_TIM_Base_Start_IT(&htim1);
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 4);
  for(uint8_t i = 0 ; i < 50; i++)
  {
	  HAL_GPIO_TogglePin(O1_GPIO_Port, O1_Pin);
	  Delay_ms(30);
	  HAL_GPIO_TogglePin(O2_GPIO_Port, O2_Pin);
	  Delay_ms(30);
  }


  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of led_lock */
  osMutexDef(led_lock);
  led_lockHandle = osMutexCreate(osMutex(led_lock));

  /* definition and creation of myMutex02 */
  osMutexDef(myMutex02);
  myMutex02Handle = osMutexCreate(osMutex(myMutex02));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of button_mes */
  osMessageQDef(button_mes, 16, uint16_t);
  button_mesHandle = osMessageCreate(osMessageQ(button_mes), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_Button */
  osThreadDef(Task_Button, StartTask_Button, osPriorityLow, 0, 384);
  Task_ButtonHandle = osThreadCreate(osThread(Task_Button), NULL);

  /* definition and creation of Task_Test_IR */
  osThreadDef(Task_Test_IR, StartTask_Test_IR, osPriorityLow, 0, 512);
  Task_Test_IRHandle = osThreadCreate(osThread(Task_Test_IR), NULL);

  /* definition and creation of Task_ADC */
  osThreadDef(Task_ADC, StartTask_ADC, osPriorityLow, 0, 256);
  Task_ADCHandle = osThreadCreate(osThread(Task_ADC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, O7_Pin|O6_Pin|O5_Pin|O1_Pin
                          |O2_Pin|O3_Pin|O4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : O7_Pin O6_Pin O5_Pin O1_Pin
                           O2_Pin O3_Pin O4_Pin */
  GPIO_InitStruct.Pin = O7_Pin|O6_Pin|O5_Pin|O1_Pin
                          |O2_Pin|O3_Pin|O4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I1_Pin */
  GPIO_InitStruct.Pin = I1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2_Pin I3_Pin I4_Pin Button_2_Pin */
  GPIO_InitStruct.Pin = I2_Pin|I3_Pin|I4_Pin|Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
  }
  if(htim->Instance == TIM1)
  {
	  timer1_counter++;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == hadc1.Instance)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(Task_Test_IRHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_Button */
/**
* @brief Function implementing the Task_Button thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Button */
void StartTask_Button(void const * argument)
{
  /* USER CODE BEGIN StartTask_Button */
	uint8_t msg;
  /* Infinite loop */
  for(;;)
  {
#if 1
	  ButtonState_t state;
	  state = BUTTON_Read(&Button_ok);
	  switch(state)
	  {
	  	  case SINGLE_CLICK:
			  break;
	  	  case DOUBLE_CLICK:
				break;
	  	  case LONGCLICK_3S:
	  		  if(product_value == REAL_PR_OK || product_value == FIRT_SAMPLE)
	  		  {
	  			msg = BTN_OK;
				xQueueSend(button_mesHandle, &msg, portMAX_DELAY);
				osMutexWait(led_lockHandle, portMAX_DELAY);
				for(int i = 0; i < 20; i++)
				 {
					GPIOB->ODR ^=  (1 << 12U);
					Delay_ms(70);
				 }
				osMutexRelease(led_lockHandle);
	  		  }
	  		  else
	  		  {
	  			osMutexWait(myMutex02Handle, portMAX_DELAY);
	  			for(int i = 0; i < 20; i++)
				 {
	  				GPIOA->ODR ^=  (1 << 8U);
					Delay_ms(70);
				 }
	  			osMutexRelease(myMutex02Handle);
	  		  }
	  		break;
	  	  default:
	  		  break;
	  }
	  state = BUTTON_Read(&Button_ng);
	  switch(state)
	  {
		  case SINGLE_CLICK:
			  break;
		  case DOUBLE_CLICK:
				break;
		  case LONGCLICK_3S:
				  if(product_value == REAL_PR_NG ||  product_value == FIRT_SAMPLE)
				  {
					  msg = BTN_NG;
					  xQueueSend(button_mesHandle, &msg, portMAX_DELAY);
					  osMutexWait(led_lockHandle, portMAX_DELAY);
					 for(int i = 0; i < 20; i++)
					 {
						 GPIOB->ODR ^=  (1 << 13U);
						Delay_ms(70);
					 }
					 osMutexRelease(led_lockHandle);
				  }
				  else
				  {
					osMutexWait(myMutex02Handle, portMAX_DELAY);
					for(int i = 0; i < 20; i++)
					 {
						GPIOA->ODR ^=  (1 << 8U);
						Delay_ms(70);
					 }
					osMutexRelease(myMutex02Handle);

				  }
			break;
		  default:
			  break;
	  }
#endif
  }
  /* USER CODE END StartTask_Button */
}

/* USER CODE BEGIN Header_StartTask_Test_IR */
/**
* @brief Function implementing the Task_Test_IR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Test_IR */
void StartTask_Test_IR(void const * argument)
{
  /* USER CODE BEGIN StartTask_Test_IR */
	uint8_t msg;
  /* Infinite loop */
  for(;;)
  {
#if 1
	  if (xQueueReceive(button_mesHandle, &msg, portMAX_DELAY) == pdPASS)
	  {
	//	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		  sample_count = 0;
		  adc_caculate = (adc_caculate_t){0};
		  while(sample_count < ADC_SAMPLE_COUNT)
		  {
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, ADC_CH_NUM);
			  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			  HAL_ADC_Stop_DMA(&hadc1);
			  adc_caculate.adc_in0[sample_count] = adc_val[0];
			  adc_caculate.adc_in1[sample_count] = adc_val[1];
			  adc_caculate.adc_in2[sample_count] = adc_val[2];
			  adc_caculate.adc_in3[sample_count] = adc_val[3];
			  sample_count++;
		  }
		  quickSortArray_uint16(adc_caculate.adc_in0, ADC_SAMPLE_COUNT);
		  quickSortArray_uint16(adc_caculate.adc_in1, ADC_SAMPLE_COUNT);
		  quickSortArray_uint16(adc_caculate.adc_in2, ADC_SAMPLE_COUNT);
		  quickSortArray_uint16(adc_caculate.adc_in3, ADC_SAMPLE_COUNT);

		  adc_tag[0] = Average_Caculate((uint16_t*)adc_caculate.adc_in0, ADC_SAMPLE_COUNT,SKIP_VALUE);
		  adc_tag[1] = Average_Caculate((uint16_t*)adc_caculate.adc_in1, ADC_SAMPLE_COUNT,SKIP_VALUE);
		  adc_tag[2] = Average_Caculate((uint16_t*)adc_caculate.adc_in2, ADC_SAMPLE_COUNT,SKIP_VALUE);
		  adc_tag[3] = Average_Caculate((uint16_t*)adc_caculate.adc_in3, ADC_SAMPLE_COUNT,SKIP_VALUE);

		  switch(msg)
		  {
		  	  case BTN_OK:
		  		  Flash_Write_Array_U16(FLASH_ADDR_OK, adc_tag, 4);
				  break;
		  	  case BTN_NG:
		  		Flash_Write_Array_U16(FLASH_ADDR_NG, adc_tag, 4);
		  		  break;
		  	  default:
		  		  break;
		  }
	  }

#endif
  }
  /* USER CODE END StartTask_Test_IR */
}

/* USER CODE BEGIN Header_StartTask_ADC */
/**
* @brief Function implementing the Task_ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ADC */
void StartTask_ADC(void const * argument)
{
  /* USER CODE BEGIN StartTask_ADC */
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, ADC_CH_NUM);
#if 1
	  if(adc_val[0] > DETECT_VALUE && adc_val[3] > DETECT_VALUE)
	  {
		  adc_read_ok = Flash_Read_U16(FLASH_ADDR_OK + 0x02);
		  adc_read_ng = Flash_Read_U16(FLASH_ADDR_NG + 0x02);
		  if((adc_read_ok != FLASH_EMPTY ) && (adc_read_ng != FLASH_EMPTY))
		  {
			  if(adc_val[1] > ((adc_read_ok+adc_read_ng)/2)) // NG
			  {
				  osMutexWait(led_lockHandle, portMAX_DELAY);
				  GPIOB->ODR &=  ~(1 << 12U);
				  GPIOB->ODR |=  (1 << 13U);
				  osMutexWait(myMutex02Handle, portMAX_DELAY);
				  GPIOA->ODR |=  (1 << 8U);
				  product_value = REAL_PR_NG;
				  osMutexRelease(led_lockHandle);
				  osMutexRelease(myMutex02Handle);
			  }
			  else // OK
			  {
				  osMutexWait(led_lockHandle, portMAX_DELAY);
				  GPIOB->ODR |=  (1 << 12U);
				  GPIOB->ODR &=  ~(1 << 13U);
				  product_value = REAL_PR_OK;
				  osMutexRelease(led_lockHandle);
			  }
		  }
		  else
		  {
			  product_value = FIRT_SAMPLE;
		  }

	  }
	  else
	  {
		  osMutexWait(led_lockHandle, portMAX_DELAY);
		  GPIOB->ODR &=  ~(1 << 12U);
		  GPIOB->ODR &=  ~(1 << 13U);
		  osMutexWait(myMutex02Handle, portMAX_DELAY);
		  GPIOA->ODR &=  ~(1 << 8U);
		  product_value = NO_PRODUCT;
		  osMutexRelease(led_lockHandle);
		  osMutexRelease(myMutex02Handle);
	  }

#endif
  }
  /* USER CODE END StartTask_ADC */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
