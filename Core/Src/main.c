/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "myuart.h"
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for accelTask */
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
  .name = "accelTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for consoleTask */
osThreadId_t consoleTaskHandle;
const osThreadAttr_t consoleTask_attributes = {
  .name = "consoleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Timer01 */
osTimerId_t Timer01Handle;
const osTimerAttr_t Timer01_attributes = {
  .name = "Timer01"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void *argument);
void StartDisplayTask(void *argument);
void StartAccelTask(void *argument);
void StartConsoleTask(void *argument);
void Callback01(void *argument);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);
  accelTaskHandle = osThreadNew(StartAccelTask, NULL, &accelTask_attributes);
  consoleTaskHandle = osThreadNew(StartConsoleTask, NULL, &consoleTask_attributes);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/**
* @}
*/
/**
* @}
*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_G_Pin|LED_R_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PWM_DEMUX_0_Pin|PWM_DEMUX_1_Pin|GPIO_PB2_Pin|GPIO_PB10_Pin
                          |GPIO_PB12_Pin|GPIO_PB13_Pin|GPIO_PB14_Pin|GPIO_PB9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PA11_Pin|GPIO_PA12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PWM_DEMUX_0_Pin PWM_DEMUX_1_Pin GPIO_PB2_Pin GPIO_PB10_Pin
                           GPIO_PB12_Pin GPIO_PB13_Pin GPIO_PB14_Pin GPIO_PB9_Pin */
  GPIO_InitStruct.Pin = PWM_DEMUX_0_Pin|PWM_DEMUX_1_Pin|GPIO_PB2_Pin|GPIO_PB10_Pin
                          |GPIO_PB12_Pin|GPIO_PB13_Pin|GPIO_PB14_Pin|GPIO_PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_PA11_Pin GPIO_PA12_Pin */
  GPIO_InitStruct.Pin = GPIO_PA11_Pin|GPIO_PA12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_PB5_Pin */
  GPIO_InitStruct.Pin = GPIO_PB5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PB5_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// String Operations
char lower(char input){
	if(input >= 'A' && input <= 'Z'){
		input = input + 32;
	}
	return input;
}
// Magnetic Transducer
// Make sure to figure out best way to deal with volume/duty-cycle
void spk_tone(uint32_t spk_freq){
	// Assuming 65535 ARR - set at initialization instead
	// Transducer range is ~240Hz (65535) to ~14.2kHz (1100)
	if(spk_freq < 240){
		spk_freq = 240;
	}
	if(spk_freq > 14200){
		spk_freq = 14200;
	}
	// Multiplied by experimentally derived tuning constant 0.9253
	TIM1->ARR = 1100+((65535-1100)/(14200-240))*(14200-spk_freq)*0.9253;
}
void spk_volume(uint32_t spk_duty){
	if(spk_duty > 100){
		spk_duty = 100;
	}
	if(spk_duty < 0){
		spk_duty = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, spk_duty*16);
}
void spk_play(char spk_note, uint8_t octave){
	spk_note = lower(spk_note);
	switch(spk_note){
	case 'c':
		spk_tone(261.6*octave);
		break;
	case 'd':
		spk_tone(293.7*octave);
		break;
	case 'e':
		spk_tone(329.6*octave);
		break;
	case 'f':
		spk_tone(349.2*octave);
		break;
	case 'g':
		spk_tone(392.0*octave);
		break;
	case 'a':
		spk_tone(440.0*octave);
		break;
	case 'b':
		spk_tone(493.9*octave);
		break;
	}
}
void spk_song(char spk_notes[], uint8_t octave, uint32_t delay){
	spk_volume(50);
	int i = 0;
	for(i = 0; i<sizeof(spk_notes); i++){
		spk_play(spk_notes[i], octave);
		osDelay(delay);
	}
	spk_volume(0);
}
void spk_setup(uint32_t spk_freq, uint32_t spk_duty){
	spk_volume(spk_duty);
	spk_tone(spk_freq);
}
// IRQs
static volatile uint32_t adc_semaphore = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PB15_Pin){
	//char message[] = "Hello!\r\n";
	//HAL_UART_Transmit(&huart1, message, sizeof(message), 500);
	adc_semaphore = 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	osDelay(500);
	spk_song("ABCDEFGHIJ", 5, 100);
	//spk_song("EFGGAFEEFGDDDAEEFGEGAFED", 3, 3000);
	char adc_string[10] = "NULL";
	uint32_t adc_val;
	for(;;){
		if(adc_semaphore==1){
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
				adc_val = HAL_ADC_GetValue(&hadc1);
				itoa(adc_val, adc_string, 10);
				HAL_UART_Transmit(&huart1, (unsigned char *) adc_string, strlen(adc_string), 500);
				HAL_UART_Transmit(&huart1, (unsigned char *) "\r\n", 2, 500);
				adc_semaphore = 0;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (adc_val/50)*16);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			}
		}
		//uart_sendchar(0x31);
		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	/* Infinite loop */
	for(;;){
		//osDelay(100);
		//HAL_GPIO_TogglePin(GPIOC, LED_R_Pin);
		osDelay(200);
		HAL_GPIO_TogglePin(GPIOC, LED_G_Pin);
		//osDelay(100);
		//HAL_GPIO_TogglePin(GPIOC, LED_B_Pin);
		//osDelay(100);
	}
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartAccelTask */
/**
* @brief Function implementing the AccelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccelTask */
void StartAccelTask(void *argument)
{
  /* USER CODE BEGIN StartAccelTask */
	uint32_t tempValues[100] = {0};
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
		//HAL_ADC_Start(&hadc1);
		osDelay(1);
		//HAL_ADC_PollForConversion(&hadc1, 1);
		osDelay(1);
		//tempValues[0] = HAL_ADC_GetValue(&hadc1);
		osDelay(1);
		//HAL_ADC_Stop(&hadc1);
	}
  /* USER CODE END StartAccelTask */
}

/* USER CODE BEGIN Header_StartConsoleTask */
/**
* @brief Function implementing the consoleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConsoleTask */
void StartConsoleTask(void *argument)
{
  /* USER CODE BEGIN StartConsoleTask */
	// can use the msp ascii shifting checker to lowercase stuff
	uint32_t buffer_size = 100;
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t m1_duty = 0;
	uint32_t m2_duty = 0;
	uint32_t m3_duty = 0;
	uint32_t m4_duty = 0;
	uint32_t spk_duty = 0;
	unsigned char uart_buffer[buffer_size];
	unsigned char uart_char;
	unsigned char header[] = "---------------------\r\nRobotics Platform CLI\r\n---------------------\n";
	unsigned char prompt[] = "> ";
	unsigned char warning[] = "Invalid command.";
	unsigned char *command[10] = {0};
	unsigned char *direction = 0;
	unsigned char *channel = 0;
	void clear_buffers(){
		uart_char = 0x00;
		for(uint8_t j=0; j<=sizeof(uart_buffer); j++){
			uart_buffer[j] = 0x00;
		}
	}
	void print(unsigned char *input){
		HAL_UART_Transmit(&huart1, (unsigned char *) input, strlen(input), 500);
	}
	void println(unsigned char *input){
		print("\n\r");
		print(input);
	}
	clear_buffers();
	println(header);
	println(prompt);
	/* Infinite loop */
	/*
	//__HAL_RCC_GPIOA_CLK_ENABLE();
	//__HAL_RCC_USART1_CLK_ENABLE();
	//MX_USART1_UART_Init();
	//__HAL_RCC_USART1_CLK_ENABLE();
	//__HAL_RCC_USART1_FORCE_RESET();
	//__HAL_RCC_USART1_RELEASE_RESET();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN);
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	GPIOA->MODER   &= ~((3U << 18) | (3U << 20));	// Reset PA9 & PA10 in MODER
	GPIOA->MODER   |= (2U << 18) | (2U << 20); 		// Enable PA9 & PA10 Alternate Function
	GPIOA->OTYPER  |= (0 << 9)  | (0 << 10); 		//
	GPIOA->OSPEEDR |= (3 << 9)  | (3 << 10);
	GPIOA->PUPDR   |= (0 << 18) | (0 << 20);
	GPIOA->AFR[1]  &= ~((15U << 4) | (15U << 8));	// Reset PA9 & PA10 in AFRH
	GPIOA->AFR[1]  |= (7U << 4) | (7U << 8);		// Set PA9 & PA10 to UART1 in AFRH
	//USART1->CR1 = 0xFFFF;
	USART1->CR1 &= ~((0x03<<26)
					|USART_CR1_OVER8
					|USART_CR1_PCE
					|(0x1F<<3)
					|USART_CR1_TE
					|USART_CR1_RE
					|USART_CR1_UE);
	//GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //GPIO_InitStruct.Pull = GPIO_NOPULL;
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    //HAL_NVIC_EnableIRQ(USART1_IRQn);
	USART1->CR1 = 0x00;	  			// Clear CR1
	USART1->CR1 = (1<<13); 			// UE: USART enable
	USART1->CR1 |= (1<<7);  			// TXEIE: TXE interrupt enable
	USART1->CR1 |= (1<<6);  			// TCIE: Transmission complete interrupt enable
	USART1->CR2 &= ~(1<<12|1<<13); 	// STOP: STOP bits
	USART1->CR3 &= ~(1<<3); 			// HDSEL: Half-duplex selection
	USART1->CR1 |= (1<<3);				// TE: Transmitter enable
	USART1->CR1 |= (1<<2);				// RE: Receiver enable
	//usart_div = (int)(clock_speed/(16*baud));
	//brr = ((uint8_t) usart_div<<0)|((uint8_t) usart_div&0x07FFFFF<<4);
	USART1->BRR = (int)(16000000/(9600));
	*/
	//uart_init_stm32(0x40011000, 9600, 1, 0, 16000000);
 	while(1){
		osDelay(10);
		HAL_UART_Receive(&huart1, &uart_char, 1, 500);
		//uart_sendchar(0x31);
		//uart_sendchar(0x32);
		/*
		USART1->DR = 'H';
		while(!(USART1->SR & (1<<6))){
			osDelay(1);
		}
		*/
		if(uart_char >= ' ' && uart_char <= '~' && i < sizeof(uart_buffer)){
			uart_buffer[i] = uart_char;
			print(&uart_buffer[i]);
			uart_char = 0x00;
			i++;
		}
		else if(uart_char == '\r'){
			command[0] = strtok(uart_buffer, " ");
			j = 1;
			while(command[j-1] != NULL && j <= 10){
				command[j] = strtok(NULL, " ");
				j++;
			}
			switch(lower(uart_buffer[0])){
				case 's':
					// ARR - change counter and autoreload on fly to change freq, duty cycle = VOLUME!
					// Create PWM lib, create speaker music lib
					if(atoi(command[2]) != NULL){
						spk_setup(atoi(command[1]), atoi(command[2]));
					}
						println("SPEAKER ON");
						spk_tone(atoi(command[1]));
					break;
				case 'h':
					println("-h: Display this help menu.\
							 -p [pin #] [duty %] [period]: Toggle PWM for a pin.\
							 -? []: ???");
					break;
				case 'p':
					// Have prints return their result
					m1_duty = atoi(command[1]);
					println(command[1]);
					channel = atoi(command[2]);
					println(command[2]);
					if(m1_duty > 100){
						m1_duty = 100;
					}
					if(channel == 1){
						println("PWM ON: Channel 1");
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_duty*16);
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
					}
					else{
						println("PWM ON: Channel 2");
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m1_duty*16);
					}
					break;
				default:
					println(warning);
			}
			println(prompt);
			clear_buffers();
			i = 0;
		}
		else if(i > 0 && (uart_char == '\b' || uart_char == 127)){
			print("\b \b");
			uart_char = 0x00;
			uart_buffer[i] = 0x00;
			i--;
		}
	}
  /* USER CODE END StartConsoleTask */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
