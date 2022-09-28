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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

//ReturnCode									func_res;
//HAL_StatusTypeDef						HAL_func_res;

JetsonComPortDataTypeDef		jetson;
NextionComPortDataTypeDef		nextion;
ComPortDataTypeDef					com1, com2, com3, com4, com5;

E2pDataTypeDef							e2p;
AdcDataTypeDef							adc1;
StatisticsTypeDef						stats;
CalibrationsTypeDef					calib;
TemperatureDataTypeDef			ds18b20;
WateringControlTypeDef			water_ctrl;
LastPumpCycleTypeDef				last_pump_cycle;


// Флаг включения св-диода индикации секундной метки
volatile uint8_t						time_led_is_on;

// Флаг обнаружения восстановления напряжения питания (>4.8В), adc watchdog
volatile uint8_t						power_up_detected;
// Флаг обнаружения падения напряжения питания (<4.6В), adc watchdog
volatile uint8_t						power_down_detected;

// Переменные слежения за целостностью связи по RS485
volatile uint8_t 						control_link_is_lost;

// Переменная яркости дисплея в привязке к systick в мсек (2000/20=100%), (100/20=5%)
volatile int16_t						display_brightness;
// Таймер задержки перед уменьшением яркости дисплея, сек
volatile int16_t						display_brightness_timer;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_IWDG_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC2_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  //MX_IWDG_Init();
  MX_CRC_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	// Initial hardware settings
	Init_sequence();
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Сброс watchdog******************************************************************************
		IWDG->KR = IWDG_KEY_RELOAD;

		// Если обнаружено падение напряжения питания +5В ниже +4.6В***********************************
		if (power_down_detected)
		{
			// Обработчик состояния падения напряжения питания ниже 4.6В
			Power_down_handler(&hcrc, &hi2c1, &hrtc, &e2p);			
			power_down_detected = 0;    
		
			Error_Handler();			
		}
		
		// Если принят блок данных с АЦП***************************************************************
		if (adc1.DataReady)
		{
			adc1.DataReady = 0;
			
			// Пересчёт значений АЦП в вольты
			Voltage_calc_from_adc_value(&e2p);
			
			// Усреднение значения давления
			Get_average_pressure_value(&e2p);
		}
		
		// Checking whether data is received via COM1**************************************************
		if (com1.RxdPacketIsReceived)
		{
			// Handles data from Com port
			Com_rxd_handler(&hcrc, com1.ComNumber, &jetson, &nextion);			
			
			com1.RxdPacketIsReceived = 0;			
		}
		
		// Checking whether data is received via COM3**************************************************
		if (com3.RxdPacketIsReceived)
		{
			// Handles data from Com port
			Com_rxd_handler(&hcrc, com2.ComNumber, &jetson, &nextion);		
			
			com3.RxdPacketIsReceived = 0;
		}
		
		// Проверка наличия принятой по COM2 строки данных из дисплея Nextion**************************
		if (com2.RxdPacketIsReceived)
		{
			// Обработчик принятого пакета по USART
			Nextion_received_data_handler(&hrtc, &e2p);	

			com2.RxdPacketIsReceived = 0;
		}		

		// Проверка готовности очередной отправки данных по COM2 в дисплей Nextion*********************
		if (com2.TxdPacketIsReadyToSend)
		{		
			com2.TxdPacketIsReadyToSend = 0;
			
			// Отрисовка на Nextion текущих значений
			Prepare_params_and_send_to_nextion(&hrtc, &e2p, &nextion);
		}
		
		// Проверка готовности очередной отправки данных состояния контроллера*************************
		if (com1.TxdPacketIsReadyToSend)
		{		
			com1.TxdPacketIsReadyToSend = 0;
			
			// Мигалка правым LED
			if (Get_time_in_sec(&hrtc) != e2p.Statistics->TimeInSeconds)
			{
				LED2_ON;
				time_led_is_on = 1;
			}
			// Чтение времени
			e2p.Statistics->TimeInSeconds = Get_time_in_sec(&hrtc);
			
			// Коррекция времени и инкремент суток
			Make_time_correction_and_day_inc(&e2p);
						
			// Выполнение автоинкремента/автодекремента каждые 125 мсек, если кнопка на дисплее удерживается
			Parsing_nextion_display_string(&hrtc, &e2p, nextion.RxdBuffer, com2.RxdPacketLenght8, com2.RxdPacketIsReceived);
			
			// Управление насосом
			PumpOn_off(&e2p);
			
			// Управление автополивом, зона 1-8
			Watering_outputs_on_off(&e2p);

			// Сформировать статистику расхода воды
			Make_water_using_statistics(&e2p);
		}
		
		// Опрос термодатчиков только при наступлении очередного момента времени***********************
		if (ds18b20.GetSensorsData)
		{
			// Опрос термодатчиков только при их наличии
			if (ds18b20.DiscoveredQuantity)
			{
				Polling_termosensors(&ds18b20);
				last_pump_cycle.current_water_temp = (int16_t) (ds18b20.TempSensorsValues[0] * 10);
			}
			// При отсутствии д.темп. или ошибке связи
			else 
			{
				last_pump_cycle.current_water_temp = 0;
				
				// Discovered termosensors qwantity on COM5
				ds18b20.DiscoveredQuantity = ds18b20_init(&huart5, &com5);
			}

			ds18b20.GetSensorsData = 0;
			
			// Сформировать статистику по температурам (раз в 1 сек)
			Make_temperature_statistics(&e2p, &last_pump_cycle);
		}
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 4095;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_10;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  AnalogWDGConfig.HighThreshold = ADC_WDG_HIGH_THRESHOLD;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 4999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, EN_TXD3_RXD3_Pin|WATER_ZONE1_Pin|EN_TXD4_RXD4_Pin|LED2_Pin
                          |LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WATER_ZONE2_Pin|WATER_ZONE3_Pin|WATER_ZONE4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WATER_ZONE5_Pin|WATER_ZONE6_Pin|WATER_ZONE7_Pin|WATER_ZONE8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISP_PWR_EN_Pin|PUMP_ON_OFF_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_TXD1_RXD1_GPIO_Port, EN_TXD1_RXD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : WATER_COUNTER_EXTI3_Pin */
  GPIO_InitStruct.Pin = WATER_COUNTER_EXTI3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(WATER_COUNTER_EXTI3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_TXD3_RXD3_Pin WATER_ZONE1_Pin EN_TXD4_RXD4_Pin LED2_Pin
                           LED1_Pin */
  GPIO_InitStruct.Pin = EN_TXD3_RXD3_Pin|WATER_ZONE1_Pin|EN_TXD4_RXD4_Pin|LED2_Pin
                          |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WATER_ZONE2_Pin WATER_ZONE3_Pin WATER_ZONE4_Pin */
  GPIO_InitStruct.Pin = WATER_ZONE2_Pin|WATER_ZONE3_Pin|WATER_ZONE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WATER_ZONE5_Pin WATER_ZONE6_Pin WATER_ZONE7_Pin WATER_ZONE8_Pin */
  GPIO_InitStruct.Pin = WATER_ZONE5_Pin|WATER_ZONE6_Pin|WATER_ZONE7_Pin|WATER_ZONE8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_PWR_EN_Pin PUMP_ON_OFF_Pin */
  GPIO_InitStruct.Pin = DISP_PWR_EN_Pin|PUMP_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_TXD1_RXD1_Pin */
  GPIO_InitStruct.Pin = EN_TXD1_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_TXD1_RXD1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
		// Сброс watchdog
		IWDG->KR=0xAAAA;
  }
}


// Reset indication
void Leds_on_off(void)
{
	LED1_ON;
	HAL_Delay(50);
	LED2_ON;
	
	HAL_Delay(200);

	LED2_OFF;
	HAL_Delay(50);
	LED1_OFF;
	HAL_Delay(50);
}


// USART Tx transmit completed callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		com1.TxdPacketIsSent = 1;
	}

	else if (huart->Instance == USART2)
	{
		com2.TxdPacketIsSent = 1;
	}

	else if (huart->Instance == USART3)
	{
		com3.TxdPacketIsSent = 1;
	}
	
	else if (huart->Instance == UART4)
	{
		com4.TxdPacketIsSent = 1;
	}
	
	else if (huart->Instance == UART5)
	{
		com5.TxdPacketIsSent = 1;
	}
}


// Обработчик принимаемых данных по USART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Com1
	if (huart->Instance == USART1 && com1.RxdPacketIsReceived == 0)
	{
		// Перезапуск таймера контроля непрерывности данных
		com1.RxDataFlowGapTimer = 1;
		
		// Сохранение приходящих символов, если начался приём строки
		if(jetson.ComLink == COM1)
		{
			if (com1.RxdIdx8 < sizeof(jetson.RxdBuffer))
			{
				jetson.RxdBuffer[com1.RxdIdx8++] = com1.ByteReceived;      
			}
				
			// проверка на заполнение буфера приёма
			if (com1.RxdIdx8 >= sizeof(jetson.RxdBuffer))
			{
				// Сохраняем длину принятой строки
				com1.RxdPacketLenght8 = com1.RxdIdx8;
				// Отключение контроля непрерывности данных
				com1.RxDataFlowGapTimer = 0;
				com1.RxdPacketIsReceived = 1;
				com1.RxdIdx8 = 0;
			} 
		}

		else if(nextion.ComLink == COM1)
		{
			if (com1.RxdIdx8 < sizeof(nextion.RxdBuffer))
			{
				nextion.RxdBuffer[com1.RxdIdx8++] = com1.ByteReceived;      
			}
				
			// проверка на заполнение буфера приёма
			if (com1.RxdIdx8 >= sizeof(nextion.RxdBuffer))
			{
				// Сохраняем длину принятой строки
				com1.RxdPacketLenght8 = com1.RxdIdx8;
				// Отключение контроля непрерывности данных
				com1.RxDataFlowGapTimer = 0;
				com1.RxdPacketIsReceived = 1;
				com1.RxdIdx8 = 0;
			} 		
		}
	}

	// Com2
	else if (huart->Instance == USART2)
	{
		// Перезапуск таймера контроля непрерывности данных
		com2.RxDataFlowGapTimer = 1;
		
		// �?ндикация приёма любого байта от дисплея Nextion
		//LED1_ON;

		// Сохранение приходящих символов, если начался приём строки
		if (com2.RxdIdx8 < sizeof(nextion.RxdBuffer))
		{
			nextion.RxdBuffer[com2.RxdIdx8++] = com2.ByteReceived;      
		}

		// проверка на завершение прихода (принимаемой в данный момент) строки по маркерам конца строки
		if (nextion.RxdBuffer[com2.RxdIdx8 - 1] == 0xFF)
		{
			if ((com2.RxdIdx8-1 != 0)&&(nextion.RxdBuffer[com2.RxdIdx8 - 2] == 0xFF))
			{
				if ((com2.RxdIdx8-2 != 0)&&(nextion.RxdBuffer[com2.RxdIdx8 - 3] == 0xFF))
				{
					// Сохраняем длину принятой строки
					com2.RxdPacketLenght8 = com2.RxdIdx8;
					// Отключение контроля непрерывности данных
					com2.RxDataFlowGapTimer = 0;
					com2.RxdPacketIsReceived = 1;
					com2.RxdIdx8 = 0;
				}
			}
		}
			
		// проверка на заполнение буфера приёма
		if (com2.RxdIdx8 >= sizeof(nextion.RxdBuffer))
		{
			// Сохраняем длину принятой строки
			com2.RxdPacketLenght8 = com2.RxdIdx8;
			// Отключение контроля непрерывности данных
			com2.RxDataFlowGapTimer = 0;
			com2.RxdPacketIsReceived = 1;
			com2.RxdIdx8 = 0;
		} 
	
		// Продолжаем, приём след. байта
		//HAL_UART_Receive_IT(&huart2, (uint8_t *) &com2.ByteReceived, 1);
	}
	
	// Com5
	else if (huart->Instance == UART5)
	{
		// Обработка в onewire.с в OW_Reset(), OW_Send() и в OW_SendBits()
		com5.RxdPacketIsReceived = 1;
	}
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	// Com1
	if (huart->Instance == USART1)
	{
		com1.RxdPacketsErrorCounter++;
		HAL_UART_Receive_DMA(&huart1,(uint8_t *) &com1.ByteReceived, 1);
	}
	
	// Com2
	else if (huart->Instance == USART2)
	{
		com2.RxdPacketsErrorCounter++;
		HAL_UART_Receive_DMA(&huart2,(uint8_t *) &com2.ByteReceived, 1);
	}
}


// Starts receiving data on COM ports
void Com_start_receiving_data(ComNum ComNumber)
{
	// Completing Com1 structure and data receive start
	if (ComNumber == COM1)
	{
		com1.RxdIdx8 = 0;
		com1.RxdPacketIsReceived = 0;
		com1.RxDataFlowGapTimer = 0;
		HAL_UART_Receive_DMA(&huart1,(uint8_t *) &com1.ByteReceived, 1);
	}		

	// Completing Com2 structure and data receive start
	else if (ComNumber == COM2)
	{
		com2.RxdIdx8 = 0;
		com2.RxdPacketIsReceived = 0;
		com2.RxDataFlowGapTimer = 0;
		HAL_UART_Receive_DMA(&huart2,(uint8_t *) &com2.ByteReceived, 1);
	}		
}




// Обработчик событий перепада уровней на входах МК
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t time_point_prev = 0;
	
	// Если сработал счётный вход (импульс каждые 10 литров)
	if (GPIO_Pin == WATER_COUNTER_EXTI3_Pin)
	{
		// Если уровень =1 (включено)
		if (WATER_COUNTER_EXTI3_READ_PIN == 1)
		{
			if(HAL_GetTick() - time_point_prev >= 300)
			{				
				// Инкремент счётчика расхода воды, литры*10 (десятки литров)
				e2p.Statistics->WaterCounterValue += 10;
				
				// Инкремент счётчика кол-ва воды, перекачанной насосом в одном цикле, литры*10 (десятки литров)
				e2p.LastPumpCycle->pumped_water_quantity_at_last_cycle++;
				
				// Инкремент общего кол-ва воды, перекачанной насосом, литры*10  (десятки литров)
				e2p.Statistics->TotalPumpedWaterQuantity++;
				
				time_point_prev = HAL_GetTick();
			}
		}
		
		// Сброс флага вторичной сработки
		EXTI->PR |= WATER_COUNTER_EXTI3_Pin;
	}
}


void HAL_SYSTICK_Callback(void)
{
	static uint8_t		time_led_timer = 0;
	static uint8_t		brightness_dim_already_done = 0;
	static uint16_t		control_data_timeout_timer = 0;
	static uint16_t		timer_1000ms=0, dry_work_timer = 0;
	static uint16_t		pump_working_time_at_last_cycle_ms = 0;
	static uint16_t		pumped_water_quantity_at_last_cycle_at_zero;
	
	// Таймер паузы между опросами термодатчиков
	ds18b20.PollingWaitTimer++;
	if (ds18b20.PollingWaitTimer >= TERMO_SENSORS_POLL_PERIOD)
	{
		// Флаг выполнения опроса датчиков
		ds18b20.GetSensorsData = 1;
		ds18b20.PollingWaitTimer = 0;
	}
	
	// Если насос запущен
	if (last_pump_cycle.pump_is_started)
	{
		pump_working_time_at_last_cycle_ms++;

		// Если насос включен, то считаем время работы за одно включение, сек
		if (pump_working_time_at_last_cycle_ms >= 1000)
		{
			e2p.LastPumpCycle->pump_working_time_at_last_cycle++;
			
			// Общее время работы насоса, секунд
			e2p.Statistics->TotalPumpWorkingTime++;
			
			pump_working_time_at_last_cycle_ms = 0;
		}

		// Обновляем точку отсчёта для обнаружения события "сухого хода" при нуле таймера
		if (dry_work_timer == 0) pumped_water_quantity_at_last_cycle_at_zero = e2p.LastPumpCycle->pumped_water_quantity_at_last_cycle;
		dry_work_timer++;
		// Если прошло контрольное время,
		if (dry_work_timer >= DRY_WORK_TIMEOUT_VALUE)
		{
			// а значение "счётчика воды за цикл" не изменилось,
			if (e2p.LastPumpCycle->pumped_water_quantity_at_last_cycle == pumped_water_quantity_at_last_cycle_at_zero)
			{
				// то фиксируем событие "сухого хода"
				e2p.LastPumpCycle->dry_work_detected = 1;
			}
			// Если значение "счётчика воды за цикл" изменилось, то сброс таймера отлова сухого хода
			else dry_work_timer = 0;
			
			dry_work_timer = 0;
		}
	}
	// Если насос не работает
	else
	{
		//pump_working_time_at_last_cycle = 0;
		pump_working_time_at_last_cycle_ms = 0;
	}
	
	// Счёт секунд
	timer_1000ms++;
	if (timer_1000ms >= 1000)
	{
		timer_1000ms = 0;
		// Общее время работы контроллера, секунд
		e2p.Statistics->TotalControllerWorkingTime++;
		// Таймер яркости дисплея
		if (display_brightness_timer > 0)
		{
			display_brightness_timer--;
			// Если досчитали до нуля, то однократная модификация флага
			if (display_brightness_timer == 0) brightness_dim_already_done = 0;
		}
	}

	// Если ещё не уменьшали плавно яркость дисплея
	if (brightness_dim_already_done == 0)
	{
		// При достижении таймаута
		if (display_brightness_timer == 0)
		{
			// Плавное уменьшение яркости дисплея в течение 2сек
			if (display_brightness > (DISPLAY_BRIGHTNESS_MIN_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED)) display_brightness--;
			// Прекращение плавного уменьшения яркости дисплея
			else brightness_dim_already_done = 1;
		}
	}
	
	// Таймер в мсек для отправки данных по COM1
	com1.TxdPacketReadyToSendTimer++;
	if (com1.TxdPacketReadyToSendTimer >= COM1_DATA_PACKET_SEND_TIMEOUT)
	{
		com1.TxdPacketIsReadyToSend = 1;
		com1.TxdPacketReadyToSendTimer = 0;
	}

	// Таймер в мсек для отправки данных по COM2
	com2.TxdPacketReadyToSendTimer++;
	if (com2.TxdPacketReadyToSendTimer >= COM2_DATA_PACKET_SEND_TIMEOUT)
	{
		com2.TxdPacketIsReadyToSend = 1;
		com2.TxdPacketReadyToSendTimer = 0;
	}

	
	// Таймер 500 мсек для определения обрыва связи
	control_data_timeout_timer++;
	if (control_data_timeout_timer >= NO_DATA_TIMEOUT_VALUE)
	{
		control_link_is_lost = 1;
		control_data_timeout_timer = 0;
	}

	// Com1
	if (com1.RxDataFlowGapTimer != 0)
	{
		com1.RxDataFlowGapTimer++;

		// По истечении 4 мс при разрыве в данных считаем пакет завершённым, проверяем
		if (com1.RxDataFlowGapTimer >= DATA_FLOW_GAP_TIME_VALUE + 1)
		{
			// If received not zero length
			if(com1.RxdIdx8)
			{
				com1.RxdPacketIsReceived = 1;
				//com1.RxdPacketIsStarted = 0;
				// Getting received string length
				com1.RxdPacketLenght8 = com1.RxdIdx8;
				com1.RxdIdx8 = 0;
			}
			// Turning off data continuity watching timer
			com1.RxDataFlowGapTimer = 0;
		}
	}
	
	// Com2
	if (com2.RxDataFlowGapTimer != 0)
	{
		com2.RxDataFlowGapTimer++;

		// По истечении 4 мс при разрыве в данных считаем пакет повреждённым, отбраковываем
		if (com2.RxDataFlowGapTimer >= DATA_FLOW_GAP_TIME_VALUE + 1)
		{
			// If received not zero length
			if(com2.RxdIdx8)
			{
				com2.RxdPacketIsReceived = 1;
				//com2.RxdPacketIsStarted = 0;
				// Getting received string length
				com2.RxdPacketLenght8 = com2.RxdIdx8;
				com2.RxdIdx8 = 0;
			}
			// Turning off data continuity watching timer
			com2.RxDataFlowGapTimer = 0;
		}
	}

	// Отключение св-диода индикации работы RTC
	if (time_led_is_on) time_led_timer++;
	if (time_led_timer >= 5)
	{
		LED2_OFF;	
		time_led_is_on = 0;
		time_led_timer = 0;
	}
}


// Копирование из одного буфера во 2-ой
void Copy_buf(uint8_t * source_buf, uint8_t * dest_buf, uint16_t buf_lenght)
{
	uint16_t idx=0;
	
	while (buf_lenght>0)
	{
		dest_buf[idx] = source_buf[idx];
		idx++;
		buf_lenght--;
	}
}


// Завершение измерения АЦП1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	{
		adc1.DataReady = 1;
	}

	else if (hadc->Instance == ADC2)
	{
		//adc2.DataReady = 1;
	}
}


// Пересчёт значений АЦП1 для каждого канала в вольты
void Voltage_calc_from_adc_value(E2pDataTypeDef * e2p)
{
	float	voltage;
	
	// Канал измерения напряжения питания +5В, аналог. вход ADC2_IN10*****************************
	{
		voltage = (float) HAL_ADC_GetValue(&hadc2);
		voltage *= ADC_LSB_VALUE;
		voltage *= FIVE_VOLTS_DIVISION_COEFF;

		// Единицы целых будут отображать единицы милливольт
		voltage /= 1;
		voltage = roundf(voltage);      

		adc1.VoltsBuf[0] = (int32_t) voltage;
	}

	// Канал измерения напряжения на аналог. входе (Д.Д.) AIN2 XP3.3 (+0,5..4,5В)******************
	{
		voltage = (float) (adc1.CountsBuf[Channel11] & 0x0000FFFF);
		voltage *= ADC_LSB_VALUE;
		voltage *= FIVE_VOLTS_DIVISION_COEFF;

		// Единицы целых будут отображать единицы милливольт
		voltage /= 1;
		voltage = roundf(voltage);      

		adc1.VoltsBuf[1] = (int32_t) voltage;
		
		// Давление воды в системе: (U тек. - U min)*( (P max - P min)/(U max - U min) )
		voltage /= 100;
		voltage -= (float)e2p->Calibrations->PsensorMinPressureVoltageValue;
		voltage *= ((float)e2p->Calibrations->PsensorMaxPressureValue - (float)e2p->Calibrations->PsensorMinPressureValue);
		voltage /= ((float)e2p->Calibrations->PsensorMaxPressureVoltageValue - (float)e2p->Calibrations->PsensorMinPressureVoltageValue);
		voltage *= 10;
		if (voltage < 0) voltage = 0;
		voltage = roundf(voltage);      

		last_pump_cycle.water_pressure_value = (int16_t) voltage;
		
		//if (water_pressure_value<0) water_pressure_value=0;
	}
	
	// Канал измерения напряжения на аналог. входе AIN3 XP3.4 (+0,5..4,5В)******************
	{
		voltage = (float) (adc1.CountsBuf[Channel12] & 0x0000FFFF);
		voltage *= ADC_LSB_VALUE;
		voltage *= FIVE_VOLTS_DIVISION_COEFF;

		// Единицы целых будут отображать единицы милливольт
		voltage /= 1;
		voltage = roundf(voltage);      

		adc1.VoltsBuf[3] = (int32_t) voltage;
	}
		
	return;
}


// Initial hardware settings
void Init_sequence(void)
{
	HAL_StatusTypeDef						HAL_func_res;

	// Определение размера структур для сохранения/восстановления из eeprom 
	e2p.StructSize = (uint16_t) sizeof(e2p);
	stats.StructSize = (uint16_t) sizeof(stats);
	calib.StructSize = (uint16_t) sizeof(calib);
	water_ctrl.StructSize = (uint16_t) sizeof(water_ctrl);
	last_pump_cycle.StructSize = (uint16_t) sizeof(last_pump_cycle);
	
	// Filling structure with addresses of sub structures
	e2p.Statistics 				= &stats;
	e2p.WateringControls 	= &water_ctrl;
	e2p.Calibrations			=	&calib;
	e2p.LastPumpCycle			= &last_pump_cycle;
	
	// Инициализация слежения за появлением питания (срабатывает не при восстановлении, а при падении)
	//PVD_Config();
	
	// Запись калибровочного коэффициента для коррекции хода часов реального времени ( сек/месяц)
	HAL_func_res = HAL_RTCEx_SetSmoothCalib(&hrtc, 0, 0, RTC_TIME_CALIBRATION_COEFF);
	
	DISPLAY_POWER_ENABLE;
	
	RXD1_DISABLE;
	RXD4_DISABLE;
	RXD3_DISABLE;
	
	WATER_PUMP_OFF;
	WATER_ZONE1_OFF;
	WATER_ZONE2_OFF;
	WATER_ZONE3_OFF;
	WATER_ZONE4_OFF;
	WATER_ZONE5_OFF;
	WATER_ZONE6_OFF;
	WATER_ZONE7_OFF;
	WATER_ZONE8_OFF;
	
	// ADC autocalibration
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);

	// без этого не работает ацп в связке с таймером 4 по capture/compare
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM4->CR2 |= TIM_CR2_OIS4;

	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
	
	// Сброс watchdog
	IWDG->KR = IWDG_KEY_RELOAD;
		
	// Запуск АЦП1
	adc1.DataReady = 0;
	HAL_func_res = HAL_ADC_Start_DMA(&hadc1, adc1.CountsBuf, sizeof(adc1.CountsBuf) / 4 );
	// Запуск АЦП2
	HAL_func_res = HAL_ADC_Start(&hadc2);

	// Сброс флага обнаружения восстановления питания
	power_up_detected = 0;
	
	ADC2->LTR = 0;
	ADC2->HTR = ADC_WDG_HIGH_THRESHOLD;
	// Clear the ADC analog watchdog flag
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD);
	// Enable ADC analog watchdog interrupt
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD);

	// Ждём восстановления напряжения питания цепи +5В, >=4,9В
	while (power_up_detected == 0)
	{
		// Reset indication
		Leds_on_off();
	}
	
	// Сброс флага обнаружения восстановления питания
	power_up_detected = 0;
	
	ADC2->LTR = ADC_WDG_LOW_THRESHOLD;
	ADC2->HTR = 4095;
	// Clear the ADC analog watchdog flag
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD);
	// Enable ADC analog watchdog interrupt
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD);
	
	// Восстановление рабочих переменных из eeprom
	Restore_all_data(&hcrc, &hi2c1, &hrtc, &e2p);

	// Начальная разметка строки для отправки данных через ком порт в дисплей Nextion
	Init_string_to_nextion();
	

	// Naming working structures of Com ports 
	com1.ComNumber = COM1;
	com2.ComNumber = COM2;
	com3.ComNumber = COM3;
	com4.ComNumber = COM4;
	
	// Link logical Com port to physical
	nextion.ComLink = NEXTION_DISPLAY_COM_PORT;

	// Link logical structure to physical usart1
	if(jetson.ComLink == COM1)
	{
		jetson.Com = &com1;
	}
	else if(nextion.ComLink == COM1)
	{
		nextion.Com = &com1;
	}

	// Link logical structure to physical usart2
	if(jetson.ComLink == COM2)
	{
		jetson.Com = &com2;
	}
	else if(nextion.ComLink == COM2)
	{
		nextion.Com = &com2;
	}
	
	// Prepare for first sending
	com1.TxdPacketIsSent = 1;
	com2.TxdPacketIsSent = 1;
	com3.TxdPacketIsSent = 1;
	com4.TxdPacketIsSent = 1;
	com5.TxdPacketIsSent = 1;
	
	// Com ports data receive starting
	Com_start_receiving_data(COM1);
	Com_start_receiving_data(COM2);
	Com_start_receiving_data(COM3);
	Com_start_receiving_data(COM4);
	//Com_start_receiving_data(COM5);
	
	// Яркость дисплея на max (2000/20=100%)
	display_brightness = DISPLAY_BRIGHTNESS_MAX_VALUE*DISPLAY_BRIGHTNESS_OFF_SPEED;
	// Запуск таймера уменьшения яркости дисплея
	display_brightness_timer = DISPLAY_BRIGHTNESS_OFF_DELAY;
	
	// Setting instance name (number)
	com5.ComNumber = COM5;
	// Discovered termosensors qwantity on COM5
	ds18b20.DiscoveredQuantity = ds18b20_init(&huart5, &com5);
	
	// Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL) are synchronized with RTC APB clock
	HAL_func_res = HAL_RTC_WaitForSynchro(&hrtc);

	HAL_Delay(500);
}


// Разбор принятой строки от дисплея Nextion
void Parsing_nextion_display_string(RTC_HandleTypeDef  * hrtc, E2pDataTypeDef * e2p, uint8_t * buf, uint16_t string_lenght, uint8_t string_status)
{
	static uint32_t	source_type = 0;
	static uint32_t	source_value = 0;
	static uint32_t	key_pressing_time_moment = 0;
	const uint32_t 	key_is_pressed = 0x71010000;
	const uint32_t 	key_is_released = 0x71000000;
	uint8_t					large_step = 0;
	static uint8_t	state_machine = 0;


	// Если статус строки !=0, то обрабатываем как вновь принятую, иначе - повторяем, как предыдущую
	if (string_status != 0)
	{
		// 4 байта - идентификатор события
		source_type  = ((uint32_t) buf[0]) << 24;
		source_type |= ((uint32_t) buf[1]) << 16;
		source_type |= ((uint32_t) buf[2]) << 8;
		source_type |= ((uint32_t) buf[3]) << 0;

		// 4 байта - значение
		source_value  = ((uint32_t) buf[4]) << 24;
		source_value |= ((uint32_t) buf[5]) << 16;
		source_value |= ((uint32_t) buf[6]) << 8;
		source_value |= ((uint32_t) buf[7]) << 0;
		
		// Фиксируем момент нажатия кнопки на дисплее
		key_pressing_time_moment = HAL_GetTick();
		// Кнопка только нажата, ещё не обработана
		state_machine = 0;

			// Яркость дисплея на max (2000/20=100%)
		display_brightness = DISPLAY_BRIGHTNESS_MAX_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED;
		// Запуск таймера уменьшения яркости дисплея
		display_brightness_timer = DISPLAY_BRIGHTNESS_OFF_DELAY;
	}
		
	// Если кнопка на дисплее нажата и обработана 1 раз
	if (state_machine == 1)
	{
		// Секундная пауза перед автоинкрементом/автодекрементом
		if (HAL_GetTick() < key_pressing_time_moment + 1000) return;
		else
		{
			state_machine = 2;
			large_step = 0;
		}
	}

	if (state_machine == 2)
	{
		// 60-ти секундная пауза перед автоинкрементом/автодекрементом числами *1000000
		if (HAL_GetTick() >= key_pressing_time_moment + 60000) large_step = 3;
		// 40-ти секундная пауза перед автоинкрементом/автодекрементом числами *10000
		if (HAL_GetTick() >= key_pressing_time_moment + 40000) large_step = 2;
		// 20-ти секундная пауза перед автоинкрементом/автодекрементом числами *100
		else if (HAL_GetTick() >= key_pressing_time_moment + 20000) large_step = 1;
		else large_step = 0;
	}
		
	// Если кнопка была отжата, то обеспечиваем реакцию на повторное нажатие
	if (source_value == key_is_released)
	{
		state_machine = 0;
		// Обновляем момент нажатия кнопки на дисплее
		key_pressing_time_moment = 0;
		return;
	}

	if (state_machine == 0) state_machine = 1;
	
	// Разбор принятой строки
	switch (source_type)
	{	
		// Если нажата кнопка ручного принудительного включения насоса
		case PumpOn:
		{
			// Включить насос
			e2p->LastPumpCycle->switch_pump_on = 1;
			
			// Если не находимся в режиме автоподкачки, то
			if (e2p->LastPumpCycle->auto_pump_is_started == 0)
			{
				// Обнуление счётчиков значений последнего цикла
				e2p->LastPumpCycle->pump_working_time_at_last_cycle = 0;
				e2p->LastPumpCycle->pumped_water_quantity_at_last_cycle = 0;					
			}
			
			break;
		}

		// Если нажата кнопка ручного принудительного выключения насоса
		case PumpOff:
		{
			if (source_value == key_is_pressed)
			{
				// Выключить насос
				e2p->LastPumpCycle->switch_pump_off = 1;

				// Если кнопка выкл. была нажата при активном событии "сухого хода", то
				if (e2p->LastPumpCycle->dry_work_detected)
				{
					// сброс события "сухого хода"
					e2p->LastPumpCycle->dry_work_detected = 0;
					
					// Возобновление автоподкачки, если была прервана "сухим ходом"
					if (e2p->LastPumpCycle->auto_pump_is_started)
					{
						// Отключение автоподкачки
						//e2p->LastPumpCycle->auto_pump_is_done = 0;
						// Включаем нанос
						e2p->LastPumpCycle->switch_pump_on = 1;
						e2p->LastPumpCycle->switch_pump_off = 0;
					}
				}
				// Если событие "сухой ход" неактивно, то
				else
				{
					// Если кнопка выкл. была нажата при штатной работе автоподкачки, то
					if (e2p->LastPumpCycle->auto_pump_is_started)
					{
						// Отключение автоподкачки
						e2p->LastPumpCycle->auto_pump_is_done = 1;
						// Разрешение повторной попытки автоподкачки воды
						e2p->LastPumpCycle->auto_pump_is_started = 0;						
					}
				}
			}

			break;
		}

		// Уменьшение давления включения насоса, атм * 10 ***************************
		case PumpOnPressureDec:
		{
			e2p->Calibrations->PumpOnPressureValue -= 1;
			if (e2p->Calibrations->PumpOnPressureValue < 0) e2p->Calibrations->PumpOnPressureValue = 0;
			break;
		}

		// Увеличение давления включения насоса, атм * 10
		case PumpOnPressureInc:
		{
			e2p->Calibrations->PumpOnPressureValue += 1;
			if (e2p->Calibrations->PumpOnPressureValue > PRESSURE_MAX_VALUE) e2p->Calibrations->PumpOnPressureValue = PRESSURE_MAX_VALUE;
			break;
		}

		// Уменьшение давления выключения насоса, атм * 10 ***************************
		case PumpOffPressureDec:
		{
			e2p->Calibrations->PumpOffPressureValue -= 1;
			if (e2p->Calibrations->PumpOffPressureValue < 0) e2p->Calibrations->PumpOffPressureValue = 0;
			break;
		}

		// Увеличение давления выключения насоса, атм * 10
		case PumpOffPressureInc:
		{
			e2p->Calibrations->PumpOffPressureValue += 1;
			if (e2p->Calibrations->PumpOffPressureValue > PRESSURE_MAX_VALUE) e2p->Calibrations->PumpOffPressureValue = PRESSURE_MAX_VALUE;
			break;
		}

		// Уменьшение минимального значения давления датчика давления, атм * 10 ******
		case PresSensorPminDec:
		{
			e2p->Calibrations->PsensorMinPressureValue -= 1;
			if (e2p->Calibrations->PsensorMinPressureValue < 0) e2p->Calibrations->PsensorMinPressureValue = 0;
			break;
		}

		// Увеличение минимального значения давления датчика давления, атм * 10
		case PresSensorPminInc:
		{
			e2p->Calibrations->PsensorMinPressureValue += 1;
			if (e2p->Calibrations->PsensorMinPressureValue > PRESSURE_MAX_VALUE) e2p->Calibrations->PsensorMinPressureValue = PRESSURE_MAX_VALUE;
			break;
		}
	
		// Уменьшение максимального значения давления датчика давления, атм * 10 ******
		case PresSensorPmaxDec:
		{
			e2p->Calibrations->PsensorMaxPressureValue -= 1;
			if (e2p->Calibrations->PsensorMaxPressureValue < 0) e2p->Calibrations->PsensorMaxPressureValue = 0;
			break;
		}

		// Увеличение максимального значения давления датчика давления, атм * 10
		case PresSensorPmaxInc:
		{
			e2p->Calibrations->PsensorMaxPressureValue += 1;
			if (e2p->Calibrations->PsensorMaxPressureValue > PRESSURE_MAX_VALUE) e2p->Calibrations->PsensorMaxPressureValue = PRESSURE_MAX_VALUE;
			break;
		}			

		// Уменьшение напряжения (0 - 5В), соотв. минимальному давлению датчика давления, мВ*100 ******
		case VoltageForPminDec:
		{
			e2p->Calibrations->PsensorMinPressureVoltageValue -= 1;
			if (e2p->Calibrations->PsensorMinPressureVoltageValue < 0) e2p->Calibrations->PsensorMinPressureVoltageValue = 0;
			break;
		}

		// Увеличение напряжения (0 - 5В), соотв. минимальному давлению датчика давления, мВ*100
		case VoltageForPminInc:
		{
			e2p->Calibrations->PsensorMinPressureVoltageValue += 1;
			if (e2p->Calibrations->PsensorMinPressureVoltageValue > MAX_VOLTAGE_VALUE_FOR_P_SENSOR)
					e2p->Calibrations->PsensorMinPressureVoltageValue = MAX_VOLTAGE_VALUE_FOR_P_SENSOR;
			break;
		}			

		// Уменьшение напряжения (0 - 5В), соотв. максимальному давлению датчика давления, мВ*100 ******
		case VoltageForPmaxDec:
		{
			e2p->Calibrations->PsensorMaxPressureVoltageValue -= 1;
			if (e2p->Calibrations->PsensorMaxPressureVoltageValue < 0) e2p->Calibrations->PsensorMaxPressureVoltageValue = 0;
			break;
		}

		// Увеличение напряжения (0 - 5В), соотв. максимальному давлению датчика давления, мВ*100
		case VoltageForPmaxInc:
		{
			e2p->Calibrations->PsensorMaxPressureVoltageValue += 1;
			if (e2p->Calibrations->PsensorMaxPressureVoltageValue > MAX_VOLTAGE_VALUE_FOR_P_SENSOR)
					e2p->Calibrations->PsensorMaxPressureVoltageValue = MAX_VOLTAGE_VALUE_FOR_P_SENSOR;
			break;
		}

		// Полив
		//{
		// Уменьшение значения текущего номера выхода полива, 1-8 ******
		case CurrWateringOutputNumberDec:
		{
			e2p->WateringControls->CurrWateringOutputNumber -= 1;
			if (e2p->WateringControls->CurrWateringOutputNumber < 1) e2p->WateringControls->CurrWateringOutputNumber = 8;
			break;
		}

		// Увеличение значения текущего номера выхода полива, 1-8 ******
		case CurrWateringOutputNumberInc:
		{
			e2p->WateringControls->CurrWateringOutputNumber += 1;
			if (e2p->WateringControls->CurrWateringOutputNumber > 8) e2p->WateringControls->CurrWateringOutputNumber = 1;
			break;
		}	
				
		// Уменьшение значения смещения времени включения полива зоны 1-8 относительно начала суток, мин ******
		case OutxZeroClockTimeDeltaDec:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 7) e2p->WateringControls->out7_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 8) e2p->WateringControls->out8_zero_clock_time_delta -= 5;

			if (e2p->WateringControls->out1_zero_clock_time_delta < 0) e2p->WateringControls->out1_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out2_zero_clock_time_delta < 0) e2p->WateringControls->out2_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out3_zero_clock_time_delta < 0) e2p->WateringControls->out3_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out4_zero_clock_time_delta < 0) e2p->WateringControls->out4_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out5_zero_clock_time_delta < 0) e2p->WateringControls->out5_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out6_zero_clock_time_delta < 0) e2p->WateringControls->out6_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out7_zero_clock_time_delta < 0) e2p->WateringControls->out7_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out8_zero_clock_time_delta < 0) e2p->WateringControls->out8_zero_clock_time_delta = 1435;
			break;
		}

		// Увеличение значения смещения времени включения полива зоны 1-8 относительно начала суток, мин
		case OutxZeroClockTimeDeltaInc:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 7) e2p->WateringControls->out7_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 8) e2p->WateringControls->out8_zero_clock_time_delta += 5;

			if (e2p->WateringControls->out1_zero_clock_time_delta > 1435) e2p->WateringControls->out1_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out2_zero_clock_time_delta > 1435) e2p->WateringControls->out2_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out3_zero_clock_time_delta > 1435) e2p->WateringControls->out3_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out4_zero_clock_time_delta > 1435) e2p->WateringControls->out4_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out5_zero_clock_time_delta > 1435) e2p->WateringControls->out5_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out6_zero_clock_time_delta > 1435) e2p->WateringControls->out6_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out7_zero_clock_time_delta > 1435) e2p->WateringControls->out7_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out8_zero_clock_time_delta > 1435) e2p->WateringControls->out8_zero_clock_time_delta = 0;
			break;
		}		

		// Уменьшение значения времени работы полива зоны 1-8, мин ***************************
		case OutxWorkingTimeDec:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1)			e2p->WateringControls->out1_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 7) e2p->WateringControls->out7_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 8) e2p->WateringControls->out8_working_time -= 5;

			if (e2p->WateringControls->out1_working_time < 0) e2p->WateringControls->out1_working_time = 1435;
			if (e2p->WateringControls->out2_working_time < 0) e2p->WateringControls->out2_working_time = 1435;
			if (e2p->WateringControls->out3_working_time < 0) e2p->WateringControls->out3_working_time = 1435;
			if (e2p->WateringControls->out4_working_time < 0) e2p->WateringControls->out4_working_time = 1435;
			if (e2p->WateringControls->out5_working_time < 0) e2p->WateringControls->out5_working_time = 1435;
			if (e2p->WateringControls->out6_working_time < 0) e2p->WateringControls->out6_working_time = 1435;
			if (e2p->WateringControls->out7_working_time < 0) e2p->WateringControls->out7_working_time = 1435;
			if (e2p->WateringControls->out8_working_time < 0) e2p->WateringControls->out8_working_time = 1435;
			break;
		}

		// Увеличение значения времени работы полива зоны 1-8, мин
		case OutxWorkingTimeInc:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 7) e2p->WateringControls->out7_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 8) e2p->WateringControls->out8_working_time += 5;

			if (e2p->WateringControls->out1_working_time > 1435) e2p->WateringControls->out1_working_time = 0;
			if (e2p->WateringControls->out2_working_time > 1435) e2p->WateringControls->out2_working_time = 0;
			if (e2p->WateringControls->out3_working_time > 1435) e2p->WateringControls->out3_working_time = 0;
			if (e2p->WateringControls->out4_working_time > 1435) e2p->WateringControls->out4_working_time = 0;
			if (e2p->WateringControls->out5_working_time > 1435) e2p->WateringControls->out5_working_time = 0;
			if (e2p->WateringControls->out6_working_time > 1435) e2p->WateringControls->out6_working_time = 0;
			if (e2p->WateringControls->out7_working_time > 1435) e2p->WateringControls->out7_working_time = 0;
			if (e2p->WateringControls->out8_working_time > 1435) e2p->WateringControls->out8_working_time = 0;
			break;
		}	

		// Уменьшение значения интервала времени между включениями полива зоны 1-8, мин
		case OutxWorkIntervalTimeDec:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 7) e2p->WateringControls->out7_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 8) e2p->WateringControls->out8_interval_time -= 5;

			if (e2p->WateringControls->out1_interval_time < 0) e2p->WateringControls->out1_interval_time = 1435;
			if (e2p->WateringControls->out2_interval_time < 0) e2p->WateringControls->out2_interval_time = 1435;
			if (e2p->WateringControls->out3_interval_time < 0) e2p->WateringControls->out3_interval_time = 1435;
			if (e2p->WateringControls->out4_interval_time < 0) e2p->WateringControls->out4_interval_time = 1435;
			if (e2p->WateringControls->out5_interval_time < 0) e2p->WateringControls->out5_interval_time = 1435;
			if (e2p->WateringControls->out6_interval_time < 0) e2p->WateringControls->out6_interval_time = 1435;
			if (e2p->WateringControls->out7_interval_time < 0) e2p->WateringControls->out7_interval_time = 1435;
			if (e2p->WateringControls->out8_interval_time < 0) e2p->WateringControls->out8_interval_time = 1435;
			break;
		}

		// Увеличение значения интервала времени между включениями полива зоны 1-8, мин
		case OutxWorkIntervalTimeInc:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 7) e2p->WateringControls->out7_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 8) e2p->WateringControls->out8_interval_time+=5;

			if (e2p->WateringControls->out1_interval_time>1435) e2p->WateringControls->out1_interval_time=0;
			if (e2p->WateringControls->out2_interval_time>1435) e2p->WateringControls->out2_interval_time=0;
			if (e2p->WateringControls->out3_interval_time>1435) e2p->WateringControls->out3_interval_time=0;
			if (e2p->WateringControls->out4_interval_time>1435) e2p->WateringControls->out4_interval_time=0;
			if (e2p->WateringControls->out5_interval_time>1435) e2p->WateringControls->out5_interval_time=0;
			if (e2p->WateringControls->out6_interval_time>1435) e2p->WateringControls->out6_interval_time=0;
			if (e2p->WateringControls->out7_interval_time>1435) e2p->WateringControls->out7_interval_time=0;
			if (e2p->WateringControls->out8_interval_time>1435) e2p->WateringControls->out8_interval_time=0;
			break;
		}

		// Сброс установок для текущей зоны полива
		case CurrOutputSettingsToDefault:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1)
			{
				e2p->WateringControls->out1_interval_time = 0;
				e2p->WateringControls->out1_working_time = 0;
				e2p->WateringControls->out1_zero_clock_time_delta = 0;
			}
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2)
			{
				e2p->WateringControls->out2_interval_time = 0;
				e2p->WateringControls->out2_working_time = 0;
				e2p->WateringControls->out2_zero_clock_time_delta = 0;				
			}

			else if (e2p->WateringControls->CurrWateringOutputNumber == 3)
			{
				e2p->WateringControls->out3_interval_time = 0;
				e2p->WateringControls->out3_working_time = 0;
				e2p->WateringControls->out3_zero_clock_time_delta = 0;				
			}

			else if (e2p->WateringControls->CurrWateringOutputNumber == 4)
			{
				e2p->WateringControls->out4_interval_time = 0;
				e2p->WateringControls->out4_working_time = 0;
				e2p->WateringControls->out4_zero_clock_time_delta = 0;				
			}

			else if (e2p->WateringControls->CurrWateringOutputNumber == 5)
			{
				e2p->WateringControls->out5_interval_time = 0;
				e2p->WateringControls->out5_working_time = 0;
				e2p->WateringControls->out5_zero_clock_time_delta = 0;				
			}

			else if (e2p->WateringControls->CurrWateringOutputNumber == 6)
			{
				e2p->WateringControls->out6_interval_time = 0;
				e2p->WateringControls->out6_working_time = 0;
				e2p->WateringControls->out6_zero_clock_time_delta = 0;				
			}

			else if (e2p->WateringControls->CurrWateringOutputNumber == 7)
			{
				e2p->WateringControls->out7_interval_time = 0;
				e2p->WateringControls->out7_working_time = 0;
				e2p->WateringControls->out7_zero_clock_time_delta = 0;				
			}

			else if (e2p->WateringControls->CurrWateringOutputNumber == 8)
			{
				e2p->WateringControls->out8_interval_time = 0;
				e2p->WateringControls->out8_working_time = 0;
				e2p->WateringControls->out8_zero_clock_time_delta = 0;				
			}
			
			break;
		}		
	//}
		// Уменьшение значения счётчика расхода воды, литры ***********************************
		case WaterCounterValueDec:
		{
			if (large_step == 0)			e2p->Statistics->WaterCounterValue -= 1;
			else if (large_step == 1)	e2p->Statistics->WaterCounterValue -= 100;
			else if (large_step == 2)	e2p->Statistics->WaterCounterValue -= 10000;
			else if (large_step == 3)	e2p->Statistics->WaterCounterValue -= 1000000;

			if (e2p->Statistics->WaterCounterValue < 0) e2p->Statistics->WaterCounterValue = 99999999;
			break;
		}

		// Увеличение значения счётчика расхода воды, литры
		case WaterCounterValueInc:
		{
			if (large_step == 0)			e2p->Statistics->WaterCounterValue += 1;
			else if (large_step == 1)	e2p->Statistics->WaterCounterValue += 100;
			else if (large_step == 2)	e2p->Statistics->WaterCounterValue += 10000;
			else if (large_step == 3)	e2p->Statistics->WaterCounterValue += 1000000;

			if (e2p->Statistics->WaterCounterValue > 99999999) e2p->Statistics->WaterCounterValue = 0;
			break;
		}

		// Уменьшение значения смещения времени автоподкачки относительно начала суток, мин ******
		case AutoPumpZeroClockDeltaDec:
		{
			e2p->LastPumpCycle->auto_pump_zero_clock_time_delta -= 5;

			if (e2p->LastPumpCycle->auto_pump_zero_clock_time_delta < 0) e2p->LastPumpCycle->auto_pump_zero_clock_time_delta = 1435;
			break;
		}

		// Увеличение значения смещения времени автоподкачки относительно начала суток, мин
		case AutoPumpZeroClockDeltaInc:
		{
			e2p->LastPumpCycle->auto_pump_zero_clock_time_delta += 5;
			
			if (e2p->LastPumpCycle->auto_pump_zero_clock_time_delta > 1435) e2p->LastPumpCycle->auto_pump_zero_clock_time_delta = 0;
			break;
		}		

		// Уменьшение объёма автоподкачки, литры*10 *******************************************
		case AutoPumpQuantityDec:
		{
			if (large_step == 0)			e2p->LastPumpCycle->auto_pump_quantity -= 1;
			else if (large_step == 1)	e2p->LastPumpCycle->auto_pump_quantity -= 100;
			else if (large_step == 2)	e2p->LastPumpCycle->auto_pump_quantity -= 100;
			else if (large_step == 3)	e2p->LastPumpCycle->auto_pump_quantity -= 100;

			if (e2p->LastPumpCycle->auto_pump_quantity < 0) e2p->LastPumpCycle->auto_pump_quantity = 999;
			
			// Если установить объём автоподкачки на 0, то можно повторить цикл
			if (e2p->LastPumpCycle->auto_pump_quantity == 0)
			{
				// Сброс флага выполнения автоподкачки
				e2p->LastPumpCycle->auto_pump_is_done = 0;
			}

			break;
		}

		// Увеличение объёма автоподкачки, литры*10 *******************************************
		case AutoPumpQuantityInc:
		{
			if (large_step == 0)			e2p->LastPumpCycle->auto_pump_quantity += 1;
			else if (large_step == 1)	e2p->LastPumpCycle->auto_pump_quantity += 100;
			else if (large_step == 2)	e2p->LastPumpCycle->auto_pump_quantity += 100;
			else if (large_step == 3)	e2p->LastPumpCycle->auto_pump_quantity += 100;
			
			if (e2p->LastPumpCycle->auto_pump_quantity > 999) e2p->LastPumpCycle->auto_pump_quantity = 0;
			
			// Если установить объём автоподкачки на 0, то можно повторить цикл
			if (e2p->LastPumpCycle->auto_pump_quantity == 0)
			{
				// Сброс флага выполнения автоподкачки
				e2p->LastPumpCycle->auto_pump_is_done = 0;
			}
			
			break;			
		}
		// Уменьшение значения текущего времени *****************************************
		case CurrentTimeDecrement:
		{
			// Чтение времени
			e2p->Statistics->TimeInSeconds = Get_time_in_sec(hrtc);
			if (large_step == 0)			e2p->Statistics->TimeInSeconds -= 60;
			else if (large_step == 1)	e2p->Statistics->TimeInSeconds -= 600;
			else if (large_step == 2)	e2p->Statistics->TimeInSeconds -= 600;
			else if (large_step == 3)	e2p->Statistics->TimeInSeconds -= 600;

			// Обнуление секунд во время настройки времени
			e2p->Statistics->TimeInSeconds -= (e2p->Statistics->TimeInSeconds % 60);
			
			if (e2p->Statistics->TimeInSeconds < 0) e2p->Statistics->TimeInSeconds = 86399;
			// Установка времени
			Write_time_to_RTC(hrtc, e2p->Statistics->TimeInSeconds);
			break;
		}

		// Увеличение значения текущего времени
		case CurrentTimeIncrement:
		{
			// Чтение времени
			e2p->Statistics->TimeInSeconds = Get_time_in_sec(hrtc);
			if (large_step == 0)			e2p->Statistics->TimeInSeconds += 60;
			else if (large_step == 1)	e2p->Statistics->TimeInSeconds += 600;
			else if (large_step == 2)	e2p->Statistics->TimeInSeconds += 600;
			else if (large_step == 3)	e2p->Statistics->TimeInSeconds += 600;

			// Обнуление секунд во время настройки времени
			e2p->Statistics->TimeInSeconds -= (e2p->Statistics->TimeInSeconds % 60);
			
			if (e2p->Statistics->TimeInSeconds > 86399) e2p->Statistics->TimeInSeconds = 0;
			// Установка времени
			Write_time_to_RTC(hrtc, e2p->Statistics->TimeInSeconds);
			break;
		}
	
		// Уменьшение значения коррекции текущего времени *******************************
		case TimeCorrectionValueDec:
		{
			e2p->Calibrations->TimeCorrectionValue -= 1;

			if (e2p->Calibrations->TimeCorrectionValue < -125) e2p->Calibrations->TimeCorrectionValue = -125;
			break;
		}

		// Увеличение значения коррекции текущего времени
		case TimeCorrectionValueInc:
		{
			e2p->Calibrations->TimeCorrectionValue += 1;

			if (e2p->Calibrations->TimeCorrectionValue > 125) e2p->Calibrations->TimeCorrectionValue = 125;
			break;
		}
	
		// Уменьшение минимальной точки источника воды *******************************
		case VoltageForPminSourceDec:
		{
			e2p->Calibrations->SourcePsensorMinPressureVoltageValue -= 1;

			if (e2p->Calibrations->SourcePsensorMinPressureVoltageValue < 0) e2p->Calibrations->SourcePsensorMinPressureVoltageValue = 0;
			break;
		}

		// Увеличение минимальной точки источника воды
		case VoltageForPminSourceInc:
		{
			e2p->Calibrations->SourcePsensorMinPressureVoltageValue += 1;

			if (e2p->Calibrations->SourcePsensorMinPressureVoltageValue > 160) e2p->Calibrations->SourcePsensorMinPressureVoltageValue = 160;
			break;
		}
	
		// Уменьшение максимальной точки источника воды *******************************
		case VoltageForPmaxSourceDec:
		{
			e2p->Calibrations->SourcePsensorMaxPressureVoltageValue -= 1;

			if (e2p->Calibrations->SourcePsensorMaxPressureVoltageValue < 0) e2p->Calibrations->SourcePsensorMaxPressureVoltageValue = 0;
			break;
		}

		// Увеличение максимальной точки источника воды
		case VoltageForPmaxSourceInc:
		{
			e2p->Calibrations->SourcePsensorMaxPressureVoltageValue += 1;

			if (e2p->Calibrations->SourcePsensorMaxPressureVoltageValue > 160) e2p->Calibrations->SourcePsensorMaxPressureVoltageValue = 160;
			break;
		}

		// Уменьшение минимальной точки накопителя воды *******************************
		case VoltageForPminTankDec:
		{
			e2p->Calibrations->TankPsensorMinPressureVoltageValue -= 1;

			if (e2p->Calibrations->TankPsensorMinPressureVoltageValue < 0) e2p->Calibrations->TankPsensorMinPressureVoltageValue = 0;
			break;
		}

		// Увеличение минимальной точки накопителя воды
		case VoltageForPminTankInc:
		{
			e2p->Calibrations->TankPsensorMinPressureVoltageValue += 1;

			if (e2p->Calibrations->TankPsensorMinPressureVoltageValue > 160) e2p->Calibrations->TankPsensorMinPressureVoltageValue = 160;
			break;
		}
	
		// Уменьшение максимальной точки накопителя воды *******************************
		case VoltageForPmaxTankDec:
		{
			e2p->Calibrations->TankPsensorMaxPressureVoltageValue -= 1;

			if (e2p->Calibrations->TankPsensorMaxPressureVoltageValue < 0) e2p->Calibrations->TankPsensorMaxPressureVoltageValue = 0;
			break;
		}

		// Увеличение максимальной точки накопителя воды
		case VoltageForPmaxTankInc:
		{
			e2p->Calibrations->TankPsensorMaxPressureVoltageValue += 1;

			if (e2p->Calibrations->TankPsensorMaxPressureVoltageValue > 160) e2p->Calibrations->TankPsensorMaxPressureVoltageValue = 160;
			break;
		}

		// Установка минимальной точки источника воды по текущему значению напряжения датч. давления****
		case SetVoltageForPminSource:
		{
			// Треб. преобр в %
			e2p->Calibrations->SourcePsensorMinPressureVoltageValue = last_pump_cycle.WellWaterLevelInVolts;

			if (e2p->Calibrations->SourcePsensorMinPressureVoltageValue < 0) e2p->Calibrations->SourcePsensorMinPressureVoltageValue = 0;
			break;
		}
		// Установка максимальной точки источника воды по текущему значению напряжения датч. давления
		case SetVoltageForPmaxSource:
		{
			// Треб. преобр в %
			e2p->Calibrations->SourcePsensorMaxPressureVoltageValue = last_pump_cycle.WellWaterLevelInVolts;

			if (e2p->Calibrations->SourcePsensorMaxPressureVoltageValue > 100) e2p->Calibrations->SourcePsensorMaxPressureVoltageValue = 100;
			break;
		}

		// Установка минимальной точки накопителя воды по текущему значению напряжения датч. давления****
		case SetVoltageForPminTank:
		{
			// Треб. преобр в %
			e2p->Calibrations->TankPsensorMinPressureVoltageValue = last_pump_cycle.TankWaterLevelInVolts;

			if (e2p->Calibrations->TankPsensorMinPressureVoltageValue < 0) e2p->Calibrations->TankPsensorMinPressureVoltageValue = 0;
			break;
		}
		// Установка максимальной точки накопителя воды по текущему значению напряжения датч. давления
		case SetVoltageForPmaxTank:
		{
			// Треб. преобр в %
			e2p->Calibrations->TankPsensorMaxPressureVoltageValue = last_pump_cycle.TankWaterLevelInVolts;

			if (e2p->Calibrations->TankPsensorMaxPressureVoltageValue > 100) e2p->Calibrations->TankPsensorMaxPressureVoltageValue = 100;
			break;
		}

		// Сброс всех настроек
		case ResetAllSettingsToDefault:
		{
			Set_all_variables_to_default(e2p);

			break;
		}
	}
}


// Проверка длины, подсчет к.с. стандартной строки NMEA0183
uint8_t Nmea_string_check_checksum(uint8_t * buf, uint16_t lenght)
{
	const uint8_t string_termination_is_absent=0xFD;
	const uint8_t checksum_error=0xFE;
	const uint8_t no_errors=0x00;
	uint8_t temp,msb,lsb,idx;
	uint8_t rx_buffer_checksum=0; 

	// Проверка на наличие в конце принятой строки терминатора строки 
	idx=(lenght-2);
	if (buf[idx++] != 0x0D)
	{
		return string_termination_is_absent;
	}
	if (buf[idx++] != 0x0A)
	{
		return string_termination_is_absent;
	}

	// Проверка контр. суммы
	for (idx=1;idx<(lenght-5);idx++)
	{
		rx_buffer_checksum^=buf[idx];
	}
	// сверяем с принятым значением
	idx=lenght-4;    
	msb=buf[idx++];
	lsb=buf[idx];
	temp=ASCII2HEX(msb,lsb);
	if (rx_buffer_checksum != temp)
	{
		return checksum_error;
	}

	return no_errors;
}

		
// Подсчет к.с. буфера и запись в передаваемую строку
void Set_string_binary_checksum(uint8_t  * buf, uint16_t lenght)
{
	uint8_t 	idx,buffer_checksum=0;

	// xor-им всё между $ и *
	for (idx=1;idx<(lenght-4);idx++)
	{
		buffer_checksum^=buf[idx];
	}

	idx=lenght-3;
	buf[idx++] = buffer_checksum;
}


// Начальная разметка строки для отправки данных через ком порт в дисплей Nextion
void Init_string_to_nextion(void)
{	
	// Страница 0
	{
	// Давление воды в системе, атм * 10
	nextion.TxdBuffer[0] = 'x';
	nextion.TxdBuffer[1] = '0';
	nextion.TxdBuffer[2] = '.';
	nextion.TxdBuffer[3] = 'v';
	nextion.TxdBuffer[4] = 'a';
	nextion.TxdBuffer[5] = 'l';
	nextion.TxdBuffer[6] = '=';
	nextion.TxdBuffer[7] = '0';
	nextion.TxdBuffer[8] = '0';
	nextion.TxdBuffer[9] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[10] = 0xFF;
	nextion.TxdBuffer[11] = 0xFF;
	nextion.TxdBuffer[12] = 0xFF;


	// Время работы насоса в одном цикле, час
	nextion.TxdBuffer[13] = 'n';
	nextion.TxdBuffer[14] = '4';
	nextion.TxdBuffer[15] = '.';
	nextion.TxdBuffer[16] = 'v';
	nextion.TxdBuffer[17] = 'a';
	nextion.TxdBuffer[18] = 'l';
	nextion.TxdBuffer[19] = '=';
	nextion.TxdBuffer[20] = '0';
	nextion.TxdBuffer[21] = '0';
	// Терминатор команды
	nextion.TxdBuffer[22] = 0xFF;
	nextion.TxdBuffer[23] = 0xFF;
	nextion.TxdBuffer[24] = 0xFF;
		
	// Время работы насоса в одном цикле, мин
	nextion.TxdBuffer[25] = 'n';
	nextion.TxdBuffer[26] = '5';
	nextion.TxdBuffer[27] = '.';
	nextion.TxdBuffer[28] = 'v';
	nextion.TxdBuffer[29] = 'a';
	nextion.TxdBuffer[30] = 'l';
	nextion.TxdBuffer[31] = '=';
	nextion.TxdBuffer[32] = '0';
	nextion.TxdBuffer[33] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[34] = 0xFF;
	nextion.TxdBuffer[35] = 0xFF;
	nextion.TxdBuffer[36] = 0xFF;

	// Время работы насоса в одном цикле, сек
	nextion.TxdBuffer[37] = 'n';
	nextion.TxdBuffer[38] = '6';
	nextion.TxdBuffer[39] = '.';
	nextion.TxdBuffer[40] = 'v';
	nextion.TxdBuffer[41] = 'a';
	nextion.TxdBuffer[42] = 'l';
	nextion.TxdBuffer[43] = '=';
	nextion.TxdBuffer[44] = '0';
	nextion.TxdBuffer[45] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[46] = 0xFF;
	nextion.TxdBuffer[47] = 0xFF;
	nextion.TxdBuffer[48] = 0xFF;


	// Объём перекачанной за один цикл воды, л
	nextion.TxdBuffer[49] = 'x';
	nextion.TxdBuffer[50] = '1';
	nextion.TxdBuffer[51] = '.';
	nextion.TxdBuffer[52] = 'v';
	nextion.TxdBuffer[53] = 'a';
	nextion.TxdBuffer[54] = 'l';
	nextion.TxdBuffer[55] = '=';
	nextion.TxdBuffer[56] = '0';
	nextion.TxdBuffer[57] = '0';
	nextion.TxdBuffer[58] = '0';
	nextion.TxdBuffer[59] = '0';
	nextion.TxdBuffer[60] = '0';
	nextion.TxdBuffer[61] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[62] = 0xFF;
	nextion.TxdBuffer[63] = 0xFF;
	nextion.TxdBuffer[64] = 0xFF;


	// t воды при перекач, 'С
	nextion.TxdBuffer[65] = 'x';
	nextion.TxdBuffer[66] = '2';
	nextion.TxdBuffer[67] = '.';
	nextion.TxdBuffer[68] = 'v';
	nextion.TxdBuffer[69] = 'a';
	nextion.TxdBuffer[70] = 'l';
	nextion.TxdBuffer[71] = '=';
	nextion.TxdBuffer[72] = '0';
	nextion.TxdBuffer[73] = '0';
	nextion.TxdBuffer[74] = '0';
	// Терминатор команды
	nextion.TxdBuffer[75] = 0xFF;
	nextion.TxdBuffer[76] = 0xFF;
	nextion.TxdBuffer[77] = 0xFF;


	// Текущ. t воды, 'С
	nextion.TxdBuffer[78] = 'x';
	nextion.TxdBuffer[79] = '3';
	nextion.TxdBuffer[80] = '.';
	nextion.TxdBuffer[81] = 'v';
	nextion.TxdBuffer[82] = 'a';
	nextion.TxdBuffer[83] = 'l';
	nextion.TxdBuffer[84] = '=';
	nextion.TxdBuffer[85] = '0';
	nextion.TxdBuffer[86] = '0';
	nextion.TxdBuffer[87] = '0';
	// Терминатор команды
	nextion.TxdBuffer[88] = 0xFF;
	nextion.TxdBuffer[89] = 0xFF;
	nextion.TxdBuffer[90] = 0xFF;
	

	// Текущее время, часы
	nextion.TxdBuffer[91] = 'n';
	nextion.TxdBuffer[92] = '1';
	nextion.TxdBuffer[93] = '.';
	nextion.TxdBuffer[94] = 'v';
	nextion.TxdBuffer[95] = 'a';
	nextion.TxdBuffer[96] = 'l';
	nextion.TxdBuffer[97] = '=';
	nextion.TxdBuffer[98] = '0';
	nextion.TxdBuffer[99] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[100] = 0xFF;
	nextion.TxdBuffer[101] = 0xFF;
	nextion.TxdBuffer[102] = 0xFF;

	// Текущее время, минуты
	nextion.TxdBuffer[103] = 'n';
	nextion.TxdBuffer[104] = '2';
	nextion.TxdBuffer[105] = '.';
	nextion.TxdBuffer[106] = 'v';
	nextion.TxdBuffer[107] = 'a';
	nextion.TxdBuffer[108] = 'l';
	nextion.TxdBuffer[109] = '=';
	nextion.TxdBuffer[110] = '0';
	nextion.TxdBuffer[111] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[112] = 0xFF;
	nextion.TxdBuffer[113] = 0xFF;
	nextion.TxdBuffer[114] = 0xFF;

	// Текущее время, секунды
	nextion.TxdBuffer[115] = 'n';
	nextion.TxdBuffer[116] = '3';
	nextion.TxdBuffer[117] = '.';
	nextion.TxdBuffer[118] = 'v';
	nextion.TxdBuffer[119] = 'a';
	nextion.TxdBuffer[120] = 'l';
	nextion.TxdBuffer[121] = '=';
	nextion.TxdBuffer[122] = '0';
	nextion.TxdBuffer[123] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[124] = 0xFF;
	nextion.TxdBuffer[125] = 0xFF;
	nextion.TxdBuffer[126] = 0xFF;


	// t воды в источнике, 'С
	nextion.TxdBuffer[127] = 'x';
	nextion.TxdBuffer[128] = '4';
	nextion.TxdBuffer[129] = '.';
	nextion.TxdBuffer[130] = 'v';
	nextion.TxdBuffer[131] = 'a';
	nextion.TxdBuffer[132] = 'l';
	nextion.TxdBuffer[133] = '=';
	nextion.TxdBuffer[134] = '0';
	nextion.TxdBuffer[135] = '0';
	nextion.TxdBuffer[136] = '0';
	// Терминатор команды
	nextion.TxdBuffer[137] = 0xFF;
	nextion.TxdBuffer[138] = 0xFF;
	nextion.TxdBuffer[139] = 0xFF;

	// Уровень воды в источнике, %
	nextion.TxdBuffer[140] = 'x';
	nextion.TxdBuffer[141] = '6';
	nextion.TxdBuffer[142] = '.';
	nextion.TxdBuffer[143] = 'v';
	nextion.TxdBuffer[144] = 'a';
	nextion.TxdBuffer[145] = 'l';
	nextion.TxdBuffer[146] = '=';
	nextion.TxdBuffer[147] = '0';
	nextion.TxdBuffer[148] = '0';
	nextion.TxdBuffer[149] = '0';
	// Терминатор команды
	nextion.TxdBuffer[150] = 0xFF;
	nextion.TxdBuffer[151] = 0xFF;
	nextion.TxdBuffer[152] = 0xFF;

	// Графический уровень воды в источнике, %
	nextion.TxdBuffer[153] = 'j';
	nextion.TxdBuffer[154] = '0';
	nextion.TxdBuffer[155] = '.';
	nextion.TxdBuffer[156] = 'v';
	nextion.TxdBuffer[157] = 'a';
	nextion.TxdBuffer[158] = 'l';
	nextion.TxdBuffer[159] = '=';
	nextion.TxdBuffer[160] = '0';
	nextion.TxdBuffer[161] = '0';
	nextion.TxdBuffer[162] = '0';
	// Терминатор команды
	nextion.TxdBuffer[163] = 0xFF;
	nextion.TxdBuffer[164] = 0xFF;
	nextion.TxdBuffer[165] = 0xFF;


	// t воды в накопителе, 'С
	nextion.TxdBuffer[166] = 'x';
	nextion.TxdBuffer[167] = '5';
	nextion.TxdBuffer[168] = '.';
	nextion.TxdBuffer[169] = 'v';
	nextion.TxdBuffer[170] = 'a';
	nextion.TxdBuffer[171] = 'l';
	nextion.TxdBuffer[172] = '=';
	nextion.TxdBuffer[173] = '0';
	nextion.TxdBuffer[174] = '0';
	nextion.TxdBuffer[175] = '0';
	// Терминатор команды
	nextion.TxdBuffer[176] = 0xFF;
	nextion.TxdBuffer[177] = 0xFF;
	nextion.TxdBuffer[178] = 0xFF;

	// Уровень воды в накопителе, %
	nextion.TxdBuffer[179] = 'x';
	nextion.TxdBuffer[180] = '7';
	nextion.TxdBuffer[181] = '.';
	nextion.TxdBuffer[182] = 'v';
	nextion.TxdBuffer[183] = 'a';
	nextion.TxdBuffer[184] = 'l';
	nextion.TxdBuffer[185] = '=';
	nextion.TxdBuffer[186] = '0';
	nextion.TxdBuffer[187] = '0';
	nextion.TxdBuffer[188] = '0';
	// Терминатор команды
	nextion.TxdBuffer[189] = 0xFF;
	nextion.TxdBuffer[190] = 0xFF;
	nextion.TxdBuffer[191] = 0xFF;

	// Графический уровень воды в накопителе, %
	nextion.TxdBuffer[192] = 'j';
	nextion.TxdBuffer[193] = '1';
	nextion.TxdBuffer[194] = '.';
	nextion.TxdBuffer[195] = 'v';
	nextion.TxdBuffer[196] = 'a';
	nextion.TxdBuffer[197] = 'l';
	nextion.TxdBuffer[198] = '=';
	nextion.TxdBuffer[199] = '0';
	nextion.TxdBuffer[200] = '0';
	nextion.TxdBuffer[201] = '0';
	// Терминатор команды
	nextion.TxdBuffer[202] = 0xFF;
	nextion.TxdBuffer[203] = 0xFF;
	nextion.TxdBuffer[204] = 0xFF;
	}


	// Страница 1 (настройки 1)
	{
	// P вкл. насоса, атм * 10
	nextion.TxdBuffer[205] = 'x';
	nextion.TxdBuffer[206] = '1';
	nextion.TxdBuffer[207] = '0';
	nextion.TxdBuffer[208] = '2';
	nextion.TxdBuffer[209] = '.';
	nextion.TxdBuffer[210] = 'v';
	nextion.TxdBuffer[211] = 'a';
	nextion.TxdBuffer[212] = 'l';
	nextion.TxdBuffer[213] = '=';
	nextion.TxdBuffer[214] = '0';
	nextion.TxdBuffer[215] = '0';
	nextion.TxdBuffer[216] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[217] = 0xFF;
	nextion.TxdBuffer[218] = 0xFF;
	nextion.TxdBuffer[219] = 0xFF;

	// P выкл. насоса, атм * 10
	nextion.TxdBuffer[220] = 'x';
	nextion.TxdBuffer[221] = '1';
	nextion.TxdBuffer[222] = '0';
	nextion.TxdBuffer[223] = '3';
	nextion.TxdBuffer[224] = '.';
	nextion.TxdBuffer[225] = 'v';
	nextion.TxdBuffer[226] = 'a';
	nextion.TxdBuffer[227] = 'l';
	nextion.TxdBuffer[228] = '=';
	nextion.TxdBuffer[229] = '0';
	nextion.TxdBuffer[230] = '0';
	nextion.TxdBuffer[231] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[232] = 0xFF;
	nextion.TxdBuffer[233] = 0xFF;
	nextion.TxdBuffer[234] = 0xFF;
		

		// Калибровка датчика давления: P min, атм * 10
	nextion.TxdBuffer[235] = 'x';
	nextion.TxdBuffer[236] = '1';
	nextion.TxdBuffer[237] = '0';
	nextion.TxdBuffer[238] = '0';
	nextion.TxdBuffer[239] = '.';
	nextion.TxdBuffer[240] = 'v';
	nextion.TxdBuffer[241] = 'a';
	nextion.TxdBuffer[242] = 'l';
	nextion.TxdBuffer[243] = '=';
	nextion.TxdBuffer[244] = '0';
	nextion.TxdBuffer[245] = '0';
	nextion.TxdBuffer[246] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[247] = 0xFF;
	nextion.TxdBuffer[248] = 0xFF;
	nextion.TxdBuffer[249] = 0xFF;

	// Калибровка датчика давления: P max, атм * 10
	nextion.TxdBuffer[250] = 'x';
	nextion.TxdBuffer[251] = '1';
	nextion.TxdBuffer[252] = '0';
	nextion.TxdBuffer[253] = '1';
	nextion.TxdBuffer[254] = '.';
	nextion.TxdBuffer[255] = 'v';
	nextion.TxdBuffer[256] = 'a';
	nextion.TxdBuffer[257] = 'l';
	nextion.TxdBuffer[258] = '=';
	nextion.TxdBuffer[259] = '0';
	nextion.TxdBuffer[260] = '0';
	nextion.TxdBuffer[261] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[262] = 0xFF;
	nextion.TxdBuffer[263] = 0xFF;
	nextion.TxdBuffer[264] = 0xFF;

	// Калибровка датчика давления: P min = U min, В/100
	nextion.TxdBuffer[265] = 'x';
	nextion.TxdBuffer[266] = '1';
	nextion.TxdBuffer[267] = '0';
	nextion.TxdBuffer[268] = '4';
	nextion.TxdBuffer[269] = '.';
	nextion.TxdBuffer[270] = 'v';
	nextion.TxdBuffer[271] = 'a';
	nextion.TxdBuffer[272] = 'l';
	nextion.TxdBuffer[273] = '=';
	nextion.TxdBuffer[274] = '0';
	nextion.TxdBuffer[275] = '0';	
	nextion.TxdBuffer[276] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[277] = 0xFF;
	nextion.TxdBuffer[278] = 0xFF;
	nextion.TxdBuffer[279] = 0xFF;

	// Калибровка датчика давления: P max = U max, В/100
	nextion.TxdBuffer[280] = 'x';
	nextion.TxdBuffer[281] = '1';
	nextion.TxdBuffer[282] = '0';
	nextion.TxdBuffer[283] = '5';
	nextion.TxdBuffer[284] = '.';
	nextion.TxdBuffer[285] = 'v';
	nextion.TxdBuffer[286] = 'a';
	nextion.TxdBuffer[287] = 'l';
	nextion.TxdBuffer[288] = '=';
	nextion.TxdBuffer[289] = '0';
	nextion.TxdBuffer[290] = '0';	
	nextion.TxdBuffer[291] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[292] = 0xFF;
	nextion.TxdBuffer[293] = 0xFF;
	nextion.TxdBuffer[294] = 0xFF;
	}

	// Страница 2 (статистика)
	{
	// Общее время работы контроллера, часов.минут
	nextion.TxdBuffer[295] = 'x';
	nextion.TxdBuffer[296] = '2';
	nextion.TxdBuffer[297] = '0';
	nextion.TxdBuffer[298] = '0';
	nextion.TxdBuffer[299] = '.';
	nextion.TxdBuffer[300] = 'v';
	nextion.TxdBuffer[301] = 'a';
	nextion.TxdBuffer[302] = 'l';
	nextion.TxdBuffer[303] = '=';
	nextion.TxdBuffer[304] = '0';
	nextion.TxdBuffer[305] = '0';
	nextion.TxdBuffer[306] = '0';
	nextion.TxdBuffer[307] = '0';
	nextion.TxdBuffer[308] = '0';
	nextion.TxdBuffer[309] = '0';
	nextion.TxdBuffer[310] = '0';	
	nextion.TxdBuffer[311] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[312] = 0xFF;
	nextion.TxdBuffer[313] = 0xFF;
	nextion.TxdBuffer[314] = 0xFF;


	// Общее время работы насоса, часов.минут
	nextion.TxdBuffer[315] = 'x';
	nextion.TxdBuffer[316] = '2';
	nextion.TxdBuffer[317] = '0';
	nextion.TxdBuffer[318] = '1';
	nextion.TxdBuffer[319] = '.';
	nextion.TxdBuffer[320] = 'v';
	nextion.TxdBuffer[321] = 'a';
	nextion.TxdBuffer[322] = 'l';
	nextion.TxdBuffer[323] = '=';
	nextion.TxdBuffer[324] = '0';
	nextion.TxdBuffer[325] = '0';
	nextion.TxdBuffer[326] = '0';
	nextion.TxdBuffer[327] = '0';
	nextion.TxdBuffer[328] = '0';
	nextion.TxdBuffer[329] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[330] = 0xFF;
	nextion.TxdBuffer[331] = 0xFF;
	nextion.TxdBuffer[332] = 0xFF;


	// Общее кол-во перекачанной воды, куб. м/100
	nextion.TxdBuffer[333] = 'x';
	nextion.TxdBuffer[334] = '2';
	nextion.TxdBuffer[335] = '0';
	nextion.TxdBuffer[336] = '2';
	nextion.TxdBuffer[337] = '.';
	nextion.TxdBuffer[338] = 'v';
	nextion.TxdBuffer[339] = 'a';
	nextion.TxdBuffer[340] = 'l';
	nextion.TxdBuffer[341] = '=';
	nextion.TxdBuffer[342] = '0';
	nextion.TxdBuffer[343] = '0';
	nextion.TxdBuffer[344] = '0';
	nextion.TxdBuffer[345] = '0';
	nextion.TxdBuffer[346] = '0';
	nextion.TxdBuffer[347] = '0';
	nextion.TxdBuffer[348] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[349] = 0xFF;
	nextion.TxdBuffer[350] = 0xFF;
	nextion.TxdBuffer[351] = 0xFF;


	// Кол-во перекачанной воды за сутки, куб. м/100
	nextion.TxdBuffer[352] = 'x';
	nextion.TxdBuffer[353] = '2';
	nextion.TxdBuffer[354] = '0';
	nextion.TxdBuffer[355] = '3';
	nextion.TxdBuffer[356] = '.';
	nextion.TxdBuffer[357] = 'v';
	nextion.TxdBuffer[358] = 'a';
	nextion.TxdBuffer[359] = 'l';
	nextion.TxdBuffer[360] = '=';
	nextion.TxdBuffer[361] = '0';
	nextion.TxdBuffer[362] = '0';
	nextion.TxdBuffer[363] = '0';
	nextion.TxdBuffer[364] = '0';
	nextion.TxdBuffer[365] = '0';
	// Терминатор команды
	nextion.TxdBuffer[366] = 0xFF;
	nextion.TxdBuffer[367] = 0xFF;
	nextion.TxdBuffer[368] = 0xFF;


	// Кол-во перекачанной воды за неделю, куб. м/100
	nextion.TxdBuffer[369] = 'x';
	nextion.TxdBuffer[370] = '2';
	nextion.TxdBuffer[371] = '0';
	nextion.TxdBuffer[372] = '4';
	nextion.TxdBuffer[373] = '.';
	nextion.TxdBuffer[374] = 'v';
	nextion.TxdBuffer[375] = 'a';
	nextion.TxdBuffer[376] = 'l';
	nextion.TxdBuffer[377] = '=';
	nextion.TxdBuffer[378] = '0';
	nextion.TxdBuffer[379] = '0';
	nextion.TxdBuffer[380] = '0';
	nextion.TxdBuffer[381] = '0';
	nextion.TxdBuffer[382] = '0';
	nextion.TxdBuffer[383] = '0';
	// Терминатор команды
	nextion.TxdBuffer[384] = 0xFF;
	nextion.TxdBuffer[385] = 0xFF;
	nextion.TxdBuffer[386] = 0xFF;


	// Минимальная суточная t воды в источнике, 'C:
	nextion.TxdBuffer[387] = 'x';
	nextion.TxdBuffer[388] = '2';
	nextion.TxdBuffer[389] = '0';
	nextion.TxdBuffer[390] = '5';
	nextion.TxdBuffer[391] = '.';
	nextion.TxdBuffer[392] = 'v';
	nextion.TxdBuffer[393] = 'a';
	nextion.TxdBuffer[394] = 'l';
	nextion.TxdBuffer[395] = '=';
	nextion.TxdBuffer[396] = '0';
	nextion.TxdBuffer[397] = '0';
	nextion.TxdBuffer[398] = '0';
	// Терминатор команды
	nextion.TxdBuffer[399] = 0xFF;
	nextion.TxdBuffer[400] = 0xFF;
	nextion.TxdBuffer[401] = 0xFF;

	// Максимальная суточная t воды в источнике, 'C:
	nextion.TxdBuffer[402] = 'x';
	nextion.TxdBuffer[403] = '2';
	nextion.TxdBuffer[404] = '0';
	nextion.TxdBuffer[405] = '6';
	nextion.TxdBuffer[406] = '.';
	nextion.TxdBuffer[407] = 'v';
	nextion.TxdBuffer[408] = 'a';
	nextion.TxdBuffer[409] = 'l';
	nextion.TxdBuffer[410] = '=';
	nextion.TxdBuffer[411] = '0';
	nextion.TxdBuffer[412] = '0';
	nextion.TxdBuffer[413] = '0';
	// Терминатор команды
	nextion.TxdBuffer[414] = 0xFF;
	nextion.TxdBuffer[415] = 0xFF;
	nextion.TxdBuffer[416] = 0xFF;


	// Минимальная суточная t воды в накопителе, 'C:
	nextion.TxdBuffer[417] = 'x';
	nextion.TxdBuffer[418] = '2';
	nextion.TxdBuffer[419] = '0';
	nextion.TxdBuffer[420] = '7';
	nextion.TxdBuffer[421] = '.';
	nextion.TxdBuffer[422] = 'v';
	nextion.TxdBuffer[423] = 'a';
	nextion.TxdBuffer[424] = 'l';
	nextion.TxdBuffer[425] = '=';
	nextion.TxdBuffer[426] = '0';
	nextion.TxdBuffer[427] = '0';
	nextion.TxdBuffer[428] = '0';
	// Терминатор команды
	nextion.TxdBuffer[429] = 0xFF;
	nextion.TxdBuffer[430] = 0xFF;
	nextion.TxdBuffer[431] = 0xFF;

	// Максимальная суточная t воды в накопителе, 'C:
	nextion.TxdBuffer[432] = 'x';
	nextion.TxdBuffer[433] = '2';
	nextion.TxdBuffer[434] = '0';
	nextion.TxdBuffer[435] = '8';
	nextion.TxdBuffer[436] = '.';
	nextion.TxdBuffer[437] = 'v';
	nextion.TxdBuffer[438] = 'a';
	nextion.TxdBuffer[439] = 'l';
	nextion.TxdBuffer[440] = '=';
	nextion.TxdBuffer[441] = '0';
	nextion.TxdBuffer[442] = '0';
	nextion.TxdBuffer[443] = '0';
	// Терминатор команды
	nextion.TxdBuffer[444] = 0xFF;
	nextion.TxdBuffer[445] = 0xFF;
	nextion.TxdBuffer[446] = 0xFF;
	}

	// Страница 3 (упр. поливом)
	{
	// Выбор выхода 1-8
	nextion.TxdBuffer[447] = 'n';
	nextion.TxdBuffer[448] = '3';
	nextion.TxdBuffer[449] = '0';
	nextion.TxdBuffer[450] = '0';
	nextion.TxdBuffer[451] = '.';
	nextion.TxdBuffer[452] = 'v';
	nextion.TxdBuffer[453] = 'a';
	nextion.TxdBuffer[454] = 'l';
	nextion.TxdBuffer[455] = '=';
	nextion.TxdBuffer[456] = '0';
	// Терминатор команды
	nextion.TxdBuffer[457] = 0xFF;
	nextion.TxdBuffer[458] = 0xFF;
	nextion.TxdBuffer[459] = 0xFF;

	// Смещение от начала суток, час, мин
	nextion.TxdBuffer[460] = 'x';
	nextion.TxdBuffer[461] = '3';
	nextion.TxdBuffer[462] = '0';
	nextion.TxdBuffer[463] = '0';
	nextion.TxdBuffer[464] = '.';
	nextion.TxdBuffer[465] = 'v';
	nextion.TxdBuffer[466] = 'a';
	nextion.TxdBuffer[467] = 'l';
	nextion.TxdBuffer[468] = '=';
	nextion.TxdBuffer[469] = '0';
	nextion.TxdBuffer[470] = '0';
	nextion.TxdBuffer[471] = '0';
	nextion.TxdBuffer[472] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[473] = 0xFF;
	nextion.TxdBuffer[474] = 0xFF;
	nextion.TxdBuffer[475] = 0xFF;

	// Время работы, час, мин
	nextion.TxdBuffer[476] = 'x';
	nextion.TxdBuffer[477] = '3';
	nextion.TxdBuffer[478] = '0';
	nextion.TxdBuffer[479] = '1';
	nextion.TxdBuffer[480] = '.';
	nextion.TxdBuffer[481] = 'v';
	nextion.TxdBuffer[482] = 'a';
	nextion.TxdBuffer[483] = 'l';
	nextion.TxdBuffer[484] = '=';
	nextion.TxdBuffer[485] = '0';
	nextion.TxdBuffer[486] = '0';
	nextion.TxdBuffer[487] = '0';
	nextion.TxdBuffer[488] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[489] = 0xFF;
	nextion.TxdBuffer[490] = 0xFF;
	nextion.TxdBuffer[491] = 0xFF;

	// Периодичность включения, час, мин
	nextion.TxdBuffer[492] = 'x';
	nextion.TxdBuffer[493] = '3';
	nextion.TxdBuffer[494] = '0';
	nextion.TxdBuffer[495] = '2';
	nextion.TxdBuffer[496] = '.';
	nextion.TxdBuffer[497] = 'v';
	nextion.TxdBuffer[498] = 'a';
	nextion.TxdBuffer[499] = 'l';
	nextion.TxdBuffer[500] = '=';
	nextion.TxdBuffer[501] = '0';
	nextion.TxdBuffer[502] = '0';
	nextion.TxdBuffer[503] = '0';
	nextion.TxdBuffer[504] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[505] = 0xFF;
	nextion.TxdBuffer[506] = 0xFF;
	nextion.TxdBuffer[507] = 0xFF;
	}
	

	// Страница 4 (настройки 2)
	{
	// Текущее значение прибора учета, куб. м/1000
	nextion.TxdBuffer[508] = 'x';
	nextion.TxdBuffer[509] = '4';
	nextion.TxdBuffer[510] = '0';
	nextion.TxdBuffer[511] = '0';
	nextion.TxdBuffer[512] = '.';
	nextion.TxdBuffer[513] = 'v';
	nextion.TxdBuffer[514] = 'a';
	nextion.TxdBuffer[515] = 'l';
	nextion.TxdBuffer[516] = '=';
	nextion.TxdBuffer[517] = '0';
	nextion.TxdBuffer[518] = '0';
	nextion.TxdBuffer[519] = '0';
	nextion.TxdBuffer[520] = '0';
	nextion.TxdBuffer[521] = '0';
	nextion.TxdBuffer[522] = '0';
	nextion.TxdBuffer[523] = '0';
	nextion.TxdBuffer[524] = '0';
	// Терминатор команды
	nextion.TxdBuffer[525] = 0xFF;
	nextion.TxdBuffer[526] = 0xFF;
	nextion.TxdBuffer[527] = 0xFF;


	// Ежесуточное автоподкачивание воды: 
	// Смещение от начала суток, час, мин
	nextion.TxdBuffer[528] = 'x';
	nextion.TxdBuffer[529] = '4';
	nextion.TxdBuffer[530] = '0';
	nextion.TxdBuffer[531] = '1';
	nextion.TxdBuffer[532] = '.';
	nextion.TxdBuffer[533] = 'v';
	nextion.TxdBuffer[534] = 'a';
	nextion.TxdBuffer[535] = 'l';
	nextion.TxdBuffer[536] = '=';
	nextion.TxdBuffer[537] = '0';
	nextion.TxdBuffer[538] = '0';
	nextion.TxdBuffer[539] = '0';
	nextion.TxdBuffer[540] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[541] = 0xFF;
	nextion.TxdBuffer[542] = 0xFF;
	nextion.TxdBuffer[543] = 0xFF;

	// Подкачиваемый объём, л
	nextion.TxdBuffer[544] = 'x';
	nextion.TxdBuffer[545] = '4';
	nextion.TxdBuffer[546] = '0';
	nextion.TxdBuffer[547] = '2';
	nextion.TxdBuffer[548] = '.';
	nextion.TxdBuffer[549] = 'v';
	nextion.TxdBuffer[550] = 'a';
	nextion.TxdBuffer[551] = 'l';
	nextion.TxdBuffer[552] = '=';
	nextion.TxdBuffer[553] = '0';
	nextion.TxdBuffer[554] = '0';
	nextion.TxdBuffer[555] = '0';
	nextion.TxdBuffer[556] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[557] = 0xFF;
	nextion.TxdBuffer[558] = 0xFF;
	nextion.TxdBuffer[559] = 0xFF;
	
	
	// Установка времени, часы
	nextion.TxdBuffer[560] = 'n';
	nextion.TxdBuffer[561] = '4';
	nextion.TxdBuffer[562] = '0';
	nextion.TxdBuffer[563] = '0';
	nextion.TxdBuffer[564] = '.';
	nextion.TxdBuffer[565] = 'v';
	nextion.TxdBuffer[566] = 'a';
	nextion.TxdBuffer[567] = 'l';
	nextion.TxdBuffer[568] = '=';
	nextion.TxdBuffer[569] = '0';
	nextion.TxdBuffer[570] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[571] = 0xFF;
	nextion.TxdBuffer[572] = 0xFF;
	nextion.TxdBuffer[573] = 0xFF;
	// Установка времени, минуты
	nextion.TxdBuffer[574] = 'n';
	nextion.TxdBuffer[575] = '4';
	nextion.TxdBuffer[576] = '0';
	nextion.TxdBuffer[577] = '1';
	nextion.TxdBuffer[578] = '.';
	nextion.TxdBuffer[579] = 'v';
	nextion.TxdBuffer[580] = 'a';
	nextion.TxdBuffer[581] = 'l';
	nextion.TxdBuffer[582] = '=';
	nextion.TxdBuffer[583] = '0';
	nextion.TxdBuffer[584] = '0';
	// Терминатор команды
	nextion.TxdBuffer[585] = 0xFF;
	nextion.TxdBuffer[586] = 0xFF;
	nextion.TxdBuffer[587] = 0xFF;
	
	
	// Корр. времени, сек/сутки
	nextion.TxdBuffer[588] = 'n';
	nextion.TxdBuffer[589] = '4';
	nextion.TxdBuffer[590] = '0';
	nextion.TxdBuffer[591] = '2';
	nextion.TxdBuffer[592] = '.';
	nextion.TxdBuffer[593] = 'v';
	nextion.TxdBuffer[594] = 'a';
	nextion.TxdBuffer[595] = 'l';
	nextion.TxdBuffer[596] = '=';
	nextion.TxdBuffer[597] = '0';
	nextion.TxdBuffer[598] = '0';
	// Терминатор команды
	nextion.TxdBuffer[599] = 0xFF;
	nextion.TxdBuffer[600] = 0xFF;
	nextion.TxdBuffer[601] = 0xFF;	

	// Сокрытие/отрисовка символа минус "-"
	nextion.TxdBuffer[602] = 'v';
	nextion.TxdBuffer[603] = 'i';
	nextion.TxdBuffer[604] = 's';
	nextion.TxdBuffer[605] = ' ';
	nextion.TxdBuffer[606] = 't';
	nextion.TxdBuffer[607] = '4';
	nextion.TxdBuffer[608] = '0';
	nextion.TxdBuffer[609] = '9';
	nextion.TxdBuffer[610] = ',';
	nextion.TxdBuffer[611] = '0';
	// Терминатор команды
	nextion.TxdBuffer[612] = 0xFF;
	nextion.TxdBuffer[613] = 0xFF;
	nextion.TxdBuffer[614] = 0xFF;
	}


	// Страница 5 (настройки 3)
	{
	// Калибровка датчика давления источника воды: P min = U min, В/100
	nextion.TxdBuffer[615] = 'x';
	nextion.TxdBuffer[616] = '5';
	nextion.TxdBuffer[617] = '0';
	nextion.TxdBuffer[618] = '0';
	nextion.TxdBuffer[619] = '.';
	nextion.TxdBuffer[620] = 'v';
	nextion.TxdBuffer[621] = 'a';
	nextion.TxdBuffer[622] = 'l';
	nextion.TxdBuffer[623] = '=';
	nextion.TxdBuffer[624] = '0';
	nextion.TxdBuffer[625] = '0';	
	nextion.TxdBuffer[626] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[627] = 0xFF;
	nextion.TxdBuffer[628] = 0xFF;
	nextion.TxdBuffer[629] = 0xFF;

	// Калибровка датчика давления источника воды: P max = U max, В/100
	nextion.TxdBuffer[630] = 'x';
	nextion.TxdBuffer[631] = '5';
	nextion.TxdBuffer[632] = '0';
	nextion.TxdBuffer[633] = '1';
	nextion.TxdBuffer[634] = '.';
	nextion.TxdBuffer[635] = 'v';
	nextion.TxdBuffer[636] = 'a';
	nextion.TxdBuffer[637] = 'l';
	nextion.TxdBuffer[638] = '=';
	nextion.TxdBuffer[639] = '0';
	nextion.TxdBuffer[640] = '0';	
	nextion.TxdBuffer[641] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[642] = 0xFF;
	nextion.TxdBuffer[643] = 0xFF;
	nextion.TxdBuffer[644] = 0xFF;

	// Калибровка датчика давления накопителя воды: P min = U min, В/100
	nextion.TxdBuffer[645] = 'x';
	nextion.TxdBuffer[646] = '5';
	nextion.TxdBuffer[647] = '0';
	nextion.TxdBuffer[648] = '2';
	nextion.TxdBuffer[649] = '.';
	nextion.TxdBuffer[650] = 'v';
	nextion.TxdBuffer[651] = 'a';
	nextion.TxdBuffer[652] = 'l';
	nextion.TxdBuffer[653] = '=';
	nextion.TxdBuffer[654] = '0';
	nextion.TxdBuffer[655] = '0';	
	nextion.TxdBuffer[656] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[657] = 0xFF;
	nextion.TxdBuffer[658] = 0xFF;
	nextion.TxdBuffer[659] = 0xFF;

	// Калибровка датчика давления накопителя воды: P max = U max, В/100
	nextion.TxdBuffer[660] = 'x';
	nextion.TxdBuffer[661] = '5';
	nextion.TxdBuffer[662] = '0';
	nextion.TxdBuffer[663] = '3';
	nextion.TxdBuffer[664] = '.';
	nextion.TxdBuffer[665] = 'v';
	nextion.TxdBuffer[666] = 'a';
	nextion.TxdBuffer[667] = 'l';
	nextion.TxdBuffer[668] = '=';
	nextion.TxdBuffer[669] = '0';
	nextion.TxdBuffer[670] = '0';	
	nextion.TxdBuffer[671] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[672] = 0xFF;
	nextion.TxdBuffer[673] = 0xFF;
	nextion.TxdBuffer[674] = 0xFF;

// Текущий уровень воды в источнике, в вольтах/100 датч. давл.
	nextion.TxdBuffer[675] = 'x';
	nextion.TxdBuffer[676] = '5';
	nextion.TxdBuffer[677] = '0';
	nextion.TxdBuffer[678] = '4';
	nextion.TxdBuffer[679] = '.';
	nextion.TxdBuffer[680] = 'v';
	nextion.TxdBuffer[681] = 'a';
	nextion.TxdBuffer[682] = 'l';
	nextion.TxdBuffer[683] = '=';
	nextion.TxdBuffer[684] = '0';
	nextion.TxdBuffer[685] = '0';	
	nextion.TxdBuffer[686] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[687] = 0xFF;
	nextion.TxdBuffer[688] = 0xFF;
	nextion.TxdBuffer[689] = 0xFF;

	// Текущий уровень воды в накопителе, в вольтах/100 датч. давл.
	nextion.TxdBuffer[690] = 'x';
	nextion.TxdBuffer[691] = '5';
	nextion.TxdBuffer[692] = '0';
	nextion.TxdBuffer[693] = '5';
	nextion.TxdBuffer[694] = '.';
	nextion.TxdBuffer[695] = 'v';
	nextion.TxdBuffer[696] = 'a';
	nextion.TxdBuffer[697] = 'l';
	nextion.TxdBuffer[698] = '=';
	nextion.TxdBuffer[699] = '0';
	nextion.TxdBuffer[700] = '0';	
	nextion.TxdBuffer[701] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[702] = 0xFF;
	nextion.TxdBuffer[703] = 0xFF;
	nextion.TxdBuffer[704] = 0xFF;
	}

	// Дополнительные команды
	{
	// Сокрытие/отрисовка сообщения "сухой ход"
	nextion.TxdBuffer[705] = 't';
	nextion.TxdBuffer[706] = '9';
	nextion.TxdBuffer[707] = '.';
	nextion.TxdBuffer[708] = 'p';
	nextion.TxdBuffer[709] = 'c';
	nextion.TxdBuffer[710] = 'o';
	nextion.TxdBuffer[711] = '=';
	nextion.TxdBuffer[712] = '0';
	nextion.TxdBuffer[713] = '0';
	nextion.TxdBuffer[714] = '0';
	nextion.TxdBuffer[715] = '0';
	nextion.TxdBuffer[716] = '0';
	// Терминатор команды
	nextion.TxdBuffer[717] = 0xFF;
	nextion.TxdBuffer[718] = 0xFF;
	nextion.TxdBuffer[719] = 0xFF;
		
	// Яркость дисплея
	nextion.TxdBuffer[720] = 'd';
	nextion.TxdBuffer[721] = 'i';
	nextion.TxdBuffer[722] = 'm';
	nextion.TxdBuffer[723] = '=';
	nextion.TxdBuffer[724] = '1';
	nextion.TxdBuffer[725] = '0';	
	nextion.TxdBuffer[726] = '0';	
	// Терминатор команды
	nextion.TxdBuffer[727] = 0xFF;
	nextion.TxdBuffer[728] = 0xFF;
	nextion.TxdBuffer[729] = 0xFF;
	
	// Скрытие/отрисовка сообщения "Автополив"
	nextion.TxdBuffer[730] = 't';
	nextion.TxdBuffer[731] = '1';
	nextion.TxdBuffer[732] = '7';
	nextion.TxdBuffer[733] = '.';
	nextion.TxdBuffer[734] = 'p';
	nextion.TxdBuffer[735] = 'c';
	nextion.TxdBuffer[736] = 'o';
	nextion.TxdBuffer[737] = '=';
	nextion.TxdBuffer[738] = '0';
	nextion.TxdBuffer[739] = '0';
	nextion.TxdBuffer[740] = '0';
	nextion.TxdBuffer[741] = '0';
	nextion.TxdBuffer[742] = '0';
	// Терминатор команды
	nextion.TxdBuffer[743] = 0xFF;
	nextion.TxdBuffer[744] = 0xFF;
	nextion.TxdBuffer[745] = 0xFF;

	// Скрытие/отрисовка сообщения "Автоподкачка"
	nextion.TxdBuffer[746] = 't';
	nextion.TxdBuffer[747] = '1';
	nextion.TxdBuffer[748] = '8';
	nextion.TxdBuffer[749] = '.';
	nextion.TxdBuffer[750] = 'p';
	nextion.TxdBuffer[751] = 'c';
	nextion.TxdBuffer[752] = 'o';
	nextion.TxdBuffer[753] = '=';
	nextion.TxdBuffer[754] = '0';
	nextion.TxdBuffer[755] = '0';
	nextion.TxdBuffer[756] = '0';
	nextion.TxdBuffer[757] = '0';
	nextion.TxdBuffer[758] = '0';
	// Терминатор команды
	nextion.TxdBuffer[759] = 0xFF;
	nextion.TxdBuffer[760] = 0xFF;
	nextion.TxdBuffer[761] = 0xFF;

	// �?зменение цвета кнопки включения насоса
	nextion.TxdBuffer[762] = 'b';
	nextion.TxdBuffer[763] = '0';
	nextion.TxdBuffer[764] = '.';
	nextion.TxdBuffer[765] = 'b';
	nextion.TxdBuffer[766] = 'c';
	nextion.TxdBuffer[767] = 'o';
	nextion.TxdBuffer[768] = '=';
	nextion.TxdBuffer[769] = '6';
	nextion.TxdBuffer[770] = '4';
	nextion.TxdBuffer[771] = '5';
	nextion.TxdBuffer[772] = '1';
	nextion.TxdBuffer[773] = '2';
	// Терминатор команды
	nextion.TxdBuffer[774] = 0xFF;
	nextion.TxdBuffer[775] = 0xFF;
	nextion.TxdBuffer[776] = 0xFF;
	
		// Дополнение к экрану 0
		{
		// Время включения насоса в последнем цикле, час
		nextion.TxdBuffer[777] = 'n';
		nextion.TxdBuffer[778] = '7';
		nextion.TxdBuffer[779] = '.';
		nextion.TxdBuffer[780] = 'v';
		nextion.TxdBuffer[781] = 'a';
		nextion.TxdBuffer[782] = 'l';
		nextion.TxdBuffer[783] = '=';
		nextion.TxdBuffer[784] = '0';
		nextion.TxdBuffer[785] = '0';
		// Терминатор команды
		nextion.TxdBuffer[786] = 0xFF;
		nextion.TxdBuffer[787] = 0xFF;
		nextion.TxdBuffer[788] = 0xFF;
			
		// Время включения насоса в последнем цикле, мин
		nextion.TxdBuffer[789] = 'n';
		nextion.TxdBuffer[790] = '8';
		nextion.TxdBuffer[791] = '.';
		nextion.TxdBuffer[792] = 'v';
		nextion.TxdBuffer[793] = 'a';
		nextion.TxdBuffer[794] = 'l';
		nextion.TxdBuffer[795] = '=';
		nextion.TxdBuffer[796] = '0';
		nextion.TxdBuffer[797] = '0';	
		// Терминатор команды
		nextion.TxdBuffer[798] = 0xFF;
		nextion.TxdBuffer[799] = 0xFF;
		nextion.TxdBuffer[800] = 0xFF;

		// Время включения насоса в последнем цикле, сек
		nextion.TxdBuffer[801] = 'n';
		nextion.TxdBuffer[802] = '9';
		nextion.TxdBuffer[803] = '.';
		nextion.TxdBuffer[804] = 'v';
		nextion.TxdBuffer[805] = 'a';
		nextion.TxdBuffer[806] = 'l';
		nextion.TxdBuffer[807] = '=';
		nextion.TxdBuffer[808] = '0';
		nextion.TxdBuffer[809] = '0';	
		// Терминатор команды
		nextion.TxdBuffer[810] = 0xFF;
		nextion.TxdBuffer[811] = 0xFF;
		nextion.TxdBuffer[812] = 0xFF;
		}
	}
}


// Отрисовка на Nextion текущего значения параметров
void Prepare_params_and_send_to_nextion(RTC_HandleTypeDef  * hrtc, E2pDataTypeDef * e2p, NextionComPortDataTypeDef * nextion)
{
	HAL_StatusTypeDef			HAL_func_res;
	uint8_t								ascii_buf[5];
	int32_t								temp_int32;
	
	// Preventing corruption of sending data
	if(nextion->Com->TxdPacketIsSent == 0) return;

	// Страница 0 (Главный экран)
	{
	// Значение давления воды в системе, атм * 10
	Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->average_water_pressure_value, ascii_buf, sizeof(ascii_buf));
	nextion->TxdBuffer[7] = ascii_buf[2];
	nextion->TxdBuffer[8] = ascii_buf[1];
	nextion->TxdBuffer[9] = ascii_buf[0];

	// Время работы насоса в последнем цикле, час
	Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->pump_working_time_at_last_cycle / 3600), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[20] = ascii_buf[1];
	nextion->TxdBuffer[21] = ascii_buf[0];
	
	// Время работы насоса в последнем цикле, мин
	Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->pump_working_time_at_last_cycle % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[32] = ascii_buf[1];
	nextion->TxdBuffer[33] = ascii_buf[0];

	// Время работы насоса в последнем цикле, сек
	Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->pump_working_time_at_last_cycle % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[44] = ascii_buf[1];
	nextion->TxdBuffer[45] = ascii_buf[0];
	
	// Кол-во воды, перекачанной насосом в одном цикле, л * 10 (старшие 3 разряда)
	Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->pumped_water_quantity_at_last_cycle * 10 / 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[56] = ascii_buf[2];
	nextion->TxdBuffer[57] = ascii_buf[1];
	nextion->TxdBuffer[58] = ascii_buf[0];

	// Кол-во воды, перекачанной насосом в одном цикле, л * 10 (младшие 3 разряда)
	Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->pumped_water_quantity_at_last_cycle * 10 % 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[59] = ascii_buf[2];
	nextion->TxdBuffer[60] = ascii_buf[1];
	nextion->TxdBuffer[61] = ascii_buf[0];
	
	// t воды при перекачивании, 'С * 10
	Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->LastPumpCycle->water_temp_while_pumped)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[72] = ascii_buf[2];
	nextion->TxdBuffer[73] = ascii_buf[1];
	nextion->TxdBuffer[74] = ascii_buf[0];	

	// текущая t воды, 'С * 10
	Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->LastPumpCycle->current_water_temp)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[85] = ascii_buf[2];
	nextion->TxdBuffer[86] = ascii_buf[1];
	nextion->TxdBuffer[87] = ascii_buf[0];	

	// Чтение времени
	e2p->Statistics->TimeInSeconds = Get_time_in_sec(hrtc);
	// Текущее время, часы
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->TimeInSeconds / 3600), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[98] = ascii_buf[1];
	nextion->TxdBuffer[99] = ascii_buf[0];	

	// Текущее время, минуты
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TimeInSeconds % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[110] = ascii_buf[1];
	nextion->TxdBuffer[111] = ascii_buf[0];

	// Текущее время, секунды
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TimeInSeconds % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[122] = ascii_buf[1];
	nextion->TxdBuffer[123] = ascii_buf[0];
	
	// t воды в источнике, 'С
	Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->LastPumpCycle->well_water_temp)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[134] = ascii_buf[2];
	nextion->TxdBuffer[135] = ascii_buf[1];
	nextion->TxdBuffer[136] = ascii_buf[0];

	// Объём воды в источнике, проценты
	Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->WellWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[147] = ascii_buf[2];
	nextion->TxdBuffer[148] = ascii_buf[1];
	nextion->TxdBuffer[149] = ascii_buf[0];
	// Графический уровень воды в источнике, проценты
	nextion->TxdBuffer[160] = ascii_buf[2];
	nextion->TxdBuffer[161] = ascii_buf[1];
	nextion->TxdBuffer[162] = ascii_buf[0];

	// t воды в накопителе, 'С
	Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->LastPumpCycle->tank_water_temp)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[173] = ascii_buf[2];
	nextion->TxdBuffer[174] = ascii_buf[1];
	nextion->TxdBuffer[175] = ascii_buf[0];	

	// Объём воды в накопителе, проценты
	Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->TankWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[186] = ascii_buf[2];
	nextion->TxdBuffer[187] = ascii_buf[1];
	nextion->TxdBuffer[188] = ascii_buf[0];
	// Графический уровень воды в накопителе, проценты
	nextion->TxdBuffer[199] = ascii_buf[2];
	nextion->TxdBuffer[200] = ascii_buf[1];
	nextion->TxdBuffer[201] = ascii_buf[0];
	}	

	// Страница 1 (настройки 1)
	{
	// Давление включения насоса, атм * 10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PumpOnPressureValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[214] = ascii_buf[2];
	nextion->TxdBuffer[215] = ascii_buf[1];
	nextion->TxdBuffer[216] = ascii_buf[0];

	// Давление выключения насоса, атм * 10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PumpOffPressureValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[229] = ascii_buf[2];
	nextion->TxdBuffer[230] = ascii_buf[1];
	nextion->TxdBuffer[231] = ascii_buf[0];

	// Калибровка датчика давления: P min, атм * 10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMinPressureValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[244] = ascii_buf[2];
	nextion->TxdBuffer[245] = ascii_buf[1];
	nextion->TxdBuffer[246] = ascii_buf[0];

	// Калибровка датчика давления: P max, атм * 10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMaxPressureValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[259] = ascii_buf[2];
	nextion->TxdBuffer[260] = ascii_buf[1];
	nextion->TxdBuffer[261] = ascii_buf[0];

	// Напряжение (0 - 5В), соотв. минимальному давлению датчика давления, вольт/10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMinPressureVoltageValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[274] = ascii_buf[2];
	nextion->TxdBuffer[275] = ascii_buf[1];
	nextion->TxdBuffer[276] = ascii_buf[0];

	// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления, вольт/10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMaxPressureVoltageValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[289] = ascii_buf[2];
	nextion->TxdBuffer[290] = ascii_buf[1];
	nextion->TxdBuffer[291] = ascii_buf[0];
	}

	// Страница 2 (статистика)
	{
	// Общее время работы контроллера, часов (старшие 8-6 разряды)
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalControllerWorkingTime / 3600) / 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[304] = ascii_buf[2];
	nextion->TxdBuffer[305] = ascii_buf[1];
	nextion->TxdBuffer[306] = ascii_buf[0];
	// Общее время работы контроллера, часов (средние 5-3 разряды)
	Hex2Dec2ASCII((uint16_t) (((e2p->Statistics->TotalControllerWorkingTime / 3600) % 10000) % 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[307] = ascii_buf[2];
	nextion->TxdBuffer[308] = ascii_buf[1];
	nextion->TxdBuffer[309] = ascii_buf[0];
	// Общее время работы контроллера, минут (младшие 2-1 разряды)
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalControllerWorkingTime / 60) % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[310] = ascii_buf[1];
	nextion->TxdBuffer[311] = ascii_buf[0];
	
	// Общее время работы насоса, часов (старшие 4 разряда)
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalPumpWorkingTime / 3600) / 100), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[324] = ascii_buf[3];
	nextion->TxdBuffer[325] = ascii_buf[2];
	nextion->TxdBuffer[326] = ascii_buf[1];
	nextion->TxdBuffer[327] = ascii_buf[0];
	// Общее время работы насоса, минут (младшие 2 разряда)
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalPumpWorkingTime / 60) % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[328] = ascii_buf[1];
	nextion->TxdBuffer[329] = ascii_buf[0];

	// Общее кол-во воды, перекачанной насосом, кубометры  (старшие 7-4 разряды)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->TotalPumpedWaterQuantity / 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[342] = ascii_buf[3];
	nextion->TxdBuffer[343] = ascii_buf[2];
	nextion->TxdBuffer[344] = ascii_buf[1];
	nextion->TxdBuffer[345] = ascii_buf[0];
	// Общее кол-во воды, перекачанной насосом, сотые кубометров  (младшие 3-1 разряды)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->TotalPumpedWaterQuantity % 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[346] = ascii_buf[2];
	nextion->TxdBuffer[347] = ascii_buf[1];
	nextion->TxdBuffer[348] = ascii_buf[0];

	// Общее кол-во воды, перекачанной за сутки, кубометры  (старшие 5-3 разряды)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityToday / 100), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[361] = ascii_buf[2];
	nextion->TxdBuffer[362] = ascii_buf[1];
	nextion->TxdBuffer[363] = ascii_buf[0];
	// Общее кол-во воды, перекачанной за сутки, сотые кубометров  (младшие 2 разряда)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityToday % 100), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[364] = ascii_buf[1];
	nextion->TxdBuffer[365] = ascii_buf[0];

	// Общее кол-во воды, перекачанной за неделю, кубометры  (старшие 3 разряда)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityLastWeek / 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[378] = ascii_buf[2];
	nextion->TxdBuffer[379] = ascii_buf[1];
	nextion->TxdBuffer[380] = ascii_buf[0];
	// Общее кол-во воды, перекачанной за неделю, сотые кубометров  (младшие 3 разряда)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityLastWeek % 1000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[381] = ascii_buf[2];
	nextion->TxdBuffer[382] = ascii_buf[1];
	nextion->TxdBuffer[383] = ascii_buf[0];

	// Минимальная суточная t воды в источнике, 'С * 10
	Hex2Dec2ASCII((uint16_t) (fabs((float)e2p->LastPumpCycle->well_water_temp_min_for_24h)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[396] = ascii_buf[2];
	nextion->TxdBuffer[397] = ascii_buf[1];
	nextion->TxdBuffer[398] = ascii_buf[0];	

	// Максимальная суточная t воды в источнике, 'С * 10
	Hex2Dec2ASCII((uint16_t) (fabs((float)e2p->LastPumpCycle->well_water_temp_max_for_24h)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[411] = ascii_buf[2];
	nextion->TxdBuffer[412] = ascii_buf[1];
	nextion->TxdBuffer[413] = ascii_buf[0];	

	// Минимальная суточная t воды в накопителе, 'С * 10
	Hex2Dec2ASCII((uint16_t) (fabs((float)e2p->LastPumpCycle->tank_water_temp_min_for_24h)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[426] = ascii_buf[2];
	nextion->TxdBuffer[427] = ascii_buf[1];
	nextion->TxdBuffer[428] = ascii_buf[0];	

	// Максимальная суточная t воды в накопителе, 'С * 10
	Hex2Dec2ASCII((uint16_t) (fabs((float)e2p->LastPumpCycle->tank_water_temp_max_for_24h)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[441] = ascii_buf[2];
	nextion->TxdBuffer[442] = ascii_buf[1];
	nextion->TxdBuffer[443] = ascii_buf[0];	
	}

	// Страница 3 (полив)
	{
	// Отображение номера выхода 1-8
	Hex2Dec2ASCII((uint16_t) e2p->WateringControls->CurrWateringOutputNumber, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[456] = ascii_buf[0];

	// Значение смещения времени включения полива зоны 1-8 относительно начала суток, час
	if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			temp_int32 = e2p->WateringControls->out1_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 2) temp_int32 = e2p->WateringControls->out2_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 3) temp_int32 = e2p->WateringControls->out3_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 4) temp_int32 = e2p->WateringControls->out4_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 5) temp_int32 = e2p->WateringControls->out5_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 6) temp_int32 = e2p->WateringControls->out6_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 7) temp_int32 = e2p->WateringControls->out7_zero_clock_time_delta;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 8) temp_int32 = e2p->WateringControls->out8_zero_clock_time_delta;
	Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[469] = ascii_buf[1];
	nextion->TxdBuffer[470] = ascii_buf[0];	
	// Значение смещения времени включения полива зоны 1-8 относительно начала суток, мин
	Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[471] = ascii_buf[1];
	nextion->TxdBuffer[472] = ascii_buf[0];

	// Значение времени работы полива зоны 1-8, час
	if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			temp_int32 = e2p->WateringControls->out1_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 2) temp_int32 = e2p->WateringControls->out2_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 3) temp_int32 = e2p->WateringControls->out3_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 4) temp_int32 = e2p->WateringControls->out4_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 5) temp_int32 = e2p->WateringControls->out5_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 6) temp_int32 = e2p->WateringControls->out6_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 7) temp_int32 = e2p->WateringControls->out7_working_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 8) temp_int32 = e2p->WateringControls->out8_working_time;
	Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[485] = ascii_buf[1];
	nextion->TxdBuffer[486] = ascii_buf[0];	
	// Значение времени работы полива зоны 1-8, мин
	Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[487] = ascii_buf[1];
	nextion->TxdBuffer[488] = ascii_buf[0];

	// �?нтервал времени между включениями полива зоны 1-8, час
	if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			temp_int32 = e2p->WateringControls->out1_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 2) temp_int32 = e2p->WateringControls->out2_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 3) temp_int32 = e2p->WateringControls->out3_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 4) temp_int32 = e2p->WateringControls->out4_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 5) temp_int32 = e2p->WateringControls->out5_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 6) temp_int32 = e2p->WateringControls->out6_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 7) temp_int32 = e2p->WateringControls->out7_interval_time;
	else if (e2p->WateringControls->CurrWateringOutputNumber == 8) temp_int32 = e2p->WateringControls->out8_interval_time;
	Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[501] = ascii_buf[1];
	nextion->TxdBuffer[502] = ascii_buf[0];	
	// �?нтервал времени между включениями полива зоны 1-8, мин
	Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[503] = ascii_buf[1];
	nextion->TxdBuffer[504] = ascii_buf[0];
	}

	// Страница 4 (настройки 2)
	{
	// Значение счётчика расхода воды, кубометры (десятки тысяч - десятки)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->WaterCounterValue / 10000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[517] = ascii_buf[3];
	nextion->TxdBuffer[518] = ascii_buf[2];
	nextion->TxdBuffer[519] = ascii_buf[1];
	nextion->TxdBuffer[520] = ascii_buf[0];
	// Значение счётчика расхода воды, единицы, десятые, сотые, тысячные кубометров (литры)
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->WaterCounterValue % 10000), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[521] = ascii_buf[3];
	nextion->TxdBuffer[522] = ascii_buf[2];
	nextion->TxdBuffer[523] = ascii_buf[1];
	nextion->TxdBuffer[524] = ascii_buf[0];

	// Ежесуточная автоподкачка: 
	// Значение смещения времени включения автоподкачивания относительно начала суток, час
	Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->auto_pump_zero_clock_time_delta / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[537] = ascii_buf[1];
	nextion->TxdBuffer[538] = ascii_buf[0];	
	// Значение смещения времени включения автоподкачивания относительно начала суток, мин
	Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->auto_pump_zero_clock_time_delta % 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[539] = ascii_buf[1];
	nextion->TxdBuffer[540] = ascii_buf[0];
	// Объём подкачиваемой воды, л
	Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->auto_pump_quantity * 10, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[553] = ascii_buf[3];
	nextion->TxdBuffer[554] = ascii_buf[2];
	nextion->TxdBuffer[555] = ascii_buf[1];
	nextion->TxdBuffer[556] = ascii_buf[0];

	// Чтение времени
	e2p->Statistics->TimeInSeconds = Get_time_in_sec(hrtc);
	// Установка времени, часы
	Hex2Dec2ASCII((uint16_t) (e2p->Statistics->TimeInSeconds / 3600), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[569] = ascii_buf[1];
	nextion->TxdBuffer[570] = ascii_buf[0];	
	// Установка времени, минуты
	Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TimeInSeconds % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[583] = ascii_buf[1];
	nextion->TxdBuffer[584] = ascii_buf[0];
	
	// Коррекция времени, сек/сутки
	Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->Calibrations->TimeCorrectionValue)), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[597] = ascii_buf[1];
	nextion->TxdBuffer[598] = ascii_buf[0];

	// Отображение знака значения коррекции времени
	if (e2p->Calibrations->TimeCorrectionValue < 0)
	{
		// Рисуем минус "-"
		nextion->TxdBuffer[611] = '1';
	}
	else
	{
		// Прячем минус "-"
		nextion->TxdBuffer[611] = '0';	
	}
	}

	// Страница 5 (настройки 3)
	{
	// Калибровка датчика давления источника воды: P min = U min, В/10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->SourcePsensorMinPressureVoltageValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[624] = ascii_buf[2];
	nextion->TxdBuffer[625] = ascii_buf[1];
	nextion->TxdBuffer[626] = ascii_buf[0];
	// Калибровка датчика давления источника воды: P max = U max, В/10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->SourcePsensorMaxPressureVoltageValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[639] = ascii_buf[2];
	nextion->TxdBuffer[640] = ascii_buf[1];
	nextion->TxdBuffer[641] = ascii_buf[0];

	// Калибровка датчика давления накопителя воды: P min = U min, В/10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->TankPsensorMinPressureVoltageValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[654] = ascii_buf[2];
	nextion->TxdBuffer[655] = ascii_buf[1];
	nextion->TxdBuffer[656] = ascii_buf[0];
	// Калибровка датчика давления накопителя воды: P max = U max, В/10
	Hex2Dec2ASCII((uint16_t) e2p->Calibrations->TankPsensorMaxPressureVoltageValue, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[669] = ascii_buf[2];
	nextion->TxdBuffer[670] = ascii_buf[1];
	nextion->TxdBuffer[671] = ascii_buf[0];

	// Текущий уровень воды в источнике, в вольтах/10 датч. давл.
	Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->WellWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[684] = ascii_buf[2];
	nextion->TxdBuffer[685] = ascii_buf[1];
	nextion->TxdBuffer[686] = ascii_buf[0];
	// Текущий уровень воды в накопителе, в вольтах/10 датч. давл.
	Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->TankWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[699] = ascii_buf[2];
	nextion->TxdBuffer[700] = ascii_buf[1];
	nextion->TxdBuffer[701] = ascii_buf[0];
	}
	

	// Дополнительные команды
	{
	// Скрытие/отрисовка сообщения "сухой ход"
	if (e2p->LastPumpCycle->dry_work_detected)
	{
		// Отрисовка сообщения "сухой ход"
		nextion->TxdBuffer[712] = '6';
		nextion->TxdBuffer[713] = '3';
		nextion->TxdBuffer[714] = '5';
		nextion->TxdBuffer[715] = '5';
		nextion->TxdBuffer[716] = '6';
	}
	else
	{
		// Сокрытие сообщения "сухой ход"
		nextion->TxdBuffer[712] = '0';
		nextion->TxdBuffer[713] = '0';
		nextion->TxdBuffer[714] = '0';
		nextion->TxdBuffer[715] = '0';
		nextion->TxdBuffer[716] = '0';
	}

	// Яркость дисплея
	Hex2Dec2ASCII((uint16_t) (display_brightness / DISPLAY_BRIGHTNESS_OFF_SPEED), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[724] = ascii_buf[2];
	nextion->TxdBuffer[725] = ascii_buf[1];
	nextion->TxdBuffer[726] = ascii_buf[0];	

	// Скрытие/отрисовка сообщения "Автополив"
	if (e2p->WateringControls->AutoWatering)
	{
		// Отрисовка сообщения "Автополив"
		nextion->TxdBuffer[738] = '3';
		nextion->TxdBuffer[739] = '4';
		nextion->TxdBuffer[740] = '7';
		nextion->TxdBuffer[741] = '8';
		nextion->TxdBuffer[742] = '4';
	}
	else
	{
		// Скрытие сообщения "Автополив"
		nextion->TxdBuffer[738] = '0';
		nextion->TxdBuffer[739] = '0';
		nextion->TxdBuffer[740] = '0';
		nextion->TxdBuffer[741] = '0';
		nextion->TxdBuffer[742] = '0';
	}

	// Сокрытие/отрисовка сообщения "Автоподкачка"
	if (e2p->LastPumpCycle->auto_pump_is_started)
	{
		// Отрисовка сообщения "Автоподкачка"
		nextion->TxdBuffer[754] = '1';
		nextion->TxdBuffer[755] = '3';
		nextion->TxdBuffer[756] = '6';
		nextion->TxdBuffer[757] = '2';
		nextion->TxdBuffer[758] = '9';
	}
	else
	{
		// Скрытие сообщения "Автоподкачка"
		nextion->TxdBuffer[754] = '0';
		nextion->TxdBuffer[755] = '0';
		nextion->TxdBuffer[756] = '0';
		nextion->TxdBuffer[757] = '0';
		nextion->TxdBuffer[758] = '0';
	}

	// Если насос включен, то кнопка вкл. насоса имеет салатовый цвет
	if (e2p->LastPumpCycle->pump_is_started)
	{
		nextion->TxdBuffer[769] = '3';
		nextion->TxdBuffer[770] = '4';
		nextion->TxdBuffer[771] = '7';
		nextion->TxdBuffer[772] = '8';
		nextion->TxdBuffer[773] = '4';
	}
	else
	{
		// Если насос выключен, то кнопка вкл. насоса имеет оранжевый цвет
		nextion->TxdBuffer[769] = '6';
		nextion->TxdBuffer[770] = '4';
		nextion->TxdBuffer[771] = '5';
		nextion->TxdBuffer[772] = '1';
		nextion->TxdBuffer[773] = '2';
	}

		// Дополнение к экрану 0
		{
		// Время включения насоса в последнем цикле, час
		Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->pump_start_time_at_last_cycle / 3600), ascii_buf, sizeof(ascii_buf));	
		nextion->TxdBuffer[784] = ascii_buf[1];
		nextion->TxdBuffer[785] = ascii_buf[0];
		
		// Время включения насоса в последнем цикле, мин
		Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->pump_start_time_at_last_cycle % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
		nextion->TxdBuffer[796] = ascii_buf[1];
		nextion->TxdBuffer[797] = ascii_buf[0];

		// Время включения насоса в последнем цикле, сек
		Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->pump_start_time_at_last_cycle % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
		nextion->TxdBuffer[808] = ascii_buf[1];
		nextion->TxdBuffer[809] = ascii_buf[0];
		}
	}	
	
	// Отправка на дисплей по привязанному в структуре Com порту
	if(nextion->ComLink == COM2)
	{	
		com2.TxdPacketIsSent = 0;
		HAL_func_res = HAL_UART_Transmit_DMA(&huart2, nextion->TxdBuffer, STRING_LENGHT_TO_NEXTION);
	}
	else if(nextion->ComLink == COM4)
	{	
		com4.TxdPacketIsSent = 0;
		HAL_func_res = HAL_UART_Transmit_DMA(&huart4, nextion->TxdBuffer, STRING_LENGHT_TO_NEXTION);
	}
}


// Обработчик принятого пакета по COM2 из дисплея Nextion
void Nextion_received_data_handler(RTC_HandleTypeDef  * hrtc, E2pDataTypeDef * e2p)
{
	ReturnCode func_res;
	
	// Проверка целостности и к.с. принятой по com строки данных
	func_res = Check_received_nextion_packet(nextion.RxdBuffer, com2.RxdPacketLenght8);
	if (func_res == OK)
	{		
		// После контроля правильности и выполнения отключаем индикатор приёма данных
		LED1_OFF;

		// Счётчик правильно принятых пакетов данных
		com2.RxdGoodPacketsCounter++;
			
		// Разбор принятой строки от дисплея Nextion
		Parsing_nextion_display_string(hrtc, e2p, nextion.RxdBuffer, com2.RxdPacketLenght8, com2.RxdPacketIsReceived);
	}	

	else com2.RxdPacketsErrorCounter++;
}


// Проверка принятой по com2 строки данных
ReturnCode Check_received_nextion_packet(uint8_t * buf, uint16_t lenght)
{
	uint16_t		idx;
	
	if (lenght != STRING_LENGHT_FROM_NEXTION)
	{
		return StringLengthError;
	}
	
	idx=lenght-3;	
	if (buf[idx++] != 0xFF)
	{
		return StringTerminationError;
	}

	if (buf[idx++] != 0xFF)
	{
		return StringTerminationError;
	}

	if (buf[idx] != 0xFF)
	{
		return StringTerminationError;
	}
	
	return OK;
}


// Get received command from Jetson
uint32_t Get_jetson_command(JetsonComPortDataTypeDef * jetson)
{
	uint8_t idx = 1;
	uint32_t command;

	memcpy(&command, jetson->RxdBuffer + idx, sizeof(command));
	// Swapping order of 4 bytes
	Buffer_bytes_swap((uint8_t *) &command, sizeof(uint32_t));
	
	return command;
}


// Handles data from JCB
ReturnCode Jetson_rxd_handler(CRC_HandleTypeDef * hcrc, JetsonComPortDataTypeDef * jetson)
{
	ReturnCode	func_res;
	uint32_t		command;

	// Check length, checksum calculation of ascii string from Jetson
	//func_res = String_from_jetson_checking(jetson);
	
	if(func_res != OK) return func_res;
	
	// Get received command from Jetson
	command = Get_jetson_command(jetson);
	
	// Going to start the bootloader
	//if(command == FromJetsonMcuBootloaderStart)
	{
		// Start MCU Firmware Update procedure
		//Bootloader_start();	// Confirmation of starting is being done inside bootloader
	}

	return OK;
}


// Handles data from External interface
ReturnCode External_if_rxd_handler(CRC_HandleTypeDef * hcrc, NextionComPortDataTypeDef * nextion)
{
	uint8_t 	temp8;
	uint16_t 	temp16;	
	uint32_t 	idx32;	

	// Check integrity of received message
	idx32 = nextion->Com->RxdPacketLenght8 - 2;
	if(nextion->Com->RxdPacketLenght8 != nextion->RxdBuffer[idx32])
	{
		return StringLengthError;
	}
	
	temp8 = 0;
	idx32 = 0;
	for(idx32 = 0; idx32 < nextion->Com->RxdPacketLenght8 - 1; idx32++)
	{
		// Getting XOR checksum
		temp8 ^= nextion->RxdBuffer[idx32];
	}
	
	// Compare calculated checksum of received message with written one inside received message
	if(temp8 != nextion->RxdBuffer[idx32])
	{
		return CheckSumError;
	}

	return OK;
}


// Handles data from Com ports
ReturnCode Com_rxd_handler(CRC_HandleTypeDef * hcrc, ComNum ComNumber, JetsonComPortDataTypeDef * jetson, NextionComPortDataTypeDef * nextion)
{
	ReturnCode func_stat;

	switch(ComNumber)
	{
		case COM1:
		{
			if(jetson->ComLink == COM1)
			{
				// Handles data from JCB
				func_stat = Jetson_rxd_handler(hcrc, jetson);
			}

			else if(nextion->ComLink == COM1)
			{
				// Handles data from External interface
				func_stat = External_if_rxd_handler(hcrc, nextion);
			}
			
			break;
		}

		case COM2:
		{
			if(jetson->ComLink == COM2)
			{
				// Handles data from JCB
				func_stat = Jetson_rxd_handler(hcrc, jetson);
			}

			else if(nextion->ComLink == COM2)
			{
				// Handles data from External interface
				func_stat = External_if_rxd_handler(hcrc, nextion);
			}
			
			break;
		}

		case COM3:
		{
			
			break;
		}

		case COM4:
		{
			
			break;
		}

		case COM5:
		{
			
			break;
		}
	}
	
	return func_stat;
}



// Управление насосом
void PumpOn_off(E2pDataTypeDef * e2p)
{
	static uint8_t	pump_start_trigger = 0;
	static uint8_t	pump_on_by_pressure_delay_timer_is_set = 0;
	static uint8_t	pump_off_by_pressure_delay_timer_is_set = 0;
	static uint32_t	auto_pump_counter_start_point = 0;
	static int32_t	time_in_seconds_prev = 0;
	static uint32_t	pump_on_by_pressure_delay_timer = 0;
	static uint32_t	pump_off_by_pressure_delay_timer = 0;
	
	// Включение по автоподкачке*****************************************************************************
	// Проверка на необходимость включения/выключения насоса по наличию какого-либо кол-ва литров для накачки
	if (e2p->LastPumpCycle->auto_pump_quantity > 0)
	{
		// Если в текущих сутках ещё не производилась автоподкачка
		if (e2p->LastPumpCycle->auto_pump_is_done == 0)
		{
			// Проверка на необходимость включения насоса по времени
			if (e2p->Statistics->TimeInSeconds / 60 == e2p->LastPumpCycle->auto_pump_zero_clock_time_delta)
			{
				// Если автоналив не активен
				if (e2p->LastPumpCycle->auto_pump_is_started == 0)
				{
					// Фиксируем начальную точку счётчика перекачанных литров
					auto_pump_counter_start_point = e2p->Statistics->TotalPumpedWaterQuantity;
					
					// Обнуление счётчиков значений последнего цикла
					e2p->LastPumpCycle->pump_working_time_at_last_cycle = 0;
					e2p->LastPumpCycle->pumped_water_quantity_at_last_cycle = 0;
					
					// Включаем нанос
					e2p->LastPumpCycle->switch_pump_on = 1;
					e2p->LastPumpCycle->auto_pump_is_started = 1;
				}			
			}
			
			// Если активна автоподкачка
			if (e2p->LastPumpCycle->auto_pump_is_started)
			{
				// Ожидание завершения автоналива по кол-ву литров
				if (e2p->Statistics->TotalPumpedWaterQuantity >= (auto_pump_counter_start_point + (uint32_t) e2p->LastPumpCycle->auto_pump_quantity))
				{
					// Команда выключения насоса
					e2p->LastPumpCycle->switch_pump_off = 1;
					e2p->LastPumpCycle->auto_pump_is_started = 0;
					// Установка признака выполнения автоподкачки за сутки
					e2p->LastPumpCycle->auto_pump_is_done = 1;
				}
			}
		}
	}
	
	// Проверка смены суток для сброса флагов и счётчиков
	if ((time_in_seconds_prev > 0) && (e2p->Statistics->TimeInSeconds == 0))
	{		
		// Сброс признака выполнения автоподкачки за сутки
		e2p->LastPumpCycle->auto_pump_is_done = 0;
		// сброс события "сухого хода" при смене суток
		e2p->LastPumpCycle->dry_work_detected = 0;
		// Разрешение повторной попытки автоподкачки воды при смене суток
		e2p->LastPumpCycle->auto_pump_is_started = 0;
	}

	time_in_seconds_prev = e2p->Statistics->TimeInSeconds;

	// Включение по давлению************************************************************************
	// Если значение минимального давления для включения насоса стало отлично от нуля, то сравниваем
	if (e2p->Calibrations->PumpOnPressureValue > 0)
	{
		// Если не было обнаружено событие "сухого хода"
		if (e2p->LastPumpCycle->dry_work_detected == 0)
		{
			// Если текущее значение давления воды <= минимального давления датчика давления
			if (e2p->LastPumpCycle->average_water_pressure_value <= e2p->Calibrations->PumpOnPressureValue)
			{
				// Если триггер таймера не установлен
				if (pump_on_by_pressure_delay_timer_is_set == 0)
				{
					pump_on_by_pressure_delay_timer=e2p->Statistics->TotalControllerWorkingTime;
					pump_on_by_pressure_delay_timer_is_set = 1;
				}
				if (pump_on_by_pressure_delay_timer_is_set)
				{
					if (e2p->Statistics->TotalControllerWorkingTime == pump_on_by_pressure_delay_timer + PUMP_ON_OFF_DELAY)
					{
						// Вторичный контроль давления воды <= минимального давления датчика давления для включения
						if (e2p->LastPumpCycle->average_water_pressure_value <= e2p->Calibrations->PumpOnPressureValue)
						{
							// Включаем нанос по давлению
							e2p->LastPumpCycle->switch_pump_on = 1;

							// Обнуление счётчиков значений последнего цикла
							e2p->LastPumpCycle->pump_working_time_at_last_cycle = 0;
							e2p->LastPumpCycle->pumped_water_quantity_at_last_cycle = 0;
						}
					}
				}
			}
		}
	}

	// Выключение по давлению*************************************************************************
	// Если значение максимального давления для выключения насоса стало отлично от нуля, то сравниваем
	if (e2p->Calibrations->PumpOffPressureValue > 0)
	{
		// Если текущее значение давления воды >= максимального значения давления датчика давления для отключения
		if (e2p->LastPumpCycle->average_water_pressure_value >= e2p->Calibrations->PumpOffPressureValue)
		{
			// Если таймер задержки отключения не установлен
			if (pump_off_by_pressure_delay_timer_is_set == 0)
			{
				pump_off_by_pressure_delay_timer=e2p->Statistics->TotalControllerWorkingTime;
				pump_off_by_pressure_delay_timer_is_set = 1;
			}
			if (pump_off_by_pressure_delay_timer_is_set)
			{
				// Отработка задержки выключения
				if (e2p->Statistics->TotalControllerWorkingTime >= pump_off_by_pressure_delay_timer+PUMP_ON_OFF_DELAY)
				{
					pump_off_by_pressure_delay_timer_is_set = 0;
					
					// Вторичный контроль давления воды >= максимального значения давления датчика давления для отключения
					if (e2p->LastPumpCycle->average_water_pressure_value >= e2p->Calibrations->PumpOffPressureValue)
					{
						
						// Если активна автоподкачка, то завершаем по давлению
						if (e2p->LastPumpCycle->auto_pump_is_started)
						{
							e2p->LastPumpCycle->auto_pump_is_started = 0;
							// Установка признака выполнения автоподкачки за сутки
							e2p->LastPumpCycle->auto_pump_is_done = 1;
						}

						// Выключаем нанос
						e2p->LastPumpCycle->switch_pump_off = 1;
						
						pump_on_by_pressure_delay_timer_is_set = 0;
					}
				}
			}
		}
	}

	// Выключение по "сухому ходу"**********
	// Если обнаружено событие "сухого хода"
	if (e2p->LastPumpCycle->dry_work_detected)
	{
		// Выключаем насос
		e2p->LastPumpCycle->switch_pump_off = 1;
		pump_on_by_pressure_delay_timer_is_set = 0;
	}	
	
	// Включение насоса******************
	// Если есть команда включения насоса
	if (e2p->LastPumpCycle->switch_pump_on)
	{
		// Включаем нанос
		e2p->LastPumpCycle->pump_is_started = 1;
		// Если насос ещё не запущен
		if (pump_start_trigger == 0)
		{
			// Если не активна автоподкачка
			//if (e2p->LastPumpCycle->auto_pump_is_started == 0)
			{
				// Обнуление счётчиков значений последнего цикла
				//e2p->LastPumpCycle->pump_working_time_at_last_cycle=0;
				//e2p->LastPumpCycle->pumped_water_quantity_at_last_cycle=0;
			}
			
			pump_start_trigger = 1;
			e2p->LastPumpCycle->switch_pump_on = 0;
			WATER_PUMP_ON;
			
			// Фиксируем время включения насоса
			e2p->LastPumpCycle->pump_start_time_at_last_cycle = e2p->Statistics->TimeInSeconds;
		}
	}

	// Выключение насоса******************
	// Если есть команда выключения насоса
	if (e2p->LastPumpCycle->switch_pump_off)
	{
		// Выключаем насос
		WATER_PUMP_OFF;
		e2p->LastPumpCycle->pump_is_started = 0;
		pump_start_trigger = 0;
		e2p->LastPumpCycle->switch_pump_off = 0;
	}

	// Управление яркостью дисплея для периода, когда насос включен
	if (e2p->LastPumpCycle->pump_is_started == 1)
	{			
		// Яркость можно только повышать
		if (display_brightness <= AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED)
		{
			// Яркость для авторежимов на 30%
			display_brightness = AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED;
		}
		// Таймер можно только повышать
		if (display_brightness_timer < AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY)
		{
			// Перезапись, если яркость уже снижена для авторежима 
			if (display_brightness == AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED)
			{
				// Таймер уменьшения яркости дисплея для авторежимов
				display_brightness_timer = AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY;
			}
		}
	}
}


// Управление автополивом
void Watering_outputs_on_off(E2pDataTypeDef * e2p)
{
	static uint8_t	out1_watering_is_started=0, out2_watering_is_started=0;
	static uint8_t	out3_watering_is_started=0, out4_watering_is_started=0;
	static uint8_t	out5_watering_is_started=0, out6_watering_is_started=0;
	static uint8_t	out7_watering_is_started=0, out8_watering_is_started=0;
	static uint8_t	out1_cycles_counter=0, out2_cycles_counter=0;
	static uint8_t	out3_cycles_counter=0, out4_cycles_counter=0;
	static uint8_t	out5_cycles_counter=0, out6_cycles_counter=0;
	static uint8_t	out7_cycles_counter=0, out8_cycles_counter=0;
	static int32_t	time_in_seconds_prev = 0;

	// Если время работы зоны полива 1 > 0 мин
	if (e2p->WateringControls->out1_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out1_zero_clock_time_delta + e2p->WateringControls->out1_interval_time * out1_cycles_counter\
				+e2p->WateringControls->out1_working_time * out1_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out1_watering_is_started == 0)
			{
				// Включение автополива зоны 1
				WATER_ZONE1_ON;
				out1_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out1_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out1_zero_clock_time_delta + e2p->WateringControls->out1_interval_time * out1_cycles_counter\
					+ e2p->WateringControls->out1_working_time * out1_cycles_counter + e2p->WateringControls->out1_working_time))
			{
				// Выключение автополива зоны 1
				WATER_ZONE1_OFF;
				out1_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out1_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out1_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 1
		WATER_ZONE1_OFF;
		out1_watering_is_started = 0;		
	}

	// Если время работы зоны полива 2 > 0 мин
	if (e2p->WateringControls->out2_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out2_zero_clock_time_delta + e2p->WateringControls->out2_interval_time * out2_cycles_counter\
				+ e2p->WateringControls->out2_working_time * out2_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out2_watering_is_started == 0)
			{
				// Включение автополива зоны 2
				WATER_ZONE2_ON;
				out2_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out2_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out2_zero_clock_time_delta + e2p->WateringControls->out2_interval_time * out2_cycles_counter\
					+ e2p->WateringControls->out2_working_time * out2_cycles_counter + e2p->WateringControls->out2_working_time))
			{
				// Выключение автополива зоны 2
				WATER_ZONE2_OFF;
				out2_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out2_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out2_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 2
		WATER_ZONE2_OFF;
		out2_watering_is_started = 0;		
	}

	// Если время работы зоны полива 3 > 0 мин
	if (e2p->WateringControls->out3_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out3_zero_clock_time_delta + e2p->WateringControls->out3_interval_time * out3_cycles_counter\
				+ e2p->WateringControls->out3_working_time * out3_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out3_watering_is_started == 0)
			{
				// Включение автополива зоны 3
				WATER_ZONE3_ON;
				out3_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out3_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out3_zero_clock_time_delta + e2p->WateringControls->out3_interval_time * out3_cycles_counter\
					+ e2p->WateringControls->out3_working_time * out3_cycles_counter + e2p->WateringControls->out3_working_time))
			{
				// Выключение автополива зоны 3
				WATER_ZONE3_OFF;
				out3_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out3_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out3_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 3
		WATER_ZONE3_OFF;
		out3_watering_is_started = 0;		
	}
	
	// Если время работы зоны полива 4 > 0 мин
	if (e2p->WateringControls->out4_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out4_zero_clock_time_delta + e2p->WateringControls->out4_interval_time * out4_cycles_counter\
				+ e2p->WateringControls->out4_working_time*out4_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out4_watering_is_started == 0)
			{
				// Включение автополива зоны 4
				WATER_ZONE4_ON;
				out4_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out4_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out4_zero_clock_time_delta + e2p->WateringControls->out4_interval_time * out4_cycles_counter\
					+ e2p->WateringControls->out4_working_time * out4_cycles_counter + e2p->WateringControls->out4_working_time))
			{
				// Выключение автополива зоны 4
				WATER_ZONE4_OFF;
				out4_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out4_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out4_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 4
		WATER_ZONE4_OFF;
		out4_watering_is_started = 0;		
	}
	
	// Если время работы зоны полива 5 > 0 мин
	if (e2p->WateringControls->out5_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out5_zero_clock_time_delta + e2p->WateringControls->out5_interval_time * out5_cycles_counter\
				+ e2p->WateringControls->out5_working_time * out5_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out5_watering_is_started == 0)
			{
				// Включение автополива зоны 5
				WATER_ZONE5_ON;
				out5_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out5_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out5_zero_clock_time_delta + e2p->WateringControls->out5_interval_time * out5_cycles_counter\
					+ e2p->WateringControls->out5_working_time * out5_cycles_counter + e2p->WateringControls->out5_working_time))
			{
				// Выключение автополива зоны 5
				WATER_ZONE5_OFF;
				out5_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out5_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out5_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 5
		WATER_ZONE5_OFF;
		out5_watering_is_started = 0;		
	}
	
	// Если время работы зоны полива 6 > 0 мин
	if (e2p->WateringControls->out6_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out6_zero_clock_time_delta + e2p->WateringControls->out6_interval_time * out6_cycles_counter\
				+ e2p->WateringControls->out6_working_time * out6_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out6_watering_is_started == 0)
			{
				// Включение автополива зоны 6
				WATER_ZONE6_ON;
				out6_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out6_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out6_zero_clock_time_delta + e2p->WateringControls->out6_interval_time * out6_cycles_counter\
					+ e2p->WateringControls->out6_working_time * out6_cycles_counter + e2p->WateringControls->out6_working_time))
			{
				// Выключение автополива зоны 6
				WATER_ZONE6_OFF;
				out6_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out6_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out6_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 6
		WATER_ZONE6_OFF;
		out6_watering_is_started = 0;		
	}
	
	
	// Если время работы зоны полива 7 > 0 мин
	if (e2p->WateringControls->out7_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out7_zero_clock_time_delta + e2p->WateringControls->out7_interval_time * out7_cycles_counter\
				+ e2p->WateringControls->out7_working_time * out7_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out7_watering_is_started == 0)
			{
				// Включение автополива зоны 7
				WATER_ZONE7_ON;
				out7_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out7_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out7_zero_clock_time_delta + e2p->WateringControls->out7_interval_time * out7_cycles_counter\
					+ e2p->WateringControls->out7_working_time * out7_cycles_counter + e2p->WateringControls->out7_working_time))
			{
				// Выключение автополива зоны 7
				WATER_ZONE7_OFF;
				out7_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out7_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out7_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 7
		WATER_ZONE7_OFF;
		out7_watering_is_started = 0;		
	}
	
	// Если время работы зоны полива 8 > 0 мин
	if (e2p->WateringControls->out8_working_time > 0)
	{
		// Проверка на необходимость включения автополива по времени
		if (e2p->Statistics->TimeInSeconds / 60 == (e2p->WateringControls->out8_zero_clock_time_delta + e2p->WateringControls->out8_interval_time * out8_cycles_counter\
				+ e2p->WateringControls->out8_working_time * out8_cycles_counter))
		{
			// Если ещё не включен автополив
			if (out8_watering_is_started == 0)
			{
				// Включение автополива зоны 8
				WATER_ZONE8_ON;
				out8_watering_is_started = 1;
			}
		}			
			
		// Если включен автополив
		if (out8_watering_is_started)
		{
			// Ожидание завершения автополива по времени
			if (e2p->Statistics->TimeInSeconds / 60 >= (e2p->WateringControls->out8_zero_clock_time_delta + e2p->WateringControls->out8_interval_time * out8_cycles_counter\
					+ e2p->WateringControls->out8_working_time * out8_cycles_counter + e2p->WateringControls->out8_working_time))
			{
				// Выключение автополива зоны 8
				WATER_ZONE8_OFF;
				out8_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out8_interval_time != 0)
				{
					// �?нкремент счётчика кол-ва раз включения автополива за сутки
					out8_cycles_counter++;
				}
			}
		}
	}
	else
	{
		// Выключение автополива зоны 8
		WATER_ZONE8_OFF;
		out8_watering_is_started=0;		
	}
	
	// Проверка смены суток для сброса флагов и счётчиков
	if ((time_in_seconds_prev > 0) && (e2p->Statistics->TimeInSeconds == 0))
	{		
		// Сброс счётчика кол-ва включений автополива за сутки
		out1_cycles_counter = 0;
		out2_cycles_counter = 0;
		out3_cycles_counter = 0;
		out4_cycles_counter = 0;
		out5_cycles_counter = 0;
		out6_cycles_counter = 0;
		out7_cycles_counter = 0;
		out8_cycles_counter = 0;

		out1_watering_is_started = 0;
		out2_watering_is_started = 0;
		out3_watering_is_started = 0;
		out4_watering_is_started = 0;
		out5_watering_is_started = 0;
		out6_watering_is_started = 0;
		out7_watering_is_started = 0;
		out8_watering_is_started = 0;

		// Принудительное отключение автополива
		WATER_ZONE1_OFF;
		WATER_ZONE2_OFF;
		WATER_ZONE3_OFF;
		WATER_ZONE4_OFF;
		WATER_ZONE5_OFF;
		WATER_ZONE6_OFF;
		WATER_ZONE7_OFF;
		WATER_ZONE8_OFF;
	}

	time_in_seconds_prev = e2p->Statistics->TimeInSeconds;
	
	// Управление яркостью дисплея, когда включен автополив
	if ((out1_watering_is_started == 1) || (out2_watering_is_started == 1) ||
			(out3_watering_is_started == 1) || (out4_watering_is_started == 1) ||
			(out5_watering_is_started == 1) || (out6_watering_is_started == 1) ||
			(out7_watering_is_started == 1) || (out8_watering_is_started == 1))
	{			
		// Яркость можно только повышать
		if (display_brightness <= AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED)
		{
			// Яркость для авторежимов на 30%
			display_brightness=AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE*DISPLAY_BRIGHTNESS_OFF_SPEED;
		}
		// Таймер можно только повышать
		if (display_brightness_timer < AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY)
		{
			// Перезапись, если яркость уже снижена для авторежима 
			if (display_brightness == AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED)
			{
				// Таймер уменьшения яркости дисплея для авторежимов
				display_brightness_timer = AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY;
			}
		}

		e2p->WateringControls->AutoWatering = 1;
	}
	else e2p->WateringControls->AutoWatering = 0;
}


void HAL_PWR_PVDCallback(void)
{
  // Обнаружено восстановление питания МК (>2.8В)
	power_up_detected = 1;
	
	//HAL_NVIC_SystemReset();
}

// Настройка PVD (Programmable Voltage Detector)
static void PVD_Config(void)
{
	PWR_PVDTypeDef sConfigPVD = {0,};
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_7; // 2.9V
	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; 					// прерывание при восстановлении питания
	HAL_PWR_ConfigPVD(&sConfigPVD); // конфигурируем
	HAL_PWR_EnablePVD(); // активируем PVD
}


// Analog watchdog callback in non blocking mode. 
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC2)
	{
		if ((HAL_ADC_GetValue(&hadc2)) <= ADC_WDG_LOW_THRESHOLD)
		{
			power_down_detected = 1;
			// Disable ADC analog watchdog interrupt
			__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD);
		}
		else if ((HAL_ADC_GetValue(&hadc2)) >= ADC_WDG_HIGH_THRESHOLD)
		{
			power_up_detected = 1;
			// Disable ADC analog watchdog interrupt
			__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD);
		}
	}
}


// Уменьшение энергопотребления контроллера для сохранения данных в eeprom
// Процедура выполняется за 70 мкс
void Reduce_mcu_power(void)
{
	DISPLAY_POWER_DISABLE;
	
	WATER_PUMP_OFF;
	WATER_ZONE1_OFF;
	WATER_ZONE2_OFF;
	WATER_ZONE3_OFF;
	WATER_ZONE4_OFF;
	WATER_ZONE5_OFF;
	WATER_ZONE6_OFF;
	WATER_ZONE7_OFF;
	WATER_ZONE8_OFF;
	
	TXD1_DISABLE;
	TXD4_DISABLE;
	TXD3_DISABLE;
	RXD1_DISABLE;
	RXD4_DISABLE;
	RXD3_DISABLE;
	
	LED1_OFF;
	LED2_OFF;
	
	HAL_UART_DeInit(&huart5);

	HAL_UART_DeInit(&huart2);

	HAL_UART_DeInit(&huart3);
	
	HAL_UART_DeInit(&huart2);
	
	HAL_UART_DeInit(&huart1);
	
  HAL_ADC_DeInit(&hadc1);

  HAL_TIM_Base_DeInit(&htim4) ;
	
  /* DMA controller clock disable */
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMA2_CLK_DISABLE();	
}


// Копирование из одного буфера по произвольному адресу во 2-ой 
void Copy_buf_random_address(uint8_t * source_buf, uint32_t source_buf_offset, uint8_t * dest_buf, uint32_t dest_buf_offset, uint32_t size_to_copy)
{
	uint32_t source_buf_idx = 0;
	uint32_t dest_buf_idx = 0;
	
	// Учёт смещений
	source_buf_idx += source_buf_offset;
	dest_buf_idx += dest_buf_offset;
	
	while (size_to_copy > 0)
	{
		dest_buf[dest_buf_idx++] = source_buf[source_buf_idx++];
		size_to_copy--;
	}
}


// Обработчик состояния падения напряжения питания ниже 4.6В
void Power_down_handler(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2pDataTypeDef * e2p)
{
	// Уменьшение энергопотребления контроллера для сохранения данных в eeprom
	Reduce_mcu_power();

	// Сохранение рабочих переменных в eeprom
	Backup_all_data(hcrc, hi2c, hrtc, e2p);
}

// Усреднение значения давления
void Get_average_pressure_value(E2pDataTypeDef * e2p)
{
	static	uint16_t		counter = 0;
	static	float				pressure_sum = 0;

	
	// Усреднение измеренных значений**********************************
	pressure_sum += e2p->LastPumpCycle->water_pressure_value;

	counter++;
	if (counter >= 5)
	{
		e2p->LastPumpCycle->average_water_pressure_value = (int16_t) roundf(pressure_sum / 5);

		counter = 0;
		pressure_sum = 0;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	
	// Сброс флага обнаружения восстановления питания шины +5В >=4.9В
	power_up_detected = 0;
	
	// Настройка на контроль восстановления питания шины +5В >=4.9В
	ADC2->LTR = 0;
	ADC2->HTR = ADC_WDG_HIGH_THRESHOLD;
	
	// Clear the ADC analog watchdog flag
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD);
	// Enable ADC analog watchdog interrupt
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD);
	
	// Ждём восстановления питания +5В >=4.9В
	while (power_up_detected == 0)
	{
		// Сброс watchdog
		IWDG->KR = IWDG_KEY_RELOAD;
	}

	// Сброс при восстановлении питания
	HAL_NVIC_SystemReset();
	
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
