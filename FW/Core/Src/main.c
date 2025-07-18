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

JetsonComPort_t		jetson;
ExtComPort_t			external;
NextionComPort_t	nextion;
ComPortData_t			com1, com2, com3, com4, com5;

E2p_t									e2p;
Adc_t									adc1;
Statistics_t					stats;
Calibrations_t				calib;
Temperature_t					ds18b20;
WateringControl_t			water_ctrl;
LastPumpCycle_t				last_pump_cycle;
RingBuffer_t					com1_ring_buf, com2_ring_buf;

PumpCurrentState_t		pumpState;
PumpControlCommands_t pumpComms;
UvLampState_t 				uvLampState;
CurrentSystemState_t 	sysState;

// Флаг включения св-диода индикации секундной метки
volatile uint8_t	time_led_is_on;

//volatile uint8_t	uv_lamp_is_on;

// Флаг обнаружения восстановления напряжения питания (>4.8В), adc watchdog
volatile uint8_t	power_up_detected;
// Флаг обнаружения падения напряжения питания (<4.6В), adc watchdog
volatile uint8_t	power_down_detected;

// Переменные слежения за целостностью связи по RS485
volatile uint8_t 	control_link_is_lost;

// Переменная яркости дисплея в привязке к systick в мсек (2000/20=100%), (100/20=5%)
volatile int16_t	display_brightness;
// Таймер задержки перед уменьшением яркости дисплея, сек
volatile int16_t	display_brightness_timer;

// Флаг разрешения выполнения основного потока обслуживания периф. устройств 
volatile uint8_t 	periph_scan_enabled;

// Счётчик ошибок
volatile uint32_t 	func_err_counter;

// Счётчик ошибок датчика температуры
volatile uint32_t 	temp_sensors_err_counter;

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
	
	ReturnCode_t 			func_res;
	HAL_StatusTypeDef	hal_func_res;
	
	int32_t time_temp, time_prev = 0;
	
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
  MX_IWDG_Init();
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
			Voltage_calc_from_adc_value(&e2p, &sysState);
			
			// Усреднение значения давления
			Get_average_pressure_value(&e2p, &sysState);
		}
		
		// Checking whether data is received via COM1**************************************************
		if (com1.RxdPacketIsReceived)
		{
			// Handles data from Com port
			func_res = Com_rxd_handler(&hcrc, com1.ComNum, &jetson, &nextion);			
			if(func_res != OK) func_err_counter++;

			com1.RxdPacketIsReceived = 0;			
		}
		
		// Checking whether data is received via COM3**************************************************
		if (com3.RxdPacketIsReceived)
		{
			// Handles data from Com port
			func_res = Com_rxd_handler(&hcrc, com3.ComNum, &jetson, &nextion);		
			if(func_res != OK) func_err_counter++;

			com3.RxdPacketIsReceived = 0;
		}
		
		// Проверка наличия принятой по COM2 строки данных из дисплея Nextion**************************
		if (com2.RxdPacketIsReceived)
		{
			// Обработчик принятого пакета по USART
			func_res = Nextion_received_data_handler(&hrtc, &e2p);	
			if(func_res != OK)
			{
				func_err_counter++;
			}

			com2.RxdPacketIsReceived = 0;
		}		

		// Проверка готовности обновления дисплея Nextion**********************************************
		if (nextion.RefreshReady)
		{		
			nextion.RefreshReady = 0;
			
			// Отрисовка на Nextion текущих значений
			func_res = Prepare_params_and_send_to_nextion(&hrtc, &e2p, &nextion, &sysState);			
			if(func_res != OK)
			{
				func_err_counter++;
			}
		}
		
		// Checking presence of unsent data in COM1 ring buffer****************************************
		if (Is_all_data_popped_from_ring_buffer(&com1_ring_buf))
		{
			// Checking whether data is ready to send via COM1
			if (com1.TxdPacketIsReadyToSend)
			{
				if (jetson.ComLink == COM1)
				{
					// Popping data from ring data buffer for sending 
					func_res = Pop_string_from_ring_buffer(jetson.ComRingBuf, jetson.PhTxdBuffer, &jetson.StringSize);
					if(func_res == OK)
					{
						com1.TxdPacketIsReadyToSend = 0;
						hal_func_res = HAL_UART_Transmit_DMA(&huart1, jetson.PhTxdBuffer, jetson.StringSize);
					}
					else func_err_counter++;
				}
				else if (external.ComLink == COM1)
				{
					// Popping data from ring data buffer for sending 
					func_res = Pop_string_from_ring_buffer(external.ComRingBuf, external.PhTxdBuffer, &external.StringSize);
					if(func_res == OK)
					{
						com1.TxdPacketIsReadyToSend = 0;
						hal_func_res = HAL_UART_Transmit_DMA(&huart1, external.PhTxdBuffer, external.StringSize);
					}
					else func_err_counter++;
				}
			}
		}
				
		// Checking presence of unsent data in COM2 ring buffer****************************************
		if (Is_all_data_popped_from_ring_buffer(&com2_ring_buf))
		{
			// Checking whether data is ready to send via COM2
			if (com2.TxdPacketIsReadyToSend)
			{
				if (nextion.ComLink == COM2)
				{
					// Popping data from ring data buffer for sending 
					func_res = Pop_string_from_ring_buffer(nextion.ComRingBuf, nextion.PhTxdBuffer, &nextion.StringSize);
					if(func_res == OK)
					{
						com2.TxdPacketIsReadyToSend = 0;
						hal_func_res = HAL_UART_Transmit_DMA(&huart2, nextion.PhTxdBuffer, nextion.StringSize);
					}
					else
					{
						func_err_counter++;
					}
				}
				else if (jetson.ComLink == COM2)
				{
					// Popping data from ring data buffer for sending 
					func_res = Pop_string_from_ring_buffer(jetson.ComRingBuf, jetson.PhTxdBuffer, &jetson.StringSize);
					if(func_res == OK)
					{
						com2.TxdPacketIsReadyToSend = 0;
						hal_func_res = HAL_UART_Transmit_DMA(&huart2, jetson.PhTxdBuffer, jetson.StringSize);
					}
					else func_err_counter++;
				}
				else if (external.ComLink == COM2)
				{
					// Popping data from ring data buffer for sending 
					func_res = Pop_string_from_ring_buffer(external.ComRingBuf, external.PhTxdBuffer, &external.StringSize);
					if(func_res == OK)
					{
						com2.TxdPacketIsReadyToSend = 0;
						hal_func_res = HAL_UART_Transmit_DMA(&huart2, external.PhTxdBuffer, external.StringSize);
					}
					else func_err_counter++;
				}
			}
		}
		
		// Проверка готовности выполнения ветки работы с периферией************************************
		if (periph_scan_enabled)
		{
			time_temp = Get_time_in_sec(&hrtc);
			
			if (time_temp != time_prev)
			{
				// Обновление секундного счётчика времени суток
				sysState.TimeInSeconds = time_temp;

				// Включение св-диода индикации секундной метки
				LED2_ON;
				time_led_is_on = 1;
				time_prev = time_temp;
			}
			
			// Коррекция времени и инкремент суток
			Make_time_correction_and_day_inc(&hrtc, &e2p, &sysState);
						
			// Выполнение автоинкремента/автодекремента каждые 150 мсек, если кнопка на дисплее удерживается
			Parsing_nextion_display_string(&hrtc, &e2p, nextion.RxdBuffer, com2.RxdPacketLenght8, com2.RxdPacketIsReceived, &sysState);
			
			// Управление системой
			SysControlLogic(&e2p, &sysState);

			// Сформировать статистику расхода воды
			Make_water_using_statistics(&e2p, &sysState);
			
			periph_scan_enabled = 0;
		}
		
		// Опрос термодатчиков только при наступлении очередного момента времени***********************
		if (ds18b20.GetSensorsData)
		{
			uint8_t err = OW_OK;
			
			// Опрос термодатчиков только при их наличии
			if (ds18b20.DiscoveredQuantity)
			{
				err = Polling_termosensors(&ds18b20);
				sysState.CurrentWaterTemp = (int16_t) (ds18b20.TempSensorsValues[0] * 10);
			}
			// При отсутствии д.темп. или ошибке связи
			if((ds18b20.DiscoveredQuantity == 0) || (err != OW_OK))
			{
				sysState.CurrentWaterTemp = 0;
				temp_sensors_err_counter++;
				
//				HAL_UART_MspDeInit(&huart5);
//				HAL_UART_MspInit(&huart5);
//				MX_UART5_Init();
				
				// Discovered termosensors qwantity on COM5
				ds18b20.DiscoveredQuantity = ds18b20_init(&huart5, &com5);
				err = OW_OK;
			}

			ds18b20.GetSensorsData = 0;
			
			// Сформировать статистику по температурам (раз в 1 сек)
			Make_temperature_statistics(&e2p, &sysState);
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
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
  hrtc.Init.AsynchPrediv = 32762;
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
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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
  HAL_GPIO_WritePin(GPIOC, WATER_ZONE5_Pin|WATER_ZONE6_Pin|PWR_CTRL1_Pin|UV_STERILIZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISP_PWR_EN_Pin|PUMP_ON_OFF_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_TXD1_RXD1_GPIO_Port, EN_TXD1_RXD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TURBINE_COUNTER_EXTI2_Pin WATER_COUNTER_EXTI3_Pin */
  GPIO_InitStruct.Pin = TURBINE_COUNTER_EXTI2_Pin|WATER_COUNTER_EXTI3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : WATER_ZONE5_Pin WATER_ZONE6_Pin PWR_CTRL1_Pin UV_STERILIZER_Pin */
  GPIO_InitStruct.Pin = WATER_ZONE5_Pin|WATER_ZONE6_Pin|PWR_CTRL1_Pin|UV_STERILIZER_Pin;
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
  HAL_NVIC_SetPriority(EXTI2_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
		// Starting interpacket timer
		com1.TxdPacketReadyToSendTimer = 1;
		
		// Если отключено добавление паузы между пакетами
		if(COM1_DATA_PACKET_SENDING_INTERVAL == 0)
		{
			com1.TxdPacketIsReadyToSend = 1;
		}
	}

	else if (huart->Instance == USART2)
	{
		com2.TxdPacketIsSent = 1;
		// Starting interpacket timer
		com2.TxdPacketReadyToSendTimer = 1;
		
		// Если отключено добавление паузы между пакетами
		if(COM2_DATA_PACKET_SENDING_INTERVAL == 0)
		{
			com2.TxdPacketIsReadyToSend = 1;
		}
	}

	else if (huart->Instance == USART3)
	{
		com3.TxdPacketIsSent = 1;
		// Starting interpacket timer
		com3.TxdPacketReadyToSendTimer = 1;
		
		// Если отключено добавление паузы между пакетами
		if(COM3_DATA_PACKET_SENDING_INTERVAL == 0)
		{
			com3.TxdPacketIsReadyToSend = 1;
		}
	}
	
	else if (huart->Instance == UART4)
	{
		com4.TxdPacketIsSent = 1;
		// Starting interpacket timer
		com4.TxdPacketReadyToSendTimer = 1;
		
		// Если отключено добавление паузы между пакетами
		if(COM4_DATA_PACKET_SENDING_INTERVAL == 0)
		{
			com4.TxdPacketIsReadyToSend = 1;
		}
	}
	
	else if (huart->Instance == UART5)
	{
		com5.TxdPacketIsSent = 1;
		// Starting interpacket timer
		com5.TxdPacketReadyToSendTimer = 1;
		
		// Если отключено добавление паузы между пакетами
		if(COM5_DATA_PACKET_SENDING_INTERVAL == 0)
		{
			com5.TxdPacketIsReadyToSend = 1;
		}
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
void Com_start_receiving_data(ComNum_t ComNum)
{
	// Completing Com1 structure and data receive start
	if (ComNum == COM1)
	{
		com1.RxdIdx8 = 0;
		com1.RxdPacketIsReceived = 0;
		com1.RxDataFlowGapTimer = 0;
		HAL_UART_Receive_DMA(&huart1,(uint8_t *) &com1.ByteReceived, 1);
	}		

	// Completing Com2 structure and data receive start
	else if (ComNum == COM2)
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
	
	// Если сработал счётный вход от счётчика воды (1 импульс на каждые 10 литров)
	if (GPIO_Pin == WATER_COUNTER_EXTI3_Pin) {
		// Если уровень =1 (включено)
		if (WATER_COUNTER_EXTI3_READ_PIN == 1) {
			if(e2p.Calibrations->WaterCounterLitersPerImpulse > 0) {
				// Задержка для борьбы с дребезгом контактов
				if(HAL_GetTick() - time_point_prev >= 100) {				
					// Если в спец. поливе
					if(*sysState.pumpCurrState == SpecialWateringMode) {
						if(e2p.LastPumpCycle->SpModeWateringVolumeCounter > 0) {
							// Счёт кол-ва воды, перекачанной насосом в спец. поливе, литры
							e2p.LastPumpCycle->SpModeWateringVolumeCounter -= e2p.Calibrations->WaterCounterLitersPerImpulse;
						}
					}
					
					// Счёт кол-ва воды, перекачанной насосом в текущем цикле, литры
					e2p.LastPumpCycle->WaterPumpedAtLastCycle += e2p.Calibrations->WaterCounterLitersPerImpulse;
					
					// Счёт общего кол-ва воды, перекачанной насосом, литры
					e2p.Statistics->WaterPumpedTotal += e2p.Calibrations->WaterCounterLitersPerImpulse;
					
					time_point_prev = HAL_GetTick();
				}
			}
		}
		
		// Сброс флага вторичной сработки
		EXTI->PR |= WATER_COUNTER_EXTI3_Pin;
	}
	
	// Если сработал счётный вход от турбины (несколько импульсов на каждый литр)
	else if (GPIO_Pin == TURBINE_COUNTER_EXTI2_Pin) {
		// Если уровень =1 (включено)
		if (TURBINE_EXTI2_READ_PIN == 1) {
			if(e2p.Calibrations->TurbineImpulsesPerLiter > 0) {			
				e2p.LastPumpCycle->TurbineImpCounter++;
				
				if(e2p.LastPumpCycle->TurbineImpCounter >= e2p.Calibrations->TurbineImpulsesPerLiter) {
					// Если в спец. поливе
					if(*sysState.pumpCurrState == SpecialWateringMode) {
						if(e2p.LastPumpCycle->SpModeWateringVolumeCounter > 0) {
							// Счёт кол-ва воды, перекачанной насосом в спец. поливе, литры
							e2p.LastPumpCycle->SpModeWateringVolumeCounter -= 1;
						}						
					}

					// Счёт кол-ва воды, перекачанной насосом в текущем цикле, литры
					e2p.LastPumpCycle->WaterPumpedAtLastCycle++;
					
					// Счёт общего кол-ва воды, перекачанной насосом, литры
					e2p.Statistics->WaterPumpedTotal++;
					
					e2p.LastPumpCycle->TurbineImpCounter = 0;
				}
			}
		}
		
		// Сброс флага вторичной сработки
		EXTI->PR |= TURBINE_COUNTER_EXTI2_Pin;
	}
}


void HAL_SYSTICK_Callback(void)
{
	static uint8_t		time_led_timer = 0;
	static uint16_t		periph_scan_timer = 0;
	static uint16_t		nextion_send_timer = 0;
	static uint16_t		control_data_timeout_timer = 0;
	static uint8_t		brightness_dim_already_done = 0;
	static uint16_t		timer_1000ms = 0, dry_work_timer = 0;
	static uint32_t		PumpedQuantityAtLastCycle_at_zero = 0;

	// Таймер отсылки данных в дисплей Nextion
	nextion_send_timer++;
	if (nextion_send_timer > NEXTION_REFRESH_PERIOD)
	{
		// Флаг выполнения обновления экрана
		nextion.RefreshReady = 1;
		nextion_send_timer = 0;
	}
	
	// Таймер паузы между опросами термодатчиков
	ds18b20.PollingWaitTimer++;
	if (ds18b20.PollingWaitTimer > TERMO_SENSORS_POLL_PERIOD)
	{
		// Флаг выполнения опроса датчиков
		ds18b20.GetSensorsData = 1;
		ds18b20.PollingWaitTimer = 0;
	}
	
	// Счёт секунд, проверка временных условий
	timer_1000ms++;
	if (timer_1000ms >= 1000)
	{
		timer_1000ms = 0;

		// Общее время работы контроллера, секунд
		e2p.Statistics->TotalControllerWorkingTime++;
		
		// Наработка УФ лампы
		if(*sysState.uvLampState != uvLampIsOff) {
			e2p.Statistics->UvLampWorkingTime++;
		}

		// Если насос запущен
		if (sysState.PumpIsStarted)
		{
			// Если в спец. режиме полива
			if(*sysState.pumpCurrState == SpecialWateringMode) {
				if(e2p.LastPumpCycle->SpModeWateringTimer) {
					e2p.LastPumpCycle->SpModeWateringTimer -= 1;
				}
			}
			else {
				// Если насос включен, то считаем время работы за одно включение, сек
				e2p.LastPumpCycle->PumpWorkingTimeAtLastCycle++;
			}
			
			// Общее время работы насоса, секунд
			e2p.Statistics->TotalPumpWorkingTime++;
			
			// Обновляем точку отсчёта для обнаружения события "сухого хода" при нуле таймера
			if (dry_work_timer == 0) PumpedQuantityAtLastCycle_at_zero = e2p.LastPumpCycle->WaterPumpedAtLastCycle;
			
			dry_work_timer++;
			
			// Если прошло контрольное время,
			if (dry_work_timer >= e2p.LastPumpCycle->PumpDryRunStopTimeout)
			{
				// а значение "счётчика воды за цикл" не изменилось,
				if (e2p.LastPumpCycle->WaterPumpedAtLastCycle == PumpedQuantityAtLastCycle_at_zero)
				{
					// то фиксируем событие "сухого хода"
					sysState.DryRunProtectiveStop = 1;
				}
				
				dry_work_timer = 0;
			}
		}
		// Если не запущен, то обнуление счётчика времени отслеживания "сухого хода"
		else
		{
			dry_work_timer = 0;
		}
		
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
	
	// Таймер в мсек для выполнения потока опроса и управления периф. устройствами
	periph_scan_timer++;
	if (periph_scan_timer >= PERIPH_SCAN_TIMER_TIMEOUT)
	{
		// Флаг разрешения выполнения основного потока обслуживания периф. устройств 
		periph_scan_enabled = 1;
		periph_scan_timer = 0;
	}

	
	// Таймер 500 мсек для определения обрыва связи
	control_data_timeout_timer++;
	if (control_data_timeout_timer > NO_DATA_TIMEOUT_VALUE)
	{
		control_link_is_lost = 1;
		control_data_timeout_timer = 0;
	}

	
	// Приём по COM1
	if(COM1_DATA_FLOW_GAP_TIME_VALUE)
	{
		if (com1.RxDataFlowGapTimer != 0)
		{
			com1.RxDataFlowGapTimer++;

			// По истечении заданных мс при разрыве в данных считаем пакет завершённым, проверяем
			if (com1.RxDataFlowGapTimer >= COM1_DATA_FLOW_GAP_TIME_VALUE + 1)
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
	}
	
	// Передача по COM1, добавление паузы между пакетами
	if(COM1_DATA_PACKET_SENDING_INTERVAL)
	{
		if(com1.TxdPacketReadyToSendTimer) com1.TxdPacketReadyToSendTimer++;
		if(com1.TxdPacketReadyToSendTimer >= COM1_DATA_PACKET_SENDING_INTERVAL + 1)
		{
			com1.TxdPacketIsReadyToSend = 1;
			com1.TxdPacketReadyToSendTimer = 0;
		}
	}

	
	// Приём по COM2
		if(COM2_DATA_FLOW_GAP_TIME_VALUE)
	{
		if (com2.RxDataFlowGapTimer != 0)
		{
			com2.RxDataFlowGapTimer++;

			// По истечении заданных мс при разрыве в данных считаем пакет завершённым, проверяем
			if (com2.RxDataFlowGapTimer >= COM2_DATA_FLOW_GAP_TIME_VALUE + 1)
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
	}

	// Передача по COM2, добавление паузы между пакетами
	if(COM2_DATA_PACKET_SENDING_INTERVAL)
	{
		if(com2.TxdPacketReadyToSendTimer) com2.TxdPacketReadyToSendTimer++;
		if(com2.TxdPacketReadyToSendTimer >= COM2_DATA_PACKET_SENDING_INTERVAL + 1)
		{
			com2.TxdPacketIsReadyToSend = 1;
			com2.TxdPacketReadyToSendTimer = 0;
		}
	}

	
	// Выключение св-диода индикации секундной метки
	if (time_led_is_on) time_led_timer++;
	if (time_led_timer > 2)
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
void Voltage_calc_from_adc_value(E2p_t * e2p, CurrentSystemState_t * sysState)
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
		//voltage /= 100;
		voltage -= (float)e2p->Calibrations->PsensorMinPressureVoltage * 100;
		voltage *= ((float)e2p->Calibrations->PsensorMaxPressure - (float)e2p->Calibrations->PsensorMinPressure);
		voltage /= ((float)e2p->Calibrations->PsensorMaxPressureVoltage * 100 - (float)e2p->Calibrations->PsensorMinPressureVoltage * 100);
		//voltage *= 10;
		if (voltage < 0) voltage = 0;
		voltage = roundf(voltage);      

		sysState->WaterPressure = (int16_t) voltage;
		
		//if (WaterPressure<0) WaterPressure=0;
	}

	return;
}


// Initial hardware user settings
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
	
	sysState.uvLampState = &uvLampState;
	sysState.pumpCurrState = &pumpState;
	sysState.pumpCtrlComms = &pumpComms;
	
	// Настройка слежения за появлением питания (срабатывает не при восстановлении, а при падении)
	//PVD_Config();
	
	// Запись калибровочного коэффициента (0-127) для коррекции хода часов реального времени ( ppm -> сек/месяц, 127 = -314 сек/мес)

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
	PWR_CTRL1_OFF;
	UV_STERILIZER_OFF;
	
	// ADC autocalibration
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);

	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
	
	// без этого не работает ацп в связке с таймером 4 по capture/compare
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM4->CR2 |= TIM_CR2_OIS4;
	
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

	// Ждём восстановления напряжения питания цепи +5В, >= ADC_WDG_HIGH_THRESHOLD
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

	// Naming working structures of Com ports 
	com1.ComNum = COM1;
	com2.ComNum = COM2;
	com3.ComNum = COM3;
	com4.ComNum = COM4;
	
	// Link logical Com port to physical
	nextion.ComLink = NEXTION_DISPLAY_COM_PORT;
	jetson.ComLink = JETSON_COM_PORT;
	external.ComLink = EXTERNAL_INTERFACE_COM_PORT;

	// Link logical structure to physical usart1
	if(jetson.ComLink == COM1)
	{
		jetson.Com = &com1;
		jetson.ComRingBuf = &com1_ring_buf;
		jetson.ComRingBuf->ComLink = COM1;
	}
	else if(nextion.ComLink == COM1)
	{
		nextion.Com = &com1;
		nextion.ComRingBuf = &com1_ring_buf;
		nextion.ComRingBuf->ComLink = COM1;
	}
	else if(external.ComLink == COM1)
	{
		external.Com = &com1;
		external.ComRingBuf = &com1_ring_buf;
		external.ComRingBuf->ComLink = COM1;
	}

	// Link logical structure to physical usart2
	if(jetson.ComLink == COM2)
	{
		jetson.Com = &com2;
		jetson.ComRingBuf = &com2_ring_buf;
		jetson.ComRingBuf->ComLink = COM2;
	}
	else if(nextion.ComLink == COM2)
	{
		nextion.Com = &com2;
		nextion.ComRingBuf = &com2_ring_buf;
		nextion.ComRingBuf->ComLink = COM2;
	}
	else if(external.ComLink == COM2)
	{
		external.Com = &com2;
		external.ComRingBuf = &com2_ring_buf;
		external.ComRingBuf->ComLink = COM2;
	}
	
	// Init ring buffers for data sending by UART
	// Setting sizes of elements of buffers
	com1_ring_buf.PBufElementSize = 4;		// uint32_t = 4 bytes
	com2_ring_buf.PBufElementSize = 4;		// uint32_t = 4 bytes
	com1_ring_buf.DBufElementSize = 1;		// uint8_t  = 1 byte
	com2_ring_buf.DBufElementSize = 1;		// uint8_t  = 1 byte
	
	// Setting mask for index wrapping when accessing ring data buffer
	com1_ring_buf.DBufMask = sizeof(com1_ring_buf.DBuf) / com1_ring_buf.DBufElementSize - 1;
	com2_ring_buf.DBufMask = sizeof(com2_ring_buf.DBuf) / com2_ring_buf.DBufElementSize - 1;
	// Setting mask for index wrapping when accessing ring pointers buffer
	com1_ring_buf.PBufMask = sizeof(com1_ring_buf.PBuf) / com1_ring_buf.PBufElementSize - 1;
	com2_ring_buf.PBufMask = sizeof(com2_ring_buf.PBuf) / com1_ring_buf.PBufElementSize - 1;

	// Prepare for first sending (starting sending timers)
	com1.TxdPacketIsReadyToSend = 1;
	com1.TxdPacketReadyToSendTimer = 1;  // Enables counting time intervals between sendings, using as pause
	com2.TxdPacketIsReadyToSend = 1;
	com2.TxdPacketReadyToSendTimer = 1;
	com3.TxdPacketIsReadyToSend = 1;
	com3.TxdPacketReadyToSendTimer = 1;
	com4.TxdPacketIsReadyToSend = 1;
	com4.TxdPacketReadyToSendTimer = 1;
	com5.TxdPacketIsReadyToSend = 1;
	com5.TxdPacketReadyToSendTimer = 1;
	
	// Com ports data receive starting
	Com_start_receiving_data(COM1);
	Com_start_receiving_data(COM2);
	Com_start_receiving_data(COM3);
	Com_start_receiving_data(COM4);
	//Com_start_receiving_data(COM5);		// Termosensor's COM port, initializing in ds18b20.c
	
	// Яркость дисплея на max (2000/20=100%)
	display_brightness = DISPLAY_BRIGHTNESS_MAX_VALUE*DISPLAY_BRIGHTNESS_OFF_SPEED;
	// Запуск таймера уменьшения яркости дисплея
	display_brightness_timer = DISPLAY_BRIGHTNESS_OFF_DELAY;
	
	// Setting instance name (number)
	com5.ComNum = COM5;
	// Discovered termosensors qwantity on COM5
	ds18b20.DiscoveredQuantity = ds18b20_init(&huart5, &com5);
	
	// Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL) are synchronized with RTC APB clock
	HAL_func_res = HAL_RTC_WaitForSynchro(&hrtc);

	HAL_Delay(500);
}


// Разбор принятой строки от дисплея Nextion
void Parsing_nextion_display_string(RTC_HandleTypeDef  * hrtc, E2p_t * e2p, uint8_t * buf,
																		uint16_t string_lenght, uint8_t string_status, CurrentSystemState_t * sysState)
{
	const uint32_t 	key_is_pressed = 0x71010000;
	const uint32_t 	key_is_released = 0x71000000;
	uint8_t					large_step = 0;
	static uint8_t	state_machine = 0;
	static uint32_t	source_type = 0;
	static uint32_t	source_value = 0;
	static uint32_t	key_pressing_time_moment = 0;
	int32_t 				time_temp;

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
		// 20-ти секундная пауза перед автоинкрементом/автодекрементом числами *1000
		if (HAL_GetTick() >= key_pressing_time_moment + 20000) large_step = 3;
		// 10-ти секундная пауза перед автоинкрементом/автодекрементом числами *100
		if (HAL_GetTick() >= key_pressing_time_moment + 10000) large_step = 2;
		// 5-ти секундная пауза перед автоинкрементом/автодекрементом числами *10
		else if (HAL_GetTick() >= key_pressing_time_moment + 5000) large_step = 1;
		else large_step = 0;
	}
		
	// Если кнопка была отжата, то обеспечиваем реакцию на повторное нажатие
	if (source_value == key_is_released)
	{
		// Если это не код номера экрана scrX
		if((source_type & 0xFFFFFF00) != 0x53637200)
		{
			state_machine = 0;
			// Обновляем момент нажатия кнопки на дисплее
			key_pressing_time_moment = 0;
			
			return;
		}
	}

	if (state_machine == 0) state_machine = 1;
	
	// Чтение текущего времени
	time_temp = Get_time_in_sec(hrtc);
	
	// Разбор принятой строки
	switch (source_type)
	{	
		// Если нажата кнопка ручного принудительного включения насоса
		case PumpOn:
		{
			if (source_value == key_is_pressed) {
				// Если не активен какой-либо режим работы
				if(*sysState->pumpCurrState == IdleMode) {
					// Включить насос
					sysState->pumpCtrlComms->SwitchPumpOn = 1;
					*sysState->pumpCurrState = HandPumpingMode;
					
					// Если не находимся в режиме автоподкачки, то
					if (sysState->AutoPumpingMode == 0)
					{
						// Обнуление счётчиков значений последнего цикла
						e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle = 0;
						e2p->LastPumpCycle->WaterPumpedAtLastCycle = 0;					
					}
				}
			}
						
			break;
		}

		// Если нажата кнопка выключения насоса
		case PumpOff:
		{
			if (source_value == key_is_pressed)
			{
				// Если в режиме прогрева УФ лампы, индицируем
				if(*sysState->uvLampState == uvLampPreheating) {
					*sysState->uvLampState = uvLampBlinkWhilePreheating;
				}
			
				// Если не в режиме спец. полива
				if(*sysState->pumpCurrState != SpecialWateringMode) {
					// Если не в режиме прогрева УФ лампы
					if((*sysState->uvLampState != uvLampPreheating) &&
						 (*sysState->uvLampState != uvLampBlinkWhilePreheating)) {
						// Выключить насос
						sysState->pumpCtrlComms->SwitchPumpOff = 1;

						// Если кнопка выкл. была нажата при активном событии "сухого хода", то
						if (sysState->DryRunProtectiveStop)
						{
							// сброс события "сухого хода"
							sysState->DryRunProtectiveStop = 0;
							
							// Возобновление автоподкачки, если была прервана "сухим ходом"
							if (*sysState->pumpCurrState == AutoPumpingMode)
							{
								// Отключение автоподкачки
								//e2p->LastPumpCycle->auto_pump_is_done = 0;
								
								// Включаем насос
								sysState->pumpCtrlComms->SwitchPumpOn = 1;
								sysState->pumpCtrlComms->SwitchPumpOff = 0;
							}
						}
						// Если событие "сухой ход" неактивно, то
						else
						{
							// Если кнопка выкл. была нажата при штатной работе автоподкачки, то
							if (*sysState->pumpCurrState == AutoPumpingMode)
							{
								// Разрешение повторной попытки автоподкачки воды
								*sysState->pumpCurrState = IdleMode;						
							}
						}
					}
				}
			}

			break;
		}

		// Уменьшение давления включения насоса, атм * 10 ***************************
		case PumpOnPressureDec:
		{
			e2p->Calibrations->PumpOnPressure -= 1;
			if (e2p->Calibrations->PumpOnPressure < 0) e2p->Calibrations->PumpOnPressure = 0;
			break;
		}

		// Увеличение давления включения насоса, атм * 10
		case PumpOnPressureInc:
		{
			e2p->Calibrations->PumpOnPressure += 1;
			if (e2p->Calibrations->PumpOnPressure > PRESSURE_MAX_VALUE) e2p->Calibrations->PumpOnPressure = PRESSURE_MAX_VALUE;
			break;
		}

		// Уменьшение давления выключения насоса, атм * 10 ***************************
		case PumpOffPressureDec:
		{
			e2p->Calibrations->PumpOffPressure -= 1;
			if (e2p->Calibrations->PumpOffPressure < 0) e2p->Calibrations->PumpOffPressure = 0;
			break;
		}

		// Увеличение давления выключения насоса, атм * 10
		case PumpOffPressureInc:
		{
			e2p->Calibrations->PumpOffPressure += 1;
			if (e2p->Calibrations->PumpOffPressure > PRESSURE_MAX_VALUE) e2p->Calibrations->PumpOffPressure = PRESSURE_MAX_VALUE;
			break;
		}

		// Уменьшение минимального значения давления датчика давления, атм * 10 ******
		case PresSensorPminDec:
		{
			e2p->Calibrations->PsensorMinPressure -= 1;
			if (e2p->Calibrations->PsensorMinPressure < 0) e2p->Calibrations->PsensorMinPressure = 0;
			break;
		}

		// Увеличение минимального значения давления датчика давления, атм * 10
		case PresSensorPminInc:
		{
			e2p->Calibrations->PsensorMinPressure += 1;
			if (e2p->Calibrations->PsensorMinPressure > PRESSURE_MAX_VALUE) e2p->Calibrations->PsensorMinPressure = PRESSURE_MAX_VALUE;
			break;
		}
	
		// Уменьшение максимального значения давления датчика давления, атм * 10 ******
		case PresSensorPmaxDec:
		{
			e2p->Calibrations->PsensorMaxPressure -= 1;
			if (e2p->Calibrations->PsensorMaxPressure < 0) e2p->Calibrations->PsensorMaxPressure = 0;
			break;
		}

		// Увеличение максимального значения давления датчика давления, атм * 10
		case PresSensorPmaxInc:
		{
			e2p->Calibrations->PsensorMaxPressure += 1;
			if (e2p->Calibrations->PsensorMaxPressure > PRESSURE_MAX_VALUE) e2p->Calibrations->PsensorMaxPressure = PRESSURE_MAX_VALUE;
			break;
		}			

		// Уменьшение напряжения (0 - 5В), соотв. минимальному давлению датчика давления, мВ*100 ******
		case VoltageForPminDec:
		{
			e2p->Calibrations->PsensorMinPressureVoltage -= 1;
			if (e2p->Calibrations->PsensorMinPressureVoltage < 0) e2p->Calibrations->PsensorMinPressureVoltage = 0;
			break;
		}

		// Увеличение напряжения (0 - 5В), соотв. минимальному давлению датчика давления, мВ*100
		case VoltageForPminInc:
		{
			e2p->Calibrations->PsensorMinPressureVoltage += 1;
			if (e2p->Calibrations->PsensorMinPressureVoltage > MAX_PRESSURE_SENSOR_VOLTAGE)
					e2p->Calibrations->PsensorMinPressureVoltage = MAX_PRESSURE_SENSOR_VOLTAGE;
			break;
		}			

		// Уменьшение напряжения (0 - 5В), соотв. максимальному давлению датчика давления, мВ*100 ******
		case VoltageForPmaxDec:
		{
			e2p->Calibrations->PsensorMaxPressureVoltage -= 1;
			if (e2p->Calibrations->PsensorMaxPressureVoltage < 0) e2p->Calibrations->PsensorMaxPressureVoltage = 0;
			break;
		}

		// Увеличение напряжения (0 - 5В), соотв. максимальному давлению датчика давления, мВ*100
		case VoltageForPmaxInc:
		{
			e2p->Calibrations->PsensorMaxPressureVoltage += 1;
			if (e2p->Calibrations->PsensorMaxPressureVoltage > MAX_PRESSURE_SENSOR_VOLTAGE)
					e2p->Calibrations->PsensorMaxPressureVoltage = MAX_PRESSURE_SENSOR_VOLTAGE;
			break;
		}
		// Уменьшение времени срабатывания останова насоса по "сухому ходу", сек ******
		case PumpRunDryStopTimeoutDec:
		{
			if (large_step == 0)			e2p->LastPumpCycle->PumpDryRunStopTimeout -= 1;
			else if (large_step == 1)	e2p->LastPumpCycle->PumpDryRunStopTimeout -= 1;
			else if (large_step == 2)	e2p->LastPumpCycle->PumpDryRunStopTimeout -= 10;
			else if (large_step == 3)	e2p->LastPumpCycle->PumpDryRunStopTimeout -= 50;
			break;
		}

		// Увеличение времени срабатывания останова насоса по "сухому ходу", сек
		case PumpRunDryStopTimeoutInc:
		{
			if (large_step == 0)			e2p->LastPumpCycle->PumpDryRunStopTimeout += 1;
			else if (large_step == 1)	e2p->LastPumpCycle->PumpDryRunStopTimeout += 1;
			else if (large_step == 2)	e2p->LastPumpCycle->PumpDryRunStopTimeout += 10;
			else if (large_step == 3)	e2p->LastPumpCycle->PumpDryRunStopTimeout += 50;
			break;
		}
		
	// Полив
		// Уменьшение значения текущего номера выхода полива, 1-6 ******
		case CurrWateringOutputNumberDec:
		{
			e2p->WateringControls->CurrWateringOutputNumber -= 1;
			if (e2p->WateringControls->CurrWateringOutputNumber < 1) e2p->WateringControls->CurrWateringOutputNumber = 6;
			break;
		}

		// Увеличение значения текущего номера выхода полива, 1-6 ******
		case CurrWateringOutputNumberInc:
		{
			e2p->WateringControls->CurrWateringOutputNumber += 1;
			if (e2p->WateringControls->CurrWateringOutputNumber > 6) e2p->WateringControls->CurrWateringOutputNumber = 1;
			break;
		}	
				
		// Уменьшение значения смещения времени включения полива зоны 1-6 относительно начала суток, мин ******
		case OutxZeroClockTimeDeltaDec:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_zero_clock_time_delta -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_zero_clock_time_delta -= 5;

			if (e2p->WateringControls->out1_zero_clock_time_delta < 0) e2p->WateringControls->out1_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out2_zero_clock_time_delta < 0) e2p->WateringControls->out2_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out3_zero_clock_time_delta < 0) e2p->WateringControls->out3_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out4_zero_clock_time_delta < 0) e2p->WateringControls->out4_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out5_zero_clock_time_delta < 0) e2p->WateringControls->out5_zero_clock_time_delta = 1435;
			if (e2p->WateringControls->out6_zero_clock_time_delta < 0) e2p->WateringControls->out6_zero_clock_time_delta = 1435;
			break;
		}

		// Увеличение значения смещения времени включения полива зоны 1-6 относительно начала суток, мин
		case OutxZeroClockTimeDeltaInc:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_zero_clock_time_delta += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_zero_clock_time_delta += 5;

			if (e2p->WateringControls->out1_zero_clock_time_delta > 1435) e2p->WateringControls->out1_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out2_zero_clock_time_delta > 1435) e2p->WateringControls->out2_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out3_zero_clock_time_delta > 1435) e2p->WateringControls->out3_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out4_zero_clock_time_delta > 1435) e2p->WateringControls->out4_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out5_zero_clock_time_delta > 1435) e2p->WateringControls->out5_zero_clock_time_delta = 0;
			if (e2p->WateringControls->out6_zero_clock_time_delta > 1435) e2p->WateringControls->out6_zero_clock_time_delta = 0;
			break;
		}		

		// Уменьшение значения времени работы полива зоны 1-6, мин ***************************
		case OutxWorkingTimeDec:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1)			e2p->WateringControls->out1_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_working_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_working_time -= 5;

			if (e2p->WateringControls->out1_working_time < 0) e2p->WateringControls->out1_working_time = 1435;
			if (e2p->WateringControls->out2_working_time < 0) e2p->WateringControls->out2_working_time = 1435;
			if (e2p->WateringControls->out3_working_time < 0) e2p->WateringControls->out3_working_time = 1435;
			if (e2p->WateringControls->out4_working_time < 0) e2p->WateringControls->out4_working_time = 1435;
			if (e2p->WateringControls->out5_working_time < 0) e2p->WateringControls->out5_working_time = 1435;
			if (e2p->WateringControls->out6_working_time < 0) e2p->WateringControls->out6_working_time = 1435;
			break;
		}

		// Увеличение значения времени работы полива зоны 1-6, мин
		case OutxWorkingTimeInc:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_working_time += 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_working_time += 5;

			if (e2p->WateringControls->out1_working_time > 1435) e2p->WateringControls->out1_working_time = 0;
			if (e2p->WateringControls->out2_working_time > 1435) e2p->WateringControls->out2_working_time = 0;
			if (e2p->WateringControls->out3_working_time > 1435) e2p->WateringControls->out3_working_time = 0;
			if (e2p->WateringControls->out4_working_time > 1435) e2p->WateringControls->out4_working_time = 0;
			if (e2p->WateringControls->out5_working_time > 1435) e2p->WateringControls->out5_working_time = 0;
			if (e2p->WateringControls->out6_working_time > 1435) e2p->WateringControls->out6_working_time = 0;
			break;
		}	

		// Уменьшение значения интервала времени между включениями полива зоны 1-6, мин
		case OutxWorkIntervalTimeDec:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_interval_time -= 5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_interval_time -= 5;

			if (e2p->WateringControls->out1_interval_time < 0) e2p->WateringControls->out1_interval_time = 1435;
			if (e2p->WateringControls->out2_interval_time < 0) e2p->WateringControls->out2_interval_time = 1435;
			if (e2p->WateringControls->out3_interval_time < 0) e2p->WateringControls->out3_interval_time = 1435;
			if (e2p->WateringControls->out4_interval_time < 0) e2p->WateringControls->out4_interval_time = 1435;
			if (e2p->WateringControls->out5_interval_time < 0) e2p->WateringControls->out5_interval_time = 1435;
			if (e2p->WateringControls->out6_interval_time < 0) e2p->WateringControls->out6_interval_time = 1435;
			break;
		}

		// Увеличение значения интервала времени между включениями полива зоны 1-6, мин
		case OutxWorkIntervalTimeInc:
		{
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			e2p->WateringControls->out1_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) e2p->WateringControls->out2_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) e2p->WateringControls->out3_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) e2p->WateringControls->out4_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) e2p->WateringControls->out5_interval_time+=5;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) e2p->WateringControls->out6_interval_time+=5;

			if (e2p->WateringControls->out1_interval_time>1435) e2p->WateringControls->out1_interval_time=0;
			if (e2p->WateringControls->out2_interval_time>1435) e2p->WateringControls->out2_interval_time=0;
			if (e2p->WateringControls->out3_interval_time>1435) e2p->WateringControls->out3_interval_time=0;
			if (e2p->WateringControls->out4_interval_time>1435) e2p->WateringControls->out4_interval_time=0;
			if (e2p->WateringControls->out5_interval_time>1435) e2p->WateringControls->out5_interval_time=0;
			if (e2p->WateringControls->out6_interval_time>1435) e2p->WateringControls->out6_interval_time=0;
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
		
			break;
		}		
		
		// Уменьшение значения смещения времени автоподкачки относительно начала суток, мин ******
		case AutoPumpTimeDeltaFromStartOfDayDec:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay -= 5;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay -= 20;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay -= 20;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay -= 20;
			
			if (e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay < 0) e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay = 1435;
			break;
		}

		// Увеличение значения смещения времени автоподкачки относительно начала суток, мин
		case AutoPumpTimeDeltaFromStartOfDayInc:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay += 5;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay += 20;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay += 20;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay += 20;
			
			if (e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay > 1435) e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay = 0;
			break;
		}		

		// Уменьшение значения интервала времени между включениями автоподкачки, мин
		case AutoPumpIntervalTimeDec:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpTimeInterval -= 5;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpTimeInterval -= 20;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpTimeInterval -= 20;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpTimeInterval -= 20;

			if (e2p->LastPumpCycle->AutoPumpTimeInterval < 0) e2p->LastPumpCycle->AutoPumpTimeInterval = 1435;
			break;
		}

		// Увеличение значения интервала времени между включениями автоподкачки, мин
		case AutoPumpIntervalTimeInc:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpTimeInterval += 5;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpTimeInterval += 20;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpTimeInterval += 20;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpTimeInterval += 20;
			
			if (e2p->LastPumpCycle->AutoPumpTimeInterval > 1435) e2p->LastPumpCycle->AutoPumpTimeInterval = 0;
			break;
		}
		
		// Уменьшение объёма автоподкачки, литры *******************************************
		case AutoPumpVolumeDec:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpVolume -= 1;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpVolume -= 10;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpVolume -= 100;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpVolume -= 1000;

			if (e2p->LastPumpCycle->AutoPumpVolume < 0) e2p->LastPumpCycle->AutoPumpVolume = 100000;

			break;
		}

		// Увеличение объёма автоподкачки, литры
		case AutoPumpVolumeInc:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpVolume += 1;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpVolume += 10;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpVolume += 100;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpVolume += 1000;
			
			if (e2p->LastPumpCycle->AutoPumpVolume > 100000) e2p->LastPumpCycle->AutoPumpVolume = 0;
			
			break;			
		}

		// Уменьшение кол-ва включений автоподкачки за сутки***********************************
		case AutoPumpTimesDec:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpTimes -= 1;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpTimes -= 10;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpTimes -= 10;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpTimes -= 10;

			if (e2p->LastPumpCycle->AutoPumpTimes < 0) e2p->LastPumpCycle->AutoPumpTimes = 100;

			break;
		}

		// Увеличение кол-ва включений автоподкачки за сутки
		case AutoPumpTimesInc:
		{
			if (large_step == 0)			e2p->LastPumpCycle->AutoPumpTimes += 1;
			else if (large_step == 1)	e2p->LastPumpCycle->AutoPumpTimes += 10;
			else if (large_step == 2)	e2p->LastPumpCycle->AutoPumpTimes += 10;
			else if (large_step == 3)	e2p->LastPumpCycle->AutoPumpTimes += 10;
			
			if (e2p->LastPumpCycle->AutoPumpTimes > 100) e2p->LastPumpCycle->AutoPumpTimes = 0;
			
			break;			
		}
		

		// Уменьшение значения текущего времени *****************************************
		case CurrentTimeDecrement:
		{
			// Обновление счётчика времени в секундах
			sysState->TimeInSeconds = time_temp;
			if (large_step == 0)			sysState->TimeInSeconds -= 60;
			else if (large_step == 1)	sysState->TimeInSeconds -= 600;
			else if (large_step == 2)	sysState->TimeInSeconds -= 600;
			else if (large_step == 3)	sysState->TimeInSeconds -= 600;

			// Обнуление секунд во время настройки времени
			sysState->TimeInSeconds -= (sysState->TimeInSeconds % 60);
			
			if (sysState->TimeInSeconds < 0) sysState->TimeInSeconds = 86399;
			// Установка времени
			Write_time_to_RTC(hrtc, sysState->TimeInSeconds);
			break;
		}

		// Увеличение значения текущего времени
		case CurrentTimeIncrement:
		{
			// Обновление счётчика времени в секундах
			sysState->TimeInSeconds = time_temp;
			if (large_step == 0)			sysState->TimeInSeconds += 60;
			else if (large_step == 1)	sysState->TimeInSeconds += 600;
			else if (large_step == 2)	sysState->TimeInSeconds += 600;
			else if (large_step == 3)	sysState->TimeInSeconds += 600;

			// Обнуление секунд во время настройки времени
			sysState->TimeInSeconds -= (sysState->TimeInSeconds % 60);
			
			if (sysState->TimeInSeconds > 86399) sysState->TimeInSeconds = 0;
			// Установка времени
			Write_time_to_RTC(hrtc, sysState->TimeInSeconds);
			break;
		}
	
		// Уменьшение значения коррекции текущего времени *******************************
		case TimeCorrectionValueDec:
		{
			e2p->Calibrations->TimeCorrectionValue -= 1;

			if (e2p->Calibrations->TimeCorrectionValue < -7) e2p->Calibrations->TimeCorrectionValue = -7;
			break;
		}

		// Увеличение значения коррекции текущего времени
		case TimeCorrectionValueInc:
		{
			e2p->Calibrations->TimeCorrectionValue += 1;

			if (e2p->Calibrations->TimeCorrectionValue > 7) e2p->Calibrations->TimeCorrectionValue = 7;
			break;
		}
	
		// Уменьшение минимальной точки источника воды *******************************
		case VoltageForPminSourceDec:
		{
			e2p->Calibrations->SourcePsensorMinPressureVoltage -= 1;

			if (e2p->Calibrations->SourcePsensorMinPressureVoltage < 0) e2p->Calibrations->SourcePsensorMinPressureVoltage = 0;
			break;
		}

		// Увеличение минимальной точки источника воды
		case VoltageForPminSourceInc:
		{
			e2p->Calibrations->SourcePsensorMinPressureVoltage += 1;

			if (e2p->Calibrations->SourcePsensorMinPressureVoltage > SOURCE_PRESSURE_MAX_VALUE)
			{
				e2p->Calibrations->SourcePsensorMinPressureVoltage = SOURCE_PRESSURE_MAX_VALUE;
			}
			break;
		}
	
		// Уменьшение максимальной точки источника воды *******************************
		case VoltageForPmaxSourceDec:
		{
			e2p->Calibrations->SourcePsensorMaxPressureVoltage -= 1;

			if (e2p->Calibrations->SourcePsensorMaxPressureVoltage < 0) e2p->Calibrations->SourcePsensorMaxPressureVoltage = 0;
			break;
		}

		// Увеличение максимальной точки источника воды
		case VoltageForPmaxSourceInc:
		{
			e2p->Calibrations->SourcePsensorMaxPressureVoltage += 1;

			if (e2p->Calibrations->SourcePsensorMaxPressureVoltage > SOURCE_PRESSURE_MAX_VALUE)
			{
				e2p->Calibrations->SourcePsensorMaxPressureVoltage = SOURCE_PRESSURE_MAX_VALUE;
			}
			break;
		}

		// Уменьшение минимальной точки накопителя воды *******************************
		case VoltageForPminTankDec:
		{
			e2p->Calibrations->TankPsensorMinPressureVoltage -= 1;

			if (e2p->Calibrations->TankPsensorMinPressureVoltage < 0) e2p->Calibrations->TankPsensorMinPressureVoltage = 0;
			break;
		}

		// Увеличение минимальной точки накопителя воды
		case VoltageForPminTankInc:
		{
			e2p->Calibrations->TankPsensorMinPressureVoltage += 1;

			if (e2p->Calibrations->TankPsensorMinPressureVoltage > DEST_PRESSURE_MAX_VALUE)
			{
				e2p->Calibrations->TankPsensorMinPressureVoltage = DEST_PRESSURE_MAX_VALUE;
			}
			break;
		}

		// Уменьшение максимальной точки накопителя воды *******************************
		case VoltageForPmaxTankDec:
		{
			e2p->Calibrations->TankPsensorMaxPressureVoltage -= 1;

			if (e2p->Calibrations->TankPsensorMaxPressureVoltage < 0) e2p->Calibrations->TankPsensorMaxPressureVoltage = 0;
			break;
		}

		// Увеличение максимальной точки накопителя воды
		case VoltageForPmaxTankInc:
		{
			e2p->Calibrations->TankPsensorMaxPressureVoltage += 1;

			if (e2p->Calibrations->TankPsensorMaxPressureVoltage > DEST_PRESSURE_MAX_VALUE)
			{
				e2p->Calibrations->TankPsensorMaxPressureVoltage = DEST_PRESSURE_MAX_VALUE;
			}
			break;
		}

		// Установка минимальной точки источника воды по текущему значению напряжения датч. давления****
		case SetVoltageForPminSource:
		{
			// Треб. преобр в %
			e2p->Calibrations->SourcePsensorMinPressureVoltage = sysState->WellWaterLevelInVolts;

			if (e2p->Calibrations->SourcePsensorMinPressureVoltage < 0) e2p->Calibrations->SourcePsensorMinPressureVoltage = 0;
			break;
		}
		// Установка максимальной точки источника воды по текущему значению напряжения датч. давления
		case SetVoltageForPmaxSource:
		{
			// Треб. преобр в %
			e2p->Calibrations->SourcePsensorMaxPressureVoltage = sysState->WellWaterLevelInVolts;

			if (e2p->Calibrations->SourcePsensorMaxPressureVoltage > 100) e2p->Calibrations->SourcePsensorMaxPressureVoltage = 100;
			break;
		}

		// Установка минимальной точки накопителя воды по текущему значению напряжения датч. давления****
		case SetVoltageForPminTank:
		{
			// Треб. преобр в %
			e2p->Calibrations->TankPsensorMinPressureVoltage = sysState->TankWaterLevelInVolts;

			if (e2p->Calibrations->TankPsensorMinPressureVoltage < 0) e2p->Calibrations->TankPsensorMinPressureVoltage = 0;
			break;
		}
		// Установка максимальной точки накопителя воды по текущему значению напряжения датч. давления
		case SetVoltageForPmaxTank:
		{
			// Треб. преобр в %
			e2p->Calibrations->TankPsensorMaxPressureVoltage = sysState->TankWaterLevelInVolts;

			if (e2p->Calibrations->TankPsensorMaxPressureVoltage > 100) e2p->Calibrations->TankPsensorMaxPressureVoltage = 100;
			break;
		}

		// Экран 0
		case Screen0:
		{
			nextion.ScreenNumber = 0;
			break;
		}
		// Экран 1
		case Screen1:
		{
			nextion.ScreenNumber = 1;
			break;
		}
		// Экран 2
		case Screen2:
		{
			nextion.ScreenNumber = 2;
			break;
		}
		// Экран 3
		case Screen3:
		{
			nextion.ScreenNumber = 3;
			break;
		}
		// Экран 4
		case Screen4:
		{
			nextion.ScreenNumber = 4;
			break;
		}
		// Экран 5
		case Screen5:
		{
			nextion.ScreenNumber = 5;
			break;
		}
		// Экран 6
		case Screen6:
		{
			nextion.ScreenNumber = 6;
			break;
		}
		// Экран 7
		case Screen7:
		{
			nextion.ScreenNumber = 7;
			break;
		}
		// Экран 8
		case Screen8:
		{
			nextion.ScreenNumber = 8;
			break;
		}
		// Экран 9
		case Screen9:
		{
			nextion.ScreenNumber = 9;
			break;
		}

		// Вкл. реж. полива при повышенном давлении с огранич. по врем., объёму
		case SpecialWateringModeOn:
		{
			if (source_value == key_is_pressed) {
				// Если не активен какой-либо режим работы
				if(*sysState->pumpCurrState == IdleMode) {
					// Если установлено давление защитного отключения для спец. режима полива
					if(e2p->Calibrations->SpModePumpOffPressure) {
						// Если установлен объём для полива,
						if((e2p->Calibrations->SpModeWateringVolume) ||
							 // либо установлена продолжительность полива
							 (e2p->Calibrations->SpModeWateringTime)) {
							// Включ. режима спец. полива
							sysState->pumpCtrlComms->SwitchPumpOn = 1;
							*sysState->pumpCurrState = SpecialWateringMode;
							
							// Начальная установка счётчиков
							e2p->LastPumpCycle->SpModeWateringTimer = e2p->Calibrations->SpModeWateringTime;
							e2p->LastPumpCycle->SpModeWateringVolumeCounter = e2p->Calibrations->SpModeWateringVolume;
								 
							// Обнуление счётчиков значений последнего цикла
							e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle = 0;
							e2p->LastPumpCycle->WaterPumpedAtLastCycle = 0;
						}
					}
				}
			}

			break;
		}
		// Выкл. реж. полива при повышенном давлении с огранич. по врем., объёму
		case SpecialWateringModeOff:
		{
			if (source_value == key_is_pressed) {
				// Если в режиме прогрева УФ лампы, индицируем
				if(*sysState->uvLampState == uvLampPreheating) {
					*sysState->uvLampState = uvLampBlinkWhilePreheating;
				}
			
				// Если не в режиме прогрева УФ лампы
				if((*sysState->uvLampState != uvLampPreheating) &&
					 (*sysState->uvLampState != uvLampBlinkWhilePreheating)) {
					// Выключить режим спец. полива
					sysState->pumpCtrlComms->SwitchPumpOff = 1;
				}
					 
				// Если кнопка выкл. была нажата при активном событии "сухого хода", то
				if (sysState->DryRunProtectiveStop)
				{
					// сброс события "сухого хода"
					sysState->DryRunProtectiveStop = 0;
					
					// Возобновление автоподкачки, если была прервана "сухим ходом"
					if (*sysState->pumpCurrState == AutoPumpingMode)
					{
						// Отключение автоподкачки
						//e2p->LastPumpCycle->auto_pump_is_done = 0;
						
						// Включаем насос
						sysState->pumpCtrlComms->SwitchPumpOn = 1;
						sysState->pumpCtrlComms->SwitchPumpOff = 0;
					}
				}
				// Если событие "сухой ход" неактивно, то
				else
				{
					// Если кнопка выкл. была нажата при штатной работе автоподкачки, то
					if (*sysState->pumpCurrState == AutoPumpingMode)
					{
						// Разрешение повторной попытки автоподкачки воды
						*sysState->pumpCurrState = IdleMode;						
					}
				}
			}

			break;
		}

		// Увеличение объёма полива в реж. полива с огранич. по врем., объёму
		case SpModeWateringVolumeInc:
		{
			if (large_step == 0)			e2p->Calibrations->SpModeWateringVolume += 10;
			else if (large_step == 1)	e2p->Calibrations->SpModeWateringVolume += 10;
			else if (large_step == 2)	e2p->Calibrations->SpModeWateringVolume += 10;
			else if (large_step == 3)	e2p->Calibrations->SpModeWateringVolume += 100;
			if (e2p->Calibrations->SpModeWateringVolume > 30000) e2p->Calibrations->SpModeWateringVolume = 0;		
			break;
		}
		// Уменьшение объёма полива в реж. полива с огранич. по врем., объёму
		case SpModeWateringVolumeDec:
		{
			if (large_step == 0)			e2p->Calibrations->SpModeWateringVolume -= 10;
			else if (large_step == 1)	e2p->Calibrations->SpModeWateringVolume -= 10;
			else if (large_step == 2)	e2p->Calibrations->SpModeWateringVolume -= 10;
			else if (large_step == 3)	e2p->Calibrations->SpModeWateringVolume -= 100;
			if (e2p->Calibrations->SpModeWateringVolume < 0) e2p->Calibrations->SpModeWateringVolume = 30000;				
			break;
		}

		// Увеличение времени полива в реж. полива с огранич. по врем., объёму
		case SpModeWateringTimeInc:
		{
			if (large_step == 0)			e2p->Calibrations->SpModeWateringTime += 60;
			else if (large_step == 1)	e2p->Calibrations->SpModeWateringTime += 60;
			else if (large_step == 2)	e2p->Calibrations->SpModeWateringTime += 600;
			else if (large_step == 3)	e2p->Calibrations->SpModeWateringTime += 600;		
			if (e2p->Calibrations->SpModeWateringTime > 36000) e2p->Calibrations->SpModeWateringTime = 0;			
			break;
		}
		// Уменьшение времени полива в реж. полива с огранич. по врем., объёму
		case SpModeWateringTimeDec:
		{
			if (large_step == 0)			e2p->Calibrations->SpModeWateringTime -= 60;
			else if (large_step == 1)	e2p->Calibrations->SpModeWateringTime -= 60;
			else if (large_step == 2)	e2p->Calibrations->SpModeWateringTime -= 600;
			else if (large_step == 3)	e2p->Calibrations->SpModeWateringTime -= 600;		
			if (e2p->Calibrations->SpModeWateringTime < 0) e2p->Calibrations->SpModeWateringTime = 36000;	
			break;
		}

		// Увеличение давления отключения насоса в реж. полива с огранич. по врем., объёму
		case SpModePumpOffPressureInc:
		{
			if (large_step == 0)			e2p->Calibrations->SpModePumpOffPressure += 1;
			else if (large_step == 1)	e2p->Calibrations->SpModePumpOffPressure += 1;
			else if (large_step == 2)	e2p->Calibrations->SpModePumpOffPressure += 1;
			else if (large_step == 3)	e2p->Calibrations->SpModePumpOffPressure += 10;	
			if (e2p->Calibrations->SpModePumpOffPressure > PRESSURE_MAX_VALUE)
				e2p->Calibrations->SpModePumpOffPressure = PRESSURE_MAX_VALUE;
			break;
		}
		// Уменьшение давления отключения насоса в реж. полива с огранич. по врем., объёму
		case SpModePumpOffPressureDec:
		{
			if (large_step == 0)			e2p->Calibrations->SpModePumpOffPressure -= 1;
			else if (large_step == 1)	e2p->Calibrations->SpModePumpOffPressure -= 1;
			else if (large_step == 2)	e2p->Calibrations->SpModePumpOffPressure -= 1;
			else if (large_step == 3)	e2p->Calibrations->SpModePumpOffPressure -= 10;				
			if (e2p->Calibrations->SpModePumpOffPressure < 0)
				e2p->Calibrations->SpModePumpOffPressure = 0;			
			break;
		}
		
		// Увеличение кол-ва счётных импульсов на 1 литр при учёте перекачиваемой жидкости
		case ImpulsesPerOneLiterInc:
		{
			if (large_step == 0)			e2p->Calibrations->TurbineImpulsesPerLiter += 1;
			else if (large_step == 1)	e2p->Calibrations->TurbineImpulsesPerLiter += 10;
			else if (large_step == 2)	e2p->Calibrations->TurbineImpulsesPerLiter += 10;
			else if (large_step == 3)	e2p->Calibrations->TurbineImpulsesPerLiter += 100;
			if (e2p->Calibrations->TurbineImpulsesPerLiter > 10000) e2p->Calibrations->TurbineImpulsesPerLiter = 0;				
			break;
		}
		// Уменьшение кол-ва счётных импульсов на 1 литр при учёте перекачиваемой жидкости
		case ImpulsesPerOneLiterDec:
		{
			if (large_step == 0)			e2p->Calibrations->TurbineImpulsesPerLiter -= 1;
			else if (large_step == 1)	e2p->Calibrations->TurbineImpulsesPerLiter -= 10;
			else if (large_step == 2)	e2p->Calibrations->TurbineImpulsesPerLiter -= 10;
			else if (large_step == 3)	e2p->Calibrations->TurbineImpulsesPerLiter -= 100;
			if (e2p->Calibrations->TurbineImpulsesPerLiter < 0) e2p->Calibrations->TurbineImpulsesPerLiter = 10000;				
			break;
		}
		
		// Увеличение кол-ва литров на 1 счётный импульс при учёте перекачиваемой жидкости
		case LitersPerOneImpulseInc:
		{
			if (large_step == 0)			e2p->Calibrations->WaterCounterLitersPerImpulse += 1;
			else if (large_step == 1)	e2p->Calibrations->WaterCounterLitersPerImpulse += 10;
			else if (large_step == 2)	e2p->Calibrations->WaterCounterLitersPerImpulse += 10;
			else if (large_step == 3)	e2p->Calibrations->WaterCounterLitersPerImpulse += 100;
			if (e2p->Calibrations->WaterCounterLitersPerImpulse > 10000) e2p->Calibrations->WaterCounterLitersPerImpulse = 0;				
			break;
		}
		// Уменьшение кол-ва литров на 1 счётный импульс при учёте перекачиваемой жидкости
		case LitersPerOneImpulseDec:
		{
			if (large_step == 0)			e2p->Calibrations->WaterCounterLitersPerImpulse -= 1;
			else if (large_step == 1)	e2p->Calibrations->WaterCounterLitersPerImpulse -= 10;
			else if (large_step == 2)	e2p->Calibrations->WaterCounterLitersPerImpulse -= 10;
			else if (large_step == 3)	e2p->Calibrations->WaterCounterLitersPerImpulse -= 100;
			if (e2p->Calibrations->WaterCounterLitersPerImpulse < 0) e2p->Calibrations->WaterCounterLitersPerImpulse = 10000;				
			break;
		}
		
		// Сброс счётчиков УФ лампы
		case ResetUVLampCounters:
		{
			e2p->Statistics->UvLampWorkingTime = 0;
			e2p->Statistics->UvLampPowerOnCycleCounter = 0;

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


// Adding triple 0xFF at the end of the command as command termination and pushing to ring buffer for sending
ReturnCode_t Add_termination_to_nextion_command_and_push_to_ring_buf(NextionComPort_t * nextion)
{
	ReturnCode_t func_res;
	uint32_t size;

	// Checking buffer boundary
	if((nextion->Com->TxdIdx8 + 3) > NEXTION_COM_TXD_BUF_SIZE_IN_BYTES)
	{
		return StringLengthExceedsBufferSize;
	}
	
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 0xFF;
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 0xFF;
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 0xFF;
	
	// Setting string lenght for sending as whole unit
	nextion->Com->TxdPacketLenght8 = nextion->Com->TxdIdx8;
	
	// Pushing data string to ring data buffer
	if((func_res = Push_string_to_ring_buffer(nextion->ComRingBuf, nextion->TxdBuffer, nextion->Com->TxdPacketLenght8)))
	{
		return func_res;
	}		
	
	return OK;
}


// Отрисовка на Nextion текущего значения параметров
ReturnCode_t Prepare_params_and_send_to_nextion(RTC_HandleTypeDef  * hrtc, E2p_t * e2p, NextionComPort_t * nextion, CurrentSystemState_t * sysState)
{
	ReturnCode_t					func_res;
	HAL_StatusTypeDef			HAL_func_res;
	uint8_t								ascii_buf[5];
	int32_t								temp_int32;
	int32_t 							time_temp;
	
	// Preventing corruption of sending data
	//if(nextion->Com->TxdPacketIsSent == 0) return;
	
	// Яркость дисплея
	nextion->Com->TxdIdx8 = 0;
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'd';
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'i';
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'm';
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
	// Яркость дисплея
	Hex2Dec2ASCII((uint16_t) (display_brightness / DISPLAY_BRIGHTNESS_OFF_SPEED), ascii_buf, sizeof(ascii_buf));	
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
	nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
	// Терминатор команды + отправка в кольцевой буфер на передачу
	if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

	// Чтение текущего времени
	time_temp = Get_time_in_sec(hrtc);

	switch(nextion->ScreenNumber)
	{
		// Страница 0 (Главный экран)
		case 0:
		{
			// Давление воды в системе, атм * 10
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) sysState->AverageWaterPressure, ascii_buf, sizeof(ascii_buf));
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Время работы насоса в последнем цикле, час
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle / 3600), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
		
			// Время работы насоса в последнем цикле, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Время работы насоса в последнем цикле, сек
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
				

			// Кол-во воды, перекачанной в последнем цикле
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Кол-во воды, перекачанной насосом в последнем цикле, л (старшие 3 разряда)
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->WaterPumpedAtLastCycle / 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Кол-во воды, перекачанной насосом в последнем цикле, л (младшие 3 разряда)
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->WaterPumpedAtLastCycle % 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// t воды при перекач
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// t воды при перекачивании, 'С * 10
			Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->LastPumpCycle->WaterTempDuringPumping)), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Текущ. t воды
			nextion->Com->TxdIdx8 = 0;	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// текущая t воды, 'С * 10
			Hex2Dec2ASCII((uint16_t) (fabs((float) sysState->CurrentWaterTemp)), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
			

			// Текущее время, часы
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Обновление счётчика времени в секундах
			sysState->TimeInSeconds = time_temp;
			// Текущее время, часы
			Hex2Dec2ASCII((uint16_t) (sysState->TimeInSeconds / 3600), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Текущее время, минуты
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Текущее время, минуты
			Hex2Dec2ASCII((uint16_t) ((sysState->TimeInSeconds % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Текущее время, секунды
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Текущее время, секунды
			Hex2Dec2ASCII((uint16_t) ((sysState->TimeInSeconds % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// t воды в источнике, 'С
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// t воды в источнике, 'С
			Hex2Dec2ASCII((uint16_t) (fabs((float) sysState->WellWaterTemp)), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Уровень воды в источнике, %
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Объём воды в источнике, проценты
			Hex2Dec2ASCII((uint16_t) sysState->WellWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Графический уровень воды в источнике, %
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'j';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Графический уровень воды в источнике, проценты
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// t воды в накопителе, 'С
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// t воды в накопителе, 'С
			Hex2Dec2ASCII((uint16_t) (fabs((float) sysState->TankWaterTemp)), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Уровень воды в накопителе, %
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Объём воды в накопителе, проценты
			Hex2Dec2ASCII((uint16_t) sysState->TankWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Графический уровень воды в накопителе, %
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'j';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Графический уровень воды в накопителе, проценты
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Сокрытие/отрисовка сообщения "сухой ход"
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 't';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'p';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'c';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'o';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Скрытие/отрисовка сообщения "сухой ход"
			if (sysState->DryRunProtectiveStop)
			{
				// Отрисовка сообщения "сухой ход"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
			}
			else
			{
				// Сокрытие сообщения "сухой ход"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			}
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			
			// Скрытие/отрисовка сообщения "Автополив"
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 't';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'p';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'c';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'o';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Скрытие/отрисовка сообщения "Автополив"
			if (sysState->AutoWatering)
			{
				// Отрисовка сообщения "Автополив"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			}
			else
			{
				// Скрытие сообщения "Автополив"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			}
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Скрытие/отрисовка сообщения "Автоподкачка"
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 't';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'p';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'c';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'o';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Скрытие/отрисовка сообщения "Автоподкачка"
			if (*sysState->pumpCurrState == AutoPumpingMode)
			{
				// Отрисовка сообщения "Автоподкачка"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
			}
			else
			{
				// Скрытие сообщения "Автоподкачка"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			}
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
			
			// Скрытие/отрисовка сообщения "Спец. полив"
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 't';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'p';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'c';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'o';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Скрытие/отрисовка сообщения "Спец. полив"
			if (*sysState->pumpCurrState == SpecialWateringMode)
			{
				// Отрисовка сообщения "Спец. полив"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			}
			else
			{
				// Скрытие сообщения "Спец. полив"
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
				nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			}
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// изменение цвета кнопки включения насоса
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'b';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'b';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'c';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'o';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			if(*sysState->pumpCurrState != SpecialWateringMode)
			{
				// Если УФ лампа включена на прогрев, то кнопка вкл. насоса имеет фиолетовый цвет
				if (*sysState->uvLampState == uvLampPreheating)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
				}
				// Если УФ лампа включена на прогрев и есть попытка отключения, то подсвечиваем кнопку вкл. насоса
				else if (*sysState->uvLampState == uvLampBlinkWhilePreheating)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					
					*sysState->uvLampState = uvLampPreheating;
				}
				// Если насос включен, то кнопка вкл. насоса имеет салатовый цвет
				else if (sysState->PumpIsStarted)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
				}
				else
				{
					// Если насос выключен, то кнопка вкл. насоса имеет оранжевый цвет
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
				}
			}
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
			

			// Время включения насоса в последнем цикле, час
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Время включения насоса в последнем цикле, час
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->PumpStartTimeAtLastCycle / 3600), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
				
			// Время включения насоса в последнем цикле, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Время включения насоса в последнем цикле, мин
			Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->PumpStartTimeAtLastCycle % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Время включения насоса в последнем цикле, сек
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Время включения насоса в последнем цикле, сек
			Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->PumpStartTimeAtLastCycle % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			break;
		}

		// Страница 1 (настройки насоса)
		case 1:
		{
			// P вкл. насоса, атм * 10
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Давление включения насоса, атм * 10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PumpOnPressure, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// P выкл. насоса, атм * 10
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Давление выключения насоса, атм * 10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PumpOffPressure, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Калибровка датчика давления: P min, атм * 10
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Калибровка датчика давления: P min, атм * 10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMinPressure, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Калибровка датчика давления: P max, атм * 10
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Калибровка датчика давления: P max, атм * 10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMaxPressure, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Калибровка датчика давления: P min = U min, мВ * 100
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Напряжение (0 - 5В), соотв. минимальному давлению датчика давления, мВ * 100
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMinPressureVoltage, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Калибровка датчика давления: P max = U max, мВ * 100
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления, мВ * 100
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->PsensorMaxPressureVoltage, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Таймаут срабатывания останова насоса по "сухому ходу"
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->PumpDryRunStopTimeout, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			break;
		}

		// Страница 2 (статистика)
		case 2:
		{
			// Общее время работы контроллера, часов.минут
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Общее время работы контроллера, часов (старшие 8-6 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalControllerWorkingTime / 3600) / 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее время работы контроллера, часов (средние 5-3 разряды)
			Hex2Dec2ASCII((uint16_t) (((e2p->Statistics->TotalControllerWorkingTime / 3600) % 10000) % 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее время работы контроллера, минут (младшие 2-1 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalControllerWorkingTime / 60) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Общее время работы насоса, часов.минут
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Общее время работы насоса, часов (старшие 8-6 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalPumpWorkingTime / 3600) / 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее время работы насоса, часов (средние 5-3 разряды)
			Hex2Dec2ASCII((uint16_t) (((e2p->Statistics->TotalPumpWorkingTime / 3600) % 10000) % 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее время работы контроллера, минут (младшие 2-1 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->TotalPumpWorkingTime / 60) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Общее кол-во перекачанной насосом воды, литры
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Общее кол-во воды, перекачанной насосом, литры (старшие 5-8 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->WaterPumpedTotal / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее кол-во воды, перекачанной насосом, литры  (младшие 1-4 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->WaterPumpedTotal % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Кол-во перекачанной воды за сутки, литры
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';		
			// Общее кол-во воды, перекачанной за сутки, литры (старшие 5-8 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityToday / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее кол-во воды, перекачанной за сутки, литры  (младшие 1-4 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityToday % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Кол-во перекачанной воды за неделю, литры*10  (десятки литров)
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';	
			// Общее кол-во воды, перекачанной за неделю, литры (старшие 5-8 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityLastWeek / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее кол-во воды, перекачанной за неделю, литры  (младшие 1-4 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->PumpedWaterQuantityLastWeek % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Общее время работы УФ лампы, часов.минут
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Общее время работы УФ лампы, часов (старшие 8-6 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->UvLampWorkingTime / 3600) / 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее время работы УФ лампы, часов (средние 5-3 разряды)
			Hex2Dec2ASCII((uint16_t) (((e2p->Statistics->UvLampWorkingTime / 3600) % 10000) % 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Общее время работы УФ лампы, минут (младшие 2-1 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Statistics->UvLampWorkingTime / 60) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Кол-во циклов включения УФ лампы
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';	
			// Кол-во циклов включения УФ лампы, (старшие 8-5 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->UvLampPowerOnCycleCounter / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Кол-во циклов включения УФ лампы, (младшие 4-1 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Statistics->UvLampPowerOnCycleCounter % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			break;
		}

		// Страница 3 (автополив)
		case 3:
		{
			// Выбор выхода 1-6
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Отображение номера выхода 1-6
			Hex2Dec2ASCII((uint16_t) e2p->WateringControls->CurrWateringOutputNumber, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Смещение от начала суток, час, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Значение смещения времени включения полива зоны 1-6 относительно начала суток, час
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			temp_int32 = e2p->WateringControls->out1_zero_clock_time_delta;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) temp_int32 = e2p->WateringControls->out2_zero_clock_time_delta;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) temp_int32 = e2p->WateringControls->out3_zero_clock_time_delta;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) temp_int32 = e2p->WateringControls->out4_zero_clock_time_delta;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) temp_int32 = e2p->WateringControls->out5_zero_clock_time_delta;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) temp_int32 = e2p->WateringControls->out6_zero_clock_time_delta;
			Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Значение смещения времени включения полива зоны 1-6 относительно начала суток, мин
			Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Время работы, час, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Значение времени работы полива зоны 1-6, час
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			temp_int32 = e2p->WateringControls->out1_working_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) temp_int32 = e2p->WateringControls->out2_working_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) temp_int32 = e2p->WateringControls->out3_working_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) temp_int32 = e2p->WateringControls->out4_working_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) temp_int32 = e2p->WateringControls->out5_working_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) temp_int32 = e2p->WateringControls->out6_working_time;
			Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Значение времени работы полива зоны 1-6, мин
			Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Периодичность включения, час, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// пнтервал времени между включениями полива зоны 1-6, час
			if (e2p->WateringControls->CurrWateringOutputNumber == 1) 			temp_int32 = e2p->WateringControls->out1_interval_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 2) temp_int32 = e2p->WateringControls->out2_interval_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 3) temp_int32 = e2p->WateringControls->out3_interval_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 4) temp_int32 = e2p->WateringControls->out4_interval_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 5) temp_int32 = e2p->WateringControls->out5_interval_time;
			else if (e2p->WateringControls->CurrWateringOutputNumber == 6) temp_int32 = e2p->WateringControls->out6_interval_time;
			Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// пнтервал времени между включениями полива зоны 1-6, мин
			Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
			
			break;
		}

		// Страница 4 (Ежесуточная автоподкачка воды)
		case 4:
		{
			// �?нтервал времени между включениями автоподкачки, час, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// �?нтервал времени между включениями автоподкачки, час
			temp_int32 = e2p->LastPumpCycle->AutoPumpTimeInterval;
			Hex2Dec2ASCII((uint16_t) (temp_int32 / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// пнтервал времени между включениями автоподкачки, мин
			Hex2Dec2ASCII((uint16_t) (temp_int32 % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Ежесуточное автоподкачивание воды: 
			// Смещение от начала суток, час, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Значение смещения времени включения автоподкачивания относительно начала суток, час
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Значение смещения времени включения автоподкачивания относительно начала суток, мин
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Подкачиваемый объём, л
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Объём подкачиваемой воды, л
			Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->AutoPumpVolume, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Кол-во включений автоподкачивания за сутки
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) e2p->LastPumpCycle->AutoPumpTimes, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
		
			break;
		}

		// Страница 5 (Настройки источника, накопителя)
		case 5:
		{
			// Калибровка датчика давления источника воды: P min = U min, В/100
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Калибровка датчика давления источника воды: P min = U min, В/10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->SourcePsensorMinPressureVoltage, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Калибровка датчика давления источника воды: P max = U max, В/100
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Калибровка датчика давления источника воды: P max = U max, В/10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->SourcePsensorMaxPressureVoltage, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Калибровка датчика давления накопителя воды: P min = U min, В/100
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Калибровка датчика давления накопителя воды: P min = U min, В/10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->TankPsensorMinPressureVoltage, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Калибровка датчика давления накопителя воды: P max = U max, В/100
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Калибровка датчика давления накопителя воды: P max = U max, В/10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->TankPsensorMaxPressureVoltage, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Текущий уровень воды в источнике, в вольтах/100 датч. давл.
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Текущий уровень воды в источнике, в вольтах/10 датч. давл.
			Hex2Dec2ASCII((uint16_t) sysState->WellWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Текущий уровень воды в накопителе, в вольтах/100 датч. давл.
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Текущий уровень воды в накопителе, в вольтах/10 датч. давл.
			Hex2Dec2ASCII((uint16_t) sysState->TankWaterLevelInVolts, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
			
			break;
		}
		
		// Страница 7 (Общие настройки)
		case 7:
		{
			// Установка времени, часы
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Обновление счётчика времени в секундах
			sysState->TimeInSeconds = time_temp;
			// Установка времени, часы
			Hex2Dec2ASCII((uint16_t) (sysState->TimeInSeconds / 3600), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Установка времени, минуты
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Установка времени, минуты
			Hex2Dec2ASCII((uint16_t) ((sysState->TimeInSeconds % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Корр. времени, сек/неделя
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Коррекция времени, сек/неделя
			Hex2Dec2ASCII((uint16_t) (fabs((float) e2p->Calibrations->TimeCorrectionValue)), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Сокрытие/отрисовка символа минус "-"
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'i';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 's';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ' ';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 't';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ',';
			// Отображение знака значения коррекции времени
			if (e2p->Calibrations->TimeCorrectionValue < 0)
			{
			// Рисуем минус "-"
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			}
			else
			{
			// Прячем минус "-"
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';	
			}
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			break;
		}

		// Страница 8 (Специальный режим полива)
		case 8:
		{
			// P защитного отключения насоса в режиме спец. полива, атм * 10
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// P защитного отключения насоса в режиме спец. полива, атм * 10
			Hex2Dec2ASCII((uint16_t) e2p->Calibrations->SpModePumpOffPressure, ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];	
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			
			// Уставка времени работы насоса в спец. режиме полива, часов.минут
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Уставка времени работы насоса в спец. режиме полива, часов (старшие 8-6 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Calibrations->SpModeWateringTime / 3600) / 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Уставка времени работы насоса в спец. режиме полива, часов (средние 5-3 разряды)
			Hex2Dec2ASCII((uint16_t) (((e2p->Calibrations->SpModeWateringTime / 3600) % 10000) % 1000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Уставка времени работы насоса в спец. режиме полива, минут (младшие 2-1 разряды)
			Hex2Dec2ASCII((uint16_t) ((e2p->Calibrations->SpModeWateringTime / 60) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Объём полива в спец. режиме полива с огранич. по врем., объёму, л
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Объём полива в спец. режиме полива с огранич. по врем., объёму, литры (старшие 5-8 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Calibrations->SpModeWateringVolume / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Объём полива в спец. режиме полива с огранич. по врем., объёму, литры  (младшие 1-4 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->Calibrations->SpModeWateringVolume % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
			
			
			// Таймер времени работы насоса в спец. режиме полива, часов
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->SpModeWateringTimer / 3600), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;
		
			// Таймер времени работы насоса в спец. режиме полива, мин
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->SpModeWateringTimer % 3600) / 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			// Таймер времени работы насоса в спец. режиме полива, сек
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'n';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			Hex2Dec2ASCII((uint16_t) ((e2p->LastPumpCycle->SpModeWateringTimer % 3600) % 60), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Текущий объём перекачанной жидкости в спец. режиме полива с огранич. по врем., объёму, л
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// Текущий объём перекачанной жидкости в спец. режиме полива с огранич. по врем., объёму, литры (старшие 5-8 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->SpModeWateringVolumeCounter / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Текущий объём перекачанной жидкости в спец. режиме полива с огранич. по врем., объёму, литры  (младшие 1-4 разряды)
			Hex2Dec2ASCII((uint16_t) (e2p->LastPumpCycle->SpModeWateringVolumeCounter % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// изменение цвета кнопки включения насоса
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'b';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'b';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'c';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'o';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			if(*sysState->pumpCurrState == SpecialWateringMode)
			{
				// Если УФ лампа включена на прогрев, то кнопка вкл. насоса имеет фиолетовый цвет
				if (*sysState->uvLampState == uvLampPreheating)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
				}
				// Если УФ лампа включена на прогрев и есть попытка отключения, то подсвечиваем кнопку вкл. насоса
				else if (*sysState->uvLampState == uvLampBlinkWhilePreheating)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					
					*sysState->uvLampState = uvLampPreheating;
				}
				// Если насос включен, то кнопка вкл. насоса имеет салатовый цвет
				else if (sysState->PumpIsStarted)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '3';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '7';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '8';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
				}
			}
			else
			{
				// Если "сухой ход", то кнопка вкл. насоса имеет красный цвет
				if (sysState->DryRunProtectiveStop)
				{
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
				}
				// Если насос выключен,
				else if (sysState->PumpIsStarted == 0) {
					// то кнопка вкл. насоса имеет оранжевый цвет
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '6';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '4';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '5';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
					nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '2';
				}
			}

			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			break;
		}

		// Страница 9 (Настройки учёта перекачиваемой жидкости)
		case 9:
		{
			// Кол-во импульсов турбины, увеличивающих счётчик литров на 1
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// старшие 5-8 разряды
			Hex2Dec2ASCII((uint16_t) (e2p->Calibrations->TurbineImpulsesPerLiter / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// младшие 1-4 разряды
			Hex2Dec2ASCII((uint16_t) (e2p->Calibrations->TurbineImpulsesPerLiter % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;


			// Кол-во литров счётчика воды на 1 импульс с его выхода
			nextion->Com->TxdIdx8 = 0;
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'x';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '9';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '0';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '1';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '.';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'v';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'a';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = 'l';
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = '=';
			// старшие 5-8 разряды
			Hex2Dec2ASCII((uint16_t) (e2p->Calibrations->WaterCounterLitersPerImpulse / 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// младшие 1-4 разряды
			Hex2Dec2ASCII((uint16_t) (e2p->Calibrations->WaterCounterLitersPerImpulse % 10000), ascii_buf, sizeof(ascii_buf));	
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[3];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[2];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[1];
			nextion->TxdBuffer[nextion->Com->TxdIdx8++] = ascii_buf[0];
			// Терминатор команды + отправка в кольцевой буфер на передачу
			if((func_res = Add_termination_to_nextion_command_and_push_to_ring_buf(nextion))) return func_res;

			break;
		}
	}
	
	return OK;
}


// Обработчик принятого пакета по COM2 из дисплея Nextion
ReturnCode_t Nextion_received_data_handler(RTC_HandleTypeDef  * hrtc, E2p_t * e2p)
{
	ReturnCode_t func_res;
	
	// Проверка целостности и к.с. принятой по com строки данных
	func_res = Check_received_nextion_packet(nextion.RxdBuffer, com2.RxdPacketLenght8);
	if (func_res == OK)
	{		
		// После контроля правильности и выполнения отключаем индикатор приёма данных
		//LED1_OFF;

		// Счётчик правильно принятых пакетов данных
		com2.RxdGoodPacketsCounter++;
			
		// Разбор принятой строки от дисплея Nextion
		Parsing_nextion_display_string(hrtc, e2p, nextion.RxdBuffer, com2.RxdPacketLenght8, com2.RxdPacketIsReceived, &sysState);
	}	

	else com2.RxdPacketsErrorCounter++;
	
	return func_res;
}


// Проверка принятой по com2 строки данных
ReturnCode_t Check_received_nextion_packet(uint8_t * buf, uint16_t lenght)
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
uint32_t Get_jetson_command(JetsonComPort_t * jetson)
{
	uint8_t idx = 1;
	uint32_t command;

	memcpy(&command, jetson->RxdBuffer + idx, sizeof(command));
	// Swapping order of 4 bytes
	Buffer_bytes_swap((uint8_t *) &command, sizeof(uint32_t));
	
	return command;
}


// Handles data from JCB
ReturnCode_t Jetson_rxd_handler(CRC_HandleTypeDef * hcrc, JetsonComPort_t * jetson)
{
	ReturnCode_t	func_res;
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
ReturnCode_t External_if_rxd_handler(CRC_HandleTypeDef * hcrc, NextionComPort_t * nextion)
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
ReturnCode_t Com_rxd_handler(CRC_HandleTypeDef * hcrc, ComNum_t ComNum, JetsonComPort_t * jetson, NextionComPort_t * nextion)
{
	ReturnCode_t func_stat;

	switch(ComNum)
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





// Checking time to switch on pump if matched
uint8_t Switch_on_pump_by_time(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	uint16_t	auto_pump_times = 0;
	uint32_t current_time_in_sec, time_sum = 0;
	
	// Текущее время в секундах
	current_time_in_sec = sysState->TimeInSeconds;
	
	while((time_sum < 86400) && (auto_pump_times < e2p->LastPumpCycle->AutoPumpTimes))
	{
		// Начальная точка счёта - смещение от начала суток
		time_sum = e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay * 60;

		// Checks time match point
		time_sum += e2p->LastPumpCycle->AutoPumpTimeInterval * 60 * auto_pump_times;
		
		if(current_time_in_sec == time_sum)
		{
			// Switch on auto pumping by time matching
			return 0x01;
		}
		auto_pump_times++;
	}
	
	// Do not switch on auto pumping
	return 0x00;
}

// Управление системой
void SysControlLogic(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static uint32_t	uv_lamp_preheating_on_time = 0;
	static int32_t	time_in_seconds_prev = 0;
	static uint32_t	auto_pump_counter_start_point = 0;
	static uint32_t	pump_on_by_pressure_delay_timer = 0;
	static uint32_t	pump_off_by_pressure_delay_timer = 0;
	static uint8_t	pump_on_by_pressure_delay_timer_is_set = 0;
	static uint8_t	pump_off_by_pressure_delay_timer_is_set = 0;
			
	// Проверка смены суток для сброса события "останов по сухому ходу"
	if ((time_in_seconds_prev > 0) && (sysState->TimeInSeconds == 0)) {
		if(sysState->DryRunProtectiveStop) {
			// сброс события "сухого хода" при смене суток,
			sysState->DryRunProtectiveStop = 0;
			// Разрешение повторной попытки автоподкачки воды при смене суток
			*sysState->pumpCurrState = IdleMode;
		}
	}
	time_in_seconds_prev = sysState->TimeInSeconds;
	
	// Включение по автоподкачке************************************************************************
	// Если не активен спец. режим полива
	if(*sysState->pumpCurrState != SpecialWateringMode) {
		// Проверка на необходимость включения/выключения насоса по наличию какого-либо кол-ва литров для накачки
		if (e2p->LastPumpCycle->AutoPumpVolume) {
			// Включение насоса, если счётчик циклов автоподкачки за сутки > 0
			if(e2p->LastPumpCycle->AutoPumpTimes) {
				// Проверка логики разрешения включения насоса, != 0 включение не запрещено
				if(PumpOnPreventiveLogicChecking(e2p, sysState)) {
					// Проверка на необходимость включения насоса по времени
					if(Switch_on_pump_by_time(e2p, sysState)) {
						// Если автоподкачка не активна
						if (*sysState->pumpCurrState != AutoPumpingMode) {
							// Фиксируем начальную точку счётчика перекачанных литров
							auto_pump_counter_start_point = e2p->Statistics->WaterPumpedTotal;
							
							// Обнуление счётчиков значений последнего цикла
							e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle = 0;
							e2p->LastPumpCycle->WaterPumpedAtLastCycle = 0;
						
							sysState->pumpCtrlComms->SwitchPumpOn = 1;
							*sysState->pumpCurrState = AutoPumpingMode;
						}			
					}
				}
			}
			
			// Если активна автоподкачка
			if (*sysState->pumpCurrState == AutoPumpingMode)
			{
				// Ожидание завершения автоналива по кол-ву литров (либо будет выключено по давлению или сухому ходу)
				if (e2p->Statistics->WaterPumpedTotal >= (auto_pump_counter_start_point + (uint32_t) e2p->LastPumpCycle->AutoPumpVolume))
				{
					// Команда выключения насоса
					sysState->pumpCtrlComms->SwitchPumpOff = 1;
				}
			}
		}
	}

	// Включение по давлению****************************************************************************
	if(*sysState->pumpCurrState != PumpingByPressureMode) {	
		// Включаем насос, если не активен спец. режим полива
		if(*sysState->pumpCurrState != SpecialWateringMode) {
			// Если текущее значение давления воды <= минимального давления датчика давления
			if (sysState->AverageWaterPressure <= e2p->Calibrations->PumpOnPressure) {
				// Проверка логики разрешения включения насоса, != 0 включение не запрещено
				if(PumpOnPreventiveLogicChecking(e2p, sysState)) {
					// Если таймер задержки включения не установлен
					if (pump_on_by_pressure_delay_timer_is_set == 0) {
						pump_on_by_pressure_delay_timer = e2p->Statistics->TotalControllerWorkingTime;
						pump_on_by_pressure_delay_timer_is_set = 1;
					}
					
					if (pump_on_by_pressure_delay_timer_is_set) {
						// Если прошло контрольное время и текущее давление в системе всё также меньше мин. давления включения насоса
						if (e2p->Statistics->TotalControllerWorkingTime >= pump_on_by_pressure_delay_timer + PUMP_ON_DELAY) {			
							pump_on_by_pressure_delay_timer_is_set = 0;

							// Включаем насос по давлению
							sysState->pumpCtrlComms->SwitchPumpOn = 1;
							*sysState->pumpCurrState = PumpingByPressureMode;

							// Обнуление счётчиков значений последнего цикла
							e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle = 0;
							e2p->LastPumpCycle->WaterPumpedAtLastCycle = 0;						
						}
					}
				}
			} else {
					pump_on_by_pressure_delay_timer_is_set = 0;
				}
		}
	}

	// Выключение по давлению*************************************************************************
	if(*sysState->pumpCurrState != SpecialWateringMode) {
		// Если значение максимального давления для выключения насоса установили > 0
		if (e2p->Calibrations->PumpOffPressure > 0) {
			// Если текущее значение давления воды >= максимального значения давления датчика давления для отключения
			if (sysState->AverageWaterPressure >= e2p->Calibrations->PumpOffPressure) {
				// Если таймер задержки отключения не установлен
				if (pump_off_by_pressure_delay_timer_is_set == 0)
				{
					pump_off_by_pressure_delay_timer = e2p->Statistics->TotalControllerWorkingTime;
					pump_off_by_pressure_delay_timer_is_set = 1;
				}
				if (pump_off_by_pressure_delay_timer_is_set)
				{
					// Если прошло контрольное время и текущее давление в системе всё также выше макс. давления выключения насоса
					if (e2p->Statistics->TotalControllerWorkingTime >= pump_off_by_pressure_delay_timer + PUMP_OFF_DELAY)
					{					
						pump_off_by_pressure_delay_timer_is_set = 0;

						// Выключаем насос
						sysState->pumpCtrlComms->SwitchPumpOff = 1;					
					}
				}
			} else {
					pump_off_by_pressure_delay_timer_is_set = 0;
				}
		}
	}

	// Выключение по "сухому ходу"**********
	// Если обнаружено событие "сухого хода"
	if (sysState->DryRunProtectiveStop)
	{
		// Выключаем насос
		sysState->pumpCtrlComms->SwitchPumpOff = 1;
		pump_on_by_pressure_delay_timer_is_set = 0;
		pump_off_by_pressure_delay_timer_is_set = 0;
	}	
	
	// Включение насоса******************
	// Если есть команда включения насоса
	if (sysState->pumpCtrlComms->SwitchPumpOn != 0) {
		if(*sysState->uvLampState == uvLampIsOff) {
			// Включаем УФ лампу на прогрев, индикация включения
			UV_STERILIZER_ON;
			uv_lamp_preheating_on_time = HAL_GetTick();
			e2p->Statistics->UvLampPowerOnCycleCounter++;
			*sysState->uvLampState = uvLampPreheating;
		}
		if((*sysState->uvLampState != uvLampIsOff) && (sysState->PumpIsStarted == 0)) {
			if(HAL_GetTick() - uv_lamp_preheating_on_time >= PUMP_ON_AFTER_UV_LAMP_ON_DELAY) {
				sysState->PumpIsStarted = 1;
				sysState->pumpCtrlComms->SwitchPumpOn = 0;
				// Включаем насос
				WATER_PUMP_ON;
				LED1_ON;

				// Фиксируем время включения насоса
				e2p->LastPumpCycle->PumpStartTimeAtLastCycle = sysState->TimeInSeconds;
				*sysState->uvLampState = uvLampIsOn;
			}
		}
	}

	// Управление насосом в режиме спец. полива
	SpecWateringModePumpOnOff(e2p, sysState);
	
	// Выключение насоса******************
	// Если есть команда выключения насоса
	if (sysState->pumpCtrlComms->SwitchPumpOff != 0) {
		if(sysState->PumpIsStarted) {
			uv_lamp_preheating_on_time = HAL_GetTick();
			sysState->PumpIsStarted = 0;

			// Выключаем насос
			WATER_PUMP_OFF;
			LED1_OFF;
			*sysState->uvLampState = uvLampPreheating;
		}
		// Пауза перед отключением УФ лампы после насоса в 5 сек
		if(HAL_GetTick() - uv_lamp_preheating_on_time >= 5000) {
			UV_STERILIZER_OFF;

			sysState->pumpCtrlComms->SwitchPumpOff = 0;
			
			*sysState->pumpCurrState = IdleMode;
			*sysState->uvLampState = uvLampIsOff;
		}
	}

	// Управление яркостью дисплея для периода, когда насос включен
	if (sysState->PumpIsStarted == 1) {	
		// Display brightness control
		DisplayBrightnessControl();
	}

	// Управление автополивом, зона 1-8
	Watering_on_off(e2p, sysState);
}	


// Проверка логики разрешения включения насоса
uint8_t PumpOnPreventiveLogicChecking(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	// Если режим спец. полива, то игнорируем следующие проверки
	if(*sysState->pumpCurrState == SpecialWateringMode) return 1;
	
	// Если значение минимального давления для включения насоса установили > 0
	if (e2p->Calibrations->PumpOnPressure > 0) {
		// Если значение максимального давления для выключения насоса установили > 0
		if (e2p->Calibrations->PumpOffPressure > 0) {
			// Если максимальное давление отключения насоса > минимального давления включения
			if (e2p->Calibrations->PumpOffPressure > e2p->Calibrations->PumpOnPressure) {
				// Если не было обнаружено событие "сухого хода"
				if (sysState->DryRunProtectiveStop == 0) {
					return 1;
				}
			}
		}
	}
					
	// Препятствуем включению насоса, т.к. не выполнены разрешающие условия
	sysState->pumpCtrlComms->SwitchPumpOn = 0;
	sysState->pumpCtrlComms->SwitchPumpOff = 1;
	*sysState->pumpCurrState = IdleMode;
	
	return 0;
}


// Управление насосом в режиме спец. полива
void SpecWateringModePumpOnOff(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static uint32_t	pump_off_by_pressure_delay_timer = 0;
	static uint8_t	pump_off_by_pressure_delay_timer_is_set = 0;
	
	// Если активен спец. режим полива
	if(*sysState->pumpCurrState == SpecialWateringMode)
	{
		// Выключение по давлению в режиме спец. полива***************************************************
		// Если значение максимального давления для выключения насоса в спец. режиме установили на отличное от нуля
		if (e2p->Calibrations->SpModePumpOffPressure > 0)
		{
			// Если текущее значение давления воды >= давления защитного отключения для режима спец. полива
			if (sysState->AverageWaterPressure >= e2p->Calibrations->SpModePumpOffPressure)
			{
				// Если таймер задержки отключения не установлен
				if (pump_off_by_pressure_delay_timer_is_set == 0)
				{
					pump_off_by_pressure_delay_timer = e2p->Statistics->TotalControllerWorkingTime;
					pump_off_by_pressure_delay_timer_is_set = 1;
				}
				if (pump_off_by_pressure_delay_timer_is_set)
				{
					// Если прошло контрольное время и текущее давление в системе всё также выше макс. давления выключения насоса
					if (e2p->Statistics->TotalControllerWorkingTime >= pump_off_by_pressure_delay_timer + PUMP_OFF_DELAY)
					{					
						pump_off_by_pressure_delay_timer_is_set = 0;

						// Выключаем насос
						sysState->pumpCtrlComms->SwitchPumpOff = 1;
					}
				}
			}
		}

		// Выключение при достижении нуля счётчика литров спец. полива
		if(e2p->LastPumpCycle->SpModeWateringVolumeCounter <= 0) {
			// Команда выключения насоса
			sysState->pumpCtrlComms->SwitchPumpOff = 1;
		}
		
		// Выключение при достижении нуля таймера спец. полива
		if(e2p->LastPumpCycle->SpModeWateringTimer <= 0) {
			// Команда выключения насоса
			sysState->pumpCtrlComms->SwitchPumpOff = 1;
		}
	}
}


// Управление автополивом
void Watering_on_off(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static uint8_t	out1_watering_is_started = 0, out2_watering_is_started = 0;
	static uint8_t	out3_watering_is_started = 0, out4_watering_is_started = 0;
	static uint8_t	out5_watering_is_started = 0, out6_watering_is_started = 0;
	static uint8_t	out1_cycles_counter = 0, out2_cycles_counter = 0;
	static uint8_t	out3_cycles_counter = 0, out4_cycles_counter = 0;
	static uint8_t	out5_cycles_counter = 0, out6_cycles_counter = 0;
	static int32_t	time_in_seconds_prev = 0;

	// Если время работы зоны полива 1 > 0 мин
	if (e2p->WateringControls->out1_working_time)
	{
		// Проверка на необходимость включения автополива по времени
		if (sysState->TimeInSeconds / 60 == (e2p->WateringControls->out1_zero_clock_time_delta + e2p->WateringControls->out1_interval_time * out1_cycles_counter + \
				e2p->WateringControls->out1_working_time * out1_cycles_counter))
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
			if (sysState->TimeInSeconds / 60 >= (e2p->WateringControls->out1_zero_clock_time_delta + e2p->WateringControls->out1_interval_time * out1_cycles_counter+ \
					e2p->WateringControls->out1_working_time * out1_cycles_counter + e2p->WateringControls->out1_working_time))
			{
				// Выключение автополива зоны 1
				WATER_ZONE1_OFF;
				out1_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out1_interval_time)
				{
					// �?нкремент счётчика кол-ва включений автополива за сутки 
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
	if (e2p->WateringControls->out2_working_time)
	{
		// Проверка на необходимость включения автополива по времени
		if (sysState->TimeInSeconds / 60 == (e2p->WateringControls->out2_zero_clock_time_delta + e2p->WateringControls->out2_interval_time * out2_cycles_counter + \
				e2p->WateringControls->out2_working_time * out2_cycles_counter))
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
			if (sysState->TimeInSeconds / 60 >= (e2p->WateringControls->out2_zero_clock_time_delta + e2p->WateringControls->out2_interval_time * out2_cycles_counter + \
					e2p->WateringControls->out2_working_time * out2_cycles_counter + e2p->WateringControls->out2_working_time))
			{
				// Выключение автополива зоны 2
				WATER_ZONE2_OFF;
				out2_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out2_interval_time)
				{
					// �?нкремент счётчика кол-ва включений автополива за сутки 
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
	if (e2p->WateringControls->out3_working_time)
	{
		// Проверка на необходимость включения автополива по времени
		if (sysState->TimeInSeconds / 60 == (e2p->WateringControls->out3_zero_clock_time_delta + e2p->WateringControls->out3_interval_time * out3_cycles_counter + \
				e2p->WateringControls->out3_working_time * out3_cycles_counter))
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
			if (sysState->TimeInSeconds / 60 >= (e2p->WateringControls->out3_zero_clock_time_delta + e2p->WateringControls->out3_interval_time * out3_cycles_counter + \
					e2p->WateringControls->out3_working_time * out3_cycles_counter + e2p->WateringControls->out3_working_time))
			{
				// Выключение автополива зоны 3
				WATER_ZONE3_OFF;
				out3_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out3_interval_time)
				{
					// �?нкремент счётчика кол-ва включений автополива за сутки 
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
	if (e2p->WateringControls->out4_working_time)
	{
		// Проверка на необходимость включения автополива по времени
		if (sysState->TimeInSeconds / 60 == (e2p->WateringControls->out4_zero_clock_time_delta + e2p->WateringControls->out4_interval_time * out4_cycles_counter + \
				e2p->WateringControls->out4_working_time*out4_cycles_counter))
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
			if (sysState->TimeInSeconds / 60 >= (e2p->WateringControls->out4_zero_clock_time_delta + e2p->WateringControls->out4_interval_time * out4_cycles_counter + \
					e2p->WateringControls->out4_working_time * out4_cycles_counter + e2p->WateringControls->out4_working_time))
			{
				// Выключение автополива зоны 4
				WATER_ZONE4_OFF;
				out4_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out4_interval_time)
				{
					// �?нкремент счётчика кол-ва включений автополива за сутки 
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
	if (e2p->WateringControls->out5_working_time)
	{
		// Проверка на необходимость включения автополива по времени
		if (sysState->TimeInSeconds / 60 == (e2p->WateringControls->out5_zero_clock_time_delta + e2p->WateringControls->out5_interval_time * out5_cycles_counter + \
				e2p->WateringControls->out5_working_time * out5_cycles_counter))
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
			if (sysState->TimeInSeconds / 60 >= (e2p->WateringControls->out5_zero_clock_time_delta + e2p->WateringControls->out5_interval_time * out5_cycles_counter + \
					e2p->WateringControls->out5_working_time * out5_cycles_counter + e2p->WateringControls->out5_working_time))
			{
				// Выключение автополива зоны 5
				WATER_ZONE5_OFF;
				out5_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out5_interval_time)
				{
					// �?нкремент счётчика кол-ва включений автополива за сутки 
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
	if (e2p->WateringControls->out6_working_time)
	{
		// Проверка на необходимость включения автополива по времени
		if (sysState->TimeInSeconds / 60 == (e2p->WateringControls->out6_zero_clock_time_delta + e2p->WateringControls->out6_interval_time * out6_cycles_counter + \
				e2p->WateringControls->out6_working_time * out6_cycles_counter))
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
			if (sysState->TimeInSeconds / 60 >= (e2p->WateringControls->out6_zero_clock_time_delta + e2p->WateringControls->out6_interval_time * out6_cycles_counter + \
					e2p->WateringControls->out6_working_time * out6_cycles_counter + e2p->WateringControls->out6_working_time))
			{
				// Выключение автополива зоны 6
				WATER_ZONE6_OFF;
				out6_watering_is_started = 0;

				// Если разрешены повторения циклов автополива
				if (e2p->WateringControls->out6_interval_time)
				{
					// �?нкремент счётчика кол-ва включений автополива за сутки 
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
	
	
	
	// Проверка смены суток для сброса флагов и счётчиков
	if ((time_in_seconds_prev > 0) && (sysState->TimeInSeconds == 0))
	{		
		// Сброс счётчика кол-ва включений автополива за сутки
		out1_cycles_counter = 0;
		out2_cycles_counter = 0;
		out3_cycles_counter = 0;
		out4_cycles_counter = 0;
		out5_cycles_counter = 0;
		out6_cycles_counter = 0;

		out1_watering_is_started = 0;
		out2_watering_is_started = 0;
		out3_watering_is_started = 0;
		out4_watering_is_started = 0;
		out5_watering_is_started = 0;
		out6_watering_is_started = 0;

		// Принудительное отключение автополива
		WATER_ZONE1_OFF;
		WATER_ZONE2_OFF;
		WATER_ZONE3_OFF;
		WATER_ZONE4_OFF;
		WATER_ZONE5_OFF;
		WATER_ZONE6_OFF;
	}

	time_in_seconds_prev = sysState->TimeInSeconds;
	
	// Управление яркостью дисплея, когда включен автополив
	if ((out1_watering_is_started == 1) || (out2_watering_is_started == 1) ||
			(out3_watering_is_started == 1) || (out4_watering_is_started == 1) ||
			(out5_watering_is_started == 1) || (out6_watering_is_started == 1))
	{			
		// Display brightness control
		DisplayBrightnessControl();

		sysState->AutoWatering = 1;
	}
	else sysState->AutoWatering = 0;
}

// Display brightness control
void DisplayBrightnessControl()
{
	// Яркость можно только повышать
	if (display_brightness <= AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED) {
		// Яркость для авторежимов на 30%
		display_brightness = AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED;
	}
	// Таймер можно только повышать
	if (display_brightness_timer < AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY) {
		// Перезапись, если яркость уже снижена для авторежима 
		if (display_brightness == AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE * DISPLAY_BRIGHTNESS_OFF_SPEED) {
			// Таймер уменьшения яркости дисплея для авторежимов
			display_brightness_timer = AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY;
		}
	}
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
void Power_down_handler(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2p_t * e2p)
{
	// Уменьшение энергопотребления контроллера для сохранения данных в eeprom
	Reduce_mcu_power();

	// Сохранение рабочих переменных в eeprom
	Backup_all_data(hcrc, hi2c, hrtc, e2p);
}

// Усреднение значения давления
void Get_average_pressure_value(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static	uint16_t		idx = 0;
	static	int16_t			pressure[5] = {0};
	float 	pressure_sum = 0;

	// Скользящее усреднение измеренных значений**********************************
	pressure[idx++] = sysState->WaterPressure;

	if (idx >= 5) idx = 0;
		
	for(uint8_t c = 0; c < 5; c++)
	{
		pressure_sum += pressure[c];
	}
	
	sysState->AverageWaterPressure = (int16_t) roundf(pressure_sum / 5);
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
