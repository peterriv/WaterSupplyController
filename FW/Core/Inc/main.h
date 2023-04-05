/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "types_def.h"
#include "config.h"
#include "backup.h"
#include "hex2ascii.h"
#include "ds18b20.h"
#include "ring_buffer.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// Handles data from Com ports
ReturnCode_t Com_rxd_handler(CRC_HandleTypeDef * hcrc, ComNum_t ComNumber, JetsonComPort_t * jetson, NextionComPort_t * nextion);

// Проверка целостности и к.с. принятой по com строки данных
ReturnCode_t Check_received_nextion_packet(uint8_t * buf, uint16_t lenght);

// Копирование из одного буфера во 2-ой
void Copy_buf(uint8_t * source_buf, uint8_t * dest_buf, uint16_t buf_lenght);

// Reset indication
void Leds_on_off(void);

// Starts receiving data on COM ports
void Com_start_receiving_data(ComNum_t ComNumber);

// Пересчёт значений АЦП1 для каждого канала в вольты
void Voltage_calc_from_adc_value(E2p_t * e2p);

// Initial hardware settings
void Init_sequence(void);

// Разбор принятой строки от дисплея Nextion
void Parsing_nextion_display_string(RTC_HandleTypeDef  * hrtc, E2p_t * e2p, uint8_t * buf, uint16_t string_lenght, uint8_t string_status);

// Проверка длины, подсчет к.с. стандартной строки NMEA0183
uint8_t Nmea_string_check_checksum(uint8_t * buf, uint16_t lenght);
		
// Подсчет к.с. буфера и запись в передаваемую строку
void Set_string_binary_checksum(uint8_t  * buf, uint16_t lenght);

// Adding triple 0xFF at the end of the command as command termination and pushing to ring buffer for sending
ReturnCode_t Add_termination_to_nextion_command_and_push_to_ring_buf(NextionComPort_t * nextion);

// Отрисовка на Nextion текущего значения джойстиков
ReturnCode_t Prepare_params_and_send_to_nextion(RTC_HandleTypeDef  * hrtc, E2p_t * e2p, NextionComPort_t * nextion);

// Обработчик принятого пакета по COM2 из дисплея Nextion
ReturnCode_t Nextion_received_data_handler(RTC_HandleTypeDef  * hrtc, E2p_t * e2p);

// Проверка целостности и к.с. принятой по com строки данных
ReturnCode_t Check_received_nextion_packet(uint8_t * buf, uint16_t lenght);

// Checking time to switch on pump if matched
ReturnCode_t Switch_on_pump_by_time(E2p_t * e2p);

// Управление насосом
void Pump_on_off(E2p_t * e2p);

// Управление автополивом, зона 1-8
void Watering_on_off(E2p_t * e2p);

// Настройка PVD (Programmable Voltage Detector)
static void PVD_Config(void);

// Уменьшение энергопотребления контроллера для сохранения данных в eeprom
void Reduce_mcu_power(void);

// Копирование из одного буфера по произвольному адресу во 2-ой 
void Copy_buf_random_address(uint8_t * source_buf, uint32_t source_buf_offset, uint8_t * dest_buf, uint32_t dest_buf_offset, uint32_t size_to_copy);

// Обработчик состояния падения напряжения питания ниже 4.6В
void Power_down_handler(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2p_t * e2p);

// Усреднение значения давления
void Get_average_pressure_value(E2p_t * e2p);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WATER_COUNTER_EXTI3_Pin GPIO_PIN_3
#define WATER_COUNTER_EXTI3_GPIO_Port GPIOC
#define WATER_COUNTER_EXTI3_EXTI_IRQn EXTI3_IRQn
#define USART2_TX_NEXTION_Pin GPIO_PIN_2
#define USART2_TX_NEXTION_GPIO_Port GPIOA
#define USART2_RX_NEXTION_Pin GPIO_PIN_3
#define USART2_RX_NEXTION_GPIO_Port GPIOA
#define EN_TXD3_RXD3_Pin GPIO_PIN_2
#define EN_TXD3_RXD3_GPIO_Port GPIOB
#define WATER_ZONE1_Pin GPIO_PIN_12
#define WATER_ZONE1_GPIO_Port GPIOB
#define WATER_ZONE2_Pin GPIO_PIN_13
#define WATER_ZONE2_GPIO_Port GPIOB
#define WATER_ZONE3_Pin GPIO_PIN_14
#define WATER_ZONE3_GPIO_Port GPIOB
#define WATER_ZONE4_Pin GPIO_PIN_15
#define WATER_ZONE4_GPIO_Port GPIOB
#define WATER_ZONE5_Pin GPIO_PIN_6
#define WATER_ZONE5_GPIO_Port GPIOC
#define WATER_ZONE6_Pin GPIO_PIN_7
#define WATER_ZONE6_GPIO_Port GPIOC
#define WATER_ZONE7_Pin GPIO_PIN_8
#define WATER_ZONE7_GPIO_Port GPIOC
#define WATER_ZONE8_Pin GPIO_PIN_9
#define WATER_ZONE8_GPIO_Port GPIOC
#define DISP_PWR_EN_Pin GPIO_PIN_8
#define DISP_PWR_EN_GPIO_Port GPIOA
#define EN_TXD1_RXD1_Pin GPIO_PIN_11
#define EN_TXD1_RXD1_GPIO_Port GPIOA
#define PUMP_ON_OFF_Pin GPIO_PIN_15
#define PUMP_ON_OFF_GPIO_Port GPIOA
#define USART4_TX_Pin GPIO_PIN_10
#define USART4_TX_GPIO_Port GPIOC
#define USART4_RX_Pin GPIO_PIN_11
#define USART4_RX_GPIO_Port GPIOC
#define EN_TXD4_RXD4_Pin GPIO_PIN_3
#define EN_TXD4_RXD4_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
