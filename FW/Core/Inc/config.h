#ifndef			_CONFIG_H
#define 		_CONFIG_H

#include "main.h"


// Nextion display communication port selection
#define NEXTION_DISPLAY_COM_PORT									COM2

// Запись калибровочного коэффициента для коррекции хода часов реального времени ( сек/месяц)
// при подобранном к. деления = 32762, при котором часы только начинают опережать (>32762 часы отстают), за сутки опережение на 4 сек при корр. =0
// 32 = 31 ppm = -79 сек/мес (-0.5c/сут); 36 = 34 ppm = -89 сек/мес (-1c/сут); 24 = 23 ppm = -59 сек/мес (+2c/сут); 48 = 46 ppm = -119 сек/мес (-4c/сут)
// 30 = 29 ppm = -74 сек/мес (-?c/сут);
#define	RTC_TIME_CALIBRATION_COEFF								30

// Вид представления данных часов реального времени (двоичн / двоичн. - десятичн.)
#define	RTC_FORMAT																RTC_FORMAT_BIN


// Максимальный размер страницы в EEPROM для разовой записи, байт
#define	E2P_PAGE_MAX_SIZE													512

// Размер страницы в EEPROM для разовой записи, байт
#define	E2P_PAGE_SIZE															256

// Кол-во термодатчиков 1-Wire на одной шине
#define	ONE_WIRE_TERMO_SENSORS_QUANTITY						1

// Период опроса термодатчиков 1-Wire, SysTicks
#define	TERMO_SENSORS_POLL_PERIOD									1000

// Коэффициент для перевода бар в атмосферы
#define	BAR_TO_ATM_COEFF													0.98692327f

// Максимальное значение давления в системе, атм * 10
#define	PRESSURE_MAX_VALUE												160

// Максимальное значение давления в источнике, атм * 10
#define	SOURCE_PRESSURE_MAX_VALUE									160

// Максимальное значение давления в накопителе, атм * 10
#define	DEST_PRESSURE_MAX_VALUE										160

// Напряжение максимального значения давления в системе, В * 10
#define	MAX_VOLTAGE_VALUE_FOR_P_SENSOR						50

#define	ANALOG_REFERENCE_VOLTAGE									3300																																	//mV
#define	ADC_RESOLUTION														4096																																	//counts
#define	ADC_LSB_VALUE															(float)((float)ANALOG_REFERENCE_VOLTAGE/(float)(ADC_RESOLUTION-1))		//mV на один отсчёт АЦП

// Определение коэффициента деления при переходе с напряжения +5В пит. д.д. к уровню для АЦП
#define	PRESSURE_SENSOR_POWER_VOLTAGE_VALUE				5.11f
#define	ADC_PRS_SENS_VOLTAGE_VALUE								2.311f
#define	FIVE_VOLTS_DIVISION_COEFF									(float)PRESSURE_SENSOR_POWER_VOLTAGE_VALUE/ADC_PRS_SENS_VOLTAGE_VALUE

// Порог напряжения питания цепи +5В, <= которому сработает сохранение в eeprom, мВ
#define	ADC_WDG_LOW_THRESHOLD_VOLTAGE_VALUE				4600
#define	ADC_WDG_LOW_THRESHOLD											(uint16_t) ((float)(ADC_WDG_LOW_THRESHOLD_VOLTAGE_VALUE)/(FIVE_VOLTS_DIVISION_COEFF*ADC_LSB_VALUE))
// Порог напряжения питания цепи +5В, >= которому сработает восстановление из eeprom, мВ
#define	ADC_WDG_HIGH_THRESHOLD_VOLTAGE_VALUE			4800
#define	ADC_WDG_HIGH_THRESHOLD										(uint16_t) ((float)(ADC_WDG_HIGH_THRESHOLD_VOLTAGE_VALUE)/(FIVE_VOLTS_DIVISION_COEFF*ADC_LSB_VALUE))


#define JETSON_COM_RXD_BUF_SIZE_IN_BYTES   				128
#define JETSON_COM_TXD_BUF_SIZE_IN_BYTES   				128


// Длина строки данных из дисплея Nextion
#define	STRING_LENGHT_FROM_NEXTION								12

// Длина строки данных для отправки в дисплей Nextion
#define	STRING_LENGHT_TO_NEXTION									813

// Длина строки данных от ведущего контроллера
#define	STRING_LENGHT_FROM_MASTER									8

// Длина строки данных для передачи в БЧ
#define	STRING_LENGHT_TO_MASTER										8


// Значение в SysTick для определения таймаута "сухого хода"
#define	DRY_WORK_TIMEOUT_VALUE										35000

// Значение в секундах для определения задержки вкл/выкл насоса по давлению
#define	PUMP_ON_OFF_DELAY													3

// Коэфф. скорости уменьшения яркости дисплея в привязке к systick (100%*(DISPLAY_BRIGHTNESS_OFF_SPEED=20)=2000, т.е. за 2 сек)
#define	DISPLAY_BRIGHTNESS_OFF_SPEED							20

// Значение в секундах задержки перед снижением яркости дисплея
#define	DISPLAY_BRIGHTNESS_OFF_DELAY							60

// Значение в секундах задержки перед снижением яркости дисплея после отключения автофункций
#define	AUTOFUNC_DISPLAY_BRIGHTNESS_OFF_DELAY			5

// Яркость дисплея  при работе автофункций, %
#define	AUTOFUNC_DISPLAY_BRIGHTNESS_VALUE					40

// Максимальное значение яркости дисплея, %
#define	DISPLAY_BRIGHTNESS_MAX_VALUE							100

// Минимальное значение яркости дисплея, %
#define	DISPLAY_BRIGHTNESS_MIN_VALUE							5


// Значение в SysTick для определения таймаута обрыва связи, SysTicks
#define	NO_DATA_TIMEOUT_VALUE											500

// Интервал выполнения потока опроса и управления периф. устройствами, SysTicks
#define	PERIPH_SCAN_TIMER_TIMEOUT									125

// Интервал отправки данных по COM1, SysTicks
#define	COM1_DATA_PACKET_SEND_TIMEOUT							100
// Интервал отправки данных по COM2 (Nextion), SysTicks
#define	COM2_DATA_PACKET_SEND_TIMEOUT							100

// Длительность перерыва в потоке принимаемых данных com-порта, означачающая разделение пакетов
#define	DATA_FLOW_GAP_TIME_VALUE									4


#define	NMEA0183_STRING_HEADER_SYMBOL							'$'

#define	DISPLAY_POWER_ENABLE											HAL_GPIO_WritePin(DISP_PWR_EN_GPIO_Port, DISP_PWR_EN_Pin, GPIO_PIN_SET)
#define	DISPLAY_POWER_DISABLE											HAL_GPIO_WritePin(DISP_PWR_EN_GPIO_Port, DISP_PWR_EN_Pin, GPIO_PIN_RESET)

#define	RXD1_ENABLE																HAL_GPIO_WritePin(EN_TXD1_RXD1_GPIO_Port, EN_TXD1_RXD1_Pin, GPIO_PIN_RESET)
#define	RXD1_DISABLE															HAL_GPIO_WritePin(EN_TXD1_RXD1_GPIO_Port, EN_TXD1_RXD1_Pin, GPIO_PIN_SET)

#define	TXD1_ENABLE																HAL_GPIO_WritePin(EN_TXD1_RXD1_GPIO_Port, EN_TXD1_RXD1_Pin, GPIO_PIN_SET)
#define	TXD1_DISABLE															HAL_GPIO_WritePin(EN_TXD1_RXD1_GPIO_Port, EN_TXD1_RXD1_Pin, GPIO_PIN_RESET)

#define	RXD4_ENABLE																HAL_GPIO_WritePin(EN_TXD4_RXD4_GPIO_Port, EN_TXD4_RXD4_Pin, GPIO_PIN_RESET)
#define	RXD4_DISABLE															HAL_GPIO_WritePin(EN_TXD4_RXD4_GPIO_Port, EN_TXD4_RXD4_Pin, GPIO_PIN_SET)

#define	TXD4_ENABLE																HAL_GPIO_WritePin(EN_TXD4_RXD4_GPIO_Port, EN_TXD4_RXD4_Pin, GPIO_PIN_SET)
#define	TXD4_DISABLE															HAL_GPIO_WritePin(EN_TXD4_RXD4_GPIO_Port, EN_TXD4_RXD4_Pin, GPIO_PIN_RESET)

#define	RXD3_ENABLE																HAL_GPIO_WritePin(EN_TXD3_RXD3_GPIO_Port, EN_TXD3_RXD3_Pin, GPIO_PIN_RESET)
#define	RXD3_DISABLE															HAL_GPIO_WritePin(EN_TXD3_RXD3_GPIO_Port, EN_TXD3_RXD3_Pin, GPIO_PIN_SET)

#define	TXD3_ENABLE																HAL_GPIO_WritePin(EN_TXD3_RXD3_GPIO_Port, EN_TXD3_RXD3_Pin, GPIO_PIN_SET)
#define	TXD3_DISABLE															HAL_GPIO_WritePin(EN_TXD3_RXD3_GPIO_Port, EN_TXD3_RXD3_Pin, GPIO_PIN_RESET)


#define	WATER_PUMP_ON															HAL_GPIO_WritePin(PUMP_ON_OFF_GPIO_Port, PUMP_ON_OFF_Pin, GPIO_PIN_RESET)
#define	WATER_PUMP_OFF														HAL_GPIO_WritePin(PUMP_ON_OFF_GPIO_Port, PUMP_ON_OFF_Pin, GPIO_PIN_SET)

#define	WATER_ZONE1_ON														HAL_GPIO_WritePin(WATER_ZONE1_GPIO_Port, WATER_ZONE1_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE1_OFF														HAL_GPIO_WritePin(WATER_ZONE1_GPIO_Port, WATER_ZONE1_Pin, GPIO_PIN_SET)

#define	WATER_ZONE2_ON														HAL_GPIO_WritePin(WATER_ZONE2_GPIO_Port, WATER_ZONE2_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE2_OFF														HAL_GPIO_WritePin(WATER_ZONE2_GPIO_Port, WATER_ZONE2_Pin, GPIO_PIN_SET)

#define	WATER_ZONE3_ON														HAL_GPIO_WritePin(WATER_ZONE3_GPIO_Port, WATER_ZONE3_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE3_OFF														HAL_GPIO_WritePin(WATER_ZONE3_GPIO_Port, WATER_ZONE3_Pin, GPIO_PIN_SET)

#define	WATER_ZONE4_ON														HAL_GPIO_WritePin(WATER_ZONE4_GPIO_Port, WATER_ZONE4_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE4_OFF														HAL_GPIO_WritePin(WATER_ZONE4_GPIO_Port, WATER_ZONE4_Pin, GPIO_PIN_SET)

#define	WATER_ZONE5_ON														HAL_GPIO_WritePin(WATER_ZONE5_GPIO_Port, WATER_ZONE5_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE5_OFF														HAL_GPIO_WritePin(WATER_ZONE5_GPIO_Port, WATER_ZONE5_Pin, GPIO_PIN_SET)

#define	WATER_ZONE6_ON														HAL_GPIO_WritePin(WATER_ZONE6_GPIO_Port, WATER_ZONE6_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE6_OFF														HAL_GPIO_WritePin(WATER_ZONE6_GPIO_Port, WATER_ZONE6_Pin, GPIO_PIN_SET)

#define	WATER_ZONE7_ON														HAL_GPIO_WritePin(WATER_ZONE7_GPIO_Port, WATER_ZONE7_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE7_OFF														HAL_GPIO_WritePin(WATER_ZONE7_GPIO_Port, WATER_ZONE7_Pin, GPIO_PIN_SET)

#define	WATER_ZONE8_ON														HAL_GPIO_WritePin(WATER_ZONE8_GPIO_Port, WATER_ZONE8_Pin, GPIO_PIN_RESET)
#define	WATER_ZONE8_OFF														HAL_GPIO_WritePin(WATER_ZONE8_GPIO_Port, WATER_ZONE8_Pin, GPIO_PIN_SET)


#define	LED1_ON																		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define	LED1_OFF																	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define	LED1_TOGGLE																HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define	LED2_ON																		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define	LED2_OFF																	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define	LED2_TOGGLE																HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)


#define WATER_COUNTER_EXTI3_READ_PIN							HAL_GPIO_ReadPin	(WATER_COUNTER_EXTI3_GPIO_Port, WATER_COUNTER_EXTI3_Pin)

#endif
