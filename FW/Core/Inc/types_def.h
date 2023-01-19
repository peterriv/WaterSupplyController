#ifndef _TYPE_DEFINITIONS_H
#define _TYPE_DEFINITIONS_H

//#include "main.h"
#include <stdint.h>
#include "config.h"


// Logic State (True, False)
typedef enum
{ 
  FALSE = 0,
  TRUE
} LogicState;


// Com Number
typedef enum
{ 
  COM1 = 1,
  COM2 = 2,
  COM3 = 3,
  COM4 = 4,
  COM5 = 5,
} ComNum;


// Com Data Source Marker
typedef enum
{ 
  PumpOn														= 0x62303030,
  PumpOff														= 0x62303031,

  PumpOnPressureDec									= 0x62313034,
  PumpOnPressureInc									= 0x62313035,
  PumpOffPressureDec								= 0x62313036,
  PumpOffPressureInc								= 0x62313037,

  PresSensorPminDec									= 0x62313030,
  PresSensorPminInc									= 0x62313031,
  PresSensorPmaxDec									= 0x62313032,
  PresSensorPmaxInc									= 0x62313033,
	
  VoltageForPminDec									= 0x62313038,
  VoltageForPminInc									= 0x62313039,	
  VoltageForPmaxDec									= 0x62313130,
  VoltageForPmaxInc									= 0x62313131,

	CurrWateringOutputNumberDec				= 0x62333037,
	CurrWateringOutputNumberInc				= 0x62333038,
	OutxZeroClockTimeDeltaDec					= 0x62333031,
	OutxZeroClockTimeDeltaInc					= 0x62333032,
	OutxWorkingTimeDec								= 0x62333033,
	OutxWorkingTimeInc								= 0x62333034,
	OutxWorkIntervalTimeDec						= 0x62333035,
	OutxWorkIntervalTimeInc						= 0x62333036,
	CurrOutputSettingsToDefault				= 0x62333039,

  //WaterCounterValueDec							= 0x62343032,
  //WaterCounterValueInc							= 0x62343033,

	AutoPumpZeroClockDeltaDec					= 0x62343034,
	AutoPumpZeroClockDeltaInc					= 0x62343035,
	AutoPumpQuantityDec								= 0x62343036,
	AutoPumpQuantityInc								= 0x62343037,	
	AutoPumpIntervalTimeDec						= 0x62343032,
	AutoPumpIntervalTimeInc						= 0x62343033,

  CurrentTimeDecrement							= 0x62343038,
  CurrentTimeIncrement							= 0x62343039,

  TimeCorrectionValueDec						= 0x62343130,
  TimeCorrectionValueInc						= 0x62343131,
	
  // Напряжения для установки  мин/макс точек давления воды в источнике
	VoltageForPminSourceDec						= 0x62353031,
  VoltageForPminSourceInc						= 0x62353032,	
  VoltageForPmaxSourceDec						= 0x62353033,
  VoltageForPmaxSourceInc						= 0x62353034,
	
  // Напряжения для установки  мин/макс точек давления воды в накопителе
  VoltageForPminTankDec							= 0x62353035,
  VoltageForPminTankInc							= 0x62353036,	
  VoltageForPmaxTankDec							= 0x62353037,
  VoltageForPmaxTankInc							= 0x62353038,

  // Напряжения для установки  мин/макс точек давления воды в источнике, накопителе
	// по текущему уровню напряжения
  SetVoltageForPminSource						= 0x62353039,
  SetVoltageForPmaxSource						= 0x62353130,
  SetVoltageForPminTank							= 0x62353131,	
  SetVoltageForPmaxTank							= 0x62353132,

	ResetAllSettingsToDefault					= 0x62353133,
	
} ComDataSourceMarker;


// Adc1 Channel Name
typedef enum
{ 
  Channel11		 				= 0,					// ADC_IN11			+(0.5-4.5)В Д. давления
  Channel12		 				= 1,					// ADC_IN12			+(0-5)В Резерв
  //Channel13		 				= 2,					// ADC_IN13			Счётный вход (EXTI3)
} Adc1ChannelName;


// Adc2 Channel Name
typedef enum
{ 
  Channel10		 				= 0,					// ADC_IN10			+5В контроль питания
} Adc2ChannelName;


// Return Code
typedef enum
{ 
  OK 																=	0,
  CheckSumError 										=	1,
	StringHeaderError									=	2,
	StringTerminationError						=	3,
	StringLengthError									=	4,
	E2pPageSizeTooBig									=	5,
	E2pMemoryWriteError								=	6,
	BackupRegsSizeExceeded						=	7,
	BackupRegsWriteError							=	8,
	BufferSizeExceeded								=	9,
} ReturnCode;


// Calibrations Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t		StructSize;

	// Давление включения насоса, атм*10
	int16_t			PumpOnPressureValue;

	// Давление выключения насоса, атм*10
	int16_t			PumpOffPressureValue;

	// Минимальное значение давления датчика давления, атм*10
	int16_t			PsensorMinPressureValue;

	// Максимальное значение давления датчика давления, атм*10
	int16_t			PsensorMaxPressureValue;
		
	// Напряжение (0 - 5В), соотв. минимальному давлению датчика давления, мВ*100
	int16_t			PsensorMinPressureVoltageValue;

	// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления, мВ*100
	int16_t			PsensorMaxPressureVoltageValue;

	// Напряжение (0 - 5В), соотв. минимальному давлению датчика давления в источнике воды, мВ*100
	int16_t			SourcePsensorMinPressureVoltageValue;

	// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления в источнике воды, мВ*100
	int16_t			SourcePsensorMaxPressureVoltageValue;
	
	// Напряжение (0 - 5В), соотв. минимальному давлению датчика давления в накопителе воды, мВ*100
	int16_t			TankPsensorMinPressureVoltageValue;

	// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления в накопителе воды, мВ*100
	int16_t			TankPsensorMaxPressureVoltageValue;
	
	// Коррекция текущего времени, секунд в неделю
	int8_t			TimeCorrectionValue;
	
} CalibrationsTypeDef;


// Statistics Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t			StructSize;

	// Счётчик циклов выключения питания
	uint32_t			PowerOffCycleCounter;
	
	// Счётчик текущего времени в секундах
	int32_t				TimeInSeconds;

	// Общее время работы контроллера, секунд
	uint32_t			TotalControllerWorkingTime;

	// Общее время работы насоса, секунд
	uint32_t			TotalPumpWorkingTime;

	// Общее кол-во воды, перекачанной насосом, литры*10  (десятки литров)
	uint32_t			TotalPumpedWaterQuantity;
	
	// Значение счётчика расхода воды, литры
	int32_t				WaterCounterValue;

	
	// Кол-во воды, перекачанной за текущие сутки, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantityToday;

	// Кол-во воды, перекачанной за вчерашние сутки, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity1dayAgo;
	
	// Кол-во воды, перекачанной за позавчерашние сутки, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity2daysAgo;

	// Кол-во воды, перекачанной в течение 3-х суток назад, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity3daysAgo;

	// Кол-во воды, перекачанной в течение 4-х суток назад, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity4daysAgo;

	// Кол-во воды, перекачанной в течение 5-х суток назад, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity5daysAgo;

	// Кол-во воды, перекачанной в течение 6-х суток назад, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity6daysAgo;

	// Кол-во воды, перекачанной в течение 7-х суток назад, литры*10  (десятки литров)
	uint16_t			PumpedWaterQuantity7daysAgo;
	
	// Кол-во воды, перекачанной за последние 7 дней (посуточная сумма)
	uint32_t			PumpedWaterQuantityLastWeek;
	
	// Номер текущих суток
	uint16_t			CurrentDayNumber;

	// Номер вчерашних суток
	uint16_t			YesterdayDayNumber;

	// Номер позавчерашних суток
	uint16_t			TwoDaysAgoDayNumber;

	// Номер суток, бывших 3 дня назад
	uint16_t			ThreeDaysAgoDayNumber;

	// Номер суток, бывших 4 дня назад
	uint16_t			FourDaysAgoDayNumber;

	// Номер суток, бывших 5 дней назад
	uint16_t			FiveDaysAgoDayNumber;

	// Номер суток, бывших 6 дней назад
	uint16_t			SixDaysAgoDayNumber;

	// Номер суток, бывших 7 дней назад
	uint16_t			SevenDaysAgoDayNumber;

} StatisticsTypeDef;


// Watering Control Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t			StructSize;

	// Текущий номер выхода полива, 1-8
	int8_t				CurrWateringOutputNumber;

	// Значение смещения времени включения полива зоны 1 относительно начала суток, мин
	int16_t				out1_zero_clock_time_delta;
	// Значение времени работы полива зоны 1, мин
	int16_t				out1_working_time;
	// Интервал времени между включениями полива зоны 1, мин
	int16_t				out1_interval_time;

	// Значение смещения времени включения полива зоны 2 относительно начала суток, мин
	int16_t				out2_zero_clock_time_delta;
	// Значение времени работы полива зоны 2, мин
	int16_t				out2_working_time;
	// Интервал времени между включениями полива зоны 2, мин
	int16_t				out2_interval_time;

	// Значение смещения времени включения полива зоны 3 относительно начала суток, мин
	int16_t				out3_zero_clock_time_delta;
	// Значение времени работы полива зоны 3, мин
	int16_t				out3_working_time;
	// Интервал времени между включениями полива зоны 3, мин
	int16_t				out3_interval_time;

	// Значение смещения времени включения полива зоны 4 относительно начала суток, мин
	int16_t				out4_zero_clock_time_delta;
	// Значение времени работы полива зоны 4, мин
	int16_t				out4_working_time;
	// Интервал времени между включениями полива зоны 4, мин
	int16_t				out4_interval_time;

	// Значение смещения времени включения полива зоны 5 относительно начала суток, мин
	int16_t				out5_zero_clock_time_delta;
	// Значение времени работы полива зоны 5, мин
	int16_t				out5_working_time;
	// Интервал времени между включениями полива зоны 5, мин
	int16_t				out5_interval_time;

	// Значение смещения времени включения полива зоны 6 относительно начала суток, мин
	int16_t				out6_zero_clock_time_delta;
	// Значение времени работы полива зоны 6, мин
	int16_t				out6_working_time;
	// Интервал времени между включениями полива зоны 6, мин
	int16_t				out6_interval_time;

	// Значение смещения времени включения полива зоны 7 относительно начала суток, мин
	int16_t				out7_zero_clock_time_delta;
	// Значение времени работы полива зоны 7, мин
	int16_t				out7_working_time;
	// Интервал времени между включениями полива зоны 7, мин
	int16_t				out7_interval_time;

	// Значение смещения времени включения полива зоны 8 относительно начала суток, мин
	int16_t				out8_zero_clock_time_delta;
	// Значение времени работы полива зоны 8, мин
	int16_t				out8_working_time;
	// Интервал времени между включениями полива зоны 8, мин
	int16_t				out8_interval_time;
	
	// Текущее состояние активности автополива (исп. для отображения на дисплее)
	uint8_t				AutoWatering;

} WateringControlTypeDef;


// Last Pump Cycle Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t			StructSize;
	
	// Включить насос
	uint8_t				switch_pump_on;

	// Выключить насос
	uint8_t				switch_pump_off;

	// Насос запущен , 0- выключен, 1- включен
	uint8_t				pump_is_started;
	
	// Время включения насоса в последнем цикле, сек
	int32_t				pump_start_time_at_last_cycle;
	
	// Время непрерывной работы насоса в предыдущем цикле, сек
	int32_t				pump_working_time_at_last_cycle;

	// Кол-во воды, перекачанной насосом в предыдущем цикле, литры*10  (десятки литров)
	uint16_t			pumped_water_quantity_at_last_cycle;

	// t воды при перекачивании, 'С * 10
	int16_t				water_temp_while_pumped;
	
	// Событие "сухого хода", когда =1
	uint8_t				dry_work_detected;
	
	// Флаг выполнения автоподкачивания воды в текущий момент
	uint8_t				auto_pump_is_started;

	// Значение смещения времени включения автоподкачки относительно начала суток, мин
	int16_t				auto_pump_zero_clock_time_delta;

	// Кол-во воды для ежесуточного автоподкачивания, литры * 10  (десятки литров)
	int16_t				auto_pump_quantity;
	
	// Интервал времени между включениями автоподкачивания, мин
	int16_t				auto_pump_interval_time;
	
	// Минимальная суточная t воды в источнике, 'С * 10
	int16_t				well_water_temp_min_for_24h;

	// Максимальная суточная t воды в источнике, 'С * 10
	int16_t				well_water_temp_max_for_24h;

	// Минимальная суточная t воды в накопителе, 'С * 10
	int16_t				tank_water_temp_min_for_24h;

	// Максимальная суточная t воды в накопителе, 'С * 10
	int16_t				tank_water_temp_max_for_24h;
	
	// Значение давления воды в системе, атм * 10
	int16_t				water_pressure_value;
	int16_t				average_water_pressure_value;

	// текущая t воды, 'С * 10
	int16_t				current_water_temp;

	// t воды в источнике, 'С * 10
	int16_t				well_water_temp;

	// Уровень воды в источнике, в вольтах/10 датч. давл.
	int16_t				WellWaterLevelInVolts;

	// t воды в накопителе, 'С * 10
	int16_t				tank_water_temp;

	// Уровень воды в накопителе, в вольтах/10 датч. давл.
	int16_t				TankWaterLevelInVolts;
	
} LastPumpCycleTypeDef;


// Com ports data type def
typedef struct
{
	// Com instance number (COM1 = 1, COM2 = 2, etc)
	uint8_t		ComNumber; //__attribute__((aligned(4)));
	
	// Variable for receiving single byte
	uint8_t		ByteReceived;

	// Data packet receiving is started
	//uint8_t		RxdPacketIsStarted;

	// Маркер готовности обработки/приёма пакета данных
	uint8_t		RxdPacketIsReceived;
	
	// Pointer to next byte in rxd buffer
	uint32_t	RxdIdx8;

	// Lenght of received data packet
	uint32_t	RxdPacketLenght8;
	
	// Counter of received data packets without errors
	uint32_t	RxdGoodPacketsCounter;
	
	// Counter of broken packets while receiving
	uint32_t	RxdPacketsErrorCounter;

	// Timer in SysTick units (1 ms). Using to separate one packet from another and find start of data packet
	uint32_t	RxDataFlowGapTimer;
	

	// Pointer to next byte in txd buffer (used if transmit via interrupt) 
	uint32_t	TxdIdx8;
	
	// Lenght of data packet to send
	uint32_t	TxdPacketLenght8;

	// Counter of sent data packets
	uint32_t	TxdPacketsSentCounter;

	// Timer in SysTicks for tx data sending interval
	uint32_t	TxdPacketReadyToSendTimer;
	
	// != 0 if data packet is ready to be transmitted, otherwise = 0
	uint8_t	TxdPacketIsReadyToSend;

	// = 1 After end of whole packet transmission
	uint8_t	TxdPacketIsSent;

} ComPortDataTypeDef;


// JetsonComPortDataTypeDef
typedef struct
{
	// Com link (COM1 = External interface for debugging, COM2 = Jetson)
	uint8_t		ComLink;
	
	ComPortDataTypeDef *Com;

	// RxD buffer
	uint8_t		RxdBuffer[JETSON_COM_RXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));	

	// TxD buffer
	uint8_t		TxdBuffer[JETSON_COM_TXD_BUF_SIZE_IN_BYTES];
	
} JetsonComPortDataTypeDef;


// Nextion Com Port Data Type Def
typedef struct
{
	// Com link (COM2 = Nextion display)
	uint8_t		ComLink;
	
	ComPortDataTypeDef *Com;

	// RxD buffer
	uint8_t		RxdBuffer[STRING_LENGHT_FROM_NEXTION]; //__attribute__((aligned(4)));	

	// TxD buffer
	uint8_t		TxdBuffer[STRING_LENGHT_TO_NEXTION];
	// Промежуточный буфер для формирования данных и отправки через Com2 в дисплей Nextion
	//uint8_t		nextion.TxdBuffer[STRING_LENGHT_TO_NEXTION];
	
} NextionComPortDataTypeDef;


// Temperature Sensor Com Port Data Type Def
typedef struct
{
	// Com link (COM5 = DS18B20)
	uint8_t		ComLink;
	
	ComPortDataTypeDef *Com;

	// RxD buffer
	uint8_t		RxdBuffer[STRING_LENGHT_FROM_NEXTION]; //__attribute__((aligned(4)));	

	// TxD buffer
	uint8_t		TxdBuffer[STRING_LENGHT_TO_NEXTION];
	
} TempSensorComPortDataTypeDef;


// Описатель структуры eeprom
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t		StructSize;

	StatisticsTypeDef				* Statistics;
	WateringControlTypeDef	* WateringControls;
	CalibrationsTypeDef			* Calibrations;
	LastPumpCycleTypeDef		* LastPumpCycle;

} E2pDataTypeDef;


// Описатель структуры значений АЦП
typedef struct
{
	// ADC channels counts
	uint32_t 								CountsBuf[4] __attribute__((aligned(4)));
	
	// ADC channels values in mv
	int32_t 								VoltsBuf[4] __attribute__((aligned(4)));
	
	uint8_t									DataReady;
	
} AdcDataTypeDef;


// Описатель структуры буфера данных термодатчиков
typedef struct
{
	// Кол-во датчиков, обнаруженных при сканировании шины 1 Wire
	uint8_t 								DiscoveredQuantity;

	// Флаг выполнения опроса датчиков при истечении периода задержки
	uint8_t 								GetSensorsData;

	// Таймер для задержки между чтениями датчиков температуры, SysTicks
	uint32_t 								PollingWaitTimer;
	
	// Значения температуры датчиков
	float 									TempSensorsValues[ONE_WIRE_TERMO_SENSORS_QUANTITY] __attribute__((aligned(4)));
} TemperatureDataTypeDef;

#endif

