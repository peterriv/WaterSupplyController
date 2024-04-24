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
} LogicState_t;


// Com Number
typedef enum
{ 
  COM1 = 1,
  COM2 = 2,
  COM3 = 3,
  COM4 = 4,
  COM5 = 5,
} ComNum_t;


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

  // Увеличение/уменьшение таймаута срабатывания останова насоса по "сухому ходу"
	PumpRunDryStopTimeoutDec					= 0x62313132,
  PumpRunDryStopTimeoutInc					= 0x62313133,

	
	CurrWateringOutputNumberDec				= 0x62333037,
	CurrWateringOutputNumberInc				= 0x62333038,
	OutxZeroClockTimeDeltaDec					= 0x62333031,
	OutxZeroClockTimeDeltaInc					= 0x62333032,
	OutxWorkingTimeDec								= 0x62333033,
	OutxWorkingTimeInc								= 0x62333034,
	OutxWorkIntervalTimeDec						= 0x62333035,
	OutxWorkIntervalTimeInc						= 0x62333036,
	CurrOutputSettingsToDefault				= 0x62333039,

	AutoPumpTimeDeltaFromStartOfDayDec	= 0x62343034,
	AutoPumpTimeDeltaFromStartOfDayInc	= 0x62343035,
	AutoPumpVolumeDec									= 0x62343036,
	AutoPumpVolumeInc									= 0x62343037,	
	AutoPumpIntervalTimeDec						= 0x62343032,
	AutoPumpIntervalTimeInc						= 0x62343033,
	AutoPumpTimesDec									= 0x62343038,
	AutoPumpTimesInc									= 0x62343039,

  CurrentTimeDecrement							= 0x62373031,
  CurrentTimeIncrement							= 0x62373032,

  TimeCorrectionValueDec						= 0x62373033,
  TimeCorrectionValueInc						= 0x62373034,
	
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
	
	// Номер активного экрана(страницы) дисплея Nextion
  Screen0														= 0x53637230,
  Screen1														= 0x53637231,
  Screen2														= 0x53637232,
  Screen3														= 0x53637233,
  Screen4														= 0x53637234,
  Screen5														= 0x53637235,
  Screen6														= 0x53637236,
  Screen7														= 0x53637237,
  Screen8														= 0x53637238,
  Screen9														= 0x53637239,
	
	SpecialWateringModeOn							= 0x62383031,
	SpecialWateringModeOff						= 0x62383032,
	SpModeWateringVolumeInc						= 0x62383038,
	SpModeWateringVolumeDec						= 0x62383037,
	SpModeWateringTimeInc							= 0x62383036,
	SpModeWateringTimeDec							= 0x62383035,
	SpModePumpOffPressureInc					= 0x62383034,
	SpModePumpOffPressureDec					= 0x62383033,

	ImpulsesPerOneLiterInc						= 0x62393032,
	ImpulsesPerOneLiterDec						= 0x62393031,
	LitersPerOneImpulseInc						= 0x62393034,
	LitersPerOneImpulseDec						= 0x62393033,

	ResetUVLampCounters								= 0x62323031,
	ResetAllSettingsToDefault					= 0x62353133,
	
} ComDataSourceMarker_t;


// Adc1 Channel Name
typedef enum
{ 
  Channel11		 				= 0,					// ADC_IN11			+(0.5-4.5)В Д. давления
  Channel12		 				= 1,					// ADC_IN12			+(0-5)В Резерв
  //Channel13		 				= 2,					// ADC_IN13			Счётный вход (EXTI3)
} Adc1ChannelName_t;


// Adc2 Channel Name
typedef enum
{ 
  Channel10		 				= 0,					// ADC_IN10			+5В контроль питания
} Adc2ChannelName_t;


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
	CommandIsAbsent										= 10,
	StringTerminationIsAbsent					=	11,
	
	StringLengthExceedsBufferSize			=	20,
	BuffersAreBusy										=	21,
	NoFreeSpaceInBuffer								=	22,
	
	Timeout														= 254,
	ERR																= 255
	
} ReturnCode_t;


// Calibrations Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t		StructSize;
	
	// Кол-во импульсов турбины, увеличивающих счётчик литров на 1
	int16_t			TurbineImpulsesPerLiter;

	// Кол-во литров счётчика воды на 1 импульс с его выхода
	int16_t			WaterCounterLitersPerImpulse;

	// Давление защитного отключения насоса в спец. режиме полива, атм*10
	int16_t			SpModePumpOffPressureValue;
	
	// Уставка длительности полива в спец. режиме полива с огранич. по врем., объёму, мин
	int16_t			SpModeWateringTime;

	// Уставка объёма полива в спец. режиме полива с огранич. по врем., объёму, л
	int16_t			SpModeWateringVolume;
	
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
	int16_t			SourcePsensorMinPressureVoltage;

	// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления в источнике воды, мВ*100
	int16_t			SourcePsensorMaxPressureVoltage;
	
	// Напряжение (0 - 5В), соотв. минимальному давлению датчика давления в накопителе воды, мВ*100
	int16_t			TankPsensorMinPressureVoltageValue;

	// Напряжение (0 - 5В), соотв. максимальному давлению датчика давления в накопителе воды, мВ*100
	int16_t			TankPsensorMaxPressureVoltageValue;
	
	// Коррекция текущего времени, секунд в неделю
	int8_t			TimeCorrectionValue;
	
} Calibrations_t;


// Statistics Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t			StructSize;

	// Счётчик циклов выключения питания
	uint32_t			PowerOffCycleCounter;
	
	// Время работы УФ лампы, секунд
	uint32_t			UvLampWorkingTime;

	// Счётчик циклов включения питания УФ лампы
	uint32_t			UvLampPowerOnCycleCounter;
	
	// Счётчик текущего времени в секундах
	int32_t				TimeInSeconds;

	// Общее время работы контроллера, секунд
	uint32_t			TotalControllerWorkingTime;

	// Общее время работы насоса, секунд
	uint32_t			TotalPumpWorkingTime;

	// Общее кол-во воды, перекачанной насосом, литры*10  (десятки литров)
	uint32_t			TotalPumpedWaterQuantity;
	
	
	// Кол-во воды, перекачанной за текущие сутки, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantityToday;

	// Кол-во воды, перекачанной за вчерашние сутки, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity1dayAgo;
	
	// Кол-во воды, перекачанной за позавчерашние сутки, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity2daysAgo;

	// Кол-во воды, перекачанной в течение третьих суток назад, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity3daysAgo;

	// Кол-во воды, перекачанной в течение четвёртых суток назад, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity4daysAgo;

	// Кол-во воды, перекачанной в течение пятых суток назад, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity5daysAgo;

	// Кол-во воды, перекачанной в течение шестых суток назад, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity6daysAgo;

	// Кол-во воды, перекачанной в течение седьмых суток назад, литры*10  (десятки литров)
	uint32_t			PumpedWaterQuantity7daysAgo;
	
	// Кол-во воды, перекачанной за последние 7 дней (посуточная сумма) без текущих суток
	uint32_t			PumpedWaterQuantityLastWeek;
	
//	// Номер текущих суток
//	uint16_t			CurrentDayNumber;

//	// Номер вчерашних суток
//	uint16_t			YesterdayDayNumber;

//	// Номер позавчерашних суток
//	uint16_t			TwoDaysAgoDayNumber;

//	// Номер суток, бывших 3 дня назад
//	uint16_t			ThreeDaysAgoDayNumber;

//	// Номер суток, бывших 4 дня назад
//	uint16_t			FourDaysAgoDayNumber;

//	// Номер суток, бывших 5 дней назад
//	uint16_t			FiveDaysAgoDayNumber;

//	// Номер суток, бывших 6 дней назад
//	uint16_t			SixDaysAgoDayNumber;

//	// Номер суток, бывших 7 дней назад
//	uint16_t			SevenDaysAgoDayNumber;

} Statistics_t;


// Watering Control Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t			StructSize;

	// Текущий номер выхода полива, 1-6
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

	// Текущее состояние активности автополива (исп. для отображения на дисплее)
	uint8_t				AutoWatering;

} WateringControl_t;


// Last Pump Cycle Type Def
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t			StructSize;
	
	// Включить насос
	uint8_t				SwitchPumpOn;

	// Выключить насос
	uint8_t				SwitchPumpOff;
	
	// Спец. режим полива (при повышенном давлении) с огранич. по врем., объёму
	uint8_t				SpecialWateringModeOn;
	
	// Таймер полива в спец. режиме с огранич. по врем., объёму, мин
	int32_t				SpModeWateringTimer;

	// Текущий объём перекачанной жидкости в спец. режиме с огранич. по врем., объёму, л
	int32_t				SpModeWateringVolumeCounter;

	// Насос запущен , 0- выключен, 1- включен
	uint8_t				PumpIsStarted;
	
	// Время включения насоса в последнем цикле, сек
	int32_t				PumpStartTimeAtLastCycle;
	
	// Время непрерывной работы насоса в предыдущем цикле, сек
	int32_t				PumpWorkingTimeAtLastCycle;
	
	// Текущее кол-во импульсов турбины между инкрементом счётчика литров
	uint32_t			TurbineImpCounter;
	
	// Кол-во воды, перекачанной насосом в предыдущем цикле, литры
	uint32_t			PumpedQuantityAtLastCycle;

	// t воды при перекачивании, 'С * 10
	int16_t				WaterTempDuringPumping;
	
	// Событие "сухого хода", когда =1
	uint8_t				DryRunDetected;
	
	// Таймаут срабатывания останова насоса по "сухому ходу", сек
	int16_t				PumpDryRunStopTimeout;
	
	
	// Флаг выполнения автоподкачивания воды в текущий момент
	uint8_t				AutoPumpIsStarted;

	// Значение смещения времени включения автоподкачки относительно начала суток, мин
	int16_t				AutoPumpTimeDeltaFromStartOfDay;

	// Кол-во воды для ежесуточного автоподкачивания, литры
	int32_t				AutoPumpVolume;
	
	// Интервал времени между включениями автоподкачивания, мин
	int16_t				AutoPumpTimeInterval;
	
	// Кол-во включений ежесуточного автоподкачивания за сутки, раз
	int16_t				AutoPumpTimes;
	
	
	// Минимальная суточная t воды в источнике, 'С * 10
	int16_t				WellWaterTempMinFor24h;

	// Максимальная суточная t воды в источнике, 'С * 10
	int16_t				WellWaterTempMaxFor24h;

	// Минимальная суточная t воды в накопителе, 'С * 10
	int16_t				TankWaterTempMinFor24h;

	// Максимальная суточная t воды в накопителе, 'С * 10
	int16_t				TankWaterTempMaxFor24h;
	
	// Значение давления воды в системе, атм * 10
	int16_t				WaterPressureValue;
	int16_t				AverageWaterPressureValue;

	// текущая t воды, 'С * 10
	int16_t				CurrentWaterTemp;

	// t воды в источнике, 'С * 10
	int16_t				WellWaterTemp;

	// Уровень воды в источнике, в вольтах/10 датч. давл.
	int16_t				WellWaterLevelInVolts;

	// t воды в накопителе, 'С * 10
	int16_t				TankWaterTemp;

	// Уровень воды в накопителе, в вольтах/10 датч. давл.
	int16_t				TankWaterLevelInVolts;
	
} LastPumpCycle_t;


// Com ports data type def
typedef struct
{
	// Com instance number (COM1 = 1, COM2 = 2, etc)
	uint8_t		ComNum; //__attribute__((aligned(4)));
	
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

} ComPortData_t;


// Ring buffer data type def
typedef struct
{
	// Com link (COM1, COM2, etc)
	uint8_t	ComLink;

	// Size of one element of buffer
	uint8_t	 PBufElementSize;		// uint32_t = 4 bytes
	uint8_t  DBufElementSize;		// uint8_t  = 1 byte

	uint32_t PBuf[TX_POINTERS_BUFFER_SIZE] __attribute__((aligned(4)));		// buffer of pointers, must be a multiple of 2 in degree x; 1,2,4,8,16,32,64,etc
	uint32_t SBuf[TX_POINTERS_BUFFER_SIZE] __attribute__((aligned(4)));		// buffer of data sting sizes, must be a multiple of 2 in degree x; 1,2,4,8,16,32,64,etc
	uint8_t	 DBuf[TX_RING_DATA_BUFFER_SIZE] __attribute__((aligned(4)));	// data buffer, must be a multiple of 2 in degree x; 1,2,4,8,16,32,64,etc

	// Mask for wrapping address inside buffer of pointers
	uint32_t PBufMask; //	= sizeof(PBuf) / PBufElementSize - 1
	// Mask for wrapping address inside data buffer
	uint32_t DBufMask; //	= sizeof(DBuf) / DBufElementSize - 1

	// Pointer to current working string pointer and pointer to size of current string while popping data
	uint32_t PBufPopPtr;
	uint32_t PBufPushPtr;
	uint32_t DBufPushPtr;

	uint8_t BuffersAreBusy;
} RingBuffer_t;


// Jetson Com Port Data Type Def
typedef struct
{
	// Com link (COM1 = External interface for debugging, COM2 = Jetson)
	uint8_t	ComLink;
	
	ComPortData_t *Com;

	// RxD buffer
	uint8_t	RxdBuffer[JETSON_COM_RXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));	

	// TxD buffer to collect data
	uint8_t	TxdBuffer[JETSON_COM_TXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));

	// TxD buffer used to physically send data
	uint8_t	PhTxdBuffer[JETSON_COM_TXD_BUF_SIZE_IN_BYTES];
	
	// Size of data string in PhTxdBuffer
	uint32_t	StringSize;
	
	RingBuffer_t * ComRingBuf;

} JetsonComPort_t;


// External Com Port Data Type Def
typedef struct
{
	// Com link (COM1 = External interface for debugging, COM2 = Jetson)
	uint8_t	ComLink;
	
	ComPortData_t *Com;

	// RxD buffer
	uint8_t	RxdBuffer[EXTERNAL_COM_RXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));	

	// TxD buffer to collect data
	uint8_t	TxdBuffer[EXTERNAL_COM_RXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));

	// TxD buffer used to physically send data
	uint8_t	PhTxdBuffer[EXTERNAL_COM_RXD_BUF_SIZE_IN_BYTES];

	// Size of data string in PhTxdBuffer
	uint32_t	StringSize;
	
	RingBuffer_t * ComRingBuf;
	
} ExtComPort_t;


// Nextion Com Port Data Type Def
typedef struct
{
	// Com link (COM2 = Nextion display)
	uint8_t		ComLink;
	
	ComPortData_t *Com;

	// RxD buffer
	uint8_t	RxdBuffer[NEXTION_COM_RXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));	

	// TxD buffer to collect data
	uint8_t	TxdBuffer[NEXTION_COM_TXD_BUF_SIZE_IN_BYTES]; //__attribute__((aligned(4)));

	// TxD buffer used to physically send data
	uint8_t	PhTxdBuffer[NEXTION_COM_TXD_BUF_SIZE_IN_BYTES];
	
	// Size of data string in PhTxdBuffer
	uint32_t StringSize;
	
	RingBuffer_t * ComRingBuf;
	
	// Data is ready for sending
	uint8_t RefreshReady;
	
	// Display screen number
	uint8_t ScreenNumber;
	
} NextionComPort_t;


// Temperature Sensor Com Port Data Type Def
typedef struct
{
	// Com link (COM5 = DS18B20)
	uint8_t		ComLink;
	
	ComPortData_t *Com;

	// RxD buffer
	uint8_t		RxdBuffer[STRING_LENGHT_FROM_NEXTION]; //__attribute__((aligned(4)));	

	// TxD buffer
	uint8_t		TxdBuffer[STRING_LENGHT_TO_NEXTION];
	
} TempSensorComPort_t;


// Описатель структуры eeprom
typedef struct
{
	// Размер структуры в байтах, заполняется при инициализации структуры
	uint16_t						StructSize;
	Statistics_t* 			Statistics;
	WateringControl_t*	WateringControls;
	Calibrations_t*			Calibrations;
	LastPumpCycle_t*		LastPumpCycle;

} E2p_t;


// Описатель структуры значений АЦП
typedef struct
{
	// ADC channels counts
	uint32_t 								CountsBuf[2] __attribute__((aligned(4)));
	
	// ADC channels values in mv
	int32_t 								VoltsBuf[2] __attribute__((aligned(4)));
	
	uint8_t									DataReady;
	
} Adc_t;


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
} Temperature_t;

#endif

