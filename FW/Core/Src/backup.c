
#include "backup.h"

//RTC_DateTypeDef 				CurrentDate = {0};

uint8_t e2p_temp_buf[1024] __attribute__((aligned(4)));

// Запись данных в BACKUP регистр
// reg_number: 1-42 для stm32f103ret
void Backup_register_write(RTC_HandleTypeDef  * hrtc, uint16_t data, uint8_t reg_number)
{
	if(reg_number > RTC_BKP_NUMBER) return;
	
	if((reg_number >= 1) && (reg_number <= 10))
	{
		HAL_RTCEx_BKUPWrite(hrtc, reg_number, data);
	}
	else if((reg_number >= 11) && (reg_number <= 42))
	{
		HAL_RTCEx_BKUPWrite(hrtc, (reg_number + 5), data);
	}
	
	//RTC_BKP_DR1=1234;
	//BKP->DR2=1234;
}


// Чтение данных из BACKUP регистра
// reg_number: 1-42 для stm32f103ret
uint16_t Backup_register_read(RTC_HandleTypeDef  * hrtc, uint8_t reg_number)
{
	uint32_t data;
	
	if(reg_number > RTC_BKP_NUMBER) return 0;
	
	if((reg_number >= 1) && (reg_number <= 10))
	{
		data = HAL_RTCEx_BKUPRead(hrtc, reg_number);
	}
	else if((reg_number >= 11) && (reg_number <= 42))
	{
		data = HAL_RTCEx_BKUPRead(hrtc, reg_number + 5);
	}

	return data & 0x0000FFFF;
}


// Сохранение рабочих переменных в eeprom
void Backup_all_data(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2p_t * e2p)
{
	uint32_t	buf_size;
	
	// Сохраняем номер текущих суток
	//e2p->Statistics->CurrentDayNumber = Get_day_number();
	
	// Инкремент счётчика циклов выключения питания
	e2p->Statistics->PowerOffCycleCounter++;
	
	// Заполнение буфера eeprom и установка контрольных сумм
	buf_size = Prepare_e2p_buf(hcrc, e2p);
		
	// Постраничная (для AT24C32AN по 32 байта) запись буфера в eeprom
	Write_to_e2p(hi2c, e2p_temp_buf, buf_size, E2P_PAGE_SIZE);	
}


// Заполнение буфера eeprom и установка контрольных сумм
// Возвращает кол-во байт, занятых в eeprom с учётом 4 байт CRC32
uint32_t Prepare_e2p_buf(CRC_HandleTypeDef * hcrc, E2p_t * e2p)
{
	uint32_t	e2p_buf_offset, struct_size;
		
	e2p_buf_offset = 0;
	struct_size = e2p->Statistics->StructSize;
	// Копирование из одного буфера по произвольному адресу во 2-ой 
	Copy_buf_random_address((uint8_t*) e2p->Statistics, 0, e2p_temp_buf, e2p_buf_offset, struct_size);
	
	// Адрес следующего байта в буфере для записи
	e2p_buf_offset += struct_size;
	struct_size = e2p->Calibrations->StructSize;
	// Копирование из одного буфера по произвольному адресу во 2-ой 
	Copy_buf_random_address((uint8_t*) e2p->Calibrations, 0, e2p_temp_buf, e2p_buf_offset, struct_size);
	
	// Адрес следующего байта в буфере для записи
	e2p_buf_offset += struct_size;
	struct_size = e2p->WateringControls->StructSize;
	// Копирование из одного буфера по произвольному адресу во 2-ой 
	Copy_buf_random_address((uint8_t*) e2p->WateringControls, 0, e2p_temp_buf, e2p_buf_offset, struct_size);
	
	// Адрес следующего байта в буфере для записи
	e2p_buf_offset += struct_size;
	struct_size = e2p->LastPumpCycle->StructSize;
	// Копирование из одного буфера по произвольному адресу во 2-ой 
	Copy_buf_random_address((uint8_t*) e2p->LastPumpCycle, 0, e2p_temp_buf, e2p_buf_offset, struct_size);
	
	// Адрес следующего байта в буфере для записи
	e2p_buf_offset += struct_size;

	// Учёт 4 байтов CRC32
	e2p_buf_offset += 4;
	
	// Расчёт и запись в конец буфера контрольной суммы CRC32
	Set_crc32_checksum(hcrc, e2p_temp_buf, e2p_buf_offset);
		
	return e2p_buf_offset;
}


// Заполнение буфера eeprom и установка контрольных сумм
// Возвращает кол-во байт, занятых в BACKUP с учётом 4 байт CRC32
uint32_t Prepare_backup_buf(CRC_HandleTypeDef * hcrc, E2p_t * e2p)
{
	uint32_t	e2p_buf_offset, struct_size;
		
	e2p_buf_offset = 0;
	struct_size = e2p->WateringControls->StructSize;
	// Копирование из одного буфера по произвольному адресу во 2-ой 
	Copy_buf_random_address((uint8_t*) e2p->WateringControls, 0, e2p_temp_buf, e2p_buf_offset, struct_size);
	
	// Адрес следующего байта в буфере для записи
	e2p_buf_offset += struct_size;
	struct_size = e2p->LastPumpCycle->StructSize;
	// Копирование из одного буфера по произвольному адресу во 2-ой 
	Copy_buf_random_address((uint8_t*) e2p->LastPumpCycle, 0, e2p_temp_buf, e2p_buf_offset, struct_size);

	// Адрес следующего байта в буфере для записи
	e2p_buf_offset += struct_size;
	
	// Учёт 4 байтов CRC32
	e2p_buf_offset += 4;
	// Расчёт и запись в конец буфера контрольной суммы CRC32
	Set_crc32_checksum(hcrc, e2p_temp_buf, e2p_buf_offset);
		
	return e2p_buf_offset;
}


// Расчёт (от 0 до buf_size-4) и запись в конец буфера контрольной суммы CRC32
void Set_crc32_checksum(CRC_HandleTypeDef * hcrc, uint8_t* buf, uint32_t buf_size)
{
	uint8_t					temp8, byte_pos;
	uint32_t				idx, sum;
			
	// Расчёт к.с. *********************************************************
				
  /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(hcrc);

  sum = 0;
	byte_pos = 3;
	/* Enter Data to the CRC calculator */
  for(idx = 0; idx < (buf_size - 4); idx++)
  {
		temp8 = buf[idx];
		sum |= ((uint32_t) (temp8) << (byte_pos * 8));
		byte_pos--;
		if (byte_pos > 3)
		{
			byte_pos = 3;
			CRC->DR = sum;
			sum = 0;
		}
	}

	// Дополняем нулями до 4-х байтового числа
	if (byte_pos != 3)
	{
		while (byte_pos < 3)
		{
			sum |= ((uint32_t) (0x00) << (byte_pos * 8));
			byte_pos--;
		}
		
		CRC->DR = sum;
	}		

	/* Return the CRC computed value */
	sum = CRC->DR;

	if (idx < buf_size)
	{
		// Сохраняем старший байт значения к.с. CRC32
		buf[idx++] = (uint8_t) ((sum & 0xFF000000) >> 24);
	}
	
	if (idx < buf_size)
	{
		// Сохраняем третий байт значения к.с. CRC32
		buf[idx++] = (uint8_t) ((sum & 0x00FF0000) >> 16);
	}

	if (idx < buf_size)
	{
		// Сохраняем второй байт значения к.с. CRC32
		buf[idx++] = (uint8_t) ((sum & 0x0000FF00) >> 8);
	}

	if (idx < buf_size)
	{
	// Сохраняем младший байт значения к.с. CRC32
	buf[idx++] = (uint8_t) (sum & 0x000000FF);
	}
}


// Проверка контрольной суммы блока
ReturnCode_t Check_crc32(CRC_HandleTypeDef * hcrc, uint8_t* buf, uint32_t buf_size)
{
	uint8_t					temp8, byte_pos;
	uint32_t				idx, sum;
			
	// Проверка к.с. *********************************************************
				
  /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(hcrc);

  sum = 0;
	byte_pos = 3;
	/* Enter Data to the CRC calculator */
  for(idx = 0; idx < (buf_size - 4); idx++)
  {
		temp8 = buf[idx];
		sum |= ((uint32_t) (temp8) << (byte_pos * 8));
		byte_pos--;
		if (byte_pos > 3)
		{
			byte_pos = 3;
			CRC->DR = sum;
			sum = 0;
		}
	}

	// Дополняем нулями до 4-х байтового числа
	if (byte_pos != 3)
	{
		while (byte_pos < 3)
		{
			sum |= ((uint32_t) (0x00) << (byte_pos * 8));
			byte_pos--;
		}
		
		CRC->DR = sum;
	}		

  /* Return the CRC computed value */
  sum = CRC->DR;

	// Сравниваем старший байт значения к.с. CRC32
	idx = buf_size - 4;
	temp8 = buf[idx++];
	byte_pos = (uint8_t) ( (sum & 0xFF000000) >> 24);

	if (temp8 != byte_pos)
	{
		return CheckSumError;
	}
	
	// Сравниваем третий байт значения к.с. CRC32
	temp8 = buf[idx++];
	byte_pos = (uint8_t) ( (sum & 0x00FF0000) >> 16);

	if (temp8 != byte_pos)
	{
		return CheckSumError;
	}
	
	// Сравниваем второй байт значения к.с. CRC32
	temp8 = buf[idx++];
	byte_pos = (uint8_t) ( (sum & 0x0000FF00) >> 8);

	if (temp8 != byte_pos)
	{
		return CheckSumError;
	}

	// Сравниваем младший байт значения к.с. CRC32
	temp8 = buf[idx];
	byte_pos = (uint8_t) (uint8_t) (sum & 0x000000FF);

	if (temp8 != byte_pos)
	{
		return CheckSumError;
	}
	
	return OK;
}


// Чтение времени в секундах от начала суток
int32_t Get_time_in_sec(RTC_HandleTypeDef  * hrtc)
{
	RTC_TimeTypeDef 	CurrentTime = {0};
  int32_t						curr_time_in_sec;
	
	if (HAL_RTC_GetTime(hrtc, &CurrentTime, RTC_FORMAT) != HAL_OK)
  {
    Error_Handler();
  }
	
	curr_time_in_sec = CurrentTime.Hours * 60 * 60;
	curr_time_in_sec += CurrentTime.Minutes * 60;
	curr_time_in_sec += CurrentTime.Seconds;
	
	return curr_time_in_sec;
}


// Получение номера текущих суток (сколько целых суток без основного питания)
uint16_t Get_day_number(RTC_HandleTypeDef  * hrtc)
{ 
	int16_t			days_elapsed = 0U;
  uint16_t 		high1 = 0U, high2 = 0U, low = 0U;
  uint32_t 		counter_time = 0U, counter_alarm = 0U, hours = 0U;

  high1 = READ_REG(RTC->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(RTC->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(RTC->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  {
    // In this case the counter roll over during reading of CNTL and CNTH registers,
    // read again CNTL register then return the counter value
    counter_time = (((uint32_t) high2 << 16U) | READ_REG(RTC->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  {
    // No counter roll over during reading of CNTL and CNTH registers, counter
    // value is equal to first value of CNTL and CNTH
    counter_time = (((uint32_t) high1 << 16U) | low);
  }
	
  // Fill the structure fields with the read parameters
  hours = counter_time / 3600U;

  if (hours >= 24U)
  {
    // Get number of days elapsed from last calculation
    days_elapsed = (hours / 24U);
	}
	
	return days_elapsed;
}


// Запись времени в аппаратный регистр времени RTC
void Write_time_to_RTC(RTC_HandleTypeDef  * hrtc, int32_t curr_time)
{
	RTC_TimeTypeDef 	CurrentTime = {0};

	CurrentTime.Hours = curr_time / 3600;
	CurrentTime.Minutes = (curr_time % 3600) / 60;
	CurrentTime.Seconds = (curr_time % 3600) % 60;

	if (HAL_RTC_SetTime(hrtc, &CurrentTime, RTC_FORMAT) != HAL_OK)
  {
    Error_Handler();
  }
}


// Постраничная (для AT24C32AN по 32 байта) запись буфера в eeprom
// Страница - не более 512 байт
ReturnCode_t Write_to_e2p(I2C_HandleTypeDef  * hi2c, uint8_t * buf, uint32_t buf_size, uint16_t e2p_page_size)
{
	HAL_StatusTypeDef		HAL_func_res;
	uint32_t 			source_buf_idx = 0;
	uint32_t 			page_buf_idx;
	uint32_t			e2p_page_num = 0;
	uint32_t			e2p_pages_to_write;
	uint8_t				page_buf[E2P_PAGE_MAX_SIZE];
	
	if (e2p_page_size > E2P_PAGE_MAX_SIZE) return E2pPageSizeTooBig;
	
	// Определение кол-ва страниц eeprom для записи
	e2p_pages_to_write = buf_size / e2p_page_size;
	// Если получилось, что вышли за границу страницы, то берем ещё одну страницу
	if (buf_size % e2p_page_size > 0) e2p_pages_to_write += 1;
	
	while (e2p_page_num < e2p_pages_to_write)
	{
		// Заполнение страницы для записи в eeprom
		for (page_buf_idx = 0; page_buf_idx < e2p_page_size; page_buf_idx++)
		{		
			// Если не вышли за пределы буфера
			if (source_buf_idx < buf_size)
			{
				page_buf[page_buf_idx] = buf[source_buf_idx];
				source_buf_idx++;
			}
			// Добиваем страницу значениями 0xFF
			else
			{
				page_buf[page_buf_idx] = 0xFF;
			}
		}
		
		// Запись страницы в eeprom
		HAL_func_res = HAL_I2C_Mem_Write(hi2c, 0xA0, e2p_page_size * e2p_page_num, 4096, page_buf, e2p_page_size, 100);
		if (HAL_func_res != HAL_OK) return E2pMemoryWriteError;
		
		e2p_page_num++;
		// Пауза для записи страницы (по док-ции на eeprom - 5 мсек)
		HAL_Delay(5);
	}

	return OK;
}


// Запись буфера в регистры BACKUP STM32
ReturnCode_t	Write_to_backup_regs(RTC_HandleTypeDef  * hrtc, uint8_t * buf, uint32_t buf_size)
{
	static uint16_t*			buf_idx16;
	static uint16_t 			bkp_regs_counter;
	static uint16_t			data16;
	
	// Если размер буфера для записи > размера BACKUP регистров
	if (buf_size > RTC_BKP_NUMBER * 2) return BackupRegsSizeExceeded;
	
	buf_idx16 = (uint16_t *) buf;
		
	for (bkp_regs_counter = 1; bkp_regs_counter <= buf_size / 2; bkp_regs_counter++)
	{
		data16 = *buf_idx16++;

		// Запись данных в BACKUP регистр
		Backup_register_write(hrtc, data16, bkp_regs_counter);		
	}

	return OK;		
}


// Чтение в буфер регистров BACKUP STM32
ReturnCode_t	Read_backup_regs(RTC_HandleTypeDef  * hrtc, uint8_t * buf, uint32_t buf_size)
{
	uint16_t*			buf_idx16;
	uint16_t 			bkp_regs_counter;
	uint16_t			data16;
	
	// Если размер буфера для записи > размера BACKUP регистров
	if (buf_size > RTC_BKP_NUMBER * 2) return BufferSizeExceeded;
	
	buf_idx16 = (uint16_t *) buf;
		
	for (bkp_regs_counter = 1; bkp_regs_counter <= buf_size / 2; bkp_regs_counter++)
	{

		// Запись данных в BACKUP регистр
		data16 = Backup_register_read(hrtc, bkp_regs_counter);		
		// Запись в буфер eeprom
		*buf_idx16++ = data16;
	}

	return OK;		
}


// Восстановление рабочих переменных из eeprom
void Restore_all_data(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2p_t * e2p)
{
	ReturnCode_t					func_res;
	HAL_StatusTypeDef		HAL_func_res;
	static uint32_t			struct_size;
	uint32_t						e2p_buf_offset;
//	uint16_t						temp16;
		
	// Проверка наличия и готовности eeprom
	HAL_func_res = HAL_I2C_IsDeviceReady(hi2c, 0xA0, 1, 100);
	
	if (HAL_func_res == HAL_OK)
	{
		struct_size = e2p->Statistics->StructSize + e2p->Calibrations->StructSize + \
									e2p->WateringControls->StructSize + e2p->LastPumpCycle->StructSize;

		// Учёт CRC32
		struct_size += 4;

		// Восстановление структур из e2p
		HAL_func_res = HAL_I2C_Mem_Read(hi2c, 0xA0, 0x0000, 4096, e2p_temp_buf, struct_size, 100);
	
		// Проверка контрольной суммы блока
		if ((Check_crc32(hcrc, e2p_temp_buf, struct_size)) == OK)
		{
			e2p_buf_offset = 0;
			// Заполнение 1-ой структуры данными
			struct_size = e2p->Statistics->StructSize;
			// Копирование из одного буфера по произвольному адресу во 2-ой 
			Copy_buf_random_address(e2p_temp_buf, e2p_buf_offset,(uint8_t*) e2p->Statistics, 0, struct_size);
			
			e2p_buf_offset += struct_size;
			// Заполнение 2-ой структуры данными
			struct_size = e2p->Calibrations->StructSize;
			// Копирование из одного буфера по произвольному адресу во 2-ой 
			Copy_buf_random_address(e2p_temp_buf, e2p_buf_offset,(uint8_t*) e2p->Calibrations, 0, struct_size);

			e2p_buf_offset += struct_size;
			// Заполнение 3-ей структуры данными
			struct_size = e2p->WateringControls->StructSize;
			// Копирование из одного буфера по произвольному адресу во 2-ой 
			Copy_buf_random_address(e2p_temp_buf, e2p_buf_offset,(uint8_t*) e2p->WateringControls, 0, struct_size);
			
			e2p_buf_offset += struct_size;
			// Заполнение 4-ой структуры данными
			struct_size = e2p->LastPumpCycle->StructSize;
			// Копирование из одного буфера по произвольному адресу во 2-ой 
			Copy_buf_random_address(e2p_temp_buf, e2p_buf_offset,(uint8_t*) e2p->LastPumpCycle, 0, struct_size);
			
//			temp16 = Get_day_number(hrtc);
//			// Проверка, начались ли новые сутки после вкл. питания
//			if (temp16 != 0)
//			{
//				// Если больше суток без основного питания, то суммируем кол-во
//				e2p->Statistics->CurrentDayNumber += temp16;

//				// сброс события "сухого хода" при смене суток
//				e2p->LastPumpCycle->DryRunDetected = 0;
//				// Разрешение повторной попытки автоподкачки воды при смене суток
//				e2p->LastPumpCycle->AutoPumpingMode = 0;
//				
//				e2p->LastPumpCycle->WellWaterTempMinFor24h = 0;
//				e2p->LastPumpCycle->WellWaterTempMaxFor24h = 0;
//				e2p->LastPumpCycle->TankWaterTempMinFor24h = 0;
//				e2p->LastPumpCycle->TankWaterTempMaxFor24h = 0;	
//			}
		}
		// Если не совпадают, то инициализация всех переменных (хранимых в e2p)
		else
		{
			Set_all_variables_to_default(e2p);
		}
	}
	// Если не работает eeprom, то инициализация всех переменных  при подаче питания
	else
	{
		Set_all_variables_to_default(e2p);
	}
}


// Инициализация всех переменных
void Set_all_variables_to_default(E2p_t * e2p)
{
	// e2p->Statistics
	{
		// Счётчик циклов выключения питания
		e2p->Statistics->PowerOffCycleCounter = 0;
		
		// Время работы УФ лампы, секунд
		e2p->Statistics->UvLampWorkingTime = 0;
		// Счётчик циклов включения питания УФ лампы
		e2p->Statistics->UvLampPowerOnCycleCounter = 0;
		
		// Общее время работы контроллера, секунд
		e2p->Statistics->TotalControllerWorkingTime = 0;
		// Общее время работы насоса, секунд
		e2p->Statistics->TotalPumpWorkingTime = 0;
		// Общее кол-во воды, перекачанной насосом, л
		e2p->Statistics->WaterPumpedTotal = 0;
		
		// Кол-во воды, перекачанной за текущие сутки, л
		e2p->Statistics->PumpedWaterQuantityToday = 0;
		// Кол-во воды, перекачанной за вчерашние сутки, л
		e2p->Statistics->PumpedWaterQuantity1dayAgo = 0;
		// Кол-во воды, перекачанной за позавчерашние сутки, л
		e2p->Statistics->PumpedWaterQuantity2daysAgo = 0;
		// Кол-во воды, перекачанной в течение 3-х суток назад, л
		e2p->Statistics->PumpedWaterQuantity3daysAgo = 0;
		// Кол-во воды, перекачанной в течение 4-х суток назад, л
		e2p->Statistics->PumpedWaterQuantity4daysAgo = 0;
		// Кол-во воды, перекачанной в течение 5-х суток назад, л
		e2p->Statistics->PumpedWaterQuantity5daysAgo = 0;
		// Кол-во воды, перекачанной в течение 6-х суток назад, л
		e2p->Statistics->PumpedWaterQuantity6daysAgo = 0;
		// Кол-во воды, перекачанной в течение 7-х суток назад, л
		e2p->Statistics->PumpedWaterQuantity7daysAgo = 0;
		// Кол-во воды, перекачанной за последние 7 дней (посуточная сумма)
		e2p->Statistics->PumpedWaterQuantityLastWeek = 0;
	}
	
	// e2p->Calibrations
	{
		e2p->Calibrations->SpModeWateringVolume = 0;
		e2p->Calibrations->SpModeWateringTime = 0;
		e2p->Calibrations->SpModePumpOffPressure = 0;
		e2p->Calibrations->WaterCounterLitersPerImpulse = 0;
		e2p->Calibrations->TurbineImpulsesPerLiter = 0;
		e2p->Calibrations->PumpOnPressure = 0;
		e2p->Calibrations->PumpOffPressure = 0;
		e2p->Calibrations->PsensorMinPressure = 0;
		e2p->Calibrations->PsensorMaxPressure = 0;
		e2p->Calibrations->PsensorMinPressureVoltage = 0;
		e2p->Calibrations->PsensorMaxPressureVoltage = 0;
		e2p->Calibrations->SourcePsensorMinPressureVoltage = 0;
		e2p->Calibrations->SourcePsensorMaxPressureVoltage = 0;
		e2p->Calibrations->TankPsensorMinPressureVoltage = 0;
		e2p->Calibrations->TankPsensorMaxPressureVoltage = 0;
		e2p->Calibrations->TimeCorrectionValue = 0;
	}
	
	// e2p->WateringControls
	{
		e2p->WateringControls->CurrWateringOutputNumber = 1;
		e2p->WateringControls->out1_interval_time = 0;
		e2p->WateringControls->out1_working_time = 0;
		e2p->WateringControls->out1_zero_clock_time_delta = 0;
		e2p->WateringControls->out2_interval_time = 0;
		e2p->WateringControls->out2_working_time = 0;
		e2p->WateringControls->out2_zero_clock_time_delta = 0;
		e2p->WateringControls->out3_interval_time = 0;
		e2p->WateringControls->out3_working_time = 0;
		e2p->WateringControls->out3_zero_clock_time_delta = 0;
		e2p->WateringControls->out4_interval_time = 0;
		e2p->WateringControls->out4_working_time = 0;
		e2p->WateringControls->out4_zero_clock_time_delta = 0;
		e2p->WateringControls->out5_interval_time = 0;
		e2p->WateringControls->out5_working_time = 0;
		e2p->WateringControls->out5_zero_clock_time_delta = 0;
		e2p->WateringControls->out6_interval_time = 0;
		e2p->WateringControls->out6_working_time = 0;
		e2p->WateringControls->out6_zero_clock_time_delta = 0;
		
		// Текущее состояние активности автополива (исп. для отображения на дисплее)
		//e2p->WateringControls->AutoWatering = 0;
	}
	
	// e2p->LastPumpCycle
	{
		// Включить насос
		//sysState->pumpCtrlComms->SwitchPumpOn = 0;
		// Выключить насос
		//sysState->pumpCtrlComms->SwitchPumpOff = 0;
		// Спец. режим полива (при повышенном давлении) с огранич. по врем., объёму
		//e2p->LastPumpCycle->SpecialWateringModeOn = 0;
		// Спец. режим полива (при повышенном давлении) с огранич. по врем., объёму
		//e2p->LastPumpCycle->SpecialWateringModeOff = 0;
		// Таймер полива в спец. режиме с огранич. по врем., объёму, мин
		//e2p->LastPumpCycle->SpModeWateringTimer = 0;
		// Текущий объём перекачанной жидкости в спец. режиме с огранич. по врем., объёму, л
		e2p->LastPumpCycle->SpModeWateringVolumeCounter = 0;
		// Насос запущен , 0- выключен, 1- включен
		//sysState->PumpIsStarted = 0;
		// Время включения насоса в последнем цикле, сек
		e2p->LastPumpCycle->PumpStartTimeAtLastCycle = 0;
		// Время непрерывной работы насоса в предыдущем цикле, сек
		e2p->LastPumpCycle->PumpWorkingTimeAtLastCycle = 0;
		// Кол-во воды, перекачанной насосом в предыдущем цикле, литры * 10  (десятки литров)
		e2p->LastPumpCycle->WaterPumpedAtLastCycle = 0;
		// t воды при перекачивании, 'С * 10
		e2p->LastPumpCycle->WaterTempDuringPumping = 0;
		// Событие "сухого хода", когда = 1
		//e2p->LastPumpCycle->DryRunDetected = 0;
		// Таймаут срабатывания останова насоса по "сухому ходу", сек
		e2p->LastPumpCycle->PumpDryRunStopTimeout = 0;		
		// Флаг выполнения автоподкачивания воды в текущий момент
		//e2p->LastPumpCycle->AutoPumpingMode = 0;
		// Значение смещения времени включения автоподкачки относительно начала суток, мин
		e2p->LastPumpCycle->AutoPumpTimeDeltaFromStartOfDay = 0;
		// Кол-во воды для ежесуточного автоподкачивания, литры * 10  (десятки литров)
		e2p->LastPumpCycle->AutoPumpVolume = 0;
		// Интервал времени между включениями автоподкачивания, мин
		e2p->LastPumpCycle->AutoPumpTimeInterval = 0;
		// Кол-во включений ежесуточного автоподкачивания за сутки, раз
		e2p->LastPumpCycle->AutoPumpTimes = 0;		
		// Минимальная суточная t воды в источнике, 'С * 10
		e2p->LastPumpCycle->WellWaterTempMinFor24h = 0;
		// Максимальная суточная t воды в источнике, 'С * 10
		e2p->LastPumpCycle->WellWaterTempMaxFor24h = 0;
		// Минимальная суточная t воды в накопителе, 'С * 10
		e2p->LastPumpCycle->TankWaterTempMinFor24h = 0;
		// Максимальная суточная t воды в накопителе, 'С * 10
		e2p->LastPumpCycle->TankWaterTempMaxFor24h = 0;
		// Значение давления воды в системе, атм * 10
		//e2p->LastPumpCycle->WaterPressure = 0;
		//e2p->LastPumpCycle->AverageWaterPressure = 0;
		// Текущее кол-во импульсов турбины между инкрементом счётчика литров
		e2p->LastPumpCycle->TurbineImpCounter = 0;

		// текущая t воды, 'С * 10
		//e2p->LastPumpCycle->CurrentWaterTemp = 0;
		// t воды в источнике, 'С * 10
		//e2p->LastPumpCycle->WellWaterTemp = 0;
		// Уровень воды в источнике, в вольтах/10 датч. давл.
		//e2p->LastPumpCycle->WellWaterLevelInVolts = 0;
		// t воды в накопителе, 'С * 10
		//e2p->LastPumpCycle->TankWaterTemp = 0;
		// Уровень воды в накопителе, в вольтах/10 датч. давл.
		//e2p->LastPumpCycle->TankWaterLevelInVolts = 0;


	}
}


//Коррекция времени (производится раз в сутки) и инкремент суток
void Make_time_correction_and_day_inc(RTC_HandleTypeDef * hrtc, E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static int8_t		weekly_cal_value = 0;
	static uint8_t	daily_cal_is_done = 0;
	static int8_t		time_corr_val_prev = 0;
	static int32_t	time_in_seconds_prev = 0;
	static uint8_t	weekly_cal_days_counter = 0;
	
	// Начальная установка, либо если 0
	if(time_corr_val_prev == 0) time_corr_val_prev = e2p->Calibrations->TimeCorrectionValue;

	// Если изменили значение коррекции времени, то переинициализация рабочей переменной
	if(e2p->Calibrations->TimeCorrectionValue != time_corr_val_prev)
	{
		// сброс флага проведения калибровки
		daily_cal_is_done = 0;

		weekly_cal_days_counter = 0;
		// Загрузка переменной, с которой производится ежесуточная коррекция каждую неделю работы
		weekly_cal_value = e2p->Calibrations->TimeCorrectionValue;
		
	}

	
	// Проверка смены суток
	if ((time_in_seconds_prev > 0) && (sysState->TimeInSeconds == 0))
	{
		// сброс флага проведения калибровки
		daily_cal_is_done = 0;

		// Загрузка переменной, с которой производится ежесуточная коррекция каждую неделю работы
		if(weekly_cal_days_counter == 0) weekly_cal_value = e2p->Calibrations->TimeCorrectionValue;
		
		weekly_cal_days_counter++;
	}
	
	// Если не выполнялась коррекция времени в текущих сутках
	if (daily_cal_is_done == 0)
	{
		// Коррекция времени при положительной поправке
		if (weekly_cal_value > 0)
		{
			// Коррекция времени вперёд в конце суток
			if (sysState->TimeInSeconds + 1 >= 86398)
			{
				sysState->TimeInSeconds += 1;
				if(weekly_cal_value > 0) weekly_cal_value--;
				
				// Запись обновлённого значения времени в аппаратный регистр времени RTC
				Write_time_to_RTC(hrtc, sysState->TimeInSeconds);
				
				if(weekly_cal_days_counter >= 7) weekly_cal_days_counter = 0;
				
				daily_cal_is_done = 1;
			}
		}
		// Коррекция времени при отрицательной поправке
		else if (weekly_cal_value < 0)
		{
			// Коррекция времени назад в конце суток
			if (sysState->TimeInSeconds >= 86398)
			{
				sysState->TimeInSeconds += 1;
				if(weekly_cal_value < 0) weekly_cal_value++;

				// Запись обновлённого значения времени в аппаратный регистр времени RTC
				Write_time_to_RTC(hrtc, sysState->TimeInSeconds);

				if(weekly_cal_days_counter >= 7) weekly_cal_days_counter = 0;
				
				daily_cal_is_done = 1;
			}
		}		
	}
	
	time_in_seconds_prev = sysState->TimeInSeconds;
}


// Сформировать статистику по температурам
void Make_temperature_statistics(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static uint8_t	WellWaterTemp_already_set = 0;
	static uint8_t	TankWaterTemp_already_set = 0;
	static int32_t	time_in_seconds_prev = 0;
	
	// Проверка смены суток для сброса суточных накоплений
	if ((time_in_seconds_prev > 0) && (sysState->TimeInSeconds == 0))
	{		
		e2p->LastPumpCycle->WellWaterTempMinFor24h = 0;
		e2p->LastPumpCycle->WellWaterTempMaxFor24h = 0;
		e2p->LastPumpCycle->TankWaterTempMinFor24h = 0;
		e2p->LastPumpCycle->TankWaterTempMaxFor24h = 0;		
	}
	// Если сутки не менялись
	else
	{
		// Если темп. в источнике стала отлична от нуля, т.е. первично пришли данные с датчиков
		if ((sysState->WellWaterTemp != 0) && (WellWaterTemp_already_set == 0))
		{
			// Минимальная суточная t воды в источнике, 'С * 10
			e2p->LastPumpCycle->WellWaterTempMinFor24h = sysState->WellWaterTemp;			
			WellWaterTemp_already_set = 1;
		}	

		// Если текущая темп. воды в источнике стала < наименьшей за сутки, то обновляем
		if (sysState->WellWaterTemp < e2p->LastPumpCycle->WellWaterTempMinFor24h)
		{
			e2p->LastPumpCycle->WellWaterTempMinFor24h = sysState->WellWaterTemp;
			// Если темп. упадёт до 0 при понижении температуры, чтобы правильно учесть
			//WellWaterTemp_already_set = 1;
		}

		// Если текущая темп. воды в источнике стала > наибольшей за сутки, то обновляем
		if (sysState->WellWaterTemp > e2p->LastPumpCycle->WellWaterTempMaxFor24h)
		{
			e2p->LastPumpCycle->WellWaterTempMaxFor24h = sysState->WellWaterTemp;
		}


		// Если темп. в накопителе стала отлична от нуля, т.е. первично пришли данные с датчиков
		if ((sysState->TankWaterTemp != 0)&&(TankWaterTemp_already_set==0))
		{
			// Минимальная суточная t воды в накопителе, 'С * 10
			e2p->LastPumpCycle->TankWaterTempMinFor24h = sysState->TankWaterTemp;			
			TankWaterTemp_already_set = 1;
		}	
		
		// Если текущая темп. воды в накопителе стала < наименьшей за сутки, то обновляем
		if (sysState->TankWaterTemp < e2p->LastPumpCycle->TankWaterTempMinFor24h)
		{
			e2p->LastPumpCycle->TankWaterTempMinFor24h = sysState->TankWaterTemp;
			// Если темп. упадёт до 0 при понижении температуры, чтобы правильно учесть
			//TankWaterTemp_already_set = 1;
		}

		// Если текущая темп. воды в накопителе стала > наибольшей за сутки, то обновляем
		if (sysState->TankWaterTemp > e2p->LastPumpCycle->TankWaterTempMaxFor24h)
		{
			e2p->LastPumpCycle->TankWaterTempMaxFor24h = sysState->TankWaterTemp;
		}
	}

	// Учитываем, когда насос запущен
	if (sysState->PumpIsStarted)
	{
		// t воды при перекачивании, 'С * 10
		e2p->LastPumpCycle->WaterTempDuringPumping = sysState->CurrentWaterTemp;
	}
	
	time_in_seconds_prev = sysState->TimeInSeconds;	
}


// Сформировать статистику расхода воды
void Make_water_using_statistics(E2p_t * e2p, CurrentSystemState_t * sysState)
{
	static int32_t	time_in_seconds_prev = 0;
	static uint8_t	pump_previous_state = 0;

	// Если насос только выключился
	if ((sysState->PumpIsStarted == 0) && (pump_previous_state == 1))
	{
		// Собираем суточную статистику перекачанного объёма воды
		e2p->Statistics->PumpedWaterQuantityToday += e2p->LastPumpCycle->WaterPumpedAtLastCycle;
	}
	
	pump_previous_state = sysState->PumpIsStarted;
	
	// Проверка смены суток для сброса флагов и счётчиков
	if ((time_in_seconds_prev > 0) && (sysState->TimeInSeconds == 0))
	{		
		// Вносим свежую суточную статистику по расходу воды
		Push_new_data_to_weekly_stat(e2p);

		e2p->Statistics->PumpedWaterQuantityToday = 0;
	}
		
	// Обновление недельной суммы расхода воды без учёта текущих суток
	e2p->Statistics->PumpedWaterQuantityLastWeek =	
																e2p->Statistics->PumpedWaterQuantity1dayAgo + e2p->Statistics->PumpedWaterQuantity2daysAgo /
																e2p->Statistics->PumpedWaterQuantity3daysAgo + e2p->Statistics->PumpedWaterQuantity4daysAgo /
																e2p->Statistics->PumpedWaterQuantity5daysAgo + e2p->Statistics->PumpedWaterQuantity6daysAgo /
																e2p->Statistics->PumpedWaterQuantity7daysAgo;

	time_in_seconds_prev = sysState->TimeInSeconds;	
}


// Внесение новых данных и сдвиг недельной статистики
void Push_new_data_to_weekly_stat(E2p_t * e2p)
{
	// Кол-во воды, перекачанной в течение 7-х суток назад, л
	e2p->Statistics->PumpedWaterQuantity7daysAgo = e2p->Statistics->PumpedWaterQuantity6daysAgo;
	// Кол-во воды, перекачанной в течение 6-х суток назад, л
	e2p->Statistics->PumpedWaterQuantity6daysAgo = e2p->Statistics->PumpedWaterQuantity5daysAgo;
	// Кол-во воды, перекачанной в течение 5-х суток назад, л
	e2p->Statistics->PumpedWaterQuantity5daysAgo = e2p->Statistics->PumpedWaterQuantity4daysAgo;
	// Кол-во воды, перекачанной в течение 4-х суток назад, л
	e2p->Statistics->PumpedWaterQuantity4daysAgo = e2p->Statistics->PumpedWaterQuantity3daysAgo;
	// Кол-во воды, перекачанной в течение 3-х суток назад, л
	e2p->Statistics->PumpedWaterQuantity3daysAgo = e2p->Statistics->PumpedWaterQuantity2daysAgo;
	// Кол-во воды, перекачанной за позавчерашние сутки, л
	e2p->Statistics->PumpedWaterQuantity2daysAgo = e2p->Statistics->PumpedWaterQuantity1dayAgo;
	// Кол-во воды, перекачанной за вчерашние сутки, л
	e2p->Statistics->PumpedWaterQuantity1dayAgo = e2p->Statistics->PumpedWaterQuantityToday;
}




