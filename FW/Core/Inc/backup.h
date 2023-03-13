#ifndef _BACKUP_H_
#define _BACKUP_H_

#include "main.h"

// Запись данных в BACKUP регистр
// reg_number: 1-42 для stm32f103ret
void Backup_register_write(RTC_HandleTypeDef  * hrtc, uint16_t data, uint8_t reg_number);

// Чтение данных из BACKUP регистра
// reg_number: 1-42 для stm32f103ret
uint16_t Backup_register_read(RTC_HandleTypeDef  * hrtc, uint8_t reg_number);

// Сохранение рабочих переменных в eeprom
void Backup_all_data(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2p_t * e2p);

// Заполнение буфера eeprom и установка контрольных сумм
// Возвращает кол-во байт, занятых в eeprom с учётом 4 байт CRC32
uint32_t Prepare_e2p_buf(CRC_HandleTypeDef * hcrc, E2p_t * e2p);

// Заполнение буфера eeprom и установка контрольных сумм
// Возвращает кол-во байт, занятых в BACKUP с учётом 4 байт CRC32
uint32_t Prepare_backup_buf(CRC_HandleTypeDef * hcrc, E2p_t * e2p);

// Расчёт (от 0 до buf_size-4) и запись в конец буфера контрольной суммы CRC32
void Set_crc32_checksum(CRC_HandleTypeDef * hcrc, uint8_t* buf, uint32_t buf_size);

// Проверка контрольной суммы блока
ReturnCode_t Check_crc32(CRC_HandleTypeDef * hcrc, uint8_t* buf, uint32_t buf_size);

// Чтение времени в секундах от начала суток
int32_t Get_time_in_sec(RTC_HandleTypeDef  * hrtc);

// Получение номера текущих суток (сколько целых суток без основного питания)
uint16_t Get_day_number(RTC_HandleTypeDef  * hrtc);

// Запись времени в аппаратный регистр времени RTC
void Write_time_to_RTC(RTC_HandleTypeDef  * hrtc, int32_t curr_time);

// Обнуление счётчиков суток недельной статистики
void Init_days_of_week_counters(E2p_t * e2p);

// Постраничная (для AT24C32AN по 32 байта) запись буфера в eeprom
// Страница - не более 512 байт
ReturnCode_t Write_to_e2p(I2C_HandleTypeDef  * hi2c, uint8_t * buf, uint32_t buf_size, uint16_t e2p_page_size);

// Запись буфера в регистры BACKUP STM32
ReturnCode_t	Write_to_backup_regs(RTC_HandleTypeDef  * hrtc, uint8_t * buf, uint32_t buf_size);

// Чтение в буфер регистров BACKUP STM32
ReturnCode_t	Read_backup_regs(RTC_HandleTypeDef  * hrtc, uint8_t * buf, uint32_t buf_size);

// Восстановление рабочих переменных из eeprom
void Restore_all_data(CRC_HandleTypeDef * hcrc, I2C_HandleTypeDef  * hi2c, RTC_HandleTypeDef  * hrtc, E2p_t * e2p);

// Инициализация всех переменных
void Set_all_variables_to_default(E2p_t * e2p);

// Инициализация всех переменных, хранимых в BACKUP регистрах
//void Set_bkp_variables_to_default(E2p_t * e2p);

//Коррекция времени (производится раз в сутки) и инкремент суток
void Make_time_correction_and_day_inc(RTC_HandleTypeDef  * hrtc, E2p_t * e2p);

// Сформировать статистику по температурам
void Make_temperature_statistics(E2p_t * e2p, LastPumpCycle_t * last_pump_cycle);

// Сформировать статистику расхода воды
void Make_water_using_statistics(E2p_t * e2p);

// Внесение новых данных и сдвиг недельной статистики
void Push_new_data_to_weekly_stat(E2p_t * e2p);




#endif

