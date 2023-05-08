#ifndef DS18X20_H_
#define DS18X20_H_

#include "main.h"
#include "onewire.h"

#define MAX_SENSORS    1

uint8_t ds18b20_start_convert(void);

// Возвращает кол-во обнаруженных термодатчиков
uint8_t ds18b20_init(UART_HandleTypeDef *RealUart, ComPortData_t *RealCom);

uint8_t ds18b20_get_temp(uint8_t dev_id, float *temp_value);

uint8_t ds18b20_crc8(uint8_t *addr, uint8_t len);

float ds18b20_tconvert(uint8_t LSB, uint8_t MSB);

// Опрос термодатчиков
uint8_t Polling_termosensors(Temperature_t *TermoSensors);




#endif
