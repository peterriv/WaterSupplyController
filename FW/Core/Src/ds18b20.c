
#include "ds18b20.h"

static owdevice_t ds18_sensors[MAX_SENSORS];       //rom code
static uint8_t owdevices = 0;                      //devices index

// Возвращает кол-во обнаруженных термодатчиков
uint8_t ds18b20_init(UART_HandleTypeDef *RealUart, ComPortData_t *RealCom)
{
	uint8_t cnt, sens_cnt = 0;
	
	// Link abstract instance of temperature com port to real
	Link_ow_com_port_to_real(RealUart, RealCom);
	
	// Resets sensors quantity to zero
	owdevices = 0;
	
//	// Resets rom_code value to 0 on all sensors for correct reinitialization
//	while(sens_cnt < MAX_SENSORS)
//	{
//		for(cnt = 0; cnt < ARRAY_SIZE(ds18_sensors[sens_cnt].rom_code); cnt++)
//		{
//			ds18_sensors[sens_cnt].rom_code[cnt] = 0;
//		}
//		
//		sens_cnt++;
//	}

	if(!OW_Init())
	{
			return 0;
	}
	while(OW_Search(ds18_sensors))
	{       
		if(ds18b20_crc8(ds18_sensors[owdevices].rom_code, 7) == ds18_sensors[owdevices].rom_code[7])
		{
			owdevices++;
		}
	}

	return owdevices;
}


uint8_t ds18b20_crc8(uint8_t *addr, uint8_t len) {
	uint8_t crc = 0, inbyte, i, mix;
	
	while (len--) {
		inbyte = *addr++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C;
			}
			inbyte >>= 1;
		}
	}
	return crc;
}


uint8_t ds18b20_start_convert(void) 
{
	uint8_t err;
	
    uint8_t send_buf[2] = {0xCC, 0x44};
    err = OW_Send(OW_SEND_RESET, send_buf, sizeof(send_buf), NULL, NULL, OW_NO_READ);
    
    return err;
}

uint8_t ds18b20_get_temp(uint8_t dev_id, float *temp_value)
{   
	uint8_t err, fbuf[2];
	uint8_t send_buf[12];
	
	send_buf[0] = 0x55;
	memcpy(&send_buf[1], ds18_sensors[dev_id].rom_code, 8);
	send_buf[9] = 0xBE;
	send_buf[10] = 0xFF;
	send_buf[11] = 0xFF;
	
	err = OW_Send(OW_SEND_RESET, send_buf, sizeof(send_buf), fbuf, sizeof(fbuf), 10);

	*temp_value = ds18b20_tconvert(fbuf[0], fbuf[1]);
	
	return err;
}

float ds18b20_tconvert(uint8_t LSB, uint8_t MSB)
{
    float data;
    
    uint16_t temperature;
    
    temperature = LSB | (MSB << 8);

	if (temperature & 0x8000) {
		temperature = ~temperature + 1;
        data = 0.0 - (temperature / 16.0);
        return data;
	}
    data = temperature / 16.0;
    
    return data ;
}


// Опрос термодатчиков
uint8_t Polling_termosensors(Temperature_t *TermoSensors)
{
	uint8_t err;
	static uint8_t termosens_ops_step = 0;
	
	// Если первый шаг, то запуск преобразования
	if ((termosens_ops_step & 0x01) == 0)
	{
		// Команда измерения темпер. всеми датчиками
		err = ds18b20_start_convert();
		termosens_ops_step++;
	}

	// Читаем темпер.
	else if ((termosens_ops_step & 0x01) == 1)
	{
		err = ds18b20_get_temp(0, &TermoSensors->TempSensorsValues[0]);
		if(err != OW_OK)
		{
			TermoSensors->TempSensorsValues[0] = 0;
			
			// Resets sensors quantity to zero
			owdevices = 0;
		}
		termosens_ops_step++;
	}
	
	return err;
}




