#include "onewire.h"

// Pointers to abstract instancies of temperature sensor data type definitions
static UART_HandleTypeDef 			*OwUart;
static ComPortDataTypeDef 			*OwCom;
static USART_TypeDef						*OW_USART;

static uint8_t ow_buf[8];

static uint8_t LastDiscrepancy;
static uint8_t LastDeviceFlag;
static uint8_t DeviceID;
static uint8_t ROM_NO[8];

// Link abstract instance of temperature com port to real
void Link_ow_com_port_to_real(UART_HandleTypeDef *RealUart, ComPortDataTypeDef *RealCom)
{
	OwUart = RealUart;
	OwCom = RealCom;

	if(OwCom->ComNumber == COM1) OW_USART = USART1;
	else if(OwCom->ComNumber == COM2) OW_USART = USART2;
	else if(OwCom->ComNumber == COM3) OW_USART = USART3;
	else if(OwCom->ComNumber == COM4) OW_USART = UART4;
	else if(OwCom->ComNumber == COM5) OW_USART = UART5;
}

static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

static uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}
	return ow_byte;
}

uint8_t OW_Init(void)
{  
  OwUart->Instance = OW_USART;
  OwUart->Init.BaudRate = 9600;
  OwUart->Init.WordLength = UART_WORDLENGTH_8B;
  OwUart->Init.StopBits = UART_STOPBITS_1;
  OwUart->Init.Parity = UART_PARITY_NONE;
  OwUart->Init.Mode = UART_MODE_TX_RX;
  OwUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  OwUart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(OwUart) != HAL_OK)
  {
    Error_Handler();
  }

	// Предотвращение появления ошибки Noise Error при приёме данных по USART
	//OW_USART->CR3 |= USART_CR3_ONEBIT;
	
	return OW_OK;
}

uint8_t OW_Reset(void)
{
	const uint8_t 	ow_reset_com = OW_RESET_COMMAND;
	uint8_t 				ow_presence;
	uint32_t				tick;

  OwUart->Instance = OW_USART;
  OwUart->Init.BaudRate = 9600;
  OwUart->Init.WordLength = UART_WORDLENGTH_8B;
  OwUart->Init.StopBits = UART_STOPBITS_1;
  OwUart->Init.Parity = UART_PARITY_NONE;
  OwUart->Init.Mode = UART_MODE_TX_RX;
  OwUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  OwUart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(OwUart) != HAL_OK)
  {
    Error_Handler();
  }

	OwCom->RxdPacketIsReceived = 0;
	HAL_UART_Receive_IT(OwUart, (uint8_t *) &ow_presence, 1);
	HAL_UART_Transmit(OwUart, (uint8_t *) &ow_reset_com, 1, 10);
	
	tick = HAL_GetTick();
	while (OwCom->RxdPacketIsReceived == 0)
	{
		IWDG->KR = IWDG_KEY_RELOAD;
		
		if (HAL_GetTick() >= tick + 3)	// 1.04 ms needed to read 10 bits at 9600 bod
		{
			return OW_NO_DEVICE;
		}
	}
	
  OwUart->Instance = OW_USART;
  OwUart->Init.BaudRate = 115200;
  OwUart->Init.WordLength = UART_WORDLENGTH_8B;
  OwUart->Init.StopBits = UART_STOPBITS_1;
  OwUart->Init.Parity = UART_PARITY_NONE;
  OwUart->Init.Mode = UART_MODE_TX_RX;
  OwUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  OwUart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(OwUart) != HAL_OK)
  {
    Error_Handler();
  }

	if (ow_presence != 0xF0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}

uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) 
{
	HAL_StatusTypeDef		HAL_func_res;
	uint32_t		tick;

	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;


		OwCom->RxdPacketIsReceived=0;
		HAL_func_res = HAL_UART_Receive_IT(OwUart, ow_buf, 8);
		HAL_func_res = HAL_UART_Transmit(OwUart, ow_buf, 8, 10);

		tick = HAL_GetTick();
		while (OwCom->RxdPacketIsReceived == 0)
		{
			IWDG->KR=IWDG_KEY_RELOAD;
			
			if (HAL_GetTick() >= tick + 2)		// 695 µs needed to read 8 bytes at 115200 bod
			{
				return OW_NO_DEVICE;
			}
		}

		if (readStart == 0 && dLen > 0)
		{
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		}
		else
		{
			if (readStart != OW_NO_READ)
			{
				readStart--;
			}
		}
	}

	return OW_OK;
}
        
static uint8_t OW_SendBits(uint8_t num_bits) {
	uint32_t		tick;

	OwCom->RxdPacketIsReceived = 0;
	HAL_UART_Receive_IT(OwUart, ow_buf, num_bits);
	HAL_UART_Transmit(OwUart, ow_buf, num_bits, 10);	
	
	tick = HAL_GetTick();
	while (OwCom->RxdPacketIsReceived == 0)
	{
		IWDG->KR = IWDG_KEY_RELOAD;
		
		if (HAL_GetTick() >= tick + 2)		// 695 µs needed to read 8 bytes at 115200 bod
		{
			return OW_TIMEOUT;
		}
	}
	return OW_OK;
}

static uint8_t OW_ReadBit(void)
{
	OW_toBits(OW_READ_SLOT, ow_buf);
	OW_SendBits(1);

	if(ow_buf[0] == 0xFF)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void OW_MatchRom(uint8_t *romValue)
{
	uint8_t cmd = OW_CMD_MATCH;

	OW_Send(OW_SEND_RESET, &cmd, sizeof(cmd), 0, 0, OW_NO_READ);  
	OW_Send(OW_NO_RESET, romValue, 8, 0, 0, OW_NO_READ);
}

uint8_t OW_Search(owdevice_t *owdevices) 
{  
	uint8_t id_bit_number = 1, rom_byte_mask = 1;
	uint8_t last_zero = 0, rom_byte_number = 0, search_result = 0;
	uint8_t cmd, id_bit, cmp_id_bit, search_direction;

	// if the last call was not the last one
	if (!LastDeviceFlag)
	{      
    cmd = OW_CMD_SEARCH;
		if (OW_Send(OW_SEND_RESET, &cmd, sizeof(cmd), 0, 0, OW_NO_READ) == OW_NO_DEVICE)
		{			
			LastDiscrepancy = 0;   /* Reset the search */
			LastDeviceFlag = 0;
      DeviceID = 0;
			return 0;
		}
        
		// loop to do the search
		do
		{
			IWDG->KR=IWDG_KEY_RELOAD;
			
			// read a bit and its complement
			id_bit = OW_ReadBit();
			cmp_id_bit = OW_ReadBit();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1))
			{
				break;
			}
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
				{
					search_direction = id_bit;  // bit write value for search
				}
				else
				{
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
					{
						search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					}
					else
					{
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);
					}
					
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0)
					{
						last_zero = id_bit_number;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
				{
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				}
				else
				{
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;
				}
				
				// serial number search direction write bit
				OW_toBits(search_direction, ow_buf);
				OW_SendBits(1);
                
				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0)
				{
					//docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65))
		{
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
			{
				LastDeviceFlag = 1;
			}

			search_result = 1;
			owdevices[DeviceID].id = DeviceID;
			for (int i = 0; i < 8; i++) owdevices[DeviceID].rom_code[i] = ROM_NO[i];
			DeviceID++;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result) {
		LastDiscrepancy = 0;
		LastDeviceFlag = 0;
		search_result = 0;
        DeviceID = 0;
	}
    
	return search_result;
}


