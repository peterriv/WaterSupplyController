#ifndef _ONEWIRE_H_
#define _ONEWIRE_H_

#include "main.h"


#define OW_0	        		0x00
#define OW_1	        		0xFF
#define OW_R_1	        	0xFF

#define OW_CMD_SEARCH   	0xF0
#define OW_CMD_MATCH    	0x55


#define OW_SEND_RESET			1
#define OW_NO_RESET				2

#define OW_OK							1
#define OW_ERROR					2
#define OW_NO_DEVICE			3
#define OW_TIMEOUT				4

#define OW_NO_READ				0xFF
#define OW_READ_SLOT			0xFF

#define OW_RESET_COMMAND	0xF0


typedef struct
{
	uint8_t rom_code[8];
	uint8_t id;

}	owdevice_t;

// Link abstract instance of temperature com port to real
void Link_ow_com_port_to_real(UART_HandleTypeDef *RealUart, ComPortData_t *RealCom);
uint8_t OW_Init(void);
uint8_t OW_Reset(void);
uint8_t OW_Search(owdevice_t *owdevices);
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
void OW_MatchRom(uint8_t *romValue);

#endif /* ONEWIRE_H_ */
