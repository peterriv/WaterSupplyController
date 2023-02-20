#ifndef			_RING_BUFFER_H
#define 		_RING_BUFFER_H

#include "main.h"


// Adding data to Com port TxD buffer
ReturnCode_t Push_string_to_ring_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t size);

// Popping data to buf from ring data buffer and setting size of data string
ReturnCode_t Pop_string_from_ring_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t * size);

// Checking whether all data are popped from ring buffer
ReturnCode_t Is_all_data_popped_from_ring_buffer(RingBuffer_t * com_ring_buf);

// Calculating size of free space in ring data buffer in bytes
static uint32_t Get_free_space_in_ring_data_buffer(RingBuffer_t * com_ring_buf);

// Copying data to ring data buffer byte by byte with address wrapping
static void Write_to_ring_data_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t size);

// Reading data from ring data buffer byte by byte with address wrapping
static void Read_from_ring_data_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t addr, uint32_t size);

#endif

