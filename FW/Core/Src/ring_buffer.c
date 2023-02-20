
#include "ring_buffer.h"

// Pushing data string to ring data buffer
ReturnCode_t Push_string_to_ring_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t size)
{
	if(com_ring_buf->BuffersAreBusy) return BuffersAreBusy;
	com_ring_buf->BuffersAreBusy = 1;

	// Checking size of data
	if((uint32_t) size > (uint32_t) (sizeof(com_ring_buf->DBuf)) / com_ring_buf->DBufElementSize)
	{
		com_ring_buf->BuffersAreBusy = 0;
		return StringLengthExceedsBufferSize;
	}
	
	// Checking for enough free space in ring data buffer
	if(Get_free_space_in_ring_data_buffer(com_ring_buf) < size)
	{
		com_ring_buf->BuffersAreBusy = 0;
		return NoFreeSpaceInBuffer;
	}
	
	// Write current pointer of data in data buffer into buffer of pointers
	com_ring_buf->PBuf[com_ring_buf->PBufPushPtr] = com_ring_buf->DBufPushPtr;	
	// Write current data string size to buffer of data string sizes
	com_ring_buf->SBuf[com_ring_buf->PBufPushPtr] = size;

	com_ring_buf->PBufPushPtr = (com_ring_buf->PBufPushPtr + 1) & com_ring_buf->PBufMask;

	Write_to_ring_data_buffer(com_ring_buf, buf, size);

	com_ring_buf->BuffersAreBusy = 0;
	return OK;
}


// Popping data to buf from ring data buffer and setting size of data string
ReturnCode_t Pop_string_from_ring_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t * size)
{
	uint8_t addr;
	uint8_t string_size;
	
	if(com_ring_buf->BuffersAreBusy) return BuffersAreBusy;
	com_ring_buf->BuffersAreBusy = 1;

	addr = com_ring_buf->PBuf[com_ring_buf->PBufPopPtr];
	string_size = com_ring_buf->SBuf[com_ring_buf->PBufPopPtr];

	com_ring_buf->PBufPopPtr = (com_ring_buf->PBufPopPtr + 1) & com_ring_buf->PBufMask;
	
	// Data reading from ring data buffer byte by byte with address wrapping
	Read_from_ring_data_buffer(com_ring_buf, buf, addr, string_size);
	
	*size = string_size;

	com_ring_buf->BuffersAreBusy = 0;
	return OK;
}


// Checking whether all data are popped from ring buffer
ReturnCode_t Is_all_data_popped_from_ring_buffer(RingBuffer_t * com_ring_buf)
{
	if(com_ring_buf->PBufPopPtr == com_ring_buf->PBufPushPtr) return OK;
	
	return ERR;
}


// Calculating size of free space in ring data buffer in bytes
static uint32_t Get_free_space_in_ring_data_buffer(RingBuffer_t * com_ring_buf)
{
	uint32_t ptr_in, ptr_out;
	uint32_t size;

	// If data push and pop pointers are equal
	if(com_ring_buf->PBufPopPtr == com_ring_buf->PBufPushPtr)
	{
		// Whole buffer is empty
		size = sizeof(com_ring_buf->DBuf) / com_ring_buf->DBufElementSize;
		return size;
	}

	// Сurrent point of popping data in the data buffer
	ptr_out = com_ring_buf->PBuf[com_ring_buf->PBufPopPtr];

	// Previous point of pushing data in the data buffer + string size
	ptr_in = com_ring_buf->PBuf[com_ring_buf->PBufPushPtr - 1];
	ptr_in += com_ring_buf->SBuf[com_ring_buf->PBufPushPtr - 1];
	ptr_in &= com_ring_buf->DBufMask;

	if(ptr_in > ptr_out)
	{
		// Size of data occupied in the buffer
		size = (ptr_in - ptr_out);
		// Size of free space in the buffer
		size = (sizeof(com_ring_buf->DBuf) / com_ring_buf->DBufElementSize) - size;		
	}
	else if(ptr_out > ptr_in)
	{
		size = (ptr_out - ptr_in);
	}

	return size;
}


// Copying data to ring data buffer byte by byte with address wrapping
static void Write_to_ring_data_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t size)
{
	uint32_t ptr = 0;
	
	while(size)
	{
		// Copy data to buffer of data with address wrapping
		com_ring_buf->DBuf[com_ring_buf->DBufPushPtr] = buf[ptr++];
		com_ring_buf->DBufPushPtr = (com_ring_buf->DBufPushPtr + 1) & com_ring_buf->DBufMask;
		size--;
	}
}


// Reading data from ring data buffer byte by byte with address wrapping
static void Read_from_ring_data_buffer(RingBuffer_t * com_ring_buf, uint8_t * buf, uint32_t addr, uint32_t size)
{
	uint32_t ptr = 0;
	
	while(size)
	{
		// Copy data to buffer of data with address wrapping
		buf[ptr++] = com_ring_buf->DBuf[(addr++) & com_ring_buf->DBufMask];
		size--;
	}
}

