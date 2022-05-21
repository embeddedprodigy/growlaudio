#include "RingBuffer.h"

void RingBuffer_Init(RingBufferType *rb){
	rb->Head = 0U;
	rb->Tail = 0U;
}
/* returns the length of available locations */
uint8_t RingBuffer_Available(RingBufferType *rb){
	uint8_t available = 0;
	if(rb->Head >= rb->Tail){
		available = BUFF_SIZE - (rb->Head - rb->Tail);
	}else{
		available = rb->Tail - rb->Head ; 
	}
	return available;
}

/* returns the length of occupied locations */
uint8_t RingBuffer_Occupied(RingBufferType *rb){
	uint8_t occupied = 0;
	if(rb->Head >= rb->Tail){
		occupied = rb->Head - rb->Tail;
	}else{
		occupied = BUFF_SIZE - (rb->Tail - rb->Head);
	}
	return occupied;
}

/* read 1 byte from the Ring Buffer */
uint16_t RingBuffer_ReadByte(RingBufferType *rb){
	uint16_t byteRead = 0;
	uint8_t tail;
	tail = rb->Tail;
	
	/* use while instead, if reading is faster than writing :) */
/*	
	while( RingBuffer_Occupied(rb) == 0 ){
	
	}
*/	
	if(RingBuffer_Occupied(rb) == 0){
		return byteRead;
	}

	
	byteRead = (0xFF00 | rb->RB_data[tail++]); 
	tail = tail % BUFF_SIZE;
	rb->Tail = tail;
	return byteRead;
}

/* write 1 byte to the Ring Buffer */
  uint8_t RingBuffer_WriteByte(RingBufferType *rb, uint8_t data){
	uint8_t head;
	head = rb->Head;
	
	//while(RingBuffer_Available(rb) == 0);
	if(RingBuffer_Available(rb) < 2 ){ (void)data; return 0;} // discard data if no space
	rb->RB_data[head++] = (uint8_t)data; head = head % BUFF_SIZE;
	rb->Head = head;

	return 1;
}


uint8_t RingBuffer_Read(RingBufferType *rb, uint8_t *destination_buffer, uint8_t size);
uint8_t RingBuffer_Write(RingBufferType *rb, uint8_t *source_buffer, uint8_t size);
