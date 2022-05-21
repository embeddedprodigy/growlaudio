#ifndef		_RINGBUFFER_H_
#define		_RINGBUFFER_H_
#include <stdint.h>

#define BUFF_SIZE 100
typedef struct{
	volatile uint8_t	Head;
	volatile uint8_t 	Tail;
	uint8_t RB_data[BUFF_SIZE];
}RingBufferType;

void RingBuffer_Init(RingBufferType *rb);
uint8_t RingBuffer_Available(RingBufferType *rb);
uint8_t RingBuffer_Occupied(RingBufferType *rb);
uint16_t RingBuffer_ReadByte(RingBufferType *rb);
uint8_t RingBuffer_WriteByte(RingBufferType *rb,  uint8_t data);
uint8_t RingBuffer_Read(RingBufferType *rb, uint8_t *destination_buffer, uint8_t size);
uint8_t RingBuffer_Write(RingBufferType *rb, uint8_t *source_buffer, uint8_t size);

#endif	//_RINGBUFFER_H_

