#ifndef _STM32F030K6T6_SPI_
#define _STM32F030K6T6_SPI_

#include "stm32f030k6t6_specifics.h"

#define SPI_CR1_BIDIMODE					(1U << 15)
#define SPI_CR1_BIDIOE						(1U << 14)
#define SPI_CR1_CRCEN							(1U << 13)
#define SPI_CR1_CRCNEXT						(1U << 12)
#define SPI_CR1_CRCL							(1U << 11)
#define SPI_CR1_RXONLY						(1U << 10)
#define SPI_CR1_SSM								(1U << 9)
#define SPI_CR1_SSI								(1U << 8)
#define SPI_CR1_LSBFIRST					(1U << 7)
#define SPI_CR1_SPE								(1U << 6)
#define SPI_CR1_BR								(7U << 3)
#define SPI_CR1_MSTR							(1U << 2)
#define SPI_CR1_CPOL							(1U << 1)
#define SPI_CR1_CPHA							(1U << 0)

#define SPI_CR2_LDMATX						(1U << 14)
#define SPI_CR2_LDMARX						(1U << 13)
#define SPI_CR2_FRXTH							(1U << 12)
#define SPI_CR2_DS								(0x0FU << 8)
#define SPI_CR2_TXEIE							(1U << 7)
#define SPI_CR2_RXNEIE						(1U << 6)
#define SPI_CR2_ERRIE							(7U << 5)
#define SPI_CR2_FRF								(1U << 4)
#define SPI_CR2_NSSP							(1U << 3)
#define SPI_CR2_SSOE							(1U << 2)
#define SPI_CR2_TXDMAEN						(1U << 1)
#define SPI_CR2_RXDMAEN						(1U << 0)

#define SPI_SR_FTLVL							(3UL << 11)
#define SPI_SR_FRLVL							(3UL << 9)
#define SPI_SR_FRE								(1UL << 8)
#define SPI_SR_BSY								(1UL << 7)
#define SPI_SR_OVR								(1UL << 6)
#define SPI_SR_MODF								(1UL << 5)
#define SPI_SR_CRCERR							(1UL << 4)
#define SPI_SR_TXE								(1UL << 1)
#define SPI_SR_RXNE								(1UL << 0)


#define SPI_STATUS_SR(_handle_ , flag ) ((_handle_)->Instance->SR & flag )
#define SPI_STATUS_CR(_handle_ , _bit_) ((_handle_)->Instance->CR & _bit_)

typedef enum{
	SPI_READY = 0, SPI_BUSY
}SPI_State;

typedef struct{
	uint32_t Mode;
	uint32_t Direction;
	uint32_t Datasize;
	uint32_t ClkPolarity;
	uint32_t ClkPhase;
	uint32_t NSS;
	uint32_t NSSP;
	uint32_t BRPrescaler;
	uint32_t FirstBit;
	uint32_t TIMode;
	uint32_t CRC;	
	uint32_t CRCLength;
	uint32_t CRCPolynomial;
	uint32_t Frxth;
	
}Init_SPI;

typedef struct{
	Regdef_SPI 					*Instance;
	Init_SPI 						Init;
	uint8_t							*pTxBuff;
	volatile uint16_t		txSize;
	uint8_t							*pRxBuff;
	volatile uint16_t		rxSize;
	SPI_State txState;
	SPI_State rxState;
}Handle_SPI;

/***********************CR1 control***********************
**********************************************************/
/**	SPI MODE - master slave
	*
	*/
#define SPI_MODE_MASTER				(SPI_CR1_MSTR | SPI_CR1_SSI)
#define SPI_MODE_SLAVE				(0x00UL)

/**	SPI DIRECTION - full half and simplex
	*
	*/
#define SPI_DIRECTION_FULLDUPLEX		(0x00UL)
#define SPI_DIRECTION_HALFDUPLEX		(SPI_CR1_BIDIMODE)	
#define SPI_DIRECTION_RXONLY				(SPI_CR1_RXONLY)

/**	SPI Clock Polarity
	*
	*/
#define SPI_CPOL_LOW				(0x00UL)
#define SPI_CPOL_HIGH				(SPI_CR1_CPOL)

/**	SPI Clock Phase
	*
	*/
#define SPI_CPHA_LEADING		(0x00UL)
#define SPI_CPHA_TRAILING		(SPI_CR1_CPHA)

/**	SPI NSS 
	*
	*/
#define SPI_NSS_SOFTWARE				(SPI_CR1_SSM)
#define SPI_NSS_HARDWARE_INPUT	(0x00UL)
#define SPI_NSS_HARDWARE_OUTPUT	(SPI_CR2_SSOE << 16U)
	
/**	SPI LSB first
	*
	*/
#define SPI_LSBFIRST_MSB				(0x00UL)
#define SPI_LSBFIRST_LSB				(SPI_CR1_LSBFIRST)


/**	SPI CRC calculation enable disable
	*
	*/
#define SPI_CRC_ENABLE		(SPI_CR1_CRCEN)
#define SPI_CRC_DISABLE		(0x00UL)

/** SPI CRC Length 0-8bit 1-16bit
	*
	*/
#define SPI_CRCLENGTH_BIT8		(0x00UL)
#define SPI_CRCLENGTH_BIT16		(SPI_CR1_CRCL)
	
/**	SPI Baud Rate Prescaler
	*
	*/	
#define SPI_BRPRESCALER_PCLKDIV2		(0x00UL << 3)
#define SPI_BRPRESCALER_PCLKDIV4		(0x01UL << 3)
#define SPI_BRPRESCALER_PCLKDIV8		(0x02UL << 3)
#define SPI_BRPRESCALER_PCLKDIV16		(0x03UL << 3)
#define SPI_BRPRESCALER_PCLKDIV32		(0x04UL << 3)
#define SPI_BRPRESCALER_PCLKDIV64		(0x05UL << 3)
#define SPI_BRPRESCALER_PCLKDIV128	(0x06UL << 3)
#define SPI_BRPRESCALER_PCLKDIV256	(0x07UL << 3)
	


/***********************CR2 control***********************
**********************************************************/

/**	SPI DATASIZE 4bit to 16bit
	*
	*/
#define SPI_DATASIZE_BIT4				(0x03UL << 8)
#define SPI_DATASIZE_BIT5				(0x04UL << 8)
#define SPI_DATASIZE_BIT6				(0x05UL << 8)
#define SPI_DATASIZE_BIT7				(0x06UL << 8)
#define SPI_DATASIZE_BIT8				(0x07UL << 8)
#define SPI_DATASIZE_BIT9				(0x08UL << 8)
#define SPI_DATASIZE_BIT10			(0x09UL << 8)
#define SPI_DATASIZE_BIT11			(0x0AUL << 8)
#define SPI_DATASIZE_BIT12			(0x0BUL << 8)
#define SPI_DATASIZE_BIT13			(0x0CUL << 8)
#define SPI_DATASIZE_BIT14			(0x0DUL << 8)
#define SPI_DATASIZE_BIT15			(0x0EUL << 8)
#define SPI_DATASIZE_BIT16			(0x0FUL << 8)

/** SPI NSS PULSE
	*
	*/	
#define SPI_NSSP_ENABLE			(SPI_CR2_NSSP)
#define SPI_NSSP_DISABLE		(0x00UL)

/** SPI TI Mode
	*
	*/
#define SPI_TIMODE_ENABLE			(SPI_CR2_FRF)
#define SPI_TIMODE_DISABLE		(0x00UL)	

	
/**	SPI 
	*
	*/
void SPI_Init(Handle_SPI *hspi);
void SPI_Send(Handle_SPI *hspi, uint8_t *pData, uint16_t size);
void SPI_Receive(void);


#endif //_STM32F030K6T6_SPI_
