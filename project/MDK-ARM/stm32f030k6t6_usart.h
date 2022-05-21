#ifndef _STM32F030K6T6_USART_H_
#define _STM32F030K6T6_USART_H_

#include "stm32f030k6t6_specifics.h"


#define USART_CR1_UE			( 1UL << 0U )
//#define res bit 1
#define USART_CR1_RE			( 1UL << 2U )
#define USART_CR1_TE			( 1UL << 3U )
#define USART_CR1_IDLEIE	( 1UL << 4U )
#define USART_CR1_RXNEIE	( 1UL << 5U )
#define USART_CR1_TCIE		( 1UL << 6U )
#define USART_CR1_TXEIE		( 1UL << 7U )
#define USART_CR1_PEIE		( 1UL << 8U )
#define USART_CR1_PS			( 1UL << 9U )
#define USART_CR1_PCE			( 1UL << 10U )
#define USART_CR1_WAKE		( 1UL << 11U )
#define USART_CR1_M0			( 1UL << 12U )
#define USART_CR1_MME			( 1UL << 13U )
#define USART_CR1_CMIE		( 1UL << 14U )
#define USART_CR1_OVER8		( 1UL << 15U )
#define USART_CR1_DEDT0		( 1UL << 16U )
#define USART_CR1_DEDT1		( 1UL << 17U )
#define USART_CR1_DEDT2		( 1UL << 18U )
#define USART_CR1_DEDT3		( 1UL << 19U )
#define USART_CR1_DEDT4		( 1UL << 20U )
#define USART_CR1_DEAT0		( 1UL << 21U )
#define USART_CR1_DEAT1		( 1UL << 22U )
#define USART_CR1_DEAT2		( 1UL << 23U )
#define USART_CR1_DEAT3		( 1UL << 24U )
#define USART_CR1_DEAT4		( 1UL << 25U )
#define USART_CR1_RTOIE		( 1UL << 26U )
//#define res bit 27
#define USART_CR1_M1			( 1UL << 28U )
//#define res bit 29 - 31


//#define reserve bit 0 - 3
#define USART_CR2_ADDM7		( 1UL << 4U )
//#define reserve bit 5 - 7
#define USART_CR2_LBCL			( 1UL << 8U )
#define USART_CR2_CPHA			( 1UL << 9U )
#define USART_CR2_CPOL			( 1UL << 10U )
#define USART_CR2_CLKEN			( 1UL << 11U )
#define USART_CR2_STOP0			( 1UL << 12U )
#define USART_CR2_STOP1			( 1UL << 13U )
//#define reserve bit 15
#define USART_CR2_SWAP			( 1UL << 15U )
#define USART_CR2_RXINV			( 1UL << 16U )
#define USART_CR2_TXINV			( 1UL << 17U )
#define USART_CR2_DATAINV		( 1UL << 18U )
#define USART_CR2_MSBFIRST	( 1UL << 19U )
#define USART_CR2_ABREN			( 1UL << 20U )
#define USART_CR2_ABRMOD0		( 1UL << 21U )
#define USART_CR2_ABRMOD1		( 1UL << 22U )
#define USART_CR2_RTOEN			( 1UL << 23U )
#define USART_CR2_ADD0			( 1UL << 24U )
#define USART_CR2_ADD1			( 1UL << 25U )
#define USART_CR2_ADD2			( 1UL << 26U )
#define USART_CR2_ADD3			( 1UL << 27U )
#define USART_CR2_ADD4			( 1UL << 28U )
#define USART_CR2_ADD5			( 1UL << 29U )
#define USART_CR2_ADD6			( 1UL << 30U )
#define USART_CR2_ADD7			( 1UL << 31U )


#define USART_CR3_EIE				( 1UL << 0U )
//#define reserve bit 1 - 2
#define USART_CR3_HDSEL			( 1UL << 3U )
//#define reserve bit 4 - 5
#define USART_CR3_DMAR			( 1UL << 6U )
#define USART_CR3_DMAT			( 1UL << 7U )
#define USART_CR3_RTSE			( 1UL << 8U )
#define USART_CR3_CTSE			( 1UL << 9U )
#define USART_CR3_CTSIE			( 1UL << 10U )
#define USART_CR3_ONEBIT		( 1UL << 11U )
#define USART_CR3_OVRDIS		( 1UL << 12U )
#define USART_CR3_DDRE			( 1UL << 13U )
#define USART_CR3_DEM				( 1UL << 14U )
#define USART_CR3_DEP				( 1UL << 15U )
//#define reserve bit 16 - 31

#define USART_RQR_ABRRQ			( 1UL << 0U )
#define USART_RQR_SBKRQ			( 1UL << 1U )
#define USART_RQR_MMRQ			( 1UL << 2U )
#define USART_RQR_RXFRQ			( 1UL << 3U )
//#define reserve bit 4 - 31

#define USART_ISR_PE				( 1UL << 0U )
#define USART_ISR_FE				( 1UL << 1U )
#define USART_ISR_NF				( 1UL << 2U )
#define USART_ISR_ORE				( 1UL << 3U )
#define USART_ISR_IDLE			( 1UL << 4U )
#define USART_ISR_RXNE			( 1UL << 5U )
#define USART_ISR_TC				( 1UL << 6U )
#define USART_ISR_TXE				( 1UL << 7U )
//#define reserve bit 8
#define USART_ISR_CTSIF			( 1UL << 9U )
#define USART_ISR_CTS				( 1UL << 10U )
#define USART_ISR_RTOF			( 1UL << 11U )
//#define reserve bit 12 - 13
#define USART_ISR_ABRE			( 1UL << 14U )
#define USART_ISR_ABRF			( 1UL << 15U )
// #define reserve bit 20 - 31

#define USART_ICR_PECF			(	1UL << 0U )
#define USART_ICR_FECF			(	1UL << 1U )
#define USART_ICR_NCF				( 1UL << 2U )
#define USART_ICR_ORECF			( 1UL << 3U )
#define USART_ICR_IDLECF		( 1UL << 4U )
//#define reserve bit 5
#define USART_ICR_TCCF			( 1UL << 6U )
//#define reserve bit 7 - 8
#define USART_ICR_CTSCF			( 1UL << 9U )
//#define reserve bit 10
#define USART_ICR_RTOCF			( 1UL << 11U )
//#define reserve bit 12 - 16
#define USART_ICR_CMCF			( 1UL << 17U )
//#define reserve bit 18 - 31

/**
	* USART MODE TX, RX, TXRX
	*/
#define USART_MODE_TX				( USART_CR1_TE )
#define USART_MODE_RX				( USART_CR1_RE )
#define USART_MODE_TXRX			( USART_CR1_TE | USART_CR1_RE )

/**
	* USART BAUD RATE
	*/
#define USART_BAUDRATE_1200				(1200)
#define USART_BAUDRATE_9600				(9600)
#define USART_BAUDRATE_19200			(19200)
#define USART_BAUDRATE_38400			(38400)
#define USART_BAUDRATE_57600			(57600)
#define USART_BAUDRATE_115200			(115200)
#define USART_BAUDRATE_230400			(230400)
#define USART_BAUDRATE_460800			(460800)
#define USART_BAUDRATE_921600			(921600)
#define USART_BAUDRATE_2M					(2000000)
#define USART_BAUDRATE_3M					(3000000)
#define USART_BAUDRATE_4M					(4000000)
#define USART_BAUDRATE_5M					(5000000)
#define USART_BAUDRATE_6M					(6000000)

/**
	* USART WORD LENGTH - 8bits, 9bits
	*/
#define USART_WORDLENGTH_7BITS		( USART_CR1_M1 )
#define USART_WORDLENGTH_8BITS		( 0x00UL )
#define USART_WORDLENGTH_9BTIS		( USART_CR1_M0 )

/**
	* USART STOP BITS - 0.5, 1, 1.5, 2
	*/
#define USART_STOP_1BIT				( 0x00UL )
#define USART_STOP_2BITS			( USART_CR2_STOP1 )

/**
	* USART PARITY - even, odd, none
	*/
#define USART_PARITY_NONE				( 0x00UL )
#define USART_PARITY_EVEN				( USART_CR1_PCE )
#define USART_PARITY_ODD				( USART_CR1_PCE | USART_CR1_PS ) 

/**
	* USART Hardware Flow Control RTS, CTS, RTS_CTS , none
	*/
#define USART_HWFLOW_NONE				( 0x00UL )
#define USART_HWFLOW_CTS				( USART_CR3_CTSE )
#define USART_HWFLOW_RTS				( USART_CR3_RTSE )
#define USART_HWFLOW_CTS_RTS		( USART_CR3_RTSE | USART_CR3_CTSE )

/**
	* USART OVERSAMPLING - by 8 , by 16
	*/
#define USART_OVERSAMPLING_BY8		( USART_CR1_OVER8 )
#define USART_OVERSAMPLING_BY16		( 0x00UL )

/**
	* USART ONE BIT SAMPLING - 
	*/ 
#define USART_ONEBIT_SAMPLE_ENABLE		( USART_CR3_ONEBIT )
#define USART_ONEBIT_SAMPLE_DISABLE		( 0x00UL )
	
typedef enum{ USART1_READY = 0, USART1_BUSY }USART_status;

typedef struct{
	uint32_t BaudRate;
	uint32_t WordLength;
	uint32_t StopBits;
	uint32_t Parity;
	uint32_t Mode;				// @USART_MODE
	uint32_t HWFlowControl;
	uint32_t OverSampling;
	uint32_t OneBitSampling;
}USART_InitTypedef;

typedef struct{
	Regdef_USART *Instance;
	USART_InitTypedef Init;
	uint8_t *pTxBuff;
	volatile uint16_t txSize;
	uint8_t *pRxBuff;
	volatile uint16_t rxSize;
	USART_status status;
}USART_Handle;

void USART_Init(USART_Handle *husart);
void USART_Send(USART_Handle *husart, uint8_t *pData, uint16_t size);
void USART_Receive(USART_Handle *husart, uint8_t *pData, uint16_t size);
#endif	//_STM32F030K6T6_USART_H_
