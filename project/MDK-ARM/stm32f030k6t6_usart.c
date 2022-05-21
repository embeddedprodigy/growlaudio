#include "stm32f030k6t6_usart.h"

void USART_Init(USART_Handle *husart){
		/* disable usart before any init recommended */
		husart->Instance->CR1 &= ~(USART_CR1_UE);
	if(husart->Instance == USART1){ RCC_CLKEN_USART1;}
	husart->Instance->CR1 = husart->Init.Mode | husart->Init.Parity | 
													husart->Init.WordLength | husart->Init.OverSampling;  
	husart->Instance->CR2 = husart->Init.StopBits ;
	husart->Instance->CR3 = husart->Init.HWFlowControl | husart->Init.OneBitSampling;
	husart->Instance->BRR = 0xD0UL;
	// for baud9600 0x0341UL;
	// for baud38400 0xD0UL
	
	//enable INTERRUPT
	husart->Instance->CR1 |= ( USART_CR1_RXNEIE );
	//husart->Instance->CR1 |= ( USART_CR1_TCIE ); // enables the TC
	
		husart->Instance->CR1 |= (USART_CR1_UE);
	
}
void USART_Send(USART_Handle *husart, uint8_t *pData, uint16_t size){

	husart->pTxBuff = (uint8_t *)pData;
	husart->txSize = size;
	
	husart->pRxBuff = (uint8_t *)0;
	husart->rxSize = 0U;
	
	while(husart->txSize){
		if(husart->Instance->ISR & USART_ISR_TXE){
			husart->Instance->TDR = (uint8_t)*husart->pTxBuff;
			husart->pTxBuff++;
			husart->txSize--;
		}
	}
	while((husart->Instance->ISR & USART_ISR_TC) == 0x00U);
}
void USART_Receive(USART_Handle *husart, uint8_t *pData,  uint16_t size){
	
	husart->pTxBuff = (uint8_t *)0;
	husart->txSize = 0U;
	
	husart->pRxBuff = (uint8_t *)pData;
	husart->rxSize = size;	
	
	while(husart->rxSize){
		if(husart->Instance->ISR & USART_ISR_RXNE){
			
			*husart->pRxBuff = (uint8_t)(husart->Instance->RDR & 0xFFU);
			husart->pRxBuff++;
			husart->rxSize--;
	
		}
	}	
		while((husart->Instance->ISR & USART_ISR_TC) == 0x00U);
}
