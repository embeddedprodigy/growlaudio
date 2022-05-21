#include "stm32f030k6t6_spi.h"

void SPI_Init(Handle_SPI *hspi){
	if(hspi->Instance == SPI1 ){ RCC_CLKEN_SPI1;}
	hspi->Instance->CR1 = (	hspi->Init.Mode 
												| hspi->Init.Direction 
												| hspi->Init.CRC
												| hspi->Init.CRCLength	
												|	(hspi->Init.NSS	& SPI_CR1_SSM)	
												| hspi->Init.ClkPolarity
												| hspi->Init.ClkPhase
												| hspi->Init.BRPrescaler
												|	hspi->Init.FirstBit	);
	
	hspi->Instance->CR2 = ( ((hspi->Init.NSS >> 16U) & SPI_CR2_SSOE)
												|	hspi->Init.TIMode
												| hspi->Init.NSSP		
												| hspi->Init.Datasize 
												| hspi->Init.Frxth );
}

void SPI_Send(Handle_SPI *hspi, uint8_t *pData, uint16_t size){
	hspi->pTxBuff = (uint8_t *)pData;
	hspi->txSize = size;
	
	hspi->pRxBuff = (uint8_t *)0;
	hspi->rxSize = 0;
	
	hspi->Instance->CR1 |= SPI_CR1_SPE;
	if(hspi->Init.Datasize > SPI_DATASIZE_BIT8)
	{
			hspi->Instance->DR = *((uint16_t *)hspi->pTxBuff);
			hspi->pTxBuff += sizeof(uint16_t);
			hspi->txSize--;
			while( hspi->txSize > 0){
				if(hspi->Instance->SR & SPI_SR_TXE){
					hspi->Instance->DR = *((uint16_t *)hspi->pTxBuff);
					hspi->pTxBuff += sizeof(uint16_t);
					hspi->txSize--;							
				}
				else{
					// put time out here
				}
			}
	}
	else{    
		*((volatile uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuff);
		hspi->pTxBuff++;
		hspi->txSize--;
		while( hspi->txSize > 0 ){
			if(hspi->Instance->SR & SPI_SR_TXE ){
					*((volatile uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuff);
					hspi->pTxBuff += sizeof(uint8_t);
					hspi->txSize--;
			}
		}
	}
	while(hspi->Instance->SR & SPI_SR_BSY);
	hspi->Instance->CR1 &= ~SPI_CR1_SPE;
}
void SPI_Receive(void){

}
