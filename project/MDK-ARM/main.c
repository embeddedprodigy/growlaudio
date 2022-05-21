//#include "stm32f0xx.h"                  // Device header
#include "main.h"
#include <string.h>

void husart_initialization(void);
void 	systickSetup(void);

void gpioReverseOut(void);
void gpioInputITsetup(void);
void gpioUARTsetup(void);

void cntDown(uint32_t cnt){
	while(cnt--);
}

Init_GPIO gpioMotor;
	
Init_GPIO gpioSPI;
Init_GPIO gpioUART;
Init_GPIO gpioInput;

Init_GPIO	gpioToggle;
Handle_SPI hspi;

USART_Handle husart1;

void spi_initial(void);

/*aircon control global variables*/
RingBufferType RxBuffer;
uint16_t data_SOF = 0x00U;
uint8_t data_Received[20];

uint32_t tickGlobal;

#define TEMP_BUFF_SIZE 50
uint8_t tempBuffer[TEMP_BUFF_SIZE];
uint8_t tempIndex;
uint16_t tempVar;
uint8_t SOFstate;
uint8_t checkSum;

uint32_t motorVal[32];

/*reverse signal input*/
uint32_t reverseInputFlag;
uint32_t reverseInputCount;

/*reverse signal output*/
uint32_t reverseOutputFlag;
uint32_t reverseOutputCount;

/* button cam switcher*/
uint8_t btnCamSwitcherFlag, btnCamSwitcherFlagTEMP;
uint32_t btnCamSwitcherCount;
uint32_t btnCamSwitcherSpaceCount;
uint32_t btnCamSwitcherSpaceFlag;

int main(void){
	

	/*reverse signal*/
	reverseInputFlag = 0; btnCamSwitcherFlagTEMP = 0;
	reverseInputCount = 0;
	reverseOutputFlag = 0;
	reverseOutputCount = 0;
	/*btn cam switcher*/
	btnCamSwitcherFlag = 0;
	btnCamSwitcherCount = 0;
	btnCamSwitcherSpaceCount = 0;
	
	checkSum = 0;
	tempIndex = 1;
	tempVar = 0U;
	SOFstate = 0;
	memset(tempBuffer, 0, sizeof(tempBuffer)/sizeof(tempBuffer[0]));
	
	RingBuffer_Init( &RxBuffer );
//	RingBuffer_Init( &ReadLocation );
	
	/* SPI gpio */
	gpioSPI.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	gpioSPI.Mode = GPIO_MODE_AF;
	gpioSPI.Otype = GPIO_OTYPE_PP;
	gpioSPI.Speed = GPIO_SPEED_HIGH;
	gpioSPI.Pull = GPIO_PULL_PU;
	gpioSPI.Alternate = GPIO_ALTERNATE_SPI1;
	GPIO_Init(GPIOA, &gpioSPI);
	
	
	/* SPI setup */
	RCC_CLKEN_SPI1;
	hspi.Instance = SPI1;
	
	hspi.Init.Mode = SPI_MODE_MASTER;
	hspi.Init.Direction = SPI_DIRECTION_FULLDUPLEX;
	hspi.Init.BRPrescaler = SPI_BRPRESCALER_PCLKDIV256;

	hspi.Init.ClkPolarity = SPI_CPOL_HIGH;
	hspi.Init.ClkPhase = SPI_CPHA_LEADING;
	
	hspi.Init.Datasize = SPI_DATASIZE_BIT8;
	hspi.Init.NSS = SPI_NSS_HARDWARE_OUTPUT;
	
	hspi.Init.CRC = SPI_CRC_DISABLE;
	hspi.Init.CRCLength = SPI_CRCLENGTH_BIT8;
	hspi.Init.TIMode = SPI_TIMODE_DISABLE;
	
	hspi.Init.Frxth = (1UL << 12U);
	
	SPI_Init(&hspi);
	
	//		SPI1->CR1 |= SPI_CR1_SPE;
	//		while( (SPI1->SR & SPI_SR_TXE) == 0U );

	/*systick setup*/
	systickSetup();
	tickGlobal = SYSTICK_getTick();
	
	/* reverse output Pin setup*/
	gpioReverseOut();
	
	/* UART setup*/

	gpioUARTsetup();
	husart_initialization();

	gpioInputITsetup();

	NVIC_EnableIRQ(IRQn_USART1);
	NVIC_EnableIRQ(IRQn_EXTI0_1);
	__enable_irq();
	GPIOA->ODR &= ~(GPIO_PIN_11 | GPIO_PIN_12);

while(1){
			

			/*	thread - Camera 360
			 *************************************************
			 */			
			if(btnCamSwitcherFlag){
				if(SYSTICK_getTick() - btnCamSwitcherCount >= 200 ){
					
					GPIOA->ODR &= ~( GPIO_PIN_11 );
					btnCamSwitcherFlag--;
					btnCamSwitcherCount = SYSTICK_getTick();
					
					btnCamSwitcherSpaceCount = SYSTICK_getTick();
					btnCamSwitcherSpaceFlag = 1;
					/*
					if(btnCamSwitcherFlag > 0){
						GPIOA->ODR |= ( GPIO_PIN_11 );
						btnCamSwitcherCount = SYSTICK_getTick();				
					}
					*/
				}			
			}
			if(btnCamSwitcherSpaceFlag){
				if(SYSTICK_getTick() - btnCamSwitcherSpaceCount >= 200 ){
					btnCamSwitcherSpaceFlag = 0;
					
					if(btnCamSwitcherFlag){
						GPIOA->ODR |= ( GPIO_PIN_11 );	
						btnCamSwitcherCount = SYSTICK_getTick();	
					}
				}
			}
			
			/*	thread - UART buffer 
			 *************************************************
			 */
		  /*read buffer and assemble the frame to tempBuffer*/
			tempVar = RingBuffer_ReadByte(&RxBuffer);
			
			if(tempVar != 0 ){ /* if not empty to ignore if read null */
				tempBuffer[tempIndex++] = (uint8_t)tempVar;
				//tempIndex++;
			}
											/*if sof detected*/
			if(SOFstate == 0 && tempBuffer[tempIndex-1] == 0x55U && tempBuffer[tempIndex-2] == 0xAAU) {
				tempBuffer[1U] = tempBuffer[tempIndex-2];
				tempBuffer[2U] = tempBuffer[tempIndex-1];
				memset((tempBuffer + 3U), 0, sizeof(tempBuffer)/sizeof(tempBuffer[0]));
				tempIndex = 3U;
				SOFstate = 1;
			}

			if(SOFstate == 1 && (tempIndex-1) == (tempBuffer[3] + 5) ){
				if(tempBuffer[3] == 0x02U){
					/* COM ID for AC*****************************/
					if(tempBuffer[4] == 0x3DU){ 
						if(tempBuffer[5] == 0x0BU && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x4AU){
							 //fan up
						}
						else if(tempBuffer[5] == 0x0CU && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x4BU){
							 //fan down
						}
						else if(tempBuffer[5] == 0x0DU && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x4CU){
							 //rotateCW(10);//triggerFanUp(); // temp left up
						}
						else if(tempBuffer[5] == 0x0EU && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x4DU){
							// rotateCCW(10);//triggerFanDown(); //temp left down
						}							
						else if(tempBuffer[5] == 0x0FU && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x4EU){
							 //temp right up
						}
						else if(tempBuffer[5] == 0x10U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x4FU){
							 // temp righ down
						}	
						else if(tempBuffer[5] == 0x02U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x41U){
							 // ac							
						}
						else if(tempBuffer[5] == 0x07U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x46U){
							 // in air or out air
						}						
						else if(tempBuffer[5] == 0x05U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x44U){
							 // max
						}
						else if(tempBuffer[5] == 0x06U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x45U){
							 // defogger
						}		
						else if(tempBuffer[5] == 0x15U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x54U){
							 // foot face side
						}						
						else if(tempBuffer[5] == 0x01U && tempBuffer[6] == 0x01U && tempBuffer[7] == 0x40U){
							 // on off
						}								
					}
					/*end AC*/
					
					/* COM ID for Camera 360 -- 0xAA 0x55 0x02 0xFD 0x01 0x00 0xFF*/
					else if(tempBuffer[4] == 0xFDU){
						if(tempBuffer[5] == 0x01U && tempBuffer[6] == 0x00 && tempBuffer[7] == 0xFFU){// || ){

							if(btnCamSwitcherFlag == 0 ){
								GPIOA->ODR |= (GPIO_PIN_11);
								btnCamSwitcherCount = SYSTICK_getTick();							
							}	
							
							btnCamSwitcherFlag++;
						}
					}
					/* end Camera 360*/
					
					else if(tempBuffer[4] == 0x0A){ /* COM ID for ?**************************/

					}
					else{
					
					}				
				}
				else if(tempBuffer[3] == 0x0D){ /* this length is ?*/
					
				}

				else{}
					SOFstate = 0;
					tempIndex = 1;				
			}				
	}			
}

//int flagTC = 0;
void USART1_IRQHandler(void){
	if(USART1->ISR & USART_ISR_ORE){ USART1->ICR |= USART_ICR_ORECF; }
	RingBuffer_WriteByte( &RxBuffer, (uint8_t)USART1->RDR );

}
void EXTI0_1_IRQHandler(){
	if(EXTI->PR & GPIO_PIN_0){
		if( !(GPIOB->IDR & 0x01U) ){ //falling
			GPIOA->ODR |= (GPIO_PIN_12);
		}
		else{ //rising
			GPIOA->ODR &= ~(GPIO_PIN_12);
		}
		EXTI->PR &= GPIO_PIN_0;
		reverseInputFlag = 1;	//GPIOA->ODR ^= (GPIO_PIN_12);		
	}
	if(EXTI->PR & GPIO_PIN_1){ 
		EXTI->PR &= GPIO_PIN_1;
			//GPIOA->ODR ^= (GPIO_PIN_12);		
	}
}

void systickSetup(){
	uint32_t temp1 = 0x1F3FU;
	uint32_t temp2 = SYSTICK_CSR_CLKSOURCE 
								| SYSTICK_CSR_ENABLE
								| SYSTICK_CSR_TICKINT;
	
	SYSTICK_Init( temp1,temp2 );
}


void husart_initialization(void){
	husart1.Instance = USART1;
	husart1.Init.BaudRate = USART_BAUDRATE_38400;
	husart1.Init.HWFlowControl = USART_HWFLOW_NONE;
	husart1.Init.Mode = USART_MODE_TXRX;
	husart1.Init.OneBitSampling = USART_ONEBIT_SAMPLE_DISABLE;
	husart1.Init.OverSampling = USART_OVERSAMPLING_BY16;
	husart1.Init.Parity = USART_PARITY_NONE;
	husart1.Init.StopBits = USART_STOP_1BIT;
	husart1.Init.WordLength = USART_WORDLENGTH_8BITS;
	USART_Init( &husart1 );
}


void gpioReverseOut(void){
	gpioMotor.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	gpioMotor.Mode = GPIO_MODE_OUT;
	gpioMotor.Otype = GPIO_OTYPE_PP;
	gpioMotor.Pull = GPIO_PULL_FL;
	gpioMotor.Speed = GPIO_SPEED_LOW;
	GPIO_Init( GPIOA, &gpioMotor );
}

void gpioInputITsetup(void){
	gpioInput.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//	gpioInput.Mode = GPIO_MODE_IN;
	gpioInput.Mode = GPIO_MODE_IN_IT_RTFT;
	gpioInput.Pull = GPIO_PULL_PU;
	GPIO_Init(GPIOB,&gpioInput);
}

void gpioUARTsetup(void){
	gpioUART.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	gpioUART.Mode = GPIO_MODE_AF;
	gpioUART.Otype = GPIO_OTYPE_PP;
	gpioUART.Speed = GPIO_SPEED_HIGH;
	gpioUART.Pull = GPIO_PULL_PU;
	gpioUART.Alternate = GPIO_ALTERNATE_UART1;
	GPIO_Init(GPIOA, &gpioUART);
}
	

