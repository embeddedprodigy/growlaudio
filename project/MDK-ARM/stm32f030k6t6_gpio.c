#include "stm32f030k6t6_gpio.h"

/**************************************************************
 *	@fn						- GPIO_ClockControl
 *	@brief				- this function enables disable gpio periph clock
 *	@param				-	GPIO address
 *	@param				- enable disable
 *  @return				-
 *	@note					-

 */
 
#define MASK_BIT10		(0x03U)
#define MASK_BIT0			(0x01U)

#define MASK_EXTI (0x10000000U)

#define MASK_IT		(0x01000000U)
#define MASK_EVT	(0x02000000U)

#define MASK_RT		(0x00100000U)
#define MASK_FT		(0x00200000U)




#define MASK_MODE			(0x01U)
#define MASK_IT_				(1U)
#define MASK_PULL			(0x10U)
#define MASK_SPEED		(1U)

void GPIO_ClockControl(RegDef_GPIO_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx ==GPIOA){
			RCC_CLKEN_GPIOA;
		}
		else if(pGPIOx == GPIOB){
			RCC_CLKEN_GPIOB;
		}
	}
	else{
		if(pGPIOx ==GPIOA){
			RCC_CLKDI_GPIOA;
		}
		else if(pGPIOx == GPIOB){
			RCC_CLKDI_GPIOB;
		}	
	}
}

/*
 *	Initialization 
 */
void GPIO_Init(RegDef_GPIO_t *pGPIOx, Init_GPIO *pGPIOconfig){
		
		uint32_t ioPinActive = 0x0U;
		uint32_t ioPinNum	= 0x0U;
		uint32_t temp = 0x0U;
		while(pGPIOconfig->Pin >> ioPinNum){
			ioPinActive = (( pGPIOconfig->Pin >> ioPinNum ) & 0x01U);
			if(ioPinActive){
				if(pGPIOx == GPIOA){GPIO_ClockControl(GPIOA, ENABLE);}
				if(pGPIOx == GPIOB){GPIO_ClockControl(GPIOB, ENABLE);}


				
				/*				set mode - MODER				*/
				temp = pGPIOx->MODER;
				temp &= ~(MASK_BIT10 << ( ioPinNum * 2 ) ) ;
				temp |= (pGPIOconfig->Mode & MASK_BIT10 ) << ( ioPinNum * 2);
				pGPIOx->MODER |= temp;
				
					/*			return if analog 		*/
					if( pGPIOconfig->Mode == GPIO_MODE_ANALOG ){ return;}
				if(pGPIOconfig->Mode == GPIO_MODE_AF){
					/*		set the Alternate functions		*/
					temp = pGPIOx->AFR[ioPinNum >> 3U];
					temp &= ~(0x0FU << ((ioPinNum & 0x07U)*4U));
					temp |= (pGPIOconfig->Alternate << ((ioPinNum & 0x07U)*4) );
					pGPIOx->AFR[ioPinNum >> 3U] = temp;				
				}				
				if(pGPIOconfig->Mode == GPIO_MODE_OUT || pGPIOconfig->Mode == GPIO_MODE_AF){
					/*				set output type - OTYPER			*/	
					temp = pGPIOx->OTYPER;
					temp &= ~(MASK_BIT0 << ioPinNum);
					temp |= ((pGPIOconfig->Otype & MASK_BIT0) << ( ioPinNum ));
					pGPIOx->OTYPER = temp;
					
					/*				set output speed - OSPEEDR		*/
					temp = pGPIOx->OSPEEDR;
					temp &=	~(MASK_BIT10 << ( ioPinNum * 2 ));
					temp |= (pGPIOconfig->Speed & MASK_BIT10) << ( ioPinNum * 2);
					pGPIOx->OSPEEDR |= temp;
										
				}
				
				/*				set pull up and pull down			*/
				temp = pGPIOx->PUPDR;
				temp &=	~(MASK_BIT10 << ( ioPinNum * 2 ));
				temp |= (pGPIOconfig->Pull & MASK_BIT10) << ( ioPinNum * 2);
				pGPIOx->PUPDR = temp;
				
				/*				EXT settings */
				if( pGPIOconfig->Mode & MASK_EXTI ){
					
					RCC_CLKEN_SYSCFG;  // enable clock for SYSCFG register
					
					/* select gpiox source */
					temp = SYSCFG->EXTICR[ioPinNum>>2U];
					temp &= ~(0x0FU << ( (ioPinNum & 0x03U ) * 4 ) );
					temp |= ( GET_BIT_GPIO(pGPIOx) << ( (ioPinNum & 0x03U ) * 4 ));
					SYSCFG->EXTICR[ioPinNum>>2] = temp;
					
					/*	IMR setting	*/
					temp = EXTI->IMR;
					temp &= ~(1<<ioPinNum);
					if(pGPIOconfig->Mode & MASK_IT){
						temp |= (1<<ioPinNum);
					}
					EXTI->IMR = temp;
					
					/*	EMR setting	*/
					temp = EXTI->EMR;
					temp &= ~(1<<ioPinNum);
					if(pGPIOconfig->Mode & MASK_EVT){
						temp |= (1<<ioPinNum);
					}
					EXTI->EMR = temp;			
					
					/*	RTSR setting	*/
					temp = EXTI->RTSR;
					temp &= ~(1<<ioPinNum);
					if(pGPIOconfig->Mode & MASK_RT){
						temp |= (1<<ioPinNum);
					}
					EXTI->RTSR = temp;			
					
					/*	FTSR setting	*/
					temp = EXTI->FTSR;
					temp &= ~(1<<ioPinNum);
					if(pGPIOconfig->Mode & MASK_FT){
						temp |= (1<<ioPinNum);
					}
					EXTI->FTSR = temp;			
					
					
				}
			}

			ioPinNum++;
		}
		/*
		uint32_t ioMode = pGPIOconfig->Mode <<( pGPIOconfig->Pin * 2 );	
		uint32_t ioPin = pGPIOconfig->Pin;
		uint32_t ioSpeed = pGPIOconfig->Speed;
		GPIOA->ODR = ioMode;
		GPIOA->ODR = ioPin;
		GPIOA->ODR = ioSpeed;
		*/
}

void GPIO_DeInit(RegDef_GPIO_t *pGPIOx);

/*
 *	 Input and Output
 */
uint8_t GPIO_ReadPin(RegDef_GPIO_t *pGPIOx, uint8_t Pin){
	uint8_t val;
	val = (uint8_t)((pGPIOx->IDR >> Pin) & 0x01U);
	return val;
}
void GPIO_WritePin(RegDef_GPIO_t *pGPIOx, uint16_t Pin, uint8_t val){
	/*
	uint32_t res;
	res = 1 >> val;
	res = res * 16;
	pGPIOx->BSRR |= (1 << (Pin + res)); 
	*/
	pGPIOx->BSRR |= (1 << (Pin + ((1>> val) * 16 ))); 	
}
void GPIO_Toggle(RegDef_GPIO_t *pGPIOx, uint8_t Pin){
	pGPIOx->ODR ^= (1<<Pin);
}
uint16_t GPIO_ReadPort(RegDef_GPIO_t *pGPIOx);
void GPIO_WritePort(RegDef_GPIO_t *pGPIOx);


/*
 *	 IRQ
 */
void GPIO_IRQConfig(uint8_t IRQnum, uint8_t IRQPrio, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t pin);
