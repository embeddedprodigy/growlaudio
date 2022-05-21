#ifndef _STM32F030K6T6_GPIO_
#define _STM32F030K6T6_GPIO_
#include "stm32f030k6t6_specifics.h"


#define GET_BIT_GPIO(_gpiox_)		(uint8_t)(((_gpiox_) == (GPIOA)) ? 0U: \
																					((_gpiox_) == (GPIOB)) ? 1U: 2U)	
																			

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

/* @GPIO_PIN_NUM
 *
 */
#define GPIO_PIN_0		((uint32_t)(0x0001U))
#define GPIO_PIN_1		((uint32_t)(0x0002U))
#define GPIO_PIN_2		((uint32_t)(0x0004U))
#define GPIO_PIN_3		((uint32_t)(0x0008U))
#define GPIO_PIN_4		((uint32_t)(0x0010U))
#define GPIO_PIN_5		((uint32_t)(0x0020U))
#define GPIO_PIN_6		((uint32_t)(0x0040U))
#define GPIO_PIN_7		((uint32_t)(0x0080U))
#define GPIO_PIN_8		((uint32_t)(0x0100U))
#define GPIO_PIN_9		((uint32_t)(0x0200U))
#define GPIO_PIN_10		((uint32_t)(0x0400U))
#define GPIO_PIN_11		((uint32_t)(0x0800U))
#define GPIO_PIN_12		((uint32_t)(0x1000U))
#define GPIO_PIN_13		((uint32_t)(0x2000U))
#define GPIO_PIN_14		((uint32_t)(0x4000U))
#define GPIO_PIN_15		((uint32_t)(0x8000U))

 
/* @GPIO_PIN_MODE
 *	GPIO pin mode
 */
#define GPIO_MODE_IN							(0x00000000U)

#define GPIO_MODE_OUT							(0x00000001U)
//#define GPIO_MODE_OUT_PP				(0x00000001U)
//#define GPIO_MODE_OUT_OD				(0x00000011U)

#define GPIO_MODE_AF							(0x00000002U)
//#define GPIO_MODE_AF_PP					(0x00000002U)
//#define GPIO_MODE_AF_OD					(0x00000012U)

#define GPIO_MODE_ANALOG					(0x00000003U)	

#define GPIO_MODE_IN_IT_RT				(0x11100000UL)
#define GPIO_MODE_IN_IT_FT				(0x11200000UL)
#define GPIO_MODE_IN_IT_RTFT			(0x11300000UL)
#define GPIO_MODE_IN_EVT_RT				(0x12100000UL)
#define GPIO_MODE_IN_EVT_FT				(0x12200000UL)
#define GPIO_MODE_IN_EVT_RTFT			(0x12300000UL)


/*	@GPIO_OTYPER
 *	GPIO Output Type - open drain and push pull
 */
#define GPIO_OTYPE_PP							(0x00000000U)
#define GPIO_OTYPE_OD							(0x00000001U)

/*	@GPIO_PUPD
 *	GPIO Pull up Pull down
 */
#define GPIO_PULL_FL						(0x00000000U)
#define GPIO_PULL_PU						(0x00000001U)
#define GPIO_PULL_PD						(0x00000002U)


/*	@GPIO_SPEED
 *	GPIO speed
 */
#define GPIO_SPEED_LOW		(0x00000000U)
#define GPIO_SPEED_MEDIUM	(0x00000001U)
#define GPIO_SPEED_HIGH		(0x00000003U)

/*	@GPIO_AF[2]
 *	GPIO alternate functions
 */
 #define GPIO_ALTERNATE_TIM1	(0x00000001U)
 #define GPIO_ALTERNATE_TIM2	(0x00000002U)
 #define GPIO_ALTERNATE_TIM3	(0x00000003U)
 #define GPIO_ALTERNATE_TIM4	(0x00000004U)
 #define GPIO_ALTERNATE_SPI1	(0x00000000U)
 #define GPIO_ALTERNATE_UART1 (0x01UL)
 
typedef struct{
	uint32_t Pin;			/* @GPIO_PIN_NUM*/
	uint32_t Mode;
	uint32_t Otype;
	uint32_t Speed;
	uint32_t Pull;
	uint32_t Alternate;
}Init_GPIO;

typedef struct{
	RegDef_GPIO_t *Instance; 		  /*holds the address of GPIO*/
	Init_GPIO  Init;	/*holds the value of GPIO*/
}Handle_GPIO;

/*
 *	clock config
 */
void GPIO_ClockControl(RegDef_GPIO_t *pGPIOx, uint8_t EnorDi);

/*
 *	Initialization 
 */
void GPIO_Init(RegDef_GPIO_t *pGPIOx, Init_GPIO *pGPIOconfig);
//void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(RegDef_GPIO_t *pGPIOx);

/*
 *	 Input and Output
 */
uint8_t GPIO_ReadPin(RegDef_GPIO_t *pGPIOx, uint8_t Pin);
void GPIO_WritePin(RegDef_GPIO_t *pGPIOx, uint16_t Pin, uint8_t val);

void GPIO_Toggle(RegDef_GPIO_t *pGPIOx, uint8_t Pin);

uint16_t GPIO_ReadPort(RegDef_GPIO_t *pGPIOx);
void GPIO_WritePort(RegDef_GPIO_t *pGPIOx);



/*
 *	 IRQ
 */
void GPIO_IRQConfig(uint8_t IRQnum, uint8_t IRQPrio, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t pin);


#endif //_STM32F030K6T6_GPIO_

