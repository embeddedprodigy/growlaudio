#ifndef _SMT32F030K6T6_REGDEF_
#define _SMT32F030K6T6_REGDEF_
#include <stdint.h>

/* CORTEX M0 headers*/
/* IRQn */
#define PRIORITY_IMPLEMENTED_BITS 2U

#define IRQn_EXTI0_1			5UL
#define IRQn_EXTI2_3			6UL
#define IRQn_EXTI4_15			7UL
#define IRQn_USART1				27U

#define BASE_ADDR_SCS				(0xE000E000UL)	
#define BASE_ADDR_SYSTICK		(BASE_ADDR_SCS + 0x0010UL)
#define BASE_ADDR_NVIC			(BASE_ADDR_SCS + 0x0100UL)
#define BASE_ADDR_SCB				(BASE_ADDR_SCS + 0x0D00UL)

typedef struct
{
  volatile uint32_t ISER[1U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register */
}  RegDef_NVIC_t;

#define NVIC	((RegDef_NVIC_t*)(BASE_ADDR_NVIC))

typedef struct
{
  volatile uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  volatile uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  volatile uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  volatile const  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} RegDef_SYSTICK_t;

#define SYSTICK 		((RegDef_SYSTICK_t*)(BASE_ADDR_SYSTICK))

typedef struct
{
  volatile const  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  volatile uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  volatile uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  volatile uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];                /*!< Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED */
  volatile uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
} RegDef_SCB_t;

#define SCB 		((RegDef_SCB_t*)(BASE_ADDR_SCB))
static inline void NVIC_EnableIRQ(uint32_t IRQn)
{
  NVIC->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}
static inline void NVIC_DisableIRQ(uint32_t IRQn){
	NVIC->ICER[0U] = (uint32_t)(1 << (IRQn & 0x1FUL));
}
static inline void NVIC_SetPendingIRQ(uint32_t IRQn){
	NVIC->ISPR[0U] = (uint32_t)( 1 << (IRQn & 0x1FUL));
}
static inline void NVIC_ClearPendinIRQ(uint32_t IRQn){
	NVIC->ICPR[0U] = (uint32_t)( 1 << (IRQn & 0x1FUL));
}
static inline void NVIC_SetPriorityIRQ(uint32_t IRQn, uint8_t priority){
	uint32_t temp = NVIC->IP[IRQn >> 2U];
	temp &= ~(0xFF << ((IRQn & 0x03UL) *8));
	temp |= ((uint32_t)( priority << (8U - PRIORITY_IMPLEMENTED_BITS) ) << ((IRQn & 0x03UL) *8));
	NVIC->IP[IRQn >> 2U] = temp;
	/*
	NVIC->IP[IRQn >> 2U]  = ((NVIC->IP[IRQn]  & ~(0xFFUL << _BIT_SHIFT(IRQn))) | \
       (((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << _BIT_SHIFT(IRQn)));
	NVIC->IP[IRQn] = (1 << (priority + (8- PRIORITY_IMPLEMENTED_BITS)))
*/
}

/*	 
 *	generic macro
 */
#define READ_BIT(word, bit)			(word & (1<<bit))
#define SET_BIT(word, bit)			(word |=(1<<bit))
#define CLEAR_BIT(word, bit)		(word &= ~(1<<bit))

#define ENABLE  1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

/* base address of memory*/
#define BASE_ADDR_FLASH				0x08000000UL
#define BASE_ADDR_SRAM				0x20000000UL
#define BASE_ADDR_PERIPH			0x40000000UL

#define BASE_ADDR_APB				  ( BASE_ADDR_PERIPH + 0x00000000UL )
//#define BASE_ADDR_APB2				( BASE_ADDR_PERIPH + 0x00001000UL )
#define BASE_ADDR_AHB1				( BASE_ADDR_PERIPH + 0x00020000UL )
#define BASE_ADDR_AHB2				( BASE_ADDR_PERIPH + 0x08000000UL )

/* APB Peripherals */
#define BASE_ADDR_TIM3				( BASE_ADDR_APB + 0x00000400UL )
#define BASE_ADDR_TIM6				( BASE_ADDR_APB + 0x00001000UL )
#define BASE_ADDR_TIM7				( BASE_ADDR_APB + 0x00001400UL )
#define BASE_ADDR_TIM14				( BASE_ADDR_APB + 0x00002000UL )
#define BASE_ADDR_RTC					( BASE_ADDR_APB + 0x00002800UL )
#define BASE_ADDR_WWDG				( BASE_ADDR_APB + 0x00002C00UL )
#define BASE_ADDR_IWDG   			( BASE_ADDR_APB + 0x00003000UL )
#define BASE_ADDR_SPI2				( BASE_ADDR_APB + 0x00003800UL )
#define BASE_ADDR_USART2			( BASE_ADDR_APB + 0x00004400UL )
#define BASE_ADDR_USART3			( BASE_ADDR_APB + 0x00004800UL )
#define BASE_ADDR_USART4			( BASE_ADDR_APB + 0x00004C00UL )
#define BASE_ADDR_USART5			( BASE_ADDR_APB + 0x00005000UL )
#define BASE_ADDR_I2C1				( BASE_ADDR_APB + 0x00005400UL )
#define BASE_ADDR_I2C2				( BASE_ADDR_APB + 0x00005800UL )
#define BASE_ADDR_USB					( BASE_ADDR_APB + 0x00005C00UL )
#define BASE_ADDR_USB_SRAM		( BASE_ADDR_APB + 0x00006000UL )
#define BASE_ADDR_PWR					( BASE_ADDR_APB + 0x00007000UL )
#define BASE_ADDR_SYSCFG			( BASE_ADDR_APB + 0x00010000UL )
#define BASE_ADDR_EXTI				( BASE_ADDR_APB + 0x00010400UL )
#define BASE_ADDR_USART6			( BASE_ADDR_APB + 0x00011400UL )
#define BASE_ADDR_ADC					( BASE_ADDR_APB + 0x00012400UL )
#define BASE_ADDR_TIM1				( BASE_ADDR_APB + 0x00012C00UL )
#define BASE_ADDR_SPI1				( BASE_ADDR_APB + 0x00013000UL )
#define BASE_ADDR_USART1			( BASE_ADDR_APB + 0x00013800UL )
#define BASE_ADDR_TIM15				( BASE_ADDR_APB + 0x00014000UL )
#define BASE_ADDR_TIM16				( BASE_ADDR_APB + 0x00014400UL )
#define BASE_ADDR_TIM17				( BASE_ADDR_APB + 0x00014800UL )
#define BASE_ADDR_DBGMCU			( BASE_ADDR_APB + 0x00015800UL )

/* AHB1 Peripherals */
#define BASE_ADDR_DMA					( BASE_ADDR_AHB1 + 0x00000000UL )
#define BASE_ADDR_RCC					( BASE_ADDR_AHB1 + 0x00001000UL )
#define BASE_ADDR_FLASHINT		( BASE_ADDR_AHB1 + 0x00002000UL )
#define BASE_ADDR_CRC					( BASE_ADDR_AHB1 + 0x00003000UL )

/* AHB2 Peripherals */
#define BASE_ADDR_GPIOA				( BASE_ADDR_AHB2 +    0x0UL )
#define BASE_ADDR_GPIOB				( BASE_ADDR_AHB2 + 0x0400UL )
#define BASE_ADDR_GPIOC				( BASE_ADDR_AHB2 + 0x0800UL )
#define BASE_ADDR_GPIOD				( BASE_ADDR_AHB2 + 0x0C00UL )
#define BASE_ADDR_GPIOF				( BASE_ADDR_AHB2 + 0x1400UL )

/* GPIO register definition and RCC */
typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
	volatile uint32_t BRR;
}RegDef_GPIO_t;

#define GPIOA 			((RegDef_GPIO_t*)(BASE_ADDR_GPIOA))
#define GPIOB 			((RegDef_GPIO_t*)(BASE_ADDR_GPIOB))
#define GPIOC 			((RegDef_GPIO_t*)(BASE_ADDR_GPIOC))
#define GPIOD 			((RegDef_GPIO_t*)(BASE_ADDR_GPIOD))
#define GPIOF 			((RegDef_GPIO_t*)(BASE_ADDR_GPIOF))

/* SYSCFG register configuration */
typedef struct{
	volatile uint32_t CFGR1;
	uint32_t RESERVE1;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CFGR2;
}RegDef_SYSCFG_t;

#define SYSCFG 			((RegDef_SYSCFG_t*)(BASE_ADDR_SYSCFG))

/*	EXTI register definition */
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}RegDef_EXTI_t;

#define EXTI				((RegDef_EXTI_t*)(BASE_ADDR_EXTI))

/* RCC register definition*/

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
	volatile uint32_t CR2;
}RegDef_RCC_t;

#define RCC					((RegDef_RCC_t*)(BASE_ADDR_RCC))

#define RCC_CLKEN_GPIOA			(RCC->AHBENR |= (1<<17))			
#define RCC_CLKEN_GPIOB			(RCC->AHBENR |= (1<<18))
#define RCC_CLKEN_GPIOC			(RCC->AHBENR |= (1<<19))
#define RCC_CLKEN_GPIOD			(RCC->AHBENR |= (1<<20))
#define RCC_CLKEN_GPIOF			(RCC->AHBENR |= (1<<22))

#define RCC_CLKDI_GPIOA			(RCC->AHBENR &= ~(1<<17))			
#define RCC_CLKDI_GPIOB			(RCC->AHBENR &= ~(1<<18))
#define RCC_CLKDI_GPIOC			(RCC->AHBENR &= ~(1<<19))
#define RCC_CLKDI_GPIOD			(RCC->AHBENR &= ~(1<<20))
#define RCC_CLKDI_GPIOF			(RCC->AHBENR &= ~(1<<22))

/* RCC IO clock enable */
#define RCC_CLKEN_DMAEN			(RCC->AHBENR |= (1<<0))
#define RCC_CLKEN_SRAMEN		(RCC->AHBENR |= (1<<2))
#define RCC_CLKEN_FLITF			(RCC->AHBENR |= (1<<4))
#define RCC_CLKEN_CRC				(RCC->AHBENR |= (1<<6))

#define RCC_CLKEN_SYSCFG		( RCC->APB2ENR |= (1<<0))
#define RCC_CLKEN_USART6		( RCC->APB2ENR |= (1<<5))
#define RCC_CLKEN_ADC				( RCC->APB2ENR |= (1<<9))
#define RCC_CLKEN_TIM1			( RCC->APB2ENR |= (1<<11))
#define RCC_CLKEN_SPI1			( RCC->APB2ENR |= (1<<12))
#define RCC_CLKEN_USART1		( RCC->APB2ENR |= (1<<14))
#define RCC_CLKEN_TIM15			( RCC->APB2ENR |= (1<<16))
#define RCC_CLKEN_TIM16			( RCC->APB2ENR |= (1<<17))
#define RCC_CLKEN_TIM17			( RCC->APB2ENR |= (1<<18))
#define RCC_CLKEN_DBGMCU		( RCC->APB2ENR |= (1<<22))

#define RCC_CLKEN_TIM3			( RCC->APB1ENR |= (1<<1))
#define RCC_CLKEN_TIM6			( RCC->APB1ENR |= (1<<4))
#define RCC_CLKEN_TIM7			( RCC->APB1ENR |= (1<<5))
#define RCC_CLKEN_TIM14			( RCC->APB1ENR |= (1<<8))
#define RCC_CLKEN_WWDG			( RCC->APB1ENR |= (1<<11))
#define RCC_CLKEN_SPI2			( RCC->APB1ENR |= (1<<14))
#define RCC_CLKEN_USART2		( RCC->APB1ENR |= (1<<17))
#define RCC_CLKEN_USART3		( RCC->APB1ENR |= (1<<18))
#define RCC_CLKEN_USART4		( RCC->APB1ENR |= (1<<19))
#define RCC_CLKEN_USART5		( RCC->APB1ENR |= (1<<20))
#define RCC_CLKEN_I2C1			( RCC->APB1ENR |= (1<<21))
#define RCC_CLKEN_I2C2			( RCC->APB1ENR |= (1<<22))
#define RCC_CLKEN_USB				( RCC->APB1ENR |= (1<<23))
#define RCC_CLKEN_PWR				( RCC->APB1ENR |= (1<<28))

/* RCC IO clock disable */
#define RCC_CLKDI_DMAEN			(RCC->AHBENR &= ~(1<<0))
#define RCC_CLKDI_SRAMEN		(RCC->AHBENR &= ~(1<<2))
#define RCC_CLKDI_FLITF			(RCC->AHBENR &= ~(1<<4))
#define RCC_CLKDI_CRC				(RCC->AHBENR &= ~(1<<6))

#define RCC_CLKDI_SYSCFG		( RCC->APB2ENR &= ~(1<<0))
#define RCC_CLKDI_USART6		( RCC->APB2ENR &= ~(1<<5))
#define RCC_CLKDI_ADC				( RCC->APB2ENR &= ~(1<<9))
#define RCC_CLKDI_TIM1			( RCC->APB2ENR &= ~(1<<11))
#define RCC_CLKDI_SPI1			( RCC->APB2ENR &= ~(1<<12))
#define RCC_CLKDI_USART1		( RCC->APB2ENR &= ~(1<<14))
#define RCC_CLKDI_TIM15			( RCC->APB2ENR &= ~(1<<16))
#define RCC_CLKDI_TIM16			( RCC->APB2ENR &= ~(1<<17))
#define RCC_CLKDI_TIM17			( RCC->APB2ENR &= ~(1<<18))
#define RCC_CLKDI_DBGMCU		( RCC->APB2ENR &= ~(1<<22))

#define RCC_CLKDI_TIM3			( RCC->APB1ENR &= ~(1<<1))
#define RCC_CLKDI_TIM6			( RCC->APB1ENR &= ~(1<<4))
#define RCC_CLKDI_TIM7			( RCC->APB1ENR &= ~(1<<5))
#define RCC_CLKDI_TIM14			( RCC->APB1ENR &= ~(1<<8))
#define RCC_CLKDI_WWDG			( RCC->APB1ENR &= ~(1<<11))
#define RCC_CLKDI_SPI2			( RCC->APB1ENR &= ~(1<<14))
#define RCC_CLKDI_USART2		( RCC->APB1ENR &= ~(1<<17))
#define RCC_CLKDI_USART3		( RCC->APB1ENR &= ~(1<<18))
#define RCC_CLKDI_USART4		( RCC->APB1ENR &= ~(1<<19))
#define RCC_CLKDI_USART5		( RCC->APB1ENR &= ~(1<<20))
#define RCC_CLKDI_I2C1			( RCC->APB1ENR &= ~(1<<21))
#define RCC_CLKDI_I2C2			( RCC->APB1ENR &= ~(1<<22))
#define RCC_CLKDI_USB				( RCC->APB1ENR &= ~(1<<23))
#define RCC_CLKDI_PWR				( RCC->APB1ENR &= ~(1<<28))


/*	SPI register definition	*/
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t	CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
}Regdef_SPI;


#define SPI1			((Regdef_SPI *) BASE_ADDR_SPI1)

/* USART register definition */
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t res1;
	volatile uint32_t RTOR;
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
	
}Regdef_USART;

#define USART1 ((Regdef_USART *) BASE_ADDR_USART1)


#endif //_SMT32F401RE_H_
