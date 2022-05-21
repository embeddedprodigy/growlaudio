#ifndef _STM32F030K6T6_SYSTICK_H_
#define _STM32F030K6T6_SYSTICK_H_

#include "stm32f030k6t6_specifics.h"


#define SYSTICK_CSR_ENABLE    ( 1U << 0 )
#define SYSTICK_CSR_TICKINT	  ( 1U << 1 )
#define SYSTICK_CSR_CLKSOURCE ( 1U << 2 )
#define SYSTICK_CSR_COUNTFLAG ( 1U << 16 )


void SYSTICK_Init(uint32_t load, uint32_t ctrl);
uint32_t SYSTICK_getTick(void);
void SYSTICK_incTick(void);
void SYSTICK_suspendTick(void);
void SYSTICK_resumeTick(void);


#endif //_STM32F030K6T6_SYSTICK_H_
