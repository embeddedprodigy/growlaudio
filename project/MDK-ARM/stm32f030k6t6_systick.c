#include "stm32f030k6t6_systick.h"

volatile uint32_t globalTick;

void SYSTICK_Init(uint32_t load, uint32_t ctrl){
	SYSTICK->LOAD = load;
	SYSTICK->VAL  = 0U;
	SYSTICK->CTRL = ctrl;
}

uint32_t SYSTICK_getTick(void){
	return globalTick;
}

void SYSTICK_incTick(void){
	globalTick++;
}

void SYSTICK_suspendTick(void){
	SYSTICK->CTRL &= ~SYSTICK_CSR_TICKINT;
}

void SYSTICK_resumeTick(void){
	SYSTICK->CTRL |= SYSTICK_CSR_TICKINT;
}

void SysTick_Handler(void){
	SYSTICK_incTick();
}
