#include <stdint.h>
#include "stm32f1xx.h"
#include "misc.h"

uint8_t lightadd(uint8_t b) {
	GPIOA->BSRR = (b&0x01)<<(15+16);
	GPIOC->BSRR = (b&0x0e)<<(9+16);
	GPIOB->BSRR = (b&0xe0)>>(2+16);
	GPIOD->BSRR = (b&0x10)>>(2+16);
	return b;
}


uint8_t light(uint8_t b) {
	b = ~b;
	GPIOA->ODR = (b&0x01)<<15;
	GPIOC->ODR = (b&0x0e)<<9;
	GPIOB->ODR = (b&0xe0)>>2;
	GPIOD->ODR = (b&0x10)>>2;
	return b;
}

uint8_t t;
void SysTick_Handler() {
	t++;
	return;
}

void wait_clk(uint32_t w, uint32_t n){
	SysTick->VAL = 0;
	SysTick->LOAD = (w-1) & 0x00ffffff;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
	t = 0;
	while(t < n)
		__asm("nop");
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
