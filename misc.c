#include <stdint.h>
#include "stm32f1xx.h"
#include "misc.h"

uint8_t lightadd(uint8_t b) {
	GPIOA->BSRR = (b&0x01)<<(15+16);
	GPIOC->BSRR = (b&0x0e)<<(9+16);
	GPIOB->BSRR = (b&0xe0)<<(-2+16);
	GPIOD->BSRR = (b&0x10)<<(-2+16);
	return b;
}

uint8_t light(uint8_t b) {
	GPIOA->BSRR = (b&0x01)<<(15+16);
	GPIOC->BSRR = (b&0x0e)<<(9+16);
	GPIOB->BSRR = (b&0xe0)<<(-2+16);
	GPIOD->BSRR = (b&0x10)<<(-2+16);
	b = ~b;
	GPIOA->BSRR = (b&0x01)<<15;
	GPIOC->BSRR = (b&0x0e)<<9;
	GPIOB->BSRR = (b&0xe0)>>2;
	GPIOD->BSRR = (b&0x10)>>2;
	return b;
}

#define D_ODR_Msk 0b0000000000000100
#define B_ODR_Msk 0b0000001111111000
#define C_ODR_Msk 0b0001100000001111
#define C_num_Msk 0b0011000000001111
#define B_num_Msk 0b0000011111110000
#define D_num_Msk 0b0000100000000000

uint16_t positions[] = {3,2,1,0,9,8,7,6,5,4,3,2,12,11};

void light_off() {
  GPIOB->ODR &= ~B_ODR_Msk;
  GPIOC->ODR &= ~C_ODR_Msk;
  GPIOD->ODR &= ~D_ODR_Msk;
}

void light_id(uint8_t id) {
	light_off();
	uint8_t id_small = id & 0x0f;
	uint8_t dir = (id>>4) & 0x01;
	uint16_t num_msk = 1;
	uint16_t gpiobodr = 0;
	uint16_t gpiocodr = 0;
	uint16_t gpiododr = 0;
	for (uint8_t i = 0; i < 14; i++,num_msk<<=1) {
		if(i==id_small+1) dir=!dir;
		if (B_num_Msk & num_msk) {
			gpiobodr |= (dir<<positions[i]);
		}
		if (C_num_Msk & num_msk) {
			gpiocodr |= (dir<<positions[i]);
		}
		if (D_num_Msk & num_msk) {
			gpiododr |= (dir<<positions[i]);
		}
	}
	GPIOB->ODR |= gpiobodr;
	GPIOC->ODR |= gpiocodr;
	GPIOD->ODR |= gpiododr;
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
