#include <stdint.h>
#include "stm32f1xx.h"
#include "misc.h"
#include "key_matrix.h"
#include "uart.h"

void tim6_setup(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	TIM6->PSC = 710;
	TIM6->ARR = 99;

	TIM6->EGR = TIM_EGR_UG;
	TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn,2);

	TIM6->CR1 |= TIM_CR1_CEN;
}


void TIM6_IRQHandler(){
	if (TIM6->SR & TIM_SR_UIF)
		TIM6->SR &= ~TIM_SR_UIF;
	read_keys();
	dma_send_usart();
}

extern uint8_t key_state[4];

void clock_setup(){
	RCC->CR |= RCC_CR_HSEON; //enable hse
	while(!(RCC->CR & RCC_CR_HSERDY));

	//setup prediv12 and prediv1scr
	RCC->CFGR2 |= RCC_CFGR2_PREDIV1_DIV2;
	RCC->CFGR |= (0b0111<<RCC_CFGR_PLLMULL_Pos);
	RCC->CFGR |= (0b0<<RCC_CFGR_OTGFSPRE_Pos);
	RCC->CFGR |= RCC_CFGR_PLLSRC;

	//enable pll
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	FLASH->ACR = FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
	//switch to pll
	RCC->CFGR |= RCC_CFGR_SW_PLL;
}

int main(void)
{
	__enable_irq();
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
		RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; // 0x3c;
	AFIO->MAPR |= (0x2 << 24);//debugging ports remap

	/* NVIC_EnableIRQ(SysTick_IRQn); */
	/* NVIC_SetPriority(SysTick_IRQn,2); */

	clock_setup();
	uart_tx_init();
	tim6_setup();
	key_setup();
	while (1) {
		/* read_keys(); */
		/* dma_send_usart(); */
		/* for(uint32_t i=0; i<100000000; i++) __NOP(); */
	}
}

void HardFault_Handler(unsigned int* hardfault_args) {
	while(1);
}
