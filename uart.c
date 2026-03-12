#include "stm32f1xx.h"

uint8_t right_key_state[4];
uint8_t usart_overrun = 0;
uint32_t usart_overrun_cnt = 0;

void uart_rx_init() {
	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
	
	//pb7
	GPIOB->CRL &= ~( GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
	GPIOB->CRL |= (0b01 << GPIO_CRL_CNF6_Pos) | (0b00 << GPIO_CRL_MODE6_Pos);

	USART1->BRR = (32<<4);//since i have a apb2 prescaler
	USART1->CR3 = 0;
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_UE | USART_CR1_RE;

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,2);
}

void USART1_IRQHandler() {
	static uint8_t pos = 0;
	if (USART1->SR & USART_SR_ORE) {
		usart_overrun=2;
		usart_overrun_cnt++;
	}
	if (USART1->SR & USART_SR_IDLE) {
		if(usart_overrun>0) usart_overrun--;
		pos = 0;
		(void) USART1->SR;
		(void) USART1->DR;
	}
	if (USART1->SR & USART_SR_RXNE) {
		if(usart_overrun>0) return;
		uint8_t data = USART1->DR;
		if(pos<4)
			right_key_state[pos++] = data;
	}

}
