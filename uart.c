#include "stm32f1xx.h"

extern uint8_t key_state[4];

void uart_tx_init() {
	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	//pb6
	GPIOB->CRL &= ~( GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
	GPIOB->CRL |= (0b10 << GPIO_CRL_CNF6_Pos) | (0b11 << GPIO_CRL_MODE6_Pos);

	USART1->BRR = 0x0271;//128.0
	USART1->CR3 = USART_CR3_DMAT;
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
	//idle interrupt.
	// on idle set dma to correct amount of data to buffer.
	// only rx


	//BR
	//CR2: 1 stop bit,
	//CR3: DMA, (half-duplex)
	// dma1_channel4,5 (fuck!)
	DMA1_Channel4->CCR = DMA_CCR_MINC
		| DMA_CCR_DIR;
	DMA1_Channel4->CPAR = (uint32_t)(&USART1->DR);
}

void dma_send_usart() {
	DMA1->IFCR = DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	DMA1_Channel4->CNDTR = 4;
	DMA1_Channel4->CMAR = (uint32_t)key_state;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}
