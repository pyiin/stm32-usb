#include "stm32f1xx.h"

void uart_rx_init() {
	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

	USART1->CR1 = USART_CR1_UE | USART_CR1_RE;
	//idle interrupt.
	// on idle set dma to correct amount of data to buffer.
	// only rx


	//BR
	//CR2: 1 stop bit,
	//CR3: DMA, (half-duplex)
	// dma1_channel4,5 (fuck!)
}
