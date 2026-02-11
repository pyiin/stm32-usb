#include <stdint.h>
#include "usb.h"
#include "misc.h"

uint8_t abuffer[AUDIO_PCKTSIZ*2];
uint32_t dataid;

void dma_setup(){
	NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel5->CCR = 0;
	DMA1_Channel5->CCR = (0b11 <<DMA_CCR_PL_Pos)
		| (0x01 << DMA_CCR_MSIZE_Pos)
		| (0x01 << DMA_CCR_PSIZE_Pos)
		| DMA_CCR_MINC
		| DMA_CCR_CIRC
		| DMA_CCR_DIR
		| DMA_CCR_HTIE
		| DMA_CCR_TCIE;
	DMA1_Channel5->CMAR = (uint32_t)abuffer;
	DMA1_Channel5->CNDTR = AUDIO_PCKTSIZ;
	DMA1_Channel5->CPAR = (uint32_t)(&SPI2->DR);
}

void stream_packet_recieved(uint32_t bcnt) {
	static uint8_t i=0;
	usb_ep_buf_set(1, (uint32_t*)abuffer);
	light(++i);
	usb_set_out_ep(AUDIO_EP, AUDIO_PCKTSIZ, 1);
	dataid = 0;
}

void i2s2_gpio() {
	GPIOC->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
	GPIOC->CRL |=  ((0b11 << GPIO_CRL_MODE6_Pos) | GPIO_CRL_CNF6_1);
	GPIOC->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);

	GPIOB->CRH &=
		~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12 |
		  GPIO_CRH_MODE13 | GPIO_CRH_CNF13 |
		  GPIO_CRH_MODE15 | GPIO_CRH_CNF15);
	GPIOB->CRH |=
		(GPIO_CRH_MODE12_1 | GPIO_CRH_CNF12_1) |  // WS
		(GPIO_CRH_MODE13_1 | GPIO_CRH_CNF13_1) |  // CK
		(GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1);   // SD

}

void audio_init() {
	//use pll3
	usb_ep_buf_set(1, (uint32_t*)abuffer);
	// x9 /1 and /51 here. Gives 44.1176k
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	RCC->CFGR2 |= RCC_CFGR2_PLL3MUL9;
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV2_Msk;
	RCC->CFGR2 |= RCC_CFGR2_PREDIV2_DIV1;
	RCC->CFGR2 |= RCC_CFGR2_I2S2SRC;
	RCC->CR |= RCC_CR_PLL3ON;
	while(!(RCC->CR & RCC_CR_PLL3RDY));
	
	SPI2->I2SCFGR = 0;
	SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD | (0b10 << SPI_I2SCFGR_I2SCFG_Pos)
		| (0b00 << SPI_I2SCFGR_I2SSTD_Pos)// pcm
		| (0b10 << SPI_I2SCFGR_DATLEN_Pos) // 32 bit
		| SPI_I2SCFGR_CHLEN;
	SPI2->CR2 |= SPI_CR2_TXDMAEN; //dma enable
	/* SPI2->CR2 |= SPI_CR2_TXEIE; */
	/* NVIC_SetPriority(SPI2_IRQn, 1); */
	/* NVIC_EnableIRQ(SPI2_IRQn); */
	
	SPI2->I2SPR = 25 | (SPI_I2SPR_ODD);// | SPI_I2SPR_MCKOE;

	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; //enable

	//dma
	dma_setup();
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void SPI2_IRQHandler() {
		SPI2->DR = abuffer[dataid++];
		if(dataid >= AUDIO_PCKTSIZ)
			dataid = 0;
}


void DMA1_Channel5_IRQHandler() { //not equal packet sizes.
	if (DMA1->ISR & DMA_ISR_GIF5)
		usb_ep_buf_set(1, (uint32_t*)abuffer);
	if (DMA1->ISR & DMA_ISR_TCIF5)
		usb_ep_buf_set(1, (uint32_t*)(abuffer + AUDIO_PCKTSIZ));

	DMA1->IFCR |= DMA_IFCR_CGIF5
		| DMA_IFCR_CTCIF5;
}
