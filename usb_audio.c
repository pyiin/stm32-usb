#include <stdint.h>
#include "usb.h"
#include "misc.h"

#define MIN_GAP 128
#define MAX_GAP 256

uint8_t abuffer[AUDIO_PCKTSIZ*2+8];

uint8_t half;
int8_t dma_offset = 0;

void audio_check_sync() {
	uint32_t usb_pos = half ? AUDIO_PCKTSIZ : 0;
	uint32_t dma_pos = 2*AUDIO_PCKTSIZ - (DMA1_Channel5->CNDTR << 1);
	if(dma_pos > usb_pos) usb_pos += 2*AUDIO_PCKTSIZ;
	if (usb_pos - dma_pos < MIN_GAP) {
		dma_offset = 1;
		light(0xff);
	}
	else if (usb_pos - dma_pos > MAX_GAP) {
		dma_offset = -1;
		light(0xff);
	}
	else {
		/* light(0x00); */
	}
	// stop dma and restart one step closer
}

void dma_en(int8_t offset) {
	DMA1_Channel5->CCR = 0;
	DMA1_Channel5->CCR = (0b11 <<DMA_CCR_PL_Pos)
		| (0x01 << DMA_CCR_MSIZE_Pos)
		| (0x01 << DMA_CCR_PSIZE_Pos)
		| DMA_CCR_MINC
		/* | DMA_CCR_CIRC */
		| DMA_CCR_DIR
		| DMA_CCR_HTIE
		| DMA_CCR_TCIE;
	DMA1_Channel5->CMAR = (uint32_t)abuffer;
	DMA1_Channel5->CNDTR = AUDIO_PCKTSIZ + (offset*4);
	DMA1_Channel5->CPAR = (uint32_t)(&SPI2->DR);
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void dma_setup(){
	NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	dma_en(0);
}

void stream_packet_recieved(uint32_t bcnt) {
	static uint8_t i=0;
	static uint8_t parity;
	usb_set_out_ep_iso(AUDIO_EP, AUDIO_PCKTSIZ, 1, parity);
	parity = !parity;
	int32_t v = *(int32_t*)abuffer;
	v = (v<0)?-v:v;
	if (i++ == 50) {
		light(v >> 21);
		i=0;
	}
	/* if(half == 1) */
	/* 	light(0xf0); */
	/* else */
	/* 	light(0x0f); */
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

uint8_t deinitflag = 0;
void audio_deinit() {
	deinitflag = 1;
}

void audio_init() {
	//use pll3
	usb_ep_buf_set(1, (uint32_t*)abuffer);
	i2s2_gpio();
	// x9 /1 and /51 here. Gives 44.1176k
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	RCC->CFGR2 |= RCC_CFGR2_PLL3MUL12;
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV2_Msk;
	RCC->CFGR2 |= RCC_CFGR2_PREDIV2_DIV5;
	RCC->CFGR2 |= RCC_CFGR2_I2S2SRC;
	RCC->CR |= RCC_CR_PLL3ON;
	while(!(RCC->CR & RCC_CR_PLL3RDY));
	
	SPI2->I2SCFGR = 0;
	SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD | (0b10 << SPI_I2SCFGR_I2SCFG_Pos)
		| (0b00 << SPI_I2SCFGR_I2SSTD_Pos)// pcm
		| (0b10 << SPI_I2SCFGR_DATLEN_Pos) // 32 bit
		| SPI_I2SCFGR_CHLEN;

	SPI2->CR2 |= SPI_CR2_ERRIE;
	
	NVIC_SetPriority(SPI2_IRQn, 0);
	NVIC_EnableIRQ(SPI2_IRQn);
	
	SPI2->I2SPR = 12 | (SPI_I2SPR_ODD);// | SPI_I2SPR_MCKOE;

	dma_setup();
	SPI2->CR2 |= SPI_CR2_TXDMAEN; //dma enable
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; //enable
	/* while(!(SPI2->I2SCFGR & SPI_I2SCFGR_I2SE)); */
}

void SPI2_IRQHandler() {
	SPI2->SR |= SPI_SR_CRCERR;
	light(0xf0);
	//restart spi and dma.
	SPI2->I2SCFGR &= ~SPI_I2SCFGR_I2SE_Msk; //enable
	dma_setup();
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; //enable
}

void DMA1_Channel5_IRQHandler() { //not equal packet sizes.
	if (DMA1->ISR & DMA_ISR_TCIF5) {
		if (deinitflag == 1) {
			SPI2->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
			DMA1_Channel5->CCR &= ~DMA_CCR_EN;
			deinitflag = 0;
			return;
		}
		usb_ep_buf_set(1, (uint32_t *)(abuffer + AUDIO_PCKTSIZ));
		half = 1;
		dma_en(dma_offset);
		dma_offset = 0;
	} else if (DMA1->ISR & DMA_ISR_HTIF5) {
		usb_ep_buf_set(1, (uint32_t *)abuffer);
		half = 0;
	}
	/* if(DMA1->ISR &  DMA_ISR_HTIF5 & DMA_ISR_TCIF5) */
	/* 	light(0xa1); */
	DMA1->IFCR = DMA_IFCR_CGIF5;
}
