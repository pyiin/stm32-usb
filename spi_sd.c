#include "stm32f1xx.h"

#define SD_COMMAND_Msk 0x3f
#define SD_COMMAND 0x40
uint8_t command[6] = {
	0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
};

enum {
	IDLE,
	DMA_CMD, //dma1 channel 3
	CRC_SPI,
	RESPONSE,
	DMA_IN, //dma1 channel 2
	DMA_OUT, //dma1 channel 3
	ERR,
} sd_state = IDLE;

void emptyfn() {}
uint8_t SD_ready = 0;
void (*dma_rcv_fn)(void) = emptyfn;
void (*dma_snd_fn)(void) = emptyfn;
void (*transaction_finished)(void) = emptyfn;


void spi1_gpio() {
	//pa4,5,6,7
	//4,5,7 push pull
	//6, input floating
	GPIOA->CRL &= ~( GPIO_CRL_MODE4 | GPIO_CRL_CNF4
					 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5
					 | GPIO_CRL_MODE6 | GPIO_CRL_CNF6
					 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
	GPIOA->CRL |= (0b00 << GPIO_CRL_CNF4_Pos) | (0b11 << GPIO_CRL_MODE4_Pos)
		| (0b10 << GPIO_CRL_CNF5_Pos) | (0b11 << GPIO_CRL_MODE5_Pos)
		| (0b10 << GPIO_CRL_CNF7_Pos) | (0b11 << GPIO_CRL_MODE7_Pos)
		| (0b01 << GPIO_CRL_CNF6_Pos) | (0b00 << GPIO_CRL_MODE6_Pos);
	GPIOA->BSRR |= GPIO_BSRR_BS4;
	/* GPIOA->BSRR |= GPIO_BSRR_BR6; */
}

void spi1_init(){
	spi1_gpio();
	// clock setup
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //4.5MhzSPI
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; //4.5MhzSPI

	SPI1->CR1 =  SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;// | SPI_CR1_SSM; SPI_CR1_CRCEN |
	SPI1->CR1 |= (0b111 << SPI_CR1_BR_Pos); //div by 16, 281K
	/* SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; */

	/* //sp1 dma */
	/* DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR; */
	/* DMA1_Channel3->CCR = (0b10 << DMA_CCR_PL_Pos) */
	/* 	| (0b00 << DMA_CCR_MSIZE_Pos) */
	/* 	| (0b00 << DMA_CCR_PSIZE_Pos) */
	/* 	| DMA_CCR_MINC */
	/* 	| (1<<DMA_CCR_DIR) //tx */
	/* 	| DMA_CCR_TCIE; */
	/* DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR; */
	/* DMA1_Channel2->CCR = (0b10 << DMA_CCR_PL_Pos) */
	/* 	| (0b00 << DMA_CCR_MSIZE_Pos) */
	/* 	| (0b00 << DMA_CCR_PSIZE_Pos) */
	/* 	| DMA_CCR_MINC */
	/* 	| (0<<DMA_CCR_DIR) //rx */
	/* 	| DMA_CCR_TCIE; */
	/* NVIC_EnableIRQ(DMA1_Channel2_IRQn); */
	/* NVIC_EnableIRQ(DMA1_Channel3_IRQn); */
	/* NVIC_SetPriority(DMA1_Channel2_IRQn,2); */
	/* NVIC_SetPriority(DMA1_Channel3_IRQn,2); */
	// CR1 BR & LSBFIRST
	SPI1->I2SCFGR = 0;       // no i2s
	SPI1->CR1 |= SPI_CR1_SPE; // spi enable
}
void sd_rcv_dmareturn();
uint8_t sd_recieve_command(uint8_t *buf, uint8_t numbytes) {
	/* if(sd_state != RESPONSE) return 0; */
	DMA1_Channel3->CNDTR = numbytes;
	DMA1_Channel3->CMAR = (uint32_t)buf;

	DMA1_Channel3->CCR |= DMA_CCR_EN;
	SPI1->CR2 |= SPI_CR2_RXDMAEN;
	
	dma_rcv_fn = sd_rcv_dmareturn;
	return 1;
}
void sd_rcv_dmareturn() {
	sd_state = CRC_SPI;
	/* SPI1->CR1 |= SPI_CR1_CRCNEXT; */
	SPI1->CR2 &= ~SPI_CR2_RXDMAEN;
	sd_state = IDLE;
	transaction_finished();
}

void sd_send_dmareturn();
uint8_t sd_send_command(uint8_t* cmd) {
	if(sd_state != IDLE) return 0;
	DMA1_Channel3->CNDTR = 5;
	DMA1_Channel3->CMAR = (uint32_t)cmd;

	DMA1_Channel3->CCR |= DMA_CCR_EN;
	SPI1->CR2 |= SPI_CR2_TXDMAEN;
	sd_state = DMA_CMD;
	
	dma_snd_fn = sd_send_dmareturn;
	return 1;
}

void sd_send_dmareturn() {
	sd_state = CRC_SPI;
	SPI1->CR1 |= SPI_CR1_CRCNEXT;
	SPI1->CR2 &= ~SPI_CR2_TXDMAEN;
	sd_state = RESPONSE;
}
uint8_t spibuffer[10];
extern uint32_t led_state;

uint8_t spi_rxtx_sync(uint8_t dat){
	while (!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = dat;
	while (!(SPI1->SR & SPI_SR_RXNE));
	return SPI1->DR;
}

void cmd_syncronous(){
	GPIOA->ODR &= ~(1<<4);
	for (uint8_t i = 0; i < 6; i++) {
		spi_rxtx_sync(command[i]);
	}

}

uint8_t reply[32];

void r1_syncronous() {
	uint8_t ans = 0xff;
	for (uint8_t i = 0; i < 8; i++) {
		ans = spi_rxtx_sync(0xff);
		if ((ans & 0x80) == 0) {
			break;
		}
	}
	reply[0] = ans;
}

void r7_syncronous() {
	r1_syncronous();
	for(uint8_t i = 1; i<5; i++)
		reply[i] = spi_rxtx_sync(0xff);
}


void read_csd_sync(){
	command[0] = SD_COMMAND | 9;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0;
	command[4] = 0;
	command[5] = 0;
	cmd_syncronous();
	r1_syncronous();
	uint8_t ans;
	while((ans = spi_rxtx_sync(0xff)) == 0xff);
	
	for(uint16_t i=0; i<16+2; i++)
		reply[i] = spi_rxtx_sync(0xff);
	__NOP();
}

void spi_sd_init2();
uint8_t spi_sd_init() {
	//100 to 400 khz, change after setup
	/* SPI1->CR1 |= SPI_CR1_SSI; */
 CMD0:
	led_state &= 0xffff;
	GPIOA->ODR |= 1<<4;
	for (int i = 0; i < 11; i++) {
		SPI1->DR = 0xff;
		while(!(SPI1->SR & SPI_SR_TXE));
	}
	while((SPI1->SR & SPI_SR_BSY));
	GPIOA->ODR &= ~(1<<4);
	uint32_t ntimes = 0;
	/* led_state = (GPIOA->IDR & GPIO_IDR_IDR6); */

	command[0] = SD_COMMAND;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0;
	command[4] = 0;
	command[5] = 0x95;
	cmd_syncronous();
	r1_syncronous();
	if (reply[0] == 0x00) {
		for(uint32_t i=0;i<7200000;i++)__NOP();
		goto CMD0;
	}
	if (reply[0] != 0x01) {
		led_state |= 0x0f000000;
		led_state |= reply[0]<<16;
		return 0;
	}

	command[0] = SD_COMMAND | 8;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0x01;
	command[4] = 0xaa;
	command[5] = 0x87;
	cmd_syncronous();
	r7_syncronous();

	if (!(reply[3] == 0x01 && reply[4] == 0xaa)) {
		led_state |= 0x08000000;
		return 0;
	}
	led_state |= 0x01000000;


 ACMD41:
	command[0] = SD_COMMAND | 55;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0;
	command[4] = 0;
	command[5] = 0;

	cmd_syncronous();
	r1_syncronous();
	command[0] = SD_COMMAND | 41;
	command[1] = 0x40;
	command[2] = 0;
	command[3] = 0;
	command[4] = 0;
	command[5] = 0;
	cmd_syncronous();
	r1_syncronous();

	if (reply[0] != 0x00) {
		led_state &= ~0x00ff0000;
		led_state |= (reply[0] << 16);
		if(ntimes++ < 800)
			goto ACMD41;
		return 0;
	}
	led_state &= ~0x00ff0000;
	led_state |= 0x02000000;

	command[0] = SD_COMMAND | 58;
	command[1] = 0x58;
	command[2] = 0;
	command[3] = 0;
	command[4] = 0;
	command[5] = 0;
	cmd_syncronous();
	r7_syncronous();
	led_state |= reply[1] << 16;

	if(reply[1] & (1<<7))
		read_csd_sync();
	else
		return 0;
	return 1;
}



uint8_t spi_sd_readsize() {
	if(!SD_ready) return 0;
	return 1;
}

uint8_t spi_sd_readblock(uint32_t blknum) {
	if(!SD_ready) return 0;
	return 1;
}

uint8_t spi_sd_writeblock(uint32_t blknum, void* blkbuf) {
	if(!SD_ready) return 0;
	return 1;
}

void DMA1_Channel2_IRQHandler() { //recieve
	DMA1->IFCR = DMA_IFCR_CGIF2;
	dma_rcv_fn();
}

void DMA1_Channel3_IRQHandler() { //transmit
	DMA1->IFCR = DMA_IFCR_CGIF3;
	dma_snd_fn();
}

void SPI1_IRQHandler() {
	if (SPI1->SR & SPI_SR_TXE) {
	}
	if (SPI1->SR & SPI_SR_RXNE) {
	}
}
