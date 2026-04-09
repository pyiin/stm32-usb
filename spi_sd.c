#include "stm32f1xx.h"
#include "usb_scsi.h"

#define BLOCK_SIZE 512
#define SPI_SD SPI1
#define SD_COMMAND_Msk 0x3f
#define SD_COMMAND 0x40

#ifdef HW2
#define SD_CS GPIOA->ODR
#define CS_Msk (1<<4)
#endif
#ifdef HW3
#define SD_CS GPIOD->ODR
#define CS_Msk (1<<2)
#endif

#define MAX_RESET_RETRIES 2

extern volatile uint8_t sd_request;
extern volatile uint8_t data_ready;
extern volatile transfer_state_t scsi_transfer;


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

void spi_set_hs(){
	while(SPI_SD->SR & SPI_SR_BSY);
	SPI_SD->CR1 &= ~SPI_CR1_BR;
	SPI_SD->CR1 |= (0b011 << SPI_CR1_BR_Pos); //div by 256, 281K
}
void spi_set_ls(){
	SPI_SD->CR1 &= ~SPI_CR1_BR;
	SPI_SD->CR1 |= (0b111 << SPI_CR1_BR_Pos); //div by 256, 281K
}

void spi1_gpio() {
	//pa4,5,6,7
	//4,5,7 push pull
	//6, input floating
	// keyboard ======
	// miso pb4
	// mosi pb5
	// sclk pb3
	// cs pd4
#ifdef HW2
	GPIOA->CRL &= ~( GPIO_CRL_MODE4 | GPIO_CRL_CNF4
					 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5
					 | GPIO_CRL_MODE6 | GPIO_CRL_CNF6
					 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
	GPIOA->CRL |= (0b00 << GPIO_CRL_CNF4_Pos) | (0b11 << GPIO_CRL_MODE4_Pos)
		| (0b10 << GPIO_CRL_CNF5_Pos) | (0b11 << GPIO_CRL_MODE5_Pos)
		| (0b10 << GPIO_CRL_CNF7_Pos) | (0b11 << GPIO_CRL_MODE7_Pos)
		| (0b01 << GPIO_CRL_CNF6_Pos) | (0b00 << GPIO_CRL_MODE6_Pos);
	GPIOA->BSRR |= GPIO_BSRR_BS4;
#endif
#ifdef HW3
	AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;
	GPIOB->CRL &= ~( GPIO_CRL_MODE4 | GPIO_CRL_CNF4
					 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5
					 | GPIO_CRL_MODE3 | GPIO_CRL_CNF3);

	GPIOB->CRL |= (0b10 << GPIO_CRL_CNF4_Pos) | (0b00 << GPIO_CRL_MODE4_Pos)
		| (0b10 << GPIO_CRL_CNF5_Pos) | (0b11 << GPIO_CRL_MODE5_Pos)
		| (0b10 << GPIO_CRL_CNF3_Pos) | (0b11 << GPIO_CRL_MODE3_Pos);

	GPIOB->BSRR = GPIO_BSRR_BS4;

	GPIOD->CRL &= ~( GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
	GPIOD->CRL |= (0b00 << GPIO_CRL_CNF2_Pos) | (0b11 << GPIO_CRL_MODE2_Pos);
	GPIOD->BSRR |= GPIO_BSRR_BR2;
#endif
		
	//pc4 debug triggger

	GPIOC->CRL |= ~( GPIO_CRL_MODE4 | GPIO_CRL_CNF4 );
	GPIOC->CRL |= (0b01 << GPIO_CRL_CNF4_Pos) | (0b00 << GPIO_CRL_MODE4_Pos);
	GPIOC->BSRR |= GPIO_BSRR_BR4;

}

void spi1_init(){
	spi1_gpio();
	// clock setup
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;


	SPI_SD->CR1 =  SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;// | SPI_CR1_SSM; SPI_CR1_CRCEN |
	SPI_SD->CR1 |= (0b111 << SPI_CR1_BR_Pos); //div by 16, 281K

	/* //sp1 dma */
	DMA1_Channel3->CPAR = (uint32_t)&SPI_SD->DR;
	DMA1_Channel3->CCR = (0b01 << DMA_CCR_PL_Pos)
		| (0b00 << DMA_CCR_MSIZE_Pos)
		| (0b00 << DMA_CCR_PSIZE_Pos)
		| DMA_CCR_MINC
		| (DMA_CCR_DIR) //tx
		| DMA_CCR_TEIE
		| DMA_CCR_TCIE;
	DMA1_Channel2->CPAR = (uint32_t)&SPI_SD->DR;
	DMA1_Channel2->CCR = (0b01 << DMA_CCR_PL_Pos)
		| (0b00 << DMA_CCR_MSIZE_Pos)
		| (0b00 << DMA_CCR_PSIZE_Pos)
		| DMA_CCR_MINC
		| DMA_CCR_TEIE
		//		| (DMA_CCR_DIR) //rx
		| DMA_CCR_TCIE;
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_SetPriority(DMA1_Channel2_IRQn,0);
	NVIC_SetPriority(DMA1_Channel3_IRQn,0);

	SPI_SD->CR2 = SPI_CR2_RXDMAEN
		| SPI_CR2_TXDMAEN;
	// CR1 BR & LSBFIRST
	SPI_SD->I2SCFGR = 0;       // no i2s
	SPI_SD->CR1 |= SPI_CR1_SPE; // spi enable
}
void sd_rcv_dmareturn();
uint8_t spibuffer[10];
extern uint32_t led_state;

uint8_t spi_rxtx_sync(uint8_t dat){
	while (!(SPI_SD->SR & SPI_SR_TXE));
	SPI_SD->DR = dat;
	while (!(SPI_SD->SR & SPI_SR_RXNE));
	return SPI_SD->DR;
}

void cmd_syncronous(){
	GPIOA->ODR &= ~(1<<4);
	for (uint8_t i = 0; i < 6; i++) {
		spi_rxtx_sync(command[i]);
	}

}

uint8_t reply[32];

inline void wait_ready() {
  while (spi_rxtx_sync(0xff) == 0x00)
    ;
}

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

struct sd_csd {
	uint8_t data0[7];
	uint8_t dsize[3];
	uint8_t data1[6];
	uint8_t crc[2];
} sd_csd;

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
	while ((ans = spi_rxtx_sync(0xff)) == 0xff)
		;

	for (uint16_t i = 0; i < 16 + 2; i++)
		*((uint8_t*)(&sd_csd)+i) = spi_rxtx_sync(0xff);
	led_state &= 0xff00ffff;
	led_state |= sd_csd.dsize[1]<<16;
	__NOP();
}

uint8_t spi_sd_init() {
	spi_set_ls();
	//100 to 400 khz, change after setup
	/* SPI_SD->CR1 |= SPI_CR1_SSI; */
	uint8_t cnt = 0;
	for(uint32_t i=0;i<7200000;i++)__NOP();
 CMD0:
	led_state &= 0xffff;
	SD_CS |= CS_Msk;
	for (int i = 0; i < 11; i++) {
		SPI_SD->DR = 0xff;
		while(!(SPI_SD->SR & SPI_SR_TXE));
	}
	while((SPI_SD->SR & SPI_SR_BSY));
	SD_CS &= ~CS_Msk;
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
	if (reply[0] != 0x01) {
		led_state |= 0x0f000000;
		led_state |= reply[0]<<16;
		return 0;
	}
	else{
		for(uint32_t i=0;i<7200000;i++)__NOP();
		if(++cnt < MAX_RESET_RETRIES)
			goto CMD0;
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
	if (reply[1] & (1 << 7)) {
		spi_set_hs();
		read_csd_sync();
	} else
		return 0;
	SD_ready = 1;
	return 1;
}



uint8_t spi_sd_readsize(uint32_t* capacity) {
	if(!SD_ready) return 0;
	/* capacity[1] = BLOCK_SIZE; */
	uint32_t blkcnt = (sd_csd.dsize[2])
				   | (sd_csd.dsize[1]<<8)
				   | (sd_csd.dsize[0]<<16);
	blkcnt <<= 10;
	blkcnt |= 0b1111111111;
	capacity[0] = __REV(blkcnt);
	/* capacity[0] <<= 10; //that was in kilobytes */
	/* capacity[0]--; // LBA  */
	return 1;
}

uint8_t spi_sd_readblock(uint32_t blknum, void* blkbuf) {
	if(!SD_ready) return 0;
	wait_ready();
	command[0] = 0x40 | 17;
	command[1] = (blknum & 0xff000000) >> 24;
	command[2] = (blknum & 0xff0000) >> 16;
	command[3] = (blknum & 0xff00) >> 8;
	command[4] = blknum & 0xff;
	command[5] = 0;
	static uint32_t ff = 0;
	ff=0xff;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	
	DMA1_Channel3->CNDTR = BLOCK_SIZE+2;
	DMA1_Channel3->CMAR = (uint32_t)(&ff);
	DMA1_Channel3->CCR &= ~DMA_CCR_MINC; //not increment

	DMA1_Channel2->CNDTR = BLOCK_SIZE+2;
	DMA1_Channel2->CMAR = (uint32_t)blkbuf;
 CMD17:
	cmd_syncronous();
	uint8_t ans = 0;
	r1_syncronous();
	if(reply[0] & 0xfe)
		goto CMD17;
	while ((ans = spi_rxtx_sync(0xff)) == 0xff);

	dma_rcv_fn = 0;
	dma_snd_fn = 0;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	sd_state = DMA_CMD;

	/* dma_rcv_fn = buffer_ready; */
	return 1;
}
uint32_t num_written = 0;
void block_written(){
	volatile uint8_t data_rsp = 0;
	while ((data_rsp = spi_rxtx_sync(0xff)) == 0xff);
	num_written ++;
	if((data_rsp & 0x0f) != 0b0101){ //fix later
		/* while(1){ */
		/* 	(void) data_rsp; */
		/* } */
	}

	data_ready=1;
	scsi_transfer = NO_REQUEST;
	sd_state = IDLE;
}

uint8_t spi_sd_writeblock(uint32_t blknum, void* blkbuf) {
	if(!SD_ready) return 0;
	wait_ready();
	spi_rxtx_sync(0xff);
	spi_rxtx_sync(0xff);
	spi_rxtx_sync(0xff);
	command[0] = 0x40 | 24;
	command[1] = (blknum & 0xff000000) >> 24;
	command[2] = (blknum & 0xff0000) >> 16;
	command[3] = (blknum & 0xff00) >> 8;
	command[4] = blknum & 0xff;
	command[5] = 0;
	
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CNDTR = BLOCK_SIZE+2;
	DMA1_Channel3->CMAR = (uint32_t)(blkbuf);
	DMA1_Channel3->CCR |= DMA_CCR_MINC; //increment
 CMD24:
	cmd_syncronous();
	uint8_t ans = 0;
	r1_syncronous();
	if(reply[0] != 0x00) goto CMD24;
	spi_rxtx_sync(0xff);
	spi_rxtx_sync(0xfe);
	dma_snd_fn = block_written;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	sd_state = DMA_CMD;
	return 1;
}

void DMA1_Channel2_IRQHandler() { //recieve
	if (DMA1->ISR & DMA_ISR_TEIF2) {
		while(1);
	}
	DMA1->IFCR = DMA_IFCR_CTCIF2;
	if(dma_rcv_fn)
		dma_rcv_fn();
	sd_state = IDLE;
	data_ready=1;
	scsi_transfer = NO_REQUEST;
}

void DMA1_Channel3_IRQHandler() { //transmit
	if (DMA1->ISR & DMA_ISR_TEIF3) {
		while(1);
	}
	DMA1->IFCR = DMA_IFCR_CTCIF3;
	if(dma_snd_fn)
		dma_snd_fn();
}
