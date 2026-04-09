#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"
#include "ps2.h"
#include "spi_sd.h"
#include "key_matrix.h"
#include "uart.h"


uint8_t keys[8] = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint8_t empty[8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint32_t buffer[10];

uint32_t led_state=0b0;

void tim6_setup(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	TIM6->PSC = 71;
	TIM6->ARR = 99;

	TIM6->EGR = TIM_EGR_UG;
	TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn,3);

	TIM6->CR1 |= TIM_CR1_CEN;
}


void TIM6_IRQHandler(){
	/* static uint32_t num = 0; */
	/* if(num==2000){ */
	/* 	led_state<<=1, num=0; */
	/* 	if(led_state == 0) led_state = 1; */
	/* } */
	static uint8_t current_led = 0;
	if (TIM6->SR & TIM_SR_UIF)
		TIM6->SR &= ~TIM_SR_UIF;
	if(led_state & (1<<(current_led)))
		light_id(current_led);
	else
		light_off();
	++current_led;
	if(current_led==13) current_led = 16;
	if(current_led==31) current_led = 0;
	/* num++; */
}

void led_setup(){
	//pc11,12; pd2; pb3..9; pc0..3;
	GPIOC->CRH = 0x00033000;
	GPIOC->CRL = 0x00003333;
	GPIOD->CRL = 0x00000300;
	GPIOB->CRL = 0x33333000;
	GPIOB->CRH = 0x00000033;
	GPIOA->ODR = 0;
	GPIOB->ODR = 0;
	GPIOC->ODR = 0;
	GPIOD->ODR = 0;
}

uint8_t blkbuf[1024];

extern uint8_t key_state[4];
extern uint8_t right_key_state[4];
extern uint8_t usart_overrun;
extern uint32_t usart_overrun_cnt;

extern volatile request_t rq;
extern volatile uint8_t data_ready;

extern volatile transfer_state_t scsi_transfer;
extern volatile uint32_t num_written;
int main(void)
{
	__enable_irq();
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
		RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; // 0x3c;
	AFIO->MAPR |= (0x2 << 24); // debugging ports remap

	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn, 3);

	clock_setup();
	
#ifdef HW2
	led_setup();
	tim6_setup();
#endif
	/* spi1_init(); */
	/* spi_sd_init(); */
	/* spi_sd_readblock(0, blkbuf); */
	/* for(uint32_t i = 0; i<1000000; i++) __NOP(); */
	/* spi_sd_readblock(1, blkbuf+512); */
	
	usb_core_init();
	usb_device_init();
	usb_ep_buf_set(0,buffer);

	/* ps2_enable(); */

#ifdef HW3
	key_setup();
	uart_rx_init();
#endif
	volatile uint8_t reply_code;
	while (1) {
		if (scsi_transfer == REQUEST_READ) {
			spi_sd_readblock(rq.blknum, rq.buf);
			while(!data_ready){}
			/* __ISB(); */
			/* __DSB(); */
			/* __DMB(); */
			__disable_irq();
			scsi_send_queued();
			__enable_irq();
		}
		if (scsi_transfer == REQUEST_WRITE) {
			USB_OTG_OUTEndpointTypeDef* epout = usbEpout(3);//SCSI_EP
			reply_code = spi_sd_writeblock(rq.blknum, rq.buf);
			while(!data_ready){}
			if (rq.next) {
				__disable_irq();
				usb_set_out_ep(3, 512, 512 >> 6);
				__enable_irq();
			} else {
				__disable_irq();
				reply_bulk_scsi();
				__enable_irq();
			}
		}
		reply_code = 0;
	}
}

void HardFault_Handler(unsigned int* hardfault_args) {
	while(1);
}
