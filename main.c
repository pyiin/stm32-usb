#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"

uint8_t keys[8] = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint8_t empty[8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void write_string(uint8_t* buf, uint32_t len){
	uint32_t idx = 0;
	while(idx < len){
		keys[2] = buf[idx] - 'a' + 4;
		if(keys[2] > 0x27) keys[2] = 0x00;
		if(buf[idx] == ' ') keys[2] = 0x2c;
		if(buf[idx] == '#') keys[2] = 0x28;

		while(!write_report(&keys));
		while(!write_report(&empty));
		idx++;
	}
	keys[2] = 0x28;
	while(!write_report(&keys));
	while(!write_report(&empty));
}

void write_string_packed(uint8_t* buf, uint32_t len){
	uint32_t idx = 0;
	while(idx < len){
		for (uint8_t i = 2; i < 8; i++) keys[i] = 0;
		for (uint8_t i = 2; i < 8 & idx < len; i++,idx++) {
			for(uint8_t j=1; j<=i-2; j++)
				if(buf[idx] == buf[idx-j])
					goto write;
            keys[i] = buf[idx] - 'a' + 4;
            if (keys[i] > 0x27)
				keys[i] = 0x00;
            if (buf[idx] == ' ')
				keys[i] = 0x2c;
            if (buf[idx] == '#')
				keys[i] = 0x28;
		}
	write:
		while(!write_report(&keys));
		while(!write_report(&empty));
	}
	keys[2] = 0x28;
	while(!write_report(&keys));
	while(!write_report(&empty));
}

uint32_t buffer[10];

void lightdebug(uint32_t t) {
	for (int i = 0; i < 4; i++, t = (t >> 8)) {
		light(t & 0xff);
		wait_clk(720000, 10);
	}
	light(0x00);
}

void EXTI15_10_IRQHandler() {
	light(0xff);
	switch (EXTI->PR) {
	case 1 << 12:
		write_string((uint8_t*)"a", 1);
		break;
	case 1 << 13:
		write_string((uint8_t*)"b", 1);
		break;
	case 1 << 15:
		write_string((uint8_t*)"c", 1);
		break;
	}
	EXTI->PR = 0b1011<<12;
}

uint8_t test[] = "test test#";
int main(void)
{
	__enable_irq();

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
		RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; // 0x3c;
	AFIO->MAPR |= (0x2 << 24);//debugging ports remap

	NVIC_EnableIRQ(SysTick_IRQn);

	/* SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk; */
	/* SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk */
	/* 	|  SCB_SHCSR_MEMFAULTENA_Msk */
	/* 	|  SCB_SHCSR_USGFAULTENA_Msk; */


	GPIOD->CRL = 0x00000300;
	GPIOC->CRH = 0x00033300;
	GPIOB->CRL = 0x00333000;
	GPIOA->CRH = 0x30000000;
	GPIOB->ODR = 0;
	GPIOA->ODR = 0;
	GPIOD->ODR = 0;
	GPIOC->ODR = 0;
	light(0);
	clock_setup();
	usb_core_init();
	usb_device_init();
	usb_ep_buf_set(0,buffer);

	//33,34,36,37
	//pb12,13,15  pc6
	GPIOB->CRH = 0x80880000; //i2s ports remapped to input pull down;
	GPIOB->ODR &= ~0x10110000;
	EXTI->IMR |= 0b1011<<12; //interrrupt
	EXTI->RTSR |= 0b1011<<12; //rising
	AFIO->EXTICR[3] |= 0x1011; //pb select

	/* NVIC_EnableIRQ(EXTI15_10_IRQn); */
	/* NVIC_SetPriority(EXTI15_10_IRQn,2); */
	
	//	wait_clk(7200000,5);
	while (1) {
		/* write_string_packed((uint8_t*)test, sizeof(test)); */
		wait_clk(7200000, 1);
		//		light(buffer[0] & 0xff);
	}
}

void HardFault_Handler(unsigned int* hardfault_args) {
	while(1);
}
