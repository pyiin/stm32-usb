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

/* void write_string_packed2(uint8_t* buf, uint32_t len){ */
/* 	uint32_t idx = 0; */
/* 	uint32_t prev_idx = 0; */
/* 	uint32_t idx_tmp = 0; */
/* 	while(idx < len){ */
/* 		idx_tmp = idx; */
/* 		for (uint8_t i = 2; i < 8; i++) keys[i] = 0; */
/* 		for (uint8_t i = 2; (i < 8) & (idx < len); i++,idx++) { */
/* 			for(uint32_t j = prev_idx; j < idx; j++){ */
/* 				if(buf[idx] == buf[j]) */
/* 					goto write; */
/* 			} */
/*             keys[i] = buf[idx] - 'a' + 4; */
/*             if (keys[i] > 0x27) */
/* 				keys[i] = 0x00; */
/*             if (buf[idx] == ' ') */
/* 				keys[i] = 0x2c; */
/*             if (buf[idx] == '#') */
/* 				keys[i] = 0x28; */
/* 		} */
/* 	write: */
/* 		prev_idx = idx_tmp; */
/* 		while(!write_report(&keys)); */
/* 	} */
/* 	keys[2] = 0x28; */
/* 	while(!write_report(&empty)); */
/* 	while(!write_report(&keys)); */
/* 	while(!write_report(&empty)); */
/* } */

uint8_t test[] = "aaaa bbbb";
int main(void)
{
	__enable_irq();

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
		RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; // 0x3c;
	AFIO->MAPR |= (0x2 << 24);//debugging ports remap

	NVIC_EnableIRQ(SysTick_IRQn);

	GPIOD->CRL = 0x00000200;
	GPIOC->CRH = 0x00022200;
	GPIOB->CRL = 0x00222000;
	GPIOA->CRH = 0x20000000;
	GPIOB->ODR = 0;
	GPIOA->ODR = 0;
	GPIOD->ODR = 0;
	GPIOC->ODR = 0;
	light(0);
	clock_setup();
	usb_core_init();
	usb_device_init();
	uint8_t st = 0;
	uint8_t b = 0;
	wait_clk(7200000,5);
	while (1) {
		/* if(b==0) */
		/* 	st = write_report(&empty); */
		/* else */
		/* 	st = write_report(&keys); */
		/* if(st == 1) b = (++b)%2; */
		write_string_packed((uint8_t*)&bee_movie, sizeof(bee_movie));
		/* write_string_packed((uint8_t*)test, sizeof(test)); */
		wait_clk(72000000, 1);
		//		if(st) keys[2] = ++keys[2]%30;
	}
}
