#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"
#include "ps2.h"
#include "spi_sd.h"

uint8_t keys[8] = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint8_t empty[8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint32_t buffer[10];

void tim6_setup(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	TIM6->PSC = 71;
	TIM6->ARR = 99;

	TIM6->EGR = TIM_EGR_UG;
	TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn,2);

	TIM6->CR1 |= TIM_CR1_CEN;
}

uint32_t led_state=0b0;
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


uint8_t test[] = "test test#";
int main(void)
{
	__enable_irq();

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
		RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; // 0x3c;
	AFIO->MAPR |= (0x2 << 24);//debugging ports remap

	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn,2);

	/* SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk; */
	/* SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk */
	/* 	|  SCB_SHCSR_MEMFAULTENA_Msk */
	/* 	|  SCB_SHCSR_USGFAULTENA_Msk; */

	led_setup();
	clock_setup();

	usb_core_init();
	usb_device_init();
	usb_ep_buf_set(0,buffer);
	tim6_setup();

	ps2_enable();
	spi1_init();

	spi_sd_init();
	
	//33,34,36,37
	//pb12,13,15  pc6
	/* GPIOB->CRH = 0x80880000; //i2s ports remapped to input pull down; */
	/* GPIOB->ODR &= ~0x10110000; */

	//	wait_clk(7200000,5);
	volatile uint64_t sleep = 0;
	volatile uint64_t all = 0;
	volatile uint32_t t1=0;
	volatile uint32_t t0=0;
	volatile uint32_t t2=0;
	/* CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; */
	/* DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; */
	while (1) {
		/* for(int i=0; i<10000;i++) __NOP(); */
		/* usb_hid_send_report(); */
		/* t0 = t1; */
		/* t1 = DWT->CYCCNT; */
		/* __DSB(); */
		/* __WFI(); */
		/* __ISB(); */
		/* t2 = DWT->CYCCNT; */
		/* if(t0>t1) all += 0x100000000ULL; */
		/* sleep += t2-t1; */
	}
}

void HardFault_Handler(unsigned int* hardfault_args) {
	while(1);
}
