#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif



int main(void)
{
	__enable_irq();

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN;// 0x3c;
    AFIO->MAPR |=  (0x2 << 24);

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
	uint8_t ll =0;
	while(1){
		/* wait(7200000, 10);// 1 second? */
		/* light(0b10101010); */
		/* wait(7200000, 10); */
		/* light(0b01010101); */
		wait_clk(7200000, 1);
		light(ll++);
	}
}
