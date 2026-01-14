#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"

uint8_t keys[8] = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
};
uint8_t keysb[8] = {
	0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
};
uint8_t empty[8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


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
  uint8_t ll = 0;
  wait_clk(720000,10);
  while (1) {
	 write_report(&keys);
	 //	 write_report(&empty);
    /* wait(7200000, 10);// 1 second? */
    /* light(0
	   b10101010); */
    /* wait(7200000, 10); */
    /* light(0b01010101); */
	wait_clk(720000, 1);
	//	write_report(&keysb);
	//	write_report(&empty);
	//	wait_clk(720000, 1);
	keys[2] = ++keys[2]%30;
	light(ll++);
  }
}
