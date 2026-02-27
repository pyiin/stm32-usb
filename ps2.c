#include "stm32f1xx.h"
#include "usb_hid.h"
#include "spi_sd.h"
/* PC10 interrupt */
uint8_t escps2[] = {
    // ps2 codes e0 xx
    [0x7c] = 0x46, [0x7e] = 0x48, [0x70] = 0x49, [0x6c] = 0x4a, [0x7d] = 0x4b,
    [0x71] = 0x4c, [0x69] = 0x4d, [0x7a] = 0x4e, [0x74] = 0x4f, [0x6b] = 0x50,
    [0x72] = 0x51, [0x75] = 0x52, [0x4a] = 0x54, [0x5a] = 0x58,
	
	[0x1f] = 0xe3 - 0xe0 - HID_REPORT_OFFSET,
	[0x14] = 0xe4 - 0xe0 - HID_REPORT_OFFSET,
	[0x11] = 0xe6 - 0xe0 - HID_REPORT_OFFSET,
	[0x27] = 0xe7 - 0xe0 - HID_REPORT_OFFSET,
};

uint8_t normps2[] = {
    // ps codes xx
    [0x1c] = 0x04, [0x32] = 0x05, [0x21] = 0x06, [0x23] = 0x07, [0x24] = 0x08,
    [0x2b] = 0x09, [0x34] = 0x0a, [0x33] = 0x0b, [0x43] = 0x0c, [0x3b] = 0x0d,
    [0x42] = 0x0e, [0x4b] = 0x0f, [0x3a] = 0x10, [0x31] = 0x11, [0x44] = 0x12,
    [0x4d] = 0x13, [0x15] = 0x14, [0x2d] = 0x15, [0x1b] = 0x16, [0x2c] = 0x17,
    [0x3c] = 0x18, [0x2a] = 0x19, [0x1d] = 0x1a, [0x22] = 0x1b, [0x35] = 0x1c,
    [0x1a] = 0x1d, [0x16] = 0x1e, [0x1e] = 0x1f, [0x26] = 0x20, [0x25] = 0x21,
    [0x2e] = 0x22, [0x36] = 0x23, [0x3d] = 0x24, [0x3e] = 0x25, [0x46] = 0x26,
    [0x45] = 0x27, [0x5a] = 0x28, [0x76] = 0x29, [0x66] = 0x2a, [0x0d] = 0x2b,
    [0x29] = 0x2c, [0x4e] = 0x2d, [0x55] = 0x2e, [0x54] = 0x2f, [0x5b] = 0x30,
    [0x5d] = 0x32, [0x4c] = 0x33, [0x52] = 0x34, [0x0e] = 0x35, [0x41] = 0x36,
    [0x49] = 0x37, [0x4a] = 0x38, [0x58] = 0x39, [0x05] = 0x3a, [0x06] = 0x3b,
    [0x04] = 0x3c, [0x0c] = 0x3d, [0x03] = 0x3e, [0x0b] = 0x3f, [0x83] = 0x40,
    [0x0a] = 0x41, [0x01] = 0x42, [0x09] = 0x43, [0x78] = 0x44, [0x07] = 0x45,
    [0x7e] = 0x47, [0x77] = 0x53, [0x7c] = 0x55, [0x7b] = 0x56, [0x79] = 0x57,
    [0x69] = 0x59, [0x72] = 0x5a, [0x7a] = 0x5b, [0x6b] = 0x5c, [0x73] = 0x5d,
    [0x74] = 0x5e, [0x6c] = 0x5f, [0x75] = 0x60, [0x7d] = 0x61, [0x70] = 0x62,
    [0x71] = 0x63, [0x61] = 0x64, [0x0f] = 0x67,

    [0x14] = 0xe0 - 0xe0 - HID_REPORT_OFFSET,
	[0x12] = 0xe1 - 0xe0 - HID_REPORT_OFFSET,
	[0x11] = 0xe2 - 0xe0 - HID_REPORT_OFFSET,
	[0x59] = 0xe5 - 0xe0 - HID_REPORT_OFFSET,
};

extern uint32_t led_state;
extern uint8_t kbd_report[32];
extern uint8_t kbd_change;

void recieve_handler();
void send_handler();
void (*ps2_byte_handler)(void) = recieve_handler;

void timeout_setup() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

	TIM5->CR1 = TIM_CR1_OPM;
	TIM5->PSC = 30000;//change
	TIM5->ARR = 1;//change


	TIM5->DIER |= TIM_DIER_UIE;
	TIM5->EGR = TIM_EGR_UG;
	
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_SetPriority(TIM5_IRQn,0);
}


enum {
	START,
	DATA,
	PARITY,
	STOP,
	ACK,
} ps2_state = START;
uint8_t ps2dat = 0;
uint8_t datcnt = 0;

void ps2_enable(){
	EXTI->IMR |= EXTI_IMR_MR10; //channel 10

	ps2_state = START;
	AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI10_Msk;
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PC;
	EXTI->RTSR &= ~EXTI_RTSR_RT10;
	EXTI->FTSR |= EXTI_FTSR_FT10; //read ps2 byte on falling edge

	//pc10 input
	GPIOC->CRH &= ~(GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk);
	GPIOC->CRH |= (0b01<<GPIO_CRH_CNF10_Pos);
	//pa15 input
	GPIOA->CRH &= ~(GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk);
	GPIOA->CRH |= (0b01<<GPIO_CRH_CNF15_Pos);

	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn,0);
	timeout_setup();
	
#ifdef PS2MOUSE
	ps2dat = 0xe4;
	send_byte();
#endif
}

void ps2_mouse_byte() { //send F4 to enable mouse
	static enum {
		BUTTON,
		XMVMT,
		YMVMT,
		CMD,
	} status = BUTTON;
	/* led_state &= ~0xff; */
	/* led_state |= ps2dat; */
	/* led_state = (led_state + 0x100) & 0xffff; */
}

void ps2_byte_rcvd() {
	led_state &= 0xffff0000;
	led_state |= ps2dat;
	static enum { NORM, CLR, ESC, CLRESC, } byte_mode = NORM;
	switch (ps2dat) {
	case 0xf0:
		if(byte_mode == ESC)
			byte_mode = CLRESC;
		else
			byte_mode = CLR;
		return;
	case 0xe0:
		if(byte_mode == CLR)
			byte_mode = CLRESC;
		else
			byte_mode = ESC;
		return;
	case 0xaa:
	case 0xfa:
	case 0xee:
	case 0xfe:
	case 0x00:
	case 0xff:
		byte_mode = NORM;
		return;
	default:
		break;
	}
	
	uint8_t position;
	uint8_t offset;
	uint32_t msk;
	switch (byte_mode) {
	case NORM:
		position = normps2[ps2dat] + HID_REPORT_OFFSET;
		offset = (position >> 3);
		msk = 1<<(position & 0x07);
		kbd_report[offset] |= msk;
		break;
	case CLR:
		position = normps2[ps2dat] + HID_REPORT_OFFSET;
		offset = (position >> 3);
		msk = 1<<(position & 0x07);
		kbd_report[offset] &= ~msk;
		break;
	case ESC:
		position = escps2[ps2dat] + HID_REPORT_OFFSET;
		offset = (position >> 3);
		msk = 1<<(position & 0x07);
		kbd_report[offset] |= msk;
		break;
	case CLRESC:
		position = escps2[ps2dat] + HID_REPORT_OFFSET;
		offset = (position >> 3);
		msk = 1<<(position & 0x07);
		kbd_report[offset] &= ~msk;
		break;
	}
	byte_mode = NORM;
	usb_hid_send_report();
	if(ps2dat == 0b01111110)
		spi_sd_init();
}


inline void start_timer(){
	TIM5->CR1 |= TIM_CR1_CEN;
}

inline void stop_timer(){
	TIM5->CR1 &= ~TIM_CR1_CEN;
}

uint8_t timeout_send = 0;
void TIM5_IRQHandler() {
	/* if (TIM5->SR & TIM_SR_UIF) */
	ps2_state = START;
	led_state |= 0x100;
	if(timeout_send)
		send_handler();
	else {
		/* EXTI->RTSR &= ~EXTI_RTSR_RT10; */
		/* EXTI->FTSR |= EXTI_FTSR_FT10; // read ps2 byte on falling edge */

		/* // pc10 input */
		/* GPIOC->CRH &= ~(GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk); */
		/* GPIOC->CRH |= (0b01 << GPIO_CRH_CNF10_Pos); */
		/* // pa15 input */
		/* GPIOA->CRH &= ~(GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk); */
		/* GPIOA->CRH |= (0b01 << GPIO_CRH_CNF15_Pos); */
	}
	TIM5->SR &= ~TIM_SR_UIF;
}

void send_byte() {
	//pc10 out
	GPIOC->CRH &= ~(GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk);
	GPIOC->CRH |= (0b01<<GPIO_CRH_CNF10_Pos) | (0b10 << GPIO_CRH_MODE10_Pos);
	GPIOC->ODR |= GPIO_ODR_ODR15;
	//pa15 out
	GPIOA->CRH &= ~(GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk);
	GPIOA->CRH |= (0b01<<GPIO_CRH_CNF15_Pos) | (0b10 << GPIO_CRH_MODE15_Pos);
	GPIOA->ODR &= ~(GPIO_ODR_ODR15);

	EXTI->FTSR &= ~EXTI_FTSR_FT10;
	EXTI->RTSR |= EXTI_RTSR_RT10;
	timeout_send = 1;
	start_timer();
	ps2_byte_handler = send_handler;
}

void send_handler() {
	timeout_send = 0;
	static uint32_t parity = 0;
	switch (ps2_state) {
	case START:
		datcnt = 0;
		parity = 0;
		GPIOA->ODR &= ~GPIO_ODR_ODR15; // send 0
		GPIOC->CRH &= ~(GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk); //clk in, and release clock
		GPIOC->CRH |= (0b01<<GPIO_CRH_CNF10_Pos);
		ps2_state = DATA;
		break;
	case DATA:
		parity ^= (ps2dat >> 7);
		GPIOA->ODR &= ~GPIO_ODR_ODR15;
		GPIOA->ODR |= ((ps2dat >> 7) << GPIO_IDR_IDR15_Pos);
		if(++datcnt == 8) ps2_state = PARITY;
		ps2dat <<= 1;
		break;
	case PARITY:
		ps2_state = STOP;
		GPIOA->ODR &= ~GPIO_ODR_ODR15;
		GPIOA->ODR |= (parity << GPIO_IDR_IDR15_Pos);
		break;
	case STOP:
		ps2_state = ACK;
		GPIOA->ODR &= ~GPIO_ODR_ODR15;
		break;
	case ACK:
		ps2_state = START;
		GPIOA->CRH &= ~(GPIO_CRH_MODE15_Msk | GPIO_CRH_CNF15_Msk);
		GPIOA->CRH |= (0b01<<GPIO_CRH_CNF15_Pos);
		EXTI->RTSR &= ~EXTI_RTSR_RT10;
		EXTI->FTSR |= EXTI_FTSR_FT10;
		ps2_byte_handler = recieve_handler;
	}
}

void recieve_handler() {
	uint8_t b = ((uint32_t)GPIOA->IDR & GPIO_IDR_IDR15) >> GPIO_IDR_IDR15_Pos; //state of data line
	switch (ps2_state) {
	case START:
		ps2_state = DATA;
		datcnt = 0;
		break;
	case DATA:
		ps2dat = (ps2dat >> 1) | (b<<7);
		if(++datcnt == 8) ps2_state = PARITY;
		break;
	case PARITY:
		ps2_state = STOP;
		break;
	case STOP:
		ps2_state = START;
#ifdef PS2MOUSE
		ps2_mouse_byte();
#else
		ps2_byte_rcvd();
#endif
		break;
	case ACK:
		ps2_state = START;
		break;
	}
}

void EXTI15_10_IRQHandler() { //add timeout to switch to start;
	EXTI->PR |= EXTI_PR_PR10;
	stop_timer();
	recieve_handler();
	start_timer();
}
