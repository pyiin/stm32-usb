#include "stm32f1xx.h"

#define GPIOA_col (1<<7)
#define GPIOB_col ((1<<2) | (1<<1) | (1<<0))
#define GPIOC_col ((1<<5) | (1<<4))

void key_setup() {
	GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5 //row4
				  | GPIO_CRL_MODE6 | GPIO_CRL_CNF6 //row3
				  | GPIO_CRL_MODE7 | GPIO_CRL_CNF7); //col6
	GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 //col3
				  | GPIO_CRL_MODE1 | GPIO_CRL_CNF1//col2
				  | GPIO_CRL_MODE2 | GPIO_CRL_CNF2);//col1
	GPIOC->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4//col5
			      | GPIO_CRL_MODE5 | GPIO_CRL_CNF5);//col4
	GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10//row2
				  | GPIO_CRH_MODE11 | GPIO_CRH_CNF11);//row1
	
	GPIOA->CRL |= (0b11<<GPIO_CRL_MODE5_Pos) | (0b00<<GPIO_CRL_CNF5_Pos) //out
	        	| (0b11<<GPIO_CRL_MODE6_Pos) | (0b00<<GPIO_CRL_CNF6_Pos) //out
	        	| (0b00<<GPIO_CRL_MODE7_Pos) | (0b10<<GPIO_CRL_CNF7_Pos);//in
	GPIOB->CRL |= (0b00<<GPIO_CRL_MODE0_Pos) | (0b10<<GPIO_CRL_CNF0_Pos)//in
	        	| (0b00<<GPIO_CRL_MODE1_Pos) | (0b10<<GPIO_CRL_CNF1_Pos) //in
		        | (0b00<<GPIO_CRL_MODE2_Pos) | (0b10<<GPIO_CRL_CNF2_Pos);//in
	GPIOC->CRL |= (0b00<<GPIO_CRL_MODE4_Pos) | (0b10<<GPIO_CRL_CNF4_Pos)//in
		        | (0b00<<GPIO_CRL_MODE5_Pos) | (0b10<<GPIO_CRL_CNF5_Pos);//in
	GPIOB->CRH |= (0b11<<GPIO_CRH_MODE10_Pos) | (0b00<<GPIO_CRH_CNF10_Pos)//row
		        | (0b11<<GPIO_CRH_MODE11_Pos) | (0b00<<GPIO_CRH_CNF11_Pos);//row
	GPIOA->BSRR = GPIO_BSRR_BR7;
	GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1 | GPIO_BSRR_BR2;
	GPIOC->BSRR = GPIO_BSRR_BR4 | GPIO_BSRR_BR5;
}

uint8_t key_state[4];

inline void get_row(uint8_t row) {
	key_state[row]  = (GPIOA_col & GPIOA->IDR);
	key_state[row] |= (GPIOB_col & GPIOB->IDR);
	key_state[row] |= (GPIOC_col & GPIOC->IDR);
	//quite lucky that they do not intersect
}

void read_keys() {
	GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
	GPIOB->BSRR = GPIO_BSRR_BR10 | GPIO_BSRR_BR11;
	
	GPIOB->BSRR = GPIO_BSRR_BS11;
	get_row(0);
	GPIOB->BSRR = GPIO_BSRR_BR11;
	
	GPIOB->BSRR = GPIO_BSRR_BS10;
	get_row(1);
	GPIOB->BSRR = GPIO_BSRR_BR10;
	
	GPIOA->BSRR = GPIO_BSRR_BS6;
	get_row(2);
	GPIOA->BSRR = GPIO_BSRR_BR6;
	
	GPIOA->BSRR = GPIO_BSRR_BS5;
	get_row(3);
	GPIOA->BSRR = GPIO_BSRR_BR5;
}

uint8_t report_transl[32] = {
	[0*8+0] = 'e'-'a'+16,
	[0*8+1] = 'r'-'a'+16,
	[0*8+2] = 't'-'a'+16,
	[0*8+4] = 'q'-'a'+16,
	[0*8+5] = 'w'-'a'+16,
	[0*8+7] = 0x2b + 12,
	
	[1*8+0] = 'd'-'a'+16,
	[1*8+1] = 'f'-'a'+16,
	[1*8+2] = 'g'-'a'+16,
	[1*8+4] = 'a'-'a'+16,
	[1*8+5] = 's'-'a'+16,
	[1*8+7] = 1,
	
	[2*8+0] = 'c'-'a'+16,
	[2*8+1] = 'v'-'a'+16,
	[2*8+2] = 'b'-'a'+16,
	[2*8+4] = 'z'-'a'+16,
	[2*8+5] = 'x'-'a'+16,
	[2*8+7] =  0x4c+12,
	
	[3*8+0] = 3,
	[3*8+1] = 0,
	[3*8+2] = 44+12,//space
	[3*8+5] = 2,

	[0*8+3] = 255,	[0*8+6] = 255,
	[1*8+3] = 255,	[1*8+6] = 255,
	[2*8+3] = 255,	[2*8+6] = 255,
	[3*8+3] = 255,	[3*8+6] = 255,
	[3*8+4] = 255,	[3*8+7] = 255,
};

extern uint8_t kbd_report[32];
void key_to_report(){ //for now simple mapping, will need to set up layers later
	uint32_t* data = (uint32_t*)&key_state; //maybe union is better
	for(uint8_t i=0; i<32; i++){
		uint8_t whole = report_transl[i];
		if(whole==255) continue;
		uint8_t bit = whole & 0x07;
		uint8_t byte = (whole>>3);
		if(*data & (1<<i))
			kbd_report[byte] |= (1<<bit);
		else
			kbd_report[byte] &= ~(1<<bit);
	}
}
