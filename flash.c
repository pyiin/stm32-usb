#define FLASH_START (uint8_t*)0x8000000
#define PAGE_SIZE_pos 11
#include "stm32f1xx.h"
#include "misc.h"

inline
void flash_ready_wait(){
	while(FLASH->SR & FLASH_SR_BSY);
}

inline
void flash_unlock() {
	if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}

inline
void flash_lock() {
	FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase(uint16_t page) {
	if (FLASH->CR)
		flash_unlock();
	FLASH->CR |= FLASH_CR_PER;
	flash_ready_wait();
	FLASH->AR = (uint32_t)(FLASH_START + (page<<PAGE_SIZE_pos));
	FLASH->CR |= FLASH_CR_STRT;
	flash_ready_wait();
	FLASH->CR &= ~FLASH_CR_STRT;
	FLASH->CR &= ~FLASH_CR_PER;
}
