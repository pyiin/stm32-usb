#include <stdint.h>

void flash_ready_wait();
void flash_unlock();
void flash_lock();
void flash_erase(uint16_t);
void flash_write_page(uint16_t pagenum, uint16_t* data);
