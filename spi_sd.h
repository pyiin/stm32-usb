void spi1_init();
uint8_t spi_sd_init();
uint8_t spi_sd_readblock(uint32_t blknum, void* blkbuf);
uint8_t spi_sd_writeblock(uint32_t blknum, void* blkbuf);
uint8_t spi_sd_readsize(uint32_t* capacity);
