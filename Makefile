main.o: main.c
	 arm-none-eabi-gcc main.c -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F105RBTx -DSTM32F1 -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"main.d" -MT"main.o" -mfloat-abi=soft -mthumb -o "main.o" --specs=nano.specs -ggdb3

main.elf: main.o STM32F105RBTX_FLASH.ld
	arm-none-eabi-gcc -o "main.elf" main.o startup_stm32f105rbtx.s  -mcpu=cortex-m3 -T"STM32F105RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="f105_test.map" -Wl,--gc-sections -static --specs=rdimon.specs -lrdimon -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group -ggdb3

main.bin: main.elf
	 arm-none-eabi-objcopy -O binary main.elf main.bin

flash: main.bin
	st-flash write main.bin 0x08000000
