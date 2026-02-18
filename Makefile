.PHONY: flash clean

PARAMS = -mcpu=cortex-m3 -std=gnu11 -DSTM32 -DSTM32F105RBTx -DSTM32F1 -c -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -ggdb3 --specs=nano.specs -mfloat-abi=soft -mthumb

usb.o: usb.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

usb_scsi.o: usb_scsi.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

usb_audio.o: usb_audio.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

usb_hid.o: usb_hid.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

main.o: main.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

misc.o: misc.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

ps2.o: ps2.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

flash.o: flash.c
	arm-none-eabi-gcc $^ ${PARAMS} -o $@

main.elf: main.o misc.o usb.o usb_scsi.o usb_audio.o usb_hid.o ps2.o flash.o STM32F105RBTX_FLASH.ld startup_stm32f105rbtx.s
	arm-none-eabi-gcc -o $@ main.o usb.o misc.o usb_scsi.o usb_audio.o usb_hid.o ps2.o flash.o startup_stm32f105rbtx.s  -mcpu=cortex-m3 -T"STM32F105RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="f105_test.map" -Wl,--gc-sections -static --specs=rdimon.specs -lrdimon -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group -ggdb3

main.bin: main.elf
	 arm-none-eabi-objcopy -O binary main.elf main.bin

flash: main.bin
	st-flash write main.bin 0x08000000

clean:
	rm *.d *.o *.su
