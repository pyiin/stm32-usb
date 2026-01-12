#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"

void usbRawWrite(volatile uint32_t* fifo, void* data, uint8_t len) {
    uint32_t fifoWord;
    uint32_t* buffer = (uint32_t*)data;
    uint8_t remains = len;
    for (uint8_t idx = 0; idx < len; idx += 4, remains -= 4, buffer++) {
        switch (remains) {
            case 0:
                break;
            case 1:
                fifoWord = *buffer & 0xFF;
                *fifo = fifoWord;
                break;
            case 2:
                fifoWord = *buffer & 0xFFFF;
                *fifo = fifoWord;
                break;
            case 3:
                fifoWord = *buffer & 0xFFFFFF;
                *fifo = fifoWord;
                break;
            default:
                *fifo = *buffer;
                break;
        }
    }
}


inline USB_OTG_INEndpointTypeDef* usbEpin(uint8_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep << 5));
}

inline USB_OTG_OUTEndpointTypeDef* usbEpout(uint8_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep << 5));
}

inline uint32_t* usbEpFifo(uint8_t ep) {
    return (uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep << 12));
}

void usbWrite(uint8_t ep, void* data, uint8_t len) {
    USB_OTG_INEndpointTypeDef* endpoint = usbEpin(ep);
    volatile uint32_t* fifo = usbEpFifo(ep);

    uint16_t wordLen = (len + 3) >> 2;
    if (wordLen > endpoint->DTXFSTS) {
        return;
    }
    if ((ep != 0) && (endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)) {
        return;
    }

    endpoint->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    endpoint->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    usbRawWrite(fifo, data, len);
}

void read_ep_fifo(void* buffer, uint8_t ep, uint8_t len) {
	uint32_t* b = (uint32_t*) buffer;
	USB_OTG_INEndpointTypeDef* fifo = usbEpin(ep);

	for (uint8_t idx = 0; idx < len; idx++, buffer++) {
        *b = *(volatile uint32_t*)fifo;
    }
}

inline void usb_stall(uint8_t ep) {
	USB_OTG_INEndpointTypeDef* epin = usbEpin(ep);
	USB_OTG_OUTEndpointTypeDef* epout = usbEpout(ep);
	epin->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	epout->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
}


//init functions

void clock_setup(){
	RCC->CR |= RCC_CR_HSEON; //enable hse
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	//setup prediv12 and prediv1scr
	RCC->CFGR |= (0b0111<<RCC_CFGR_PLLMULL_Pos);
	RCC->CFGR |= (0b0<<RCC_CFGR_OTGFSPRE_Pos);
	RCC->CFGR |= RCC_CFGR_PLLSRC;

	//enable pll
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	FLASH->ACR = FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
	//switch to pll
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// USB CLK
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
}

void usb_core_init() {
	//make sure to set up vsense pin
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN; // enable USB transceiver
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT | USB_OTG_GAHBCFG_PTXFELVL | USB_OTG_GAHBCFG_TXFELVL; //fifos completely empty

    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_TOCAL_0 | USB_OTG_GUSBCFG_TOCAL_1 | USB_OTG_GUSBCFG_TOCAL_2; // add clock cycles inter-packet timeout 
    USB_OTG_FS->GUSBCFG |= (0x6 << USB_OTG_GUSBCFG_TRDT_Pos); //turnaround time
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD; //force device mode
	wait(72000,25);

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT; // un-mask global interrupt and mode mismatch interrupt
	USB_OTG_FS->GINTSTS = 0xffffffff; //zero all interrupts

	//set up interrupts
	NVIC_SetPriority(OTG_FS_IRQn, 0);
    NVIC_EnableIRQ(OTG_FS_IRQn);
}


/** Initialize the peripheral as device (not as host) */
void usb_device_init() {
    USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_DSPD_0 | USB_OTG_DCFG_DSPD_1; // set device speed to full-speed
    USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_NZLSOHSK; // send a STALL packet on non-zero-length status OUT transaction (default USB behavior)

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_RXFLVLM; // unmask interrupts
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN; // enable V_BUS sensing "B"
}
