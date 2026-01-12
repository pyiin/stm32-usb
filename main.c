#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"

uint8_t l = 0;

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

uint8_t lightadd(uint8_t b) {
	GPIOA->BSRR = (b&0x01)<<(15+16);
	GPIOC->BSRR = (b&0x0e)<<(9+16);
	GPIOB->BSRR = (b&0xe0)>>(2+16);
	GPIOD->BSRR = (b&0x10)>>(2+16);
	return b;
}


uint8_t light(uint8_t b) {
	b = ~b;
	GPIOA->ODR = (b&0x01)<<15;
	GPIOC->ODR = (b&0x0e)<<9;
	GPIOB->ODR = (b&0xe0)>>2;
	GPIOD->ODR = (b&0x10)>>2;
	return b;
}

uint8_t t;
void SysTick_Handler() {
	t++;
	return;
}

uint8_t address_pending = 0;
uint16_t address;
setup_packet_t setup = {0};


device_descriptor_t usb_device_descriptor = {
    .bLength = 18,
    .bDescriptorType = USB_DESCRIPTOR_DEVICE,
    .bcdUSB = USB_BCD_2,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x413c,//0x0483,
    .idProduct = 0x2010,//0x5740,
    .bcdDevice = 0x0100,
    .iManufacturer = 0,
    .iProduct = 0,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};



full_configuration_descriptor_t configuration_descriptor = {
    .usb_configuration_descriptor = {
            .bLength = 9,
            .bDescriptorType = USB_DESCRIPTOR_CONFIGURATION,
            .wTotalLength = 9+9+7, // TODO
            .bNumInterfaces = 1,
            .bConfigurationValue = 1,
            .iConfiguration = 0, // index of string descriptor
            .bmAttributes = 0b10000000,
            .bMaxPower = 50, // in 2mA units
        },
    .usb_interface_descriptor = {
            .bLength = 9,
            .bDescriptorType = 0x04,
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = USB_HID_CLASS,
            .bInterfaceSubClass = 0x01,
            .bInterfaceProtocol = 0x01,
            .iInterface = 0,
        },
    .usb_endpoint1_descriptor = {
        .bLength = 7,
        .bDescriptorType = 0x05,
        .bEndpointAddress = 0b10000001,
        .bmAttributes = 0b00000011,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    },
};

#define RX_FIFO_DEPTH_IN_WORDS 64
#define TX_FIFO_DEPTH_IN_WORDS 64

void usb_reset_handler() {
    USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL |= USB_OTG_DOEPCTL_SNAK  ; // set the NAK bit for end-point 0
	 
    // interrupt un-masking
	USB_OTG_FS_DEV->DAINTMSK |= 0x10001;//TODO add ep1
    USB_OTG_FS_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_OTEPDM | USB_OTG_DOEPMSK_OTEPSPRM;
    USB_OTG_FS_DEV->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;// | USB_OTG_DIEPINT_TXFE;// | (1<<3);

    USB_OTG_FS->GRXFSIZ = RX_FIFO_DEPTH_IN_WORDS; // size of RxFIFO (arbitrarily chosen)
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_DEPTH_IN_WORDS << USB_OTG_TX0FD_Pos) | TX_FIFO_DEPTH_IN_WORDS; // size of TxFIFO, address of txFifo
	//    USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos); // receive 1 setup packet, 1 packet, 8 bytes
	//flush fifos
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);

	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0 << USB_OTG_GRSTCTL_TXFNUM_Pos);
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);

	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
    (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
    (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
    (8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
    USB_OTG_DOEPCTL_EPENA |
    USB_OTG_DOEPCTL_CNAK;
}

void usb_enum_done_handler() {
    uint32_t enum_speed = (USB_OTG_FS_DEV->DSTS & 0x6) >> 1; // read the enumerated speed

    if (enum_speed == 3) {
		USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPCTL =
			(USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPCTL & ~USB_OTG_DIEPCTL_MPSIZ_Msk) |
			(64 << USB_OTG_DIEPCTL_MPSIZ_Pos);
    } else {
        // error probably
        while(1);
    }
	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
		(1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
		(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
		(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
		USB_OTG_DOEPCTL_EPENA |
		USB_OTG_DOEPCTL_CNAK;
}


void set_address_pending(uint16_t new_address) {
    address = new_address;
    address_pending = 1;
	USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
                                            (0 << USB_OTG_DIEPTSIZ_XFRSIZ_Pos);
    USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
}



void setup_device_to_host() {
    if (setup.bRequest == BREQUEST_GET_DESCRIPTOR) {
        switch ((setup.wValue & 0xf00) >> 8) { // extract descriptor type
		case USB_DESCRIPTOR_DEVICE:
			if (setup.wLength > 18) {
				usbWrite(0, &usb_device_descriptor, 18);
			} else {
				usbWrite(0, &usb_device_descriptor, setup.wLength);
			}
			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos); // receive 1 packet, 0 bytes
			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
			break;
		case USB_DESCRIPTOR_CONFIGURATION:
			if (setup.wLength == 9) { // first time
				usbWrite(0, &configuration_descriptor, setup.wLength);
			}
			else if (setup.wLength ==
					 configuration_descriptor.usb_configuration_descriptor.wTotalLength) {
				usbWrite(0, &configuration_descriptor, setup.wLength);
			}
			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos); // receive 1 packet, 0 bytes
			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
			break;
		case USB_DESCRIPTOR_STRING:
			break;
		case USB_DESCRIPTOR_INTERFACE:
			break;
		case USB_DESCRIPTOR_ENDPOINT:
			break;
		case USB_DESCRIPTOR_QUALIFIER:
			usb_stall(0);
			break;
		default:
			while (1);
        }

    } else if (setup.bRequest == BREQUEST_GET_STATUS) {
        uint16_t status = USB_STATUS_RESPONSE;
        usbWrite(0, &status, 2);
    }

}

void setup_host_to_device() {
    if (setup.bRequest == BREQUEST_SET_ADDRESS) {
		usbWrite(0, 0, 0);
        set_address_pending(setup.wValue);
		light(++l);
    }
	else if(setup.bRequest == BREQUEST_SET_CONFIGURATION) {
		usbWrite(0, 0, 0);
    }
}

void usb_handle_setup_packet() {
    if (setup.bmRequestType & 0x80) {
        setup_device_to_host();
    } else {
        setup_host_to_device();
    }
}

void usb_interrupt_out_handler() {
	uint32_t flags = USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT;
	if (flags & USB_OTG_DOEPINT_STUP) {
		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = USB_OTG_DOEPINT_STUP;
	}
    if (flags & USB_OTG_DOEPINT_XFRC) {
        USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = USB_OTG_DOEPINT_XFRC;
    }

    if (flags & (USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR)) {
        USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = flags & (USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR);
    }

}

void usb_interrupt_in_handler() {
	//	if (USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT & USB_OTG_DIEPINT_) {
	//	}
    if (USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT & USB_OTG_DIEPINT_XFRC) {
        // transfer finished interrupt
		/* USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT = */
		/* 	USB_OTG_DIEPINT_XFRC; // clear interrupt */
		if (address_pending) {
			USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT =
				USB_OTG_DIEPINT_XFRC; // clear interrupt

		
			USB_OTG_FS_DEV->DCFG &= ~USB_OTG_DCFG_DAD;
			USB_OTG_FS_DEV->DCFG |= (address << USB_OTG_DCFG_DAD_Pos);
			address_pending = 0;
        } 
    }
}

void usb_vbus_handler() {
	
}


void read_setup() {
    uint32_t* buffer = (uint32_t*) &setup;

    for (uint16_t idx = 0; idx < 2; idx++, buffer++) {
        *buffer = *USB_OTG_FS_ENDPOINT0_FIFO;
    }
}

void usb_read_data() {
    uint32_t otg_status = USB_OTG_FS->GRXSTSP;
    uint32_t endpoint_number = otg_status & USB_OTG_GRXSTSP_EPNUM_Msk;
    uint32_t byte_count = (otg_status & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;
    /* uint32_t data_pid = (otg_status & USB_OTG_GRXSTSP_DPID_Msk) >> USB_OTG_GRXSTSP_DPID_Pos; */
    uint32_t packet_status = (otg_status & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
    switch (packet_status) {
	case 0b0110:// PCKT_STS_SETUP:
		if (endpoint_number != 0 || byte_count != 8) {
			break;//while(1);
		}
		read_setup();
		usb_handle_setup_packet();
		break;
	case 0b0100:// PCKT_STS_SETUP_COMPLETE:
		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
			(1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
			(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
			(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
			USB_OTG_DOEPCTL_EPENA |
			USB_OTG_DOEPCTL_CNAK;
		break;
	case 0b0010:
		for (uint32_t i = 0; i < (byte_count + 3) / 4; i++) {
			(void)*USB_OTG_FS_ENDPOINT0_FIFO;
		}

		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
			(1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
			(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
			(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
			USB_OTG_DOEPCTL_EPENA |
			USB_OTG_DOEPCTL_CNAK;
		break;
    }
}

void usb_otgint_handler() {
	USB_OTG_FS->GOTGINT = 0xffffffff;
}

void OTG_FS_IRQHandler() {
	uint32_t gintsts = USB_OTG_FS->GINTSTS;
	uint32_t gintmsk = USB_OTG_FS->GINTMSK;

	if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) {
        USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBRST;
        usb_reset_handler();
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) {
        USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        usb_enum_done_handler();
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
		USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
		
		while (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
			usb_read_data();
		}
    
		USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT) {
        usb_interrupt_out_handler();
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT) {
        usb_interrupt_in_handler();
    }
	if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_MMIS) {
        USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_MMIS;
    }
	if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OTGINT) {
		usb_otgint_handler();
    }
}

void wait(uint32_t w, uint32_t n){
	SysTick->VAL = 0;
	SysTick->LOAD = (w-1) & 0x00ffffff;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
	t = 0;
	while(t < n)
		__asm("nop");
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}


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
	clock_setup();
	light(0);
	usb_core_init();
	usb_device_init();
	uint8_t ll =0;
	while(1){
		/* wait(7200000, 10);// 1 second? */
		/* light(0b10101010); */
		/* wait(7200000, 10); */
		/* light(0b01010101); */
		wait(7200000, 1);
		light(ll++);
	}
}
