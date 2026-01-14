#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"

setup_packet_t setup = {0};
uint8_t address_pending = 0;
uint16_t address;
#define HID_REPORT_SIZE 46
uint8_t l = 0;

enum  {
	idle,
	stup,
	zlp_device,
	data_out,
	zlp_host,
} ep0_state;

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
    .usb_configuration_descriptor =
        {
            .bLength = 9,
            .bDescriptorType = USB_DESCRIPTOR_CONFIGURATION,
            .wTotalLength = 9 + 9 + 7 + 9, // TODO
            .bNumInterfaces = 1,
            .bConfigurationValue = 1,
            .iConfiguration = 0, // index of string descriptor
            .bmAttributes = 0b10000000,
            .bMaxPower = 50, // in 2mA units
        },
    .usb_interface_descriptor =
        {
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
    .usb_HID_descriptor =
	{
		.bLength = 9,
		.bDescriptorType = 0x21,
		.bcdHid = 0x0111,
		.bCountryCode = 0x00,
		.bNumDescriptors = 0x01,
		.bHidDescriptorType = 0x22,
		.wDescriptorLength = HID_REPORT_SIZE,
	},
	.usb_endpoint1_descriptor =
	{
		.bLength = 7,
		.bDescriptorType = 0x05,
		.bEndpointAddress = 0b10000001,
		.bmAttributes = 0b00000011,
		.wMaxPacketSize = 8,
		.bInterval = 1,
	},
};


uint8_t hid_report[HID_REPORT_SIZE] = {
	0x05, 0x01, //usage page
	0x09, 0x06, //generic keyboard
	0xa1, 0x01, //collection
	0x05, 0x07, //usage page
	0x19, 0xe0, //usage min
	0x29, 0xe7,
	0x15, 0x00, //log min
	0x25, 0x01,
	0x95, 0x08,
	0x75, 0x01, //rep size
	0x81, 0x02,
	0x95, 0x01,
	0x75, 0x08,
	0x81, 0x01,
	0x05, 0x07,
	0x19, 0x00,
	0x29, 0xff,
	0x15, 0x00,
	0x26, 0xff, 0x00,
	0x95, 0x06,
	0x75, 0x08,
	0x81, 0x00,
	0xc0
};


void usbRawWrite(volatile uint32_t* fifo, void* data, uint8_t len) {
  uint32_t fifoWord;
  uint32_t *buffer = (uint32_t *)data;
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


uint8_t ll = 0;
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
	__asm("nop");
}


void read_ep_fifo(void* buffer, uint8_t ep, uint8_t len) {
	uint32_t* b = (uint32_t*) buffer;
	volatile uint32_t* fifo = usbEpFifo(ep);

	for (uint8_t idx = 0; idx < len; idx++, buffer++) {
        *b = *(volatile uint32_t*)fifo;
    }
}

inline void usb_stall(uint8_t ep) { //TODO: add dir!
	USB_OTG_INEndpointTypeDef* epin = usbEpin(ep);
	USB_OTG_OUTEndpointTypeDef* epout = usbEpout(ep);
	epin->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	epout->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
}

void ep_in_enable(uint8_t epn, uint8_t txnum, uint8_t eptype) {
	USB_OTG_INEndpointTypeDef* ep = usbEpin(epn);
	ep->DIEPCTL |= ((txnum & 0x03) << USB_OTG_DIEPCTL_TXFNUM_Pos);
	ep->DIEPCTL |= (eptype << USB_OTG_DIEPCTL_EPTYP_Pos);
	ep->DIEPCTL |= 8;
	ep->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;
	/* – Endpoint start data toggle (for interrupt and bulk endpoints) */
	ep->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
	//	ep->DIEPCTL |= USB_OTG_DIEPCTL_EPENA;// | USB_OTG_DIEPCTL_SNAK;
	USB_OTG_FS_DEV->DAINTMSK |= (1<<epn);
	
}

void set_ep0_idle() {
	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
		(3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
		(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
		(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
		USB_OTG_DOEPCTL_EPENA |
		USB_OTG_DOEPCTL_CNAK;
	ep0_state = idle;
}

void set_ep0_zlpdev() {
	USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
		(0 << USB_OTG_DIEPTSIZ_XFRSIZ_Pos);
    USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

	ep0_state = zlp_device;
}

void set_ep0_zlphost() {
	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
		(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);

	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
		USB_OTG_DOEPCTL_EPENA |
		USB_OTG_DOEPCTL_CNAK;

	ep0_state = zlp_host;
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
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;// | USB_OTG_GAHBCFG_PTXFELVL | USB_OTG_GAHBCFG_TXFELVL; //fifos completely empty

    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_TOCAL_0 | USB_OTG_GUSBCFG_TOCAL_1 | USB_OTG_GUSBCFG_TOCAL_2; // add clock cycles inter-packet timeout 
    USB_OTG_FS->GUSBCFG |= (0x6 << USB_OTG_GUSBCFG_TRDT_Pos); //turnaround time
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD; //force device mode
	for(uint32_t i=100000; i>0; i--)
		__asm("nop");
	//  	wait(72000,25);

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST |  USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT;//USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_OTGINT | // un-mask global interrupt and mode mismatch interrupt
	USB_OTG_FS->GINTSTS = 0xffffffff; //zero all interrupts

	//set up interrupts
	NVIC_SetPriority(OTG_FS_IRQn, 0);
    NVIC_EnableIRQ(OTG_FS_IRQn);
}


/** Initialize the peripheral as device (not as host) */
void usb_device_init() {
    USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_DSPD_0 | USB_OTG_DCFG_DSPD_1; // set device speed to full-speed
    USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_NZLSOHSK; // send a STALL packet on non-zero-length status OUT transaction (default USB behavior)

    USB_OTG_FS->GINTMSK |=  USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_GINAKEFFM; // unmask interrupts
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN; // enable V_BUS sensing "B"

	set_ep0_idle();
}

uint8_t r;
void usb_reset_handler() {
	if (++r == 2) {
		__asm("nop");
	}
    USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL |= USB_OTG_DOEPCTL_SNAK  ; // set the NAK bit for end-point 0
	 
    // interrupt un-masking
	USB_OTG_FS_DEV->DAINTMSK |= 0x10001;//TODO add ep1
    USB_OTG_FS_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_OTEPDM | USB_OTG_DOEPMSK_OTEPSPRM;
    USB_OTG_FS_DEV->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;// | USB_OTG_DIEPMSK_ITTXFEMSK;// | USB_OTG_DIEPINT_TXFE;// | (1<<3);

    USB_OTG_FS->GRXFSIZ = RX_FIFO_DEPTH_IN_WORDS; 
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_DEPTH_IN_WORDS << USB_OTG_TX0FD_Pos) | RX_FIFO_DEPTH_IN_WORDS;

	USB_OTG_INEndpointTypeDef* ep1 = usbEpin(1);
	USB_OTG_FS->DIEPTXF[1] = ((TX_FIFO_DEPTH_IN_WORDS) << USB_OTG_TX0FD_Pos) | (TX_FIFO_DEPTH_IN_WORDS+RX_FIFO_DEPTH_IN_WORDS);

	//flush fifos
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);

	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0b10000 << USB_OTG_GRSTCTL_TXFNUM_Pos);
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);

	USB_OTG_FS_DEV->DCTL |= USB_OTG_DCTL_CGONAK | USB_OTG_DCTL_CGINAK;
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
	set_ep0_idle();
}

uint8_t can_write = 0;

void write_report(void* buf){
	if (can_write) {
		usbWrite(1, buf, 8);
	}
}

void setup_device_to_host() {
    if (setup.bRequest == BREQUEST_GET_DESCRIPTOR) {
        switch ((setup.wValue & 0xff00) >> 8) { // extract descriptor type
		case USB_DESCRIPTOR_DEVICE:
			if (setup.wLength > 18) {
				usbWrite(0, &usb_device_descriptor, 18);
			} else {
				usbWrite(0, &usb_device_descriptor, setup.wLength);
			}
			ep0_state = data_out;
			break;
		case USB_DESCRIPTOR_CONFIGURATION:
			if (setup.wLength == 9) { // first time
				usbWrite(0, &configuration_descriptor, setup.wLength);
			}
			else if (setup.wLength ==
					 configuration_descriptor.usb_configuration_descriptor.wTotalLength) {
				usbWrite(0, &configuration_descriptor, setup.wLength);
			}
			ep0_state = data_out;
			break;
		case USB_DESCRIPTOR_HID_REPORT:
			usbWrite(0,&hid_report, HID_REPORT_SIZE);
			ep0_state = data_out;
			break;
		case USB_DESCRIPTOR_STRING:
		case USB_DESCRIPTOR_INTERFACE:
		case USB_DESCRIPTOR_ENDPOINT:
		case USB_DESCRIPTOR_QUALIFIER:
		default:
			usb_stall(0);
			break;
        }


    } else if (setup.bRequest == BREQUEST_GET_STATUS) {
		uint16_t status = USB_STATUS_RESPONSE;
		usbWrite(0, &status, 2);
		ep0_state = data_out;
    }
}

void setup_host_to_device() {
    if (setup.bRequest == BREQUEST_SET_ADDRESS) {
		set_ep0_zlpdev();
		address = setup.wValue;
		address_pending = 1;
    }
	else if(setup.bRequest == BREQUEST_SET_CONFIGURATION) {
		set_ep0_zlpdev();
		ep_in_enable(1, 1, EP_interrupt);
		can_write = 1;
    } else if (setup.bRequest == BREQUEST_SET_IDLE) {
		usb_stall(0);
		set_ep0_idle();
    }
}


void usb_handle_setup_packet() {
    uint32_t* buffer = (uint32_t*) &setup;

    for (uint16_t idx = 0; idx < 2; idx++, buffer++) {
        *buffer = *USB_OTG_FS_ENDPOINT0_FIFO;
    }
    if (setup.bmRequestType & 0x80) {
        setup_device_to_host();
    } else {
        setup_host_to_device();
    }
}

uint8_t rr=0;

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
		ep0_state = stup;
		usb_handle_setup_packet();
		break;
	case 0b0100:// PCKT_STS_SETUP_COMPLETE:
		break;
	case 0b0010: // out packet recieved
		//		set_ep0_idle();
		break;
	case 0b0011: //out transfer complete
		break;
	default:
		break;
    }
}


void usb_interrupt_out_handler() {
	uint32_t flags = USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT;
	if (flags & USB_OTG_DOEPINT_STUP) {
		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = USB_OTG_DOEPINT_STUP;
		// na razie nic bo nie akceptuję kilka setup pod rząd!
	}
    if (flags & USB_OTG_DOEPINT_XFRC) {
        USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = USB_OTG_DOEPINT_XFRC;
		if(ep0_state == zlp_host)
			set_ep0_idle();
    }

    if (flags & (USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR)) {
        USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = flags & (USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR);
		//error!
    }
	if (address_pending) {
		/* USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT = */
		/* 	USB_OTG_DIEPINT_XFRC; // clear interrupt */
		USB_OTG_FS_DEV->DCFG &= ~USB_OTG_DCFG_DAD;
		USB_OTG_FS_DEV->DCFG |= (address << USB_OTG_DCFG_DAD_Pos);
		address_pending = 0;
		set_ep0_zlphost();
	} 

}

void usb_interrupt_in_handler() {
	
	if(USB_OTG_FS_DEV->DAINT & 0x01){
          if (USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT & USB_OTG_DIEPINT_XFRC) {
            /* // transfer finished interrupt */
            USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT =
                USB_OTG_DIEPINT_XFRC;
            if (ep0_state == data_out)
              set_ep0_zlphost();
          }
	}
	if (USB_OTG_FS_DEV->DAINT & 0x02) {
		USB_OTG_INEndpointTypeDef* ep = usbEpin(1);

		if (ep->DIEPINT & USB_OTG_DIEPINT_XFRC) {
			ep->DIEPINT = USB_OTG_DIEPINT_XFRC;
			//			light(0x01);
		}
		if (ep->DIEPINT & USB_OTG_DIEPINT_ITTXFE) {
			ep->DIEPINT = USB_OTG_DIEPINT_ITTXFE;
			//			light(++l);
			/* write_report(); */
		}
	}
}


void OTG_FS_IRQHandler() {
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
		
		if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
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
}
