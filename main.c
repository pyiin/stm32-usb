#include <stdint.h>
#include "stm32f1xx.h"

uint8_t l = 0;

#define USB_OTG_FS_DEV    ((USB_OTG_DeviceTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_FS_DEV_ENDPOINT0_OUT     ((USB_OTG_OUTEndpointTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE))
#define USB_OTG_FS_DEV_ENDPOINT0_IN  ((USB_OTG_INEndpointTypeDef *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))
#define USB_OTG_FS_ENDPOINT0_FIFO (volatile uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE)

typedef struct setup_packet_t {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} setup_packet_t;

typedef struct device_descriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} device_descriptor_t;

typedef struct __attribute__((packed)) interface_descriptor_t {
    uint8_t bLength; //0x09
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} interface_descriptor_t;

typedef struct __attribute__((packed)) endpoint_descriptor_t {
    uint8_t bLength; //0x07
    uint8_t bDescriptorType; //0x05
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval; //interval for iso and int
} endpoint_descriptor_t;

typedef struct __attribute__((packed)) configuration_descriptor_t {
    uint8_t bLength; // 0x09
    uint8_t bDescriptorType; //0x02
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration; // index of string descriptor
    uint8_t bmAttributes;
    uint8_t bMaxPower; // in 2mA units
} configuration_descriptor_t;

typedef struct qualifier_descriptor_t {
    uint8_t bLength; // 
    uint8_t bDescriptorType; //0x06
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint8_t bNumConfigurations;
	uint8_t bReserved;
} qualifier_descriptor_t;

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

#define USB_DESCRIPTOR_DEVICE 0x01
#define USB_DESCRIPTOR_CONFIGURATION 0x02
#define USB_BCD_2 0x0200 //USB 2.0
#define USB_HID_CLASS 0x03
#define USB_CDC_ACM_SUBCLASS 0x00
#define USB_NO_SPECIFIC_PROTOCOL 0x00 //


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

typedef struct __attribute__((packed)) full_configuration_descriptor_t {
	configuration_descriptor_t usb_configuration_descriptor;
	interface_descriptor_t usb_interface_descriptor;
	endpoint_descriptor_t usb_endpoint1_descriptor;
} full_configuration_descriptor_t;

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
	USB_OTG_FS_DEV->DAINTMSK |= 0x10001;
    USB_OTG_FS_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_OTEPDM | USB_OTG_DOEPMSK_OTEPSPRM;
    USB_OTG_FS_DEV->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM | (1<<3);

    USB_OTG_FS->GRXFSIZ = RX_FIFO_DEPTH_IN_WORDS; // size of RxFIFO (arbitrarily chosen)
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_DEPTH_IN_WORDS << USB_OTG_TX0FD_Pos) | TX_FIFO_DEPTH_IN_WORDS; // size of TxFIFO, address of txFifo
	//    USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos); // receive 1 setup packet, 1 packet, 8 bytes

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

USB_OTG_INEndpointTypeDef* usbEpin(uint8_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep << 5));
}

USB_OTG_OUTEndpointTypeDef* usbEpout(uint8_t ep) {
    return (void*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep << 5));
}

uint32_t* usbEpFifo(uint8_t ep) {
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

void set_address(uint16_t new_address) {
    address = new_address;
    address_pending = 1;
	USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
                                            (0 << USB_OTG_DIEPTSIZ_XFRSIZ_Pos);
    USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
}

#define BREQUEST_GET_DESCRIPTOR 0x06
#define USB_DESCRIPTOR_CONFIGURATION 0x02
#define USB_DESCRIPTOR_STRING 0x03
#define USB_DESCRIPTOR_INTERFACE 0x04
#define USB_DESCRIPTOR_ENDPOINT 0x05
#define USB_DESCRIPTOR_QUALIFIER 0x06

#define BREQUEST_GET_STATUS 0x00
#define BREQUEST_SET_ADDRESS 0x05
#define BREQUEST_SET_CONFIGURATION 0x09

#define USB_STATUS_RESPONSE 0x0000

void usb_stall(uint8_t ep) {
	USB_OTG_INEndpointTypeDef* epin = usbEpin(ep);
	USB_OTG_OUTEndpointTypeDef* epout = usbEpout(ep);
	epin->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	epout->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
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
        set_address(setup.wValue);
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
		USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT |= USB_OTG_DOEPINT_STUP;
	}
    if (flags & USB_OTG_DOEPINT_XFRC) {
        USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = USB_OTG_DOEPINT_XFRC;
    }

    if (flags & (USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR)) {
        USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = flags & (USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR);
    }

}

void usb_interrupt_in_handler() {
    if (USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT & USB_OTG_DIEPINT_XFRC) {
        // transfer finished interrupt
        if (address_pending) {
			USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT |=
				USB_OTG_DIEPINT_XFRC; // clear interrupt

			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
				(1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
				(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
				(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);
			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL =
				USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;

			USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;
			
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
    uint32_t data_pid = (otg_status & USB_OTG_GRXSTSP_DPID_Msk) >> USB_OTG_GRXSTSP_DPID_Pos;
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
		/* USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ = */
		/* 	(1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | */
		/* 	(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | */
		/* 	(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos); */

		/* USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL = */
		/* 	USB_OTG_DOEPCTL_EPENA | */
		/* 	USB_OTG_DOEPCTL_CNAK; */
	default:
		break;
    }
}

void OTG_FS_IRQHandler() {
	if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) {
		//        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST;
    }
	if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST) {
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST;
        usb_reset_handler();
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) {
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
        usb_enum_done_handler();
    }
    while (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
        // USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_RXFLVL;
        usb_read_data();
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT) {
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_OEPINT;
        usb_interrupt_out_handler();
    }
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT) {
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_IEPINT;
        usb_interrupt_in_handler();
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

void clock_setup(){
	RCC->CR |= RCC_CR_HSEON; //enable hse
	while(!(RCC->CR & RCC_CR_HSERDY))
		light(0x00);
	light(0x01);
	
	//setup prediv12 and prediv1scr
	RCC->CFGR |= (0b0111<<RCC_CFGR_PLLMULL_Pos);
	RCC->CFGR |= (0b0<<RCC_CFGR_OTGFSPRE_Pos);
	RCC->CFGR |= RCC_CFGR_PLLSRC;


	//enable pll
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));
	light(0x03);

	FLASH->ACR = FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;
	//switch to pll
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	light(0x07);

	// USB CLK
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
}

void usb_gpio_set() {
	// since vbus sensing pin is zero by initial setup
}

void usb_core_init() {
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN; // enable USB transceiver
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT | USB_OTG_GAHBCFG_PTXFELVL | USB_OTG_GAHBCFG_TXFELVL; //fifos completely empty

    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_TOCAL_0 | USB_OTG_GUSBCFG_TOCAL_1 | USB_OTG_GUSBCFG_TOCAL_2; // add clock cycles inter-packet timeout (based on manual in MOODLE)
    USB_OTG_FS->GUSBCFG |= (0x6 << USB_OTG_GUSBCFG_TRDT_Pos); //turnaround time
	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	wait(72000,25);

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT; // un-mask global interrupt and mode mismatch interrupt
	USB_OTG_FS->GINTSTS = 0xffffffff;
    // force device mode if in host mode
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD) {
        USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_SOF | USB_OTG_GINTSTS_CIDSCHG;
        while (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD);
    }
}


/** Initialize the peripheral as device (not as host) */
void usb_device_init() {
    USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_DSPD_0 | USB_OTG_DCFG_DSPD_1; // set device speed to full-speed
    USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_NZLSOHSK; // send a STALL packet on non-zero-length status OUT transaction (default USB behavior)

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_RXFLVLM; // unmask interrupts
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN; // enable V_BUS sensing "B"
}

/** Enable interrupts in the NVIC for USB OTG FS peripheral */
void usb_interrupt_init() {
    NVIC_SetPriority(OTG_FS_IRQn, 0);
    NVIC_EnableIRQ(OTG_FS_IRQn);
}



int main(void)
{
	//	AFIO->MAPR |= 0x0180;
	//	AFIO->MAPR &= ~(0x7 << 24);

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
	usb_gpio_set();
	usb_core_init();
	usb_device_init();
	usb_interrupt_init();
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
