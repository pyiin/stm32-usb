#include <stdint.h>
#include "stm32f1xx.h"
#include "usb.h"
#include "misc.h"
#include "usb_scsi.h"

setup_packet_t setup = {0};
uint8_t address_pending = 0;
uint16_t address;


uint32_t buf[AUDIO_PCKTSIZ*2];

uint8_t l = 0;

#define RX_FIFO_DEPTH_IN_WORDS 256
#define TX0_FIFO_DEPTH_IN_WORDS 64
#define TX1_FIFO_DEPTH_IN_WORDS 0
#define TX2_FIFO_DEPTH_IN_WORDS 0

enum  {
	idle,
	stup,
	zlp_device,
	data_in,
	data_out,
	zlp_host,
} ep0_state;


// mass_storage defines
uint32_t lun_number = 0;


// ======================= descriptors =======================
// every report needs to have uint32 size value in the beginning that is skipped when sending
#define HID_REPORT_SIZE 78
uint8_t pre_hid_report[] = {
	HID_REPORT_SIZE, 0x00, 0x00, 0x00,
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
	0x95, 0x06,//0x06
	0x75, 0x08,
	0x81, 0x00,
	0x05, 0x08, //led page
	0x19, 0x01,
	0x29, 0x04,
	0x15, 0x00,
	0x25, 0x01,
	0x95, 0x04,
	0x75, 0x01,
	0x91, 0x02,
	0x05, 0x08, //led page
	0x19, 0x06,
	0x29, 0x09,
	0x15, 0x00,
	0x25, 0x01,
	0x95, 0x04,
	0x75, 0x01,
	0x91, 0x02,
	0xc0
};
uint32_t** hid_report = (uint32_t**)&(pre_hid_report);


device_descriptor_t pre_usb_device_descriptor = {
	.send_size = 18,
    .bLength = 18,
    .bDescriptorType = USB_DESCRIPTOR_DEVICE,
    .bcdUSB = USB_BCD_2,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0xdead,//0x413c,//0x0483,
    .idProduct = 0xbeef,//0x2010,//0x5740,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};
uint32_t** usb_device_descriptor = (uint32_t**)&(pre_usb_device_descriptor);

#define CONF_SIZE 9+9+9+9+9+9+7+11+7+12+7

full_configuration_descriptor_t pre_configuration_descriptor = {
    .send_size = CONF_SIZE,
    .usb_configuration_descriptor =
	{
		.bLength = 9,
		.bDescriptorType = USB_DESCRIPTOR_CONFIGURATION,
		.wTotalLength = CONF_SIZE,
		.bNumInterfaces = 2,
		.bConfigurationValue = 1,
		.iConfiguration = 0, // index of string descriptor
		.bmAttributes = 0b10000000,
		.bMaxPower = 50, // in 2mA units
	},
    .usb_interface_control_descriptor =
	{
		.bLength = 9,
		.bDescriptorType = 0x04,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = USB_AUDIO_CLASS,
		.bInterfaceSubClass = 0x01,
		.bInterfaceProtocol = 0x00,
		.iInterface = 0,
	},
    .audio_class =
	{
		.bLength = 9,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x01,
		.bcdADC = 0x0100,
		.wTotalLength = 9 + 12 + 9,
		.bInCollection = 1,
		.baInterfaceNr = 1,
	},
    .input_terminal =
	{
		.bLength = 12,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x02,
		.bTerminalID = 0x01,
		.wTerminalType = 0x0101,
		.bAssocTerminal = 0x00,
		.bNrChannels = 2,
		.wChannelConfig = 0x03,
		.iChannelNames = 0,
		.iTerminal = 0,
	},
    .output_terminal =
	{
		.bLength = 9,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x03,
		.bTerminalID = 0x02,
		.wTerminalType = 0x0301,
		.bAssocTerminal = 0x00,
		.bSourceID = 0x01,
		.iTerminal = 0x00,
	},
    .usb_interface_streaming0_descriptor =
	{
		.bLength = 9,
		.bDescriptorType = 0x04,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = USB_AUDIO_CLASS,
		.bInterfaceSubClass = 0x02,
		.bInterfaceProtocol = 0x00,
		.iInterface = 0,
	},
    .usb_interface_streaming1_descriptor =
	{
		.bLength = 9,
		.bDescriptorType = 0x04,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 1,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_AUDIO_CLASS,
		.bInterfaceSubClass = 0x02,
		.bInterfaceProtocol = 0x00,
		.iInterface = 0,
	},
    .audio_stream =
	{
		.bLength = 7,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x01,
		.bTerminalLink = 0x02,
		.bDelay = 0,
		.wFormatTag = 0x01,
	},
    .format_type =
	{
		.bLength = 11,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x02,
		.bFormatType = 0x01,
		.bNrChannels = 0x02,
		.bSubFrameSize = 0x04,
		.bBitResolution = 0x20,
		.bSamFreqType = 0x01,
		.bfreq0 = 0x80,
		.bfreq1 = 0xbb,
		.bfreq2 = 0x00,
	},
    .usb_endpoint1_descriptor =
	{
		.bLength = 7,
		.bDescriptorType = 0x05,
		.bEndpointAddress = 0x01,
		.bmAttributes = 0b00000001, // isynchronous?
		.wMaxPacketSize = AUDIO_PCKTSIZ,
		.bInterval = 1,
	},
    .endpoint_ACD = {
		.bLength = 7,
		.bDescriptorType = 0x25,
		.bDescriptorSubtype = 0x01,
		.bmAttributes = 0b00000000,
		.bLockDelayUnits = 0x00,
		.wLockDelay = 0,
    }
};
uint32_t** configuration_descriptor = (uint32_t**)&(pre_configuration_descriptor);

struct str_langid_s {
	uint32_t send_size;
	string_descriptor_t dsc;
	uint16_t langid;
} str_langid = {
	.send_size = 4,
	.dsc = {
		.bLength = 4,
		.bDescriptorType = USB_DESCRIPTOR_STRING,
	},
	.langid = 0x0409,
};

#define MANUF_LEN 5
struct str_manuf {
	uint32_t send_size;
	string_descriptor_t dsc;
	uint16_t str[MANUF_LEN];
} str_manuf = {
	.send_size = 2*MANUF_LEN+2,
    .dsc =
	{
		.bLength = 2*MANUF_LEN+2,
		.bDescriptorType = USB_DESCRIPTOR_STRING,
	},
    .str =
	{
		'S', 'T', 'M', '3', '2',
	},
};

#define PROD_LEN 15

struct str_prod {
	uint32_t send_size;
	string_descriptor_t dsc;
	uint16_t str[PROD_LEN];
} str_prod = {
	.send_size = 2*PROD_LEN+2,
    .dsc =
	{
		.bLength = 2*PROD_LEN+2,
		.bDescriptorType = USB_DESCRIPTOR_STRING,
	},
    .str =
	{
		'a', 'u', 'd', 'i', 'o', ' ', 'a', 'm', 'p', 'l', 'i', 'f', 'i', 'e', 'r',
	},
};

#define SN_LEN 12
struct str_sn {
	uint32_t send_size;
	string_descriptor_t dsc;
	uint16_t str[SN_LEN];
} str_sn = {
	.send_size = 2*SN_LEN+2,
    .dsc =
	{
		.bLength = 2*SN_LEN+2,
		.bDescriptorType = USB_DESCRIPTOR_STRING,
	},
    .str =
	{
		'0', '0', '0', '0', '0', '0',
		'0', '0', '0', '0', '0', '0'
	},
};

uint32_t *str_descs[] = {
	(uint32_t*)&str_langid,
	(uint32_t*)&str_manuf,
	(uint32_t*)&str_prod,
	(uint32_t*)&str_sn,
};

// ======================= helper fn =======================

void usbRawWrite(volatile uint32_t* fifo, void* data, uint32_t len) {
	uint32_t fifoWord;
	uint32_t *buffer = (uint32_t *)data;
	uint32_t remains = len;
	for (uint32_t idx = 0; idx < len; idx += 4, remains -= 4, buffer++) {
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

inline volatile uint32_t* usbEpFifo(uint8_t ep) {
    return (uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep << 12));
}


uint8_t ll = 0;
void usbZLP(uint8_t ep) {
	USB_OTG_INEndpointTypeDef* endpoint = usbEpin(ep);
	endpoint->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    endpoint->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;	
}

void usbWrite(uint8_t ep, void* data, uint32_t len) {
    USB_OTG_INEndpointTypeDef* endpoint = usbEpin(ep);
    volatile uint32_t* fifo = usbEpFifo(ep);

    uint32_t wordLen = (len + 3) >> 2;
	uint32_t rem = (endpoint->DTXFSTS & 0xffff);
    while (wordLen > (endpoint->DTXFSTS & 0xffff));
	if ((ep != 0) && (endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)) {
        return;
    }
	uint32_t pcktcnt = ((len+63)>>6);
	endpoint->DIEPTSIZ = (pcktcnt << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
	uint32_t epsiz = endpoint->DIEPTSIZ;
    endpoint->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;	
    usbRawWrite(fifo, data, len);
}

void read_ep_fifo(uint32_t* b, uint8_t ep, uint32_t len) {
	volatile uint32_t* fifo = usbEpFifo(ep);

	for (uint8_t idx = 0; idx < len; idx++, b++)
        *b = *fifo;
}

uint32_t* usb_ep_buf[4];

void usb_ep_buf_set(uint8_t ep, uint32_t *buf) {
	if(ep < 4)
		usb_ep_buf[ep] = buf;
}

inline void usb_stall(uint8_t ep) { //TODO: add dir!
	USB_OTG_INEndpointTypeDef* epin = usbEpin(ep);
	USB_OTG_OUTEndpointTypeDef* epout = usbEpout(ep);
	epin->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	epout->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
}

void ep_in_enable(uint8_t epn, uint8_t txnum, uint8_t eptype, uint16_t packet_size) {
	USB_OTG_INEndpointTypeDef* ep = usbEpin(epn);
	ep->DIEPCTL  = ((txnum & 0x03) << USB_OTG_DIEPCTL_TXFNUM_Pos);
	ep->DIEPCTL |= (eptype << USB_OTG_DIEPCTL_EPTYP_Pos);
	ep->DIEPCTL |= packet_size;
	ep->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;
	ep->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
	USB_OTG_FS_DEV->DAINTMSK |= (1<<epn);
}

void ep_out_enable(uint8_t epn, uint8_t eptype, uint16_t packet_size) {
	USB_OTG_OUTEndpointTypeDef* ep = usbEpout(epn);
	ep->DOEPCTL  = (eptype << USB_OTG_DOEPCTL_EPTYP_Pos);
	ep->DOEPCTL |= packet_size;
	ep->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;
	ep->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
	USB_OTG_FS_DEV->DAINTMSK |= (1<<(epn+16));
}

void set_ep0_idle() {
	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPTSIZ =
		(3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
		(1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
		(8 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

	USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL = USB_OTG_DOEPCTL_EPENA |
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

void usb_set_out_ep_iso(uint8_t epnum, uint32_t size, uint8_t pcktcnt, uint8_t parity) {
	USB_OTG_OUTEndpointTypeDef* ep = usbEpout(epnum);
	if(parity)
		ep->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
	else
		ep->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
	usb_set_out_ep(epnum, size, pcktcnt);
}

void usb_set_out_ep(uint8_t epnum, uint32_t size, uint8_t pcktcnt) {
	USB_OTG_OUTEndpointTypeDef* ep = usbEpout(epnum);
	ep->DOEPTSIZ = (pcktcnt << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
		(size << USB_OTG_DOEPTSIZ_XFRSIZ_Pos);

	ep->DOEPCTL |=
		USB_OTG_DOEPCTL_EPENA |
		USB_OTG_DOEPCTL_CNAK;
}

//init functions

void clock_setup(){
	RCC->CR |= RCC_CR_HSEON; //enable hse
	while(!(RCC->CR & RCC_CR_HSERDY));

	/* RCC->BDCR |= RCC_BDCR_LSEON; //enable lse */
	/* while(!(RCC->CR & RCC_BDCR_LSEON)) light(0xab); */
	/* RCC->BDCR |= (1 <<RCC_BDCR_RTCSEL_Pos); */
	/* RCC->BDCR |= RCC_BDCR_RTCEN; */
	
	//setup prediv12 and prediv1scr
	RCC->CFGR2 |= RCC_CFGR2_PREDIV1_DIV2;
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
	wait_clk(72000,25);

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST |  USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_SOFM;//USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_OTGINT | // un-mask global interrupt and mode mismatch interrupt
	USB_OTG_FS->GINTSTS = 0xffffffff; //zero all interrupts

	//set up interrupts
	NVIC_SetPriority(OTG_FS_IRQn, 1);
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


void usb_reset_handler() {
    USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPCTL |= USB_OTG_DOEPCTL_SNAK  ; // set the NAK bit for end-point 0
	 
    // interrupt un-masking
	USB_OTG_FS_DEV->DAINTMSK |= 0x10001;
    USB_OTG_FS_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_OTEPSPRM;
    USB_OTG_FS_DEV->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;// | USB_OTG_DIEPMSK_ITTXFEMSK;// | USB_OTG_DIEPINT_TXFE;// | (1<<3);

    USB_OTG_FS->GRXFSIZ = RX_FIFO_DEPTH_IN_WORDS;
	
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX0_FIFO_DEPTH_IN_WORDS << USB_OTG_TX0FD_Pos) | RX_FIFO_DEPTH_IN_WORDS;
	USB_OTG_FS->DIEPTXF[0] = ((TX1_FIFO_DEPTH_IN_WORDS) << USB_OTG_TX0FD_Pos)
		| (TX0_FIFO_DEPTH_IN_WORDS+RX_FIFO_DEPTH_IN_WORDS);
	USB_OTG_FS->DIEPTXF[1] = ((TX2_FIFO_DEPTH_IN_WORDS) << USB_OTG_TX0FD_Pos)
		| (TX0_FIFO_DEPTH_IN_WORDS+TX1_FIFO_DEPTH_IN_WORDS+RX_FIFO_DEPTH_IN_WORDS);

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

uint8_t write_report(void* buf){
	if (can_write) {
		can_write = 0;
		usbWrite(1, buf, AUDIO_PCKTSIZ);
		return 1;
	}
	return 0;
}

typedef struct descriptor_t {
	uint32_t** buf;
	uint8_t jmp;
	uint8_t maxidx;
} descriptor_t;

descriptor_t desc_array[] = {
    [USB_DESCRIPTOR_DEVICE] =        {.buf = (uint32_t **)&usb_device_descriptor, .jmp = 0, .maxidx = 0xff},
	[USB_DESCRIPTOR_CONFIGURATION] = {.buf = (uint32_t **)&configuration_descriptor, .jmp = 0, .maxidx = 0xff},
    [USB_DESCRIPTOR_HID_REPORT] =    {.buf = (uint32_t **)&hid_report, .jmp = 0, .maxidx = 0xff},
    [USB_DESCRIPTOR_STRING] =        {.buf = (uint32_t **)str_descs, .jmp = 1, .maxidx = 4},
    [USB_DESCRIPTOR_INTERFACE] =     {},
    [USB_DESCRIPTOR_ENDPOINT] =      {},
    [USB_DESCRIPTOR_QUALIFIER] =     {},
};

#define MIN(x,y) (x<y ? x : y)
void send_descriptor(uint8_t descidx, uint8_t descsub) {
	if (desc_array[descidx].buf == 0 || descsub >= desc_array[descidx].maxidx)
		usb_stall(0);
	else {
		uint32_t** buf = desc_array[descidx].buf + descsub*desc_array[descidx].jmp;
		usbWrite(0, *buf+1, MIN(setup.wLength, **buf));
		ep0_state = data_in;
	}
}

void setup_device_to_host() {
	if (setup.bRequest == BREQUEST_GET_DESCRIPTOR) {
		send_descriptor((setup.wValue & 0xff00)>>8, setup.wValue & 0x00ff);
	} else if (setup.bRequest == BREQUEST_GET_STATUS) {
		uint16_t status = USB_STATUS_RESPONSE;
		usbWrite(0, &status, 2);
		ep0_state = data_in;
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
		/* init_scsi(); */

		can_write = 1;
    }
	else if(setup.bRequest == BREQUEST_SET_INTERFACE) {
		set_ep0_zlpdev();
		if (setup.wValue == 1) {
			audio_init();
			ep_out_enable(1, EP_isyncronous, AUDIO_PCKTSIZ);
			usb_set_out_ep(1, AUDIO_PCKTSIZ, 1);
		}
	}
}

uint16_t in_size = 0;
uint8_t ready_for_datain = 0;

void usb_handle_setup_packet() {
    uint32_t* buffer = (uint32_t*) &setup;
    for (uint16_t idx = 0; idx < 2; idx++, buffer++) {
        *buffer = *USB_OTG_FS_ENDPOINT0_FIFO;
    }
	
    if (setup.bmRequestType & 0x20) { // <----------------------- handle 0x21 control transfer
		if (setup.bRequest == 0x09)
			ready_for_datain = 1;
		if (setup.bRequest == BREQUEST_SET_IDLE) {
			usb_stall(0);
			set_ep0_idle();
		}
		if (setup.bRequest == 0xfe) { // get_max_LUN
			usbWrite(0, &lun_number, 1);
			USB_OTG_OUTEndpointTypeDef* epout = usbEpout(2);
			ep0_state = data_in;
			//			usb_set_out_ep(2, 32, 1);
		}
	}
	else if (setup.bmRequestType & 0x80)
        setup_device_to_host();
    else
        setup_host_to_device();
}

void usb_read_data() {
	volatile uint32_t* fifo;
	uint32_t* epbuf;
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
		if (ready_for_datain) {
            ready_for_datain = 0;
			in_size = setup.wLength;
            usb_set_out_ep(0, in_size, 1); // in_size
            ep0_state = data_out;
		}
		break;
	case 0b0010: // out packet recieved
		if(byte_count == 0) break;
		fifo = usbEpFifo(endpoint_number);

		epbuf = usb_ep_buf[endpoint_number];//make sure it is set
		for (uint16_t i = 0; i < ((byte_count + 3) >> 2); i++, epbuf++)
			*epbuf = *fifo;
		/* usb_ep_buf[endpoint_number] = epbuf; */
		if(endpoint_number == 2)
			scsi_packet_recieved(byte_count);
		if(endpoint_number == 1)
			stream_packet_recieved(byte_count);
		break;
	case 0b0011: //out transfer complete
		if (endpoint_number == 2)
			scsi_packet_in();
		if (endpoint_number == 0 && ep0_state == data_out)
			set_ep0_zlpdev();   // STATUS stage
		break;
	default:
		break;
    }
}


void usb_interrupt_out_handler() {
	uint32_t flags = USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT;
	if(USB_OTG_FS_DEV->DAINT & (0x01 << 16)){
		if (flags & USB_OTG_DOEPINT_STUP) {
			USB_OTG_FS_DEV_ENDPOINT0_OUT->DOEPINT = USB_OTG_DOEPINT_STUP;
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
			USB_OTG_FS_DEV->DCFG &= ~USB_OTG_DCFG_DAD;
			USB_OTG_FS_DEV->DCFG |= (address << USB_OTG_DCFG_DAD_Pos);
			address_pending = 0;
			set_ep0_zlphost();
		}
	}
	if (USB_OTG_FS_DEV->DAINT & (0x02<<16)) {
		USB_OTG_OUTEndpointTypeDef* ep = usbEpout(1);
		if (ep->DOEPINT & USB_OTG_DOEPINT_XFRC)
			ep->DOEPINT = USB_OTG_DOEPINT_XFRC;
	}
	if (USB_OTG_FS_DEV->DAINT & (0x04<<16)) {
		USB_OTG_OUTEndpointTypeDef* ep = usbEpout(2);
		if (ep->DOEPINT & USB_OTG_DOEPINT_XFRC)
			ep->DOEPINT = USB_OTG_DOEPINT_XFRC;
	}
}

void usb_interrupt_in_handler() {
	if(USB_OTG_FS_DEV->DAINT & 0x01){
          if (USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT & USB_OTG_DIEPINT_XFRC) {
            /* transfer finished interrupt */
            USB_OTG_FS_DEV_ENDPOINT0_IN->DIEPINT = USB_OTG_DIEPINT_XFRC;
            if (ep0_state == data_in)
				set_ep0_zlphost();
          }
	}
	if (USB_OTG_FS_DEV->DAINT & 0x02) {
		USB_OTG_INEndpointTypeDef* ep = usbEpin(1);
		if (ep->DIEPINT & USB_OTG_DIEPINT_XFRC) {
			ep->DIEPINT = USB_OTG_DIEPINT_XFRC;
			can_write = 1;
		}
	}
	if (USB_OTG_FS_DEV->DAINT & 0x04) {
		USB_OTG_INEndpointTypeDef* ep = usbEpin(2);
		if (ep->DIEPINT & USB_OTG_DIEPINT_XFRC)
			ep->DIEPINT = USB_OTG_DIEPINT_XFRC;
		scsi_packet_sent();
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
	if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF) {
		USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_SOF;
		audio_check_sync();
    }
}
