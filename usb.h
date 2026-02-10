#include <stdint.h>
#include "stm32f1xx.h"
#include "usb_audio.h"

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
	uint32_t send_size;
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

typedef struct __attribute__((packed)) string_descriptor_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
} string_descriptor_t;

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

typedef struct __attribute__((packed)) HID_descriptor_t {
    uint8_t bLength; // 0x09
    uint8_t bDescriptorType; //0x22
    uint16_t bcdHid; //0x0111
    uint8_t bCountryCode; //0x00
    uint8_t bNumDescriptors;
    uint8_t bHidDescriptorType; // 0x22 -- report
    uint16_t wDescriptorLength;
} HID_descriptor_t;


typedef struct __attribute__((packed)) full_configuration_descriptor_t {
	uint32_t send_size;
	configuration_descriptor_t usb_configuration_descriptor;
	interface_descriptor_t usb_interface_control_descriptor;
	usb_audio_class_header_t audio_class;
	input_terminal_ACD_t input_terminal;
	output_terminal_ACD_t output_terminal;
	interface_descriptor_t usb_interface_streaming0_descriptor;
	interface_descriptor_t usb_interface_streaming1_descriptor;
	audio_stream_ACD_t audio_stream;
	format_type_ACD_t format_type;
	endpoint_descriptor_t usb_endpoint1_descriptor;
	isynchronous_endpoint_ACD_t endpoint_ACD;
} full_configuration_descriptor_t;


#define	EP_control 0x0
#define	EP_isyncronous 0x1
#define	EP_bulk 0x2
#define	EP_interrupt 0x3


#define USB_BCD_2 0x0200 //USB 2.0
#define USB_HID_CLASS 0x03
#define USB_AUDIO_CLASS 0x01
#define USB_MASS_STORAGE 0x08
#define USB_CDC_ACM_SUBCLASS 0x00
#define USB_NO_SPECIFIC_PROTOCOL 0x00 //

#define USB_DESCRIPTOR_DEVICE 0x01
#define USB_DESCRIPTOR_CONFIGURATION 0x02
#define USB_DESCRIPTOR_STRING 0x03
#define USB_DESCRIPTOR_INTERFACE 0x04
#define USB_DESCRIPTOR_ENDPOINT 0x05
#define USB_DESCRIPTOR_QUALIFIER 0x06
#define USB_DESCRIPTOR_HID_REPORT 0x22

#define BREQUEST_SET_IDLE 0x0a
#define BREQUEST_GET_STATUS 0x00
#define BREQUEST_SET_ADDRESS 0x05
#define BREQUEST_GET_DESCRIPTOR 0x06
#define BREQUEST_SET_REPORT 0x0
#define BREQUEST_SET_CONFIGURATION 0x09
#define BREQUEST_SET_INTERFACE 0x0b

#define USB_STATUS_RESPONSE 0x0000

void usbZLP(uint8_t ep);
volatile uint32_t* usbEpFifo(uint8_t ep);
USB_OTG_OUTEndpointTypeDef* usbEpout(uint8_t ep);
USB_OTG_INEndpointTypeDef* usbEpin(uint8_t ep);
void usb_stall(uint8_t ep);
void clock_setup();
void usb_core_init();
void usb_device_init();
uint8_t write_report(void*);
void usb_ep_buf_set(uint8_t ep, uint32_t *buf);
void read_ep_fifo(uint32_t*, uint8_t, uint32_t);
void usbWrite(uint8_t ep, void* data, uint32_t len);
void ep_in_enable(uint8_t epn, uint8_t txnum, uint8_t eptype, uint16_t packet_size);
void ep_out_enable(uint8_t epn, uint8_t eptype, uint16_t packet_size);
void usb_set_out_ep(uint8_t epnum, uint32_t size, uint8_t pcktcnt);
