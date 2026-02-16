#include <stdint.h>
#define AUDIO_EP 1

void stream_packet_recieved(uint32_t);
void audio_init();
void audio_deinit();
void audio_check_sync();

#define AUDIO_PCKTSIZ (uint16_t)48*4*2

typedef struct __attribute__((packed)) usb_audio_class_header_t {
	uint8_t bLength;
	uint8_t bDescriptorType; //0x24
	uint8_t bDescriptorSubtype; //0x01
	uint16_t bcdADC; //0x0100
	uint16_t wTotalLength;
	uint8_t bInCollection;
	uint8_t baInterfaceNr;
} usb_audio_class_header_t;

typedef struct __attribute__((packed)) input_terminal_ACD_t {
	uint8_t bLength; //9
	uint8_t bDescriptorType; //0x24
	uint8_t bDescriptorSubtype; //0x02
	uint8_t bTerminalID; 
	uint16_t wTerminalType; //0x0710 radio
	uint8_t bAssocTerminal; //0x00
	uint8_t bNrChannels;
	uint16_t wChannelConfig;
	uint8_t iChannelNames;
	uint8_t iTerminal;
} input_terminal_ACD_t;


typedef struct __attribute__((packed)) output_terminal_ACD_t {
	uint8_t bLength; //9
	uint8_t bDescriptorType; //0x24
	uint8_t bDescriptorSubtype; //0x02
	uint8_t bTerminalID; 
	uint16_t wTerminalType; //0x0101 streaming
	uint8_t bAssocTerminal; //0x00
	uint8_t bSourceID;
	uint8_t iTerminal;
} output_terminal_ACD_t;

typedef struct __attribute__((packed)) audio_stream_ACD_t {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalLink;
	uint8_t bDelay;
	uint16_t wFormatTag;
} audio_stream_ACD_t;

typedef struct __attribute__((packed)) format_type_ACD_t {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatType;
	uint8_t bNrChannels;
	uint8_t bSubFrameSize;
	uint8_t bBitResolution;
	uint8_t bSamFreqType;
	uint8_t bfreq0;
	uint8_t bfreq1;
	uint8_t bfreq2;
} format_type_ACD_t;

typedef struct __attribute__((packed)) isynchronous_endpoint_ACD_t {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bmAttributes;
	uint8_t bLockDelayUnits;
	uint16_t wLockDelay;
} isynchronous_endpoint_ACD_t;
