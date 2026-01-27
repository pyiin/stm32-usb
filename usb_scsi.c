#include "usb.h"
#include "stm32f1xx.h"
#include "usb_scsi.h"
#include "flash.h"
#include "misc.h"

#define SCSI_EP 2
#define BLOCK_SIZE_pos 11
#define BLOCK_SIZE (1<<BLOCK_SIZE_pos)
#define BASE_BLOCK_OFFSET 16
//CBW 32 bytes, CSW 13 bytes exactly

/* __attribute__((section(".pendrive"))) */
uint8_t data[48*1024];
uint16_t* flash = (uint16_t*)(0x8000000 + (BASE_BLOCK_OFFSET<<BLOCK_SIZE_pos));
uint32_t sent = 0;
uint32_t written = 0;

typedef struct scsi_cbw_t {
	uint32_t signature;
	uint32_t tag;
	uint32_t data_transfer_length;
	uint8_t flags;
	uint8_t lun;
	uint8_t cbw_len;
	uint8_t cbw[16];
} scsi_cbw_t;

typedef struct scsi_csw_t {
  uint32_t signature;
  uint32_t tag;
  uint32_t data_residue;
  uint8_t status;
} scsi_csw_t;

scsi_cbw_t scsi_cbw;
scsi_csw_t scsi_csw = {.signature = 0x53425355};

void recieve_bulk_scsi();
void scsi_error_transaction();
void parse_scsi_command();
void reply_bulk_scsi();
void scsi_packet_sent();
void scsi_read();
void scsi_packet_recieved(uint8_t);

enum {
	cbw,
	in,
	out,
	csw,
} scsi_status = cbw;

void init_scsi() {
	flash_erase(BASE_BLOCK_OFFSET);
	usb_ep_buf_set(2, (uint32_t *)&scsi_cbw);
	ep_in_enable(2, 2, EP_bulk, 64);
	ep_out_enable(2, EP_bulk, 64);
	usb_set_out_ep(2, 31, 1);
	scsi_status = cbw;
}

void scsi_packet_in() {//out packet recieved
	switch (scsi_status) {
	case cbw:
		recieve_bulk_scsi();
		break;
	case out:
		parse_scsi_command();
		break;
	case in:
		scsi_error_transaction();
		break;
	case csw:
		reply_bulk_scsi();
		break;
	}
}

void scsi_packet_sent() {//in packet correctly sent
	switch (scsi_status) {
	case cbw:
		parse_scsi_command();
		break;
	case out:
		scsi_error_transaction();
		break;
	case in:
		reply_bulk_scsi();
		break;
	case csw:
		scsi_status = cbw;
		break;
	}	
}


void recieve_bulk_scsi() { //rename to cbw
	if(scsi_cbw.tag != 0x43425355)
		scsi_error_transaction();
	scsi_csw.tag = scsi_cbw.tag;
	scsi_csw.data_residue = 0;
	sent = 0;
	written = 0;
	if (scsi_cbw.lun != 0) // bad
		scsi_error_transaction();
	parse_scsi_command();
}

void scsi_error_transaction() {
	
}

uint8_t mode_sense_response[192] = {
	0x03, 0x00, 0x00, 0x00,
	0x08, 0x0a,
	0x01,
};

uint8_t inquiry_response[36] = {
    // https://knowledge.seagate.com/files/staticfiles/support/docs/manual/Interface%20manuals/100293068m.pdf,
    // p.104
    0x00,             // direct access device
    0x80,             // removable
    0x06,             // jakaś wersja
    0x02,             // response data format
    0x03,             // additional length TODO
    0x00, 0x00, 0x00, // not supported
}; // minimum 36 bytes

uint8_t capacity[8] = {
	0x00, 0x00, 0x00, 48,
	0x00, 0x00, (1<<(BLOCK_SIZE_pos-8)), 0x00, //block_size
};

void scsi_read() {
	uint32_t block = scsi_cbw.cbw[4]<<8 | scsi_cbw.cbw[5];
	uint16_t numblocks;//TODO start block
	usbWrite(2, (uint32_t*)(flash+(block<<BLOCK_SIZE_pos)+sent), 128);
	sent += 128;
	if(sent < scsi_cbw.data_transfer_length)
		scsi_status = cbw;
	else
		scsi_status = in;
}

uint16_t* flash_buffer;

void scsi_packet_recieved(uint8_t psize) {
	flash_unlock();
	FLASH->CR |= FLASH_CR_PG;
	if(scsi_cbw.cbw[0] != 0x2a) return;
	for (uint8_t i = 0; i < ((psize+1)>>1); i++) {
		*(flash_buffer++) = *(((uint16_t*)data)+i);
		flash_ready_wait();
	}
	usb_ep_buf_set(2, (uint32_t*)data);
	FLASH->CR &= ~FLASH_CR_PG;
	flash_lock();
}

void scsi_write() {
	uint32_t block = scsi_cbw.cbw[4]<<8 | scsi_cbw.cbw[5];
	uint16_t numblocks = scsi_cbw.cbw[7]<<8 | scsi_cbw.cbw[8];
	for(uint16_t i = 0; i<numblocks; i++)
		flash_erase(block+i+BASE_BLOCK_OFFSET);

	flash_buffer = flash + (block<<(BLOCK_SIZE_pos - 1));
	usb_ep_buf_set(2, (uint32_t*)data);
	/* usb_ep_buf_set(2, (uint32_t *)(data + (block<<BLOCK_SIZE_pos))); */
	usb_set_out_ep(2, scsi_cbw.data_transfer_length, scsi_cbw.data_transfer_length>>6);
	/* scsi_csw.data_residue = scsi_cbw.data_transfer_length - 512; */
	scsi_status = csw;
}

// read10, read_capacity10, mode_sense6, inquiry, pewnie jakieś write
void parse_scsi_command() {
	switch (scsi_cbw.cbw[0]) {
	case 0x00: // test unit ready
		usbZLP(2);
		scsi_status = in;
		break;
	case 0x12: // inquiry
		usbWrite(2, (uint32_t *)inquiry_response, scsi_cbw.data_transfer_length);
		scsi_status = in;
		break;
	case 0x1a: // mode_sense6
		usbWrite(2, (uint32_t*)mode_sense_response, sizeof(mode_sense_response));
		scsi_csw.data_residue = scsi_cbw.data_transfer_length - sizeof(mode_sense_response);
		scsi_status = in;
		break;
	case 0x1e: // prevent/allow removal
		usbZLP(2);
		scsi_status = in;
		break;
	case 0x25: // read_capacity10
		usbWrite(2, (uint32_t*)capacity, 8);
		scsi_status = in;
		break;
	case 0x28: // read10
		scsi_read();
		break;
	case 0x2a:
		scsi_write();
		break;
	}
}

void reply_bulk_scsi() {
	usbWrite(SCSI_EP, (uint32_t*)&scsi_csw, 13);
	usb_ep_buf_set(2, (uint32_t *)&scsi_cbw);
	usb_set_out_ep(2, 31, 1);
	scsi_status = csw;
}
