#include <stdint.h>
void recieve_bulk_scsi();
void reply_bulk_scsi();
void init_scsi();
void scsi_packet_in();
void scsi_packet_sent();
void scsi_packet_recieved(uint8_t);
void scsi_send_queued();

#ifndef REQUEST_T
#define REQUEST_T
typedef struct request {
	uint32_t blknum;
	void *buf;
	uint8_t next;
} request_t;
typedef enum transfer_state{
	NO_REQUEST,
	REQUEST_READ,
	REQUEST_WRITE,
} transfer_state_t;
#endif
