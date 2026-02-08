#include <stdint.h>
#include "usb.h"
#include "misc.h"

uint32_t abuffer[192];

void stream_packet_recieved(uint32_t bcnt) {
	static uint8_t i=0;
	usb_ep_buf_set(1, abuffer);
	light(++i);
	usb_set_out_ep(AUDIO_EP, 192, 1);
}
