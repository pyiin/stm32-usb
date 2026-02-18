#include "usb.h"

uint8_t kbd_report[32];
uint8_t kbd_change = 0;
uint8_t hiddis = 1;
extern uint32_t led_state;

void usb_hid_send_report() {
	if(hiddis) return;
	/* if(kbd_change) */
	/* led_state++; */
	usbWrite(HID_EPID, &kbd_report, HID_PCKTSIZ);
	kbd_change = 0;
}

