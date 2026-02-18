#include <stdint.h>

void usb_hid_send_report();

#define HID_PCKTSIZ 22
#define HID_EPID 2
#define HID_REPORT_OFFSET (8+8-4)

