
#ifndef __HW_USB_PRODUCTS_H
#define __HW_USB_PRODUCTS_H

#include <mach/usb_gadget_fserial.h>

#define HW_VENDOR_ID						0x12D1

#define TMO_ADB_DIAG_PCUI_MS_PID			0x1034
#define TMO_MICROSOFT_ADB_DIAG_PCUI_MS_PID	0x1032
#define TMO_PCUI_MS_PID						0x1030

#define HW_ADB_DIAG_PCUI_MS_PID				0x1035
#define HW_MICROSOFT_ADB_DIAG_PCUI_MS_PID	0x1033
#define HW_PCUI_MS_PID						0x1031

#define GOOGLE_ADB_MS_PID					0x1038
#define GOOGLE_MS_PID						0x1037

#define HW_RNDIS_PID						0x1039

#define INVALID_PID							0xFFFF

#define NV_NORMAL					21
#define NV_CTS						22
#define NV_MANUFACTURE				0
#define NV_GOOGLE_AUTHENICATION		25
#define NV_MICROSOFT_AUTHENICATION	99
#define NV_OTHER						// Any nv value other than the values above

#endif //__HW_USB_PRODUCTS_H

