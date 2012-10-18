/*
 * hsi_driver_if.h
 *
 * Header for the HSI driver low level interface.
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __HSI_DRIVER_IF_H__
#define __HSI_DRIVER_IF_H__

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/notifier.h>

/* The number of ports handled by the driver (MAX:2). Reducing this value
 * optimizes the driver memory footprint.
 */
#define HSI_MAX_PORTS		2

/* bit-field definition for allowed controller IDs and channels */
#define ANY_HSI_CONTROLLER	-1

/* HSR special divisor values set to control the auto-divisor Rx mode */
#define HSI_HSR_DIVISOR_AUTO		0x1000	/* Activate auto Rx */
#define HSI_SSR_DIVISOR_USE_TIMEOUT	0x1001	/* De-activate auto-Rx (SSI) */

#define HSI_SPEED_LOW_SPEED	0	/* Switch to 96MHz */
#define HSI_SPEED_HI_SPEED	1	/* Switch to 192MHz */

enum {
	HSI_EVENT_BREAK_DETECTED = 0,
	HSI_EVENT_ERROR,
	HSI_EVENT_PRE_SPEED_CHANGE,
	HSI_EVENT_POST_SPEED_CHANGE,
	HSI_EVENT_CAWAKE_UP,
	HSI_EVENT_CAWAKE_DOWN,
	HSI_EVENT_HSR_DATAAVAILABLE,
};

enum {
	HSI_IOCTL_ACWAKE_DOWN = 0,	/* Unset HST ACWAKE line for channel */
	HSI_IOCTL_ACWAKE_UP,	/* Set HSI wakeup line (acwake) for channel */
	HSI_IOCTL_SEND_BREAK,	/* Send a HW BREAK frame in FRAME mode */
	HSI_IOCTL_GET_ACWAKE,	/* Get HST CAWAKE line status */
	HSI_IOCTL_FLUSH_RX,	/* Force the HSR to idle state */
	HSI_IOCTL_FLUSH_TX,	/* Force the HST to idle state */
	HSI_IOCTL_GET_CAWAKE,	/* Get CAWAKE (HSR) line status */
	HSI_IOCTL_SET_RX,	/* Set HSR configuration */
	HSI_IOCTL_GET_RX,	/* Get HSR configuration */
	HSI_IOCTL_SET_TX,	/* Set HST configuration */
	HSI_IOCTL_GET_TX,	/* Get HST configuration */
	HSI_IOCTL_SW_RESET,	/* Force a HSI SW RESET */
	HSI_IOCTL_GET_FIFO_OCCUPANCY, /* Get amount of words in RX FIFO */
	HSI_IOCTL_SET_WAKE_RX_3WIRES_MODE, /* Enable RX wakeup 3-wires mode */
	HSI_IOCTL_SET_WAKE_RX_4WIRES_MODE, /* Enable RX wakeup 4-wires mode */
	HSI_IOCTL_SET_HI_SPEED, /* Change HSI Fclock (96MHz/192MHz) */
	HSI_IOCTL_GET_SPEED, /* Get HSI Fclock (96MHz/192MHz) */

	HSI_IOCTL_SET_3WIRES_MODE,	/* Enable HSI 3-wires mode */
	HSI_IOCTL_SET_4WIRES_MODE,	/* Enable HSI 4-wires mode */

	HSI_IOCTL_SET_HSI_MODE,	/* 15, Set HSI hsi_mode value for all channel */
	HSI_IOCTL_GET_HSI_MODE,	/* 16, Get HSI hsi_mode value for all channel */

	HSI_IOCTL_FLUSH_RXSTATE,	/* Force the HSR to idle state */
	HSI_IOCTL_FLUSH_TXSTATE,	/* Force the HST to idle state */
};

//#include "../../arch/arm/mach-omap2/xmd_boot_tty.h"   

#define XMM_HSI_PSI_STATE   (1<<0)  //bit0,XMM_HSI_PSI_STATE, HSI PSI_STATE标志;
#define XMM_HSI_EBL_START   (1<<1)  //bit1,XMM_HSI_EBL_START, HSI EBL_START标志;
#define XMM_HSI_IMG_STATE   (1<<2)  //bit2,XMM_HSI_IMG_STATE, HSI IMG_STATE标志;   //目前无动作
#define XMM_HSI_CODE_DOWN   (1<<3)  //bit3,XMM_HSI_CODE_DOWN, HSI CODE_DOWN标志;   //目前无动作

#define XMM_HSI_3_WIRE_TYPE (1<<4)  //bit4,XMM_HSI_3_WIRE_TYPE, HSI线模式标志;
#define XMM_HSI_READY_LVL   (1<<5)  //bit5,XMM_HSI_READY_LVL, HSI READY线默认值;    //目前不支持,值同 XMM_HSI_3_WIRE_TYPE;
#define XMM_HSI_EXHSI_TYPE  (1<<6)  //bit6,XMM_HSI_EXHSI_TYPE, HSI ExHSI模式标志;
#define XMM_HSI_CAWAKE_LVL  (1<<7)  //bit7,XMM_HSI_CAWAKE_LVL, HSI CAWAKE线默认值;

#define XMM_HSI_CLK_UNDIS   (1<<18) //bit18,XMM_HSI_CLK_UNDIS, HSI时钟取消关闭标志;
#define XMM_HSI_PM_UNDFT    (1<<19) //bit19,XMM_HSI_PM_UNDFT, HSI PM_unset_default标志;
#define XMM_HSI_DISINT_CAWAKE   (1<<20) //bit20,XMM_HSI_DISINT_CAWAKE, HSI CAWAKE中断禁止标志;
#define XMM_HSI_RD_MDELAY   (1<<21) //bit21,XMM_HSI_RD_MDELAY, HSI hsi_read后mdelay标志;    //##调试而设#

#define XMM_HSI_USE_BOOT    (1<<30) //bit30,XMM_HSI_USE_BOOT, HSI使用模式标志;
#define XMM_HSI_Modem_Flashless (1<<31) //bit31,XMM_HSI_Modem_Flashless, Modem存储类型;

/* Forward references */
struct hsi_device;
struct hsi_channel;

/* DPS */
struct hst_ctx {
	u32 mode;
	u32 flow;
	u32 frame_size;
	u32 divisor;
	u32 arb_mode;
	u32 channels;
};

struct hsr_ctx {
	u32 mode;
	u32 flow;
	u32 frame_size;
	u32 divisor;
	u32 counters;
	u32 channels;
};

struct hsi_port_ctx {
	int port_number; /* Range [1, 2] */
	u32 sys_mpu_enable[2];
	struct hst_ctx hst;
	struct hsr_ctx hsr;
};

/**
 * struct hsi_ctrl_ctx - hsi controller regs context
 * @sysconfig: keeps HSI_SYSCONFIG reg state
 * @gdd_gcr: keeps DMA_GCR reg state
 * @dll: keeps HSR_DLL state
 * @pctx: array of port context
 */
struct hsi_ctrl_ctx {
	u32 sysconfig;
	u32 gdd_gcr;
	u32 dll;
	struct hsi_port_ctx *pctx;
};
/* END DPS */

/**
 * struct hsi_device - HSI device object (Virtual)
 * @n_ctrl: associated HSI controller platform id number
 * @n_p: port number
 * @n_ch: channel number
 * @ch: channel descriptor
 * @device: associated device
*/
struct hsi_device {
	int n_ctrl;
	unsigned int n_p;
	unsigned int n_ch;
	struct hsi_channel *ch;
	struct device device;
};

#define to_hsi_device(dev)	container_of(dev, struct hsi_device, device)

/**
 * struct hsi_device_driver - HSI driver instance container
 * @ctrl_mask: bit-field indicating the supported HSI device ids
 * @ch_mask: bit-field indicating enabled channels for this port
 * @probe: probe callback (driver registering)
 * @remove: remove callback (driver un-registering)
 * @suspend: suspend callback
 * @resume: resume callback
 * @driver: associated device_driver object
*/
struct hsi_device_driver {
	unsigned long ctrl_mask;
	unsigned long ch_mask[HSI_MAX_PORTS];
	int (*probe) (struct hsi_device *dev);
	int (*remove) (struct hsi_device *dev);
	int (*suspend) (struct hsi_device *dev, pm_message_t mesg);
	int (*resume) (struct hsi_device *dev);
	struct device_driver driver;
	void *priv_data;

};

#define to_hsi_device_driver(drv) container_of(drv, \
						struct hsi_device_driver, \
						driver)

int hsi_register_driver(struct hsi_device_driver *driver);
void hsi_unregister_driver(struct hsi_device_driver *driver);
int hsi_open(struct hsi_device *dev);
int hsi_write(struct hsi_device *dev, u32 * addr, unsigned int size);
int hsi_write_cancel(struct hsi_device *dev);
int hsi_read(struct hsi_device *dev, u32 * addr, unsigned int size);
int hsi_read_cancel(struct hsi_device *dev);
int hsi_poll(struct hsi_device *dev);
int hsi_unpoll(struct hsi_device *dev);
int hsi_ioctl(struct hsi_device *dev, unsigned int command, void *arg);
void hsi_close(struct hsi_device *dev);
void hsi_set_read_cb(struct hsi_device *dev,
		     void (*read_cb) (struct hsi_device *dev,
				      unsigned int size));
void hsi_set_write_cb(struct hsi_device *dev,
		      void (*write_cb) (struct hsi_device *dev,
					unsigned int size));
void hsi_set_port_event_cb(struct hsi_device *dev,
			   void (*port_event_cb) (struct hsi_device *dev,
						  unsigned int event,
						  void *arg));
#endif /* __HSI_DRIVER_IF_H__ */
