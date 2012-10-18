/*
 * arch/arm/mach-omap2/xmd-boot-hsi.c
 *
 * xmd-boot-hsi.c -- Configuration of HSI GPIOs.
 *
 * Copyright (C) 2010 Infineon Technologies AG. All rights reserved.
 *
 * Author: Khened Chaitanya <Chaitanya.Khened@infineon.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
//#include <linux/smp_lock.h>	//f00171359, fenghaiming modify£¬ file smp_lock.h has been removed from kernel3.0.
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#if defined(CONFIG_ARCH_OMAP3)
#include <mach/mux.h>
#endif
#if defined(CONFIG_ARCH_OMAP4)
#include "mux.h"
#endif
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/wakelock.h>

#include <mach/msm_smd.h>
#include "smd_private.h"
#include <mach/xmd.h>
#include "xmd-ch.h"
#include "xmd-hsi-ll-if.h"
#include "xmd_hsi_mem.h"


extern struct xmd_data my_xmd;
extern void hsi_ll_wakeup_cp(unsigned int val);

#define h (&my_xmd.hsi_data)

void xmd_hsi_boot_cb (void);

void
xmd_hsi_init (void)
{
  printk("\nhsiboot: in xmd hsi init function\n");

  hsi_mem_init ();
  xmd_ch_init ();
  xmd_ch_register_xmd_boot_cb (xmd_hsi_boot_cb);

#if 0   //move it to xmd_hsi_board_init function, initialize it early 
  init_waitqueue_head (&cp_ready.cp_ready_wait);
  cp_ready.cp_ready_flag = 0;
#endif  //#if 0
}

void
xmd_hsi_exit (void)
{
  xmd_ch_exit ();
}

int
xmd_boot_enable_fw_tty (void)
{
  return 0;			//FIRMWARE download not supported.
}

void
xmd_boot_disable_fw_tty (void)
{
  return;			//FIRMWARE download not supported.
}

int
xmd_hsi_power_off (void)
{
  int status = 0;
  struct xmd_hsi_platform_data *pd;

  pd = h->hsi_platform_data;
  if (!pd)
    {
      printk (KERN_INFO "\nCRASH: NULL access in xmd_hsi_power_off\n");
      return -EINVAL;
    }

  xmd_omap_mux_init_signal (pd->hsi_cawake_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_cadata_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caflag_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acready_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acwake_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acdata_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acflag_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caready_safe_pinmux);

  //hsi_ll_wakeup_cp(0);
  //cp_ready.cp_ready_flag = 0;   /* < Modify for Modem boot xmd_ready PIN indication > */

  return (status);
}

void xmd_hsi_power_on_reset (void)
{

  struct xmd_hsi_platform_data *pd;

  pd = h->hsi_platform_data;
  if (!pd)
    {
      printk (KERN_INFO "\nCRASH: NULL access in xmd_hsi_power_on_reset\n");
      return;
    }

  //hsi_ll_wakeup_cp(0);

  xmd_omap_mux_init_signal (pd->hsi_cawake_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_cadata_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caflag_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acready_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acwake_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acdata_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acflag_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caready_pinmux);

}

void
xmd_hsi_board_init (struct xmd_hsi_platform_data *d)
{
  //Set wake lines to high [untill power mgmt is ready] TBD
    h->hsi_platform_data = d;
}

#if 0   //now using "int wait_for_xmd_ack_timeout(unsigned int ms)"
int wait_for_xmd_ack (void)
{

  wait_event_interruptible_timeout (cp_ready.cp_ready_wait,
				    cp_ready.cp_ready_flag == 1, 500);
#if 0
  if (!cp_ready.cp_ready_flag)	//spinlock protection TBD
    return -EINVAL;
#endif
  return 0;
}
#endif  //#if   0

void xmd_hsi_boot_cb (void)
{
    spin_lock(&my_xmd.boot_data.cp_ready.lock);
    my_xmd.boot_data.cp_ready.cp_ready_flag = XMD_STATE_READY;	//spinlock protection
    wake_up_interruptible (&my_xmd.boot_data.cp_ready.cp_ready_wait);
    spin_unlock(&my_xmd.boot_data.cp_ready.lock);
}

