/*
 * xmd-tty.c
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>

#include "xmd-ch.h"
#include "xmd-hsi-mcm.h"


 #define XMD_TTY_ENABLE_DEBUG_MSG
 #define XMD_TTY_ENABLE_ERR_MSG

#include "xmd-hsi-ll-cfg.h"
extern uint32_t dynamic_debug_mask;
#define dynadbg_module(mask, x...)    dynamic_debug( ((mask)|(DYNADBG_XMD_TTY_EN)), x)


static DEFINE_MUTEX(xmd_tty_lock);

//f00171359, fenghaiming begin£¬ SPIN_LOCK_UNLOCKED has been removed from kernel3.0.
static struct xmd_ch_info tty_channels[MAX_SMD_TTYS] = {
	{0,  "CHANNEL1",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[0].lock)},
	{1,  "CHANNEL2",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[1].lock)},
	{2,  "CHANNEL3",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[2].lock)},
	{3,  "CHANNEL4",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[3].lock)},
	{4,  "CHANNEL5",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[4].lock)},
	{5,  "CHANNEL6",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[5].lock)},
	{6,  "CHANNEL7",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[6].lock)},
	{7,  "CHANNEL8",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[7].lock)},
	{8,  "CHANNEL9",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[8].lock)},
	{9,  "CHANNEL10", 0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[9].lock)},
	{10, "CHANNEL11", 0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[10].lock)},
	{11, "CHANNEL12", 0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(tty_channels[11].lock)},
};
//f00171359, fenghaiming end

static int tty_channels_len = ARRAY_SIZE(tty_channels);

static void xmd_ch_tty_send_to_user(int chno)
{
	struct tty_struct *tty = NULL;
	unsigned char *buf = NULL;
	unsigned char *tbuf = NULL;
	int i,len;

	buf = (unsigned char *)xmd_ch_read(chno, &len);
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
    
	if( ( (dynamic_debug_mask)&((DYNADBG_DEBUG)|(DYNADBG_RX)|(DYNADBG_XMD_TTY_EN)|(DYNADBG_GLOBAL_EN)) ) == \
                    ((DYNADBG_DEBUG)|(DYNADBG_RX)|(DYNADBG_XMD_TTY_EN)|(DYNADBG_GLOBAL_EN)) )
	{
          char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
          if(str != NULL) {
		memcpy(str, buf, len);
            *(str+len) = '\0';
		printk("\nxmdtty: Receiving data of size %d from ch %d, buf = %s\n",
					len,chno, str);
		kfree(str);
          }
	}
#endif
	for (i=0; i<tty_channels_len; i++) {
		if (tty_channels[i].chno == chno)
			tty = (struct tty_struct *)tty_channels[i].priv;
	}

	if (!tty) {
#if defined (XMD_TTY_ENABLE_ERR_MSG)
		printk("\nxmdtty: invalid chno %d \n", chno);
#endif
		return;
	}

	tty->low_latency = 1;

	tty_prepare_flip_string(tty, &tbuf, len);

	if (!tbuf) {
#if defined (XMD_TTY_ENABLE_ERR_MSG)
		printk("\nxmdtty: memory not allocated by tty core to send to user space\n");
#endif
		return;
	}
	memcpy((void *)tbuf, (void *)buf, len);

	tty_flip_buffer_push(tty);
	tty_wakeup(tty);
}

static int xmd_ch_tty_open(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch;
	char init_flag = 0;

	int n = tty->index;

	if (n >= tty_channels_len) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
             dynadbg_module(DYNADBG_NOTICE|DYNADBG_OPEN_CLOSE,"\nxmdtty: Error opening channel %d\n",n);
//		printk("\nxmdtty: Error opening channel %d\n",n);
#endif
		return -ENODEV;
	}

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
       dynadbg_module(DYNADBG_NOTICE|DYNADBG_OPEN_CLOSE,"\nxmdtty:Opening channel %d\n",n+1);
//	printk("\nxmdtty:Opening channel %d\n",n+1);
#endif

	tty_ch = tty_channels + n;

	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 0)
		init_flag = 1;

	tty_ch->open_count++;

	if(init_flag) {
		mutex_unlock(&xmd_tty_lock);
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
             dynadbg_module(DYNADBG_NOTICE|DYNADBG_OPEN_CLOSE,"\nxmdtty: Channel already opened successfully %d\n", tty_ch->chno);
//  		printk("\nxmdtty: Channel already opened successfully %d\n",
//					tty_ch->chno);
#endif
		return 0;
	}

	tty_ch->chno = xmd_ch_open(tty_ch, xmd_ch_tty_send_to_user);
	if (0 > tty_ch->chno) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_OPEN_CLOSE,"\nError opening channel %d\n",n);
//		printk("\nError opening channel %d\n",n);
#endif
		mutex_unlock(&xmd_tty_lock);
		tty_ch->open_count = 0;
		tty_ch->chno = 0;
		return -ENOMEM;
	}

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
         dynadbg_module(DYNADBG_DEBUG|DYNADBG_OPEN_CLOSE,"\nxmdtty: Channel opened successfully %d\n",tty_ch->chno);
//	printk("\nxmdtty: Channel opened successfully %d\n",tty_ch->chno);
#endif
	tty->driver_data = (void *)tty_ch;
	tty_ch->priv = (void*) tty;
	mutex_unlock(&xmd_tty_lock);

	return 0;
}

static void xmd_ch_tty_close(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch = (struct xmd_ch_info*)tty->driver_data;
	char cleanup_flag = 1;

	if (!tty_ch) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_OPEN_CLOSE,"\nxmdtty: Channel close function error\n");
//		printk("\nxmdtty: Channel close function\n");
#endif
		return;
	}
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
       dynadbg_module(DYNADBG_NOTICE|DYNADBG_OPEN_CLOSE,"\nxmdtty: Channel close function [ch %d]\n",tty_ch->chno);
//    printk("\nxmdtty: Channel close function [ch %d]\n",tty_ch->chno);
#endif

	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 1)
		cleanup_flag = 0;

	if (tty_ch->open_count > 1)
		tty_ch->open_count--;
	else
		tty_ch->open_count = 0;

	if (cleanup_flag) {
		xmd_ch_close(tty_ch->chno);
		tty->driver_data = NULL;
	}
	mutex_unlock(&xmd_tty_lock);
}

static int xmd_ch_tty_write(
	struct tty_struct *tty,
	const unsigned char *buf,
	int len)
{
	struct xmd_ch_info *tty_ch = tty->driver_data;
	int ret = 0;

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
    if( ( (dynamic_debug_mask)&((DYNADBG_DEBUG)|(DYNADBG_RX)|(DYNADBG_XMD_TTY_EN)|(DYNADBG_GLOBAL_EN)) ) == \
                    ((DYNADBG_DEBUG)|(DYNADBG_RX)|(DYNADBG_XMD_TTY_EN)|(DYNADBG_GLOBAL_EN)) )
    {
	char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
	if(str != NULL) {
	    memcpy(str, buf, len);
	    *(str+len) = '\0';
	    printk("\nxmdtty: writing data of size %d to ch %d, data: %s\n",
                               len,tty_ch->chno,str);
	    kfree(str);
	}
    }
#endif

	ret = xmd_ch_write(tty_ch->chno, (void *)buf, len);
	if (ret >= 0)
        ret = len;

	return ret;
}

static int xmd_ch_tty_write_room(struct tty_struct *tty)
{
	return 8192;
}

static int xmd_ch_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static void xmd_ch_tty_unthrottle(struct tty_struct *tty)
{
	return;
}

#define XMD_TTY_IOCTL_MAGIC   't'
#define XMD_TTY_IOCTL_GET_WRITE_TIMEOUT     _IOR(XMD_TTY_IOCTL_MAGIC, 0xB0,int)
#define XMD_TTY_IOCTL_SET_WRITE_TIMEOUT     _IOW(XMD_TTY_IOCTL_MAGIC, 0xB1,int)
#define XMD_TTY_IOCTL_NR 0xB1 //"xmd-tty" ioctl cmd NR
#define XMD_TTY_IOCTL_NUM 2 //"xmd-tty" ioctl cmd number

static int xmd_ch_tty_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
    struct xmd_ch_info *tty_ch = tty->driver_data;
    int val;
    int result=0;

    if (!tty_ch) {
        dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\nxmdtty: %s function error. ch %d cmd=0x%08X;\n",__func__, tty_ch->chno, cmd);
        result = -ENODEV;
        goto quit_ioctl;
    }

    dynadbg_module(DYNADBG_DEBUG|DYNADBG_WRITE,"xmdtty: %s. ch %d cmd=0x%08X, arg=0x%08lX;\n",
                                __func__, tty_ch->chno, cmd, arg );
    if(_IOC_TYPE(cmd) != XMD_TTY_IOCTL_MAGIC)
        return -ENOIOCTLCMD;

    switch( cmd ) {
        case XMD_TTY_IOCTL_GET_WRITE_TIMEOUT:
            result = hsi_ch_ioctl(tty_ch->chno, HSI_CH_IOCTL_GET_WRITE_TIMEOUT, (void*)&val);
            if( result<0 )
                goto quit_ioctl;
            if( copy_to_user((void *)arg, (void*)&val, sizeof(int)) ) {
                result = - EFAULT;
                goto quit_ioctl;
            }
            break;
        case XMD_TTY_IOCTL_SET_WRITE_TIMEOUT:
            if( copy_from_user((void*)&val, (void *)arg, sizeof(int)) ) {
                result = - EFAULT;
                goto quit_ioctl;
            }
            result = hsi_ch_ioctl(tty_ch->chno, HSI_CH_IOCTL_SET_WRITE_TIMEOUT, (void*)&val);
            if( result<0 )
                goto quit_ioctl;
            break;

        default:
            result = -ENOIOCTLCMD;
            goto quit_ioctl;
    }
    return 0;

quit_ioctl:
    dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"xmdtty: %s. ch %d cmd=0x%08X, arg=0x%08lX, ioctl failed. %d\n",
                                __func__, tty_ch->chno, cmd, arg, result);
    return result;
}

static struct tty_operations xmd_ch_tty_ops = {
	.open = xmd_ch_tty_open,
	.close = xmd_ch_tty_close,
	.write = xmd_ch_tty_write,
	.write_room = xmd_ch_tty_write_room,
	.ioctl = xmd_ch_tty_ioctl,
	.chars_in_buffer = xmd_ch_tty_chars_in_buffer,
	.unthrottle = xmd_ch_tty_unthrottle,
};

static struct tty_driver *xmd_ch_tty_driver;

static int __init xmd_ch_tty_init(void)
{
	int ret, i;

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
       dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nxmdtty: xmd_ch_tty_init\n");
//	printk("\nxmdtty: xmd_ch_tty_init\n");
#endif
	xmd_ch_tty_driver = alloc_tty_driver(MAX_SMD_TTYS);
	if (xmd_ch_tty_driver == 0) {
		return -ENOMEM;
	}

	xmd_ch_tty_driver->owner = THIS_MODULE;
	xmd_ch_tty_driver->driver_name = "xmd_ch_tty_driver";
	xmd_ch_tty_driver->name = "xmd-tty"; /* "ttyspi"; "xmd-tty"; */
	xmd_ch_tty_driver->major = 0;
	xmd_ch_tty_driver->minor_start = 0;
	xmd_ch_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	xmd_ch_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	xmd_ch_tty_driver->init_termios = tty_std_termios;
	xmd_ch_tty_driver->init_termios.c_iflag = 0;
	xmd_ch_tty_driver->init_termios.c_oflag = 0;
	xmd_ch_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	xmd_ch_tty_driver->init_termios.c_lflag = 0;
	xmd_ch_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
								TTY_DRIVER_REAL_RAW 	|
								TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(xmd_ch_tty_driver, &xmd_ch_tty_ops);

	ret = tty_register_driver(xmd_ch_tty_driver);
	if (ret) return ret;

	for (i = 0; i < tty_channels_len; i++)
		tty_register_device(xmd_ch_tty_driver, tty_channels[i].id, 0);

	/* xmd_ch_init(); */

	return 0;
}

module_init(xmd_ch_tty_init);
