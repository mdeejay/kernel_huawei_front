/*
 * arch/arm/mach-omap2/xmd-boot-common.c
 *
 * xmd-boot-common.c -- Boot control for IFX XMM modem.
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
//#include <linux/smp_lock.h>	//f00171359, fenghaiming modify， file smp_lock.h has been removed from kernel3.0.
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#if defined(CONFIG_ARCH_OMAP3)
#include <mach/mux.h>
#endif
#if defined(CONFIG_ARCH_OMAP4)
#include "mux.h"
#endif
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <net/sock.h>
#include <net/netlink.h>
#include <linux/skbuff.h>

#include <linux/wakelock.h>

#include <mach/msm_smd.h>
#include "smd_private.h"
#include <mach/xmd.h>
#include "xmd-ch.h"
#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
#include "xmd-hsi-ll-if.h"
#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
#include "hsad/config_mgr.h"   


/*
* Local definitions
*/

static DEFINE_MUTEX(boot_state_mutex);

#if	defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
static int flashless = 1;   
#else
static int flashless = 0;
#endif  //#if defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)

struct xmd_data my_xmd;

#define b (&my_xmd.boot_data)

static struct sock *g_nlfd=NULL;
DEFINE_MUTEX(receive_sem);
static unsigned int g_rild_pid = 0;

const char * xmd_event_str[] = {"FREE", "OFF", "POWER","READY"};
/* Format of the message send from kernel to userspace */
struct xmd_nl_packet_msg {
	int xmd_event;
};
/* The message type send between rild and hsi */
typedef enum Xmd_KnlMsgType
{
	NETLINK_XMD_REG=0,	//send from rild to register the PID for netlink kernel socket
	NETLINK_XMD_KER_MSG,	//send from xmd to rild
	NETLINK_XMD_UNREG	//when rild exit send this type message to unregister
}XMD_MSG_TYPE_EN;

/*
 Receive the message from rild,to register rild PID or unregister the PID
*/
static void kernel_xmd_receive(struct sk_buff *skb)
{
	mutex_lock(&receive_sem);

	if(skb->len >= sizeof(struct nlmsghdr))
	{
		struct nlmsghdr *nlh;
		nlh = (struct nlmsghdr *)skb->data;
		if((nlh->nlmsg_len >= sizeof(struct nlmsghdr))&& (skb->len >= nlh->nlmsg_len))
		{
			if(nlh->nlmsg_type == NETLINK_XMD_REG)
			{
				pr_info("XMD: %s, netlink receive reg packet ;\n",__func__ );
				g_rild_pid = nlh->nlmsg_pid;
			}
			else if(nlh->nlmsg_type == NETLINK_XMD_UNREG)
			{
				pr_info("XMD: %s, netlink NETLINK_TIMER_UNREG ;\n",__func__ );
				g_rild_pid = 0;
			}
		}
	}
        mutex_unlock(&receive_sem);
}
/*
 Setup the netlink kernel socket with type NETLINK_XMD=21
*/
static void xmd_netlink_init(void)
{
	g_nlfd =netlink_kernel_create(&init_net,
			NETLINK_XMD, 0, kernel_xmd_receive, NULL, THIS_MODULE);
	if(!g_nlfd)
		pr_info("XMD: %s, netlink_kernel_create faile ;\n",__func__ );
	else
		pr_info("XMD: %s, netlink_kernel_create success ;\n",__func__ );
	return;
}

/*
 Send the netlink message to userspace (rild)
*/
static int xmd_change_notify_event(int event)
{
	int ret;
	int size;
	unsigned char *old_tail;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct xmd_nl_packet_msg *packet;

	mutex_lock(&receive_sem);
	if (!g_rild_pid || !g_nlfd ) {
            ret = -1;
            goto end;
       }
	size = NLMSG_SPACE(sizeof(struct xmd_nl_packet_msg));

	skb = alloc_skb(size, GFP_ATOMIC);
	if (!skb) {
		pr_info("XMD: %s,  alloc skb fail\n",__func__ );
                ret = -1;
                goto end;
	}
	old_tail = skb->tail;

	nlh = NLMSG_PUT(skb, 0, 0, NETLINK_XMD_KER_MSG, size-sizeof(*nlh));
	packet = NLMSG_DATA(nlh);
	memset(packet, 0, sizeof(struct xmd_nl_packet_msg));

	packet->xmd_event=event;

	nlh->nlmsg_len = skb->tail - old_tail;

	pr_info("XMD: %s,  notify event %s to Rild with PID %d;\n", __func__, xmd_event_str[event], g_rild_pid);
	ret = netlink_unicast(g_nlfd, skb, g_rild_pid, MSG_DONTWAIT);
        goto end;

nlmsg_failure:
        ret = -1;
	if(skb)
            kfree_skb(skb);
end:
	mutex_unlock(&receive_sem);
	return ret;
}

/*
* OMAP mux pad configuration function
*/
int xmd_omap_mux_init_signal(struct xmm_pad_mux_config muxcfg)
{
#if defined(CONFIG_ARCH_OMAP4)
	return omap_mux_init_signal(muxcfg.muxname, muxcfg.muxval);
#endif
#if defined(CONFIG_ARCH_OMAP3)
	return omap_cfg_reg(muxcfg.muxval);
#endif
};


/*------------------------- POWER-ON RESET SUPPORT -------------------------*/

static int xmd_boot_power_off(void)
{
	int status = 0;
	struct xmd_boot_platform_data *pd;

	pd = b->boot_platform_data;

	if (pd->reset2_gpio != -1) {
		if (b->reset2_irq_registered) {
			free_irq(OMAP_GPIO_IRQ(pd->reset2_gpio), b);
			b->reset2_irq_registered = 0;
			pr_info("XMD: %s. cp_ready_flag: %d, free_irq and set_xmd_ack_cp_ready( XMD_STATE_FREE );\n", __func__,get_xmd_ack_cp_ready());
			set_xmd_ack_cp_ready( XMD_STATE_FREE );

			xmd_change_notify_event(XMD_STATE_FREE);
		}
		xmd_omap_mux_init_signal(pd->reset2_safe_pinmux);
	}
    if( pd->xmd_ready_gpio!= -1 ) {
            if (b->xmd_ready_irq_registered) {
                free_irq(OMAP_GPIO_IRQ(pd->xmd_ready_gpio), b);
                b->xmd_ready_irq_registered = 0;
            }
            xmd_omap_mux_init_signal(pd->xmd_ready_safe_pinmux);
    }

	if (pd->baseband_reset_gpio != -1)
		xmd_omap_mux_init_signal(pd->baseband_reset_safe_pinmux);

	/* XMM power off - after software switch off, make sure reset asserted */
	gpio_set_value(pd->on_off_gpio, 0);
	gpio_set_value(pd->reset_pmu_req_gpio, 0);
	if (pd->pwr_supply_gpio != -1)
		gpio_set_value(pd->pwr_supply_gpio, 0);

	xmd_omap_mux_init_signal(pd->reset_pmu_req_pinmux);
	xmd_omap_mux_init_signal(pd->on_off_pinmux);
	if (pd->pwr_supply_gpio != -1)
		xmd_omap_mux_init_signal(pd->pwr_supply_pinmux);

	return (status);
}


int xmd_board_power_off(void)
{
	int status = 0;

	pr_info("XMD: xmd_board_power_off\n");

#ifdef CONFIG_OMAP4_XMM_SPI
	xmd_spi_power_off();
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	xmd_hsi_power_off();
#endif
	xmd_boot_power_off();
#ifdef CONFIG_OMAP4_XMM_C2C
	xmd_c2c_power_off(); /* Keep last */
#endif

	return (status);
}
extern int notify_mdm_off_to_pm(void);
static int cp_shutdown;
static DEFINE_MUTEX(cp_shutdown_mutex);
static int cp_shutdown_get(void)
{
    int value=-1;

    mutex_lock(&cp_shutdown_mutex);
    value = cp_shutdown;
    mutex_unlock(&cp_shutdown_mutex);
    return value;
}
static ssize_t cp_shutdown_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    char *_buf = buf;

    mutex_lock(&cp_shutdown_mutex);
    _buf += snprintf(_buf, PAGE_SIZE, "%d\n", cp_shutdown);

    mutex_unlock(&cp_shutdown_mutex);
    return (_buf - buf);
}
static ssize_t cp_shutdown_store(struct device *dev,struct device_attribute *attr,
                            const char *buf, size_t count)
{
    int value=0;

    mutex_lock(&cp_shutdown_mutex);
    sscanf(buf, "%d", &value);
    pr_info("\nXMD: %s. [cp_shutdown] %d -> %d;\n",__func__,cp_shutdown,value);
    cp_shutdown= value;
    mutex_unlock(&cp_shutdown_mutex);
    return count;
}
/* sysfs entries for "cp_shutdown" node control */
static DEVICE_ATTR(cp_shutdown, S_IRUGO | S_IWUSR, cp_shutdown_show, cp_shutdown_store);
/*
 * RESET2_N worker
 */
static void xmd_boot_reset2_work (struct work_struct *work)
{
	int val;

	val = gpio_get_value(b->boot_platform_data->reset2_gpio);
	pr_info("XMD: RESET2_N transition: %d, on_off_gpio: %d;\n", val,gpio_get_value(b->boot_platform_data->on_off_gpio) );
	if( 1==val ) {
            set_xmd_ack_cp_ready( XMD_STATE_POWER );
            gpio_set_value(b->boot_platform_data->on_off_gpio, 0);
            xmd_omap_mux_init_signal(b->boot_platform_data->on_off_safe_pinmux);
            pr_info("XMD: %s. RESET2_N rising edge: modem PMU on at %u ms, clear on_off_gpio to %d;\n",
                            __func__,jiffies_to_msecs(jiffies),gpio_get_value(b->boot_platform_data->on_off_gpio) );
	    xmd_change_notify_event(XMD_STATE_POWER);
	} else {
            set_xmd_ack_cp_ready( XMD_STATE_OFF );
            pr_info("XMD: %s. RESET2_N falling edge: modem abnormal power off;\n", __func__ );
            /* modem abnormal power off handing */
            if (cp_shutdown_get() == false){
                notify_mdm_off_to_pm();
                pr_info("notify modem hardware shutdown to PM");
            }
            xmd_change_notify_event(XMD_STATE_OFF);
	}

	return;
}

/*
 * RESET2_N hard irq
 */
static irqreturn_t c2c_boot_reset2_isr(int irq, void *handle)
{
	if (b->reset2_irq_registered) {
		/* Kick workqueue */
		schedule_work(&b->reset2_work);
	}
	else
		pr_err("XMD: C2C RESET2_N spurious interrupt\n");
	return IRQ_HANDLED;
}

static void xmd_boot_xmd_ready_work (struct work_struct *work)
{
    int val;

    val = gpio_get_value(b->boot_platform_data->xmd_ready_gpio);
    pr_info("XMD: XMD_READY_N transition: %d, reset2_gpio: %d;\n", val,gpio_get_value(b->boot_platform_data->reset2_gpio) );

    if( 0==val ) {
        if( get_xmd_ack_cp_ready()==XMD_STATE_POWER ) {
            pr_info("XMD: %s. XMD_STATE_POWER, wake_up_xmd_ack(), CP ready at %u ms, notify!\n",__func__,
                                jiffies_to_msecs(jiffies) );
            //xmd_omap_mux_init_signal(b->boot_platform_data->xmd_ready_safe_pinmux);
            //set_xmd_ack_cp_ready( XMD_STATE_READY );  //already be called in wake_up_xmd_ack();
            wake_up_xmd_ack();
            xmd_change_notify_event(XMD_STATE_READY);
        } else {
            pr_info("XMD: %s. cp_ready_flag: %d, xmd_ready_gpio exception;\n",__func__,get_xmd_ack_cp_ready() );
        }
    }
    else {
        if( get_xmd_ack_cp_ready()==XMD_STATE_READY ) {
		set_xmd_ack_cp_ready( XMD_STATE_POWER );
		pr_info("XMD: %s. cp_ready_flag: %d, CP not ready at %u ms, notify!\n",__func__,get_xmd_ack_cp_ready(),jiffies_to_msecs(jiffies) );
		xmd_change_notify_event(XMD_STATE_POWER);
         }
    }

	return;
}
static irqreturn_t xmd_boot_xmd_ready_isr(int irq, void *handle)
{
	if (b->xmd_ready_irq_registered) {
		/* Kick workqueue */
		schedule_work(&b->xmd_ready_work);
	}
	else
		pr_err("XMD: xmd_boot xmd_ready spurious interrupt\n");
	return IRQ_HANDLED;
}


/*
 * Modem is being powered on
 * Enable control signals IO muxing and proceed with XMM power on sequence
 */
static int xmd_boot_power_on_reset(void)
{
	int status = 0;
	struct xmd_boot_platform_data *pd;

	pd = b->boot_platform_data;

	if (pd->reset2_gpio != -1) {
		xmd_omap_mux_init_signal( pd->reset2_pinmux ); 

            status = gpio_get_value(pd->reset2_gpio);
            if( 1==status ) {
                set_xmd_ack_cp_ready( XMD_STATE_POWER );
                pr_info("XMD: %s. previous reset2_gpio: %d, set_xmd_ack_cp_ready( XMD_STATE_POWER );\n", __func__,status);
            } else {
                set_xmd_ack_cp_ready( XMD_STATE_OFF);
                pr_info("XMD: %s. previous reset2_gpio: %d, set_xmd_ack_cp_ready( XMD_STATE_OFF );\n", __func__,status);
            }

		/* RESET2_N interrupt registration */
		if (request_irq(OMAP_GPIO_IRQ(pd->reset2_gpio), c2c_boot_reset2_isr,
				IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "xmd_reset2_n", b) < 0) {
			pr_err("XMD: ERROR requesting RESET2_N IRQ\n");
		}
		else {
			b->reset2_irq_registered = 1;
			//f00171359, fenghaiming modify
			//set_irq_wake(OMAP_GPIO_IRQ(pd->reset2_gpio), 1);
			irq_set_irq_wake(OMAP_GPIO_IRQ(pd->reset2_gpio), 1);
		}
	}

        if( pd->xmd_ready_gpio!=-1 ) {
            xmd_omap_mux_init_signal (pd->xmd_ready_pinmux );

            status = gpio_get_value(pd->xmd_ready_gpio);
            if( (XMD_STATE_POWER==get_xmd_ack_cp_ready())&&(0==status) ) {
                set_xmd_ack_cp_ready( XMD_STATE_READY );
                pr_info("XMD: %s. previous xmd_ready_gpio: %d, set_xmd_ack_cp_ready( XMD_STATE_READY );\n", __func__,status);
            }

            /* XMD_READY_N interrupt registration */
            if (request_irq(OMAP_GPIO_IRQ(pd->xmd_ready_gpio), xmd_boot_xmd_ready_isr,
                    IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "xmd_ready_n", b) < 0) {
                pr_err("XMD: ERROR requesting XMD_READY_N IRQ\n");
            }
            else {
                b->xmd_ready_irq_registered = 1;
                //set_irq_wake(OMAP_GPIO_IRQ(pd->xmd_ready_gpio), 1);
                irq_set_irq_wake(OMAP_GPIO_IRQ(pd->xmd_ready_gpio), 1);
            }
        }

	/* Pin muxing and default states: ON_OFF / RESET_PMU_REQ and alike */
	if (pd->pwr_supply_gpio != -1) {
		gpio_set_value(pd->pwr_supply_gpio, 1);
		xmd_omap_mux_init_signal(pd->pwr_supply_pinmux);
		msleep(1);
	}

	/*
	* The power on sequence for 6260 requires that "On" signal goes high
	* after releasing the reset signal.
	*/
	gpio_set_value(pd->reset_pmu_req_gpio, 1);
	if (pd->baseband_reset_gpio != -1)
		gpio_set_value(pd->baseband_reset_gpio, 1);
	xmd_omap_mux_init_signal(pd->reset_pmu_req_pinmux);
	if (pd->baseband_reset_gpio != -1)
		xmd_omap_mux_init_signal(pd->baseband_reset_pinmux);
	msleep(50);    // msleep(500); /* additional 500ms delay */
	gpio_set_value(pd->on_off_gpio, 1);
	xmd_omap_mux_init_signal(pd->on_off_pinmux);
	pr_info("XMD: %s. on_off_gpio: %d, CP power on at %u ms;\n", __func__,
                gpio_get_value(b->boot_platform_data->on_off_gpio),jiffies_to_msecs(jiffies) ); 

	return (status);
}


int xmd_board_power_on_reset(void)
{
	int status = 0;

	pr_info("XMD: xmd_board_power_on_reset\n");

#ifdef CONFIG_OMAP4_XMM_C2C
	/* Keep first */
	xmd_c2c_power_on_reset();
#endif
	xmd_boot_power_on_reset();
#ifdef CONFIG_OMAP4_XMM_SPI
	xmd_spi_power_on_reset();
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	xmd_hsi_power_on_reset();
#endif

	return (status);
}

static int xmd_boot_monitor_init(struct xmd_boot *xmd_boot_data)
{
    init_waitqueue_head (&xmd_boot_data->cp_ready.cp_ready_wait);
    //cp_ready.lock= SPIN_LOCK_UNLOCKED;
    spin_lock_init(&xmd_boot_data->cp_ready.lock);   //cp_ready.lock= __SPIN_LOCK_UNLOCKED(cp_ready.lock);
    //set_xmd_ack_cp_ready(XMD_STATE_FREE); //will be set in xmd_boot_power_off function

    return true;
}

static int xmd_boot_board_init(struct xmd_boot_platform_data *boot_platform_data)
{
	int retval = 0;
	struct xmd_boot_platform_data *pd = boot_platform_data;

	b->boot_platform_data = pd;

	xmd_boot_monitor_init (&my_xmd.boot_data);

	retval = gpio_request(pd->reset_pmu_req_gpio, "XMD_RESET_PMU_REQ_GPIO");
	if(retval < 0) {
		pr_err("Can't get RESET_PMU_REQ GPIO\n");
		goto out;
	}
	gpio_direction_output(pd->reset_pmu_req_gpio, 0);

	retval = (gpio_request(pd->on_off_gpio, "XMD_ON_OFF_GPIO") < 0);
	if(retval < 0) {
		pr_err("Can't get XMD_ON_OFF_GPIO\n");
		goto out;
	}
	gpio_direction_output(pd->on_off_gpio, 0);

	/* BASEBAND_RESET - optional signal */
	if (pd->baseband_reset_gpio != -1) {
		retval = (gpio_request(pd->baseband_reset_gpio,
						"XMD_BB_RESET_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_BB_RESET_GPIO\n");
			goto out;
		}
		gpio_direction_output(pd->baseband_reset_gpio, 0);
	}

	/* PWR_SUPPLY - optional signal */
	if (pd->pwr_supply_gpio != -1) {
		retval = (gpio_request(pd->pwr_supply_gpio,
					"XMD_PWR_SUPPLY_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_PWR_SUPPLY_GPIO\n");
			goto out;
		}
		gpio_direction_output(pd->pwr_supply_gpio, 0);
	}

	/* RESET2_N - optional signal */
	if (pd->reset2_gpio != -1) {
		retval = (gpio_request(pd->reset2_gpio,
					"XMD_RESET_2_N_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_RESET_2_N_GPIO\n");
			goto out;
		}
		gpio_direction_input(pd->reset2_gpio);
	}
	b->reset2_irq_registered = 0;

	/* XMD_READY_N - optional signal */
	retval = get_hw_config_xmd_ready();  //boardid: xmd->xmd_ready_gpio_enable;
	if( retval==false ) {
            pd->xmd_ready_gpio = -1;
            pr_info("XMD: %s. xmd_ready gpio %d, not support in this board;\n",__func__,pd->xmd_ready_gpio);
	} else {
            pr_info("XMD: %s. xmd_ready gpio %d, support in this board;\n",__func__,pd->xmd_ready_gpio);
	}

	if( pd->xmd_ready_gpio!=-1 ) {
		xmd_omap_mux_init_signal (pd->xmd_ready_safe_pinmux);   //Configurate to pull down enable;
		retval = (gpio_request(pd->xmd_ready_gpio,
					"XMD_READY_N_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_READY_N_GPIO\n");
			goto out;
		}
		gpio_direction_input(pd->xmd_ready_gpio);
	}
	b->xmd_ready_irq_registered = 0;

out:
	return retval;
}


int xmd_board_init(struct xmd_platform_data *platform_data)
{
	int retval = 0;
	struct xmd_platform_data *pd = platform_data;

#ifdef CONFIG_OMAP4_XMM_SPI
	xmd_spi_board_init (&(pd->spi_platform_data));
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	xmd_hsi_board_init (&(pd->hsi_platform_data));
#endif
#ifdef CONFIG_OMAP4_XMM_C2C
	xmd_c2c_board_init (&(pd->c2c_platform_data));
#endif

	xmd_boot_board_init (&(pd->boot_platform_data));

	xmd_board_power_off();

	return retval;
}
EXPORT_SYMBOL_GPL(xmd_board_init);


/*---------------------------- SYSFS BOOT STATES ----------------------------*/

enum {XMD_BOOT_OFF, XMD_BOOT_FIRMWARE, XMD_BOOT_ON, XMD_BOOT_READY};
const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON", "READY"};   // Modify for Modem_PowerOn race between RIL and usbswitch */  static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
/*Add end for Modem boot xmd_ready PIN indication > */

static ssize_t xmd_boot_modem_state_show
       (struct device *, struct device_attribute *, char *);
ssize_t xmd_boot_modem_state_store(struct device *,
       struct device_attribute *, const char *, size_t);
static DEVICE_ATTR(state, S_IRUGO | S_IWUSR, xmd_boot_modem_state_show,
	xmd_boot_modem_state_store);

static ssize_t xmd_boot_modem_state_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int status;

	mutex_lock(&boot_state_mutex);

	if (attr == &dev_attr_state) {
		status = snprintf(buf, PAGE_SIZE, "%s\n",
			xmd_boot_state_str[b->boot_state]);
	} else
		status = -EINVAL;

	mutex_unlock(&boot_state_mutex);
	return status;
}


static int xmd_boot_state_change(int boot_state)
{
	int status=0;	/* Modify for Flashless HSIswitch */

	pr_info("\nXMD: BOOT STATE (%s) -> (%s)\n",
		xmd_boot_state_str[b->boot_state],xmd_boot_state_str[boot_state]);

	switch (boot_state) {
	case XMD_BOOT_OFF:
		switch(b->boot_state) {
		case XMD_BOOT_ON:
#ifdef CONFIG_OMAP4_XMM_SPI
			xmd_mux_disable(); /* Ignore returned value */
#endif
			/* Electrical power-off */
			xmd_board_power_off();
			break;
		case XMD_BOOT_FIRMWARE:
#if defined(CONFIG_OMAP4_XMM_HSI)
			xmd_boot_disable_fw_tty();
#endif
			/* Electrical power-off */
			xmd_board_power_off();
			break;
		default:
			goto error1;
		}
		break;

	case XMD_BOOT_FIRMWARE:
		if (!flashless)
			goto error1;
		switch(b->boot_state) {
		case XMD_BOOT_OFF:
#if defined(CONFIG_OMAP4_XMM_HSI)
			xmd_boot_enable_fw_tty();
#endif
			/* Electrical power-on/reset sequence,
			   immediate return for ROM code interaction */
			xmd_board_power_on_reset();
			break;
		default:
			goto error1;
		}
		break;

	case XMD_BOOT_READY:    /* < Add for Modem boot xmd_ready PIN indication > */
	case XMD_BOOT_ON:
		switch(b->boot_state) {
		case XMD_BOOT_OFF:
			if (flashless)
				goto error1;

			/* Electrical power-on/reset sequence,
			   and modem firmware boot time      */
			xmd_board_power_on_reset();

			/* No break */
		case XMD_BOOT_FIRMWARE:
#if defined(CONFIG_OMAP4_XMM_HSI)
			if (flashless)
				xmd_boot_disable_fw_tty();
#endif
#ifdef CONFIG_OMAP4_XMM_SPI
			status = xmd_mux_enable(); /* wait for MODEM to send msg on startup to make sure MODEM is on */
#endif
#if defined(CONFIG_OMAP4_XMM_HSI)
                   if( boot_state==XMD_BOOT_READY ) {
                        boot_state = XMD_BOOT_ON;
                        //status = wait_for_xmd_ack(); /* wait for MODEM to send msg on startup to make sure MODEM is on */
                        if( flashless ) {
                            pr_info("XMD: %s. wait_for_xmd_ack_timeout(%u ms) at %u ms;\n", __func__,XMD_BOOT_TIMEOUT_MS_FLASHLESS,jiffies_to_msecs(jiffies) );
                            status = wait_for_xmd_ack_timeout( XMD_BOOT_TIMEOUT_MS_FLASHLESS ); //wait for MODEM ready
                        } else {
                            pr_info("XMD: %s. wait_for_xmd_ack_timeout(%u ms) at %u ms;\n", __func__,XMD_BOOT_TIMEOUT_MS_FLASH,jiffies_to_msecs(jiffies) );
                            status = wait_for_xmd_ack_timeout( XMD_BOOT_TIMEOUT_MS_FLASH ); //wait for MODEM ready
                        }
                        pr_info("XMD: %s. wake_up_xmd_ack at %u ms, status. %d;\n", __func__,jiffies_to_msecs(jiffies),status);
                        status = 0; //not process here, just notify upper to dispose;
                   }
#endif

			if (status < 0)
				goto error2;
			break;
		default:
			goto error1;
		}
		break;
	}

	/* Transition success */
	b->boot_state = boot_state;
	status = 0;
	goto end;

error1:
	pr_err("XMD: BOOT wrong state transition\n");
	status = -1;
	goto end;
error2:
	pr_err("XMD: BOOT transition failed\n");
	/* Unknown modem state, force power-off */
	xmd_board_power_off();
	b->boot_state = XMD_BOOT_OFF;
	status =  -2;
	goto end;

end:
	return status;
}

/*
ms - timeout value;
return - status: -1,timeout elapsed;<0,was interrupted by a signal;
                    ==0, XMD_STATE_READY; >0, XMD_STATE_READY, elapsed time (ms);
*/
int wait_for_xmd_ack_timeout(unsigned int ms)
{   long mstime;
    int status;

      //set_xmd_ack_cp_ready(0);
      mstime = wait_event_interruptible_timeout (my_xmd.boot_data.cp_ready.cp_ready_wait,
                    get_xmd_ack_cp_ready()==XMD_STATE_READY, msecs_to_jiffies(ms) );

      if( mstime>0 )    //remaining jiffies
          status = ms<jiffies_to_msecs(mstime) ? 0 : (ms-jiffies_to_msecs(mstime)); //remaining jiffies --> elapsed time
      else if( mstime==0 )
          status = -1;  //-1,was interrupted by a signal;
      else
          status = mstime;  //mstime<0;

      if( (status<0) && (get_xmd_ack_cp_ready()==XMD_STATE_READY) )
          status = 0; //XMD_STATE_READY

      pr_debug("\n XMD_HSI: %s([ms]%u->%u), get_xmd_ack_cp_ready()=%d;\n", __func__,
                    ms,status<0?ms:status,get_xmd_ack_cp_ready() );

    return( status );
}
void wake_up_xmd_ack(void)
{
    spin_lock(&my_xmd.boot_data.cp_ready.lock);
    my_xmd.boot_data.cp_ready.cp_ready_flag = XMD_STATE_READY;	//spinlock protection
    wake_up_interruptible (&my_xmd.boot_data.cp_ready.cp_ready_wait);
    spin_unlock(&my_xmd.boot_data.cp_ready.lock);
}

/* int get_xmd_ack_cp_ready(void)
    get cp_ready.cp_ready_flag value;
return - cp_ready_flag: enum {XMD_STATE_FREE, XMD_STATE_OFF, XMD_STATE_POWER, XMD_STATE_READY};
*/
int get_xmd_ack_cp_ready(void)
{
    int ret;

    spin_lock(&my_xmd.boot_data.cp_ready.lock);
    ret = my_xmd.boot_data.cp_ready.cp_ready_flag;   //spinlock protection
    spin_unlock(&my_xmd.boot_data.cp_ready.lock);
    return( ret );
}
void set_xmd_ack_cp_ready(int val)
{
    spin_lock(&my_xmd.boot_data.cp_ready.lock);
    my_xmd.boot_data.cp_ready.cp_ready_flag = val;  //spinlock protection
    spin_unlock(&my_xmd.boot_data.cp_ready.lock);
}

int is_cp_ready(void)
{
    int ret;

    spin_lock(&my_xmd.boot_data.cp_ready.lock);
    ret = my_xmd.boot_data.cp_ready.cp_ready_flag;   //spinlock protection
    spin_unlock(&my_xmd.boot_data.cp_ready.lock);

    if( (ret==XMD_STATE_READY) || ((my_xmd.boot_data.boot_platform_data->xmd_ready_gpio==-1)&&(ret==XMD_STATE_POWER)) )   //xmd_ready_gpio not support
        return true;
    else
        return false;
}

/*------------------------------ SYSFS XMD xmd_state ------------------------------*/
static ssize_t xmd_state_show(struct device *dev,struct device_attribute *attr, char *buf);
//module_param(xmd_state, int, 0444);
static DEVICE_ATTR(xmd_state, S_IRUGO | S_IWUSR, xmd_state_show, NULL);

static ssize_t xmd_state_show(struct device *dev,struct device_attribute *attr, char *buf)
{
int xmd_state=-1;	//xmd_state, xmd_state状态值: +-enum {XMD_STATE_FREE, XMD_STATE_OFF, XMD_STATE_POWER, XMD_STATE_READY};
char *_buf = buf;

      xmd_state = get_xmd_ack_cp_ready();   //enum {XMD_STATE_FREE, XMD_STATE_OFF, XMD_STATE_POWER, XMD_STATE_READY};
      if( XMD_STATE_FREE==xmd_state ) {
          if( 1==gpio_get_value(b->boot_platform_data->reset2_gpio) ) {
              if( (-1!=b->boot_platform_data->xmd_ready_gpio)&&(0==gpio_get_value(b->boot_platform_data->xmd_ready_gpio)) ) {
                  xmd_state = XMD_STATE_READY;
                  pr_debug("XMD: %s. XMD_STATE_READY. xmd_state=%d;\n", __func__,xmd_state );
              } else {
                  xmd_state = XMD_STATE_POWER;
                  pr_debug("XMD: %s. XMD_STATE_POWER. xmd_state=%d;\n", __func__,xmd_state );
              }
          } else {
                xmd_state = XMD_STATE_OFF;
                pr_debug("XMD: %s. XMD_STATE_OFF. xmd_state=%d;\n", __func__,xmd_state );
          }
      }
      if( b->boot_platform_data->xmd_ready_gpio==-1 ) {  //xmd_ready_gpio not support
          xmd_state = -xmd_state;
      }

	_buf += snprintf(_buf, PAGE_SIZE, "%d\n", xmd_state);

    return (_buf - buf);
}


#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
//extern unsigned int xmd_boot_tty_drv_mode;	//0=>BOOT mode, 1=>NORMAL

/*------------------------------ SYSFS XMD hsi_drvmode ------------------------------*/
//enum {XMD_HSI_DRVMODE_NORM, XMD_HSI_DRVMODE_BOOT, XMD_HSI_DRVMODE_BOOT_SW,XMD_HSI_DRVMODE_BOOT_HW};

#if defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
static int hsi_drvmode = XMD_HSI_DRVMODE_BOOT;	//hsi_drvmode, HSI驱动类型配置值: 0,NORMAL mode, >0,BOOT mode;
#else  //#if defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
static int hsi_drvmode = XMD_HSI_DRVMODE_NORM;	//hsi_drvmode, HSI驱动类型配置值: 0,NORMAL mode, >0,BOOT mode;
#endif  //#if defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)

static DEFINE_MUTEX(hsi_drvmode_mutex);

int get_hsi_drvmode( void ) //XMD_HSI_DRVMODE_NORM, XMD_HSI_DRVMODE_BOOT, ...
{
    int status=0;
    
        mutex_lock(&hsi_drvmode_mutex);        
        status = hsi_drvmode;
        mutex_unlock(&hsi_drvmode_mutex);
        
        return status;
}

static ssize_t xmd_hsi_drvmode_show(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t xmd_hsi_drvmode_store(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(hsi_drvmode, S_IRUGO | S_IWUSR, xmd_hsi_drvmode_show, xmd_hsi_drvmode_store);

static ssize_t xmd_hsi_drvmode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
char *_buf = buf;

    mutex_lock(&hsi_drvmode_mutex);
    if (attr == &dev_attr_hsi_drvmode)
        _buf += snprintf(_buf, PAGE_SIZE, "%d\n", hsi_drvmode);
    mutex_unlock(&hsi_drvmode_mutex);
    return (_buf - buf);
}

static ssize_t xmd_hsi_drvmode_store(struct device *dev,
	                       struct device_attribute *attr, const char *buf, size_t count)
{
    int ret=count,value=0,status=0;
//pr_info("\nXMD: %s. buf=\"%s\", buf=0x%08X, count=%d;\n",__func__,buf,*(int *)buf,count);
    
    mutex_lock(&hsi_drvmode_mutex);
    
    sscanf(buf, "%d", &value);
    pr_info("\nXMD: %s. [hsi_drvmode] %d -> %d;\n",__func__,hsi_drvmode,value);

    if (count > 0) {
        if( (bool)value==(bool)hsi_drvmode ) {
            pr_warn("\nXMD: %s. wrong hsi_drvmode shift!\n",__func__);
        } else {
            switch( value ) { //enum {XMD_HSI_DRVMODE_BOOT, XMD_HSI_DRVMODE_NORM, XMD_HSI_DRVMODE_NORM_SW,XMD_HSI_DRVMODE_NORM_HW};
                case XMD_HSI_DRVMODE_NORM:
                    pr_info("\nXMD: %s. hsi_ll_phy_drv_init start.\n",__func__);
                    status = hsi_ll_phy_drv_init();/*Register with HSI for normal mode*/
                    hsi_drvmode = value;    //hsi_drvmode, HSI驱动类型配置值: 0,NORMAL mode, >0,BOOT mode;
                    break;
                case XMD_HSI_DRVMODE_BOOT:
                case XMD_HSI_DRVMODE_BOOT_SW:
//#y00185015#20110822#HSIswitch.softswitch# XMD_BOOT_FIRMWARE --> XMD_BOOT_OFF/XMD_BOOT_ON --> XMD_BOOT_FIRMWARE
                    { //if (flashless) {
                        pr_info("XMD: %s. hsi_ll_phy_drv_exit start.\n",__func__);
                        hsi_ll_phy_drv_exit();/*unregister with HSI for normal mode*/
                        //xmd_boot_tty_drv_mode = 0; /*0=>BOOT mode, 1=>NORMAL*/   /* Add for Flashless HSIswitch */
                    }
                    hsi_drvmode = value;    //hsi_drvmode, HSI驱动类型配置值: 0,NORMAL mode, >0,BOOT mode;
                    break;
                case XMD_HSI_DRVMODE_BOOT_HW:
//#y00185015#20110822#HSIswitch.hardswitch# XMD_BOOT_FIRMWARE --> XMD_BOOT_ON --> XMD_BOOT_OFF --> XMD_BOOT_FIRMWARE
                    {//if (flashless) {
#if 0   //#if   1   //#else    //#endif
                        pr_info("XMD: %s. hsi_ll_reset\n",__func__);
                            hsi_ll_reset();
#else   //#if   1   //#else    //#endif
                        pr_info("XMD: %s. xmd_hsi_exit start.\n",__func__);
                            xmd_hsi_exit();
#endif   //#if   1   //#else    //#endif
            
                        pr_info("XMD: xmd_ch_init start.\n");
                            xmd_ch_init (); //先关闭，然后再重新初始化；
                        //xmd_boot_tty_drv_mode = 0; //0=>BOOT mode, 1=>NORMAL   /* Add for Flashless HSIswitch */
                    }
                    hsi_drvmode = value;    //hsi_drvmode, HSI驱动类型配置值: 0,NORMAL mode, >0,BOOT mode;
                    break;
                    
                default:
                    ret = -EINVAL;
                    break;
            }
        }
    
        ret = count;
    } else {
        ret = -EINVAL;
    }

    mutex_unlock(&hsi_drvmode_mutex);
    return ret;
}
/*------------------------------ SYSFS XMD hsi_drvmode ------------------------------*/

#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)


ssize_t xmd_boot_modem_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i, status;

	mutex_lock(&boot_state_mutex);

	if (count > 0) {
		for (i = 0; i < ARRAY_SIZE(xmd_boot_state_str); i++) {
			if (strstr(buf, xmd_boot_state_str[i]))
				break;
		}

		if (i < ARRAY_SIZE(xmd_boot_state_str)) {
			xmd_boot_state_change(i);
			status = count;
		} else
			status = -EINVAL;
	} else
		status = -EINVAL;

	mutex_unlock(&boot_state_mutex);
	return status;
}


/*------------------------------ SYSFS BOOT IO ------------------------------*/

static ssize_t xmd_boot_io_show
	(struct device *, struct device_attribute *, char *);
static ssize_t xmd_boot_io_store(struct device *,
	struct device_attribute *, const char *, size_t);
static DEVICE_ATTR(io, S_IRUGO | S_IWUSR, xmd_boot_io_show, xmd_boot_io_store);


struct xmd_boot_io_attr {
	char *name;
	int  pd_gpio_offset;
	int  pd_pinmux_offset;
	int  pd_pinmux_safe_offset;
};

static const struct xmd_boot_io_attr platform_data_io_attr[] =
{
	{"RESET_PMU_REQ",
	offsetof(struct xmd_boot_platform_data, reset_pmu_req_gpio),
	offsetof(struct xmd_boot_platform_data, reset_pmu_req_pinmux),
	offsetof(struct xmd_boot_platform_data, reset_pmu_req_safe_pinmux)},
	{"BASEBAND_RESET",
	offsetof(struct xmd_boot_platform_data, baseband_reset_gpio),
	offsetof(struct xmd_boot_platform_data, baseband_reset_pinmux),
	offsetof(struct xmd_boot_platform_data, baseband_reset_safe_pinmux)},
	{"ON_OFF",
	offsetof(struct xmd_boot_platform_data, on_off_gpio),
	offsetof(struct xmd_boot_platform_data, on_off_pinmux),
	offsetof(struct xmd_boot_platform_data, on_off_safe_pinmux)},
	{"PWR_SUPPLY",
	offsetof(struct xmd_boot_platform_data, pwr_supply_gpio),
	offsetof(struct xmd_boot_platform_data, pwr_supply_pinmux),
	offsetof(struct xmd_boot_platform_data, pwr_supply_safe_pinmux)},
};


enum {XMD_BOOT_IO_ENABLE, XMD_BOOT_IO_SAFE, XMD_BOOT_IO_SET_1, XMD_BOOT_IO_SET_0};

struct xmd_boot_action_attr {
	char *name;
	int  id;
};

static const struct xmd_boot_action_attr action_attr[] =
{
	{"ENABLE", XMD_BOOT_IO_ENABLE},
	{"SAFE",   XMD_BOOT_IO_SAFE},
	{"1",      XMD_BOOT_IO_SET_1},
	{"0",      XMD_BOOT_IO_SET_0}
};

static ssize_t xmd_boot_io_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int i;
	char *_buf = buf;

	_buf += snprintf(_buf, PAGE_SIZE, "XMD IO SYNTAX: <IO> <ACTION>\n");
	_buf += snprintf(_buf, PAGE_SIZE, "\nIO:\n");

	for (i = 0; i < ARRAY_SIZE(platform_data_io_attr); i++) {
		_buf += snprintf(_buf, PAGE_SIZE, "%s\n",
					platform_data_io_attr[i].name);
	}
	_buf += snprintf(_buf, PAGE_SIZE, "\nACTION:\n");
	for (i = 0; i < ARRAY_SIZE(platform_data_io_attr); i++) {
		_buf += snprintf(_buf, PAGE_SIZE, "%s\n",
					action_attr[i].name);
	}
	_buf += snprintf(_buf, PAGE_SIZE, "\n");

	return (_buf - buf);
}


static ssize_t xmd_boot_io_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i, j;

	if (count > 0) {
		for (i = 0; i < ARRAY_SIZE(platform_data_io_attr); i++) {
			if (strstr(buf, platform_data_io_attr[i].name))
				break;
		}
		if (i < ARRAY_SIZE(platform_data_io_attr)) {
			buf += strlen(platform_data_io_attr[i].name);
			for (j = 0; j < ARRAY_SIZE(action_attr); j++) {
				if (strstr(buf, action_attr[j].name))
					break;
			}
			if (j < ARRAY_SIZE(action_attr)) {
				int gpio;
				struct xmm_pad_mux_config pinmux, pinmux_safe;
				struct xmd_boot_platform_data *bpd;

				pr_info("XMD IO: %s %s\n",
						platform_data_io_attr[i].name,
						action_attr[j].name);
				bpd = b->boot_platform_data;
				gpio = *((int*) ((char*) bpd +
					platform_data_io_attr[i].
							pd_gpio_offset));
				pinmux = *((struct xmm_pad_mux_config*)
						((char*) bpd +
						platform_data_io_attr[i].
							pd_pinmux_offset));
				pinmux_safe = *((struct xmm_pad_mux_config*)
						((char*) bpd +
						platform_data_io_attr[i].
							pd_pinmux_safe_offset));


				switch(action_attr[j].id) {
				case XMD_BOOT_IO_ENABLE:
					xmd_omap_mux_init_signal(pinmux);
					break;
				case XMD_BOOT_IO_SAFE:
					xmd_omap_mux_init_signal(pinmux_safe);
					break;
				case XMD_BOOT_IO_SET_1:
					if (gpio != -1)
						gpio_set_value(gpio, 1);
					else
						pr_err("XMD: undef GPIO\n");
					break;
				case XMD_BOOT_IO_SET_0:
					if (gpio != -1)
						gpio_set_value(gpio, 0);
					else
						pr_err("XMD: undef GPIO\n");
					break;
				}

			} else {
				pr_err("XMD: Uknown Action\n");
				return -EINVAL;
			}
		} else {
			pr_err("XMD: Uknown IO\n");
			return -EINVAL;
		}

		return count;
	}
	return -EINVAL;
}


static DEFINE_MUTEX(boot_flashless_mutex);  /* Add for Flashless "flashless" node control */
//module_param(flashless, int, 0444);	/* Delete for Flashless "flashless" node control */
MODULE_PARM_DESC(flashless,
	"Set to true if XMM modem is flashless (firmware required)");


static ssize_t xmd_boot_flashless_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	char *_buf = buf;
    
	mutex_lock(&boot_flashless_mutex);
	_buf += snprintf(_buf, PAGE_SIZE, "%d\n", flashless);

	mutex_unlock(&boot_flashless_mutex);
	return (_buf - buf);
}

static ssize_t xmd_boot_flashless_store(struct device *dev,struct device_attribute *attr,
	                        const char *buf, size_t count)
{
    int value=0;

    mutex_lock(&boot_flashless_mutex);
    sscanf(buf, "%d", &value);
    pr_info("\nXMD: %s. [flashless] %d -> %d;\n",__func__,flashless,value);
    if( value==0 || value==1 ) {
        flashless= value;
        value = count;
    } else {
        value = -EINVAL;
    }
    mutex_unlock(&boot_flashless_mutex);
    return value;
}

//static DEVICE_ATTR(flashless, S_IRUGO, xmd_boot_flashless_show, NULL);
static DEVICE_ATTR(flashless, S_IRUGO | S_IWUSR, xmd_boot_flashless_show, xmd_boot_flashless_store);

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
//void hsi_set_hsimode(int hsimode);
int xmd_set_hsimode(int hsimode);
int xmd_get_hsimode(void);

/*------------------------------ SYSFS XMD BOOT hsimode ------------------------------*/
//extern int boot_hsimode = -1;	//boot_hsimode, HSI模式配置值;
//static DEFINE_MUTEX(boot_hsimode_mutex);

static ssize_t xmd_boot_hsimode_show(struct device *, struct device_attribute *, char *);
static ssize_t xmd_boot_hsimode_store(struct device *,struct device_attribute *, const char *, size_t);
static DEVICE_ATTR(hsimode, S_IRUGO | S_IWUSR, xmd_boot_hsimode_show, xmd_boot_hsimode_store);


static ssize_t xmd_boot_hsimode_store(struct device *dev,
	                       struct device_attribute *attr, const char *buf, size_t count)
{
    int status=count,value=0,hsimode_value=-1;
//pr_info("\nXMD: %s. buf=\"%s\", buf=0x%08X, count=%d;\n",__func__,buf,*(int *)buf,count);
    
    //mutex_lock(&boot_hsimode_mutex);
    
    sscanf(buf, "%x", &value);  //sscanf(buf, "%d", &value);
    hsimode_value = xmd_get_hsimode();
    pr_info("\nXMD: %s. [hsimode_value]0x%08X -> 0x%08X;\n",__func__,hsimode_value,value);

    if (count > 0) {
        if( hsimode_value==value ) {
            pr_warn("XMD: %s. wrong hsimode_value shift, but continue!\n",__func__);
            //mutex_unlock(&boot_hsimode_mutex);
            //return status;
        }
        
        //xmd_boot_hsimode_change(value);   // /*Modify for extend usbswitch function*/
//        xmd_set_hsimode(value);
        if( (bool)(value&XMM_HSI_USE_BOOT)==true ) {  //XMM_HSI_Modem_Flashless, Modem存储类型;
            pr_info("\nXMD: %s. call xmd_set_hsimode(0x%08X);\n",__func__,value);
            xmd_set_hsimode(value);
        } else {
            pr_info("\nXMD: %s. call hsi_ll_set_hsimode(0x%08X);\n",__func__,value);
            hsi_ll_set_hsimode(value);
        }
        //hsi_set_hsimode(value);
        //boot_hsimode = value;    //cawake状态: 0,"DISABLE"; 1,"ENABLE"
    
        status = count;
    } else {
        status = -EINVAL;
    }

    //mutex_unlock(&boot_hsimode_mutex);
    return status;
}

static ssize_t xmd_boot_hsimode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
        int status, hsimode_value=-1;
    
        //mutex_lock(&boot_hsimode_mutex);
    
        if (attr == &dev_attr_hsimode) {
        if( get_hsi_drvmode()==XMD_HSI_DRVMODE_NORM ) { //XMD_HSI_DRVMODE_NORM, XMD_HSI_DRVMODE_BOOT, ...
            pr_info("\nXMD_CMM: %s hsi_drvmode is in XMD_HSI_DRVMODE_NORM now!\n",__func__);
            hsimode_value = hsi_ll_get_hsimode();
        } else
            hsimode_value = xmd_get_hsimode();

            status = snprintf(buf, PAGE_SIZE,"0x%08x\n", hsimode_value);
        } else
            status = -EINVAL;
    
        //mutex_unlock(&boot_hsimode_mutex);
        return status;
}

#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)


/*------------------------------ BOOT DRIVER ------------------------------*/

static int xmd_boot_probe(struct platform_device *);
static int xmd_boot_remove(struct platform_device *);

static struct platform_driver xmd_boot_driver = {
	.probe = xmd_boot_probe,
	.remove = xmd_boot_remove,
	.driver = {
		.name ="xmm_boot",
		.owner = THIS_MODULE,
	},
};


static int xmd_boot_probe(struct platform_device *pdev)
{
	int status;

	pr_info("XMD: boot probe function\n");

	/* Modem has been forced to reset in board-xxx.c */
	b->boot_state = XMD_BOOT_OFF;

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
        /* sysfs entries for "hsimode" node control */
        status = device_create_file(&(pdev->dev), &dev_attr_hsimode);
        if (status) {
            pr_err("XMD: BOOT error creating sysfs entry hsimode\n");
            return status;
        }
#endif

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
            /* sysfs entries for "hsi_drvmode" node control */
            status = device_create_file(&(pdev->dev), &dev_attr_hsi_drvmode);
            if (status) {
                pr_err("XMD: BOOT error creating sysfs entry hsi_drvmode\n");
                return status;
            }
#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)

            /* sysfs entries for "xmd_state" node control */
            status = device_create_file(&(pdev->dev), &dev_attr_xmd_state);
            if (status) {
                pr_err("XMD: BOOT error creating sysfs entry xmd_state\n");
                return status;
            }

            /* sysfs entries for "cp_shutdown" node control */
            status = device_create_file(&(pdev->dev), &dev_attr_cp_shutdown);
            if (status) {
                pr_err("XMD: BOOT error creating sysfs entry cp_shutdown\n");
                return status;
            }

	/* sysfs entries for IO control */
	status = device_create_file(&(pdev->dev), &dev_attr_io);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry io\n");
		return status;
	}

	/* sysfs entries for boot control */
	status = device_create_file(&pdev->dev, &dev_attr_state);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry state\n");
		return status;
	}
	status = device_create_file(&pdev->dev, &dev_attr_flashless);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry flashless\n");
		return status;
	}

	/* RESET2_N worker */
	INIT_WORK(&b->reset2_work, xmd_boot_reset2_work);

	/* XMD_READY_N worker */
	INIT_WORK(&b->xmd_ready_work, xmd_boot_xmd_ready_work);

	if (flashless)
		pr_info("XMD: ### Flash-less modem\n");
	else
		pr_info("XMD: ### Flash modem\n");

	return 0;
}


static int xmd_boot_remove(struct platform_device *pdev)
{
	/* Disgraceful modem switch off */
	xmd_board_power_off();

	/* sysfs entries for IO control */
	device_remove_file(&(pdev->dev), &dev_attr_io);

	/* sysfs entries for boot control */
	device_remove_file(&pdev->dev, &dev_attr_state);
	device_remove_file(&pdev->dev, &dev_attr_flashless);

	return 0;
}


static int __init xmd_boot_init(void)
{
	int status = 0;

	status = platform_driver_register(&xmd_boot_driver);
	if (status < 0){
		pr_err("Failed to register XMD BOOT driver");
		return status;
	}
	return 0;
}

static void __exit xmd_boot_exit(void)
{
	platform_driver_unregister(&xmd_boot_driver);
}

static int __init xmd_module_init(void)
{
	int status = 0;

        xmd_netlink_init();

	/* Upper layers initialization */
	xmd_boot_init();
#ifdef CONFIG_OMAP4_XMM_SPI
	xmd_spi_init();
	xmd_mux_init();
	xmd_smd_init();
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	xmd_hsi_init();
#endif

#if	defined(CONFIG_HSI_FLASHLESS_SUPPORT) && !defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
	printk("%s. hsi_ll_phy_drv_init start\n",__func__);
    hsi_ll_phy_drv_init();  /*Register with HSI for normal mode*/
#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT) && !defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)

#ifdef CONFIG_OMAP4_XMM_C2C
	xmd_c2c_init();
#endif

	return status;
}

static void __exit xmd_module_exit(void)
{
	netlink_kernel_release(g_nlfd);
	/* Upper layers deinitialization */
#ifdef CONFIG_OMAP4_XMM_SPI
	xmd_smd_exit();
	xmd_mux_exit();
	xmd_spi_exit();
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	xmd_hsi_exit();
#endif
#ifdef CONFIG_OMAP4_XMM_C2C
	xmd_c2c_exit();
#endif

	xmd_boot_exit();
}

#ifdef MODULE
module_init(xmd_module_init);
#else
/* HSI devices to be initialized first */
late_initcall(xmd_module_init);
#endif
module_exit(xmd_module_exit);

MODULE_LICENSE("GPL");
