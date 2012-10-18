/* switch_usb.c
 *
 * Copyright (C) 2011 Huawei Terminal Equipment, Inc.
 * Author: xiehaitao <xiehaitao@huawei.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

#include <linux/gpio.h>

#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gfp.h>
#include <asm/uaccess.h>
#define USB_SWITCH_SET_VBUS_VALID   1
#define USB_SWITCH_SET_VBUS_INVALID 2
#define USB_SWITCH_CALL_ISR_PROC    0

extern int omap_mux_init_gpio(int gpio, int val);

#define USB_SWITCH_CONTROL_GPIO     34
#define USB_SWITCH_EN_GPIO          163
#define USB_SWITCH_INTERRUPT_GPIO     2
#define USB_SWITCH_DELAY_MILLISECOND    200

#define SWITCH_CONTROL    0                /*gpio34  */
#define SWITCH_OE        1                /*gpio163  */

#define GPIO_HI            1
#define    GPIO_LOW        0

#define USB_AP             0
#define USB_MODEM         1
#define USB_OFF            2

#define USB_MODEM_ON    3   //USB切给Modem，且Modem置"ON"；
#define USB_MODEM_OFF   4   //USB切给Modem，且Modem置"OFF"；
#define USB_MODEM_RESTART   5   //USB切给Modem，且Modem重启后置"ON"；

//#define USB_MODEM_SHIFT    6X   //Modem状态切换；
#define USB_MODEM_OFFTOON_RETURN   7   //USB切给Modem，且Modem重启后置"ON"，然后ssleep(30)后再切回AP；
#define USB_MODEM_OFFTOFIRMWARE_RETURN   8   //USB切给Modem，且Modem重启后置"FIRMWARE"，然后ssleep(30)后再切回AP；
//#define USB_MODEM_FROM_TO_RETURN_TIMEOUT  9XYZZZ    //表示USB切给CP侧，MODEM由X状态转换为Y状态，然后等待ZZZ秒后，再切回AP侧；

static struct class *usb_switch_class;
static struct workqueue_struct    *kusbswitch_wq;
static irqreturn_t switch_interrupt_handler(void);
static unsigned int usb_irq = 0;
#define USB_IRQ_NOT_REQUESTED 0
#define USB_IRQ_REQUEST_SUCCESS 1
static int usb_irq_request_flag = USB_IRQ_NOT_REQUESTED;

struct usb_switch_device{
    const char    *name;
    struct device    *dev;
    int        index;
    int        state;
};

static struct usb_switch_device usb_switch = {
    .name = "usbsw",
    .index = 0,
    .state = USB_OFF,
};

static void set_usb_switch_gpio(int gpio, int state)
{
    switch ( gpio )
    {
        case SWITCH_CONTROL :
            gpio_set_value(USB_SWITCH_CONTROL_GPIO,state);
            break;
        case SWITCH_OE :
            gpio_set_value(USB_SWITCH_EN_GPIO,state);
            break;
        default:
            break;
    }
    return;
}

int __init init_usb_switch_gpio(void)
{
    int status;

    /* gpio mux config */
    omap_mux_init_gpio(USB_SWITCH_CONTROL_GPIO,0);
    omap_mux_init_gpio(USB_SWITCH_EN_GPIO,0);

    /* Request of GPIO lines */
    status = gpio_request(USB_SWITCH_CONTROL_GPIO, "usb_switch");
    if (status) {
        pr_err("Cannot request GPIO %d\n", USB_SWITCH_CONTROL_GPIO);
        return status;
    }

    status = gpio_request(USB_SWITCH_EN_GPIO, "usb_sw_en");
    if (status) {
        pr_err("Cannot request GPIO %d\n", USB_SWITCH_EN_GPIO);
        goto error1;
    }

    /* Configuration of requested GPIO lines */

    status = gpio_direction_output(USB_SWITCH_CONTROL_GPIO, 1);
    if (status) {
        pr_err("Cannot set output GPIO %d\n", USB_SWITCH_CONTROL_GPIO);
        goto error2;
    }

    status = gpio_direction_output(USB_SWITCH_EN_GPIO, 1);
    if (status) {
        pr_err("Cannot set output GPIO %d\n", USB_SWITCH_EN_GPIO);
        goto error2;
    }

    /* init gpio34, gpio163
    // GPIO34  -- to control USB switch
    //        - High -> AP
    //        - Low  -> Modem
    // GPIO163  -- Usb Switch OE
    //        - High -> USB_OFF
    //        - Low  -> USB_ON
    // for initial state, set GPIO34 = High, GPIO163 = High
    */
    usb_switch.state = USB_AP;
    set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);
    set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);

    return 0;

error2:
    gpio_free(USB_SWITCH_EN_GPIO);
error1:
    gpio_free(USB_SWITCH_CONTROL_GPIO);

    return status;
}

//extern int xmd_boot_state_change(int boot_state);   
enum {XMD_BOOT_OFF, XMD_BOOT_FIRMWARE, XMD_BOOT_ON};
extern const char *xmd_boot_state_str[];   

#ifdef CONFIG_MACH_OMAP_XMM
extern ssize_t xmd_boot_modem_state_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count);
#else  //#ifdef CONFIG_MACH_OMAP_XMM
    #define xmd_boot_modem_state_store(...)    {}
#endif  //#ifdef CONFIG_MACH_OMAP_XMM


static int usb_switch_gpio_request_irq(void)
{
    int result;

    if((result = request_irq(usb_irq, (irq_handler_t)switch_interrupt_handler,0, "switch", NULL)))
    {
        printk(KERN_INFO"[FAILED: Cannot register switch_Interrupt!]\n");
        return -EBUSY;
    }
    else
    {
        usb_irq_request_flag = USB_IRQ_REQUEST_SUCCESS;
        printk("[request irq OK]\n");
    }

    return 0;
}

static int usb_switch_gpio_free_irq(void)
{
    if(USB_IRQ_REQUEST_SUCCESS == usb_irq_request_flag)
    {
        usb_irq_request_flag = USB_IRQ_NOT_REQUESTED;
        free_irq(usb_irq,NULL);
    }

    return 0;
}

static int usb_send_at_cmd(int cmd)
{
    int ret;
    char buf[32] = "AT+USBSWITCH=";
    char buf_ack[40] = "0";                       /*AT Command ack buf*/
    struct file * filp = NULL;
    int read_times = 2;
    static const int READ_SIZE = 20;

    mm_segment_t fs = get_fs();
    set_fs(KERNEL_DS);

    sprintf(buf+strlen(buf), "%d\r", cmd);

    filp = filp_open(CONFIG_SWITCH_USB_TTY_NAME, O_WRONLY|O_RDWR, 0644);
    if(IS_ERR(filp))
    {
        pr_err("failed to open %s", CONFIG_SWITCH_USB_TTY_NAME);
        return -EIO;
    }

    if(filp->f_op != NULL && filp->f_op->write != NULL)
    {
        ret = filp->f_op->write(filp, buf, strlen(buf), &filp->f_pos);
    }

    /*loop for waiting ack OK, after this ,close the channel*/
    while(read_times)
    {
        read_times--;

        filp->f_op->read(filp,buf_ack,READ_SIZE,&filp->f_pos);

        if(strstr(buf_ack,"OK"))
        {
            break;
        }
    }

    set_fs(fs);
    filp_close(filp, NULL);

    return 0;
}

static void setUsbSwitch(int value)
{
    switch(value)
    {
        case USB_AP:
            if ( usb_switch.state == USB_AP )
                return;
            usb_switch.state = USB_AP;
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
            usb_send_at_cmd(USB_SWITCH_SET_VBUS_INVALID);
            usb_send_at_cmd(USB_SWITCH_CALL_ISR_PROC);
            break;

        case USB_MODEM:
            if ( usb_switch.state == USB_MODEM )
                return;
            usb_switch.state = USB_MODEM;
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
            usb_send_at_cmd(USB_SWITCH_SET_VBUS_VALID);
            usb_send_at_cmd(USB_SWITCH_CALL_ISR_PROC);
            usb_switch_gpio_request_irq();
            break;

        case USB_OFF:
            if ( usb_switch.state == USB_OFF )
                return;
            usb_switch.state = USB_OFF;
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            ssleep(2);
            break;

        case USB_MODEM_ON:
        //    if ( usb_switch.state == USB_MODEM )
        //        return;
            usb_switch.state = USB_MODEM;
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw

            //xmd_boot_state_change(2);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[2],sizeof(xmd_boot_state_str[2]));
            break;

        case USB_MODEM_OFF:
      //      if ( usb_switch.state == USB_MODEM )
      //          return;
            usb_switch.state = USB_MODEM;
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
            //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[0],sizeof(xmd_boot_state_str[0]));
            break;

        case USB_MODEM_RESTART:
        //    if ( usb_switch.state == USB_MODEM )
        //        return;
            usb_switch.state = USB_MODEM;
        printk("USB switch to modem,and modem restart!\n");
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
        //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
        xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[0],sizeof(xmd_boot_state_str[0]));
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw

            ssleep(2);
        //xmd_boot_state_change(2);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
        xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[2],sizeof(xmd_boot_state_str[2]));
            break;

        case USB_MODEM_OFFTOON_RETURN:
        //    if ( usb_switch.state == USB_MODEM )
        //        return;
        //    usb_switch.state = USB_MODEM;
        pr_info("USB_MODEM_OFFTOON_RETURN begin!\n");
        //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
        xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[0],sizeof(xmd_boot_state_str[0]));
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw

            ssleep(2);
        //xmd_boot_state_change(2);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
        xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[2],sizeof(xmd_boot_state_str[2]));
        ssleep(30);

            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
        pr_info("USB_MODEM_OFFTOON_RETURN end!\n");
            break;

        case USB_MODEM_OFFTOFIRMWARE_RETURN:
        //    if ( usb_switch.state == USB_MODEM )
        //        return;
        //    usb_switch.state = USB_MODEM;
        pr_info("USB_MODEM_OFFTOFIRMWARE_RETURN begin!\n");
        //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
        xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[0],sizeof(xmd_boot_state_str[0]));
            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw

            ssleep(2);
        //xmd_boot_state_change(1);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
        xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[1],sizeof(xmd_boot_state_str[1]));
        ssleep(30);

            set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
            set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
            ssleep(2);
            set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
        pr_info("USB_MODEM_OFFTOFIRMWARE_RETURN end!\n");
            break;
        default:
    {
//#define USB_MODEM_SHIFT_STATE 6X   //6X    Modem状态切换，X: 0,1,2代表{"OFF","FIRMWARE","ON"}三种状态；
        if( (value>=60)&&(value<=62) ) {   //#define USB_MODEM_SHIFT_STATE   6X   //y00185015 20110822
            pr_info("USB_MODEM_SHIFT_STATE begin!\n");
            //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            //xmd_boot_state_change(value-60);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[value-60],sizeof(xmd_boot_state_str[value-60]));
            pr_info("USB_MODEM_SHIFT_STATE end!\n");
        }
//#define USB_MODEM_SHIFT_RETURN   6XXX   //6XXX    功能类似USB_MODEM；USB切给Modem，然后ssleep(XXX)后再切回AP；XXX指定USB切回时间；
        else if( (value>=6000)&&(value<7000) ) {   //#define USB_MODEM_SHIFT_RETURN   6XXX   //y00185015 20110822
            pr_info("USB_MODEM_SHIFT_RETURN begin!\n");
                set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
                ssleep(2);
                set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
                usb_send_at_cmd(USB_SWITCH_SET_VBUS_VALID);
                usb_send_at_cmd(USB_SWITCH_CALL_ISR_PROC);
            ssleep(value-6000);

                set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
                ssleep(2);
                set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
                usb_send_at_cmd(USB_SWITCH_SET_VBUS_INVALID);
                usb_send_at_cmd(USB_SWITCH_CALL_ISR_PROC);
            pr_info("USB_MODEM_SHIFT_RETURN end!\n");
        }
//#define USB_MODEM_OFFTOON_RETURN_TIMEOUT   7XXX   //USB切给Modem，且Modem重启后置"ON"，然后ssleep(XXX)后再切回AP；XXX指定USB切回时间；
        else if( (value>=7000)&&(value<8000) ) {    //case USB_MODEM_OFFTOON_RETURN:
            //    if ( usb_switch.state == USB_MODEM )
            //        return;
            //    usb_switch.state = USB_MODEM;
            pr_info("USB_MODEM_OFFTOON_RETURN_TIMEOUT begin!\n");
            //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[0],sizeof(xmd_boot_state_str[0]));
                set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
                ssleep(2);
                set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw

                ssleep(2);
            //xmd_boot_state_change(2);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[2],sizeof(xmd_boot_state_str[2]));
            ssleep(value-7000);

                set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
                ssleep(2);
                set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
            pr_info("USB_MODEM_OFFTOON_RETURN_TIMEOUT end!\n");
        }
            //break;
//#define USB_MODEM_OFFTOFIRMWARE_RETURN_TIMEOUT  8XXX  //功能类似USB_MODEM_OFFTOFIRMWARE_RETURN；USB切给Modem，
//且Modem重启后置"FIRMWARE"，然后ssleep(XXX)后再切回AP；XXX指定USB切回时间；
        else if( (value>=8000)&&(value<9000) ) {   //case USB_MODEM_OFFTOFIRMWARE_RETURN:
            //    if ( usb_switch.state == USB_MODEM )
            //        return;
            //    usb_switch.state = USB_MODEM;
            pr_info("USB_MODEM_OFFTOFIRMWARE_RETURN_TIMEOUT begin!\n");
            //xmd_boot_state_change(0);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[0],sizeof(xmd_boot_state_str[0]));
                set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
                ssleep(2);
                set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw

                ssleep(2);
            //xmd_boot_state_change(1);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
            xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[1],sizeof(xmd_boot_state_str[1]));
            ssleep(value-8000);

                set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
                ssleep(2);
                set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
            pr_info("USB_MODEM_OFFTOFIRMWARE_RETURN_TIMEOUT end!\n");
        }
//#define USB_MODEM_FROM_TO_RETURN_TIMEOUT    9XYZZZ    //表示USB切给CP侧，MODEM由X状态转换为Y状态，然后等待ZZZ秒后，再切回AP侧；
        else if( (value>=900000)&&(value<=1000000) ) {
            int state;
                //    if ( usb_switch.state == USB_MODEM )
                //        return;
                //    usb_switch.state = USB_MODEM;
                pr_info("USB_MODEM_FROM_TO_RETURN_TIMEOUT begin!\n");
                state = value%100000/10000;
                if( state<=2 )
                    //xmd_boot_state_change(state);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
                    xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[state],sizeof(xmd_boot_state_str[state]));
                state = value%1000;
                if( state!=0 ) {
                    set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                    set_usb_switch_gpio(SWITCH_CONTROL, GPIO_LOW);  //sw to cp
                    ssleep(2);
                    set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
                }
            state = value%10000/1000;
            if( state<=2 ) {
                    ssleep(2);
                //xmd_boot_state_change(state);   //static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
                xmd_boot_modem_state_store(NULL,NULL,xmd_boot_state_str[state],sizeof(xmd_boot_state_str[state]));
            }

            state = value%1000;
            if( state!=0 && state!=999  ) {
                ssleep(state);

                    set_usb_switch_gpio(SWITCH_OE, GPIO_HI);    //close sw
                    set_usb_switch_gpio(SWITCH_CONTROL, GPIO_HI);   //sw to ap
                    ssleep(2);
                    set_usb_switch_gpio(SWITCH_OE, GPIO_LOW);   //enable sw
            }
            pr_info("USB_MODEM_FROM_TO_RETURN_TIMEOUT end!\n");
        }
            //break;
    }
            break;
    }
}

static ssize_t swstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", usb_switch.state);
}

static ssize_t swstatus_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int    value;

    sscanf(buf, "%d", &value);

    setUsbSwitch(value);
    return size;
}

static DEVICE_ATTR(swstate, S_IRUGO | S_IWUSR, swstatus_show, swstatus_store);

static int create_usb_switch_class(void)
{
    if (!usb_switch_class)
    {
        usb_switch_class = class_create(THIS_MODULE, "usbswitch");
        if (IS_ERR(usb_switch_class))
            return PTR_ERR(usb_switch_class);
    }
    return 0;
}

static void usb_switch_do(struct work_struct *work)
{
    /* waiting CP side [OK] return */
    msleep(USB_SWITCH_DELAY_MILLISECOND);

    /*USB connect to AP*/
    setUsbSwitch(USB_AP);

    /*free workqueue*/
    if(work)
    {
        printk(KERN_INFO "%s:kfree [%x]\n",__func__,(unsigned int)work);
        kfree(work);
    }

    /* free irq */
    usb_switch_gpio_free_irq();

}

irqreturn_t switch_interrupt_handler(void)
{
    struct work_struct *work;
    /* disable irq interrupt*/
    disable_irq_nosync(usb_irq);
    /*workqueue call*/
    work = kzalloc(sizeof(struct work_struct), GFP_ATOMIC);
    if (!work)
    {
        printk(KERN_WARNING
                   "%s: switch unable to allocate memory for "
                   "load of sub-modules.\n", __func__);
    }
    else
    {
        printk(KERN_INFO "%s:kzlloc [%x]\n",__func__,(unsigned int)work);
        printk(KERN_INFO "switch loading update usb_switch_do\n");
        INIT_WORK(work, usb_switch_do);
        /*schedule_work(work);*/
        queue_work(kusbswitch_wq, work);
    }

    return IRQ_HANDLED;
}

int __init init_usb_switch_gpio_interrupt(void)
{
    int status;

    /* Request of GPIO lines */
    status = gpio_request(USB_SWITCH_INTERRUPT_GPIO, "usb_sw_interrupt");
    if (status)
    {
        pr_err("Cannot request GPIO %d\n", USB_SWITCH_INTERRUPT_GPIO);
        return status;
    }

    status = gpio_direction_input(USB_SWITCH_INTERRUPT_GPIO);
    if (status)
    {
        pr_err("Cannot set output GPIO %d\n", USB_SWITCH_INTERRUPT_GPIO);
        goto error1;
    }

    usb_irq = gpio_to_irq(USB_SWITCH_INTERRUPT_GPIO);
    //set_irq_type(usb_irq,IRQ_TYPE_LEVEL_HIGH);
    irq_set_irq_type(usb_irq,IRQ_TYPE_LEVEL_HIGH);

    return 0;

    error1:
        gpio_free(USB_SWITCH_INTERRUPT_GPIO);

    return status;

}

void __init init_usb_switch(void)
{
    int ret;

    ret = init_usb_switch_gpio();

    if(ret < 0)
    {
        printk(KERN_ERR"init_usb_switch: Failed to initialize GPIOs for usb switch\n");
        return;
    }

    ret = init_usb_switch_gpio_interrupt();
    if(ret < 0)
    {
        printk(KERN_ERR"init_usb_switch_interrupt: Failed to create usb switch interrupt\n");
        return;
    }

    ret = create_usb_switch_class();
    if(ret < 0)
    {
        printk(KERN_ERR"init_usb_switch: Failed to create usb switch class\n");
        return;
    }

    usb_switch.dev = device_create(usb_switch_class, NULL, MKDEV(0, usb_switch.index), NULL, usb_switch.name);

    if (IS_ERR(usb_switch.dev))
    {
        printk(KERN_ERR"init_usb_switch: Failed to create device\n");
        return;
    }

    ret = device_create_file(usb_switch.dev, &dev_attr_swstate);

    if (ret < 0)
    {
        device_destroy(usb_switch_class, MKDEV(0, usb_switch.index));
        printk(KERN_ERR "init_usb_switch: Failed to create device file\n");
    }
    kusbswitch_wq = create_workqueue("kusbswitch_wq");
}

MODULE_DESCRIPTION("usb switch driver");
MODULE_LICENSE("GPL v2");

