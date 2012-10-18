/*
 * LED Kernel Timer Trigger
 *
 * Copyright 2005-2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/* all the file use the g4 version probe */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include "leds.h"
#include <linux/delay.h>

#include <linux/io.h>
#include <hsad/config_interface.h>

struct timer_trig_data {
	int brightness_on;		/* LED brightness during "on" period.
					 * (LED_OFF < brightness_on <= LED_FULL)
					 */
	unsigned long delay_on;		/* milliseconds on */
	unsigned long delay_off;	/* milliseconds off */
	struct timer_list timer;
};

static void led_timer_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *) data;
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long brightness;
	unsigned long delay;

	if (!timer_data->delay_on || !timer_data->delay_off) {
		led_set_brightness(led_cdev, LED_OFF);
		return;
	}

	brightness = led_get_brightness(led_cdev);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = timer_data->brightness_on;
		delay = timer_data->delay_on;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		timer_data->brightness_on = brightness;
		brightness = LED_OFF;
		delay = timer_data->delay_off;
	}
	led_set_brightness(led_cdev, brightness);

	mod_timer(&timer_data->timer, jiffies + msecs_to_jiffies(delay));
}

static ssize_t led_delay_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;

	return sprintf(buf, "%lu\n", timer_data->delay_on);
}

static void led_set_red_delay(unsigned long ontime,unsigned long offtime)
{
    void __iomem *phymux_base = NULL;
    void __iomem *phymux_base1 = NULL;

    phymux_base1 = ioremap(GPTIMER8_CLKCTRL,0x4);
    raw_reg_writel(GPTIMER_CLKCTRL_VAL,phymux_base1);
    iounmap(phymux_base1);
    udelay(150);
    phymux_base = ioremap(GPT_8_ADDR, MEM_LONGTH);
    raw_reg_writel(0xfffffffe,phymux_base+GPT_TCRR_8_OFFSET);//TCRR
    udelay(300);
    raw_reg_writel(0x00001842,phymux_base+GPT_TCLR_8_OFFSET);//GPT_TCLR   start
    if((offtime == 0)|(ontime == 0))
    {
        raw_reg_writel(0x01,phymux_base+GPT_TLDR_8_OFFSET);//GPT_TLDR
        raw_reg_writel(0x02,phymux_base+GPT_TMAR_8_OFFSET);//GPT_TMAR
    }
    else
    {
        raw_reg_writel(0xffffffff-(offtime+ontime)*32,phymux_base+GPT_TLDR_8_OFFSET);//GPT_TLDR
        raw_reg_writel(0xffffffff-offtime*32,phymux_base+GPT_TMAR_8_OFFSET);//GPT_TMAR
    }
    raw_reg_writel(GPT_TCLR_VAL,phymux_base+GPT_TCLR_8_OFFSET);//GPT_TCLR   start
    raw_reg_writel(GPT_TCRR_VAL,phymux_base+GPT_TCRR_8_OFFSET);//TCRR
    iounmap(phymux_base);

}
static void led_set_green_delay(unsigned long ontime,unsigned long offtime)
{
    void __iomem *phymux_base = NULL;
    void __iomem *phymux_base1 = NULL;

    phymux_base1 = ioremap(GPTIMER10_CLKCTRL,0x4);
    raw_reg_writel(GPTIMER_CLKCTRL_VAL,phymux_base1);
    iounmap(phymux_base1);
    udelay(150);
    phymux_base = ioremap(GPT_10_ADDR, MEM_LONGTH);
    raw_reg_writel(0xfffffffe,phymux_base+GPT_TCRR_10_OFFSET);//TCRR
    udelay(300);
    raw_reg_writel(0x00001842,phymux_base+GPT_TCLR_10_OFFSET);//GPT_TCLR  start
    if((offtime == 0)|(ontime == 0))
    {
        raw_reg_writel(0x01,phymux_base+GPT_TLDR_10_OFFSET);//GPT_TLDR
        raw_reg_writel(0x02,phymux_base+GPT_TMAR_10_OFFSET);//GPT_TMAR
    }
    else
    {
        raw_reg_writel(0xffffffff-(offtime+ontime)*32,phymux_base+GPT_TLDR_10_OFFSET);//GPT_TLDR
        raw_reg_writel(0xffffffff-offtime*32,phymux_base+GPT_TMAR_10_OFFSET);//GPT_TMAR
    }
    raw_reg_writel(GPT_TCLR_VAL,phymux_base+GPT_TCLR_10_OFFSET);//GPT_TCLR  start
    raw_reg_writel(GPT_TCRR_VAL,phymux_base+GPT_TCRR_10_OFFSET);//TCRR
    iounmap(phymux_base);


}

static ssize_t led_delay_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;


	if (isspace(*after))
		count++;

	if (count == size) {
			timer_data->delay_on = state;
        switch(get_board_id_int()){
        case BOARD_ID_VIVA:
        case BOARD_ID_FRONT:
        {
		/* deactivate previous settings */
		del_timer_sync(&timer_data->timer);

		/* try to activate hardware acceleration, if any */
		if (!led_cdev->blink_set ||
		    led_cdev->blink_set(led_cdev,
		      &timer_data->delay_on, &timer_data->delay_off)) {
			/* no hardware acceleration, blink via timer */
			mod_timer(&timer_data->timer, jiffies + 1);
		}
		break;
        }
        case BOARD_ID_FRONTT1P5:
        {
                if(!strcmp(led_cdev->name,"red"))           //dmtimer8_pwm_evt  gpio27
                {
			led_set_red_delay(timer_data->delay_on , timer_data->delay_off);
                }
                else if(!strcmp(led_cdev->name,"green")) //dmtimer10_pwm_evt gpio190
                {
			led_set_green_delay(timer_data->delay_on , timer_data->delay_off);
                }
                else if(!strcmp(led_cdev->name,"blue")) //blue light use gpio after front T1.5
                {
                        del_timer_sync(&timer_data->timer);

                        if (!led_cdev->blink_set ||
                           led_cdev->blink_set(led_cdev,
                              &timer_data->delay_on, &timer_data->delay_off)) {
                        mod_timer(&timer_data->timer, jiffies + 1);
                        }
                }
                break;
        }
        default:
        {
                if(!strcmp(led_cdev->name,"red"))           //dmtimer8_pwm_evt  gpio27
                {
			led_set_red_delay(timer_data->delay_on , timer_data->delay_off);
                }
                else if(!strcmp(led_cdev->name,"green")) //dmtimer10_pwm_evt gpio190
                {
			led_set_green_delay(timer_data->delay_on , timer_data->delay_off);
                }
                break;
        }
        }
		ret = count;
	}

	return ret;
}

static ssize_t led_delay_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;

	return sprintf(buf, "%lu\n", timer_data->delay_off);
}

static ssize_t led_delay_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	int ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;


	if (isspace(*after))
		count++;

	if (count == size) {
			timer_data->delay_off = state;
        switch(get_board_id_int()){
        case BOARD_ID_VIVA:
        case BOARD_ID_FRONT:
        {
		/* deactivate previous settings */
		del_timer_sync(&timer_data->timer);

		/* try to activate hardware acceleration, if any */
		if (!led_cdev->blink_set ||
		    led_cdev->blink_set(led_cdev,
		      &timer_data->delay_on, &timer_data->delay_off)) {
			/* no hardware acceleration, blink via timer */
			mod_timer(&timer_data->timer, jiffies + 1);
		}
		break;
        }
        case BOARD_ID_FRONTT1P5:
        {
                if(!strcmp(led_cdev->name,"red"))           //dmtimer8_pwm_evt  gpio27
                {
			led_set_red_delay(timer_data->delay_on , timer_data->delay_off);
                }
                else if(!strcmp(led_cdev->name,"green"))//dmtimer10_pwm_evt gpio190)
                {
			led_set_green_delay(timer_data->delay_on , timer_data->delay_off);
                }
                else if(!strcmp(led_cdev->name,"blue"))//blue light use gpio only after front T1.5
                {
                        del_timer_sync(&timer_data->timer);
                        if (!led_cdev->blink_set ||
                            led_cdev->blink_set(led_cdev,
                              &timer_data->delay_on, &timer_data->delay_off)) {
                        mod_timer(&timer_data->timer, jiffies + 1);
                }
                }
        }
        break;
        default:
        {
                if(!strcmp(led_cdev->name,"red"))           //dmtimer8_pwm_evt  gpio27
                {
			led_set_red_delay(timer_data->delay_on , timer_data->delay_off);
                }
                else if(!strcmp(led_cdev->name,"green"))//dmtimer10_pwm_evt gpio190)
                {
			led_set_green_delay(timer_data->delay_on , timer_data->delay_off);
                }
                break;
        }
        }
		ret = count;
	}

	return ret;
}

static DEVICE_ATTR(delay_on, 0666, led_delay_on_show, led_delay_on_store);
static DEVICE_ATTR(delay_off, 0666, led_delay_off_show, led_delay_off_store);
static void timer_trig_activate(struct led_classdev *led_cdev)
{
	struct timer_trig_data *timer_data;
	int rc;

	timer_data = kzalloc(sizeof(struct timer_trig_data), GFP_KERNEL);
	if (!timer_data)
		return;

	timer_data->brightness_on = led_get_brightness(led_cdev);
	if (timer_data->brightness_on == LED_OFF)
		timer_data->brightness_on = led_cdev->max_brightness;
	led_cdev->trigger_data = timer_data;

	init_timer(&timer_data->timer);
	timer_data->timer.function = led_timer_function;
	timer_data->timer.data = (unsigned long) led_cdev;

	rc = device_create_file(led_cdev->dev, &dev_attr_delay_on);
	if (rc)
		goto err_out;
	rc = device_create_file(led_cdev->dev, &dev_attr_delay_off);
	if (rc)
		goto err_out_delayon;

	/* If there is hardware support for blinking, start one
	 * user friendly blink rate chosen by the driver.
	 */
	if (led_cdev->blink_set)
		led_cdev->blink_set(led_cdev,
			&timer_data->delay_on, &timer_data->delay_off);

	return;

err_out_delayon:
	device_remove_file(led_cdev->dev, &dev_attr_delay_on);
err_out:
	led_cdev->trigger_data = NULL;
	kfree(timer_data);
}

static void timer_trig_deactivate(struct led_classdev *led_cdev)
{
	struct timer_trig_data *timer_data = led_cdev->trigger_data;
	unsigned long on = 0, off = 0;

	if (timer_data) {
		device_remove_file(led_cdev->dev, &dev_attr_delay_on);
		device_remove_file(led_cdev->dev, &dev_attr_delay_off);
		del_timer_sync(&timer_data->timer);
		kfree(timer_data);
	}

	/* If there is hardware support for blinking, stop it */
	if (led_cdev->blink_set)
		led_cdev->blink_set(led_cdev, &on, &off);
}

static struct led_trigger timer_led_trigger = {
	.name     = "timer",
	.activate = timer_trig_activate,
	.deactivate = timer_trig_deactivate,
};

static int __init timer_trig_init(void)
{
	return led_trigger_register(&timer_led_trigger);
}

static void __exit timer_trig_exit(void)
{
	led_trigger_unregister(&timer_led_trigger);
}

module_init(timer_trig_init);
module_exit(timer_trig_exit);

MODULE_AUTHOR("Richard Purdie <rpurdie@openedhand.com>");
MODULE_DESCRIPTION("Timer LED trigger");
MODULE_LICENSE("GPL");
