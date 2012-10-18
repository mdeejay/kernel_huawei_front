/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/leds.h>

static inline void led_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	if (value > led_cdev->max_brightness)
		value = led_cdev->max_brightness;
	led_cdev->brightness = value;
	if (!(led_cdev->flags & LED_SUSPENDED))
		led_cdev->brightness_set(led_cdev, value);
}

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

#ifdef CONFIG_LEDS_TRIGGERS
void led_trigger_set_default(struct led_classdev *led_cdev);
void led_trigger_set(struct led_classdev *led_cdev,
			struct led_trigger *trigger);
void led_trigger_remove(struct led_classdev *led_cdev);

static inline void *led_get_trigger_data(struct led_classdev *led_cdev)
{
	return led_cdev->trigger_data;
}

#else
#define led_trigger_set_default(x) do {} while (0)
#define led_trigger_set(x, y) do {} while (0)
#define led_trigger_remove(x) do {} while (0)
#define led_get_trigger_data(x) (NULL)
#endif

ssize_t led_trigger_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_trigger_show(struct device *dev, struct device_attribute *attr,
			char *buf);

#define GPTIMER8_CLKCTRL       0x4a004580
#define GPTIMER10_CLKCTRL      0x4a009428
#define GPTIMER_CLKCTRL_VAL  0x01000002

#define GPT_8_ADDR                  0x4013e000
#define GPT_TLDR_8_OFFSET    0x40
#define GPT_TMAR_8_OFFSET    0x4c
#define GPT_TCLR_8_OFFSET    0x38
#define GPT_TCRR_8_OFFSET    0x3c

#define GPT_10_ADDR                  0x48086000
#define GPT_TLDR_10_OFFSET    0x2c
#define GPT_TMAR_10_OFFSET    0x38
#define GPT_TCLR_10_OFFSET    0x24
#define GPT_TCRR_10_OFFSET    0x28

#define GPT_TCLR_VAL                 0x00001843
#define GPT_TCRR_VAL                 0xfffffffe
#define MEM_LONGTH                   0x150


#define raw_reg_readl(a)    (*(volatile unsigned int *)(a))
#define raw_reg_writel(v,a) (*(volatile unsigned int *)(a) = (v))
#define raw_reg_readw(a)    (*(volatile unsigned short *)(a))
#define raw_reg_writew(v,a) (*(volatile unsigned short *)(a) = (v))
#endif	/* __LEDS_H_INCLUDED */
