/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>

#undef  KEYMAP_INDEX
#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(huawei_keypad_col_gpios) + (col))

/*                                          HUAWEI keypad begin                                                                 */
//static unsigned int huawei_keypad_row_gpios[] = { 42, 43, 178, 177, 176, 175};
static unsigned int huawei_keypad_row_gpios[] = { 42, 43};
static unsigned int huawei_keypad_col_gpios[] = { 106 };//gpio_106 is reseverd in this board

static const unsigned short huawei_keypad_keymap_surf[ARRAY_SIZE(huawei_keypad_col_gpios) *
					  ARRAY_SIZE(huawei_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(1, 0)] = KEY_VOLUMEDOWN,
        #if 0
	[KEYMAP_INDEX(2, 0)] = KEY_UP,
	[KEYMAP_INDEX(3, 0)] = KEY_DOWN,
	[KEYMAP_INDEX(4, 0)] = KEY_LEFT,
	[KEYMAP_INDEX(5, 0)] = KEY_RIGHT,
        #endif
};

#define GPIO_WAKEUP_DISABLED    0
#define GPIO_WAKEUP_ENABLED     1

static const unsigned short huawei_keypad_wakeup_enable[ARRAY_SIZE(huawei_keypad_col_gpios) *
					  ARRAY_SIZE(huawei_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = GPIO_WAKEUP_DISABLED,
	[KEYMAP_INDEX(1, 0)] = GPIO_WAKEUP_DISABLED,
};

/* huawei keypad platform device information */
static struct gpio_event_matrix_info huawei_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= huawei_keypad_keymap_surf,
	.wakeup_enable = huawei_keypad_wakeup_enable,
	.output_gpios	= huawei_keypad_col_gpios,
	.input_gpios	= huawei_keypad_row_gpios,
	.noutputs	= ARRAY_SIZE(huawei_keypad_col_gpios),
	.ninputs	= ARRAY_SIZE(huawei_keypad_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *huawei_keypad_info[] = {
	&huawei_keypad_matrix_info.info
};

static struct gpio_event_platform_data huawei_keypad_data = {
	.name		= "huawei_keypad",
	.info		= huawei_keypad_info,
	.info_count	= ARRAY_SIZE(huawei_keypad_info)
};

struct platform_device huawei_keypad = {
	.name	=	GPIO_EVENT_DEV_NAME,
	.id	=	-1,
	.dev	= {
		.platform_data = &huawei_keypad_data,
	},
};
