/*
 * include/linux/touch_platfrom_config.h - platform data structure for touchscreen
 *
 * Copyright (C) 2010 Google, Inc.
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


#ifndef _TOUCH_PLATFORM_CONFIG_H
#define _TOUCH_PLATFORM_CONFIG_H

#define TOUCH_OMAP4430_INT_PIN 35
#define TOUCH_OMAP4430_RESET_PIN  36

/*jKF31740 Begin Atmel Touchscreen mXT224 */

#ifdef CONFIG_TOUCHSCREEN_MXT224E_ATMEL

#include <linux/atmel_qt602240.h> 

static struct atmel_i2c_platform_data atmel_tp_platform_data = {
    .version = 0x10,
    .source = 0,
    .abs_x_min = 0x00,
    .abs_x_max = 539,//959,
    .abs_y_min = 0x00,
    .abs_y_max = 959,//539,
    .abs_pressure_min = 0x00,
    .abs_pressure_max = 255,
    .abs_width_min = 0,
    .abs_width_max = 255,
    .gpio_irq = 35,
    .power = NULL,
    .config_T6 = {0, 0, 0, 0, 0,
    0},
    .config_T7 = {48, 255, 25},
    .config_T8 = {39, 0, 20, 20, 0,
    0, 255, 1, 0, 0},
    .config_T9 = {139, 0, 0, 19, 11,
    0, 32, 60, 2, 1,
    0, 5, 2, 1, 5,
    10, 10, 10, 191, 3,
    27, 2, 13, 13, 12,
    12, 128, 0, 128, 0,
    0, 10, 0, 0, 0},
    .config_T15 = {0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0},
    .config_T19 = {0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0},
    .config_T23 = {0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0},
    .config_T25 = {0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0},
    .config_T40 = {0, 0, 0, 0, 0},
    .config_T42 = {0, 0, 0, 0, 0,
    0, 0, 0},
    .config_T46 = {0, 3, 16, 32, 0,
    0, 0, 0, 0},
    .config_T47 = {0, 20, 50, 5, 2,
    40, 40, 180, 0, 100},
    .config_T48 = {1, 4, 66, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 6, 6,
    0, 0, 100, 4, 64,
    10, 0, 20, 5, 0,
    38, 0, 20, 0, 0,
    0, 0, 0, 0, 0,
    50, 2, 3, 1, 0,
    5, 10, 40, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 3},
    .object_crc = {0x41, 0x0e, 0x90},//CRC 
    .cable_config = {70, 30, 32, 32},
    .cable_config_T7 = {48, 255, 25},
    .cable_config_T8 = {39, 0, 20, 20, 0,
    0, 255, 1, 0, 0},
    .cable_config_T9 = {139, 0, 0, 19, 11,
    0, 32, 60, 2, 1,
    0, 5, 2, 1, 5,
    10, 10, 10, 191, 3,
    27, 2, 13, 13, 12,
    12, 128, 0, 128, 0,
    0, 10, 0, 0, 0},
    .noise_config = {70, 3, 35}, //temporary
    .filter_level = {0, 0, 539, 539},
    .GCAF_level = {8, 16, 24, 32, 40},
};
#endif
/*jKF31740 End Atmel Touchscreen mXT224 */


/*jKF31740 Begin Synaptics Touchscreen RMI4 */
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS

#include <linux/synaptics_i2c_rmi_1564.h>

/*define some tp type*/
#define LCD_X_QVGA         320
#define LCD_Y_QVGA         240
#define LCD_X_HVGA         320
#define LCD_Y_HVGA         480
#define LCD_X_WVGA         480
#define LCD_Y_WVGA         800
#define LCD_JS_WVGA        882
#define LCD_JS_HVGA        510


struct tp_resolution_conversion{
    int lcd_x;
    int lcd_y;
    int jisuan;
};

struct synaptics_i2c_platform_data {
    int (*touch_power)(int on);	/* Only valid in first array entry */
    int (*touch_gpio_config_interrupt)(void);/*it will config the gpio*/
    void (*set_touch_probe_flag)(int detected);/*we use this to detect the probe is detected*/
    int (*read_touch_probe_flag)(void);/*when the touch is find we return a value*/
    int (*touch_reset)(void);
    int (*get_touch_reset_pin)(void);
    int (*get_phone_version)(struct tp_resolution_conversion *tp_resolution_type);/*add this function for judge the tp type*/
};

#endif
/*jKF31740 End Synaptics Touchscreen RMI4 */

#endif /*_TOUCH_PLATFORM_CONFIG_H */
