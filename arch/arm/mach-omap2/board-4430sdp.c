/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2415x.h>
#ifdef CONFIG_HUAWEI_MHL_SII9244
#include <linux/i2c/sii_9244.h>
#endif
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/omapfb.h>
#include <linux/reboot.h>
#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
#include <linux/mfd/twl6040-codec.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <linux/cdc_tcxo.h>

#include <linux/i2c/bq2416x.h>
#define BQ2416X_CHARGER_GPIO_45          45
#define BQ2416X_CHARGER_INTR_IRQ_GPIO_49    49
#define OMAP4_BQ27510_BAT_LOW_GPIO     16

#if defined(CONFIG_ELPIDA_DDR_2G_S4)
#include <mach/lpddr2-elpida.h>
#elif defined(CONFIG_SAMSUNG_DDR_4G_S4)
#include <mach/lpddr2-samsung.h>
#else
#include <mach/lpddr2-elpida.h>
#endif

#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <video/omap-panel-nokia-dsi.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include <linux/wakelock.h>

#include <plat/huawei-dsi-panel.h>

#include "board-blaze.h"
#include "omap4_ion.h"
#include "omap_ram_console.h"
#include <hsad/config_interface.h>
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-4430sdp-wifi.h"
#include "control.h"
#include "common-board-devices.h"

#ifdef CONFIG_MACH_OMAP_XMM
#include <mach/xmd.h>
#endif  //#ifdef CONFIG_MACH_OMAP_XMM

#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
/* for TI WiLink devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>
#define WILINK_UART_DEV_NAME "/dev/ttyO1"

#include <linux/input/cypress-touchkey.h>
#include <plat/toshiba-dsi-panel.h>
/*  Reason: lcd-compatible  */
#include <linux/panel_detect.h>
/*  Reason: Add for NFC  */
#ifdef CONFIG_HUAWEI_NFC_PN544
#include <linux/nfc/pn544.h>
#define CLK3_FREQ    26000000
#endif
#include <linux/touch_platform_config.h>
//#include <linux/board_sensors.h>   //delete for sensors update version.I

#include <linux/keypad_huawei.h> 
#include <linux/es305.h>
#include <hsad/config_debugfs.h>
#include <hsad/gpiomux.h>
#include <plat/config_twl6030_default.h>
#include <hsad/plat_power_mux.h>
#include <linux/i2c.h>
#include <plat/omap_hsi.h>
#define NELEMENTS(ARRAY)    \
    (sizeof (ARRAY) / sizeof ((ARRAY) [0]))

#define ETH_KS8851_IRQ            34
#define ETH_KS8851_POWER_ON        48
#define ETH_KS8851_QUART        138
#define OMAP4_TOUCH_IRQ_1        35
#define OMAP4_TOUCH_IRQ_2        36
#define HDMI_GPIO_CT_CP_HPD      -1 //set to invalid,this is original from Ti Balze, it should be removed for T2 Board
#define HDMI_GPIO_LS_OE          -1 //set to invalid,this is original from Ti Balze, it should be removed for T2 Board
#define HDMI_GPIO_HPD            63  /* Hot plug pin for HDMI */

#define LCD_BL_GPIO        27    /* LCD Backlight GPIO */

#ifdef CONFIG_PANEL_TOSHIBA_MDV20
#define OMAP4SDP_MDM_PWR_EN_GPIO        -1 //set to invalid
#else
#define OMAP4SDP_MDM_PWR_EN_GPIO        157
#endif

/* PWM2 and TOGGLE3 register offsets */
#define LED_PWM2ON        0x03
#define LED_PWM2OFF        0x04
#define TWL6030_TOGGLE3        0x92

#define TPS62361_GPIO   7
#define GPIO_GPS_RESET 161
#define GPIO_GPS_POWER 166

#define OMAP_HDMI_HPD_ADDR    0x4A100098
#define OMAP_HDMI_PULLTYPE_MASK    0x00000010
#define GPIO_WIFI_PMENA         54
#define GPIO_WIFI_IRQ           0

#define GPIO_BT_EN        55
#define GPIO_BT_RST        173
#ifdef CONFIG_HUAWEI_MHL_SII9244
/*#define GPIO_MHL_RST        39*/
/*#define GPIO_MHL_INT        1*/
/*#define GPIO_MHL_3V3_EN        37*/
#endif
#define PHYS_ADDR_SMC_SIZE    (SZ_1M * 3)
#define PHYS_ADDR_SMC_MEM    (0x80000000 + SZ_1G - PHYS_ADDR_SMC_SIZE)
#ifdef CONFIG_KEYPAD_ATMEL_TOUCH
 #define GPIO_ATMEL_TK_INT     151
#endif
//del some lines about leds_gpio define
#define GPIO_LED_TK             137
#define OMAP_ION_HEAP_SECURE_INPUT_SIZE    (SZ_1M * 90)
#define PHYS_ADDR_DUCATI_SIZE    (SZ_1M * 105)
#define PHYS_ADDR_DUCATI_MEM    (PHYS_ADDR_SMC_MEM - PHYS_ADDR_DUCATI_SIZE - \
                OMAP_ION_HEAP_SECURE_INPUT_SIZE)

#define OMAP4SDP_MDM_PWR_EN_GPIO    157


#if 0
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS
atomic_t touch_detected_yet = ATOMIC_INIT(0);

/*
 *use the touch_gpio_config_interrupt to config the gpio
 *which we used, but the gpio number can't exposure to user
 *so when the platform or the product changged please self self adapt
 */

int touch_gpio_config_interrupt(void)
{
    return 0;
}
/*
 *the fucntion set_touch_probe_flag when the probe is detected use this function can set the flag ture
 */

void set_touch_probe_flag(int detected)/*we use this to detect the probe is detected*/
{
    if(detected >= 0)
    {
        atomic_set(&touch_detected_yet, 1);
    }
    else
    {
        atomic_set(&touch_detected_yet, 0);
    }
    return;
}

/*
 *the fucntion read_touch_probe_flag when the probe is ready to detect first we read the flag
 *if the flag is set ture we will break the probe else we
 *will run the probe fucntion
 */

int read_touch_probe_flag(void)
{
    int ret = 0;
    ret = atomic_read(&touch_detected_yet);
    return ret;
}

/*this function reset touch panel */
int touch_reset(void)
{
    int ret = 0;

    gpio_request(TOUCH_OMAP4430_RESET_PIN,"TOUCH_RESET");
    ret = gpio_direction_output(TOUCH_OMAP4430_RESET_PIN, 1);
    mdelay(5);
    ret = gpio_direction_output(TOUCH_OMAP4430_RESET_PIN, 0);
    ret = gpio_direction_output(TOUCH_OMAP4430_RESET_PIN, 1);
    mdelay(50);//must more than 10ms.

    return ret;
}

int get_touch_reset_pin()
{
    int ret = TOUCH_OMAP4430_RESET_PIN;
    return ret;
}

/*this function get the tp  resolution*/
static int get_phone_version(struct tp_resolution_conversion *tp_resolution_type)
{

    tp_resolution_type->lcd_x = LCD_X_HVGA;
    tp_resolution_type->lcd_y = LCD_Y_HVGA;
    tp_resolution_type->jisuan = LCD_JS_HVGA;
    return 0;
}


static struct synaptics_i2c_platform_data synaptics_tp_platform_data = {
    .touch_power = NULL,
    .touch_gpio_config_interrupt = touch_gpio_config_interrupt,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .touch_reset = touch_reset,
    .get_touch_reset_pin = get_touch_reset_pin,
    .get_phone_version = get_phone_version,
};
#endif
#endif
#ifdef CONFIG_SWITCH_USB
void __init init_usb_switch(void);
#endif

#ifdef CONFIG_KEYBOARD_OMAP4 
static const int sdp4430_keymap[] = {
    KEY(0, 0, KEY_E),
    KEY(0, 1, KEY_R),
    KEY(0, 2, KEY_T),
    KEY(0, 3, KEY_HOME),
    KEY(0, 4, KEY_F5),
    KEY(0, 5, KEY_UNKNOWN),
    KEY(0, 6, KEY_I),
    KEY(0, 7, KEY_LEFTSHIFT),

    KEY(1, 0, KEY_D),
    KEY(1, 1, KEY_F),
    KEY(1, 2, KEY_G),
    KEY(1, 3, KEY_SEND),
    KEY(1, 4, KEY_F6),
    KEY(1, 5, KEY_UNKNOWN),
    KEY(1, 6, KEY_K),
    KEY(1, 7, KEY_ENTER),

    KEY(2, 0, KEY_X),
    KEY(2, 1, KEY_C),
    KEY(2, 2, KEY_V),
    KEY(2, 3, KEY_END),
    KEY(2, 4, KEY_F7),
    KEY(2, 5, KEY_UNKNOWN),
    KEY(2, 6, KEY_DOT),
    KEY(2, 7, KEY_CAPSLOCK),

    KEY(3, 0, KEY_Z),
    KEY(3, 1, KEY_KPPLUS),
    KEY(3, 2, KEY_B),
    KEY(3, 3, KEY_F1),
    KEY(3, 4, KEY_F8),
    KEY(3, 5, KEY_UNKNOWN),
    KEY(3, 6, KEY_O),
    KEY(3, 7, KEY_SPACE),

    KEY(4, 0, KEY_W),
    KEY(4, 1, KEY_Y),
    KEY(4, 2, KEY_U),
    KEY(4, 3, KEY_F2),
    KEY(4, 4, KEY_VOLUMEUP),
    KEY(4, 5, KEY_UNKNOWN),
    KEY(4, 6, KEY_L),
    KEY(4, 7, KEY_LEFT),

    KEY(5, 0, KEY_S),
    KEY(5, 1, KEY_H),
    KEY(5, 2, KEY_J),
    KEY(5, 3, KEY_F3),
    KEY(5, 4, KEY_F9),
    KEY(5, 5, KEY_VOLUMEDOWN),
    KEY(5, 6, KEY_M),
    KEY(5, 7, KEY_RIGHT),

    KEY(6, 0, KEY_Q),
    KEY(6, 1, KEY_A),
    KEY(6, 2, KEY_N),
    KEY(6, 3, KEY_BACK),
    KEY(6, 4, KEY_BACKSPACE),
    KEY(6, 5, KEY_UNKNOWN),
    KEY(6, 6, KEY_P),
    KEY(6, 7, KEY_UP),

    KEY(7, 0, KEY_PROG1),
    KEY(7, 1, KEY_PROG2),
    KEY(7, 2, KEY_PROG3),
    KEY(7, 3, KEY_PROG4),
    KEY(7, 4, KEY_F4),
    KEY(7, 5, KEY_UNKNOWN),
    KEY(7, 6, KEY_OK),
    KEY(7, 7, KEY_DOWN),
};

static struct matrix_keymap_data sdp4430_keymap_data = {
    .keymap            = sdp4430_keymap,
    .keymap_size        = ARRAY_SIZE(sdp4430_keymap),
};

void keypad_pad_wkup(int enable)
{
    int (*set_wkup_fcn)(const char *muxname);

    /* PAD wakup for keyboard is needed for off mode
     * due to IO isolation.
     */
    if (!off_mode_enabled)
        return;

    if (enable)
        set_wkup_fcn = omap_mux_enable_wkup;
    else
        set_wkup_fcn = omap_mux_disable_wkup;

    set_wkup_fcn("kpd_col0.kpd_col0");
    set_wkup_fcn("kpd_col1.kpd_col1");
    set_wkup_fcn("kpd_col2.kpd_col2");
    set_wkup_fcn("kpd_col0.kpd_col0");
    set_wkup_fcn("kpd_col1.kpd_col1");
    set_wkup_fcn("kpd_col2.kpd_col2");
    set_wkup_fcn("kpd_col3.kpd_col3");
    set_wkup_fcn("kpd_col4.kpd_col4");
    set_wkup_fcn("kpd_col5.kpd_col5");
    set_wkup_fcn("gpmc_a23.kpd_col7");
    set_wkup_fcn("gpmc_a22.kpd_col6");
    set_wkup_fcn("kpd_row0.kpd_row0");
    set_wkup_fcn("kpd_row1.kpd_row1");
    set_wkup_fcn("kpd_row2.kpd_row2");
    set_wkup_fcn("kpd_row3.kpd_row3");
    set_wkup_fcn("kpd_row4.kpd_row4");
    set_wkup_fcn("kpd_row5.kpd_row5");
    set_wkup_fcn("gpmc_a18.kpd_row6");
    set_wkup_fcn("gpmc_a19.kpd_row7");

}

static struct omap4_keypad_platform_data sdp4430_keypad_data = {
    .keymap_data        = &sdp4430_keymap_data,
    .rows            = 8,
    .cols            = 8,
    .keypad_pad_wkup        = keypad_pad_wkup,
};
#endif

static struct gpio_led sdp4430_gpio_leds[] = {
//del some lines  not used
//del .gpio  and use get_gpio_num_by_name
/* select the correct gpio used in this project wkf40768 */
    {
        .name    = "blue",
        .default_trigger = "timer",
    },
    {
        .name    = "red",
        .default_trigger = "timer",
    },
    {
        .name    = "green",
        .default_trigger = "timer",
    },
};

static struct gpio_led_platform_data sdp4430_led_data = {
    .leds    = sdp4430_gpio_leds,
    .num_leds    = ARRAY_SIZE(sdp4430_gpio_leds),
};

static struct led_pwm sdp4430_pwm_leds[] = {
    {
        .name        = "omap4:green:chrg",
        .pwm_id        = 1,
        .max_brightness    = 255,
        .pwm_period_ns    = 7812500,
    },
};

static struct led_pwm_platform_data sdp4430_pwm_data = {
    .num_leds    = ARRAY_SIZE(sdp4430_pwm_leds),
    .leds        = sdp4430_pwm_leds,
};

static struct platform_device sdp4430_leds_pwm = {
    .name    = "leds_pwm",
    .id    = -1,
    .dev    = {
        .platform_data = &sdp4430_pwm_data,
    },
};

static struct platform_device sdp4430_leds_gpio = {
    .name    = "leds-gpio",
    .id    = -1,
    .dev    = {
        .platform_data = &sdp4430_led_data,
    },
};
//add new code for board not viva_t1
static struct gpio_led sdp4430_dmtimer_leds[] = {

    {
        .name    = "blue",
        .default_trigger = "timer",
    },
    {
        .name    = "red",
        .default_trigger = "timer",
    },
    {
        .name    = "green",
        .default_trigger = "timer",
    },
};

static struct gpio_led_platform_data sdp4430_led_dmtimer_data = {
    .leds    = sdp4430_dmtimer_leds,
    .num_leds    = ARRAY_SIZE(sdp4430_dmtimer_leds),
};

static struct platform_device sdp4430_leds_dmtimer = {
    .name    = "leds-dmtimer",
    .id    = -1,
    .dev    = {
        .platform_data = &sdp4430_led_dmtimer_data,
    },
};
static struct gpio_led gpio_TK_leds = {
    .name        = "TK-backlight",
    .gpio        = 137,
    };

static struct platform_device sdp4460_leds_TK = {
    .name      = "TK-backlight",
    .id        = -1,
    .dev       = {
         .platform_data = &gpio_TK_leds,
    },
};

#ifdef CONFIG_KEYBOARD_OMAP4 
void keyboard_mux_init(void)
{
#if 0
    omap_mux_init_signal("kpd_col0.kpd_col0",
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
    omap_mux_init_signal("kpd_col1.kpd_col1",
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
    //omap_mux_init_signal("kpd_col2.kpd_col2",
                //OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
#ifndef CONFIG_HUAWEI_MHL_SII9244
    omap_mux_init_signal("kpd_col2.kpd_col2",
                OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
#endif
    omap_mux_init_signal("kpd_col3.kpd_col3",
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
    omap_mux_init_signal("kpd_col4.kpd_col4",
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
//    omap_mux_init_signal("kpd_col5.kpd_col5",
//                OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
    omap_mux_init_signal("gpmc_a23.kpd_col7",
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
    omap_mux_init_signal("gpmc_a22.kpd_col6",
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1);
    omap_mux_init_signal("kpd_row0.kpd_row0",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("kpd_row1.kpd_row1",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("kpd_row2.kpd_row2",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("kpd_row3.kpd_row3",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("kpd_row4.kpd_row4",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("kpd_row5.kpd_row5",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("gpmc_a18.kpd_row6",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
    omap_mux_init_signal("gpmc_a19.kpd_row7",
            OMAP_PULL_ENA | OMAP_PULL_UP |
            OMAP_WAKEUP_EN | OMAP_MUX_MODE1 |
            OMAP_INPUT_EN);
#endif
}
#endif

static void omap_bluetooth_init()
{
    int error;

    /*set the pins as gpio mode*/
    omap_mux_init_signal("gpmc_clk.gpio_55",
            OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
    omap_mux_init_signal("kpd_col5.gpio_173",
            OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);


    error = gpio_request(GPIO_BT_EN, "GPIO_BT_EN");
    if (error < 0) {
        pr_err("%s: GPIO request failed: GPIO %d, error %d\n"
            , __func__, GPIO_BT_EN, error);
            return;
      }

    error = gpio_direction_output(GPIO_BT_EN, 0);
    if (error < 0) {
        pr_err("%s: GPIO configure failed: GPIO %d, error %d\n"
            , __func__, GPIO_BT_EN, error);
        goto fail1;
    }

    error = gpio_request(GPIO_BT_RST, "GPIO_BT_RST");
    if (error < 0) {
           pr_err("%s: GPIO request failed: GPIO %d, error %d\n"
            , __func__, GPIO_BT_RST, error);
            goto fail1;
    }

    error = gpio_direction_output(GPIO_BT_RST, 0);
    if (error < 0) {
        pr_err("%s: GPIO configure failed: GPIO %d, error %d\n"
            , __func__, GPIO_BT_RST, error);
        goto fail2;
    }

    return;
fail2:
    gpio_free(GPIO_BT_RST);
fail1:
    gpio_free(GPIO_BT_EN);
}


static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_bt_wake",
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device bcm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

#if 0
static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
    {
        .modalias               = "ks8851",
        .bus_num                = 1,
        .chip_select            = 0,
        .max_speed_hz           = 24000000,
        .irq                    = ETH_KS8851_IRQ,
    },
};

static struct gpio sdp4430_eth_gpios[] __initdata = {
    { ETH_KS8851_POWER_ON,    GPIOF_OUT_INIT_HIGH,    "eth_power"    },
    { ETH_KS8851_QUART,    GPIOF_OUT_INIT_HIGH,    "quart"        },
    { ETH_KS8851_IRQ,    GPIOF_IN,        "eth_irq"    },
};

static int __init omap_ethernet_init(void)
{
    int status;

    /* Request of GPIO lines */
    status = gpio_request_array(sdp4430_eth_gpios,
                    ARRAY_SIZE(sdp4430_eth_gpios));
    if (status)
        pr_err("Cannot request ETH GPIOs\n");

    return status;
}
#endif
#ifdef    CONFIG_KEYPAD_CYPRESS_TOUCH
static void touch_keypad_onoff(int onoff)
{
   #if 0
    gpio_direction_output(_3_GPIO_TOUCH_EN, onoff);

    if (onoff == TOUCHKEY_OFF)
        msleep(30);
    else
        msleep(25);
   #endif
}

static const int touch_keypad_code[] = {
    KEY_MENU,
    KEY_HOME,
    KEY_BACK,
    KEY_SEARCH
};

static struct touchkey_platform_data touchkey_data = {
    .keycode_cnt = ARRAY_SIZE(touch_keypad_code),
    .keycode = touch_keypad_code,
    .touchkey_onoff = touch_keypad_onoff,
    .i2c_slave_addr = CY8C20236A_I2C_SLAVER_ADD,
    .irq_gpio_id    = CY8C20236A_IRQ_GPIO_ID,
};
#endif



/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is capable enough to wake-up the
 * OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
        state)
{
    return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
    return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
    int port_id = 0;
    int err = 0;
    if (uart_req) {
        sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
        err = omap_serial_ext_uart_disable(port_id);
        if (!err)
            uart_req = false;
    }
    wake_unlock(&st_wk_lock);
    return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
    int port_id = 0;
    int err = 0;
    if (!uart_req) {
        sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
        err = omap_serial_ext_uart_enable(port_id);
        if (!err)
            uart_req = true;
    }
    wake_lock(&st_wk_lock);
    return err;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
    .nshutdown_gpio = -EINVAL, //wkf40768 del for wifi conflict with leds
    .dev_name = WILINK_UART_DEV_NAME,
    .flow_cntrl = 1,
    .baud_rate = 3686400,
    .suspend = plat_wlink_kim_suspend,
    .resume = plat_wlink_kim_resume,
    .chip_asleep = plat_uart_disable,
    .chip_awake  = plat_uart_enable,
    .chip_enable = plat_uart_enable,
    .chip_disable = plat_uart_disable,
};
extern int suspend_set_state_ext(struct regulator *rdev,bool state);
static void bluetooth_power_vdd_control(int channel,int on)
{
    char channel_str[2][20]={"BT_VDD_ANA","BT_VDD_2V1"};
    static struct regulator *bt_regulater[2] = {NULL,NULL};
    int ret = 0;

    pr_debug("%s: in channel(%d) on(%d)",__FUNCTION__,channel,on);
	 
    if(on){
        bt_regulater[channel] = regulator_get(NULL, channel_str[channel]);
        if(bt_regulater[channel])
        {
            ret = regulator_enable(bt_regulater[channel] );
            if(0!=ret)
                pr_err("%s: regulator_enable failed",__FUNCTION__);
            else
                pr_debug("%s: regulator_enable ok",__FUNCTION__);
			
            ret = suspend_set_state_ext(bt_regulater[channel] ,true);
            if(0!=ret)
                pr_err("%s: suspend_set_state_ext failed",__FUNCTION__);
            else
                pr_debug("%s: suspend_set_state_ext ok",__FUNCTION__);
        }
        else
        {
            pr_debug("%s: consumer_regulator_enable null",__FUNCTION__);
        }
    }
    else
    {
        if(bt_regulater[channel] )
        {
            ret = suspend_set_state_ext(bt_regulater[channel] ,false);	
            if(0!=ret)
                pr_err("%s: suspend_set_state_ext failed",__FUNCTION__);
            else
                pr_debug("%s: suspend_set_state_ext ok",__FUNCTION__);
            ret = regulator_disable(bt_regulater[channel] );
            if(0!=ret)
                pr_err("%s: regulator_disable failed",__FUNCTION__);
            else
                pr_debug("%s: regulator_disable ok",__FUNCTION__);
            regulator_put(bt_regulater[channel] );
            bt_regulater[channel] = NULL;
        }
        else
        {
            pr_debug("%s: consumer_regulator_disable null",__FUNCTION__);
        }
    }
}
static void bluetooth_power_clk_control(int on)
{
    pr_debug("%s: set 32k clk %s",__FUNCTION__,on?"on":"off");	
    if(on)
    {
        twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x7, 0xBc);
        twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x5, 0xBd);
        twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x55, 0xBE);
    }
    else
    {
        twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x1, 0xBc);
        twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x1, 0xBd);
        twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x5, 0xBE);
    }
}
/*****************************************************************************
     Function    : twl_suspend_power_control
     Description  : VDD_2V1 VDD_ANA 32k_clk
     Input    : 
     unsigned char device    
        BT        0
        WIFI      1
     int on   
        power up      1
        power down    0
     Output       : None
     Return Value : void
     Called By    : bluetooth and wifi power on/off    
*****************************************************************************/
void twl_suspend_power_control(unsigned char device,int on)
{
    static unsigned char mask = 0;
    static int flag = 1;
    static struct mutex lock;    
    unsigned int mask_old = 0;

    pr_debug("\n%s:device(%d),on(%d) mask(0x%x)\n",__FUNCTION__,device,on,mask);
    if(device>1)
    {
        pr_err("\n%s:device(%d) err\n",__FUNCTION__,device);
        return;
    }
    if(flag)
    {
        mutex_init(&lock);
        flag = 0;
    }

    mutex_lock(&lock);
    mask_old = mask;

    if(on)
        mask |= 1<<device;
    else
        mask &= ~(1<<device);
    
    if((mask_old == mask) || (mask_old&&mask))  // equal or both not zero means power state not change
    {
        pr_debug("\n%s:power state not change mask(0x%x)\n",__FUNCTION__,mask);        
        mutex_unlock(&lock);  
        return;
    }
    
    pr_debug("\n%s:change power state mask(0x%x)",__FUNCTION__,mask);

    bluetooth_power_vdd_control(1,on);//VDD_2V1 first
    bluetooth_power_vdd_control(0,on);

    mutex_unlock(&lock);  
}
EXPORT_SYMBOL(twl_suspend_power_control);
void bluetooth_power_up(void)
{
    bluetooth_power_clk_control(1);//32k clk
#if 0
    mdelay(10);    
    gpio_set_value(GPIO_BT_EN, 0);
    mdelay(10);
    gpio_set_value(GPIO_BT_RST, 0);
    mdelay(10);
    gpio_set_value(GPIO_BT_EN, 1);
    mdelay(10);
    gpio_set_value(GPIO_BT_RST, 1);
    mdelay(10);
#endif
}
EXPORT_SYMBOL(bluetooth_power_up);
static int bluetooth_power(int on)
{
    if(on){
        twl_suspend_power_control(0,on);
        
        pr_debug("%s: on",__FUNCTION__);
         gpio_set_value(GPIO_BT_EN, 1);
         mdelay(10);
         gpio_set_value(GPIO_BT_RST, 1);
         mdelay(10);


        omap_mux_init_signal("kpd_row2.gpio_3", OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3|OMAP_WAKEUP_EN);

         //plat_uart_enable();
    }
    else{
        pr_debug("%s: off",__FUNCTION__);
        gpio_set_value(GPIO_BT_RST, 0);
        gpio_set_value(GPIO_BT_EN, 0);

        twl_suspend_power_control(0,on);  	
        omap_mux_init_signal("kpd_row2.gpio_3",OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3);

        //plat_uart_disable();
    }

    return 0;
}
static struct platform_device wl128x_device = {
    .name        = "kim",
    .id        = -1,
    .dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
    .name = "btwilink",
    .id = -1,
};

static struct twl4030_madc_platform_data twl6030_gpadc = {
    .irq_line = -1,
};
static struct platform_device btbcm_device = {
    .name = "bt_power",
    .id = -1,
    .dev.platform_data = &bluetooth_power,
};

#ifdef CONFIG_MACH_OMAP_XMM

struct xmd_platform_data huawei_xmd_platform_data ={   
    .boot_platform_data  = {
        .reset_pmu_req_pinmux              = {"gpmc_ncs3.gpio_53",
                                                          OMAP_PIN_OUTPUT},
        .baseband_reset_pinmux             = {"gpmc_ncs2.gpio_52",
                                                             OMAP_PIN_OUTPUT},
        .on_off_pinmux                   = {"gpmc_nbe0_cle.gpio_59",
                                                             OMAP_PIN_OUTPUT},
        .reset2_pinmux                         = { "gpmc_ncs6.gpio_103",    
                                                          OMAP_PIN_INPUT |\
                                                             OMAP_PULL_ENA |\
                                                             OMAP_PIN_OFF_WAKEUPENABLE},
        .pwr_supply_pinmux                   =  {"",
                                                            OMAP_PIN_OUTPUT},

        .reset_pmu_req_safe_pinmux       = { "gpmc_ncs3.safe_mode",
                                                               OMAP_PIN_INPUT},
        .baseband_reset_safe_pinmux     ={ "gpmc_ncs2.safe_mode",
                                                          OMAP_PIN_INPUT},
        .pwr_supply_safe_pinmux           = {"sys_boot3.safe_mode",
                                                          OMAP_PIN_INPUT},
        .on_off_safe_pinmux                  = {"gpmc_nbe0_cle.gpio_59",
                                                          OMAP_PIN_INPUT_PULLDOWN},     //OMAP_PIN_INPUT
        .reset2_safe_pinmux                  = { "gpmc_ncs6.safe_mode",
                                                            OMAP_PIN_INPUT},

        .xmd_ready_pinmux                   = { "gpmc_ncs1.gpio_51",
                                                               OMAP_PIN_INPUT_PULLDOWN |\
                                                               OMAP_PULL_ENA |\
                                                               OMAP_PIN_OFF_WAKEUPENABLE},
        .xmd_ready_safe_pinmux            = { "gpmc_ncs1.safe_mode",
                                                               OMAP_PIN_INPUT},
        .xmd_ready_gpio                      = 51,
        .reset_pmu_req_gpio                  = 53,
        .baseband_reset_gpio                 = 52,
            .on_off_gpio                     = 59,
        .pwr_supply_gpio                     =-1,
        .reset2_gpio                         = 103, 
    },

#ifdef CONFIG_OMAP4_XMM_HSI
    .hsi_platform_data =  {
        .hsi_cawake_pinmux ={
            .muxname = "usbb1_ulpitll_clk.hsi1_cawake",
            .muxval = OMAP_PIN_INPUT_PULLDOWN | \
                OMAP_PIN_OFF_NONE | \
                OMAP_PIN_OFF_WAKEUPENABLE,
        },
        .hsi_caflag_pinmux ={
            .muxname = "usbb1_ulpitll_dir.hsi1_caflag",
            .muxval = OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE,
        },
        .hsi_cadata_pinmux = {
            .muxname = "usbb1_ulpitll_stp.hsi1_cadata",
            .muxval = OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE,
        },
        .hsi_acready_pinmux = {
            .muxname = "usbb1_ulpitll_nxt.hsi1_acready",
            .muxval = OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_OUTPUT_LOW,
        },
        .hsi_acwake_pinmux = {
            .muxname = "usbb1_ulpitll_dat0.hsi1_acwake" ,
            .muxval = OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_NONE,
        },
        .hsi_acdata_pinmux = {
            .muxname = "usbb1_ulpitll_dat1.hsi1_acdata",
            .muxval = OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_NONE,
        },
        .hsi_acflag_pinmux ={
            .muxname = "usbb1_ulpitll_dat2.hsi1_acflag",
            .muxval = OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_NONE ,
        },
        .hsi_caready_pinmux = {
            .muxname = "usbb1_ulpitll_dat3.hsi1_caready",
            .muxval = OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE,
        },

        .hsi_cawake_safe_pinmux = {
            .muxname = "usbb1_ulpitll_clk.safe_mode",
            .muxval = OMAP_PIN_INPUT_PULLDOWN | \
                OMAP_PIN_OFF_NONE | \
                OMAP_PIN_OFF_WAKEUPENABLE,
        },
        .hsi_cadata_safe_pinmux = {
            .muxname = "usbb1_ulpitll_stp.safe_mode",
            .muxval = OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE,
        },
        .hsi_caflag_safe_pinmux ={
            .muxname = "usbb1_ulpitll_dir.safe_mode",
            .muxval = OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE,
        },
        .hsi_acready_safe_pinmux = {
            .muxname = "usbb1_ulpitll_nxt.safe_mode",
            .muxval = OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_OUTPUT_LOW,
        },
        .hsi_acwake_safe_pinmux = {
            .muxname = "usbb1_ulpitll_dat0.safe_mode" ,
            .muxval = OMAP_PIN_OUTPUT | \
            OMAP_PIN_OFF_NONE,
        },
        .hsi_acdata_safe_pinmux = {
            .muxname = "usbb1_ulpitll_dat1.safe_mode",
            .muxval = OMAP_PIN_OUTPUT | \
            OMAP_PIN_OFF_NONE,
        },
        .hsi_acflag_safe_pinmux = {
            .muxname = "usbb1_ulpitll_dat2.safe_mode",
            .muxval = OMAP_PIN_OUTPUT | \
            OMAP_PIN_OFF_NONE ,
        },
        .hsi_caready_safe_pinmux = {
            .muxname = "usbb1_ulpitll_dat3.safe_mode",
            .muxval = OMAP_PIN_INPUT | \
            OMAP_PIN_OFF_NONE,
        },
    },
#endif
};

static struct platform_device sdp4430_xmm_boot_device = {
    .name   =   "xmm_boot",
    .id =   -1,
    .dev    = {
        .platform_data = &huawei_xmd_platform_data, 
    },
};

#endif  //#ifdef CONFIG_MACH_OMAP_XMM

#if 0
static struct twl4030_madc_platform_data twl6030_madc = {
    .irq_line = -1,
};
#endif
static struct platform_device twl6030_madc_device = {
    .name   = "twl6030_madc",
    .id = -1,
    .dev    = {
        .platform_data    = &twl6030_madc,
    },
};
static struct platform_device *sdp4430_devices[] __initdata = {
//wkf40768 added for have changed the led-gpio
    &sdp4430_leds_gpio,
    &sdp4430_leds_pwm,
    &sdp4430_leds_dmtimer,
    &wl128x_device,
    &btwilink_device,
#ifdef CONFIG_MACH_OMAP_XMM
    &sdp4430_xmm_boot_device,
#endif  //#ifdef CONFIG_MACH_OMAP_XMM
    &btbcm_device,

   &bcm_bluesleep_device,

        &twl6030_madc_device,
    &sdp4460_leds_TK,
};

static struct omap_board_config_kernel sdp4430_config[] __initdata = {
};

static void __init omap_4430sdp_init_early(void)
{
    omap2_init_common_infrastructure();
    omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
    omap2_gp_clockevent_set_gptimer(1);
#endif
}

#ifdef CONFIG_HUAWEI_GPIO_KEYPAD
static struct platform_device *huawei_devices[] __initdata = {
    &huawei_keypad,//jKF31740
};

#endif

static struct omap_musb_board_data musb_board_data = {
    .interface_type        = MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
    .mode            = MUSB_OTG,
#else
    .mode            = MUSB_PERIPHERAL,
#endif
    .power            = 200,
};

#ifndef CONFIG_TIWLAN_SDIO
static int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
    static int power_state;

    pr_debug("Powering %s wifi", (power_on ? "on" : "off"));

    if (power_on == power_state)
        return 0;
    power_state = power_on;

    if (power_on) {
        gpio_set_value(GPIO_WIFI_PMENA, 1);
        mdelay(15);
        gpio_set_value(GPIO_WIFI_PMENA, 0);
        mdelay(1);
        gpio_set_value(GPIO_WIFI_PMENA, 1);
        mdelay(70);
    } else {
        gpio_set_value(GPIO_WIFI_PMENA, 0);
    }

    return 0;
}
#endif

// Defined this data struct in config_twl6030_default.h
#if 0
static struct twl4030_usb_data omap4_usbphy_data = {
    .phy_init    = omap4430_phy_init,
    .phy_exit    = omap4430_phy_exit,
    .phy_power    = omap4430_phy_power,
    .phy_set_clock    = omap4430_phy_set_clk,
    .phy_suspend    = omap4430_phy_suspend,
};
#endif

static struct omap2_hsmmc_info mmc[] = {
    {
        .mmc        = 2,
        .caps        = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
                    MMC_CAP_1_8V_DDR,
        .gpio_cd    = -EINVAL,
        .gpio_wp    = -EINVAL,
        .nonremovable   = true,
        .ocr_mask    = MMC_VDD_29_30,
        .no_off_init    = true,
    },
    {
        .mmc        = 1,
        .caps        = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
                    MMC_CAP_1_8V_DDR,
        .gpio_cd    = -EINVAL,
        .gpio_wp    = -EINVAL,
    },
#ifdef CONFIG_TIWLAN_SDIO
    {
        .mmc        = 5,
        .caps        = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
        .gpio_cd    = -EINVAL,
        .gpio_wp        = -EINVAL,//4,    
        .ocr_mask       = MMC_VDD_165_195|MMC_VDD_20_21,    
    },
#else
    {
        .mmc        = 5,
        .caps        = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
        .gpio_cd    = -EINVAL,
        .gpio_wp    = -EINVAL,
        .ocr_mask    = MMC_VDD_165_195,
        .nonremovable    = true,
    },
#endif
    {}    /* Terminator */
};

static struct regulator_consumer_supply sdp4430_vaux_supply[] = {
    {
        .supply = "vmmc",
        .dev_name = "omap_hsmmc.1",
    },
};

#if 0
static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
    {
        .supply = "vmmc",
        .dev_name = "omap_hsmmc.0",
    },
};

static struct regulator_consumer_supply sdp4430_cam2_supply[] = {
    {
        .supply = "cam2pwr",
    },
#ifdef HUAWEI_SENSORS
    {
        .supply = COMPASS_POWER_NAME,
    },
    {
        .supply = ACCL_POWER_NAME,
    },
    {
        .supply = GYRO_POWER_NAME,
    },
    {
        .supply = PL_POWER_NAME,
    },
#endif
};
#endif

static struct regulator_consumer_supply sdp4430_vcxio_supply[] = {
    REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
    REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};
static struct regulator_consumer_supply omap4_sdp4430_vmmc5_supply = {
    .supply = "vmmc",
    .dev_name = "omap_hsmmc.4",
};
static struct regulator_init_data sdp4430_vmmc5 = {
    .constraints = {
        .valid_ops_mask = REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = 1,
    .consumer_supplies = &omap4_sdp4430_vmmc5_supply,
};
static struct fixed_voltage_config sdp4430_vwlan = {
    .supply_name = "vwl1271",
    .microvolts = 1800000, /* 1.8V */
    .gpio = GPIO_WIFI_PMENA,
    .startup_delay = 70000, /* 70msec */
    .enable_high = 1,
    .enabled_at_boot = 0,
    .init_data = &sdp4430_vmmc5,
};
static struct platform_device omap_vwlan_device = {
    .name        = "reg-fixed-voltage",
    .id        = 1,
    .dev = {
        .platform_data = &sdp4430_vwlan,
               }
};

struct regulator *enable_power_for_device(struct device* dev , const char* id,int uV)
{
    struct regulator *vaux = NULL;
    int  voltage, ret = 0;
    if(NULL == id)
    {
        printk(KERN_ERR"turn_on_aux for dev(%s) id(%s) error", dev_name(dev), id);
        vaux = NULL;
        return vaux;
    }

    vaux = regulator_get(dev, id);
    if (IS_ERR(vaux))
    {
        printk(KERN_ERR"turn_on_aux for dev(%s) id(%s) error", dev_name(dev), id);
        vaux = NULL;
        return vaux;
    }

    regulator_set_voltage(vaux, uV, uV);
    printk(KERN_INFO"vaux set voltage  == %dv \n", uV/100000);

    printk(KERN_INFO"turn_on_aux for dev(%s) id(%s)", dev_name(dev), id );
    ret =  regulator_enable(vaux);
    return vaux;
}

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
    int ret = 0;
    struct platform_device *pdev = container_of(dev,
                struct platform_device, dev);
    struct omap_mmc_platform_data *pdata = dev->platform_data;

    /* Setting MMC1 Card detect Irq */
    if (pdev->id == 0) {
        ret = twl6030_mmc_card_detect_config();
        if (ret)
            pr_err("Failed configuring MMC1 card detect\n");
        pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
                        MMCDETECT_INTR_OFFSET;
        pdata->slots[0].card_detect = twl6030_mmc_card_detect;
    }
#ifndef CONFIG_TIWLAN_SDIO
    /* Set the MMC5 (wlan) power function */
    if (pdev->id == 4)
        pdata->slots[0].set_power = wifi_set_power;
#endif
    return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
    struct omap_mmc_platform_data *pdata;

    /* dev can be null if CONFIG_MMC_OMAP_HS is not set */
    if (!dev) {
        pr_err("Failed %s\n", __func__);
        return;
    }
    pdata = dev->platform_data;
    pdata->init =    omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
    struct omap2_hsmmc_info *c;

    omap2_hsmmc_init(controllers);
    for (c = controllers; c->mmc; c++)
        omap4_twl6030_hsmmc_set_late_init(c->dev);

    return 0;
}

// These data structs are generated automatically by power tree excel files
#if 0
static struct regulator_init_data sdp4430_vaux1 = {
    .constraints = {
        .min_uV            = 1000000,
        .max_uV            = 3000000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies  = 1,
    .consumer_supplies      = sdp4430_vaux_supply,
};

static struct regulator_consumer_supply sdp4430_vaux2_supply[] = {
    REGULATOR_SUPPLY("av-switch", "soc-audio"),
#ifdef CONFIG_TOUCHSCREEN_MXT224E_ATMEL
    REGULATOR_SUPPLY("atmel_ts_power","2-004a"),
#endif
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS
    REGULATOR_SUPPLY("synaptics_ts_power","2-0070"),
#endif
    REGULATOR_SUPPLY("vaux2", "display0"),
};
static struct regulator_init_data sdp4430_vaux2 = {
    .constraints = {
        .min_uV            = 1200000,
        .max_uV            = 2800000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies    = ARRAY_SIZE(sdp4430_vaux2_supply),
    .consumer_supplies    = sdp4430_vaux2_supply,
};

static struct regulator_init_data sdp4430_vaux3 = {
    .constraints = {
        .min_uV            = 1000000,
        .max_uV            = 3000000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = ARRAY_SIZE(sdp4430_cam2_supply),

    .consumer_supplies = sdp4430_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
    .constraints = {
        .min_uV            = 1200000,
        .max_uV            = 3000000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies  = 1,
    .consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
    .constraints = {
        .min_uV            = 1800000,
        .max_uV            = 2500000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
};

// added vusim regulator for fuse contorl
static struct regulator_consumer_supply sdp4430_vusim_supply[] = {
    REGULATOR_SUPPLY("vusim-fuse", "twl6030_bci"),
};


static struct regulator_init_data sdp4430_vusim = {
    .constraints = {
        .min_uV            = 1200000,
        .max_uV            = 2900000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = ARRAY_SIZE(sdp4430_vusim_supply),
    .consumer_supplies = sdp4430_vusim_supply,
};


static struct regulator_init_data sdp4430_vana = {
    .constraints = {
        .min_uV            = 2100000,
        .max_uV            = 2100000,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
        .always_on    = true,
    },
};

static struct regulator_init_data sdp4430_vcxio = {
    .constraints = {
        .min_uV            = 1800000,
        .max_uV            = 1800000,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
        .always_on    = true,
    },
    .num_consumer_supplies    = ARRAY_SIZE(sdp4430_vcxio_supply),
    .consumer_supplies    = sdp4430_vcxio_supply,
};

static struct regulator_consumer_supply sdp4430_vdac_supply[] = {
    {
        .supply = "hdmi_vref",
    },
};

static struct regulator_init_data sdp4430_vdac = {
    .constraints = {
        .min_uV            = 1800000,
        .max_uV            = 1800000,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     = REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
        .always_on    = true,
    },
    .num_consumer_supplies  = ARRAY_SIZE(sdp4430_vdac_supply),
    .consumer_supplies      = sdp4430_vdac_supply,
};

static struct regulator_init_data sdp4430_vusb = {
    .constraints = {
        .min_uV            = 3300000,
        .max_uV            = 3300000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask     =    REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
};

static struct regulator_init_data sdp4430_clk32kg = {
    .constraints = {
        .valid_ops_mask        = REGULATOR_CHANGE_STATUS,
        .always_on        = true,
    },
};


static int tps6130x_enable(int on)
{
    u8 val = 0;
    int ret;

    ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, TWL6040_REG_GPOCTL);
    if (ret < 0) {
        pr_err("%s: failed to read GPOCTL %d\n", __func__, ret);
        return ret;
    }

    /* TWL6040 GPO2 connected to TPS6130X NRESET */
    if (on)
        val |= TWL6040_GPO2;
    else
        val &= ~TWL6040_GPO2;

    ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, TWL6040_REG_GPOCTL);
    if (ret < 0)
        pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);

    return ret;
}

static struct tps6130x_platform_data tps6130x_pdata = {
    .chip_enable    = tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
    REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf = {
    .constraints = {
        .min_uV            = 4075000,
        .max_uV            = 4950000,
        .apply_uV        = true,
        .valid_modes_mask    = REGULATOR_MODE_NORMAL
                    | REGULATOR_MODE_STANDBY,
        .valid_ops_mask        = REGULATOR_CHANGE_VOLTAGE
                    | REGULATOR_CHANGE_MODE
                    | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies    = ARRAY_SIZE(twl6040_vddhf_supply),
    .consumer_supplies    = twl6040_vddhf_supply,
    .driver_data        = &tps6130x_pdata,
};

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1 and ES1.2 (both share same ASICREV value)
	 */
	if (rev == TWL6040_REV_1_1)
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);

	return 0;
}

static struct twl4030_codec_audio_data twl6040_audio = {
    /* single-step ramp for headset and handsfree */
    .hs_left_step    = 0x0f,
    .hs_right_step    = 0x0f,
    .hf_left_step    = 0x1d,
    .hf_right_step    = 0x1d,
    .vddhf_uV    = 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
    .max_timeout    = 15000,
    .initial_vibrate = 0,
    .voltage_raise_speed = 0x26,
};

static struct twl4030_codec_data twl6040_codec = {
    .audio        = &twl6040_audio,
    .vibra        = &twl6040_vibra,
    .audpwron_gpio    = 127,
    .naudint_irq    = OMAP44XX_IRQ_SYS_2N,
    .irq_base    = TWL6040_CODEC_IRQ_BASE,
    .init        = twl6040_init,
};

static int sdp4430_batt_table[] = {
    /* adc code for temperature in degree C */
    929, 925, /* -2 ,-1 */
    920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
    875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
    816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
    748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
    671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
    591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
    511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data sdp4430_bci_data = {
    .monitoring_interval        = 10,
    .max_charger_currentmA        = 1500,
    .max_charger_voltagemV        = 4560,
    .max_bat_voltagemV        = 4200,
    .low_bat_voltagemV        = 3300,
    .battery_tmp_tbl        = sdp4430_batt_table,
    .tblsize            = ARRAY_SIZE(sdp4430_batt_table),
};

static struct twl4030_platform_data sdp4430_twldata = {
    .irq_base    = TWL6030_IRQ_BASE,
    .irq_end    = TWL6030_IRQ_END,

    /* Regulators */
    .vmmc        = &sdp4430_vmmc,
    .vpp        = &sdp4430_vpp,
    .vusim        = &sdp4430_vusim,
    .vana        = &sdp4430_vana,
    .vcxio        = &sdp4430_vcxio,
    .vdac        = &sdp4430_vdac,
    .vusb        = &sdp4430_vusb,
    .vaux1        = &sdp4430_vaux1,
    .vaux2        = &sdp4430_vaux2,
    .vaux3        = &sdp4430_vaux3,
    .clk32kg    = &sdp4430_clk32kg,
    .usb        = &omap4_usbphy_data,
    .madc           = &twl6030_madc,
    .bci        = &sdp4430_bci_data,
    /* children */
    .codec        = &twl6040_codec,
    //.madc        = &twl6030_gpadc,

};
#endif

static struct bq2415x_platform_data sdp4430_bqdata = {
    .max_charger_voltagemV = 4200,
    .max_charger_currentmA = 1550,
};

static struct bq2416x_platform_data sdp4460_bqdata = {
    .max_charger_voltagemV = 4200,
    .max_charger_currentmA = 1000,
};

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */
static struct cdc_tcxo_platform_data sdp4430_cdc_data = {
    .buf = {
        CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
        CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
        CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,

        CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
        CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,

        CDC_TCXO_LDOEN1,

        0 },
};

static struct es305_platform_data audience_es305_platform_data = {
     .gpio_es305_wakeup_uart1 = 141,
     .gpio_es305_wakeup_uart3 = 17,
};

#ifndef CONFIG_ARCH_OMAP
static void omap4_audio_conf(void)
{
    /* twl6040 naudint */
    omap_mux_init_signal("sys_nirq2.sys_nirq2", \
        OMAP_PIN_INPUT_PULLUP);
}
#else
static void omap4_audio_conf(void)
{
/*Reason: The probability that TI TWL6040
          Codec Powerup fail is high.  */
    gpio_request(twl6040_codec.audpwron_gpio,"AUDPWR");
    omap_mux_init_gpio(twl6040_codec.audpwron_gpio, OMAP_PIN_OUTPUT);
    gpio_set_value(twl6040_codec.audpwron_gpio,0);
    gpio_free(twl6040_codec.audpwron_gpio);

           /* twl6040 naudint */
    omap_mux_init_signal("sys_nirq2.sys_nirq2", \
        OMAP_PIN_INPUT_PULLUP |OMAP_OFFOUT_VAL | OMAP_WAKEUP_EN);
}
#endif
#ifdef CONFIG_HUAWEI_MHL_SII9244

static void sii9244_reset(void)
{
    int gpio_mhl_rst = get_gpio_num_by_name("GPIO_MHL_RST_N");
    if(gpio_mhl_rst < 0)
    {
        pr_err("%s: get gpio GPIO39_MHL_RST_N number failed\n",__func__);
        return;
    }

    gpio_request(gpio_mhl_rst, "mhl_rst");
    // gpio_direction_output(GPIO_MHL_RST, 0);
    // msleep(100);
    // gpio_direction_output(GPIO_MHL_RST, 1);
    printk(KERN_INFO "%s:%d:Sii9244 reset successed\n", __func__,__LINE__);
}

static struct sii9244_platform_data sii9244_platform_data = {
    .reset = sii9244_reset,
};

#endif

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {

// TWL6030 data struct is generated automatically by power tree excel file
#if 0
    {
    I2C_BOARD_INFO("twl6030", 0x48),
    .flags = I2C_CLIENT_WAKE,
    .irq = OMAP44XX_IRQ_SYS_1N,
    .platform_data = &sdp4430_twldata,
    },
#endif
#if 0
    {
    I2C_BOARD_INFO("bq24156", 0x6a),
    .platform_data = &sdp4430_bqdata,
    },
#endif
    {
    I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
    .platform_data = &sdp4430_cdc_data,
    },
    {
        I2C_BOARD_INFO("bq24161", 0x6b),
        .platform_data = &sdp4460_bqdata,
        //.irq = BQ2416X_CHARGER_INTR_IRQ_GPIO_49,
    },
    {
         I2C_BOARD_INFO("bq27510", 0x55),
         .irq = OMAP_GPIO_IRQ(OMAP4_BQ27510_BAT_LOW_GPIO),
    },
    {
        I2C_BOARD_INFO("audience_es305", 0x3e),
        .platform_data = &audience_es305_platform_data,
    },
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
#if 0
    /*{
        I2C_BOARD_INFO("tm12xx_ts_primary", 0x4b),
        .platform_data = &tm12xx_platform_data[0],
    },
    {
        I2C_BOARD_INFO("picoDLP_i2c_driver", 0x1b),
        .platform_data = &picodlp_platform_data[0],
    },*/
#ifdef CONFIG_TOUCHSCREEN_MXT224E_ATMEL
    {
        I2C_BOARD_INFO("atmel_qt602240", 0x4a),
        .platform_data = &atmel_tp_platform_data,
    },
#endif
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS
    {
        I2C_BOARD_INFO("Synaptics_rmi", 0x70),//jKF31740 ADD
        .platform_data = &synaptics_tp_platform_data,
        .irq = 35,
        .flags = true, //this flags is the switch of the muti_touch
    },
#endif
#endif
#if 1
#ifdef CONFIG_KEYPAD_ATMEL_TOUCH
    {
        I2C_BOARD_INFO("qt1060", 0x12),
        //.irq = OMAP_GPIO_IRQ(151),
    },
#endif
#endif
};
static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
    {
        I2C_BOARD_INFO("tps61310", 0x33),
    },
};

/*  Reason: Modified for NFC  */
#ifdef CONFIG_HUAWEI_NFC_PN544
/* this function is used to reset pn544 by controlling the ven pin */
static void __init omap_clk3_init(void)
{
    struct clk *aux_clk3;

    aux_clk3 = clk_get(NULL, "auxclk3_ck");
    clk_set_rate(aux_clk3, CLK3_FREQ);
    clk_enable(aux_clk3);

    omap_mux_init_signal("fref_clk3_out",
    OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_MUX_MODE0);
    omap_mux_init_signal("fref_clk3_req",
    OMAP_INPUT_EN|OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_MUX_MODE0);

}
static int pn544_ven_reset(void)
{
    int ret=0;
    int gpio_config=0;
    ret = gpio_request(GPIO_NFC_VEN, "gpio 130 for NFC pn544");

    ret = gpio_direction_output(GPIO_NFC_VEN,0);



    printk("^-^^-^HT:Entry  pn544_ven_reset!\n");

    gpio_set_value(GPIO_NFC_VEN, 1);
    mdelay(5);

    gpio_set_value(GPIO_NFC_VEN, 0);
    mdelay(50);

    gpio_set_value(GPIO_NFC_VEN, 1);
    mdelay(5);
    return 0;
}

static int pn544_interrupt_gpio_config(void)
{
    int ret=0;
    int gpio_config=0;
    ret = gpio_request(GPIO_NFC_INT, "gpio 49 for NFC pn544");
    ret = gpio_direction_input(GPIO_NFC_INT);
    return 0;
}


static struct pn544_nfc_platform_data pn544_hw_data =
{
    .pn544_ven_reset = pn544_ven_reset,
    .pn544_interrupt_gpio_config = pn544_interrupt_gpio_config,
};

#endif
static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {
/*  Reason: Modified for NFC  */
#ifdef CONFIG_HUAWEI_NFC_PN544
    {
        I2C_BOARD_INFO(PN544_DRIVER_NAME, PN544_I2C_ADDR),
        .irq = OMAP_GPIO_IRQ(GPIO_NFC_INT),
        .platform_data = &pn544_hw_data,
    },
#endif

#if 0
#ifdef HUAWEI_SENSORS_COMPASS_AKM8975
    {
        I2C_BOARD_INFO(M_I2C_NAME, M_SA),
        .platform_data = &compass_platform_data,
        .flags = I2C_CLIENT_WAKE,
        .irq = OMAP_GPIO_IRQ(25),
    },
#endif

#ifdef HUAWEI_SENSORS_GSENSOR_LIS3DH
    {
        I2C_BOARD_INFO(GS_I2C_NAME, GS_SA),
        .platform_data = &gs_platform_data,
    },
#endif

#ifdef CONFIG_INPUT_MMA8452Q
    {
        I2C_BOARD_INFO("mma8452", 0x1c),
    },
#endif

#ifdef HUAWEI_SENSORS_GYRO_L3G4200D
    {
        I2C_BOARD_INFO("l3g4200d_gyr", 0x68),
        .platform_data = &l3g4200d_gyr_platform_data,
    },
#endif

#ifdef HUAWEI_SENSORS_PROXIMITY_LIGHT_TMD2771
    {
        //I2C_BOARD_INFO("tmd2771", 0x39),
    },
#endif

#ifdef CONFIG_INPUT_ADXL34X_I2C
    {
        I2C_BOARD_INFO("adxl34x", 0x53),
        .irq = OMAP_GPIO_IRQ(23),
    },
#endif
#ifdef CONFIG_INPUT_APDS990x
    {
        I2C_BOARD_INFO("apds990x", 0x39),
        .irq = OMAP_GPIO_IRQ(28),
    },
#endif
#endif
#ifdef CONFIG_HUAWEI_MHL_SII9244

       {
        I2C_BOARD_INFO("mhl_Sii9244_page0", 0x39),
        .platform_data = &sii9244_platform_data,
        //.irq = OMAP_GPIO_IRQ(GPIO_MHL_INT),
       },
    {
        I2C_BOARD_INFO("mhl_Sii9244_page1", 0x3D),
    },
    {
        I2C_BOARD_INFO("mhl_Sii9244_page2", 0x49),
    },
    {
        I2C_BOARD_INFO("mhl_Sii9244_cbus", 0x64),
    },
#endif
#ifdef CONFIG_KEYPAD_ATMEL_TOUCH
    {
        I2C_BOARD_INFO("qt1060", 0x12),
    },
#endif
    {
        I2C_BOARD_INFO("led_colors", 0x45),
    },
};

static void __init blaze_pmic_mux_init(void)
{

    omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
                        OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
                struct omap_i2c_bus_board_data *pdata)
{
    /* spinlock_id should be -1 for a generic lock request */
    if (spinlock_id < 0)
        pdata->handle = hwspin_lock_request();
    else
        pdata->handle = hwspin_lock_request_specific(spinlock_id);

    if (pdata->handle != NULL) {
        pdata->hwspin_lock_timeout = hwspin_lock_timeout;
        pdata->hwspin_unlock = hwspin_unlock;
    } else {
        pr_err("I2C hwspinlock request failed for bus %d\n", \
                                bus_id);
    }
}

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

/* Init i2c controller1, added slave devices to i2c1 bus.
 * Should added twl6030 PMIC firstly and added all devices once.
 */
int __init omap4_i2c_bus1_init(void)
{
    int i;
    struct i2c_board_info *sdp4430_i2c_boardinfo_all;
    const unsigned int device_num = NELEMENTS(sdp4430_i2c_boardinfo)+1;//number of all devices, added pmic to i2c1 bus
    sdp4430_i2c_boardinfo_all = (struct i2c_board_info *)kzalloc(sizeof(struct i2c_board_info)*device_num, GFP_KERNEL);
    sdp4430_i2c_boardinfo_all[0] = *((struct i2c_board_info *) get_board_powerconf());

    for( i=0; i<ARRAY_SIZE(sdp4430_i2c_boardinfo); i++ )
        sdp4430_i2c_boardinfo_all[i+1]= sdp4430_i2c_boardinfo[i];

    omap_register_i2c_bus(1, 400, sdp4430_i2c_boardinfo_all, device_num);
    kfree(sdp4430_i2c_boardinfo_all);
    return 0;
}

static int __init omap4_i2c_init(void)
{
    int i; 
    int gpio_enable_charger = get_gpio_num_by_name("GPIO_ENABLE_CHARGER");

    if (get_mhl_ci2ca_value())
    {
        // If CI2CA is pulled up
        // MHL chip is special, the I2C address is affected by this pin
            for(i = 0; i < ARRAY_SIZE(sdp4430_i2c_4_boardinfo); i++)
            {
                if (   !strcmp(sdp4430_i2c_4_boardinfo[i].type, "mhl_Sii9244_page0")
                    || !strcmp(sdp4430_i2c_4_boardinfo[i].type, "mhl_Sii9244_page1")
                    || !strcmp(sdp4430_i2c_4_boardinfo[i].type, "mhl_Sii9244_page2")
                    || !strcmp(sdp4430_i2c_4_boardinfo[i].type, "mhl_Sii9244_cbus")
                    )
                {
                    sdp4430_i2c_4_boardinfo[i].addr += 0x2;
                }
            }
    }

    omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_1_bus_pdata);
    omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
    omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
    omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);

    omap_register_i2c_bus_board_data(1, &sdp4430_i2c_1_bus_pdata);
    omap_register_i2c_bus_board_data(2, &sdp4430_i2c_2_bus_pdata);
    omap_register_i2c_bus_board_data(3, &sdp4430_i2c_3_bus_pdata);
    omap_register_i2c_bus_board_data(4, &sdp4430_i2c_4_bus_pdata);

    //omap4_pmic_init("twl6030", &sdp4430_twldata);
    //i2c_register_board_info(1, &sdp4430_i2c_boardinfo, 1);

        /* GPIO enable VBUS_USB GPIO init */
        gpio_request(BQ2416X_CHARGER_GPIO_45, "gpio_45_enable_charger");
        gpio_direction_output(BQ2416X_CHARGER_GPIO_45, 1);
    //sdp4430_i2c_boardinfo[2].irq = gpio_to_irq(BQ2416X_CHARGER_INTR_IRQ_GPIO_49);

//Modified i2c1 bus initialization process. defined pmic data struct separately
/*
    omap_register_i2c_bus(1, 400, sdp4430_i2c_boardinfo, ARRAY_SIZE(sdp4430_i2c_boardinfo));
*/
    omap4_i2c_bus1_init();

    //omap_register_i2c_bus(1, 400, sdp4430_i2c_boardinfo, ARRAY_SIZE(sdp4430_i2c_boardinfo));
    omap_register_i2c_bus(2, 100, sdp4430_i2c_2_boardinfo,
                ARRAY_SIZE(sdp4430_i2c_2_boardinfo)); 
    omap_register_i2c_bus(3, 400, sdp4430_i2c_3_boardinfo,
                ARRAY_SIZE(sdp4430_i2c_3_boardinfo));
    omap_register_i2c_bus(4, 400, sdp4430_i2c_4_boardinfo,   //400 ->100
                ARRAY_SIZE(sdp4430_i2c_4_boardinfo));


    /*
     * This will allow unused regulator to be shutdown. This flag
     * should be set in the board file. Before regulators are registered.
     */
    regulator_has_full_constraints();

    /*
     * Drive MSECURE high for TWL6030 write access.
     */
    omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
    gpio_request(6, "msecure");
    gpio_direction_output(6, 1);

    return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

static void omap_bq27510battery_init(void)
{
       /* Configure OMAP4_BQ27510_BAT_LOW_GPIO to:
            1<<3 : PU/PD enable
            1<<4 : PU selected
            1<<14 : Enable wake_up function
       */
       omap_mux_init_signal("dpm_emu5.gpio_16", 1<<14 | 1<<4 | 1<<3);
       omap_mux_init_gpio(OMAP4_BQ27510_BAT_LOW_GPIO, OMAP_PIN_INPUT |
                            OMAP_PIN_OFF_WAKEUPENABLE);
       if (gpio_request(OMAP4_BQ27510_BAT_LOW_GPIO, "Batteryctl") < 0) {
               pr_err("Batteryctl GPIO request failed\n");
               return;
       }
       gpio_direction_input(OMAP4_BQ27510_BAT_LOW_GPIO);
}


static int dsi1_panel_set_backlight(struct omap_dss_device *dssdev, int level)
{
    int r;

    r = twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
    if (r)
        return r;

    if (level > 1) {
        if (level == 255)
            level = 0x7F;
        else
            level = (~(level/2)) & 0x7F;

        r = twl_i2c_write_u8(TWL_MODULE_PWM, level, LED_PWM2ON);
        if (r)
            return r;
        r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
        if (r)
            return r;
    } else if (level <= 1) {
        r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
        if (r)
            return r;
        r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x28, TWL6030_TOGGLE3);
        if (r)
            return r;
        r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x00, TWL6030_TOGGLE3);
        if (r)
            return r;
    }

    return 0;
}

static struct nokia_dsi_panel_data dsi1_panel;
static struct sp_dsi_panel_data dsi_panel_sp;
static struct toshiba_dsi_panel_data mdv20_dsi_panel;

static struct gpio sdp4430_hdmi_gpios[] = {
    {HDMI_GPIO_CT_CP_HPD,  GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_hpd"   },
    {HDMI_GPIO_LS_OE,      GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_ls_oe" },
};


static void sdp4430_hdmi_mux_init(void)
{
    u32 r;
    int status;
    /* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
    omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
                OMAP_PIN_INPUT_PULLDOWN);
    omap_mux_init_signal("gpmc_wait2.gpio_100",
            OMAP_PIN_INPUT_PULLDOWN);
    omap_mux_init_signal("hdmi_cec.hdmi_cec",
            OMAP_PIN_INPUT_PULLUP);
    /* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
    omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
            OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
            OMAP_PIN_INPUT_PULLUP);

    /* strong pullup on DDC lines using unpublished register */
    r = ((1 << 24) | (1 << 28)) ;
    omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

    gpio_request(HDMI_GPIO_HPD, NULL);
    omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
    gpio_direction_input(HDMI_GPIO_HPD);

    status = gpio_request_array(sdp4430_hdmi_gpios,
            ARRAY_SIZE(sdp4430_hdmi_gpios));
    if (status)
        pr_err("%s:Cannot request HDMI GPIOs %x \n", __func__, status);
}



#ifdef    CONFIG_KEYPAD_CYPRESS_TOUCH
    {
        I2C_BOARD_INFO(CYPRESS_TOUCHKEY_I2C_NAME, CY8C20236A_I2C_SLAVER_ADD),
        .platform_data = &touchkey_data,
    },
#endif

#ifdef CONFIG_PANEL_TAAL
static struct nokia_dsi_panel_data dsi1_panel = {
        .name        = "taal",
        .reset_gpio    = 102,
        .use_ext_te    = false,
        .ext_te_gpio    = 101,
        .esd_interval    = 0,
        .set_backlight    = dsi1_panel_set_backlight,
};

static struct omap_dss_device sdp4430_lcd_device = {
    .name            = "lcd",
    .driver_name        = "taal",
    .type            = OMAP_DISPLAY_TYPE_DSI,
    .data            = &dsi1_panel,
    .phy.dsi        = {
        .clk_lane    = 1,
        .clk_pol    = 0,
        .data1_lane    = 2,
        .data1_pol    = 0,
        .data2_lane    = 3,
        .data2_pol    = 0,
    },

    .clocks = {
        .dispc = {
            .channel = {
                .lck_div    = 1,    /* Logic Clock = 172.8 MHz */
                .pck_div    = 5,    /* Pixel Clock = 34.56 MHz */
                .lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
            },
            .dispc_fclk_src    = OMAP_DSS_CLK_SRC_FCK,
        },

        .dsi = {
            .regn        = 16,    /* Fint = 2.4 MHz */
            .regm        = 180,    /* DDR Clock = 216 MHz */
            .regm_dispc    = 5,    /* PLL1_CLK1 = 172.8 MHz */
            .regm_dsi    = 5,    /* PLL1_CLK2 = 172.8 MHz */

            .lp_clk_div    = 10,    /* LP Clock = 8.64 MHz */
            .dsi_fclk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
        },
    },
    .channel = OMAP_DSS_CHANNEL_LCD,
    .skip_init = false,
};
#endif
#ifdef CONFIG_PANEL_HUAWEI
static struct nokia_dsi_panel_data dsi_panel = {
        .name    = "taal_huawei",
        .reset_gpio    = 102,
        .use_ext_te    = false,
        .ext_te_gpio    = 101,
        .esd_interval    = false,
        .set_backlight    = NULL,
};

static struct omap_dss_device huawei_sdp4430_lcd_device = {
    .name            = "lcd",
    .driver_name        = "taal_huawei",
    .type            = OMAP_DISPLAY_TYPE_DSI,
    .data            = &dsi1_panel,
    .phy.dsi        = {
        .clk_lane    = 1,
        .clk_pol    = 0,
        .data1_lane    = 2,
        .data1_pol    = 0,
        .data2_lane    = 3,
        .data2_pol    = 0,
    },

    .clocks = {
        .dispc = {
            .channel = {
                .lck_div    = 1,    /* Logic Clock = 172.8 MHz */
                .pck_div    = 5,    /* Pixel Clock = 34.56 MHz */
                .lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
            },
            .dispc_fclk_src    = OMAP_DSS_CLK_SRC_FCK,
        },

        .dsi = {
            .regn        = 16,    /* Fint = 2.4 MHz */
            .regm        = 180,    /* DDR Clock = 216 MHz */
            .regm_dispc    = 5,    /* PLL1_CLK1 = 172.8 MHz */
            .regm_dsi    = 5,    /* PLL1_CLK2 = 172.8 MHz */

            .lp_clk_div    = 10,    /* LP Clock = 8.64 MHz */
            .dsi_fclk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
        },
    },
    .channel        = OMAP_DSS_CHANNEL_LCD,
};

/*
static struct omap_dss_device huawei_sdp4430_lcd2_device = {
    .name            = "lcd2",
    .driver_name        = "taal2_huawei",
    .type            = OMAP_DISPLAY_TYPE_DSI,
    .data            = &dsi2_panel,
    .phy.dsi        = {
        .clk_lane    = 1,
        .clk_pol    = 0,
        .data1_lane    = 2,
        .data1_pol    = 0,
        .data2_lane    = 3,
        .data2_pol    = 0,
        .div        = {
            .lck_div    = 1,
            .pck_div    = 5,
            .regm        = 150,
            .regn        = 17,
            .regm_dispc    = 4,
            .regm_dsi    = 4,
            .lp_clk_div    = 8,
        },
    },
    .channel        = OMAP_DSS_CHANNEL_LCD2,
};
*/
#endif

static struct sp_dsi_panel_data dsi_panel_sp = {
                .name   = "spanel",
                .reset_gpio     = 38,
                .use_ext_te     = false,
                .ext_te_gpio    = 101,
                .use_esd_check  = false,
                .set_backlight  = NULL,
};

static struct omap_dss_device t0_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "spanel",
        .type                   = OMAP_DISPLAY_TYPE_DSI,
        .data                   = &dsi_panel_sp,
        .phy.dsi                = {
                .clk_lane       = 1,
                .clk_pol        = 0,
                .data1_lane     = 2,
                .data1_pol      = 0,
                .data2_lane     = 3,
                .data2_pol      = 0,
         },
         .clocks = {
             .dispc = {
            .channel = {
                .lck_div    = 1,
                .pck_div    = 4,
                .lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
            },
            .dispc_fclk_src    = OMAP_DSS_CLK_SRC_FCK,
        },
            .dsi     = {
                .regn        = 13,
            .regm        = 260,
            .regm_dispc    = 7,
            .regm_dsi    = 7,

            .lp_clk_div    = 8,    /* LP Clock = 8.64 MHz */
            .dsi_fclk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
            .offset_ddr_clk =1,
                },
                //.xfer_mode = OMAP_DSI_XFER_CMD_MODE,
    },
/*
        .panel                  = {
                .width_in_mm = 50,
                .height_in_mm = 89,
        },
*/
        .channel                = OMAP_DSS_CHANNEL_LCD,
};

#ifdef CONFIG_PANEL_TOSHIBA_MDV20
#define PANEL_GPIO_RESET       38
#define PANEL_GPIO_POWER_5V4   157

static struct toshiba_dsi_panel_data mdv20_dsi_panel = {
        .name   = "mdv20",
        .reset_gpio     = PANEL_GPIO_RESET,
        .use_ext_te     = false,
        .ext_te_gpio    = -1,
        .v5_4_enable = PANEL_GPIO_POWER_5V4,
        .chip_pwr_save = -1,
        .use_esd_check  = true,
        .set_backlight  = NULL,
};

static struct omap_dss_device front_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "mdv20",
        .type                   = OMAP_DISPLAY_TYPE_DSI,
        .data                   = &mdv20_dsi_panel,
        .phy.dsi                = {
                .clk_lane       = 1,
                .clk_pol        = 0,
                .data1_lane     = 2,
                .data1_pol      = 0,
                .data2_lane     = 3,
                .data2_pol      = 0,
                .data3_lane     = 4,
                .data3_pol      = 0,
                .data4_lane     = 5,
                .data4_pol      = 0,
                .type           = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
        },
        .clocks                 = {
                .dispc          = {
                        .channel        = {
                                .lck_div     = 1,
                                .pck_div     = 2,
                                .lcd_clk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
                        },
                        .dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
                },
                .dsi            = {
                        .regn           = 11,    /* DSI_PLL_REGN */
                        .regm           = 208,   /* 207-DSI_PLL_REGM */
                        .regm_dispc     = 6,     /* PLL_CLK1 (M4) */
                        .regm_dsi       = 6,     /* PLL_CLK2 (M5) */
                        .lp_clk_div     = 17,    /* LPDIV */
            .dsi_fclk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
            .offset_ddr_clk =1,
                },
        },
        /*
        .panel                  = {
                .width_in_mm = 210,
                .height_in_mm = 158,
        },
        */
        .channel                = OMAP_DSS_CHANNEL_LCD,
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
		.skip_init = true,
#else
		.skip_init = false,
#endif
        .platform_enable = NULL,
    .platform_disable = NULL,
};
#endif

static struct omap_dss_device sdp4430_hdmi_device = {
    .name = "hdmi",
    .driver_name = "hdmi_panel",
    .type = OMAP_DISPLAY_TYPE_HDMI,
    .clocks    = {
        .dispc    = {
            .dispc_fclk_src    = OMAP_DSS_CLK_SRC_FCK,
        },
        .hdmi    = {
            .regn    = 10,
            .regm2    = 1,
        },
    },
    .hpd_gpio = HDMI_GPIO_HPD,
    .channel = OMAP_DSS_CHANNEL_DIGIT,
};

/*  Reason: lcd-compatible  */
static struct omap_dss_device *sdp4430_dss_devices[] = {
#ifdef CONFIG_PANEL_TAAL
    &sdp4430_lcd_device,
#endif
#ifdef CONFIG_PANEL_HUAWEI
    &huawei_sdp4430_lcd_device,
    //&huawei_sdp4430_lcd2_device,
#endif
#ifdef CONFIG_PANEL_SP_HUAWEI
    &t0_lcd_device,
#endif
#ifdef CONFIG_PANEL_TOSHIBA_MDV20
        &front_lcd_device,
#endif
#ifdef CONFIG_OMAP4_DSS_HDMI
    &sdp4430_hdmi_device,
#endif
};

static struct omap_dss_board_info sdp4430_dss_data = {
    .num_devices    = ARRAY_SIZE(sdp4430_dss_devices),
    .devices    = sdp4430_dss_devices,
#if defined(CONFIG_PANEL_TAAL)
    .default_device    =    &sdp4430_lcd_device,
#elif defined(CONFIG_PANEL_HUAWEI)
    .default_device    =    &huawei_sdp4430_lcd_device,
#elif defined(CONFIG_PANEL_SP_HUAWEI)
    .default_device    =    &t0_lcd_device,
#elif defined(CONFIG_PANEL_TOSHIBA_MDV20)
    .default_device =    &front_lcd_device,
#else
    .default_device = NULL,
#endif
};

#define BLAZE_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */
static struct omapfb_platform_data blaze_fb_pdata = {
    .mem_desc = {
        .region_cnt = 1,
        .region = {
            [0] = {
                .size = BLAZE_FB_RAM_SIZE,
            },
        },
    },
};


static void sdp4430_lcd_init(void)
{
    u32 reg;
    u32 temp = 0;
    int status;

    /* Enable 3 lanes in DSI1 module, disable pull down */
    reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
    //reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
    //reg |= 0x7 << OMAP4_DSI1_LANEENABLE_SHIFT;
    //reg &= ~OMAP4_DSI1_PIPD_MASK;
    //reg |= 0x7 << OMAP4_DSI1_PIPD_SHIFT;
    if(sdp4430_dss_data.default_device->phy.dsi.data3_lane)
        reg = OMAP4_DSI1_LANEENABLE_DATA|OMAP4_DSI1_FRONT_PIPD_DATA_MASK;
    else
        reg = OMAP4_DSI1_LANEENABLE_DATA|OMAP4_DSI1_VIVA_PIPD_DATA_MASK;

    temp = !sdp4430_dss_data.default_device->phy.dsi.data1_lane; /*lane 1*/
    reg &=  ~(temp << 25);

    temp = !sdp4430_dss_data.default_device->phy.dsi.data2_lane; /*lane 2*/
    reg &=  ~(temp << 26);

    temp = !sdp4430_dss_data.default_device->phy.dsi.data3_lane; /*lane 3*/
    reg &=  ~(temp << 27);

    temp = !sdp4430_dss_data.default_device->phy.dsi.data4_lane; /*lane 4*/
    reg &=  ~(temp << 28);

    omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);

#if 0
    /* Panel Taal reset and backlight GPIO init */
    status = gpio_request_one(dsi1_panel.reset_gpio, GPIOF_DIR_OUT,
        "lcd_reset_gpio");
    if (status)
        pr_err("%s: Could not get lcd_reset_gpio\n", __func__);

    if (dsi1_panel.use_ext_te) {
        status = omap_mux_init_signal("gpmc_ncs4.gpio_101",
                OMAP_PIN_INPUT_PULLUP);
        if (status)
            pr_err("%s: Could not get ext_te gpio\n", __func__);
    }
#endif

#ifdef CONFIG_PANEL_SP_HUAWEI
    gpio_request(dsi_panel_sp.reset_gpio, "dsi_en_gpio");
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
    gpio_direction_output(dsi_panel_sp.reset_gpio, 1);
#endif
#endif

#ifdef CONFIG_PANEL_TOSHIBA_MDV20
        gpio_request(mdv20_dsi_panel.v5_4_enable,"led_pow_en");
        gpio_direction_output(mdv20_dsi_panel.v5_4_enable,1);
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
        gpio_request(mdv20_dsi_panel.reset_gpio, "dsi_en_gpio");
        gpio_direction_output(mdv20_dsi_panel.reset_gpio, 1);
        gpio_request(mdv20_dsi_panel.chip_pwr_save,"led_pow_pd");
        gpio_direction_output(mdv20_dsi_panel.chip_pwr_save,0);
#else
        gpio_request(mdv20_dsi_panel.reset_gpio, "dsi_en_gpio");
        gpio_request(mdv20_dsi_panel.chip_pwr_save,"led_pow_pd");
#endif
#endif
#if 0
    status = gpio_request_one(LCD_BL_GPIO, GPIOF_DIR_OUT, "lcd_bl_gpio");
    if (status)
        pr_err("%s: Could not get lcd_bl_gpio\n", __func__);

    gpio_set_value(LCD_BL_GPIO, 0);
#endif
}


static void omap_4430sdp_display_init(void)
{
/*  Reason: lcd-compatible  */
#ifndef CONFIG_PANEL_HUAWEI
    panel_set_support_devices(&sdp4430_dss_data);
#endif
    sdp4430_lcd_init();
    sdp4430_hdmi_mux_init();
    omap_vram_set_sdram_vram(BLAZE_FB_RAM_SIZE, 0);
    omapfb_set_platform_data(&blaze_fb_pdata);
    omap_display_init(&sdp4430_dss_data);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
//the GPIO configuration has moved to the excel GPIO config.
//the path locates in kernel/drivers/huawei/hsad/$PRODUCTION/gpio_mux.xls
#if 0
#ifdef CONFIG_HUAWEI_GPIO_KEYPAD
    OMAP4_MUX(GPMC_A18, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
    OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
#if 0
    OMAP4_MUX(UNIPRO_RX0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
    OMAP4_MUX(UNIPRO_RY0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
    OMAP4_MUX(UNIPRO_RX1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
    OMAP4_MUX(UNIPRO_RY1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
#endif
#endif

#ifndef  CONFIG_PANEL_TOSHIBA_MDV20
    /* Mux gpio-157 to USBB2_ULPITLL_CLK pad which is used to enable
    * power to modem circuit required when using HSI and EHCI drivers.
    */
    //OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
    OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_INPUT_EN),
#endif

    OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
    OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
    OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
    /* IO optimization pdpu and offmode settings to reduce leakage */
    OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
    OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
    OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
    OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
                     | OMAP_OFF_EN | OMAP_OFF_PULL_EN),
    OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
    OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
                     | OMAP_OFF_EN | OMAP_OFF_PULL_EN),
    OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
    OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_INPUT_EN),
    /*OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
                    | OMAP_OFF_EN | OMAP_OFF_PULL_EN),*/
    OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
                    | OMAP_OFF_EN | OMAP_OFF_PULL_EN),
    OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_OFF_EN
                | OMAP_OFF_PULL_EN),

//#ifdef CONFIG_KEYPAD_ATMEL_TOUCH
    //OMAP4_MUX(MCSPI4_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
//#endif
#ifdef CONFIG_HUAWEI_MHL_SII9244
    OMAP4_MUX(UNIPRO_TY2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
#endif
      OMAP4_MUX(FREF_CLK4_OUT,OMAP_MUX_MODE0),
      OMAP4_MUX(USBB2_ULPITLL_DAT1,OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA | OMAP_PULL_UP),
    OMAP4_MUX(GPMC_AD11, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
    OMAP4_MUX(GPMC_AD12, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT| OMAP_PULL_ENA | OMAP_PULL_UP),
    OMAP4_MUX(MCSPI1_CS0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT| OMAP_PULL_ENA | OMAP_PULL_UP),
    OMAP4_MUX(GPMC_AD14, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA | OMAP_PULL_UP),

#ifdef    CONFIG_KEYPAD_CYPRESS_TOUCH
    OMAP4_MUX(MCSPI4_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP), //for cypress gpio_151 irq
    OMAP4_MUX(MCSPI4_SIMO, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), //for cypress gpio_152 reset
#endif

    OMAP4_MUX(GPMC_WAIT2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN),
#ifdef CONFIG_PANEL_TOSHIBA_MDV20
    OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA | OMAP_PULL_UP),//gpio 157
    OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA | OMAP_PULL_UP),//gpio 158
#endif
#endif
   { .reg_offset = OMAP_MUX_TERMINATOR },
};

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *    EMIF1 - CS0 -    2 Gb
 *        CS1 -    2 Gb
 *    EMIF2 - CS0 -    2 Gb
 *        CS1 -    2 Gb
 *    --------------------
 *    TOTAL -        8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
#if defined(CONFIG_ELPIDA_DDR_2G_S4)
static __initdata struct emif_device_details emif_devices = {
    .cs0_device = &elpida_2G_S4,
    .cs1_device = &elpida_2G_S4
};
#elif defined(CONFIG_SAMSUNG_DDR_4G_S4)
static __initdata struct emif_device_details emif_devices = {
    .cs0_device = &samsung_4G_S4,
    .cs1_device = NULL
};
#else
static __initdata struct emif_device_details emif_devices = {
    .cs0_device = &elpida_2G_S4,
    .cs1_device = &elpida_2G_S4
};
#endif

#else
#define board_mux    NULL
#define board_wkup_mux NULL
#endif

static struct omap_device_pad blaze_uart1_pads[] __initdata = {
    {
        .name    = "uart1_cts.uart1_cts",
        .enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
    },
    {
        .name    = "uart1_rts.uart1_rts",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
    },
    {
        .name    = "uart1_tx.uart1_tx",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
    },
    {
        .name    = "uart1_rx.uart1_rx",
        .flags    = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
        .enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
        .idle    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
    },
};


static struct omap_device_pad blaze_uart2_pads[] __initdata = {
    {
        .name    = "uart2_cts.uart2_cts",
        .enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
        .flags  = OMAP_DEVICE_PAD_REMUX,
        .idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
              OMAP_MUX_MODE0,
    },
    {
        .name    = "uart2_rts.uart2_rts",
//        .flags  = OMAP_DEVICE_PAD_REMUX,
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
//        .idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
    },
    {
        .name    = "uart2_tx.uart2_tx",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
    },
    {
        .name    = "uart2_rx.uart2_rx",
        .enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
    },
};

/* This modification recorrect the index of uart pad group and relation setting */
static struct omap_device_pad blaze_uart3_pads[] __initdata = {
    {
        //.name    = "uart3_cts_rctx.uart3_cts_rctx",
        //.enable    =  OMAP_MUX_MODE1,
        .name    = "dpm_emu9.uart3_cts_rctx",
        .enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE2,
    },
    {
        //.name    = "uart3_rts_sd.uart3_rts_sd",
        //.enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
        .name    = "dpm_emu8.uart3_rts_sd",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE2,
    },
    {
        //.name    = "uart3_tx_irtx.uart3_tx_irtx",
        //.enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
        .name    = "dpm_emu6.uart3_tx_irtx",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE2,
    },
    {
        //.name    = "uart3_rx_irrx.uart3_rx_irrx",
        //.flags    = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
        //.enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3,//mode is GPIO143 and enable pullup trigger by wanguanglin 20110914
        //.idle    = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
        .name    = "dpm_emu7.uart3_rx_irrx",
        .enable   = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE2,
    },
};

static struct omap_device_pad blaze_uart4_pads[] __initdata = {
    {
        .name    = "abe_dmic_clk1.uart4_cts",
        .enable    = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE5 | OMAP_OFF_EN | OMAP_OFFOUT_EN | OMAP_OFF_PULL_EN,
    },
    {
        .name    = "abe_dmic_din1.uart4_rts",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE5 | OMAP_OFF_EN,
    },
    {
        .name    = "uart4_tx.uart4_tx",
        .enable    = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
    },
    {
        .name    = "uart4_rx.uart4_rx",
        .flags    = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
        .enable    = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
        .idle    = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
    },
};

static struct omap_uart_port_info blaze_uart_info_uncon __initdata = {
    .use_dma    = 0,
    .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = 0,
};

static struct omap_uart_port_info blaze_uart_info __initdata = {
    .use_dma    = 0,
   .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
 //   .auto_sus_timeout = -1,
        //.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
        .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_CTS),
};
static struct omap_uart_port_info blaze_uart_info_bt __initdata = {
    .use_dma    = 0,
    .auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
    .rts_mux_driver_control = 1,
    .wer = 0,
};
static inline void __init board_serial_init(void)
{
    omap_serial_init_port_pads(0, blaze_uart1_pads,
        ARRAY_SIZE(blaze_uart1_pads), &blaze_uart_info_uncon);
    omap_serial_init_port_pads(1, blaze_uart2_pads,
        ARRAY_SIZE(blaze_uart2_pads), &blaze_uart_info_bt);
    omap_serial_init_port_pads(2, blaze_uart3_pads,
        ARRAY_SIZE(blaze_uart3_pads), &blaze_uart_info);
    omap_serial_init_port_pads(3, blaze_uart4_pads,
        ARRAY_SIZE(blaze_uart4_pads), &blaze_uart_info_uncon);
}

static void omap4_sdp4430_wifi_mux_init(void)
{
    omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
                OMAP_PIN_OFF_WAKEUPENABLE);
    omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

    omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data omap4_sdp4430_wlan_data __initdata = {
    .irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
    .board_ref_clock = WL12XX_REFCLOCK_26,
    .board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};

#if 0
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS

static char buf_virtualkey[500];
static ssize_t  buf_vkey_size=0;
/* add virtual keys fucntion */
static ssize_t synaptics_virtual_keys_show(struct kobject *kobj,
                   struct kobj_attribute *attr, char *buf)
{
        memcpy( buf, buf_virtualkey, buf_vkey_size );
        return buf_vkey_size;
}

static struct kobj_attribute synaptics_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.synaptics",
        .mode = S_IRUGO,
    },
    .show = &synaptics_virtual_keys_show,
};

static struct attribute *synaptics_properties_attrs[] = {
    &synaptics_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group synaptics_properties_attr_group = {
    .attrs = synaptics_properties_attrs,
};

static void __init virtualkeys_init(void)
{
    //struct kobject *properties_kobj;
    extern struct kobject *prop_kobj_virtual;
    int ret;
    /* calibrate virtual key's coordinate */
    if (true)
    {
        /* calibrate virtual key's coordinate */
        buf_vkey_size = sprintf(buf_virtualkey,
                     __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":115:1325:180:70"
                    ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":360:1325:180:70"
                    ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":605:1325:180:70"
                   "\n");
    }

       //properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (prop_kobj_virtual)
        ret = sysfs_create_group(prop_kobj_virtual,
                     &synaptics_properties_attr_group);
    if (!prop_kobj_virtual || ret)
        pr_err("failed to create board_properties\n");
}
#endif
#endif

static void omap4_sdp4430_wifi_init(void)
{
    omap4_sdp4430_wifi_mux_init();
    if (wl12xx_set_platform_data(&omap4_sdp4430_wlan_data))
        pr_err("Error setting wl12xx data\n");
    platform_device_register(&omap_vwlan_device);
}

static void configGPS()
{
    omap_mux_init_gpio(GPIO_GPS_POWER, OMAP_PIN_OUTPUT);
    omap_mux_init_gpio(GPIO_GPS_RESET, OMAP_PIN_OUTPUT);

    if(gpio_request(GPIO_GPS_POWER, "gps_reset")>=0){
        gpio_direction_output(GPIO_GPS_POWER, 0);
        gpio_set_value(GPIO_GPS_POWER, 0);
        gpio_export(GPIO_GPS_POWER, false);
    }

    if(gpio_request(GPIO_GPS_RESET, "gps_ctr")>=0){
        gpio_direction_output(GPIO_GPS_RESET, 0);
        gpio_set_value(GPIO_GPS_RESET, 1);
        gpio_export(GPIO_GPS_RESET, false);
    }
}

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
struct usbhs_omap_board_data usbhs_bdata __initdata = {
    .port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
    .port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
    .port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
    .phy_reset  = false,
    .reset_gpio_port[0]  = -EINVAL,
    .reset_gpio_port[1]  = -EINVAL,
    .reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_ohci_init(void)
{
#if 0
    omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
        OMAP_PIN_OUTPUT | \
        OMAP_PIN_OFF_NONE);

    /* Power on the ULPI PHY */
    if (gpio_is_valid(BLAZE_MDM_PWR_EN_GPIO)) {
        gpio_request(BLAZE_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
        gpio_direction_output(BLAZE_MDM_PWR_EN_GPIO, 1);
    }

    usbhs_init(&usbhs_bdata);
#endif
    return;

}
#else
static void __init omap4_ehci_ohci_init(void){}
#endif

static void blaze_set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_set_osc_lp_time(4000, 1);
}

static int blaze_notifier_call(struct notifier_block *this,
                    unsigned long code, void *cmd)
{
    void __iomem *sar_base;
    u32 v = 0;

    sar_base = omap4_get_sar_ram_base();

    if (!sar_base)
        return notifier_from_errno(-ENOMEM);
    if (code == SYS_RESTART && cmd == NULL)
    {
        strcpy(sar_base + 0xA0C, "huawei_reboot");
        v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
    }

    if ((code == SYS_RESTART) && (cmd != NULL)) {
        /* cmd != null; case: warm boot */
        if (!strcmp(cmd, "bootloader")) {
            /* Save reboot mode in scratch memory */
            strcpy(sar_base + 0xA0C, cmd);
            v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
        } else if (!strcmp(cmd, "recovery")) {
            /* Save reboot mode in scratch memory */
            strcpy(sar_base + 0xA0C, cmd);
            v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;

        } else if (!strcmp(cmd, "resetfactory")) {

            /* Save reboot mode in scratch memory */
            strcpy(sar_base + 0xA0C, cmd);

            v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
        } else if (!strcmp(cmd, "resetuser")) {

            /* Save reboot mode in scratch memory */
            strcpy(sar_base + 0xA0C, cmd);

            v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
        } else if (!strcmp(cmd, "sdupdate")) {

            /* Save reboot mode in scratch memory */
            strcpy(sar_base + 0xA0C, cmd);

            v |= OMAP4430_RST_GLOBAL_WARM_SW_MASK;
/* The device is charging in recovery mode;When the power-off alarm irq is triggered,the device need to be rebooted*/
	}else if (!strcmp(cmd, "Recovery_alarm_reboot")) {
		strcpy(sar_base + 0xA0C, "Recovery_alarm_reboot");
		v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
	}else if (!strcmp(cmd, "charger_poweroff_reboot")) {
		strcpy(sar_base + 0xA0C, "charger_poweroff_reboot");
		v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
	}else if (!strcmp(cmd, "resize")) {
		strcpy(sar_base + 0xA0C, "resize");
		v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
	}else {
            strcpy(sar_base + 0xA0C, "huawei_reboot");
            v |= OMAP4430_RST_GLOBAL_COLD_SW_MASK;
        }

    /*omap4_prm_write_inst_reg(0xfff, OMAP4430_PRM_DEVICE_INST,
            OMAP4_RM_RSTST);
    omap4_prm_write_inst_reg(v, OMAP4430_PRM_DEVICE_INST, OMAP4_RM_RSTCTRL);
    v = omap4_prm_read_inst_reg(WKUP_MOD, OMAP4_RM_RSTCTRL);*/
    }

    return NOTIFY_DONE;
}

static struct notifier_block blaze_reboot_notifier = {
    .notifier_call = blaze_notifier_call,
};

/*
 * As OMAP4430 mux HSI and USB signals, when HSI is used (for instance HSI
 * modem is plugged) we should configure HSI pad conf and disable some USB
 * configurations.
 * HSI usage is declared using bootargs variable:
 * board-4430sdp.modem_ipc=hsi
 * Any other or missing value will not setup HSI pad conf, and port_mode[0]
 * will be used by USB.
 * Variable modem_ipc is used to catch bootargs parameter value.
 */
#ifdef CONFIG_MACH_OMAP_XMM
        static char *modem_ipc = "hsi";
#else  //#ifdef CONFIG_MACH_OMAP_XMM
        static char *modem_ipc = "n/a";
#endif  //#ifdef CONFIG_MACH_OMAP_XMM
module_param(modem_ipc, charp, 0);
MODULE_PARM_DESC(modem_ipc, "Modem IPC setting");

static void __init omap_4430sdp_init(void)
{
    int status;
    int package = OMAP_PACKAGE_CBS;

    board_muxconf_init();

    if (omap_rev() == OMAP4430_REV_ES1_0)
        package = OMAP_PACKAGE_CBL;
    omap4_mux_init(board_mux, NULL, package);

    omap_emif_setup_device_details(&emif_devices, &emif_devices);

    omap_board_config = sdp4430_config;
    omap_board_config_size = ARRAY_SIZE(sdp4430_config);

    omap_init_board_version(0);

    omap4_audio_conf();
#ifdef CONFIG_HUAWEI_NFC_PN544
    omap_clk3_init();
#endif
    omap4_create_board_props();
    register_reboot_notifier(&blaze_reboot_notifier);
    blaze_pmic_mux_init();
    blaze_set_osc_timings();
    omap4_i2c_init();
    blaze_sensor_init();
    blaze_touch_init();
    omap4_register_ion();
    omap_bluetooth_init();
    platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
    wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
    omap4_twl6030_hsmmc_init(mmc);
    board_serial_init();
#ifdef CONFIG_TIWLAN_SDIO
    config_wlan_mux();
#else
    omap4_4430sdp_wifi_init();
#endif

    configGPS();


    /* blaze_modem_init shall be called before omap4_ehci_ohci_init */
    pr_info("Configured modem_ipc: %s", modem_ipc);
    if (!strcmp(modem_ipc, "hsi")) {
        pr_info("Modem HSI detected, set USB port_mode[0] as UNUSED");
        //blaze_modem_init(true);   //for blaze modem hsi_interface

        /* USBB1 I/O pads conflict with HSI1 port */
        usbhs_bdata.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED;
        /* USBB2 I/O pads conflict with McBSP2 port */
        usbhs_bdata.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED;
        omap_hsi_allow_registration();
    }
    else {
             pr_info("Modem HSI not detected");
        //blaze_modem_init(false);
    }
    omap_4430sdp_display_init();
    omap4_ehci_ohci_init();

    usb_musb_init(&musb_board_data);

#if 0
    status = omap_ethernet_init();
    if (status) {
        pr_err("Ethernet initialization failed: %d\n", status);
    } else {
        sdp4430_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
        spi_register_board_info(sdp4430_spi_board_info,
                ARRAY_SIZE(sdp4430_spi_board_info));
    }
#endif


#ifdef CONFIG_HUAWEI_GPIO_KEYPAD
    platform_add_devices(huawei_devices, ARRAY_SIZE(huawei_devices));
#endif
#ifdef CONFIG_KEYBOARD_OMAP4
    keyboard_mux_init();
    status = omap4_keyboard_init(&sdp4430_keypad_data);
    if (status)
        pr_err("Keypad initialization failed: %d\n", status);
#endif

    omap_dmm_init();
    //omap_4430sdp_display_init();

#if 0
    blaze_panel_init();
    blaze_keypad_init();
#endif

        omap_bq27510battery_init();//add bq27510 bat_low gpio


    if (cpu_is_omap446x()) {
        /* Vsel0 = gpio, vsel1 = gnd */
        status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
                    OMAP_PIN_OFF_OUTPUT_HIGH, -1);
        if (status)
            pr_err("TPS62361 initialization failed: %d\n", status);
    }

    omap_enable_smartreflex_on_init();
        if (enable_suspend_off)
                omap_pm_enable_off_mode();



#ifdef CONFIG_MACH_OMAP_XMM
    status=xmd_board_init(&huawei_xmd_platform_data); 
    if( status!=0 )
        pr_info("board-4430sdp: xmd_board_init failed: err=%d. %s %d\n", status,__func__, __LINE__);
    else
        pr_info("board-4430sdp: xmd_board_init success. %s %d\n",__func__, __LINE__);
#endif  //#ifdef CONFIG_MACH_OMAP_XMM

#if 0
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS
    virtualkeys_init();
#endif
#endif

#ifdef CONFIG_SWITCH_USB
    init_usb_switch();
#endif
#ifdef CONFIG_BOARD_ID
    config_debugfs_init();
#endif
#if defined(CONFIG_GET_LPDDR2_INFO)
    setup_lpddr2_debugfs();
#endif

}

static void __init omap_4430sdp_map_io(void)
{
    omap2_set_globals_443x();
    omap44xx_map_common_io();
}
static void __init omap_4430sdp_reserve(void)
{
    omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
                    OMAP_RAM_CONSOLE_SIZE_DEFAULT);

    /* do the static reservations first */
    memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
    memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
    /* ipu needs to recognize secure input buffer area as well */
    omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
                    OMAP4_ION_HEAP_SECURE_INPUT_SIZE);
#ifdef CONFIG_ION_OMAP
    omap_ion_init();
#endif

    omap_reserve();
}

MACHINE_START(OMAP_4430SDP, "OMAP4460")
    /* Maintainer: Santosh Shilimkar - Texas Instruments Inc */
    .boot_params    = 0x80000000,
    .reserve    = omap_4430sdp_reserve,
    .map_io        = omap_4430sdp_map_io,
    .init_early    = omap_4430sdp_init_early,
    .init_irq    = gic_init_irq,
    .init_machine    = omap_4430sdp_init,
    .timer        = &omap_timer,
MACHINE_END
