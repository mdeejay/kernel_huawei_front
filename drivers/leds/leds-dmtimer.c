/*
 * LEDs driver for GPIOs
 *
 * Copyright (C) 2007 8D Technologies inc.
 * Raphael Assenat <raph@8d.com>
 * Copyright (C) 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <asm/gpio.h>
#include <linux/leds_pwm.h>
#include <linux/pwm.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/core.h>
#include <linux/mfd/twl6040-codec.h>
#include <hsad/config_interface.h>
#include <linux/io.h>
#include <linux/delay.h>
#include "leds.h"

#define SET_HIGHIST_BIT         0x80
#define PWM_VALUE_RANGE         64
#define PWM_RANGE_RATIO         4
#define LED_PWM1_ON             0xba
#define LED_PWM2_ON             0xbd
#define LED_PWM_EN              0x92
#define LED_PWM_EN_set          0x36


struct gpio_led_data {
	struct led_classdev cdev;
	unsigned gpio;
	struct work_struct work;
	u8 new_level;
	u8 can_sleep;
	u8 active_low;
	u8 blinking;
	int (*platform_gpio_blink_set)(unsigned gpio, int state,
			unsigned long *delay_on, unsigned long *delay_off);
};


static void gptimer_set_value(struct led_classdev *gptimer_led_cdev,int level)
{
        void __iomem *phymux_base = NULL;
        void __iomem *phymux_base1 = NULL;

        struct gpio_led_data *led_dat =
                container_of(gptimer_led_cdev, struct gpio_led_data, cdev);

        phymux_base1 = ioremap(GPTIMER8_CLKCTRL,0x4);
        raw_reg_writel(GPTIMER_CLKCTRL_VAL,phymux_base1);
        iounmap(phymux_base1);


        phymux_base1 = ioremap(GPTIMER10_CLKCTRL,0x4);
        raw_reg_writel(GPTIMER_CLKCTRL_VAL,phymux_base1);
        iounmap(phymux_base1);
        udelay(150);

        if(!strcmp(gptimer_led_cdev->name,"red"))//dmtimer8_pwm_evt  gpio27
        {
                phymux_base = ioremap(GPT_8_ADDR, MEM_LONGTH);
                raw_reg_writel(0x00001842,phymux_base+GPT_TCLR_8_OFFSET);//GPT_TCLR   start
                udelay(300);
                raw_reg_writel(0xfffffffe,phymux_base+GPT_TCRR_8_OFFSET);//TCRR
                if(level)
                {
                        raw_reg_writel(0x1,phymux_base+GPT_TLDR_8_OFFSET);//GPT_TLDR
                        raw_reg_writel(0xfffffffe,phymux_base+GPT_TMAR_8_OFFSET);//GPT_TMAR
                }
                else
                {
                        raw_reg_writel(0x01,phymux_base+GPT_TLDR_8_OFFSET);//GPT_TLDR
                        raw_reg_writel(0x02,phymux_base+GPT_TMAR_8_OFFSET);//GPT_TMAR
                }
                raw_reg_writel(GPT_TCLR_VAL,phymux_base+GPT_TCLR_8_OFFSET);//GPT_TCLR   start
                raw_reg_writel(0xfffffffe,phymux_base+GPT_TCRR_8_OFFSET);//TCRR
                iounmap(phymux_base);
        }
        else if(!strcmp(gptimer_led_cdev->name,"green")) //dmtimer10_pwm_evt gpio190
        {
                phymux_base = ioremap(GPT_10_ADDR, MEM_LONGTH);
                raw_reg_writel(0x00001842,phymux_base+GPT_TCLR_10_OFFSET);//GPT_TCLR  start
                udelay(300);
                raw_reg_writel(0xfffffffe,phymux_base+GPT_TCRR_10_OFFSET);//TCRR
                if(level)
                {
                        raw_reg_writel(0x1,phymux_base+GPT_TLDR_10_OFFSET);//GPT_TLDR
                        raw_reg_writel(0xfffffffe,phymux_base+GPT_TMAR_10_OFFSET);//GPT_TMAR
                }
                else
                {
                        raw_reg_writel(0x1,phymux_base+GPT_TLDR_10_OFFSET);//GPT_TLDR
                        raw_reg_writel(0x2,phymux_base+GPT_TMAR_10_OFFSET);//GPT_TMAR
                }
                raw_reg_writel(GPT_TCLR_VAL,phymux_base+GPT_TCLR_10_OFFSET);//GPT_TCLR  start
                raw_reg_writel(0xfffffffe,phymux_base+GPT_TCRR_10_OFFSET);//TCRR
                iounmap(phymux_base);
        }
        else if(!strcmp(gptimer_led_cdev->name,"blue")) //only used in front T1.5
        {
                if(get_board_id_int() == BOARD_ID_FRONTT1P5)
                {
                        gpio_set_value(led_dat->gpio, level);
                }
        }

}

static void gpio_led_work(struct work_struct *work)
{
	struct gpio_led_data	*led_dat =
		container_of(work, struct gpio_led_data, work);

	if (led_dat->blinking) {
		led_dat->platform_gpio_blink_set(led_dat->gpio,
						 led_dat->new_level,
						 NULL, NULL);
		led_dat->blinking = 0;
	} else
		gpio_set_value_cansleep(led_dat->gpio, led_dat->new_level);
}

static void gpio_ledtimer_work(struct work_struct *work)
{
        u8 value_pwm_reg = 0;
        int level;
        int value;

        struct gpio_led_data	*led_dat =
            container_of(work, struct gpio_led_data, work);
        value = led_dat->cdev.brightness;
        if (value == LED_OFF)
            level = 0;
        else
	{
	            level = 1;
	}
                        value_pwm_reg = SET_HIGHIST_BIT|(PWM_VALUE_RANGE-value/PWM_RANGE_RATIO);
                        if(!strcmp(led_dat->cdev.name,"red"))
                        {
                                gpio_set_value(led_dat->gpio, level);
                        }
                        else if(!strcmp(led_dat->cdev.name,"green"))
                        {
                                twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_set, LED_PWM_EN); //enable clk
                                twl_i2c_write_u8(TWL6030_MODULE_ID1, value_pwm_reg, LED_PWM1_ON); /*2ms*/
                        }
                        else if(!strcmp(led_dat->cdev.name,"blue"))
                        {
                                twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_set, LED_PWM_EN); //enable clk
                                twl_i2c_write_u8(TWL6030_MODULE_ID1, value_pwm_reg, LED_PWM2_ON);  /*2ms*/
                        }
}
static void gpio_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct gpio_led_data *led_dat =
		container_of(led_cdev, struct gpio_led_data, cdev);
	int level;
	if (value == LED_OFF)
		level = 0;
	else
		level = 1;

	if (led_dat->active_low)
		level = !level;

	/* Setting GPIOs with I2C/etc requires a task context, and we don't
	 * seem to have a reliable way to know if we're already in one; so
	 * let's just assume the worst.
	 */
	if (led_dat->can_sleep) {
		led_dat->new_level = level;
		schedule_work(&led_dat->work);
	} else {
		if (led_dat->blinking) {
			led_dat->platform_gpio_blink_set(led_dat->gpio, level,
							 NULL, NULL);
			led_dat->blinking = 0;
		} else
		{
                switch(get_board_id_int()){
                case BOARD_ID_VIVA:
                {
                        gpio_set_value(led_dat->gpio, level);
                        break;
                }
                case BOARD_ID_FRONT:
                {
                        schedule_work(&led_dat->work);
                        break;
                }
                default:
                {
                        gptimer_set_value(&led_dat->cdev,level);
                        break;
                }
            }
            }
	}
}

static int gpio_blink_set(struct led_classdev *led_cdev,
	unsigned long *delay_on, unsigned long *delay_off)
{
	struct gpio_led_data *led_dat =
		container_of(led_cdev, struct gpio_led_data, cdev);

	led_dat->blinking = 1;
	return led_dat->platform_gpio_blink_set(led_dat->gpio, GPIO_LED_BLINK,
						delay_on, delay_off);
}

static int __devinit create_gpio_led(const struct gpio_led *template,
	struct gpio_led_data *led_dat, struct device *parent,
	int (*blink_set)(unsigned, int, unsigned long *, unsigned long *))
{
	int ret, state;

	led_dat->gpio = -1;
        {
	/* skip leds that aren't available */
	if (!gpio_is_valid(template->gpio)) {
		printk(KERN_INFO "Skipping unavailable LED gpio %d (%s)\n",
				template->gpio, template->name);
		return 0;
	}

	ret = gpio_request(template->gpio, template->name);
	if (ret < 0)
		return ret;
        }
	led_dat->cdev.name = template->name;
	led_dat->cdev.default_trigger = template->default_trigger;
	led_dat->gpio = template->gpio;
	led_dat->can_sleep = gpio_cansleep(template->gpio);
	led_dat->active_low = template->active_low;
	led_dat->blinking = 0;
	if (blink_set) {
		led_dat->platform_gpio_blink_set = blink_set;
		led_dat->cdev.blink_set = gpio_blink_set;
	}
	led_dat->cdev.brightness_set = gpio_led_set;
	if (template->default_state == LEDS_GPIO_DEFSTATE_KEEP)
		state = !!gpio_get_value(led_dat->gpio) ^ led_dat->active_low;
	else
		state = (template->default_state == LEDS_GPIO_DEFSTATE_ON);
	led_dat->cdev.brightness = state ? LED_FULL : LED_OFF;
	if (!template->retain_state_suspended)
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;
	ret = gpio_direction_output(led_dat->gpio, led_dat->active_low ^ state);
	if (ret < 0)
		goto err;
	INIT_WORK(&led_dat->work, gpio_led_work);
	INIT_WORK(&led_dat->work, gpio_ledtimer_work);

	ret = led_classdev_register(parent, &led_dat->cdev);
	if (ret < 0)
		goto err;

	return 0;
err:
	gpio_free(led_dat->gpio);
	return ret;
}

static void delete_gpio_led(struct gpio_led_data *led)
{
	if (!gpio_is_valid(led->gpio))
		return;
	led_classdev_unregister(&led->cdev);
	cancel_work_sync(&led->work);
	gpio_free(led->gpio);
}

struct gpio_leds_priv {
	int num_leds;
	struct gpio_led_data leds[];
};

static inline int sizeof_gpio_leds_priv(int num_leds)
{
	return sizeof(struct gpio_leds_priv) +
		(sizeof(struct gpio_led_data) * num_leds);
}

/* Code to create from OpenFirmware platform devices */
#ifdef CONFIG_LEDS_GPIO_OF
static struct gpio_leds_priv * __devinit gpio_leds_create_of(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct gpio_leds_priv *priv;
	int count = 0, ret;

	/* count LEDs in this device, so we know how much to allocate */
	for_each_child_of_node(np, child)
		count++;
	if (!count)
		return NULL;

	priv = kzalloc(sizeof_gpio_leds_priv(count), GFP_KERNEL);
	if (!priv)
		return NULL;

	for_each_child_of_node(np, child) {
		struct gpio_led led = {};
		enum of_gpio_flags flags;
		const char *state;

		led.gpio = of_get_gpio_flags(child, 0, &flags);
		led.active_low = flags & OF_GPIO_ACTIVE_LOW;
		led.name = of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		state = of_get_property(child, "default-state", NULL);
		if (state) {
			if (!strcmp(state, "keep"))
				led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
			else if (!strcmp(state, "on"))
				led.default_state = LEDS_GPIO_DEFSTATE_ON;
			else
				led.default_state = LEDS_GPIO_DEFSTATE_OFF;
		}

		ret = create_gpio_led(&led, &priv->leds[priv->num_leds++],
				      &pdev->dev, NULL);
		if (ret < 0) {
			of_node_put(child);
			goto err;
		}
	}

	return priv;

err:
	for (count = priv->num_leds - 2; count >= 0; count--)
		delete_gpio_led(&priv->leds[count]);
	kfree(priv);
	return NULL;
}

static const struct of_device_id of_gpio_leds_match[] = {
	{ .compatible = "gpio-leds", },
	{},
};
#else
//del some lines
#define of_gpio_leds_match NULL
#endif

static int __devinit gpio_led_probe(struct platform_device *pdev)
{
	struct gpio_led_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_led_data *leds_data;
	struct gpio_led *gpio_led;
	int i, ret = 0;

	if((get_board_id_int() != BOARD_ID_VIVAT2RF19)
		&&(get_board_id_int() != BOARD_ID_FRONTT1P5)
		&&(get_board_id_int() != BOARD_ID_VIVAT2RF12458)
		&&(get_board_id_int() != BOARD_ID_FRONT))
		return 0;

	if (!pdata)
		return -EBUSY;

leds_data = kzalloc(sizeof(struct gpio_led_data) * pdata->num_leds,
				GFP_KERNEL);
	if (!leds_data)
		return -ENOMEM;

	for (i = 0; i < pdata->num_leds; i++) {

		gpio_led = (struct gpio_led * )&pdata->leds[i];
		if(0 == strcmp(gpio_led->name,"blue"))
		{
			gpio_led->gpio = get_gpio_num_by_name("GPIO_LED_B");
			if(gpio_led->gpio < 0)
			{
				pr_err("%s: get GPIO_LED_B number failed\n",__func__);
				kfree(leds_data);
				return -EINVAL;
			}
		}
		else if(0 == strcmp(gpio_led->name,"red"))
		{
			gpio_led->gpio = get_gpio_num_by_name("GPIO_LED_R");
			if(gpio_led->gpio < 0)
			{
				pr_err("%s: get GPIO_LED_R number failed\n",__func__);
				kfree(leds_data);
				return -EINVAL;
			}
		}
		else if(0 == strcmp(gpio_led->name,"green"))
		{
			gpio_led->gpio = get_gpio_num_by_name("GPIO_LED_G");
			if(gpio_led->gpio < 0)
			{
				pr_err("%s: get GPIO_LED_G number failed\n",__func__);
				kfree(leds_data);
				return -EINVAL;
			}
		}
		else
		{
			pr_err("unrecognized leds name %s\n",gpio_led->name);
			kfree(leds_data);
			return -EINVAL;
		}
		ret = create_gpio_led(gpio_led, &leds_data[i],
				      &pdev->dev, pdata->gpio_blink_set);
		if (ret < 0)
			goto err;
	}

	platform_set_drvdata(pdev, leds_data);

	return 0;

err:
	for (i = i - 1; i >= 0; i--)
		delete_gpio_led(&leds_data[i]);

	kfree(leds_data);

	return ret;

}



static int __devexit gpio_led_remove(struct platform_device *pdev)
{
	int i;

        struct gpio_led_platform_data *pdata = pdev->dev.platform_data;
        struct gpio_led_data *leds_data;
		if((get_board_id_int() != BOARD_ID_VIVAT2RF19)
		&&(get_board_id_int() != BOARD_ID_FRONTT1P5)
		&&(get_board_id_int() != BOARD_ID_VIVAT2RF12458)
		&&(get_board_id_int() != BOARD_ID_FRONT))
		return 0;


	leds_data = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++)
		delete_gpio_led(&leds_data[i]);

	kfree(leds_data);

	return 0;
}

//kill warning
static int gpio_led_suspend(struct platform_device *pdev,pm_message_t state)
{
        void __iomem *phymux_base1 = NULL;
	u32 ret;
	if((get_board_id_int() != BOARD_ID_VIVAT2RF19)
		&&(get_board_id_int() != BOARD_ID_FRONTT1P5)
		&&(get_board_id_int() != BOARD_ID_VIVAT2RF12458))
		return 0;


        phymux_base1 = ioremap(GPTIMER8_CLKCTRL,0x4);
        ret = raw_reg_readl(phymux_base1);
        ret &= 0xfffffffc;

        raw_reg_writel(ret,phymux_base1);
        iounmap(phymux_base1);

        phymux_base1 = ioremap(GPTIMER10_CLKCTRL,0x4);
        ret = raw_reg_readl(phymux_base1);
        ret &= 0xfffffffc;
        raw_reg_writel(ret,phymux_base1);
        iounmap(phymux_base1);
        udelay(150);

        return 0;
}

static struct platform_driver dmtimer_led_driver = {
	.probe		= gpio_led_probe,
	.remove		= __devexit_p(gpio_led_remove),
	.suspend	= gpio_led_suspend,
	.driver		= {
		.name	= "leds-dmtimer",
		.owner	= THIS_MODULE,
		.of_match_table = of_gpio_leds_match,
	},
};

MODULE_ALIAS("platform:leds-gpio");

static int __init dmtimer_led_init(void)
{
	return platform_driver_register(&dmtimer_led_driver);
}

static void __exit dmtimer_led_exit(void)
{
	platform_driver_unregister(&dmtimer_led_driver);
}

module_init(dmtimer_led_init);
module_exit(dmtimer_led_exit);

MODULE_AUTHOR("Raphael Assenat <raph@8d.com>, Trent Piepho <tpiepho@freescale.com>");
MODULE_DESCRIPTION("GPIO LED driver");
MODULE_LICENSE("GPL");

