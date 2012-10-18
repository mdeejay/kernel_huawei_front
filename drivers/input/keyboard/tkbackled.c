
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/pwm.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl6040-codec.h>
#include <hsad/config_interface.h>
#include <linux/earlysuspend.h>

#define LED_PWM1_ON             0xba
#define LED_PWM1_OFF             0xbb
#define LED_PWM_EN              0x92
#define LED_PWM_EN_SET          0x06
#define LED_UP                  1
#define LED_DOWN                0
#define PWM_ON_VALUE            0x01
#define PWM_OFF_VALUE           0x7F
#define PWM_LED_DOWN_VALUE      0xBF

static int board_id;
struct early_suspend early_suspend;
struct platform_device *pdev_tkled;
static int gpio_tkbcakled;

static void tkled_early_suspend(struct early_suspend *h)
{
     if(board_id) 
     {
    //printk(KERN_ERR "Tk backlight supend!\n");
    twl_i2c_write_u8(TWL6030_MODULE_ID1, PWM_LED_DOWN_VALUE, LED_PWM1_ON);
    twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_DOWN, LED_PWM1_OFF);
    twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_SET, LED_PWM_EN);
     }
     else
     {
        //printk(KERN_ERR "gpio off!Tk backlight supend!\n");
        gpio_set_value(gpio_tkbcakled, LED_DOWN);
     }
}

static void tkled_early_resume(struct early_suspend *h)
{
    return;
}

static void TouchKey_bl_led_set(struct led_classdev *led_cdev,
    enum led_brightness value)
{
    if (!value)
    {
        if(board_id) 
        {
            printk(KERN_ERR "pwm led off!\n");
            twl_i2c_write_u8(TWL6030_MODULE_ID1, PWM_LED_DOWN_VALUE, LED_PWM1_ON);
            twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_DOWN, LED_PWM1_OFF);
            twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_SET, LED_PWM_EN);
        }
        else
        {
            gpio_set_value(gpio_tkbcakled, LED_DOWN);
        }
    }
    else if(board_id) 
    {
        dev_info(&pdev_tkled->dev, "led pwm control!<<<<<<<<<<<<<\n");
        twl_i2c_write_u8(TWL6030_MODULE_ID1, PWM_ON_VALUE, LED_PWM1_ON);
        twl_i2c_write_u8(TWL6030_MODULE_ID1, PWM_OFF_VALUE, LED_PWM1_OFF);
        twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_SET, LED_PWM_EN);
    }
    else
    {
        dev_info(&pdev_tkled->dev, "led up! boardid =%x<<<<<<<<<<\n", board_id);
        gpio_set_value(gpio_tkbcakled, LED_UP);
    }
    return;
}

static struct led_classdev Tk_bl_led = {
    .name              = "button-backlight-tk",
    .brightness_set    = TouchKey_bl_led_set,
    .brightness        = LED_OFF,
};
static int __devinit tkbackled_probe(struct platform_device *pdev)
{
    int ret;
    dev_info(&pdev->dev, "Touch key backlight probe begin!\n");

        pdev_tkled = pdev;
        //board_id = get_board_id_int();
        board_id = get_touchkey_light_boardId_value();
        switch(get_board_id_int())
        {
                case BOARD_ID_VIVA:
				case BOARD_ID_VIVAVN1RF19:
				case BOARD_ID_VIVAVN1RF12458: 
				case BOARD_ID_VIVAT2RF12458: 
				case BOARD_ID_VIVAT2RF19:
						dev_info(&pdev->dev, "No necessary to probe this function!\n");
                        return 0; 
				default:   
				        break;
				        
        }
        if(board_id)
        {
            dev_info(&pdev->dev, "Phone ver. front T1.5!\n");
            twl_i2c_write_u8(TWL6030_MODULE_ID1, PWM_LED_DOWN_VALUE, LED_PWM1_ON);
        twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_DOWN, LED_PWM1_OFF);
        twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_SET, LED_PWM_EN);
        }
        else
        {
            gpio_tkbcakled = get_gpio_num_by_name("GPIO_LED_BL");
        ret = gpio_request(gpio_tkbcakled, "tk_bk_led");
            if (ret < 0)
            {
                dev_err(&pdev->dev, "unable to get INT GPIO\n");
                ret = -ENODEV;
                goto err_free_mem;
            }
            gpio_direction_output(gpio_tkbcakled, LED_DOWN);
        }
        ret = led_classdev_register(&pdev->dev, &Tk_bl_led);
        if (ret) {
           printk(KERN_ERR "unable to register led class driver\n");
           gpio_free(gpio_tkbcakled);
        }
        early_suspend.suspend = tkled_early_suspend;
    early_suspend.resume = tkled_early_resume;
    register_early_suspend(&early_suspend);
    dev_info(&pdev->dev, "Touch key backlight probe OK!\n");
    return 0;

err_free_mem:
    return ret;
}

static int __devexit tkbackled_remove(struct platform_device *pdev)
{
    if(board_id) 
     {
    twl_i2c_write_u8(TWL6030_MODULE_ID1, PWM_LED_DOWN_VALUE, LED_PWM1_ON);
    twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_DOWN, LED_PWM1_OFF);
    twl_i2c_write_u8(TWL6030_MODULE_ID1, LED_PWM_EN_SET, LED_PWM_EN);
     }
    unregister_early_suspend(&early_suspend);
    return 0;
}
static struct platform_driver tkbackled_driver = {
    .probe        = tkbackled_probe,
    .remove        = __devexit_p(tkbackled_remove),
    .driver        = {
        .name    = "TK-backlight",
        .owner    = THIS_MODULE,
    },
};

static int __init tkbackled_init(void)
{
    int ret;
    ret = platform_driver_register(&tkbackled_driver);
    return ret;
}

static void __exit tkbackled_exit(void)
{
    platform_driver_unregister(&tkbackled_driver);
}

module_init(tkbackled_init);
module_exit(tkbackled_exit);
MODULE_AUTHOR("zhdd");
MODULE_DESCRIPTION("Driver for tkbackled");
MODULE_LICENSE("GPL");
