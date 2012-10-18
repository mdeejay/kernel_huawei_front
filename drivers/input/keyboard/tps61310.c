

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <hsad/config_interface.h>
#include <linux/../../arch/arm/mach-omap2/mux.h>

#define REGISTER0  0x00
#define REGISTER1  0x01
#define REGISTER2  0x02
#define REGISTER3  0x03
#define REGISTER5  0x05
#define STATE_BRIGHT_OFF       0x0
#define STATE_BRIGHT_LOW       0x09
#define STATE_BRIGHT_MEDIUM    0x12
#define STATE_BRIGHT_HIGH      0x1b
#define STATE_LEFT_BRIGHT_HIGH  0x18
#define STATE_RIGHT_BRIGHT_HIGH  0x03

struct tps61310_data {
    struct i2c_client *client;
    struct early_suspend early_suspend;
};
struct i2c_client *tps61310_client;
static int gpio_rst, gpio_strb1;

static char led_status = '0';
static int __devinit tps61310_write(struct i2c_client *client, u8 reg, u8 data)
{
    int error;

    error = i2c_smbus_write_byte_data(client, reg, data);
    if (error) {
        dev_err(&client->dev,
            "couldn't send request. Returned %d\n", error);
        return error;
    }
    return error;
}

static int __devinit tps61310_read(struct i2c_client *client, u8 reg)
{
    int ret;

    ret = i2c_smbus_write_byte(client, reg);
    if (ret) {
        dev_err(&client->dev,
            "couldn't send request. Returned %d\n", ret);
        return ret;
    }

    ret = i2c_smbus_read_byte(client);
    if (ret < 0) {
        dev_err(&client->dev,
            "couldn't read register. Returned %d\n", ret);
        return ret;
    }

    return ret;
}


static ssize_t tps61310_led_get_brightness(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    return 1;
}


static int tps61310_gpio_request(void)
{
    int ret=0;

    //printk("tps61310 to request GPIO!\n");

    gpio_rst = get_gpio_num_by_name("GPIO_FLASH_RST");
    if(gpio_rst < 0)
    {
        dev_err(&tps61310_client->dev, "%s:%d:fail to get GPIO_FLASH_RST.\n", __func__, __LINE__);
        return -EINVAL;
    }

    ret = gpio_request(gpio_rst, "tps61310_rst");
    if (ret < 0)
    {
		ret = gpio_request(gpio_rst, "tps61310_rst");
		if (ret < 0)
		{
			dev_err(&tps61310_client->dev, "%s:%d:fail to request GPIO_FLASH_RST.\n", __func__, __LINE__);
			return -ENODEV;
		}
    }

    gpio_direction_output(gpio_rst, 1);
    mdelay(1);
    gpio_set_value(gpio_rst, 1);

    gpio_strb1 = get_gpio_num_by_name("GPIO136_FLASH_STR1");
    if(gpio_strb1 < 0)
    {
        dev_err(&tps61310_client->dev, "%s:%d:get GPIO136_FLASH_STR1 %d failed\n",__func__,__LINE__, gpio_strb1);
        return -EINVAL;
    }

    ret = gpio_request(gpio_strb1, "tps61310_strb1");
    if (ret < 0)
    {
		ret = gpio_request(gpio_strb1, "tps61310_strb1");
		if (ret < 0)
		{
			dev_err(&tps61310_client->dev, "%s:%d:fail to request GPIO136_FLASH_STR1.\n", __func__, __LINE__);
			return -ENODEV;
		}
    }
    gpio_direction_output(gpio_strb1, 1);//GPIO_HIGH
    mdelay(1);
    gpio_set_value(gpio_strb1, 1);
    return 0;
}

static int tps61310_register_config(void)
{
    int error = 0;
    error = tps61310_write(tps61310_client, REGISTER2 , 0x40);
    if (error) {
        dev_err(&tps61310_client->dev, "failed to set leds mode!\n");
        return error;
    }
    error = tps61310_read(tps61310_client, REGISTER2);
    if (error != 0x40) {
        dev_err(&tps61310_client->dev, "read leds mode fail\n");
        return error;
    }
    error = tps61310_write(tps61310_client, REGISTER5 , 0x6e);
    if (error) {
        dev_err(&tps61310_client->dev, "failed to set leds mode!\n");
        return error;
    }
    return 0;
}
static ssize_t tps61310_led_set_brightness(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    int ret, error;

    if(led_status != buf[0])
    {
        led_status = buf[0];
    }
    else
    {
        return count;
    }
    
    if (buf[0] == '0')//close
    {
        //dev_info(&tps61310_client->dev, "%s:shutdown flash led.\n", __func__);
        ret = tps61310_write(tps61310_client, REGISTER0 , STATE_BRIGHT_OFF);
        if (ret) {
            dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
        }
        gpio_set_value(gpio_rst, 0);
	gpio_set_value(gpio_strb1, 0);
        mdelay(1);
        gpio_free(gpio_rst);
	gpio_free(gpio_strb1);
    }
    else if (buf[0] == '1')//class 1
    {
        //dev_info(&tps61310_client->dev, "%s:set flash led brightness to level 1.\n", __func__);
        tps61310_gpio_request();
        error = tps61310_register_config();
        if (error) {
            dev_err(&tps61310_client->dev, "failed to config register!\n");
            return error;
        }
        ret = tps61310_write(tps61310_client, REGISTER0 , STATE_BRIGHT_LOW);
        if (ret) {
            dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
        }
    }
    else if (buf[0] == '2')//class 2
    {
        //dev_info(&tps61310_client->dev, "%s:set flash led brightness to level 2.\n", __func__);
        tps61310_gpio_request();
        error = tps61310_register_config();
        if (error) {
            dev_err(&tps61310_client->dev, "failed to config register!\n");
            return error;
        }
        ret = tps61310_write(tps61310_client, REGISTER0 , STATE_BRIGHT_MEDIUM);
            if (ret) {
                dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
            }
    }
    else if(buf[0] == '3')
    {
        //dev_info(&tps61310_client->dev, "%s:set flash led brightness to level 3.\n", __func__);
        tps61310_gpio_request();
        error = tps61310_register_config();
        if (error) {
            dev_err(&tps61310_client->dev, "failed to config register!\n");
            return error;
        }
        ret = tps61310_write(tps61310_client, REGISTER0, STATE_BRIGHT_HIGH);
            if (ret) {
                dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
            }
    }
    else if(buf[0] == '4')
    {
         tps61310_gpio_request();
         error = tps61310_register_config();
         if (error) {
             dev_err(&tps61310_client->dev, "failed to config register!\n");
             return error;
         }
         ret = tps61310_write(tps61310_client, REGISTER0, STATE_LEFT_BRIGHT_HIGH);
         if (ret) {
              dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
         }

    }
    else if(buf[0] == '5')
    {
         tps61310_gpio_request();
         error = tps61310_register_config();
         if (error) {
             dev_err(&tps61310_client->dev, "failed to config register!\n");
             return error;
         }
         ret = tps61310_write(tps61310_client, REGISTER0, STATE_RIGHT_BRIGHT_HIGH);
         if (ret) {
              dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
         }
    }
    else
    {
		error = -1;
        printk("Input the wrong number!\n");
        return error;
    }
    return count;
}

static ssize_t tps61310_led_hw_free(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    if ('0' != led_status)
    {
        int ret, error;
        led_status = '0';
        ret = tps61310_write(tps61310_client, REGISTER0 , STATE_BRIGHT_OFF);
        if (ret) {
            dev_err(&tps61310_client->dev, "failed to set leds lightness!\n");
        }
        gpio_set_value(gpio_rst, 0);
        gpio_set_value(gpio_strb1, 0);
        mdelay(1);
        gpio_free(gpio_rst);
        gpio_free(gpio_strb1);
    }

    return 1;
}

 static struct device_attribute tps61310_led=
    __ATTR(tps61310_led_lightness, 0664, tps61310_led_get_brightness,
                        tps61310_led_set_brightness);

 static struct device_attribute tps61310_led_hw =
    __ATTR(tps61310_led_hw, 0664, tps61310_led_hw_free, NULL);

static int __devinit tps61310_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{
    int error;
    dev_info(&client->dev, "Tps61310 probe begin!\n");
    /* Check functionality */
    error = i2c_check_functionality(client->adapter,
            I2C_FUNC_SMBUS_BYTE);
    if (!error) {
        dev_err(&client->dev, "%s adapter not supported\n",
                dev_driver_string(&client->adapter->dev));
        return -ENODEV;
    }
    tps61310_client = client;

    if (device_create_file(&client->dev, &tps61310_led))
    {
        dev_err(&client->dev, "%s:Unable to create interface\n", __func__);
        goto err_free_mem;
    }

    if (device_create_file(&client->dev, &tps61310_led_hw))
    {
        dev_err(&client->dev, "%s:Unable to create interface\n", __func__);
        goto err_free_mem;
    }
    
    dev_info(&client->dev, "Tps61310 probe OK!\n");
    return 0;

err_free_mem:
    return error;
}

static int __devexit tps61310_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id tps61310_idtable[] = {
    { "tps61310", 0, },
    { }
};

MODULE_DEVICE_TABLE(i2c, tps61310_idtable);

static struct i2c_driver tps61310_driver = {
    .driver = {
        .name   = "tps61310",
        .owner  = THIS_MODULE,
    },

    .id_table   = tps61310_idtable,
    .probe      = tps61310_probe,
    .remove     = __devexit_p(tps61310_remove),
};

static int __init tps61310_init(void)
{
    return i2c_add_driver(&tps61310_driver);
}
module_init(tps61310_init);

static void __exit tps61310_cleanup(void)
{
    i2c_del_driver(&tps61310_driver);
}
module_exit(tps61310_cleanup);

MODULE_AUTHOR("zhdd");
MODULE_DESCRIPTION("Driver for tps61310");
MODULE_LICENSE("GPL");
