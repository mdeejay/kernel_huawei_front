/*
 *  atmel_1060_kpd.c - Atmel AT42QT1060 Touch Sense Controller
 *
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>  
#include <hsad/config_interface.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/i2c/bq27510_battery.h>

#define QT1060_VALID_CHIPID  49

#define QT1060_CMD_CHIPID     0
#define QT1060_CMD_VERSION    1
#define QT1060_CMD_SUBVER   2
#define QT1060_CMD_DETCTION_STATUS     4
#define QT1060_CMD_INPUT_PORT_STATUS   5
#define QT1060_CMD_CALIBRATE     12
#define QT1060_CMD_RESET     13
#define QT1060_CMD_DRIFT    14
#define QT1060_CMD_NTHR_KEY0     16
#define QT1060_CMD_NTHR_KEY1     17
#define QT1060_CMD_NTHR_KEY2     18
#define QT1060_CMD_LP_MODE      22
#define QT1060_CMD_IO_MASK      23
#define QT1060_CMD_KEY_MASK      24
#define QT1060_CMD_AKS_MASK      25
#define QT1060_CMD_DETECTION_MASK      27
#define QT1060_CMD_ACTIVE_LEVEL      28
#define QT1060_CMD_USER_OUTPUT_BUFFER      29
#define QT1060_CMD_DETECTION_INTERGRATOR 30
#define QT1060_KEY_LIGHT_DOWN 0x00
#define QT1060_LP_VALUE  0x01  
#define QT1060_CAL_VALUE  0x01
#define SENSITIVE 0x07
#define TRUE  1
#define FALSE 0
//#define LED_LIGHT

extern u8 touch_is_pressed;
extern u32 time_check;
static bool flag_tk_status = false;
static unsigned short qt1060_key2code[] = {
    KEY_HOME,//102
    KEY_BACK,//158
    KEY_MENU,//139
};
struct qt1060_data {
    struct i2c_client *client;
    struct input_dev *input;
    struct workqueue_struct *atmel_wq;
    struct workqueue_struct *timer_work_queue;
    struct work_struct work;
    struct work_struct timer_work;
    //spinlock_t lock;        /* Protects canceling/rescheduling of dwork */
    unsigned short *keycodes;
    int keynum;
    u16 key_matrix;
    struct early_suspend early_suspend;
    struct hrtimer timer;
    struct mutex lock;
};

//for controlling LED light
struct qt1060_data*  qt1060copy = NULL;
extern struct bq27510_device_info *g_battery_measure_by_bq27510_device;
static int old_tmp;
static int qt1060_set_register(struct i2c_client *client);
static int __devinit qt1060_read(struct i2c_client *client, u8 reg)
{
    int ret;
    msleep(5);
    ret = i2c_smbus_write_byte(client, reg);
    if (ret)
    {
        dev_err(&client->dev,
            "couldn't send request. Returned %d\n", ret);
        return ret;
    }

    ret = i2c_smbus_read_byte(client);
    if (ret < 0)
    {
        dev_err(&client->dev,
            "couldn't read register. Returned %d\n", ret);
        return ret;
    }

    return ret;
}
static int __devinit qt1060_write(struct i2c_client *client, u8 reg, u8 data)
{
    int error;
    msleep(5);
    error = i2c_smbus_write_byte_data(client, reg, data);
    if (error)
    {
        dev_err(&client->dev,
            "couldn't send request. Returned %d\n", error);
        return error;
    }
    return error;
}
static void all_keys_up(struct qt1060_data *devdata)
{
    int i;
    for (i = 0; i < devdata->keynum; i++)
        input_report_key(devdata->input,
                                devdata->keycodes[i], 0);
    input_sync(devdata->input);
}

static int atmel_touchkey_suspend(struct i2c_client *client)
{
    int mode;
    struct qt1060_data *devdata = i2c_get_clientdata(client);
	disable_irq(devdata->client->irq);
    all_keys_up(devdata);
    hrtimer_cancel(&devdata->timer);
    cancel_work_sync(&devdata->timer_work);
    mode = cancel_work_sync(&devdata->work);
    if (mode)
        enable_irq(client->irq);
    mutex_lock(&devdata->lock);
    mode = qt1060_write(devdata->client,QT1060_CMD_USER_OUTPUT_BUFFER, QT1060_KEY_LIGHT_DOWN);
    if (mode)
    {
        dev_err(&devdata->client->dev, "failed to active user buffer\n");
        qt1060_write(devdata->client,QT1060_CMD_USER_OUTPUT_BUFFER, QT1060_KEY_LIGHT_DOWN);
    }
	mode = qt1060_read(devdata->client, QT1060_CMD_DETCTION_STATUS);
    mode = qt1060_write(devdata->client, QT1060_CMD_LP_MODE, 0);
    mutex_unlock(&devdata->lock);
    if (mode)
    {
        dev_err(&devdata->client->dev, "atmel Tk failed to mode device\n");
    }
    dev_info(&qt1060copy->client->dev,"qt1060_model has suspend\n");
	return 0;
}
static int atmel_touchkey_resume(struct i2c_client *client)
{
    int ret, temp, i;
    struct qt1060_data *devdata = i2c_get_clientdata(client);
    /*set calibrate device*/
    ret = qt1060_write(devdata->client, QT1060_CMD_CALIBRATE, 1);
    if (ret)
    {
        dev_err(&devdata->client->dev, "failed to calibrate device\n");
    }
    /*try 10 times to active chip*/
    for(i = 0; i < 10; i++)
    {
        ret = qt1060_write(devdata->client, QT1060_CMD_LP_MODE, QT1060_LP_VALUE);
        if (ret)
            dev_err(&devdata->client->dev, "atmel_qt1060 failed to resume device\n");
        temp = qt1060_read(devdata->client, QT1060_CMD_LP_MODE);
        if (temp == QT1060_LP_VALUE) break;
    }
    if (temp != QT1060_LP_VALUE)
    {
        dev_err(&devdata->client->dev, "atmel_qt1060 failed to active the chip!\n");
    }
    enable_irq(devdata->client->irq);
    hrtimer_start(&devdata->timer, ktime_set(10, 0), HRTIMER_MODE_REL);
    dev_info(&qt1060copy->client->dev,"qt1060_model has resume\n");
    return 0;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_touchkey_early_suspend(struct early_suspend *h)
{
    struct qt1060_data *devdata =
              container_of(h, struct qt1060_data, early_suspend);
    atmel_touchkey_suspend(devdata->client);
}
static void atmel_touchkey_later_resume(struct early_suspend *h)
{
    struct qt1060_data *devdata =
              container_of(h, struct qt1060_data, early_suspend);
    atmel_touchkey_resume(devdata->client);
}
#endif
static int qt1060_get_key_matrix(struct qt1060_data *qt1060)
{
    struct i2c_client *client = qt1060->client;
    struct input_dev *input = qt1060->input;
    int i, status, old_matrix, new_matrix, mask;
    u32 t,check;
    dev_dbg(&client->dev, "requesting keys...\n");

    /*
     * Read all registers from General Status Register
     * to GPIOs register
     */
    status = qt1060_read(client, QT1060_CMD_DETCTION_STATUS);
    if(status < 0)
    {
        enable_irq(qt1060->client->irq);
        return 0;
    }
    t = (u32) ktime_to_ms(ktime_get());
    check = t - time_check;
    if(((touch_is_pressed == 0 )&& (check > 120))||flag_tk_status)
    {
	    old_matrix = qt1060->key_matrix;
	    qt1060->key_matrix = new_matrix = status;
	    mask = 0x01;
	    for (i = 0; i <  qt1060->keynum; ++i, mask <<= 1)
	    {
	        int keyval = new_matrix & mask;
	        if ((old_matrix & mask) != keyval)
	        {
	            input_report_key(input, qt1060->keycodes[i], keyval);
	            dev_info(&qt1060->client->dev,"key %d keycode%d %s  status=%x oldstatus=%x\n",
	                i, qt1060->keycodes[i], keyval ? "pressed" : "released", status, old_matrix);  
	            flag_tk_status = (bool)keyval;
	            break;
	        }
	    }
	    input_sync(input);
    }
    enable_irq(qt1060->client->irq);
    return 0;
}

static irqreturn_t qt1060_irq(int irq, void *_qt1060)
{
    struct qt1060_data *qt1060 = _qt1060;
    disable_irq_nosync(qt1060->client->irq);
    queue_work(qt1060->atmel_wq, &qt1060->work);

    return IRQ_HANDLED;
}

static void atmel_tpk_work_func(struct work_struct *work)
{
    struct qt1060_data *qt1060 =
        container_of(work, struct qt1060_data, work);

    //dev_dzbg(&qt1060->client->dev, "worker\n");

    qt1060_get_key_matrix(qt1060);

    /* Avoid device lock up by checking every so often */
    //qt1060_schedule_read(qt1060);
}


static bool __devinit qt1060_identify(struct i2c_client *client)
{
    int id, ver, rev;

    /* Read Chid ID to check if chip is valid */
    id = qt1060_read(client, QT1060_CMD_CHIPID);
    if (id != QT1060_VALID_CHIPID)
    {
        dev_err(&client->dev, "ID %d not supported\n", id);
        return false;
    }
    /* Read chip version */
    ver = qt1060_read(client, QT1060_CMD_VERSION);
    if (ver < 0)
    {
        dev_err(&client->dev, "could not get firmware version\n");
        return false;
    }

    /* Read chip firmware revision */
    rev = qt1060_read(client, QT1060_CMD_SUBVER);
    if (rev < 0)
    {
        dev_err(&client->dev, "could not get firmware revision\n");
        return false;
    }

    dev_info(&client->dev, "AT42QT1060 firmware version %d.%d.%d\n",
            ver >> 4, ver & 0xf, rev);

    return true;
}

static void qt1060_keypad_bl_led_set(struct led_classdev *led_cdev,
    enum led_brightness value)
{
    int ret;
    mutex_lock(&qt1060copy->lock);
    if (!value)
    {
        ret = qt1060_write(qt1060copy->client,QT1060_CMD_USER_OUTPUT_BUFFER, 0x00);
        if (ret)
        {
            dev_err(&qt1060copy->client->dev, "failed to active user buffer\n");
        }
    }
    else
    {
        ret = qt1060_write(qt1060copy->client,QT1060_CMD_USER_OUTPUT_BUFFER, 0x01);
        if (ret)
        {
            dev_err(&qt1060copy->client->dev, "failed to active user buffer\n");
        }
    }
    mutex_unlock(&qt1060copy->lock);
}

static int qt1060_set_sensitivity(struct qt1060_data *qt1060)
{
    int ret = 0;

    /*set HOME key0 sensitivity*/
    ret = qt1060_write(qt1060->client, QT1060_CMD_NTHR_KEY0, SENSITIVE);
    if (ret)
    {
        dev_err(&qt1060->client->dev, "failed to write QT1060_CMD_NTHR_KEY0.\n");
        return ret;
    }
    ret = qt1060_read(qt1060->client, QT1060_CMD_NTHR_KEY0);
    if (ret != SENSITIVE)
    {
        dev_err(&qt1060->client->dev, "failed to read QT1060_CMD_NTHR_KEY0.\n");
        return -1;
    }

    /*set BACK key1 sensitivity*/
    ret = qt1060_write(qt1060->client, QT1060_CMD_NTHR_KEY1, SENSITIVE);
    if (ret)
    {
        dev_err(&qt1060->client->dev, "failed to write QT1060_CMD_NTHR_KEY0.\n");
        return ret;
    }
    ret = qt1060_read(qt1060->client, QT1060_CMD_NTHR_KEY1);
    if (ret != SENSITIVE)
    {
        dev_err(&qt1060->client->dev, "failed to read QT1060_CMD_NTHR_KEY0.\n");
        return -1;
    }

    /*set MENU key2 sensitivity*/
    ret = qt1060_write(qt1060->client, QT1060_CMD_NTHR_KEY2, SENSITIVE);
    if (ret)
    {
        dev_err(&qt1060->client->dev, "failed to write QT1060_CMD_NTHR_KEY0.\n");
        return ret;
    }
    ret = qt1060_read(qt1060->client, QT1060_CMD_NTHR_KEY2);
    if (ret != SENSITIVE)
    {
        dev_err(&qt1060->client->dev, "failed to read QT1060_CMD_NTHR_KEY0.\n");
        return -1;
    }
    return 0;
}

static struct led_classdev qt1060_kp_bl_led = {
    .name                   = "button-backlight-tk",
    .brightness_set     = qt1060_keypad_bl_led_set,
    .brightness     = LED_OFF,
};

static ssize_t qt1060_reg_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct qt1060_data *qt1060 = dev_get_drvdata(dev);
    u8 i,ret;
    u8 val;
    ssize_t count=0;

    disable_irq(qt1060->client->irq);
    for(i=0;i<64;i++)
    {
        val = qt1060_read(qt1060->client,i);
        ret = sprintf(buf, "reg[0x%02x] val[0x%02x] \t", i,val);
        buf += ret;
        count += ret;
        if(i%4 == 0)
            ret = sprintf(buf,"\n");
        buf += ret;
        count += ret;
    }
    enable_irq(qt1060->client->irq);
    return count;
}

static ssize_t qt1060_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct qt1060_data *qt1060 = dev_get_drvdata(dev);
    int regs,value;
    size_t ret;
    if (2 != sscanf(buf, "%x %x", &regs, &value))
    {
        dev_err(&qt1060->client->dev, "failed to reg store\n");
        return -EINVAL;
    }
    disable_irq(qt1060->client->irq);
    ret = qt1060_write(qt1060->client,regs, value);
    enable_irq(qt1060->client->irq);
    return ret;
}

static DEVICE_ATTR(reg, 0664, qt1060_reg_show,
           qt1060_reg_store);

static struct attribute *qt1060_attributes[] = {
    &dev_attr_reg.attr,
    NULL
};

static const struct attribute_group qt1060_attr_group = {
    .attrs = qt1060_attributes,
};
static enum hrtimer_restart qt1060_timer_func(struct hrtimer *timer)
{
    queue_work(qt1060copy->timer_work_queue, &qt1060copy->timer_work);
    return HRTIMER_NORESTART;
}

static void qt1060_timer_work_func(struct work_struct *work)
{
    int cal = 0, new_tmp = 0, delta = 0;
    long sesc, nsesc;
    struct qt1060_data *qt1060 =
        container_of(work, struct qt1060_data, timer_work);
    sesc = 60;
    nsesc = 0;
    new_tmp = bq27510_battery_temperature(g_battery_measure_by_bq27510_device);
    delta = new_tmp >= old_tmp? (new_tmp - old_tmp):( old_tmp - new_tmp);
    if( delta >= 50)
    {
        cal = qt1060_write(qt1060->client, QT1060_CMD_CALIBRATE, 1);
        if (cal)
        {
            dev_err(&qt1060->client->dev, "failed to calibrate device\n");
        }
        dev_info(&qt1060->client->dev,"QT1060 calibrate the chip for tempreture changeed!\n");
    }
    old_tmp = new_tmp;
    hrtimer_start(&qt1060->timer, ktime_set(sesc, nsesc), HRTIMER_MODE_REL);
}

static int qt1060_set_register(struct i2c_client *client)
{
    int ret, value, temp, i;

    /*colse the unsed key*/
    ret = qt1060_write(client,QT1060_CMD_KEY_MASK, 0x07); 
    if (ret)
    {
        dev_err(&client->dev, "failed to close the unsed key device\n");
        goto err_set_register;
    }
    ret = qt1060_write(client, QT1060_CMD_DRIFT, 0);
    if (ret)
    {
        dev_err(&client->dev, "failed to set drift mode!\n");
        goto err_set_register;
    }
    /*set calibrate device*/
    ret = qt1060_write(client, QT1060_CMD_CALIBRATE, 2);
    if (ret)
    {
        dev_err(&client->dev, "failed to calibrate device\n");
        goto err_set_register;
    }
    /*set aks mask device*/
    ret = qt1060_write(client, QT1060_CMD_AKS_MASK , 0x3f);
    if (ret)
    {
        dev_err(&client->dev, "failed to aks mask device\n");
        goto err_set_register;
    }

    value = qt1060_read(client, QT1060_CMD_AKS_MASK);
    if (value != 63)
    {
        dev_err(&client->dev, "read QT1060_CMD_AKS_MASK fail\n");
        ret = -1;
        goto err_set_register;
    }
	ret = qt1060_write(client, QT1060_CMD_DETECTION_INTERGRATOR , 0x02);
	if (ret)
    {
        dev_err(&client->dev, "failed to aks detection intergrator\n");
        goto err_set_register;
    }
    /*clear the detection mask*/
    ret = qt1060_write(client, QT1060_CMD_DETECTION_MASK , 0x00);
    if (ret)
    {
        dev_err(&client->dev, "failed to active level device\n");
        goto err_set_register;
    }
    /*set the active level*/
    ret = qt1060_write(client, QT1060_CMD_ACTIVE_LEVEL , 0x7f);
    if (ret)
    {
        dev_err(&client->dev, "failed to active level device\n");
        goto err_set_register;
    }
	/*set the IO as output leave open*/
    ret = qt1060_write(client, QT1060_CMD_IO_MASK , 0x7f);
    if (ret)
    {
        dev_err(&client->dev, "failed to io mask device\n");
        goto err_set_register;
    }
    value = qt1060_read(client, QT1060_CMD_IO_MASK);
    if (value != 0x7f)
    {
        dev_err(&client->dev, "read QT1060_CMD_IO_MASK fail\n");
        ret = -1;
        goto err_set_register;
    }
    ret = qt1060_set_sensitivity(qt1060copy);
    if(ret)
    {
        dev_err(&client->dev, "failed to set sensitivity.\n");
        goto err_set_register;
    }
    /*try 10 times to active chip*/
    for(i = 0; i < 10; i++)
    {
        ret = qt1060_write(client, QT1060_CMD_LP_MODE, QT1060_LP_VALUE);
        if (ret)
            dev_err(&client->dev, "atmel_qt1060 failed to resume device\n");
        temp = qt1060_read(client, QT1060_CMD_LP_MODE);
        if (temp == QT1060_LP_VALUE) break;
    }
    if (temp != QT1060_LP_VALUE)
    {
        dev_err(&client->dev, "atmel_qt1060 failed to active the chip!\n");
		ret = -1;
        goto err_set_register;
    }
err_set_register:
            return ret;

}
static int __devinit qt1060_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{
    struct qt1060_data *qt1060;
    struct input_dev *input;
    int i;
    int error;
    int gpio_atmel_int;
    dev_info(&client->dev, "Atmel qt1060 probe begin!\n");
    gpio_atmel_int = get_gpio_num_by_name("GPIO_CHANGE");
    if(gpio_atmel_int < 0)
    {
        printk(KERN_WARNING"%s:%d:get GPIO_CHANGE failed\n",__func__,__LINE__);
        return -EINVAL;
    }
    else
    {
        client->irq = OMAP_GPIO_IRQ(gpio_atmel_int);
    }
    /* Check functionality */
    error = i2c_check_functionality(client->adapter,
            I2C_FUNC_SMBUS_BYTE);
    if (!error)
    {
        dev_err(&client->dev, "%s adapter not supported\n",
                dev_driver_string(&client->adapter->dev));
        return -ENODEV;
    }

    if (!qt1060_identify(client))
    {
        printk(KERN_ERR"qt1060_identify error\n");
        return -ENODEV;
    }
    /* Chip is valid and active. Allocate structure */
    qt1060 = kzalloc(sizeof(struct qt1060_data), GFP_KERNEL);
    input = input_allocate_device();
    if (!qt1060 || !input)
    {
        dev_err(&client->dev, "insufficient memory\n");
        error = -ENOMEM;
        goto err_alloc_input_mem;
    }
    qt1060copy = qt1060;
    qt1060->client = client;
    qt1060->input = input;
    dev_set_drvdata(&input->dev, qt1060);
    mutex_init(&qt1060->lock);
    //INIT_DELAYED_WORK(&qt1060->dwork, qt1060_worker);
    qt1060->atmel_wq = create_singlethread_workqueue("atmel_tpk_wq");//for what ? [ atmel_ts_irq_handler ]'s work queue
    if (!qt1060->atmel_wq)
    {
        printk(KERN_ERR"%s: create workqueue failed\n", __func__);
        error = -ENOMEM;
        goto err_cread_wq_failed1;
    }
    qt1060->timer_work_queue =
        create_singlethread_workqueue("qt1060_timer_wq");
    if (!qt1060->timer_work_queue)
    {
        printk(KERN_ERR"%s: create timer workqueue failed\n", __func__);
        error = -ENOMEM;
        goto err_cread_wq_failed2;
    }
    INIT_WORK(&qt1060->work, atmel_tpk_work_func);//创建中断服务的下半部,一些具体的处理
    INIT_WORK(&qt1060->timer_work, qt1060_timer_work_func);
    //spin_lock_init(&qt1060->lock);

    input->name = "qt1060";
    input->id.bustype = BUS_I2C;

    input->keycode = qt1060->keycodes;
    input->keycodesize = sizeof(qt1060->keycodes[0]);
    input->keycodemax = qt1060->keynum;

    __set_bit(EV_KEY, input->evbit);
    __clear_bit(EV_REP, input->evbit);
    qt1060->keycodes = qt1060_key2code;
    qt1060->keynum = ARRAY_SIZE(qt1060_key2code);
	for (i = 0; i < qt1060->keynum; i++)
	{
		input_set_capability(qt1060->input, EV_KEY,
		qt1060->keycodes[i]);
	}
    __clear_bit(KEY_RESERVED, input->keybit);

    /*set the chip register*/
	error = qt1060_set_register(client);
    if (error)
    {
        dev_err(&client->dev,
            "Failed to set register value!\n");
    }
	error = input_register_device(qt1060->input);
    if (error)
    {
        dev_err(&client->dev,
            "Failed to register input device\n");
        goto err_input_register_failed;
    }

    if (client->irq)
    {
        error = request_irq(client->irq, qt1060_irq,
                   IRQF_TRIGGER_LOW, "qt1060", qt1060);// IRQF_TRIGGER_LOW // | IRQF_TRIGGER_RISING
        if (error)
        {
            dev_err(&client->dev,
                "failed to allocate irq %d\n", client->irq);
            goto err_request_irq;
        }
    }


    i2c_set_clientdata(client, qt1060);

#ifdef CONFIG_HAS_EARLYSUSPEND
    qt1060->early_suspend.suspend = atmel_touchkey_early_suspend;
    qt1060->early_suspend.resume = atmel_touchkey_later_resume;
    register_early_suspend(&qt1060->early_suspend);
#endif
    error = led_classdev_register(&client->dev, &qt1060_kp_bl_led);
    if (error)
    {
        dev_err(&client->dev, "unable to register led class driver\n");
        goto err_register_led_class;
    }
    error = sysfs_create_group(&client->dev.kobj, &qt1060_attr_group);
    if(error)
    {
        dev_err(&client->dev, "unable to creat attribute\n");
    }
    hrtimer_init(&qt1060->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    qt1060->timer.function = qt1060_timer_func;
	hrtimer_start(&qt1060->timer, ktime_set(100, 0), HRTIMER_MODE_REL);
    dev_info(&client->dev,"QT1060 first time start timer!\n");
    dev_info(&client->dev, "Atmel qt1060 probe end!\n"); 
    return 0;


err_register_led_class:
    if (client->irq)
        free_irq(client->irq, qt1060);
err_request_irq:
    input_unregister_device(qt1060->input);
err_input_register_failed:
    destroy_workqueue(qt1060->timer_work_queue);
err_cread_wq_failed2:
    destroy_workqueue(qt1060->atmel_wq);
err_cread_wq_failed1:
    input_free_device(input);
err_alloc_input_mem:
    kfree(qt1060);
    qt1060copy = NULL;
    return error;
}
static int qt1060_shutdown(struct i2c_client *client)
{
    struct qt1060_data *qt1060 = i2c_get_clientdata(client);
    qt1060_write(qt1060->client,QT1060_CMD_USER_OUTPUT_BUFFER, QT1060_KEY_LIGHT_DOWN);
    return 0;
}
static int __devexit qt1060_remove(struct i2c_client *client)
{
    struct qt1060_data *qt1060;
    qt1060 = i2c_get_clientdata(client);
    dev_info(&client->dev, "Atmel qt1060 remove!\n");
    unregister_early_suspend(&qt1060->early_suspend);
    if(client->irq)
        free_irq(client->irq, qt1060);
    /* Release IRQ so no queue will be scheduled */
    destroy_workqueue(qt1060->atmel_wq);
    destroy_workqueue(qt1060->timer_work_queue);
    all_keys_up(qt1060);
    qt1060_write(qt1060->client,QT1060_CMD_USER_OUTPUT_BUFFER, QT1060_KEY_LIGHT_DOWN);
    input_unregister_device(qt1060->input);
    kfree(qt1060);
    qt1060copy = NULL;
    led_classdev_unregister(&qt1060_kp_bl_led);
    sysfs_remove_group(&client->dev.kobj, &qt1060_attr_group);
    return 0;
}

static const struct i2c_device_id qt1060_idtable[] = {
    { "qt1060", 0, },
    { }
};

MODULE_DEVICE_TABLE(i2c, qt1060_idtable);

static struct i2c_driver qt1060_driver = {
    .driver = {
        .name   = "qt1060",
        .owner  = THIS_MODULE,
    },

    .id_table   = qt1060_idtable,
    .probe      = qt1060_probe,
    .shutdown = qt1060_shutdown,
    .remove     = __devexit_p(qt1060_remove),
};

static int __init qt1060_init(void)
{
    return i2c_add_driver(&qt1060_driver);
}
module_init(qt1060_init);

static void __exit qt1060_cleanup(void)
{
    i2c_del_driver(&qt1060_driver);
}
module_exit(qt1060_cleanup);

MODULE_AUTHOR("Raphael Derosso Pereira <raphaelpereira@gmail.com>");
MODULE_DESCRIPTION("Driver for AT42QT1060 Touch Sensor");
MODULE_LICENSE("GPL");
