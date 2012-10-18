/*
 * Copyright 2006-2010, Cypress Semiconductor Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>

#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/input/cypress-touchkey.h>

#define TOUCH_UPDATE

#define CYPRESS_I2C_RETRY_TIMES 2

#define BACKLIGHT_ON		0x03
#define BACKLIGHT_OFF		0x00

#define LED_OPEN		0x01
#define LED_CLOSE		0x00
#define LED_BREATH		0x10
	
#define DRIVER_NAME "cypress_tk"

//I2C add define
#define TP_LEDCMD_REG       0
#define TP_RESERVED_REG     1
#define TP_BREATH_SPEED_REG 2
#define TP_KEYCODE_REG      3
#define TP_VER_REG          4



struct cypress_touchkey_devdata {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct touchkey_platform_data *pdata;
	struct early_suspend early_suspend;
	u8 backlight_on;
	u8 backlight_off;
	u8 state;
	bool is_dead;
	bool is_powering_on;
	bool has_legacy_keycode;

	struct delayed_work  dwork;
};

static int i2c_cypresstk_read(struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t length)
{
	int retry;
	uint8_t addr[1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	addr[0] = address;
	
	for (retry = 0; retry < CYPRESS_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(10);
	}
	if (retry == CYPRESS_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",
			CYPRESS_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

static int i2c_cypresstk_write(struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = address;	

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < CYPRESS_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == CYPRESS_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_write_block retry over %d\n",
			CYPRESS_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}


static uint8_t i2c_cypresstk_read_byte_data(struct i2c_client *client, uint8_t address)
{
    uint8_t temp_read_byte = 0;
	i2c_cypresstk_read(client, address, &temp_read_byte, 1);
	return temp_read_byte;
}


static int i2c_cypresstk_write_byte_data(struct i2c_client *client, uint8_t address, uint8_t value)
{
	return i2c_cypresstk_write(client, address, &value, 1);	
}


static void all_keys_up(struct cypress_touchkey_devdata *devdata)
{
	int i;

	for (i = 0; i < devdata->pdata->keycode_cnt; i++)
		input_report_key(devdata->input_dev,
						devdata->pdata->keycode[i], 0);

	input_sync(devdata->input_dev);
}

#if 0
static int recovery_routine(struct cypress_touchkey_devdata *devdata)
{
	int ret = -1;
	int retry = 10;
	u8 data;
	int irq_eint;

	if (unlikely(devdata->is_dead)) {
		dev_err(&devdata->client->dev, "%s: Device is already dead, "
				"skipping recovery\n", __func__);
		return -ENODEV;
	}

	irq_eint = devdata->client->irq;

	all_keys_up(devdata);

	disable_irq_nosync(irq_eint);
	while (retry--) {
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
		ret = i2c_touchkey_read_byte(devdata, &data);
		if (!ret) {
			enable_irq(irq_eint);
			goto out;
		}
		dev_err(&devdata->client->dev, "%s: i2c transfer error retry = "
				"%d\n", __func__, retry);
	}
	devdata->is_dead = true;
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	dev_err(&devdata->client->dev, "%s: touchkey died\n", __func__);
out:
	return ret;
}
#endif

//extern unsigned int touch_state_val;
//extern void TSP_forced_release(void);

unsigned int touch_state_val= 0;

static irqreturn_t touchkey_interrupt_thread(int irq, void *touchkey_devdata)
{
	u8 button_data;
	u8 changed;
	int i;	
	int code;

	
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;

	button_data = i2c_cypresstk_read_byte_data(devdata->client, TP_KEYCODE_REG);

	
	//printk("========hubert! button_data=0x%x \n",button_data);

    changed = button_data ^ devdata->state;
	
	if (!changed)
			goto out;
	
	
	for (i = 0; i < 4; i++) 
	{
	  if (changed & (1 << i))
	  {
			code = devdata->pdata->keycode[i];

			//if touchscreen is press or move, then not report key press
			if( (button_data & (1 << i)) && (touch_state_val) )
			{
			  goto out; 
			}
			
			input_report_key(devdata->input_dev,code, button_data & (1 << i));
			//printk("########hubert %s button_data=0x%x code=0x%x state = %d \n",
			//	__func__,button_data,code,button_data & (1 << i)); 
			
			if (i == 1)
			{
			  //TSP_forced_release();
			}
			
	  }
	}

	
	input_sync(devdata->input_dev);
	
	devdata->state = button_data;
	
	
out:
	return IRQ_HANDLED;
}
	


static irqreturn_t touchkey_interrupt_handler(int irq, void *touchkey_devdata)
{
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;
	
    //printk("#########hubert %s \n", __func__); 
	
	if (devdata->is_powering_on) {
		dev_dbg(&devdata->client->dev, "%s: ignoring spurious boot "
					"interrupt\n", __func__);        
		
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_touchkey_early_suspend(struct early_suspend *h)
{
  #if 0
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

	devdata->is_powering_on = true;

	if (unlikely(devdata->is_dead))
		return;

	disable_irq(devdata->client->irq);
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);

	all_keys_up(devdata);
 #endif
}

static void cypress_touchkey_early_resume(struct early_suspend *h)
{
#if 0
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
//#if 0
	if (i2c_touchkey_write_byte(devdata, devdata->backlight_on)) {
		devdata->is_dead = true;
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		dev_err(&devdata->client->dev, "%s: touch keypad not responding"
				" to commands, disabling\n", __func__);
		return;
	}
//#endif
	devdata->is_dead = false;
	enable_irq(devdata->client->irq);
	devdata->is_powering_on = false;
#endif
}
#endif


//#################TOUCH_UPDATE about2 begin
#if defined(TOUCH_UPDATE)
extern int ISSP_main(void);
static int touchkey_update_status = 0;
struct work_struct touch_update_work;
struct workqueue_struct *touchkey_wq;


static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,	
	.name = DRIVER_NAME,
	.fops = NULL,
};



static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;
	struct cypress_touchkey_devdata *devdata = dev->platform_data;
		
	data[0]=i2c_cypresstk_read_byte_data(devdata->client, TP_VER_REG);
	count = sprintf(buf, "0x%x\n", data[0]);

	printk("touch_version_read 0x%x\n", data[0]);
	return count;
}

static ssize_t touch_version_write(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}

static void touchkey_update_func(struct work_struct *p)
{
	int retry = 10;
	touchkey_update_status = 1;
	//printk("%s start\n", __FUNCTION__);	
	while (retry--) {		
		if (ISSP_main() == 0) {
			touchkey_update_status = 0;
			printk("========touchkey_update succeeded========\n");
			enable_irq(gpio_to_irq(CY8C20236A_IRQ_GPIO_ID));
			return;
		}
	}
	touchkey_update_status = -1;
	printk("touchkey_update failed\n");
	return;
}

static ssize_t touch_update_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{	
	if (buf[0] == 'U') {
		printk("\n========touchkey firmware updating========\n");		
	
		disable_irq(gpio_to_irq(CY8C20236A_IRQ_GPIO_ID));
		INIT_WORK(&touch_update_work, touchkey_update_func);
		queue_work(touchkey_wq, &touch_update_work);		
	}
	return size;
}

static ssize_t touch_update_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk("touch_update_read: touchkey_update_status %d\n",
	       touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "Fail\n");
	}

	return count;
}
 


static ssize_t touch_led_ctrl(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct cypress_touchkey_devdata *devdata = dev_get_drvdata(dev);
	int ret = 0;    
	
	if (devdata && !devdata->is_powering_on) {		
		if (strncmp(buf, "open", 4) == 0)
		{		    
			ret = i2c_cypresstk_write_byte_data(devdata->client, TP_LEDCMD_REG, LED_OPEN);
		}
		else if (strncmp(buf, "close", 5) == 0)
		{		    
		    ret = i2c_cypresstk_write_byte_data(devdata->client, TP_LEDCMD_REG, LED_CLOSE);	
		}
		else if (strncmp(buf, "breath", 6) == 0)
		{		    
		    ret = i2c_cypresstk_write_byte_data(devdata->client, TP_LEDCMD_REG, LED_BREATH);	
		}

		if (ret)
			dev_err(dev, "%s: touchkey led i2c failed\n", __func__);
	}
	return size;
}


static ssize_t touchkey_enable_disable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
#if 0
	printk("touchkey_enable_disable %c \n", *buf);
	if (*buf == '0') {
		set_touchkey_debug('d');
		disable_irq(IRQ_TOUCH_INT);
		gpio_direction_output(GPIO_TOUCH_EN, 0);
		touchkey_enable = -2;
	} else if (*buf == '1') {
		if (touchkey_enable == -2) {
			set_touchkey_debug('e');
			gpio_direction_output(GPIO_TOUCH_EN, 1);
			touchkey_enable = 1;
			enable_irq(IRQ_TOUCH_INT);
		}
	} else {
		printk("touchkey_enable_disable: unknown command %c \n", *buf);
	}
#endif
	return size;
}

static ssize_t touchkey_breath_setvalue(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct cypress_touchkey_devdata *devdata = dev_get_drvdata(dev);
	int ret=0;    
	int setval = 0;

	//setval = atoi(buf);
	
    sscanf(buf,"%d",&setval);
	
	//printk("========hubert! val=%d \n",setval);	

	ret = i2c_cypresstk_write_byte_data(devdata->client, TP_BREATH_SPEED_REG, setval&0xff);	

	if (ret)
		dev_err(dev, "%s: touchkey i2c set breath val  failed\n", __func__);
	
	return size;
}


static DEVICE_ATTR(touch_version, 0664, touch_version_read, touch_version_write);
static DEVICE_ATTR(touch_update, 0664, touch_update_read, touch_update_write);
static DEVICE_ATTR(led_ctrl, 0664, NULL, touch_led_ctrl);
static DEVICE_ATTR(enable_disable, 0664, NULL, touchkey_enable_disable);
static DEVICE_ATTR(breath_setval, 0664, NULL, touchkey_breath_setvalue);

#endif
//#################TOUCH_UPDATE about2 end


static int cypress_touchkey_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	struct cypress_touchkey_devdata *devdata;	
	int err;
	int cnt;
#if defined(TOUCH_UPDATE)
	int ret;
#endif

    //printk("#########hubert %s\n", __func__);         
	
	if (!dev->platform_data) {
		dev_err(dev, "%s: Platform data is NULL\n", __func__);
		return -EINVAL;
	}

	devdata = kzalloc(sizeof(*devdata), GFP_KERNEL);
	if (devdata == NULL) {
		dev_err(dev, "%s: failed to create our state\n", __func__);
		return -ENODEV;
	}

	devdata->client = client;
	i2c_set_clientdata(client, devdata);

	devdata->pdata = client->dev.platform_data;
	if (!devdata->pdata->keycode) {
		dev_err(dev, "%s: Invalid platform data\n", __func__);
		err = -EINVAL;
		goto err_null_keycodes;
	}

		
	err = gpio_request(devdata->pdata->irq_gpio_id, "cy8c20236a Interrupt");
	if (err < 0) {
		dev_dbg(&client->dev, "unable to get INT GPIO\n");
		err = -ENODEV;
		goto err_gpio_request;
	}
	gpio_direction_input(devdata->pdata->irq_gpio_id);

      
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_input_alloc_dev;
	}

	devdata->input_dev = input_dev;
	dev_set_drvdata(&input_dev->dev, devdata);
	input_dev->name = DRIVER_NAME;
	input_dev->id.bustype = BUS_HOST;

	for (cnt = 0; cnt < devdata->pdata->keycode_cnt; cnt++)
		input_set_capability(input_dev, EV_KEY,
					devdata->pdata->keycode[cnt]);

	err = input_register_device(input_dev);
	if (err)
		goto err_input_reg_dev;

	devdata->is_powering_on = true;

	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
	
#if 0
	err = i2c_master_recv(client, data, sizeof(data));
	if (err < sizeof(data)) {
		if (err >= 0)
			err = -EIO;
		dev_err(dev, "%s: error reading hardware version\n", __func__);
		goto err_read;
	}

	dev_info(dev, "%s: hardware rev1 = %#02x, rev2 = %#02x\n", __func__,
				data[1], data[2]);
#endif

	devdata->backlight_on = BACKLIGHT_ON;
	devdata->backlight_off = BACKLIGHT_OFF;

	devdata->has_legacy_keycode = 1;
#if 0
	err = i2c_touchkey_write_byte(devdata, devdata->backlight_on);
	if (err) {
		dev_err(dev, "%s: touch keypad backlight on failed\n",
				__func__);
		goto err_backlight_on;
	}
#endif
    client->irq = gpio_to_irq(devdata->pdata->irq_gpio_id);
    
	if (request_threaded_irq(client->irq, touchkey_interrupt_handler,
				touchkey_interrupt_thread, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				DRIVER_NAME, devdata)) {
		dev_err(dev, "%s: Can't allocate irq.\n", __func__);
		goto err_req_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	devdata->early_suspend.suspend = cypress_touchkey_early_suspend;
	devdata->early_suspend.resume = cypress_touchkey_early_resume;
#endif
	register_early_suspend(&devdata->early_suspend);

	devdata->is_powering_on = false;

//#################TOUCH_UPDATE about3 begin	
#if defined(TOUCH_UPDATE)
	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk("%s misc_register fail\n", __FUNCTION__);
		goto err_misc_reg;
	}

	dev_set_drvdata(touchkey_update_device.this_device, devdata);

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_touch_version) < 0) {
		printk("%s device_create_file fail dev_attr_touch_version\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_touch_version.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_touch_update) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_touch_update.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_led_ctrl) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_led_ctrl.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_enable_disable) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_enable_disable.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_breath_setval) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_breath_setval.attr.name);
	}

  
	touchkey_wq = create_singlethread_workqueue(DRIVER_NAME);
	if (!touchkey_wq)
		goto err_create_wq; 	
#endif
//#################TOUCH_UPDATE about3 end

	return 0;

err_create_wq:
#if defined(TOUCH_UPDATE)
	misc_deregister(&touchkey_update_device);
#endif
err_misc_reg:
err_req_irq:
//err_backlight_on:
//err_read:
//	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
//	input_unregister_device(input_dev);
//	goto err_input_alloc_dev;
err_input_reg_dev:
	input_free_device(input_dev);
err_input_alloc_dev:
err_null_keycodes:
err_gpio_request:
	kfree(devdata);
	return err;
}

static int __devexit i2c_touchkey_remove(struct i2c_client *client)
{
	struct cypress_touchkey_devdata *devdata = i2c_get_clientdata(client);

#if defined(TOUCH_UPDATE)
	misc_deregister(&touchkey_update_device);
#endif
	unregister_early_suspend(&devdata->early_suspend);
	/* If the device is dead IRQs are disabled, we need to rebalance them */
	if (unlikely(devdata->is_dead))
		enable_irq(client->irq);
	else
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	free_irq(client->irq, devdata);
	all_keys_up(devdata);
	input_unregister_device(devdata->input_dev);
	kfree(devdata);
	return 0;
}

static const struct i2c_device_id cypress_touchkey_id[] = {
	{ CYPRESS_TOUCHKEY_I2C_NAME, 10 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cypress_touchkey_id);

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.id_table = cypress_touchkey_id,
	.probe = cypress_touchkey_probe,
	.remove = __devexit_p(i2c_touchkey_remove),
};

static int __init touchkey_init(void)
{
	int ret = 0;
    //printk("#########hubert %s\n", __FUNCTION__);
	ret = i2c_add_driver(&touchkey_i2c_driver);
	if (ret)
		pr_err("%s: cypress touch keypad registration failed. (%d)\n",
				__func__, ret);

	return ret;
}

static void __exit touchkey_exit(void)
{
	i2c_del_driver(&touchkey_i2c_driver);
}

//late_initcall(touchkey_init);
module_init(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("cypress touch keypad");
