#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <hsad/config_interface.h>
#include <linux/string.h>
#include "../../compass/akm8975.h"
#include "../../accelerometer/lis3dh.h"
#include "../../accelerometer/adxl34x.h"
#include "../../accelerometer/gs_mma8452.h"
#include "../../gyroscope/l3g4200d.h"
#include "../../light/tmd2771.h"
#include "../../light/apds990x.h"
#include "../../light/ltr_558.h"
#include "../../touchscreen/touch_platform_config.h"

static struct regulator *regulator_st_acc_sensor = NULL;
static struct regulator *regulator_adi_acc_sensor = NULL;
static struct regulator *regulator_fsc_acc_sensor = NULL;
static struct regulator *regulator_akm_sensor = NULL;
static struct regulator *regulator_gyr_sensor = NULL;
static struct regulator *regulator_light_sensor = NULL;

/*Input Device Name*/
#define    ACCL_INPUT_DEV_NAME    "input_accl"
#define    COMPASS_INPUT_DEV_NAME    "input_compass"
/*Input Device Name*/
#define COMPASS_INT_GPIO  25
#define PROXIMITY_INT_GPIO  28
#define GSENSOR_INT_GPIO  23
#define TOUCH_OMAP4430_INT_PIN 35
#define TOUCH_OMAP4430_RESET_PIN  36

#define CONFIG_TOUCHSCREEN_VIRTUALKEY

#define MAX_TP_SIZE_LEN 4
static char config_tp_size[MAX_TP_SIZE_LEN];

static bool panel_get_touchscreen_config_size(void)
{
    if(get_hw_config("touchscreen/size",config_tp_size,MAX_TP_SIZE_LEN,NULL))
    {
        printk(KERN_INFO"(tp size =%s).\n",config_tp_size);
        return true;
    }
    else
    {
        printk(KERN_ERR"Get config name faild.\n");
        return false;
    }
}

#ifdef CONFIG_TOUCHSCREEN_VIRTUALKEY
static char buf_virtualkey[500];
static ssize_t  buf_vkey_size=0;
atomic_t touch_detected_yet = ATOMIC_INIT(0);

static ssize_t virtual_keys_show(struct kobject *kobj,
                   struct kobj_attribute *attr, char *buf)
{
        memcpy( buf, buf_virtualkey, buf_vkey_size );
        return buf_vkey_size;
}

static struct kobj_attribute virtual_keys_attr[] ={
    {
    .attr = {
        .name = "virtualkeys.synaptics",
        .mode = S_IRUGO,
        },
        .show = &virtual_keys_show,
    },
    {
        .attr = {
            .name = "virtualkeys.atmel-touchscreen",
            .mode = S_IRUGO,
        },
        .show = &virtual_keys_show,
    }
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr[0].attr,
    &virtual_keys_attr[1].attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void __init virtualkeys_init(void)
{
    extern struct kobject *prop_kobj_virtual;
    int ret;
    #if 0
    if (!panel_get_touchscreen_config_size())
    {
         pr_err("virtualkeys_init error!\n");
    }
    if (!strncmp(config_tp_size, "WVGA",MAX_TP_SIZE_LEN))
    {
            buf_vkey_size = sprintf(buf_virtualkey,
                __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":230:880:100:70"
                 ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:880:100:70"
                 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":350:880:100:70"
                   "\n");
     }
    #endif
    if ((!strncmp(config_tp_size, "720P",MAX_TP_SIZE_LEN)) || (!strncmp(config_tp_size, "720A",MAX_TP_SIZE_LEN)))
    {
        buf_vkey_size = sprintf(buf_virtualkey,
                __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":365:1380:160:150"
                 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":120:1380:160:150"
                 ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":605:1380:160:150"
                 "\n");
    }
    //properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (prop_kobj_virtual)
        ret = sysfs_create_group(prop_kobj_virtual,
                     &properties_attr_group);
    if (!prop_kobj_virtual || ret)
        printk(KERN_ERR "failed to create board_properties\n");
}
#endif
#if 1
int gpio_config_interrupt(int gpio,char * name)
{
    int err = 0;

    err = gpio_request(gpio, name);
    if (err) {
        printk(KERN_ERR"%s: gpio_request failed for intr %d\n", name,gpio);
        return err;
    }
    err = gpio_direction_input(gpio);
    if (err) {
        printk(KERN_ERR"%s: gpio_direction_input failed for intr %d\n",name, gpio);
        return err;
    }
    return 0;
}
/*
 *use the touch_gpio_config_interrupt to config the gpio
 *which we used, but the gpio number can't exposure to user
 *so when the platform or the product changged please self self adapt
 */
int touch_gpio_config_interrupt(void)
{
    return gpio_config_interrupt(TOUCH_OMAP4430_INT_PIN,"touch_irq");
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
#if 1
int touch_reset(void)
{
    int ret = 0;

    gpio_request(TOUCH_OMAP4430_RESET_PIN,"TOUCH_RESET");
    ret = gpio_direction_output(TOUCH_OMAP4430_RESET_PIN, 1);
    mdelay(5);
    ret = gpio_direction_output(TOUCH_OMAP4430_RESET_PIN, 0);
    mdelay(10);
    ret = gpio_direction_output(TOUCH_OMAP4430_RESET_PIN, 1);
    mdelay(50);//must more than 10ms.
    gpio_free(TOUCH_OMAP4430_RESET_PIN);

    return ret;
}
int get_touch_reset_pin(void)
{
    int ret = TOUCH_OMAP4430_RESET_PIN;
    return ret;
}
#else
int touch_reset(void)
{
    int ret = 0;
    int touch_reset_gpio = get_gpio_num_by_name("GPIO_TOUCH_RST_N");
    if(touch_reset_gpio < 0)
    {
         pr_err("%s: get GPIO_TOUCH_RST_N number failed\n",__func__);
         return -EINVAL;
    }
    gpio_request(touch_reset_gpio,"TOUCH_RESET");
    ret = gpio_direction_output(touch_reset_gpio, 1);
    mdelay(5);
    ret = gpio_direction_output(touch_reset_gpio, 0);
    mdelay(10);
    ret = gpio_direction_output(touch_reset_gpio, 1);
    mdelay(50);//must more than 10ms.
    gpio_free(touch_reset_gpio);

    return ret;
}

int get_touch_reset_pin()
{
    int touch_reset_gpio = get_gpio_num_by_name("GPIO_TOUCH_RST_N");
    if (touch_reset_gpio < 0)
    {
        pr_err("%s: get GPIO_TOUCH_RST_N number failed\n",__func__);
        return -EINVAL;
    }
    return touch_reset_gpio;
}
#endif




/*this function get the tp  resolution*/
static int get_phone_version(struct tp_resolution_conversion *tp_resolution_type)
{
    int ret = 0;
    tp_resolution_type->lcd_x = LCD_X_WVGA;
    tp_resolution_type->lcd_y = LCD_Y_WVGA;
    tp_resolution_type->jisuan = LCD_JS_WVGA;
    ret = touch_reset();
    if(ret)
    {
        printk(KERN_ERR "%s: reset failed \n", __func__);
        return -1;
    }
    return 0;
}


#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS
static struct regulator *synaptics_tp_power = NULL;

 int enable_power_for_synaptics_TP(struct device * dev)
{
    if(!synaptics_tp_power)
        synaptics_tp_power = get_consumer_regulator(dev, "TOUCH_SCREEN_TP_AVDD");
    if (IS_ERR(synaptics_tp_power))
    {
        printk(KERN_ERR "get_consumer_regulator error, when enable the power for synaptics TP\n");
        synaptics_tp_power = NULL;
        return -1;
    }
    consumer_regulator_enable(synaptics_tp_power);
    printk(KERN_INFO "enable the power for synaptics touchscreen!\n ");
    return 0;
}

void disable_power_of_synaptics_TP(void)
{
    if(synaptics_tp_power)
    {
        consumer_regulator_disable(synaptics_tp_power);
        put_consumer_regulator(synaptics_tp_power);
        synaptics_tp_power = NULL;
    }
    return;
}

static struct synaptics_i2c_platform_data synaptics_tp_platform_data = {
    .power_on = NULL,
    .power_off = NULL,
    .touch_gpio_config_interrupt = touch_gpio_config_interrupt,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .touch_reset = touch_reset,
    .get_touch_reset_pin = get_touch_reset_pin,
    .get_phone_version = get_phone_version,
};

#endif

#ifdef CONFIG_TOUCHSCREEN_MXT224E_ATMEL

static struct regulator *atmel_tp_power = NULL;

int enable_power_for_atmel_TP(struct device * dev)
{
    if(!atmel_tp_power)
        atmel_tp_power = get_consumer_regulator(dev, "atmel_ts_power");
    if (IS_ERR(atmel_tp_power))
    {
        printk(KERN_ERR "get_consumer_regulator error, when enable the power for atmel TP\n");
        atmel_tp_power = NULL;
        return -1;
    }
    consumer_regulator_enable(atmel_tp_power);
    return 0;
}

int disable_power_of_atmel_TP(void)
{
    if(atmel_tp_power)
    {
        consumer_regulator_disable(atmel_tp_power);
        put_consumer_regulator(atmel_tp_power);
        atmel_tp_power = NULL;
    }
    return 0;
}

/*parameters for usb plug-in*/
static struct atmel_i2c_platform_data atmel_tp_WVGA_data = {
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
    .gpio_irq = TOUCH_OMAP4430_INT_PIN,
    .power_on = enable_power_for_atmel_TP,
    .power_off = disable_power_of_atmel_TP,
    .reset = touch_reset,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .config_T6 = {0, 0, 0, 0, 0,
    0},
    .config_T7 = {32, 12, 25},  
    .config_T8 = {21, 0, 5, 5, 0,
    0, 1, 55, 10, 192},
    .config_T9 = {139, 0, 0, 19, 11,
    0, 32, 55, 1, 1,  
    0, 5, 2, 46, 10,      
    12, 20, 10, 191, 3,
    27, 2, 0, 0, 0,
    0, 151, 48, 151, 90,
    25, 15, 0, 0, 0},
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
    .config_T42 = {3, 40, 40, 80, 128,
    0, 0, 0},
    .config_T46 = {0, 3, 48, 63, 0,
    0, 0, 0, 0},
    .config_T47 = {0, 20, 50, 5, 2,
    40, 40, 180, 0, 100},
    .config_T48 = {1, 4, 66, 0, 0,
    0, 0, 0, 5, 9,
    0, 0, 0, 6, 6,
    0, 0, 63, 6, 64,
    10, 0, 20, 5, 0,
    38, 0, 20, 0, 0,
    0, 0, 0, 0, 0,
    50, 2, 3, 1, 0,
    5, 10, 40, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0},
    .object_crc = {0x41, 0x0e, 0x90},
    .cable_config = {70, 30, 32, 32},
    .cable_config_T7 = {48, 255, 25},
    .cable_config_T8 = {21, 0, 5, 5, 0,
    0, 1, 55, 10, 192},
    .cable_config_T9 = {139, 0, 0, 19, 11,
    0, 32, 60, 2, 7,
    0, 5, 2, 1, 10,      
    10, 10, 10, 191, 3,
    27, 2, 13, 13, 12,
    12, 128, 0, 128, 0,
    0, 10, 0, 0, 0},
	.cable_config_T46 = {4, 3, 40, 40, 0,
    0, 0, 0, 0},

	.cable_config_T48={1, 128, 114, 0, 0,
    0, 0, 0, 5, 9,
    0, 0, 0, 10, 10,
    0, 0, 63, 6, 64,
    0, 0, 0, 0, 0,
    0, 0, 20, 0, 0,
    0, 0, 0, 0, 0,
    42, 2, 5, 2, 15,
    10, 12, 20, 0, 0, 
    0,0, 151, 48, 151,
    90, 25, 15, 0},
    
    .noise_config = {70, 3, 35},
    .filter_level = {0, 0, 539, 539},
    .GCAF_level = {8, 16, 24, 32, 40},
    .ATCH_NOR = {0, 0, 1, 55, 10, 192},
    .ATCH_NOR_20S = {0, 0, 255, 1, 0, 0},
};

static struct atmel_i2c_platform_data atmel_tp_720P_data = {
    .version = 0x10,
    .source = 0,
    .abs_x_min = 0x00,
    .abs_x_max = 719,
    .abs_y_min = 0x00,
    .abs_y_max = 1279,
    .abs_pressure_min = 0x00,
    .abs_pressure_max = 255,
    .abs_width_min = 0,
    .abs_width_max = 255,
    .gpio_irq = TOUCH_OMAP4430_INT_PIN,
    .power_on = NULL,
    .power_off = NULL,
    .reset = touch_reset,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .config_T6 = {0, 0, 0, 0, 0,
    0},
    .config_T7 = {32, 255, 25},
    .config_T8 = {19, 0, 5, 5, 0,
    0, 5, 60, 10, 192},
    .config_T9 = {139, 0, 0, 19, 11,
    0, 32, 70, 2, 3,
    0, 5, 2, 32, 10,
    15, 22, 10, 106, 5,/*XRANGE = 1386*/
    207, 2, 0, 0, 2,/* YRANGE = 719*/
    2, 161, 40, 183, 64,
    30, 20, 0, 0, 0},
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
    .config_T42 = {0, 40, 60, 80, 128,
    0, 0, 0},
    .config_T46 = {0, 3, 32, 32, 0,
    0, 0, 0, 0},
    .config_T47 = {0, 20, 50, 5, 2,
    40, 40, 180, 0, 100},
    .config_T48 = {1, 0, 10, 0, 0,
    0, 0, 0, 5, 6,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0},
    .object_crc = {0x41, 0x0e, 0x90},
    .cable_config = {70, 30, 32, 32},
    .cable_config_T7 = {32, 16, 25},
    .cable_config_T8 = {19, 0, 5, 5, 0,
    0, 5, 60, 10, 192},
    .cable_config_T9 = {139, 0, 0, 19, 11,
    0, 32, 65, 3, 3,
    0, 5, 2, 0, 10,
    10, 10, 10, 122, 5,/*XRANGE = 1402*/
    216, 2, 0, 6, 0,/* YRANGE = 728*/
    0, 153, 50, 151, 90,
    15, 10, 0, 0, 0},

    .cable_config_T46 = {0, 3, 48, 48, 0,
    0, 0, 0, 0},

    .cable_config_T48={1, 128, 114, 0, 0,
    0, 0, 0, 5, 6,
    0, 0, 0, 10, 10,
    0, 0, 63, 6, 64,
    0, 0, 0, 0, 0,
    0, 0, 20, 0, 0,
    0, 0, 0, 0, 0,
    42, 2, 5, 2, 32,
    10, 12, 20, 241, 251,
    0,0, 191, 40, 183, 
    64, 30, 15, 0},  
    .config_T15 = {0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0},
    .noise_config = {70, 3, 35},
    .filter_level = {0, 0, 539, 539},
    .GCAF_level = {8, 16, 24, 32, 40},
   .ATCH_NOR = {0, 0, 5, 60, 10, 192},
   .ATCH_NOR_20S = {0, 0, 255, 1, 0, 0},
};


static struct atmel_i2c_platform_data atmel_tp_720A_data = {
    .version = 0x10,
    .source = 0,
    .abs_x_min = 0x00,
    .abs_x_max = 719,
    .abs_y_min = 0x00,
    .abs_y_max = 1279,
    .abs_pressure_min = 0x00,
    .abs_pressure_max = 255,
    .abs_width_min = 0,
    .abs_width_max = 255,
    .gpio_irq = TOUCH_OMAP4430_INT_PIN,
    .power_on = NULL,
    .power_off = NULL,
    .reset = touch_reset,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .config_T6 = {0, 0, 0, 0, 0,
    0},
    .config_T7 = {32, 12, 25},
    .config_T8 = {23, 0, 1, 10, 0,
    0, 5, 60, 10, 192},
    .config_T9 = {139, 0, 0, 19, 11,
    0, 32, 66, 2, 3,
    0, 5, 2, 47, 10,
    15, 22, 10, 106, 5,/*XRANGE = 1386*/
    207, 2, 0, 0, 2,/* YRANGE = 719*/
    2, 161, 40, 183, 64,
    30, 20, 0, 0, 1},
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
    .config_T42 = {3, 40, 60, 80, 128,
    0, 0, 0},

    .config_T47 = {0, 20, 50, 5, 2,
    40, 40, 180, 0, 100},
    
    .config_T46 = {0, 3, 32, 32, 0,
    0, 0, 0, 0},

    .config_T48 = {1, 4, 10, 1, 0,
    0, 0, 0, 1, 2,
    0, 0, 0, 6, 6,
    0, 0, 63, 6, 64,
    10, 0, 20, 5, 0,
    38, 0, 20, 0, 0,
    0, 0, 0, 0, 0,
    40, 2, 5, 2, 32,
    10, 12, 20, 241, 251,
    0, 0, 191, 40, 183,
    64, 30, 15, 0},
    .object_crc = {0x41, 0x0e, 0x90},
    .cable_config = {70, 30, 32, 32},
    .cable_config_T7 = {32, 16, 25},
    .cable_config_T8 = {23, 0, 5, 5, 0,
    0, 5, 60, 10, 192},
    .cable_config_T9 = {139, 0, 0, 19, 11,
    0, 32, 65, 3, 3,
    0, 5, 2, 0, 10,
    10, 10, 10, 122, 5,/*XRANGE = 1402*/
    216, 2, 0, 6, 0,/* YRANGE = 728*/
    0, 153, 50, 151, 90,
    15, 10, 0, 0, 0},

    .cable_config_T46 = {0, 3, 40, 40, 0,
    0, 0, 0, 0},
    .cable_config_T48={1, 128, 114, 1, 0,
    0, 0, 0, 3, 8,
    0, 0, 0, 6, 6,
    0, 0, 63, 6, 64,
    10, 0, 20, 5, 0,
    38, 0, 20, 0, 0,
    0, 0, 0, 0, 0,
    40, 2, 5, 2, 32,
    10, 12, 20, 241, 251,
    0, 0, 191, 40, 183,
    64, 30, 15, 0},
    .noise_config = {70, 3, 35},
    .filter_level = {0, 0, 539, 539},
    .GCAF_level = {8, 16, 24, 32, 40},
   .ATCH_NOR = {0, 0, 5, 60, 10, 192},
   .ATCH_NOR_20S = {0, 0, 255, 1, 0, 0},
};

static void atmel_tp_platform_data_detect(struct atmel_i2c_platform_data **ppData )
{
    //printk("___+___ %s ENTER\n",__func__);
    if (!panel_get_touchscreen_config_size())
    {
         pr_err("virtualkeys_init error!\n");
    }
    printk("%s : config_tp_size = %s\n",__func__,config_tp_size);
    if (!strncmp(config_tp_size, "WVGA",MAX_TP_SIZE_LEN))
    {
        *ppData = &atmel_tp_WVGA_data;
    }
    else if (!strncmp(config_tp_size, "720P",MAX_TP_SIZE_LEN))
    {
    	*ppData = &atmel_tp_720P_data;
    }
    else if (!strncmp(config_tp_size, "720A",MAX_TP_SIZE_LEN))
    {
    	*ppData = &atmel_tp_720A_data;
    }
    else
    {
        printk("ERROR !!!!  %s\n",__func__);
    }

    return;
}
#endif
#endif

int init_st_acc_regulator(struct device* dev)
{
    if(!regulator_st_acc_sensor)    {        /*Get PMU_VPP regulator,don't set voltage manually*/
        regulator_st_acc_sensor = get_consumer_regulator(dev, ST_ACCL_POWER_NAME);
        if (IS_ERR(regulator_st_acc_sensor))
            {
                printk(KERN_ERR"get PW for dev error");
                regulator_st_acc_sensor = NULL;
                return -1;
            }
        }
    consumer_regulator_enable(regulator_st_acc_sensor);
    return 0;
}

void power_off_st_acc(void)
{
    if(regulator_st_acc_sensor)
    {
        consumer_regulator_disable(regulator_st_acc_sensor);
        put_consumer_regulator(regulator_st_acc_sensor);
        regulator_st_acc_sensor = NULL;
    }
    return;
}
int init_adi_acc_regulator(struct device* dev)
{
    if(!regulator_adi_acc_sensor)    {        /*Get PMU_VPP regulator,don't set voltage manually*/
        regulator_adi_acc_sensor = get_consumer_regulator(dev, ADI_ACCL_POWER_NAME);
        if (IS_ERR(regulator_adi_acc_sensor))
            {
                printk(KERN_ERR"get PW for dev error");
                regulator_adi_acc_sensor = NULL;
                return -1;
            }
        }
    consumer_regulator_enable(regulator_adi_acc_sensor);
    return 0;
}

void power_off_adi_acc(void)
{
    if(regulator_adi_acc_sensor)
    {
        consumer_regulator_disable(regulator_adi_acc_sensor);
        put_consumer_regulator(regulator_adi_acc_sensor);
        regulator_adi_acc_sensor = NULL;
    }
    return;
}
int init_fsc_acc_regulator(struct device* dev)
{
    if(!regulator_fsc_acc_sensor)    {        /*Get PMU_VPP regulator,don't set voltage manually*/
        regulator_fsc_acc_sensor = get_consumer_regulator(dev, FREESCALE_ACCL_POWER_NAME);
        if (IS_ERR(regulator_fsc_acc_sensor))
            {
                printk(KERN_ERR"get PW for dev error");
                regulator_fsc_acc_sensor = NULL;
                return -1;
            }
        }
    consumer_regulator_enable(regulator_fsc_acc_sensor);
    return 0;
}

void power_off_fsc_acc(void)
{
    if(regulator_fsc_acc_sensor)
    {
        consumer_regulator_disable(regulator_fsc_acc_sensor);
        put_consumer_regulator(regulator_fsc_acc_sensor);
        regulator_fsc_acc_sensor = NULL;
    }
    return;
}

int init_akm_regulator(struct device* dev)
{
    if(!regulator_akm_sensor)    {        /*Get PMU_VPP regulator,don't set voltage manually*/
        regulator_akm_sensor = get_consumer_regulator(dev, COMPASS_POWER_NAME);
        if (IS_ERR(regulator_akm_sensor))
            {
                printk(KERN_ERR"get PW for dev error");
                regulator_akm_sensor = NULL;
                return -1;
            }
        }
    consumer_regulator_enable(regulator_akm_sensor);
    return 0;
}

void power_off_akm(void)
{
    if(regulator_akm_sensor)
    {
        consumer_regulator_disable(regulator_akm_sensor);
        put_consumer_regulator(regulator_akm_sensor);
        regulator_akm_sensor = NULL;
    }
    return;
}

int init_gyr_regulator(struct device* dev)
{
    if(!regulator_gyr_sensor)    {        /*Get PMU_VPP regulator,don't set voltage manually*/
        regulator_gyr_sensor = get_consumer_regulator(dev, GYRO_POWER_NAME);
        if (IS_ERR(regulator_gyr_sensor))
            {
                printk(KERN_ERR"get PW for dev error");
                regulator_gyr_sensor = NULL;
                return -1;
            }
        }
    consumer_regulator_enable(regulator_gyr_sensor);
    return 0;
}

void power_off_gyr(void)
{
    if(regulator_gyr_sensor)
    {
        consumer_regulator_disable(regulator_gyr_sensor);
        put_consumer_regulator(regulator_gyr_sensor);
        regulator_gyr_sensor = NULL;
    }
    return;
}

int init_light_regulator(struct device* dev)
{
    if(!regulator_light_sensor)    {        /*Get PMU_VPP regulator,don't set voltage manually*/
        regulator_light_sensor = get_consumer_regulator(dev, PL990x_POWER_NAME);
        if (IS_ERR(regulator_light_sensor))
            {
                printk(KERN_ERR"get PW for dev error");
                regulator_light_sensor = NULL;
                return -1;
            }
        }
    consumer_regulator_enable(regulator_light_sensor);
    return 0;
}

void power_off_light(void)
{
    if(regulator_light_sensor)
    {
        consumer_regulator_disable(regulator_light_sensor);
        put_consumer_regulator(regulator_light_sensor);
        regulator_light_sensor = NULL;
    }
    return;
}
//add
#if 0
int gpio_config_interrupt(int gpio,char * name)
{
       int err = 0;

    err = gpio_request(gpio, name);
    if (err) {
        printk(KERN_ERR"%s: gpio_request failed for intr %d\n", name,gpio);
        return err;
    }
    err = gpio_direction_input(gpio);
    if (err) {
        printk(KERN_ERR"%s: gpio_direction_input failed for intr %d\n",name, gpio);
        return err;
    }
    return 0;
}
#endif
int gsensor_interrupt_config(void)
{
        return gpio_config_interrupt(GSENSOR_INT_GPIO,"gsensor_irq");
}
int compass_interrupt_config(void)
{
        return gpio_config_interrupt(COMPASS_INT_GPIO,"compass_irq");
}
int proximity_interrupt_config(void)
{
        return gpio_config_interrupt(PROXIMITY_INT_GPIO,"proximity_irq");
}
static struct lis3dh_acc_platform_data gs_platform_data = {

    .poll_interval = 10,
    .min_interval = 10,

    .g_range = 0x00,

    .axis_map_x = 0,
    .axis_map_y = 1,
    .axis_map_z = 2,

    .negate_x = 1,
    .negate_y = 1,
    .negate_z = 0,

    .gpio_int1 = -1,
    .gpio_int2 = -1,
    .get_regulator = init_st_acc_regulator,
    .put_regulator = power_off_st_acc,
};

static struct akm8975_platform_data compass_platform_data = {
    .gpio_DRDY = 25,
    .gpio_config_interrupt = compass_interrupt_config,
    .power_on = init_akm_regulator,
    .power_off = power_off_akm,
};

static struct l3g4200d_gyr_platform_data l3g4200d_gyr_platform_data = {
    .poll_interval = 10,
    .min_interval =10,

    .fs_range = 0x30,

    .axis_map_x = 1,
    .axis_map_y = 0,
    .axis_map_z = 2,

    .negate_x = 1,
    .negate_y = 1,
    .negate_z = 0,
    .get_regulator = init_gyr_regulator,
        .put_regulator = power_off_gyr,
};

static struct mma8452_acc_platform_data mma8452_platform_data = {
        .power_on = init_fsc_acc_regulator,
        .power_off = power_off_fsc_acc,
};

static const struct adxl34x_platform_data adxl34x_default_init = {
    .tap_threshold = 35,
    .tap_duration = 3,
    .tap_latency = 20,
    .tap_window = 20,
    .tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
    .act_axis_control = 0xFF,
    .activity_threshold = 6,
    .inactivity_threshold = 4,
    .inactivity_time = 3,
    .free_fall_threshold = 8,
    .free_fall_time = 0x20,
    .data_rate = 10,
    //.data_rate = 8,
    .data_range = ADXL_FULL_RES,

    .ev_type = EV_ABS,
    .ev_code_x = ABS_X,    /* EV_REL */
    .ev_code_y = ABS_Y,    /* EV_REL */
    .ev_code_z = ABS_Z,    /* EV_REL */

    .ev_code_tap_x = BTN_TOUCH,    /* EV_KEY */
    .ev_code_tap_y = BTN_TOUCH,    /* EV_KEY */
    .ev_code_tap_z = BTN_TOUCH,    /* EV_KEY */
    .power_mode = ADXL_LINK,
    .fifo_mode = FIFO_STREAM,
    .watermark = 0,
    .gpio_config_interrupt = gsensor_interrupt_config,
    .power_on = init_adi_acc_regulator,
        .power_off = power_off_adi_acc,
};
/*struct tmd2771_platform_data light_platform_data = {
        .power_on = init_light_regulator,
        .power_off = power_off_light,
};*/
struct apds990x_platform_data apds990x_light_platform_data = {
        .gpio_config_interrupt = proximity_interrupt_config,
        .power_on = init_light_regulator,
        .power_off = power_off_light,
};
static struct ltr558_hw_platform_data ltr558_hw_data = {
        .ltr558_power = init_light_regulator,
        .ltr558_gpio_config_interrupt = proximity_interrupt_config,
};
static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_RMI4_SYNAPTICS
    {
        I2C_BOARD_INFO("Synaptics_rmi", 0x70),
        .platform_data = &synaptics_tp_platform_data,
        .irq = OMAP_GPIO_IRQ(TOUCH_OMAP4430_INT_PIN),
        .flags = true, //flags of the muti_touch
    },
#endif
#ifdef CONFIG_TOUCHSCREEN_MXT224E_ATMEL
    {
        I2C_BOARD_INFO("atmel_qt602240", 0x4a),
    },
#endif
};

static struct i2c_board_info  sdp4430_i2c_4_boardinfo[] = {

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_AK8975
    {
        I2C_BOARD_INFO("akm8975", 0x0c),
        .platform_data = &compass_platform_data,
        .flags = I2C_CLIENT_WAKE,
        .irq = OMAP_GPIO_IRQ(25),
    },
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
    {
        I2C_BOARD_INFO("lis3dh_acc", 0x18),
        .platform_data = &gs_platform_data,
    },

#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
    {
        I2C_BOARD_INFO("mma8452", 0x1c),
        .platform_data = &mma8452_platform_data,
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_GYROSCOPE_L3G4200DH
    {
        I2C_BOARD_INFO("l3g4200d_gyr", 0x68),
        .platform_data = &l3g4200d_gyr_platform_data,
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_TMD2771
    {
        I2C_BOARD_INFO("tmd2771", 0x39),
        .platform_data = &light_platform_data,
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
    {
        I2C_BOARD_INFO("adxl34x", 0x53),
        .irq = OMAP_GPIO_IRQ(23),
        .platform_data = &adxl34x_default_init,
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_LITEON_LTR_558

    {
        I2C_BOARD_INFO("ltr-558", 0x23),
        .irq = OMAP_GPIO_IRQ(28), 
        .platform_data = &ltr558_hw_data,
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_APDS990X
    {
        I2C_BOARD_INFO("apds990x", 0x39),
        .irq = OMAP_GPIO_IRQ(28),
        .platform_data = &apds990x_light_platform_data,
    },
#endif
};

static int __devinit hw_devices_init(void)
{
    int num = 0;
    while(num < ARRAY_SIZE(sdp4430_i2c_2_boardinfo))
    {
        if(!strncmp(sdp4430_i2c_2_boardinfo[num].type,"atmel_qt602240",13))
        {
           atmel_tp_platform_data_detect((struct atmel_i2c_platform_data **)&sdp4430_i2c_2_boardinfo[num].platform_data);
           break;
        }
        num++;
    }
    if (num == ARRAY_SIZE(sdp4430_i2c_2_boardinfo))
    {
        pr_err("register atmel touchscreen ERROR!\n");
    }
    else
    {
        i2c_register_board_info(2, sdp4430_i2c_2_boardinfo, ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
    }
    i2c_register_board_info(4, sdp4430_i2c_4_boardinfo, ARRAY_SIZE(sdp4430_i2c_4_boardinfo));
    #ifdef CONFIG_TOUCHSCREEN_VIRTUALKEY
    virtualkeys_init();
    #endif
     return 0;
}
arch_initcall(hw_devices_init);

MODULE_DESCRIPTION("hawei devices init");
MODULE_LICENSE("GPL");

