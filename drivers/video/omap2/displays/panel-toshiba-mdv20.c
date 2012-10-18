/*
 * Toshiba MDV20 panel support
 *
 * Copyright (C) Texas Instruments  Corporation
 * Author: Hari Nagalla <hnagalla@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
  


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
//#include <plat/display.h>
#include <video/omapdss.h>
#include <plat/toshiba-dsi-panel.h>
#include <linux/lcd_tuning.h>
#include <linux/device.h>

/*  Reason: lcd-compatible  */
#include <linux/panel_detect.h>
#define TOSHIBA_PANEL_DEBUG
#define PANEL_TAG  "TOSHIBA: "
#ifdef TOSHIBA_PANEL_DEBUG
#define TOSHIBA_PANEL_DBG(fmt, ...) \
    printk(PANEL_TAG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define TOSHIBA_PANEL_DBG(fmt, ...) do{ \
    }while(0)
#endif


static struct lcd_tuning_dev *p_tuning_dev = NULL;

#ifdef CONFIG_LCDD
#include "lcdd.h"
#endif
/* END:   Added by meijinfang, 2011/10/24 */
/* DSI Command Virtual channel */
#define CMD_VC_CHANNEL 0

#define DRIVER_NAME     "mdv20"

#define CHIP_R63306    0x06
#define CHIP_R63308    0x08

static int boot_first = true;
/* define this if you want debug print messages */
/* #define DEB */
static int cabc_mode = 1;	//allow application to set cabc mode to ui mode
#ifdef DEB
#define PRINT_DEBUG(...) printk(__VA_ARGS__)
#else
#define PRINT_DEBUG(...)
#endif

static int chip_ver;

static int mdv20_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h);
int mdv20_config(struct omap_dss_device *dssdev);

static void mdv20_stop(struct omap_dss_device *dssdev);
static int mdv20_start(struct omap_dss_device *dssdev);
struct panel_regulator {
	struct regulator *regulator;
	const char *name;
	int min_uV;
	int max_uV;
};

/*  Reason: lcd-compatible  */
static struct panel_regulator power_regulator[] =
{
	{
		.name = "vaux2",
		.min_uV = 2800000,
		.max_uV = 2800000,
	},
};

static void free_regulators(struct panel_regulator *regulators, int n)
{
	int i;
	for (i = 0; i < n; i++) {
		/* disable/put in reverse order */
		regulator_disable(regulators[n - i - 1].regulator);
		regulator_put(regulators[n - i - 1].regulator);
	}
}

static int init_regulators(struct omap_dss_device *dssdev,
			   struct panel_regulator *regulators, int n)
{
	int r, i, v;

	for (i = 0; i < n; i++) {
		struct regulator *reg;

		reg = regulator_get(&dssdev->dev, regulators[i].name);
		if (IS_ERR(reg)) {
			dev_err(&dssdev->dev, "failed to get regulator %s\n",
				regulators[i].name);
			r = PTR_ERR(reg);
			goto err;
		}

		/* FIXME: better handling of fixed vs. variable regulators */
		v = regulator_get_voltage(reg);
		if (v < regulators[i].min_uV || v > regulators[i].max_uV) {
			r = regulator_set_voltage(reg, regulators[i].min_uV,
						  regulators[i].max_uV);
			if (r) {
				dev_err(&dssdev->dev,
					"failed to set regulator %s voltage\n",
					regulators[i].name);
				regulator_put(reg);
				goto err;
			}
		}

		r = regulator_enable(reg);
		if (r) {
			dev_err(&dssdev->dev, "failed to enable regulator %s\n",
				regulators[i].name);
			regulator_put(reg);
			goto err;
		}

		regulators[i].regulator = reg;
	}

	return 0;

err:
	free_regulators(regulators, i);

	return r;
}

/**
 * struct panel_config - panel configuration
 * @name: panel name
 * @type: panel type
 * @timings: panel resolution
 * @sleep: various panel specific delays, passed to msleep() if non-zero
 * @reset_sequence: reset sequence timings, passed to udelay() if non-zero
 * @regulators: array of panel regulators
 * @num_regulators: number of regulators in the array
 */
struct panel_config {
	const char *name;
	int type;

	struct omap_video_timings timings;
	//struct omap_dsi_video_timings dsi_video_timings;
	u32 width_in_um;
	u32 height_in_um;

	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int hw_reset;
		unsigned int enable_te;
	} sleep;

	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;

	struct panel_regulator *regulators;
	int num_regulators;
};

enum {
	PANEL_MDV20,
};

/*  Reason: Moidify for LCD display error on top line  */
static struct panel_config panel_configs[] = {
	{
	 .name = "mdv20",
	 .type = PANEL_MDV20,
	 .timings = {
		     .x_res = 720,
		     .y_res = 1280,
		     //.pixel_clock = 69300,
		     .pixel_clock = 81300,
	/* the blanking perdiods specified here are not considered.
           the values are set in dsi.c */
		     .hfp = 250,
		     .hsw = 9,
		     .hbp = 1,
		     .vfp = 20,
		     .vsw = 1,
		     .vbp = 6,
		     },
         .width_in_um = 57000,
         .height_in_um = 101000,
	/*
	 .dsi_video_timings = {
		     .hsa = 1,
		     .hfp = 283,
		     .hbp = 44,
		     .vsa = 1,
		     .vfp = 14,
		     .vbp = 14,
		     },
	*/
	 .sleep		= {
			.sleep_in	= 5,
			.sleep_out	= 5,
			.hw_reset	= 10,//wait 10ms after reset to high 
			.enable_te	= 100, /* possible panel bug */
		},
	.reset_sequence	= {
			.high		= 50,
			.low		= 1500,//reset keep low time us
		},	
	.regulators = power_regulator,
	.num_regulators = ARRAY_SIZE(power_regulator),
	},
};


struct mdv20_data {
	struct mutex lock;

	/* 0004-Added-the-backlight-adjustment-support.patch */
	struct backlight_device *bldev;
	/* patch end */
	struct omap_dss_device *dssdev;
	bool enabled;
	u8 rotate;
	bool mirror;
	bool use_dsi_bl;
	unsigned long hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long hw_guard_wait;	/* max guard time in jiffies */

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;

	bool cabc_broken;
	unsigned cabc_mode;

	struct workqueue_struct *workqueue;
	struct delayed_work esd_work;
	bool force_update;
	struct panel_config *panel_config;
};

static inline struct toshiba_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct toshiba_dsi_panel_data *)dssdev->data;
}

/*************************************
**** Interface utility functions *****
*************************************/


/****************************
********* DEBUG *************
****************************/

/***********************
*** DUMMY FUNCTIONS ****
***********************/

static void mdv20_esd_work(struct work_struct *work);
static int mdv20_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	return 0;
}
static void mdv20_queue_esd_work(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);

	queue_delayed_work(mdv20data->workqueue, &mdv20data->esd_work, msecs_to_jiffies(10000));
}

static void mdv20_cancel_esd_work(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);

	cancel_delayed_work(&mdv20data->esd_work);
}


static void mdv20_esd_work(struct work_struct *work)
{
	struct mdv20_data *mdv20data = container_of(work, struct mdv20_data,esd_work.work);
	struct omap_dss_device *dssdev = mdv20data->dssdev;
	u8 power_mode;
	u8 data_buf[80];
	u8 expected_mode;
	int r, i;

	mutex_lock(&mdv20data->lock);
	if (!mdv20data->enabled) {
		mutex_unlock(&mdv20data->lock);
		return;
	}

	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	r = dsi_vc_dcs_read(dssdev, CMD_VC_CHANNEL, 0x0a, data_buf,1);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);

	if (r < 0) {
		dev_err(&dssdev->dev, "Failed to get power mode, r = %d\n", r);
		printk("HARI: %s , Failed to get power mode, r = %d\n",__func__, r);
		goto err;
	}



	if ((data_buf[0] & 0x0f) != 0x0c) {
		dev_err(&dssdev->dev,
			"Power mode in incorrect state, "
			"mode = 0x%02x, expected = 0x%02x\n",
			power_mode, expected_mode);
		goto err;
	}

	mdv20_queue_esd_work(dssdev);
	mutex_unlock(&mdv20data->lock);
	return;
err:
	mutex_unlock(&mdv20data->lock);
	dev_err(&dssdev->dev, "ESD: performing LCD reset\n");
	mdv20_stop(dssdev);
	mdelay(20);
	printk(KERN_INFO"ESD: mdv20_panel_power_on.\n");
	/* Try to turn panel back on up to 10 times */
	for (i  = 0; i < 10; i++) {
		r = mdv20_start(dssdev);
		if (!r)
			break;
	}
	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		printk("HARI : %s, enable failed\n",__func__);
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}
	return r;
}

static u8 mdv20_get_rotate(struct omap_dss_device *dssdev)
{
	return 0;
}

static int mdv20_mirror(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static bool mdv20_get_mirror(struct omap_dss_device *dssdev)
{
	return 0;
}

static void mdv20_get_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void mdv20_set_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int mdv20_check_timings(struct omap_dss_device *dssdev,
			     struct omap_video_timings *timings)
{
	if (timings->x_res != 720 || timings->y_res != 1280)
		return -EINVAL;

	return 0;
}

static void mdv20_get_resolution(struct omap_dss_device *dssdev,
			       u16 *xres, u16 *yres)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);

	if (mdv20data->rotate == 0 || mdv20data->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
	dev_dbg(&dssdev->dev, "xres: %d, yres: %d\n", *xres, *yres);
}


static char cabc_user_parameter[3] = {0xbe, 0xff, 0x0f};
static ssize_t mdv20_bl_set_locked(int level, struct omap_dss_device *dssdev)
{
    int r = 0;
    
    if (level < 0)
        level = 0;
    else if (level > 255)
        level = 255;
    if(chip_ver ==  CHIP_R63308)
    {
        cabc_user_parameter[1] = ((level & 0xf0) >> 4);
        cabc_user_parameter[2] = level & 0x0f;
    }
    else if (chip_ver ==  CHIP_R63306)
    {
        cabc_user_parameter[1] = level;
        cabc_user_parameter[2] = 0x0f;
    }
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0, 1);
	r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, cabc_user_parameter,3);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0, 0);
    return r;
}
static int mdv20_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct mdv20_data *td = dev_get_drvdata(&dssdev->dev);
	struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;
	int level;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;
	/*we compare the brightness of toshiba lcd to iphone4,
	and find toshiba's brightness is a little stronger,then we reduce
	20% of the brightness to make it is the same to iphone4.*/
	level = (level*8)/10;
	dev_dbg(&dssdev->dev, "update brightness to %d\n", level);

	mutex_lock(&td->lock);

	if (td->use_dsi_bl) {
		if (td->enabled) {
			dsi_bus_lock(dssdev);
			r = mdv20_bl_set_locked(level, dssdev);
			dsi_bus_unlock(dssdev);
		} else {
			r = 0;
		}
	} else {
		if (!panel_data->set_backlight)
			r = -EINVAL;
		else
			r = panel_data->set_backlight(dssdev, level);
	}

	mutex_unlock(&td->lock);

	return r;
}
/* patch end */
static ssize_t mdv20_num_errors_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	u8 errors[10];
	int r = 0;
	struct omap_dss_device *dssdev = to_dss_device(dev);

	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	r = dsi_vc_dcs_read(dssdev, CMD_VC_CHANNEL, 0xb5, errors,3);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
	if (r < 0) {
		printk("HARII:: %s, failed !!! \n",__func__);
		return r;
	}

	printk("HARII:: error1 = %d \n",(int)errors[0]);	
	printk("HARII:: error2 = %d \n",(int)errors[1]);	
	printk("HARII:: error3 = %d \n",(int)errors[2]);	

	return snprintf(buf, PAGE_SIZE, "%d\n", errors[0]);

}

static ssize_t mdv20_power_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int r = 0;
	u8 data_buf[80];
	struct omap_dss_device *dssdev = to_dss_device(dev);

	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	r = dsi_vc_dcs_read(dssdev, CMD_VC_CHANNEL, 0x0a, data_buf,1);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
	if (r < 0) {
		printk("HARII:: %s, failed !!! \n",__func__);
		return r;
	}

	printk("HARII:: POWER_MODE = %d \n",(int)data_buf[0]);	

	return snprintf(buf, PAGE_SIZE, "%d\n", data_buf[0]);
}

static ssize_t mdv20_address_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{	
	int r = 0;
	u8 data_buf[80];
	struct omap_dss_device *dssdev = to_dss_device(dev);


	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	r = dsi_vc_dcs_read(dssdev, CMD_VC_CHANNEL, 0x0b, data_buf,1);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
	if (r < 0) {
		printk("HARII:: %s, failed !!! \n",__func__);
		return r;
	}

	printk("HARII:: ADDRESS_MODE = %d \n",(int)data_buf[0]);	

	return snprintf(buf, PAGE_SIZE, "%d\n", data_buf[0]);
}

static ssize_t mdv20_display_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{	
	int r = 0;
	u8 data_buf[80];
	struct omap_dss_device *dssdev = to_dss_device(dev);

	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	r = dsi_vc_dcs_read(dssdev, CMD_VC_CHANNEL, 0x0d, data_buf,1);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
	if (r < 0) {
		printk("HARII:: %s, failed !!! \n",__func__);
		return r;
	}

	printk("HARII:: DISPLAY_MODE = %d \n",(int)data_buf[0]);	

	return snprintf(buf, PAGE_SIZE, "%d\n", data_buf[0]);
}

static ssize_t mdv20_signal_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int r = 0;
	u8 data_buf[7] = {0xbe, 0xff, 0x0f, 0x1a, 0x18, 0x02, 0x40};
	struct omap_dss_device *dssdev = to_dss_device(dev);
	static index = 0;
	if(index>255)
       	 index=0;
	
	data_buf[1] = index;
	index +=50;
	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, data_buf,7);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
	if (r < 0) {
		printk("HARII:: %s, failed !!! \n",__func__);
		return r;
	}
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0, 0);
       dsi_bus_unlock(dssdev);

	printk("HARII:: SIGNALK_MODE = %d \n",(int)data_buf[0]);	

	return 0;

	//	snprintf(buf, PAGE_SIZE, "%d\n", data_buf[0]);
}

static ssize_t mdv20_restart_video_xfer(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	int r = 0;
	u8 data_buf[80];
	
	// Set backlight on
	data_buf[0] = 0xbb;
	data_buf[1] = 0x08;
	r = dsi_vc_dcs_write(dssdev, CMD_VC_CHANNEL, data_buf,2);
	if (r)
		printk("HARII::  failed to send command \n");

	// Enable HS mode
	omapdss_dsi_vc_enable_hs(dssdev, 0, true);

	// Enable Video mode
	//dsi_videomode_panel_postinit(dssdev);
	dsi_video_mode_enable(dssdev, 0x3e);

	return snprintf(buf, PAGE_SIZE, "%d\n", r);
}
/* 0002-add-sysfs-entry-for-diaplay-on-off-for-testing-inter.patch */
static ssize_t store_mdv20_displayoff(struct device *dev,
				    struct device_attribute *attr, char * buf)
 {
	int r = 0;
	u8 data_buf[80];
	u8 off;
	struct omap_dss_device *dssdev = to_dss_device(dev);
		
	off = simple_strtoul(buf, NULL, 10);
	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	
	printk( "request is %d\n", off); 
	if  (off == 1)
		data_buf[0] = 0x28;
	else
		data_buf[0] = 0x29;
	r= dsi_vc_dcs_write_nosync(dssdev, CMD_VC_CHANNEL, data_buf,1);
	if (r < 0) {
		printk("HARII:: %s, failed !!! \n",__func__);
		return r;
	}	
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
 	return 0;
 } 
 /* Patch end */

static ssize_t show_lcd_info(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    char * lcd_info = "Toshiba TFT 4.5' LCD.";
    strncpy(buf, lcd_info, strlen(lcd_info)+1);
    return strlen(lcd_info)+1;
}

static ssize_t show_cabc_mode(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	return sprintf(buf, "%d\n", cabc_mode);
}
static int mdv20_set_cabc(struct lcd_tuning_dev *ltd, enum tft_cabc cabc);
static ssize_t store_cabc_mode(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	int rc;
	unsigned long val;
	rc = strict_strtoul(buf, 0, &val);
	if (rc)
		return rc;
	if(val == 1)
	{
	    cabc_mode =1;//allow application to set cabc mode to ui mode
	    mdv20_set_cabc(p_tuning_dev,CABC_UI);
	}
	else if(val == 2)
	{
	    cabc_mode =2;//force cabc mode to video mode
	    mdv20_set_cabc(p_tuning_dev,CABC_VID);
	}
	return sprintf(buf, "%d\n", cabc_mode);
}

static DEVICE_ATTR(power_mode, S_IRUGO, mdv20_power_mode_show, NULL);
static DEVICE_ATTR(address_mode, S_IRUGO, mdv20_address_mode_show, NULL);
static DEVICE_ATTR(display_mode, S_IRUGO, mdv20_display_mode_show, NULL);
// static DEVICE_ATTR(signal_mode, S_IRUGO, mdv20_signal_mode_show, NULL);
static DEVICE_ATTR(num_dsi_errors, S_IRUGO, mdv20_num_errors_show, NULL);
static DEVICE_ATTR(restart_video_xfer, S_IRUGO, mdv20_restart_video_xfer, NULL);
//static DEVICE_ATTR(hw_revision, S_IRUGO, mdv20_hw_revision_show, NULL);
/* 0002-add-sysfs-entry-for-diaplay-on-off-for-testing-inter.patch */
static DEVICE_ATTR(display_off, S_IRUGO |S_IWUSR, NULL, store_mdv20_displayoff);
/* Patch end */
/* 0004-Added-the-backlight-adjustment-support.patch */
static DEVICE_ATTR(signal_mode, S_IRUGO, mdv20_signal_mode_show, NULL);
/* patch end */
static DEVICE_ATTR(lcd_info, S_IRUGO,
        show_lcd_info, NULL);
static DEVICE_ATTR(cabc_mode, 0644,show_cabc_mode, store_cabc_mode);
static struct attribute *mdv20_attrs[] = {
	&dev_attr_power_mode.attr,
	&dev_attr_address_mode.attr,
	&dev_attr_display_mode.attr,
	// &dev_attr_signal_mode.attr,
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_restart_video_xfer.attr,
	//&dev_attr_hw_revision.attr,
	/* 0002-add-sysfs-entry-for-diaplay-on-off-for-testing-inter.patch */
	&dev_attr_display_off.attr,
	/* Patch end */
	/* 0004-Added-the-backlight-adjustment-support.patch */
	&dev_attr_signal_mode.attr,
    /* patch end */
    &dev_attr_lcd_info.attr,
    &dev_attr_cabc_mode.attr,
	NULL,
};

static struct attribute_group mdv20_attr_group = {
	.attrs = mdv20_attrs,
};

static int mdv20_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static int mdv20_hw_reset(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (panel_data->reset_gpio == -1)
		return -1;

	gpio_set_value(panel_data->reset_gpio, 1);
	if (mdv20data->panel_config->reset_sequence.high)
		udelay(mdv20data->panel_config->reset_sequence.high);
	/* reset the panel */
	gpio_set_value(panel_data->reset_gpio, 0);
	/* assert reset */
	if (mdv20data->panel_config->reset_sequence.low)
		udelay(mdv20data->panel_config->reset_sequence.low);
	gpio_set_value(panel_data->reset_gpio, 1);
	/* wait after releasing reset */
	if (mdv20data->panel_config->sleep.hw_reset)
		msleep(mdv20data->panel_config->sleep.hw_reset);
	
	/* enable 5.4V */
	if (panel_data->chip_pwr_save != -1)
		gpio_set_value(panel_data->chip_pwr_save,0);
	if (panel_data->v5_4_enable != -1)
		gpio_set_value(panel_data->v5_4_enable,1);
       msleep(10);
	
	return 0;
}

/* 0004-Added-the-backlight-adjustment-support.patch */
static int mdv20_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static int _set_cabc(struct omap_dss_device *dssdev, enum tft_cabc cabc)
{

	int r = 0;
	u8 buf[80];

	switch(cabc)
	{
		case CABC_UI:
		{
		    buf[0] = 0xb7;
		    buf[1] = 0x18;
		    buf[2] = 0x00;
		    buf[3] = 0x18;
		    buf[4] = 0x18;
		    buf[5] = 0x0c;
		    buf[6] = 0x10;
		    buf[7] = 0x5c;
		    buf[8] = 0x10;
		    buf[9] = 0xac;
		    buf[10] = 0x10;
		    buf[11] = 0x0c;
		    buf[12] = 0x10;
		    buf[13] = 0x00;
		    buf[14] = 0x10;


		    r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf,15);
		    if (r)
			goto err;

		    buf[0] = 0xb8;
		    buf[1] = 0xf8;
		    buf[2] = 0xda;
		    buf[3] = 0x6d;
		    buf[4] = 0xfb;
		    buf[5] = 0xff;
		    buf[6] = 0xff;
		    buf[7] = 0xcf;
		    buf[8] = 0x1f;
		    buf[9] = 0xc0;
		    buf[10] = 0xd3;
		    buf[11] = 0xe3;
		    buf[12] = 0xf1;
		    buf[13] = 0xff;


		    r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf,14);
		    if (r)
			goto err;

			buf[0] = 0xbe;
			buf[1] = cabc_user_parameter[1];	/* Dont touch backlight level */
			buf[2] = cabc_user_parameter[2];
			buf[3] = 0x2;
			buf[4] = 0x2;
			buf[5] = 0x4;
			buf[6] = 0x4;
		    r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf,7);
		    if (r)
			goto err;

		}
		break;

		case CABC_VID:
		{

		    buf[0] = 0xb7;
		    buf[1] = 0x18;
		    buf[2] = 0x00;
		    buf[3] = 0x18;
		    buf[4] = 0x18;
		    buf[5] = 0x0c;
		    buf[6] = 0x13;
		    buf[7] = 0x5c;
		    buf[8] = 0x13;
		    buf[9] = 0xac;
		    buf[10] = 0x13;
		    buf[11] = 0x0c;
		    buf[12] = 0x13;
		    buf[13] = 0x00;
		    buf[14] = 0x10;

		    r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf,15);
		    if (r)
			goto err;

		    buf[0] = 0xb8;
		    buf[1] = 0xf8;
		    buf[2] = 0xda;
		    buf[3] = 0x6d;
		    buf[4] = 0xfb;
		    buf[5] = 0xff;
		    buf[6] = 0xff;
		    buf[7] = 0xcf;
		    buf[8] = 0x1f;
		    buf[9] = 0x67;
		    buf[10] = 0x89;
		    buf[11] = 0xaf;
		    buf[12] = 0xd6;
		    buf[13] = 0xff;


		    r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf,14);
		    if (r)
			goto err;

			buf[0] = 0xbe;
			buf[1] = cabc_user_parameter[1];	/* Dont touch backlight level */
			buf[2] = cabc_user_parameter[2];
			buf[3] = 0x0;
			buf[4] = 0x18;
			buf[5] = 0x4;
			buf[6] = 0x40;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf,7);
			if (r)
			goto err;

		}
		break;

		default:
		case CABC_OFF:
		{
			r = 0; //Not implemented for this panel
		}
		break;
	}
	r = 0;
err:

	return r;
}

static int mdv20_set_cabc(struct lcd_tuning_dev *ltd, enum tft_cabc cabc)
{

	struct omap_dss_device *dssdev = (struct omap_dss_device *)ltd->data;
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&mdv20data->lock);
	if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
	{
		mutex_unlock(&mdv20data->lock);
		return -1;
	}

	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);

	if(cabc_mode==1) //allow application to set cabc mode to ui mode
		_set_cabc(dssdev, cabc);
	else
		_set_cabc(dssdev, CABC_VID);
	
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mdv20data->lock);

	return 0;
}

static int mdv20_set_gamma(struct lcd_tuning_dev *ltd, enum lcd_gamma gamma)
{
	int r = 0;
	u8 buf[80];
	struct omap_dss_device *dssdev = (struct omap_dss_device *)ltd->data;
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);


	mutex_lock(&mdv20data->lock);
	if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
	{
		mutex_unlock(&mdv20data->lock);
		return -1;
	}


	dsi_bus_lock(dssdev);
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
	


	switch(gamma)
	{
		case GAMMA25 :
		{
			buf[0] = 0xc9;
			buf[1] = 0x0f;
			buf[2] = 0x14;
			buf[3] = 0x21;
			buf[4] = 0x2e;
			buf[5] = 0x32;
			buf[6] = 0x2e;
			buf[7] = 0x3a;
			buf[8] = 0x45;
			buf[9] = 0x3f;
			buf[10] = 0x42;
			buf[11] = 0x52;
			buf[12] = 0x39;
			buf[13] = 0x33;


			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xca;
			buf[1] = 0x30;
			buf[2] = 0x2b;
			buf[3] = 0x3e;
			buf[4] = 0x31;
			buf[5] = 0x2d;
			buf[6] = 0x31;
			buf[7] = 0x25;
			buf[8] = 0x1a;
			buf[9] = 0x20;
			buf[10] = 0x1d;
			buf[11] = 0x0d;
			buf[12] = 0x06;
			buf[13] = 0x0c;

			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;


			buf[0] = 0xcb;
			buf[1] = 0x0f;
			buf[2] = 0x14;
			buf[3] = 0x21;
			buf[4] = 0x2e;
			buf[5] = 0x32;
			buf[6] = 0x2e;
			buf[7] = 0x3a;
			buf[8] = 0x45;
			buf[9] = 0x3f;
			buf[10] = 0x42;
			buf[11] = 0x52;
			buf[12] = 0x39;
			buf[13] = 0x33;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xcc;
			buf[1] = 0x30;
			buf[2] = 0x2b;
			buf[3] = 0x3e;
			buf[4] = 0x31;
			buf[5] = 0x2d;
			buf[6] = 0x31;
			buf[7] = 0x25;
			buf[8] = 0x1a;
			buf[9] = 0x20;
			buf[10] = 0x1d;
			buf[11] = 0x0d;
			buf[12] = 0x06;
			buf[13] = 0x0c;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xcd;
			buf[1] = 0x0f;
			buf[2] = 0x14;
			buf[3] = 0x21;
			buf[4] = 0x2e;
			buf[5] = 0x32;
			buf[6] = 0x2e;
			buf[7] = 0x3a;
			buf[8] = 0x45;
			buf[9] = 0x3f;
			buf[10] = 0x42;
			buf[11] = 0x52;
			buf[12] = 0x39;
			buf[13] = 0x33;

			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xce;
			buf[1] = 0x30;
			buf[2] = 0x2b;
			buf[3] = 0x3e;
			buf[4] = 0x31;
			buf[5] = 0x2d;
			buf[6] = 0x31;
			buf[7] = 0x25;
			buf[8] = 0x1a;
			buf[9] = 0x20;
			buf[10] = 0x1d;
			buf[11] = 0x0d;
			buf[12] = 0x06;
			buf[13] = 0x0c;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;
		}
		break;

		case GAMMA22:
		{
			buf[0] = 0xc9;
			buf[1] = 0x0f;
			buf[2] = 0x10;
			buf[3] = 0x1a;
			buf[4] = 0x25;
			buf[5] = 0x28;
			buf[6] = 0x25;
			buf[7] = 0x32;
			buf[8] = 0x3e;
			buf[9] = 0x38;
			buf[10] = 0x3b;
			buf[11] = 0x4d;
			buf[12] = 0x36;
			buf[13] = 0x33;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xca;
			buf[1] = 0x30;
			buf[2] = 0x2f;
			buf[3] = 0x45;
			buf[4] = 0x3a;
			buf[5] = 0x37;
			buf[6] = 0x3a;
			buf[7] = 0x2d;
			buf[8] = 0x21;
			buf[9] = 0x27;
			buf[10] = 0x24;
			buf[11] = 0x12;
			buf[12] = 0x09;
			buf[13] = 0x0c;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;


			buf[0] = 0xcb;
			buf[1] = 0x0f;
			buf[2] = 0x10;
			buf[3] = 0x1a;
			buf[4] = 0x25;
			buf[5] = 0x28;
			buf[6] = 0x25;
			buf[7] = 0x32;
			buf[8] = 0x3e;
			buf[9] = 0x38;
			buf[10] = 0x3b;
			buf[11] = 0x4d;
			buf[12] = 0x36;
			buf[13] = 0x33;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xcc;
			buf[1] = 0x30;
			buf[2] = 0x2f;
			buf[3] = 0x45;
			buf[4] = 0x3a;
			buf[5] = 0x37;
			buf[6] = 0x3a;
			buf[7] = 0x2d;
			buf[8] = 0x21;
			buf[9] = 0x27;
			buf[10] = 0x24;
			buf[11] = 0x12;
			buf[12] = 0x09;
			buf[13] = 0x0c;

			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;


			buf[0] = 0xcd;
			buf[1] = 0x0f;
			buf[2] = 0x10;
			buf[3] = 0x1a;
			buf[4] = 0x25;
			buf[5] = 0x28;
			buf[6] = 0x25;
			buf[7] = 0x32;
			buf[8] = 0x3e;
			buf[9] = 0x38;
			buf[10] = 0x3b;
			buf[11] = 0x4d;
			buf[12] = 0x36;
			buf[13] = 0x33;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;

			buf[0] = 0xce;
			buf[1] = 0x30;
			buf[2] = 0x2f;
			buf[3] = 0x45;
			buf[4] = 0x3a;
			buf[5] = 0x37;
			buf[6] = 0x3a;
			buf[7] = 0x2d;
			buf[8] = 0x21;
			buf[9] = 0x27;
			buf[10] = 0x24;
			buf[11] = 0x12;
			buf[12] = 0x09;
			buf[13] = 0x0c;
			r = dsi_vc_mcs_write_nosync(dssdev,CMD_VC_CHANNEL, buf, 14);
			if(r)
				goto out;
		}
		break;
		
		default:
			r = -1;
	}
out:
	omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
	dsi_bus_unlock(dssdev);

	mutex_unlock(&mdv20data->lock);

	return r;
};
/* END:   Added by meijinfang, 2011/12/22 */
static struct backlight_ops sp_bl_ops = {
	.get_brightness = mdv20_bl_get_intensity,
	.update_status  = mdv20_bl_update_status,
};
/* patch end */
static struct lcd_tuning_ops mdv20_ops = {
	.set_gamma = mdv20_set_gamma,
	.set_cabc = mdv20_set_cabc,
};
/* END:   Added by meijinfang, 2011/12/22 */

static int mdv20_probe(struct omap_dss_device *dssdev)
{
	int ret = 0;
	int i;
	struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct mdv20_data *mdv20data = NULL;
	struct panel_config *panel_config = NULL;
	/* 0004-Added-the-backlight-adjustment-support.patch */
	struct backlight_properties props;
	struct backlight_device *bldev;
	struct lcd_tuning_dev *tuning_dev = 0; 
	/* patch end */
	int channel = 0;

	dev_dbg(&dssdev->dev, "mdv20_probe\n");

	/* 0004-Added-the-backlight-adjustment-support.patch */	
	printk( "mdv20_probe\n");
	/* patch end */
	if (!panel_data || !panel_data->name) {
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		ret = -EINVAL;
		goto err;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = panel_config->timings;
	dssdev->panel.width_in_um = panel_config->width_in_um;
	dssdev->panel.height_in_um = panel_config->height_in_um;
	dssdev->panel.data_type = DSI_DT_PXLSTREAM_24BPP_PACKED;
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	mdv20data = kzalloc(sizeof(*mdv20data), GFP_KERNEL);
	if (!mdv20data) {
		ret = -ENOMEM;
		goto err;
	}

	mdv20data->dssdev = dssdev;

	mdv20data->panel_config = panel_config;

	mutex_init(&mdv20data->lock);

	ret = init_regulators(dssdev, panel_config->regulators,
			      panel_config->num_regulators);
	if (ret)
		goto err;
	mdv20data->workqueue = create_singlethread_workqueue("mdv20_esd");
	if (mdv20data->workqueue == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue \n");
		ret = -ENOMEM;
		goto err_wq;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&mdv20data->esd_work, mdv20_esd_work);
	dev_set_drvdata(&dssdev->dev, mdv20data);
	
	ret = omap_dsi_request_vc(dssdev, &channel);
	if (ret)
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");

	ret = omap_dsi_set_vc_id(dssdev, channel, 0);
	if (ret)
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");

	if (cpu_is_omap44xx())
		mdv20data->force_update = true;

/* 0004-Added-the-backlight-adjustment-support.patch */
/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	memset(&props, 0, sizeof(struct backlight_properties));
	if (!panel_data->set_backlight)
		mdv20data->use_dsi_bl = true;

	if (mdv20data->use_dsi_bl)
		props.max_brightness = 255;
	else
		props.max_brightness = 127;

        props.type = BACKLIGHT_RAW;
		bldev = backlight_device_register(dssdev->name,
			&dssdev->dev, dssdev, &sp_bl_ops, &props);

	if (IS_ERR(bldev)) {
		PTR_ERR(bldev);
		goto err_bl;
	}
	mdv20data->bldev = bldev;

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.power = FB_BLANK_UNBLANK;
	if (mdv20data->use_dsi_bl)
		bldev->props.brightness = 255;
	else
		bldev->props.brightness = 127;
	tuning_dev = lcd_tuning_dev_register(0, &mdv20_ops, dssdev);
	p_tuning_dev = tuning_dev;
	if (IS_ERR(tuning_dev)) {
		PTR_ERR(tuning_dev);
		goto err;
	}
	/* END:   Added by meijinfang, 2011/12/22 */
//	mdv20_bl_update_status(bldev);

/* patch end */

	ret = sysfs_create_group(&dssdev->dev.kobj, &mdv20_attr_group);
	if (ret)
		dev_err(&dssdev->dev, "failed to create sysfs files\n");

	return ret;

	/* 0004-Added-the-backlight-adjustment-support.patch */
	//backlight_device_unregister(bldev);

err_bl:
/* patch end */
err_wq:
err:
	kfree(mdv20data);

	return ret;
}

static void mdv20_remove(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	mdv20_cancel_esd_work(dssdev);
	destroy_workqueue(mdv20data->workqueue);
	kfree(mdv20data);
}

#ifdef CONFIG_LCDD
static int _mdv20_config(struct omap_dss_device *dssdev, struct params *pars)
{

	int i;
	u8 buf[80];
	enum omap_dsi_index ix;
	int r;
	ix = DSI1;

	for(i = 0; i < pars->line_nr; i++)
	{
		uint32_t data_len;
		if(!pars->params[i])
		{
			printk("%s : Unexpected void param\n", __func__);
			continue;
		}
		buf[0] = pars->params[i]->cmd;
		if((data_len = pars->params[i]->data_len) >0)
			memcpy(&buf[1], pars->params[i]->data, data_len);

		r = dsi_vc_dcs_write(dssdev,CMD_VC_CHANNEL, buf, 1+data_len);
		if (r)
			goto err;
		if(pars->params[i]->delay)
			udelay(pars->params[i]->delay);
	}
	return 0;

err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset \n");

	//mdv20_hw_reset(dssdev);//if enable this ,system will locked in 	mutex_lock(&p_dsi->lock); by z00174260
	omapdss_dsi_display_disable(dssdev, true, false);
	return r;
}
#endif

static int mdv20_power_on(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	//struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int ret = 0;

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

//	if (mdv20data->enabled != 1)
	{
		/* enable 5.4V */
	//	gpio_set_value(panel_data->chip_pwr_save,0);
	//	gpio_set_value(panel_data->v5_4_enable,1);

		//gpio_direction_output(157,1);
		//gpio_direction_output(158,0);

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			goto err;
		}

		if(!dssdev->skip_init)
		{
		/* reset mdv20 */
		mdv20_hw_reset(dssdev);
		msleep(10);

		/* do extra job to match kozio registers */
		dsi_videomode_panel_preinit(dssdev);
		//printk("HARII: %s videomode _panel_preinit completed !!! \n",__func__);
   		omapdss_dsi_vc_enable_hs(dssdev, 0, false);

                /* Toshiba Bridge Constraint */
		msleep(10);

#ifdef CONFIG_LCDD
		{
			int i, r;
			struct params *params;
			if(0 == lcdd_parse_init(&params))
			{
				r = lcdd_parse();
				if(0==r)
				{
					for(i = 0; i < params->line_nr; i++)
					{
						struct lcd_param *p = params->params[i];
						if(p)
						{
						int j;
						printk("%d,0x%x,", p->delay, p->cmd);
						for(j = 0; j < p->data_len; j++)
							printk("0x%x,", p->data[j]);
						printk("\n");
						}

					}
					ret = _mdv20_config(dssdev, params);
				}
				//lcdd_parse_deinit(params);
			}
			else
			{		
				if(!boot_first)
				{
					ret =mdv20_config(dssdev);
				}
			}
		}
#else

			ret =mdv20_config(dssdev);
#endif
		if(ret) 
		{
			dev_err(&dssdev->dev, "Failed toconfig mdv20\n");
			goto err;
		}
   		omapdss_dsi_vc_enable_hs(dssdev, 0, true);
   		/* 0x0e - 16bit
		 * 0x1e - packed 18bit
		 * 0x2e - unpacked 18bit
		 * 0x3e - 24bit
		 */
		dsi_video_mode_enable(dssdev, 0x3e);
		//dsi_videomode_panel_postinit(dssdev);
		}
		//printk("HARII: %s videomode _panel_postinit completed !!! \n",__func__);
		mdv20data->enabled = 1;
	}

err:
	return ret;
}
/* END:   Added by meijinfang, 2011/10/24 */
/**
 * d2l_config - Configure D2L
 *
 * Initial configuration for D2L configuration registers, PLL...
 */
int mdv20_config(struct omap_dss_device *dssdev)
{
	//struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	struct mdv20_data *md = dev_get_drvdata(&dssdev->dev);
	u8 buf[80];
	int r;

	buf[0] = 0xb0;
	buf[1] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
		if (r)
			r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
			if (r)
				goto err;

    r = dsi_vc_gen_read(md->dssdev,CMD_VC_CHANNEL, 0xbf, buf, 30);
    if (r<0)
    {
        TOSHIBA_PANEL_DBG(KERN_INFO"%s: Failed to read the device code\n",__func__);
        goto R63306;

    }
    chip_ver = buf[3];
    if(chip_ver ==  CHIP_R63308)
        goto R63308;
R63306:
	buf[0] = 0xb2;
	buf[1] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xb3;
	buf[1] = 0x0c;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xb4;
	buf[1] = 0x02;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xb9;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = 0x75;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,4);
	if (r)
		goto err;

	buf[0] = 0xbb;
	buf[1] = 0x08;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xbe;
	buf[1] = 0xff;
	buf[2] = 0x0f;
	buf[3] = 0x1a;
	buf[4] = 0x18;
	buf[5] = 0x02;
	buf[6] = 0x40;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,7);
	if (r)
		goto err;

	buf[0] = 0xc0;
	buf[1] = 0x40;
	buf[2] = 0x02;
	buf[3] = 0x7f;
	buf[4] = 0xce;
	buf[5] = 0x0e;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,6);
	if (r)
		goto err;

	buf[0] = 0xc1;
	buf[1] = 0x00;
	buf[2] = 0xa8;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = 0x9c;
	buf[9] = 0x08;
	buf[10] = 0x24;
	buf[11] = 0x0b;
	buf[12] = 0x00;
	buf[13] = 0x00;
	buf[14] = 0x00;
	buf[15] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,16);
	if (r)
		goto err;

	buf[0] = 0xc2;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x0b;
	buf[4] = 0x00;
	buf[5] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,6);
	if (r)
		goto err;

	buf[0] = 0xc3;
	buf[1] = 0x04;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xc4;
	buf[1] = 0x4d;
	buf[2] = 0x83;
	buf[3] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,4);
	if (r)
		goto err;

	buf[0] = 0xc6;
	buf[1] = 0x13;
	buf[2] = 0x00;
	buf[3] = 0x08;
	buf[4] = 0x71;
	buf[5] = 0x00;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = 0x00;
	buf[9] = 0x00;
	buf[10] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,11);
	if (r)
		goto err;

	buf[0] = 0xc7;
	buf[1] = 0x22;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xc8;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,5);
	if (r)
		goto err;

	buf[0] = 0xc9;
	buf[1] = 0x43;
	buf[2] = 0x25;
	buf[3] = 0x3b;
	buf[4] = 0x39;
	buf[5] = 0x31;
	buf[6] = 0x23;
	buf[7] = 0x27;
	buf[8] = 0x2c;
	buf[9] = 0x26;
	buf[10] = 0x29;
	buf[11] = 0x40;
	buf[12] = 0x2d;
	buf[13] = 0x74;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,14);
	if (r)
		goto err;

	buf[0] = 0xca;
	buf[1] = 0x7c;
	buf[2] = 0x1a;
	buf[3] = 0x24;
	buf[4] = 0x26;
	buf[5] = 0x2e;
	buf[6] = 0x3c;
	buf[7] = 0x38;
	buf[8] = 0x33;
	buf[9] = 0x39;
	buf[10] = 0x36;
	buf[11] = 0x1f;
	buf[12] = 0x12;
	buf[13] = 0x4b;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,14);
	if (r)
		goto err;

	buf[0] = 0xcb;
	buf[1] = 0x43;
	buf[2] = 0x25;
	buf[3] = 0x3b;
	buf[4] = 0x39;
	buf[5] = 0x31;
	buf[6] = 0x23;
	buf[7] = 0x27;
	buf[8] = 0x2c;
	buf[9] = 0x26;
	buf[10] = 0x29;
	buf[11] = 0x40;
	buf[12] = 0x20;
	buf[13] = 0x74;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,14);
	if (r)
		goto err;

	buf[0] = 0xcc;
	buf[1] = 0x7c;
	buf[2] = 0x1a;
	buf[3] = 0x24;
	buf[4] = 0x26;
	buf[5] = 0x2e;
	buf[6] = 0x3c;
	buf[7] = 0x38;
	buf[8] = 0x33;
	buf[9] = 0x39;
	buf[10] = 0x36;
	buf[11] = 0x1f;
	buf[12] = 0x12;
	buf[13] = 0x4b;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,14);
	if (r)
		goto err;

	buf[0] = 0xcd;
	buf[1] = 0x43;
	buf[2] = 0x25;
	buf[3] = 0x3b;
	buf[4] = 0x39;
	buf[5] = 0x31;
	buf[6] = 0x23;
	buf[7] = 0x27;
	buf[8] = 0x2c;
	buf[9] = 0x26;
	buf[10] = 0x29;
	buf[11] = 0x40;
	buf[12] = 0x20;
	buf[13] = 0x74;

	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,14);
	if (r)
		goto err;

	buf[0] = 0xce;
	buf[1] = 0x7c;
	buf[2] = 0x1a;
	buf[3] = 0x24;
	buf[4] = 0x26;
	buf[5] = 0x2e;
	buf[6] = 0x3c;
	buf[7] = 0x38;
	buf[8] = 0x33;
	buf[9] = 0x39;
	buf[10] = 0x36;
	buf[11] = 0x1f;
	buf[12] = 0x12;
	buf[13] = 0x4b;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,14);
	if (r)
		goto err;

	buf[0] = 0xd0;
	buf[1] = 0x69;
	buf[2] = 0x65;
	buf[3] = 0x01;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,4);
	if (r)
		goto err;

	buf[0] = 0xd1;
	buf[1] = 0x77;
	buf[2] = 0xd4;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,3);
	if (r)
		goto err;

	buf[0] = 0xd3;
	buf[1] = 0x33;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0xd5;
	buf[1] = 0x0F;
	buf[2] = 0x0F;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,3);
	if (r)
		goto err;

	buf[0] = 0xd8;
	buf[1] = 0x34;
	buf[2] = 0x64;
	buf[3] = 0x23;
	buf[4] = 0x25;
	buf[5] = 0x62;
	buf[6] = 0x32;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,7);
	if (r)
		goto err;

	buf[0] = 0xde;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = 0x31;
	buf[4] = 0x03;
	buf[5] = 0x00;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = 0x00;
	buf[9] = 0x00;
	buf[10] = 0x00;
	buf[11] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,12);
	if (r)
		goto err;

	buf[0] = 0xe2;
	buf[1] = 0x00;
	r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,2);
	if (r)
		goto err;

	buf[0] = 0x11;
	r = dsi_vc_dcs_write(md->dssdev, CMD_VC_CHANNEL, buf,1);
	if (r)
		goto err;
	msleep(120);
	
	buf[0] = 0x29;
	r = dsi_vc_dcs_write(md->dssdev, CMD_VC_CHANNEL, buf,1);
	if (r)
		goto err;
	msleep(10);
	
	return 0;


R63308:
    buf[0] = 0xbb;
    buf[1] = 0x0d;	/* color enhancement on, cabc on, hard coded */

    r = dsi_vc_mcs_write(md->dssdev,CMD_VC_CHANNEL, buf,2);
    if (r)
        goto err;

    buf[0] = 0xc6;
    buf[1] = 0x14;
    buf[2] = 0x00;
    buf[3] = 0x08;
    buf[4] = 0x71;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
    buf[8] = 0x00;
    buf[9] = 0x00;
    buf[10] = 0x00;
    r = dsi_vc_mcs_write(md->dssdev, CMD_VC_CHANNEL, buf,11);
    if (r)
	goto err;

    buf[0] = 0xe2;
    buf[1] = 0x01;
    r = dsi_vc_mcs_write(md->dssdev,CMD_VC_CHANNEL, buf,2);
    if (r)
        goto err;

    buf[0] = 0x11;
    r = dsi_vc_mcs_write(md->dssdev,CMD_VC_CHANNEL, buf,1);
    if (r)
        goto err;
    msleep(120);

    buf[0] = 0x29;
    r = dsi_vc_mcs_write(md->dssdev,CMD_VC_CHANNEL, buf,1);
    if (r)
        goto err;
    msleep(10);

    buf[0] = 0xbb;
    buf[1] = 0x0d;

    r = dsi_vc_mcs_write(md->dssdev,CMD_VC_CHANNEL, buf,2);
    if (r)
        goto err;

    buf[0] = 0xbd;
    buf[1] = 0x90;
    buf[2] = 0xc0;
    buf[3] = 0xc0;
    buf[4] = 0xc0;
    buf[5] = 0x90;
    buf[6] = 0x90;
    buf[7] = 0x90;
    buf[8] = 0x90;
    buf[9] = 0x20;
    buf[10] = 0x00;
    buf[11] = 0x80;

    r = dsi_vc_mcs_write(md->dssdev,CMD_VC_CHANNEL, buf,12);
    if (r)
        goto err;

	r = _set_cabc(md->dssdev, CABC_UI);	/* default to UI cabc */
	if(r)
		goto err;

    return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset \n");

	//mdv20_hw_reset(dssdev);//if enable this ,system will locked in 	mutex_lock(&p_dsi->lock); by z00174260
	omapdss_dsi_display_disable(dssdev, true, false);

	return r;
}
EXPORT_SYMBOL(mdv20_config);

static void mdv20_power_off(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
    /*  Reason: Modify for color enhance lcd  */
    struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);	
	//msleep(10);

	mdv20data->enabled = 0;
	omapdss_dsi_display_disable(dssdev, true, false);
    
	if(!dssdev->skip_init)
	{
		/* disable 5.4V */
		if (panel_data->chip_pwr_save != -1)
			gpio_set_value(panel_data->chip_pwr_save,0);
		if (panel_data->v5_4_enable != -1)
			gpio_set_value(panel_data->v5_4_enable,0);
		   msleep(10);
	}

}

static int mdv20_start(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&mdv20data->lock);

	dsi_bus_lock(dssdev);

	r = mdv20_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		dispc_enable_channel(dssdev->channel, dssdev->type, true);
	}

	mutex_unlock(&mdv20data->lock);
	mdv20_queue_esd_work(dssdev);
	return r;
}

static void mdv20_stop(struct omap_dss_device *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	struct toshiba_dsi_panel_data *panel_data = get_panel_data(dssdev);

	mutex_lock(&mdv20data->lock);
	mdv20_cancel_esd_work(dssdev);

	dispc_enable_channel(dssdev->channel, dssdev->type, false);

	dsi_bus_lock(dssdev);

	mdv20_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	mutex_unlock(&mdv20data->lock);

	//udelay(1500);
	//gpio_set_value(panel_data->reset_gpio, 1);	
}

static void mdv20_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		mdv20_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int mdv20_enable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return mdv20_start(dssdev);
}

static void mdv20_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;

	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(dssdev);
}

static int mdv20_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&mdv20data->lock);

	dsi_bus_lock(dssdev);

	if (!mdv20data->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	/* We use VC(0) for VideoPort Data and VC(1) for commands */
	r = omap_dsi_update(dssdev, 0, x, y, w, h, mdv20_framedone_cb, dssdev);
	if (r)
		goto err;

	dsi_bus_unlock(dssdev);
	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&mdv20data->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mdv20data->lock);
	return r;
}

static int mdv20_sync(struct omap_dss_device *dssdev)
{
	/* TODO? */
	return 0;
}

static int mdv20_set_update_mode(struct omap_dss_device *dssdev,
			       enum omap_dss_update_mode mode)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);

	if (mdv20data->force_update) {
		if (mode != OMAP_DSS_UPDATE_AUTO)
			return -EINVAL;
	} else {
		if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	}

	return 0;
}

static enum omap_dss_update_mode mdv20_get_update_mode(struct omap_dss_device
						     *dssdev)
{
	struct mdv20_data *mdv20data = dev_get_drvdata(&dssdev->dev);

	if (mdv20data->force_update)
		return OMAP_DSS_UPDATE_AUTO;
	else
		return OMAP_DSS_UPDATE_MANUAL;
}

#ifdef CONFIG_PM
static int mdv20_resume(struct omap_dss_device *dssdev)
{
	int r=0;
	dev_dbg(&dssdev->dev, "resume\n");

	if(dssdev->skip_init)
		dssdev->skip_init = false;

	r = mdv20_start(dssdev);
    if(r)
    {
        r = mdv20_start(dssdev);
    }

	return r;
}

static int mdv20_suspend(struct omap_dss_device *dssdev)
{	
		/* 0001-Fix-for-supporting-interleaving-sending-the-non-vide.patch */
		u8 buf[80];
		int r =0;

		dev_dbg(&dssdev->dev, "suspend\n");
	if(!dssdev->skip_init)
	{
		/* 0001-Fix-for-supporting-interleaving-sending-the-non-vide.patch */
		dsi_bus_lock(dssdev);
		buf[0] = 0x10;
		omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,1);
		r= dsi_vc_dcs_write_nosync(dssdev, CMD_VC_CHANNEL, buf,1);
		if (r < 0) {
			printk("HARII:: %s, failed !!! \n",__func__);
		}
		omapdss_dsi_vc_enable_lp_cmd_mode(dssdev, 0,0);
		dsi_bus_unlock(dssdev);
	}
		mdv20_stop(dssdev);
		dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	if(boot_first)
	{
	    boot_first = false;
	    u32 mask;
		u32 v;
        mask = CLK_FLAG& CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;
	    v = __raw_readl(OMAP4430_CM_DIV_M5_DPLL_PER);
		/* Clear the bit to allow gatectrl */
		v &= ~mask;
		__raw_writel(v,OMAP4430_CM_DIV_M5_DPLL_PER);
	}
	return 0;
}
#endif

static struct omap_dss_driver mdv20_driver = {
	.probe = mdv20_probe,
	.remove = mdv20_remove,

	.enable = mdv20_enable,
	.disable = mdv20_disable,
#ifdef CONFIG_PM
	.suspend = mdv20_suspend,
	.resume = mdv20_resume,
#endif

	.set_update_mode = mdv20_set_update_mode,
	.get_update_mode = mdv20_get_update_mode,

	.update = mdv20_update,
	.sync = mdv20_sync,

	.get_resolution = mdv20_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	/* dummy entry start */
	.enable_te = mdv20_enable_te,
	.set_rotate = mdv20_rotate,
	.get_rotate = mdv20_get_rotate,
	.set_mirror = mdv20_mirror,
	.get_mirror = mdv20_get_mirror,
	/* dummy entry end */

	.get_timings = mdv20_get_timings,
	.set_timings = mdv20_set_timings,
	.check_timings = mdv20_check_timings,

	.driver = {
		   .name  = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

/*  Reason: lcd-compatible  */
static int __init mdv20_init(void)
{
	int ret;

	TOSHIBA_PANEL_DBG(KERN_INFO"%s \n",__func__);
	if(panel_is_the_panel_supported(mdv20_driver.driver.name))
	{
		TOSHIBA_PANEL_DBG(KERN_INFO"Function(%s): Read mdv20 config sucess!\n",__FUNCTION__);         
		ret = omap_dss_register_driver(&mdv20_driver);
		if (ret < 0) 
		{
			TOSHIBA_PANEL_DBG(KERN_ERR"Function(%s) mdv20 driver regist failed, ret = %d\n",__FUNCTION__,ret);
		}
	}
	else
	{
		TOSHIBA_PANEL_DBG(KERN_INFO"Function(%s) mdv20 panel is not configed\n",__FUNCTION__);
		ret = -1;
	}

	return ret;
}

static void __exit mdv20_exit(void)
{
	omap_dss_unregister_driver(&mdv20_driver);
}

module_init(mdv20_init);
module_exit(mdv20_exit);

MODULE_AUTHOR("Hari Nagalla <hnagalla@ti.com>");
MODULE_DESCRIPTION("MDV20 Driver");
MODULE_LICENSE("GPL");
