/*
 * Samsung DSI command mode panel
 *
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
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>


#include <video/omapdss.h>
#include <plat/huawei-dsi-panel.h>
#include <linux/device.h>
/*  Reason: lcd-compatible  */
#include <linux/panel_detect.h>

#ifdef CONFIG_LCDD
#include "lcdd.h"
#endif
/* END:   Added by meijinfang, 2011/10/24 */

#include <linux/lcd_tuning.h>
/* END:   Added by meijinfang, 2011/12/22 */

/* DSI Virtual channel. Hardcoded for now. */
#define TCH 0

#define DCS_READ_NUM_ERRORS	0x05
#define DCS_READ_POWER_MODE	0x0a
#define DCS_READ_MADCTL		0x0b
#define DCS_READ_PIXEL_FORMAT	0x0c
#define DCS_READ_IMG_MODE		0x0d
#define DCS_READ_SIG_MODE	0x0e
#define DCS_READ_MEM	0x2e
#define DCS_READ_MEM_CONTINUE	0x3e
#define DCS_RDDSDR		0x0f
#define DCS_SLEEP_IN		0x10
#define DCS_SLEEP_OUT		0x11
#define DCS_DISPLAY_OFF		0x28
#define DCS_DISPLAY_ON		0x29
#define DCS_COLUMN_ADDR		0x2a
#define DCS_PAGE_ADDR		0x2b
#define DCS_MEMORY_WRITE	0x2c
#define DCS_TEAR_OFF		0x34
#define DCS_TEAR_ON		0x35
#define DCS_MEM_ACC_CTRL	0x36
#define DCS_PIXEL_FORMAT	0x3a
#define DCS_BRIGHTNESS		0x51
#define DCS_CTRL_DISPLAY	0x53
#define DCS_WRITE_CABC		0x55
#define DCS_READ_CABC		0x56
#define DCS_GET_ID1		0xda
#define DCS_GET_ID2		0xdb
#define DCS_GET_ID3		0xdc
#define PANEL_BLACK_DISPLAY     0x08
#define PANEL_SLEEP_IN          0x10
#define SP_ESD_CHECK_PERIOD	msecs_to_jiffies(5000)

#define PANEL_TAG  "AMOLED: "
//#define PANEL_DEBUG /*调试时打开，入库时关闭-------just used for debug*/
#ifdef PANEL_DEBUG	    
#define PANEL_DBG(fmt, ...) \
	    printk(PANEL_TAG pr_fmt(fmt), ##__VA_ARGS__)	    
#else
#define PANEL_DBG(fmt, ...) do{ \
	}while(0)
#endif

/*Adjust the source scanning direction by register F7H*/
//#define PANEL_INVERT_XY  
/* END:   Modified by meijinfang, 2011/11/22 */

static irqreturn_t sp_te_isr(int irq, void *data);
static void sp_te_timeout_work_callback(struct work_struct *work);
static int _sp_enable_te(struct omap_dss_device *dssdev, bool enable);

static int sp_panel_reset(struct omap_dss_device *dssdev);


/*  Reason: For the can't-wakeup issue.  */
//#define RESUME_SUSPEND_TEST
static int first_failed_times = 0;
static int resume_failed_times = 0;
static int sp_resume(struct omap_dss_device *dssdev);
static int sp_suspend(struct omap_dss_device *dssdev);
static int sp_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h);
/*  Reason: Modify for disorder screen when wakeup  */
static u8 frame_count = 0;
static struct regulator *regulator_sp_panel = NULL;
static int boot_first = true;
static int boot_skip = 5;

/*  Reason: Modify for yellow leftline  */
static int sp_frame_num = 0;
/*reason: add for ACL fouction */
static int acl_switch = false;

struct panel_regulator {
	struct regulator *regulator;
	const char *name;
	int min_uV;
	int max_uV;
};

static int sp_bl_set_locked(struct omap_dss_device *dssdev,int level);

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

		PANEL_DBG("maxiaowei---------init_regulators-----%s\n",regulators[i].name);
		
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
	PANEL_SP,
};
/*  Reason: lcd-compatible  */
static struct panel_regulator power_regulator[] = 
{
	{
		.name = "vaux2",
		.min_uV = 2800000,
		.max_uV = 2800000,
	}
};

static struct panel_config panel_configs[] = {
	{
		.name		= "spanel",
		.type		= PANEL_SP,
		.timings	= {	
			.hsw		= 1,
			.hfp		= 1,
			.hbp		= 1,
			.vsw		= 1,
			.vfp		= 0,
			.vbp		= 0,
			.pixel_clock    = 34028,
			.x_res		= 540,
			.y_res		= 960,
		},
		.width_in_um = 57000,
                .height_in_um = 101000,
		.sleep		= {
			.sleep_in	= 5,
			.sleep_out	= 5,
/*  Reason: Change the reset time  */
			.hw_reset	= 120,
			.enable_te	= 100, /* possible panel bug */
		},
		.reset_sequence	= {
			.high		= 50,//10
			.low		= 25,//10
		},
		.regulators = power_regulator,
		.num_regulators = 1,
	},
};

struct sp_data {
	struct mutex lock;

	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	struct omap_dss_device *dssdev;

	bool enabled;
	u8 rotate;
	bool mirror;

	bool te_enabled;

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;
	
	int channel;
	
	struct delayed_work te_timeout_work;

	bool use_dsi_bl;

	bool cabc_broken;
	unsigned cabc_mode;

	bool intro_printed;

	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
	
	unsigned esd_interval;

	bool ulps_enabled;
	unsigned ulps_timeout;
	struct delayed_work ulps_work;

/*  Reason: Modify for disorder screen when wakeup  */
	struct workqueue_struct *frame_wq;
	struct work_struct frame_work;

	struct panel_config *panel_config;
};

static inline struct sp_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct sp_dsi_panel_data *) dssdev->data;
}

static void sp_esd_work(struct work_struct *work);
static void sp_ulps_work(struct work_struct *work);

/*  Reason: Modify for disorder screen when wakeup  */
static void sp_frame_work(struct work_struct *work);
static void hw_guard_start(struct sp_data *pd, int guard_msec)
{
	pd->hw_guard_wait = msecs_to_jiffies(guard_msec);
	pd->hw_guard_end = jiffies + pd->hw_guard_wait;
}

static void hw_guard_wait(struct sp_data *pd)
{
	unsigned long wait = pd->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= pd->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int sp_dcs_read_1(struct sp_data *pd, u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];

	r = dsi_vc_dcs_read(pd->dssdev, pd->channel, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int sp_dcs_write_0(struct sp_data *pd, u8 dcs_cmd)
{
	return dsi_vc_dcs_write(pd->dssdev, pd->channel, &dcs_cmd, 1);
}

static int sp_dcs_write_1(struct sp_data *pd, u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
}

static int sp_sleep_in(struct sp_data *pd)

{
	u8 cmd;
	int r;

	hw_guard_wait(pd);

	cmd = DCS_SLEEP_IN;
	r = dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, &cmd, 1);
	if (r)
		return r;

	hw_guard_start(pd, 120);

	if (pd->panel_config->sleep.sleep_in)
		msleep(pd->panel_config->sleep.sleep_in);

	return 0;
}

static int sp_sleep_out(struct sp_data *pd)
{
	int r;

	hw_guard_wait(pd);

	r = sp_dcs_write_0(pd, DCS_SLEEP_OUT);
	if (r)
		return r;

	hw_guard_start(pd, 120);

	if (pd->panel_config->sleep.sleep_out)
		msleep(pd->panel_config->sleep.sleep_out);

	return 0;
}

static int sp_get_id(struct sp_data *pd,
	u8 *id1, u8 *id2, u8 *id3)
{
	int r;

	r = sp_dcs_read_1(pd, DCS_GET_ID1, id1);
	if (r)
		return r;
	r = sp_dcs_read_1(pd, DCS_GET_ID2, id2);
	if (r)
		return r;
	r = sp_dcs_read_1(pd, DCS_GET_ID3, id3);
	if (r)
		return r;

	return 0;
}


static int sp_set_update_window(struct sp_data *pd,
	u16 x, u16 y, u16 w, u16 h)
{
	int r = 0;
	
	u16 sp_x = x+30;//wangguanglin 20111017
#ifdef PANEL_INVERT_XY
        u16 sp_y = y+64;
#else
        u16 sp_y = y;        
#endif
	
	u16 x1 = sp_x;
	u16 x2 = sp_x + w - 1;
	u16 y1 = sp_y;
	u16 y2 = sp_y + h - 1;

	u8 buf[5];
	buf[0] = DCS_COLUMN_ADDR;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = DCS_PAGE_ADDR;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, buf, sizeof(buf));
	if (r)
		return r;

	dsi_vc_send_bta_sync(pd->dssdev, pd->channel);

	return r;
}

static void sp_queue_esd_work(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	if (pd->esd_interval > 0)
		queue_delayed_work(pd->esd_wq, &pd->esd_work,
				msecs_to_jiffies(pd->esd_interval));
}

static void sp_cancel_esd_work(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	cancel_delayed_work(&pd->esd_work);
}

static void sp_queue_ulps_work(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	if (pd->ulps_timeout > 0)
		queue_delayed_work(pd->esd_wq, &pd->ulps_work,
				msecs_to_jiffies(pd->ulps_timeout));
}

static void sp_cancel_ulps_work(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	cancel_delayed_work(&pd->ulps_work);
}

static int sp_enter_ulps(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if (pd->ulps_enabled)
		return 0;

	sp_cancel_ulps_work(dssdev);

	r = _sp_enable_te(dssdev, false);
	if (r)
		goto err;

	disable_irq(gpio_to_irq(panel_data->ext_te_gpio));

	omapdss_dsi_display_disable(dssdev, false, true);

	pd->ulps_enabled = true;

	return 0;

err:
	dev_err(&dssdev->dev, "enter ULPS failed");
	sp_panel_reset(dssdev);

	pd->ulps_enabled = false;

	sp_queue_ulps_work(dssdev);

	return r;
}

static int sp_exit_ulps(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if (!pd->ulps_enabled)
		return 0;

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err1;
	}

	omapdss_dsi_vc_enable_hs(dssdev, pd->channel, true);

	r = _sp_enable_te(dssdev, true);
	if (r) {
		dev_err(&dssdev->dev, "failed to re-enable TE");
		goto err2;
	}

	enable_irq(gpio_to_irq(panel_data->ext_te_gpio));

	sp_queue_ulps_work(dssdev);

	pd->ulps_enabled = false;

	return 0;

err2:
	dev_err(&dssdev->dev, "failed to exit ULPS");

	r = sp_panel_reset(dssdev);
	if (!r) {
		enable_irq(gpio_to_irq(panel_data->ext_te_gpio));
		pd->ulps_enabled = false;
	}
err1:
	sp_queue_ulps_work(dssdev);

	return r;
}

static int sp_wake_up(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	if (pd->ulps_enabled)
		return sp_exit_ulps(dssdev);

	sp_cancel_ulps_work(dssdev);
	sp_queue_ulps_work(dssdev);
	return 0;
}


static int sp_bl_set_locked(struct omap_dss_device *dssdev,int level)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int r = 0;
	u8 buf[2] = {0xfa, 0x03};

	int index,range, num_bl_steps;
	u8 bl_data[][26] = {
               {0xfa, 0x02, 0x00, 0x10, 0x48, 0xEB, 0xBC, 0xA1, 0xDC, 0xCD, 
                 0xA3, 0xE4, 0xE5, 0xC3, 0xCB, 0xC9, 0xB0, 0xDA, 0xD6, 0xC2, 
                 0x00, 0x65, 0x00, 0x5F, 0x00, 0x71}, /* 30 */
               {0xfa, 0x02, 0x10, 0x10, 0x49, 0xE8, 0xC6, 0x9D, 0xD7, 0xCF, 
                 0xA3, 0xE0, 0xE6, 0xC8, 0xC3, 0xC6, 0xAB, 0xD5, 0xD5, 0xC1, 
                 0x00, 0x6E, 0x00, 0x67, 0x00, 0x7B}, /* 40 */
               {0xfa, 0x02, 0x24, 0x00, 0x4D, 0xDA, 0xDC, 0x92, 0xCF, 0xD7, 
                 0x9F, 0xDA, 0xE9, 0xCC, 0xBB, 0xC8, 0xA5, 0xCD, 0xD8, 0xBF, 
                 0x00, 0x76, 0x00, 0x6E, 0x00, 0x83}, /* 50 */
               {0xfa, 0x02, 0x2E, 0x01, 0x4D, 0xD4, 0xDE, 0x93, 0xCA, 0xDB, 
                 0xA5, 0xD6, 0xE6, 0xCC, 0xB4, 0xC7, 0xA4, 0xCB, 0xD5, 0xBF, 
                 0x00, 0x7D, 0x00, 0x76, 0x00, 0x8B}, /* 60*/
               /* END:   Added by meijinfang, 2012/3/15 */
               {0xfa, 0x02, 0x38, 0x00, 0x56, 0xc9, 0xdd, 0x94, 0xc2, 0xd9,
                 0xac, 0xd3, 0xe6, 0xce, 0xae, 0xc5, 0x9d, 0xc6, 0xd4, 0xb9,
                 0x00, 0x8a, 0x00, 0x82, 0x00, 0x9f}, /* 70 */
               {0xfa, 0x02, 0x3c, 0x10, 0x57, 0xc6, 0xd3, 0x90, 0xc0, 0xd7, 
                 0xb1, 0xd0, 0xe2, 0xca, 0xab, 0xbf, 0x9b, 0xc4, 0xcf, 0xb8, 
                 0x00, 0x90, 0x00, 0x88, 0x00, 0xa6}, /* 80 */
               {0xfa, 0x02, 0x43, 0x00, 0x57, 0xbd, 0xe1, 0x91, 0xbb, 0xdc,
                 0xb3, 0xcd, 0xe3, 0xca, 0xa7, 0xc4, 0x9b, 0xc0, 0xd1, 0xb5,
                 0x00, 0x95, 0x00, 0x8d, 0x00, 0xad}, /* 90 */
               {0xfa, 0x02, 0x43, 0x00, 0x58, 0xbd, 0xe2, 0x90, 0xbc, 0xde,
                 0xb6, 0xcc, 0xe2, 0xc8, 0xa6, 0xc2, 0x99, 0xc0, 0xd1, 0xb4,
                 0x00, 0x9a, 0x00, 0x92, 0x00, 0xb3}, /* 100*/
               {0xfa, 0x02, 0x47, 0x00, 0x57, 0xb9, 0xe3, 0x93, 0xba, 0xde,
                 0xb9, 0xc9, 0xe2, 0xc9, 0xa4, 0xc1, 0x98, 0xbc, 0xd0, 0xb3,
                 0x00, 0x9f, 0x00, 0x96, 0x00, 0xb9}, /* 110*/
               {0xfa, 0x02, 0x48, 0x00, 0x57, 0xb7, 0xe5, 0x95, 0xb9, 0xde,
                 0xba, 0xca, 0xe2, 0xc8, 0xa2, 0xc0, 0x97, 0xbe, 0xce, 0xb5,
                 0x00, 0xa1, 0x00, 0x9b, 0x00, 0xbc}, /* 120*/
               {0xfa, 0x02, 0x4a, 0x19, 0x58, 0xb6, 0xd7, 0x95, 0xb7, 0xd8,
                 0xba, 0xc8, 0xdb, 0xc6, 0x9f, 0xb7, 0x96, 0xba, 0xc9, 0xb4,
                 0x00, 0xaa, 0x00, 0xa0, 0x00, 0xc3}, /* 130*/
               {0xfa, 0x02, 0x4a, 0x1d, 0x57, 0xb5, 0xd4, 0x98, 0xb6, 0xd7,
                 0xbb, 0xc8, 0xd9, 0xc6, 0xa0, 0xb5, 0x96, 0xb9, 0xc7, 0xb2,
                 0x00, 0xac, 0x00, 0xa3, 0x00, 0xc8}, /* 140*/
               {0xfa, 0x02, 0x4c, 0x1f, 0x57, 0xb3, 0xd5, 0x9c, 0xb4, 0xd6,
                 0xbb, 0xc7, 0xd8, 0xc6, 0x9e, 0xb4, 0x96, 0xb8, 0xc5, 0xaf,
                 0x00, 0xb0, 0x00, 0xa7, 0x00, 0xce}, /* 150*/
               {0xfa, 0x02, 0x4c, 0x26, 0x58, 0xb5, 0xd0, 0x99, 0xb5, 0xd3,
                 0xbb, 0xc7, 0xd6, 0xc5, 0x9c, 0xb0, 0x94, 0xb9, 0xc3, 0xb0,
                 0x00, 0xb4, 0x00, 0xab, 0x00, 0xd2}, /* 160*/
               {0xfa, 0x02, 0x4d, 0x2a, 0x58, 0xb3, 0xce, 0x9c, 0xb4, 0xd2,
                 0xbb, 0xc6, 0xd4, 0xc4, 0x9b, 0xad, 0x93, 0xb9, 0xc1, 0xb1,
                 0x00, 0xb7, 0x00, 0xaf, 0x00, 0xd6}, /* 170*/
               {0xfa, 0x02, 0x4f, 0x28, 0x57, 0xb1, 0xd0, 0x9d, 0xb2, 0xd3,
                 0xbb, 0xc5, 0xd5, 0xc5, 0x9b, 0xad, 0x94, 0xb5, 0xc1, 0xad,
                 0x00, 0xbb, 0x00, 0xb2, 0x00, 0xdc}, /* 180*/
               {0xfa, 0x02, 0x50, 0x2e, 0x58, 0xb0, 0xcd, 0x9d, 0xb2, 0xd0,
                 0xbb, 0xc4, 0xd2, 0xc3, 0x9a, 0xa9, 0x92, 0xb4, 0xc0, 0xae,
                 0x00, 0xbf, 0x00, 0xb6, 0x00, 0xe0}, /* 190*/
               {0xfa, 0x02, 0x4f, 0x31, 0x57, 0xb2, 0xcb, 0xa0, 0xb3, 0xce,
                 0xbb, 0xc4, 0xd1, 0xc4, 0x9b, 0xa9, 0x92, 0xb3, 0xbc, 0xad,
                 0x00, 0xc3, 0x00, 0xba, 0x00, 0xe5}, /* 200*/
               {0xfa, 0x02, 0x4f, 0x34, 0x58, 0xb2, 0xc9, 0x9f, 0xb3, 0xcc,
                 0xba, 0xc4, 0xcf, 0xc3, 0x9a, 0xa7, 0x91, 0xb3, 0xbb, 0xac,
                 0x00, 0xc6, 0x00, 0xbd, 0x00, 0xe9}, /* 210*/
               {0xfa, 0x02, 0x4f, 0x38, 0x57, 0xb3, 0xc6, 0xa3, 0xb4, 0xca,
                 0xbb, 0xc4, 0xce, 0xc3, 0x99, 0xa4, 0x92, 0xb3, 0xb9, 0xab,
                 0x00, 0xc9, 0x00, 0xc0, 0x00, 0xed}, /* 220*/
               {0xfa, 0x02, 0x51, 0x38, 0x58, 0xb0, 0xc8, 0xa3, 0xb1, 0xc9,
                 0xb9, 0xc3, 0xce, 0xc3, 0x98, 0xa4, 0x90, 0xb1, 0xb8, 0xab,
                 0x00, 0xcd, 0x00, 0xc4, 0x00, 0xf1}, /* 230*/
               {0xfa, 0x02, 0x52, 0x39, 0x58, 0xaf, 0xc7, 0xa3, 0xb1, 0xc8,
                 0xba, 0xc3, 0xcd, 0xc3, 0x97, 0xa3, 0x8f, 0xb1, 0xb7, 0xab,
                 0x00, 0xcf, 0x00, 0xc7, 0x00, 0xf4}, /* 240*/
               {0xfa, 0x02, 0x53, 0x3a, 0x57, 0xae, 0xc7, 0xa7, 0xb1, 0xc8,
                 0xba, 0xc2, 0xcd, 0xc2, 0x95, 0xa1, 0x90, 0xb0, 0xb7, 0xa9,
                 0x00, 0xd3, 0x00, 0xca, 0x00, 0xfa}, /* 250*/
               {0xfa, 0x02, 0x52, 0x3c, 0x58, 0xb0, 0xc6, 0xa6, 0xb1, 0xc7,
                 0xb9, 0xc2, 0xcb, 0xc1, 0x96, 0xa0, 0x8f, 0xb0, 0xb6, 0xa9,
                 0x00, 0xd6, 0x00, 0xcd, 0x00, 0xfd}, /* 260*/
               {0xfa, 0x02, 0x53, 0x3d, 0x58, 0xae, 0xc4, 0xa6, 0xb1, 0xc5,
                 0xb9, 0xc2, 0xcb, 0xc1, 0x95, 0xa0, 0x8e, 0xae, 0xb4, 0xa8,
                 0x00, 0xd9, 0x00, 0xd0, 0x01, 0x01}, /* 270*/
               {0xfa, 0x02, 0x55, 0x3e, 0x58, 0xac, 0xc4, 0xa7, 0xaf, 0xc5,
                 0xb8, 0xc1, 0xcb, 0xc1, 0x94, 0x9f, 0x8e, 0xac, 0xb2, 0xa7,
                 0x00, 0xdd, 0x00, 0xd4, 0x01, 0x06}, /* 280*/
               {0xfa, 0x02, 0x53, 0x41, 0x5a, 0xb0, 0xc4, 0xa4, 0xb0, 0xc2,
                 0xb6, 0xc2, 0xca, 0xc1, 0x94, 0x9c, 0x8c, 0xae, 0xb2, 0xa5,
                 0x00, 0xdf, 0x00, 0xd6, 0x01, 0x0a}, /* 290*/
               {0xfa, 0x02, 0x51, 0x39, 0x55, 0xb0, 0xc7, 0xa0, 0xb0, 0xc5,
                 0xb8, 0xc2, 0xcb, 0xc1, 0x94, 0xa0, 0x8f, 0xad, 0xb3, 0xa6,
                 0x00, 0xe0, 0x00, 0xd7, 0x01, 0x08} /* 300*/
               /* END:   Modified by meijinfang, 2012/1/31 */
	};	

       num_bl_steps = (sizeof(bl_data)/sizeof(bl_data[0]));

    if (level < 20)
        level = 20;
    else if (level > 255)
        level = 255;

    level = (level - 20) * 255 / (255 - 20);
    range = 255 / num_bl_steps;
    /* END:   Modified by meijinfang, 2012/3/15 */
        index = level / range;

        if (index >= num_bl_steps)
                index = num_bl_steps - 1;	
//dev_dbg( "%s update brightness to %d\n", __func__,index);
        msleep(5);//add for BTA error
	r = dsi_vc_dcs_write(pd->dssdev, pd->channel, bl_data[index], sizeof(bl_data[index]));	
	if (r)
		goto err;	

	/* GAMMA set update enable */
	r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, sizeof(buf));
	msleep(5);//add for BTA error
	if (r)
			goto err;
	 
	return 0;

err:
        PANEL_DBG("Failed to set AMOLED BL !!! \n"); 
        
	return r;
}
static int sp_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;
	int level;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;

	dev_dbg(&dssdev->dev, "update brightness to %d\n", level);

	mutex_lock(&pd->lock);

	if (pd->use_dsi_bl) {
		if (pd->enabled) {
			dsi_bus_lock(dssdev);
			r = sp_bl_set_locked(dssdev,level);
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

	mutex_unlock(&pd->lock);

	return r;
}

static int sp_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static struct backlight_ops sp_bl_ops = {
	.get_brightness = sp_bl_get_intensity,
	.update_status  = sp_bl_update_status,
};

static void sp_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void sp_set_timings(struct omap_dss_device *dssdev,
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

static int sp_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	if (timings->x_res != 540 || timings->y_res != 960)
		return -EINVAL;

	return 0;
}

static void sp_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	if (pd->rotate == 0 || pd->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static ssize_t sp_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	u8 errors;
	int r;

	PANEL_DBG("HARII : %s \n",__func__);

	mutex_lock(&pd->lock);

	if (pd->enabled) {
		dsi_bus_lock(dssdev);
		r = sp_dcs_read_1(pd, DCS_READ_NUM_ERRORS, &errors);
		dsi_bus_unlock(dssdev);
		PANEL_DBG("HARII : %s, %d \n",__func__,errors);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&pd->lock);

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%d\n", errors);
}

static ssize_t sp_hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	u8 id1, id2, id3;
	int r;

	mutex_lock(&pd->lock);

	if (pd->enabled) {
		dsi_bus_lock(dssdev);
		r = sp_get_id(pd, &id1, &id2, &id3);
		dsi_bus_unlock(dssdev);
		PANEL_DBG("HARII:: %s, %d, %d, %d \n",__func__,id1,id2,id3);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&pd->lock);

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%02x.%02x.%02x\n", id1, id2, id3);
}

static const char *cabc_modes[] = {
	"off",		/* used also always when CABC is not supported */
	"ui",
	"still-image",
	"moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	const char *mode_str;
	int mode;
	int len;

	mode = pd->cabc_mode;

	mode_str = "unknown";
	if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
		mode_str = cabc_modes[mode];
	len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t store_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
		if (sysfs_streq(cabc_modes[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(cabc_modes))
		return -EINVAL;

	mutex_lock(&pd->lock);

	if (pd->enabled) {
		dsi_bus_lock(dssdev);
		/*
		if (!pd->cabc_broken)
			taal_dcs_write_1(ix, DCS_WRITE_CABC, i); */
		dsi_bus_unlock(dssdev);
	}

	pd->cabc_mode = i;

	mutex_unlock(&pd->lock);

	return count;
}

static ssize_t show_cabc_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	int i;

	for (i = 0, len = 0;
	     len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
		len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
			i ? " " : "", cabc_modes[i],
			i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

	return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static ssize_t sp_store_esd_interval(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&pd->lock);
	sp_cancel_esd_work(dssdev);
	pd->esd_interval = t;
	if (pd->enabled)
		sp_queue_esd_work(dssdev);
	mutex_unlock(&pd->lock);

	return count;
}

static ssize_t sp_show_esd_interval(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&pd->lock);
	t = pd->esd_interval;
	mutex_unlock(&pd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t sp_store_ulps(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&pd->lock);

	if (pd->enabled) {
		dsi_bus_lock(dssdev);

		if (t)
			r = sp_enter_ulps(dssdev);
		else
			r = sp_wake_up(dssdev);

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&pd->lock);

	if (r)
		return r;

	return count;
}

static ssize_t sp_show_ulps(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&pd->lock);
	t = pd->ulps_enabled;
	mutex_unlock(&pd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t sp_store_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&pd->lock);
	pd->ulps_timeout = t;

	if (pd->enabled) {
		/* taal_wake_up will restart the timer */
		dsi_bus_lock(dssdev);
		r = sp_wake_up(dssdev);
		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&pd->lock);

	if (r)
		return r;

	return count;
}

static ssize_t sp_show_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&pd->lock);
	t = pd->ulps_timeout;
	mutex_unlock(&pd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}



/*  Reason: For the can't-wakeup issue.  */
#ifdef RESUME_SUSPEND_TEST
static ssize_t show_test1(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    int i,r;
    struct omap_dss_device *dssdev = to_dss_device(dev);
    first_failed_times = 0;
    resume_failed_times = 0;
    for (i = 0; i < 1000; i++)
    {
        dev_dbg(&dssdev->dev,"^-^^-^HT: Test1 for %d times\n",i+1);
        r = sp_resume(dssdev);
        r |= sp_suspend(dssdev);

        if(r)
        {
            dev_err(&dssdev->dev,"^-^^-^HT: Entry %s and test1 failed! \n",__FUNCTION__);
            break;
        }
    }

    return 0;
}

static ssize_t show_test2(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    struct omap_dss_device *dssdev = to_dss_device(dev);
    dev_dbg(&dssdev->dev,"^-^^-^HT: Entry %s first_failed_times = %d .\n",__FUNCTION__,first_failed_times);
    dev_dbg(&dssdev->dev,"^-^^-^HT: Entry %s resume_failed_times = %d .\n",__FUNCTION__,resume_failed_times);
    return 0;
}
#endif

static ssize_t show_lcd_info(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    char * lcd_info = "Samsung amoled 4.3' LCD.";
    strncpy(buf, lcd_info, strlen(lcd_info)+1);
    return strlen(lcd_info)+1;
}

/*Gamma high light(gamma1.9/200)*/
static char samsung_s6e39a_gamma19_set[26] = {0xfa, 0x02, 0x4f, 0x31, 0x57, 0xb5, 
                    0xcf, 0xa5, 0xbc, 0xd1, 0xc2, 0xc8, 0xd2, 0xc6, 0xa1, 0xad, 0x9a, 
                    0xbc, 0xc5, 0xb5, 0x00, 0xc3, 0x00, 0xba, 0x00, 0xe5};
/* END:   Modified by meijinfang, 2012/1/31 */

/*Gamma update*/
static char samsung_s6e39a_gamma_update[2] = {0xfa, 0x03};

/*Record the state of the panel, it's forbidden to write to the IC when the panel is not active.*/
static int g_suspendstate = false;

int sp_set_gamma(struct lcd_tuning_dev *ltd, enum lcd_gamma gamma)
{
    int r = 0;
	int brightness;
	
	struct omap_dss_device *pdev = (struct omap_dss_device *)(ltd->data);
	struct sp_data *pd = dev_get_drvdata(&pdev->dev);
	struct backlight_device *bldev;

    if ((gamma != GAMMA22)
        && (gamma != GAMMA19))
    {
        return -1;
    }

    if (true == g_suspendstate)
    {
        return 0;
    }

    dsi_bus_lock(pdev);

    if (GAMMA19 == gamma)
    {
        r = dsi_vc_dcs_write(pd->dssdev, TCH, samsung_s6e39a_gamma19_set, 
                             ARRAY_SIZE(samsung_s6e39a_gamma19_set));
        if (r)
            goto err;
        
        r = dsi_vc_dcs_write(pd->dssdev, TCH, samsung_s6e39a_gamma_update, 
                             ARRAY_SIZE(samsung_s6e39a_gamma_update));
        if (r)
            goto err;
    }
    else
    {
        bldev = pd->bldev;
        brightness = bldev->props.brightness;

        /* Gamma / set backlight */
        r = sp_bl_set_locked(pdev, brightness);
    }

    dsi_bus_unlock(pdev);
    return 0;
    
    err:
        dsi_bus_unlock(pdev);
        return r;
}
/* END:   Added by meijinfang, 2011/12/22 */

/*reason: add for ACL fouction */
int sp_set_acl (struct lcd_tuning_dev *ltd, enum amoled_acl acl)
{
    int r = 0;
	struct omap_dss_device *pdev = (struct omap_dss_device *)(ltd->data);
	struct sp_data *pd = dev_get_drvdata(&pdev->dev);
    u8 buf[30];
	if ((acl != ACL_ON)
        && (acl != ACL_OFF))
    {
        return -1;
	}

	if(acl == ACL_ON)
	{
	    acl_switch = true;
	}
	else if(acl == ACL_OFF)
	{
	    acl_switch = false;
	}
	
    if (true == g_suspendstate)
    {
        return 0;
    }

    dsi_bus_lock(pdev);
    if(acl == ACL_ON)			
	{		
		buf[0]  = 0xc0;			
		buf[1]  = 0x01;              
	    r=dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);

        if (r)
            goto err0;
	}		
	else if(acl == ACL_OFF)			
	{    			
		buf[0]  = 0xc0;			
		buf[1]  = 0x00;                   
		r=dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
        if (r)
            goto err0;
	 }
	
	dsi_bus_unlock(pdev);
	return 0;
	  
err0:
	dsi_bus_unlock(pdev);
	return r;
}
/*
 *gram window register access func
 *           sc ec  sp ep
 *usage:echo 30 569 0 959 > /sys/bus/omapdss/devices/display0/window
 */
static ssize_t store_window(struct device *dev,
		struct device_attribute *attr,
		char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int sc,ec,sp,ep;
    s32 r;
    u8 reg_buf[10];

    if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
        return count;
    printk("%s enter %s\n",__func__,buf);

	r = sscanf(buf,"%d %d %d %d",&sc,&ec,&sp,&ep);
    printk("0x%04x 0x%04x 0x%04x 0x%04x\n",sc,ec,sp,ep);


    mutex_lock(&pd->lock);
    dsi_bus_lock(dssdev);

    reg_buf[0]  = 0x2a;
    reg_buf[1]  = (u8)(sc>>8);
    reg_buf[2]  = (u8)(sc&0xff);
    reg_buf[3]  = (u8)(ec>>8);
    reg_buf[4]  = (u8)(ec&0xff);
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  reg_buf, 5);
    if (r)
    {
        printk("window1 set failed\n");
    }

    reg_buf[0]  = 0x2b;
    reg_buf[1]  = (u8)(sp>>8);
    reg_buf[2]  = (u8)(sp&0xff);
    reg_buf[3]  = (u8)(ep>>8);
    reg_buf[4]  = (u8)(ep&0xff);

    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  reg_buf, 5);
    if(r)
    {
        printk("window2 set failed\n");
    }

    dsi_bus_unlock(dssdev);
    mutex_unlock(&pd->lock);
    printk("%s out\n",__func__);

    return count;
}
static DEVICE_ATTR(window, S_IRUGO| S_IWUSR, NULL, store_window);

/*
 *get dcs readable register val
 *this func is used to cat all the readable registers
 *usage:cat /sys/bus/omapdss/devices/display0/reg
 */
static ssize_t show_reg_read_all(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
    u8 pwr_mode,madctl,img_mode,sig_mode;
    s32 r;

    if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
        return 0;

    printk("%s enter\n",__func__);
    mutex_lock(&pd->lock);
    dsi_bus_lock(dssdev);

    r = sp_dcs_read_1(pd, DCS_READ_POWER_MODE, &pwr_mode);
    if(r)
    {
        printk("pwr_mode read failed\n");
    }

    r = sp_dcs_read_1(pd, DCS_READ_MADCTL, &madctl);
    if(r)
    {
        printk("madctl read failed\n");
    }
    r = sp_dcs_read_1(pd, DCS_READ_IMG_MODE, &img_mode);
    if(r)
    {
        printk("img_mode read failed\n");
    }
    r = sp_dcs_read_1(pd, DCS_READ_SIG_MODE, &sig_mode);
    if(r)
    {
        printk("sig_mode read failed\n");
    }
    dsi_bus_unlock(dssdev);
    mutex_unlock(&pd->lock);
    printk("%s out\n",__func__);

    r = sprintf(buf,"pwr_mode:%02x madctl:%02x img_mode:%02x sig_mode:%02x\n",pwr_mode,madctl,img_mode,sig_mode);
	return r;
}

/*
 *dcs register write func
 *be carefull use this funciton.Some registers are read only,and some
 *registers can be accessed in special mode.Write these registers may
 *cause hardware error.
 *usage:if write 0x00 to MADCTL reg(0x36)
 *echo 0x36 0x00 > /sys/bus/omapdss/devices/display0/reg
 */
static ssize_t show_reg_store(struct device *dev,
		struct device_attribute *attr,
		char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
    u8 reg_buf[30];
    u8 reg_count = 0;
    char *p;
	char buf_tmp[5];

    if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
        return count;

    p=buf;

    while(*p != '\0')
    {
        if((*(p+1) == 'x')&&(*p == '0'))
        {
            memset(buf_tmp, 0x0, sizeof(buf_tmp));
            memcpy(buf_tmp, p, 4);
            reg_buf[reg_count] = simple_strtol(buf_tmp, NULL, 16);
            reg_count++;
        }
        p++;
    }

    mutex_lock(&pd->lock);
    dsi_bus_lock(dssdev);
    dsi_vc_dcs_write(pd->dssdev, pd->channel, reg_buf, reg_count);
    dsi_bus_unlock(dssdev);
    mutex_unlock(&pd->lock);
	return count;
}

static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, show_reg_read_all, show_reg_store);

#if 0
#define QHD_MEM_SIZE 960*540*3
static int mem_read_x = 0;
static int mem_read_y = 0;
static int mem_read_w = 100;
static int mem_read_h = 100;

/*
 *gram read func.
 *WARN:be carefull use this funciton,mem window can't be too large, or system will block.
 *usage:if want to get gram data(window is x y w h)
 *echo x y w h > /sys/bus/omapdss/devices/display0/ram
 *cat /sys/bus/omapdss/devices/display0/ram > /data/gram.raw
 *return size of the gram = w*h*3
 */
static ssize_t show_ram_read(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
    s32 r;
    ssize_t count = 0;
    s32 i;

    if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
        return 0;
    printk("%s enter\n",__func__);

    count = sp_memory_read(dssdev,buf,QHD_MEM_SIZE,mem_read_x,mem_read_y,mem_read_w,mem_read_h);

    /*show 60 bytes to kernel log*/
    for(i=0;i<60;i+=6)
    {
        printk("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",buf[i],buf[i+1],buf[i+2],buf[i+3],buf[i+4],buf[i+5]);
    }

    printk("%s out. get data size %d\n",__func__,count);
	return count;
}

/*
 *set window for gram read function.
 *WARN:be carefull use this funciton,mem window can't be too large, or system will block.
 *usage:if want to get gram data(window is x y w h)
 *echo x y w h > /sys/bus/omapdss/devices/display0/ram
 *cat /sys/bus/omapdss/devices/display0/ram > /data/gram.raw
 */
static ssize_t show_ram_store(struct device *dev,
		struct device_attribute *attr,
		char *buf, size_t count)
{
	sscanf(buf, "%d %d %d %d", &mem_read_x,&mem_read_y,&mem_read_w,&mem_read_h);

    if(mem_read_x>=540)
        mem_read_x = 539;
    if(mem_read_y>=960)
        mem_read_y = 959;
    if(mem_read_w>=540)
        mem_read_w = 539;
    if(mem_read_h>=960)
        mem_read_h = 959;
    printk("x:%d y:%d w:%d h:%d\n",mem_read_x,mem_read_y,mem_read_w,mem_read_h);
	return count;
}

static DEVICE_ATTR(mem, S_IRUGO | S_IWUSR, show_ram_read, show_ram_store);
#endif

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, sp_num_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, sp_hw_revision_show, NULL);
static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
		show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
		show_cabc_available_modes, NULL);
static DEVICE_ATTR(esd_interval, S_IRUGO | S_IWUSR,
		sp_show_esd_interval, sp_store_esd_interval);
static DEVICE_ATTR(ulps, S_IRUGO | S_IWUSR,
		sp_show_ulps, sp_store_ulps);
static DEVICE_ATTR(ulps_timeout, S_IRUGO | S_IWUSR,
		sp_show_ulps_timeout, sp_store_ulps_timeout);

#ifdef RESUME_SUSPEND_TEST
static DEVICE_ATTR(test1, S_IRUGO,
		show_test1, NULL);

static DEVICE_ATTR(test2, S_IRUGO,
		show_test2, NULL);
#endif

static DEVICE_ATTR(lcd_info, S_IRUGO,
		show_lcd_info, NULL);


static struct attribute *sp_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	&dev_attr_esd_interval.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,

#ifdef RESUME_SUSPEND_TEST
	&dev_attr_test1.attr,
	&dev_attr_test2.attr,
#endif
	&dev_attr_lcd_info.attr,
	&dev_attr_window.attr,
	&dev_attr_reg.attr,
	NULL,
};



static struct attribute_group sp_attr_group = {
	.attrs = sp_attrs,
};

static void sp_hw_reset(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (panel_data->reset_gpio == -1)
		return;
		
	PANEL_DBG("__________________sp_hw_reset____________________\n");
	
	gpio_set_value(panel_data->reset_gpio, 1);
	if (pd->panel_config->reset_sequence.high)
		udelay(pd->panel_config->reset_sequence.high);
	/* reset the panel */
	gpio_set_value(panel_data->reset_gpio, 0);
	/* assert reset */
	if (pd->panel_config->reset_sequence.low)
		udelay(pd->panel_config->reset_sequence.low);
	gpio_set_value(panel_data->reset_gpio, 1);
	/* wait after releasing reset */
	if (pd->panel_config->sleep.hw_reset)
		msleep(pd->panel_config->sleep.hw_reset);
}

static int sp_init_regulator(struct device *dev)
{
	/*Get regulator, don't set voltage manually*/
	if(!regulator_sp_panel)
	{
		regulator_sp_panel = get_consumer_regulator(dev, "LCD_LCD_VCI");
		if (IS_ERR(regulator_sp_panel))
		{
			printk(KERN_ERR"get PW for dev error");
			regulator_sp_panel = NULL;
			return -1;
		}
	}

	consumer_regulator_enable(regulator_sp_panel);

	return 0;
}

static void sp_uninit_regulator(void)
{
	if(regulator_sp_panel)
	{
		consumer_regulator_disable(regulator_sp_panel);
		put_consumer_regulator(regulator_sp_panel);
		regulator_sp_panel = NULL;
	}
}

static struct lcd_tuning_ops sp_tuning_ops = {
	.set_gamma = sp_set_gamma,
/*reason: add for ACL fouction */
	.set_acl = sp_set_acl,
};
/* END:   Added by meijinfang, 2011/12/22 */

static int sp_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct sp_data *pd;
	struct backlight_device *bldev;
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct panel_config *panel_config = NULL;
	struct lcd_tuning_dev *ltd;
	struct lcd_properities lcd_props;
	/* END:   Added by meijinfang, 2011/12/22 */
	int r, i;

	dev_dbg(&dssdev->dev, "probe\n");
	
	PANEL_DBG("----probe1----\n");

	if (!panel_data || !panel_data->name) {
		r = -EINVAL;
		goto err;
	}
	PANEL_DBG("----probe2----\n");

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		r = -EINVAL;
		goto err;
	}
	PANEL_DBG("----probe3----\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = panel_config->timings;
	dssdev->panel.width_in_um = panel_config->width_in_um;
	dssdev->panel.height_in_um = panel_config->height_in_um;
	dssdev->ctrl.pixel_size = 24; 

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		r = -ENOMEM;
		goto err;
	}
	PANEL_DBG("----probe4----\n");
	pd->dssdev = dssdev;
	pd->panel_config = panel_config;
	pd->esd_interval = panel_data->esd_interval;
	pd->ulps_enabled = false;
	pd->ulps_timeout = panel_data->ulps_timeout;

	mutex_init(&pd->lock);

	atomic_set(&pd->do_update, 0);

#if 0
	r = init_regulators(dssdev, panel_config->regulators,
			panel_config->num_regulators);
	if (r)
		goto err_reg;
#else
	r = sp_init_regulator(&dssdev->dev);
	if (r)
		goto err_reg;
#endif

	pd->esd_wq = create_singlethread_workqueue("sp_esd");
    /*  Reason: Modify for disorder screen when wakeup  */
    pd->frame_wq = create_singlethread_workqueue("sp_frame");
	if (pd->esd_wq == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err_wq;
	}
	PANEL_DBG("----probe6----\n");
	INIT_DELAYED_WORK_DEFERRABLE(&pd->esd_work, sp_esd_work);
	INIT_DELAYED_WORK(&pd->ulps_work, sp_ulps_work);
    /*  Reason: Modify for disorder screen when wakeup  */
    INIT_WORK(&pd->frame_work, sp_frame_work);


	dev_set_drvdata(&dssdev->dev, pd);

	msleep(25);
	sp_hw_reset(dssdev);

	/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	memset(&props, 0, sizeof(struct backlight_properties));
	if (!panel_data->set_backlight)
		pd->use_dsi_bl = true;

	if (pd->use_dsi_bl)
		props.max_brightness = 255;
	else
		props.max_brightness = 127;

		/*
		bldev = backlight_device_register(dev_name(&dssdev->dev),
			&dssdev->dev, dssdev, &sp_bl_ops, &props);
		*/
		PANEL_DBG("--->Lintar dssdev->name =%s dev_name =%s\n",dssdev->name,dev_name(&dssdev->dev));
	props.type = BACKLIGHT_RAW;
	        bldev = backlight_device_register(dssdev->name,
			&dssdev->dev, dssdev, &sp_bl_ops, &props);


	if (IS_ERR(bldev)) {
		r = PTR_ERR(bldev);
		goto err_bl;
	}
	PANEL_DBG("----probe7----\n");
	pd->bldev = bldev;

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.power = FB_BLANK_UNBLANK;
	if (pd->use_dsi_bl)
		bldev->props.brightness = 255;
	else
		bldev->props.brightness = 127;

	sp_bl_update_status(bldev);

    lcd_props.type = OLED;
    lcd_props.default_gamma = GAMMA22;
    /*reason: add for ACL fouction */
    lcd_props.default_aclsetting = SUPPORT_ACL;
    ltd = lcd_tuning_dev_register(&lcd_props, &sp_tuning_ops, (void *)dssdev);
	if (IS_ERR(ltd)) {
		r = PTR_ERR(ltd);
		goto err_gpio;
	}
	/* END:   Added by meijinfang, 2011/12/22 */

	if (panel_data->use_ext_te) {
		int gpio = panel_data->ext_te_gpio;

		r = gpio_request(gpio, "sp irq");
		if (r) {
			dev_err(&dssdev->dev, "GPIO request failed\n");
			goto err_gpio;
		}
		PANEL_DBG("----probe71----\n");
		gpio_direction_input(gpio);

		if (dssdev->channel == OMAP_DSS_CHANNEL_LCD) {
			r = request_irq(gpio_to_irq(gpio), sp_te_isr,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"sp vsync", dssdev);
		}/* else {
			r = request_irq(gpio_to_irq(gpio), sp_te_isr2,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"sp vsync2", dssdev);
		}*/
		if (r) {
			dev_err(&dssdev->dev, "IRQ request failed\n");
			gpio_free(gpio);
			goto err_irq;
		}
		PANEL_DBG("----probe72----\n");

		INIT_DELAYED_WORK_DEFERRABLE(&pd->te_timeout_work,
				sp_te_timeout_work_callback);

		dev_dbg(&dssdev->dev, "Using GPIO TE\n");
	}
	
	r = omap_dsi_request_vc(dssdev, &pd->channel);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel\n");
		goto err_req_vc;
	}

	r = omap_dsi_set_vc_id(dssdev, pd->channel, TCH);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID\n");
		goto err_vc_id;
	}
	
	PANEL_DBG("----probe8----\n");
	r = sysfs_create_group(&dssdev->dev.kobj, &sp_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		goto err_vc_id;
	}
	PANEL_DBG("----probe9----\n");
	return 0;
	
err_vc_id:
        omap_dsi_release_vc(dssdev, pd->channel);
err_req_vc:
	if (panel_data->use_ext_te)
		free_irq(gpio_to_irq(panel_data->ext_te_gpio), dssdev);
err_irq:
	if (panel_data->use_ext_te)
		gpio_free(panel_data->ext_te_gpio);
err_gpio:
	backlight_device_unregister(bldev);
err_bl:
	destroy_workqueue(pd->esd_wq);
err_wq:
	//free_regulators(panel_config->regulators, panel_config->num_regulators);
err_reg:
	kfree(pd);
err:
	return r;
}

static void sp_remove(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &sp_attr_group);

	if (panel_data->use_ext_te) {
		int gpio = panel_data->ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}

	bldev = pd->bldev;
	bldev->props.power = FB_BLANK_POWERDOWN;
	sp_bl_update_status(bldev);
	backlight_device_unregister(bldev);

	cancel_delayed_work(&pd->esd_work);
	destroy_workqueue(pd->esd_wq);

    /*  Reason: Modify for disorder screen when wakeup  */
	cancel_work_sync(&pd->frame_work);
	destroy_workqueue(pd->frame_wq);

	/* reset, to be sure that the panel is in a valid state */
	sp_hw_reset(dssdev);

	//free_regulators(pd->panel_config->regulators,
			//pd->panel_config->num_regulators);

	kfree(pd);
}

/*Default lcd config*/
static int samsung_lcd_config(struct omap_dss_device *dssdev)
{
    u8 buf[30];
    int r;
    struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

    
    /* Etc Condition Set1 */    
    buf[0] = 0xF0;
    buf[1] = 0x5A;
    buf[2] = 0x5A;

    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 3);

    /*
     * Work-around for power-up stability issue where the first DCS
     * command sent after power-up fails due to glitches on MIPI bus. If
     * the command fails, retry once
     */
    if (r) {
		dev_err(&dssdev->dev,"^-^^-^HT:(%s) Write f0 failed first time!\n",__FUNCTION__);
		first_failed_times++;
        r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 3);
        if (r)
            goto err;
    }

    buf[0] = 0xF1;
    buf[1] = 0x5A;
    buf[2] = 0x5A;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 3);
    if (r)
        goto err;

    buf[0] = 0xFC;
    buf[1] = 0x5A;
    buf[2] = 0x5A;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 3);
    if (r)
        goto err;

    /* Gamma / set backlight */
    r = sp_bl_set_locked(dssdev,300);

    /* Panel Condition Set */
    buf[0]  = 0xF8;
    buf[1]  = 0x27;
    buf[2]  = 0x27;
    buf[3]  = 0x08;
    buf[4]  = 0x08;
    buf[5]  = 0x4e;
    buf[6]  = 0xaa;
    buf[7]  = 0x5e;
    buf[8]  = 0x8a;
    buf[9]  = 0x10;
    buf[10] = 0x3f;
    buf[11] = 0x10;
    buf[12] = 0x10;
    buf[13] = 0x00;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 14);

    if (r)
        goto err;

    /* ETc Condition Set 2  */
    buf[0]  = 0xF6;
    buf[1]  = 0x00;
    buf[2]  = 0x84;
    buf[3]  = 0x09;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 4);
    if (r)
        goto err;

    buf[0]  = 0xb0;

    buf[1]  = 0x09;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if (r)
        goto err;


    buf[0] = 0xd5;
    buf[1] = 0x64;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);

    if (r)
        goto err;

    buf[0]  = 0xb0;
    buf[1]  = 0x0b;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if (r)
        goto err;


    buf[0] = 0xd5;
    buf[1] = 0xa4;
    buf[2] = 0x7e;
    buf[3] = 0x20;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 4);
    if (r)
        goto err;

    buf[0]  = 0xf7;
    buf[1]  = 0x03;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
    if (r)
        goto err;

    buf[0]  = 0xb0;
    buf[1]  = 0x02;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
    if (r)
        goto err;

    buf[0]  = 0xb3;
    buf[1]  = 0xc3;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);

    if (r)
        goto err;
    /* END:   Added by meijinfang, 2011/11/22 */

    buf[0]  = 0xb0;
    buf[1]  = 0x08;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if (r)
        goto err;

    buf[0]  = 0xfd;
    buf[1]  = 0xf8;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if (r)
        goto err;
        
    buf[0]  = 0xb0;
    buf[1]  = 0x04;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if (r)
        goto err;
    
    buf[0]  = 0xf2;
    buf[1]  = 0x4d;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if (r)
        goto err;

    buf[0]  = 0xb0;
    buf[1]  = 0x05;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
    if (r)
        goto err;

    buf[0]  = 0xfd;
    buf[1]  = 0x1f;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
    if (r)
        goto err;

    buf[0] = 0xB1;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x16;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 4);
	if (r)
		goto err;

    buf[0] = 0xB2;
    buf[1] = 0x06;
    buf[2] = 0x06;
    buf[3] = 0x06;
    buf[4] = 0x06;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 5);
	if (r)
		goto err;

    /*sleep out*/
    buf[0]  = 0x11;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
    if (r)
        goto err;

    mdelay(200);//120
    /* END:   Modified by meijinfang, 2011/11/22 */

/*reason: add for ACL fouction to mactch the standerd sequence*/
	#if 0
    /*TE ON*/
    buf[0]  = 0x35;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
	
    if (r)
        goto err;
    /* END:   Added by meijinfang, 2011/11/22 */
   #endif

#ifdef PANEL_INVERT_XY
    buf[0] = 0x36;
    buf[1] = 0xc0;
#else
    buf[0] = 0x36;
    buf[1] = 0x00;
#endif
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
    if (r)
        goto err;
    
    /*reason: add for ACL fouction to mactch the standerd sequence*/
    /*TE ON*/
    buf[0]  = 0x35;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
    if (r)
        goto err;
    /* memory window setting1 */
    buf[0]  = 0x2a;
    buf[1]  = 0x00;
    buf[2]  = 0x1e;
    buf[3]  = 0x02;
    buf[4]  = 0x39;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 5);
    if (r)
        goto err;

    buf[0]  = 0x2b;

#ifdef PANEL_INVERT_XY
    buf[1]  = 0x00;
    buf[2]  = 0x40;
    buf[3]  = 0x03;
    buf[4]  = 0xff;
#else
    buf[1]  = 0x00;
       buf[2]  = 0x00;
    buf[3]  = 0x03;
    buf[4]  = 0xbf;
#endif
    
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 5);
    if (r)
        goto err;

    /*VINIT : 2.0V*/
    buf[0]  = 0xd1;
    buf[1]  = 0x8a;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 2);
    if (r)
        goto err;

    /*ACL ON*/
    buf[0]  = 0xc0;
	if(true == acl_switch)
	{
	    buf[1]  = 0x01;
	}
	else
	{
	    buf[1]  = 0x00;
	}
    buf[2]  = 0x00;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 3);
	if (r)
		goto err;
//    #if 0
    /*ACL setting 70%*/
    buf[0]  = 0xc1;
    buf[1]  = 0x47;
    buf[2]  = 0x53;
    buf[3]  = 0x13;
    buf[4]  = 0x53;
    buf[5]  = 0x00;
    buf[6]  = 0x00;
    buf[7]  = 0x01;
    buf[8]  = 0xdf;
    buf[9]  = 0x00;
    buf[10]  = 0x00;
    buf[11]  = 0x03;
    buf[12]  = 0x1f;
    buf[13]  = 0x00;
    buf[14]  = 0x00;
    buf[15]  = 0x00;
    buf[16]  = 0x00;
    buf[17]  = 0x00;
    buf[18]  = 0x01;
    buf[19]  = 0x02;
    buf[20]  = 0x03;
    buf[21]  = 0x07;
    buf[22]  = 0x0e;
    buf[23]  = 0x14;
    buf[24]  = 0x1c;
    buf[25]  = 0x24;
    buf[26]  = 0x2d;
    buf[27]  = 0x2d;
    buf[28]  = 0x00;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 29);
	if (r)
		goto err;
    /* END:   Added by meijinfang, 2011/11/22 */
//    #endif
    /* END:   Modified by meijinfang, 2012/1/5 */

/*  Reason:Remove the pixel off action and do it in resume for yellow leftline  */

    udelay(32);
    /* Display on */
    buf[0]  = DCS_DISPLAY_ON;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 1);
    if (r)
        goto err;

    return 0;

    err:
        return r;
}
static int samsung_lcd_convert_window(struct omap_dss_device *dssdev)
{
	u8 buf[8];
	int r;

	#ifdef PANEL_INVERT_XY
	buf[0] = 0x36;
	buf[1] = 0xc0;
	#else
	buf[0] = 0x36;
	buf[1] = 0x00;
	#endif
	r = dsi_vc_dcs_write(dssdev, dssdev->channel, buf, 2);
	if (r)
		goto err;

	/* memory window setting1 */
	buf[0]  = 0x2a;
	buf[1]  = 0x00;
	buf[2]  = 0x1e;
	buf[3]  = 0x02;
	buf[4]  = 0x39;
	r = dsi_vc_dcs_write(dssdev, dssdev->channel, buf, 5);
	if (r)
		goto err;

	buf[0]  = 0x2b;
	#ifdef PANEL_INVERT_XY
	buf[1]  = 0x00;
	buf[2]  = 0x40;
	buf[3]  = 0x03;
	buf[4]  = 0xff;
	#else
	buf[1]  = 0x00;
	buf[2]  = 0x00;
	buf[3]  = 0x03;
	buf[4]  = 0xbf;
	#endif
	r = dsi_vc_dcs_write(dssdev, dssdev->channel, buf, 5);
	if (r)
		goto err;

	return 0;

err:
	return r;
}

#ifdef CONFIG_LCDD
/*Dynamic lcd config*/
static int _samsung_lcd_config(struct omap_dss_device *dssdev, struct params *pars)
{
	int i;
	u8 buf[80];
	enum omap_dsi_index lcd_ix;
	int r;
	u32 delay;
	u8 cmd;

    lcd_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	for (i = 0; i < pars->line_nr; i++)
	{
		uint32_t data_len;
		if (!pars->params[i])
		{
			dev_err("%s : Unexpected void param\n", __func__);
			continue;
		}

		cmd = pars->params[i]->cmd;
		buf[0] = cmd;
		
		data_len = pars->params[i]->data_len;
		if (data_len > 0)
		{
			memcpy(&(buf[1]), pars->params[i]->data, data_len);
		}

		r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 1 + data_len);
		if (r)
		{
            if (0xF0 == cmd)
            {
                r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 1 + data_len);
                if (r)
                    goto err;
            }
            else
            {
                goto err;
            }
		}

		delay = pars->params[i]->delay;
		if (delay)
		{
            if (delay >= 1000)
            {
                mdelay(delay/1000);
            }
            else
            {
                udelay(delay);
            }
		}
	}
	
	return 0;

err:
	return r;
}
#endif
/* END:   Added by meijinfang, 2011/11/22 */

static int sp_power_on(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	//u8 id1, id2, id3;

    /*reason: add for ACL fouction */
     u8 buf[30];
#ifdef CONFIG_LCDD
	int ret;
#endif
	/* END:   Added by meijinfang, 2011/11/5 */
	
	int r;

	r = omapdss_dsi_display_enable(dssdev); /* DSI reset, DSI PLL, complexio config etc */
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

	if (!boot_first)
		sp_hw_reset(dssdev); /* NOP for Samsung panel */

	omapdss_dsi_vc_enable_hs(dssdev, pd->channel, false);

        /* Panel vendor suggested command sequence */
	msleep(25);

    #ifdef CONFIG_LCDD
    {
        int i;
        struct params *params;
        ret = lcdd_parse_init(&params);

		if (0 == ret)
		{
			r = lcdd_parse();
			if (0 == r)
			{
				/*print the lcd params*/
				for(i = 0; i < params->line_nr; i++)
				{
					struct lcd_param *p = params->params[i];
					if (p)
					{
						int j;
						dev_dbg("%d,0x%x,", p->delay, p->cmd);
						for(j = 0; j < p->data_len; j++)
						{
							dev_dbg("0x%x,", p->data[j]);
						}
						dev_dbg("\n");
					}
				}

				r = _samsung_lcd_config(dssdev, params);
			}
		}
		else
		{
			if (!boot_first)
				r = samsung_lcd_config(dssdev);
		}
	}
#else
    /*reason: add for ACL fouction to mactch the standerd sequence*/
	if (!boot_first)
		{
		r = samsung_lcd_config(dssdev);
		}
	/* END:   Modified by meijinfang, 2011/10/20 */
	else  //add else branch
		{
	    buf[0]  = 0xc0;
		if(true == acl_switch)
		{
		    buf[1]  = 0x01;
		}
		else
		{
		    buf[1]  = 0x00;
		}
		
	    buf[2]  = 0x00;
	    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 3);

	    /*try writing 0xC0 register three times, when write 0xc0 failed*/
	    if(r)
            {
               r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 3);

	        if(r)
	            r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 3);
            }
		}
#endif
	/* END: Deleted by meijinfang, 2011/10/20 */
	
	if (r)
		goto err;
	
	pd->enabled = 1;

    /*  Reason: Modify for disorder screen when wakeup  */
    frame_count = 0;

	omapdss_dsi_vc_enable_hs(dssdev, pd->channel, true);


	return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	sp_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev,true,false);
err0:
	return r;
}

static void sp_power_off(struct omap_dss_device *dssdev)
{
    struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
    int r=0;

    u8 data[8];

	data[0] = DCS_SLEEP_IN;
	dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, data, 1); 

	data[0] = DCS_DISPLAY_OFF;
	dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, data, 1);

	msleep(120);
	if (r) {
		dev_err(&dssdev->dev,
				"error disabling panel, issuing HW reset\n");
		sp_hw_reset(dssdev);
	}

	omapdss_dsi_display_disable(dssdev,true,false);

	pd->enabled = 0;
}

static int sp_start(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r = 0;


	dsi_bus_lock(dssdev);

	r = sp_power_on(dssdev);

#if 0
{
	u8 id1, id2, id3;


	if (pd->enabled) {
		PANEL_DBG("HARII:: Get Panel ID %s!!! \n",__func__);
		// dsi_bus_lock(ix);
		r = sp_get_id(ix, &id1, &id2, &id3);
		// dsi_bus_unlock(ix);
	} 
	PANEL_DBG("HARII::: panel id.. %s : id1=%d,id2=%d,id3=%d \n",__func__,id1,id2,id3);
}
#endif
	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		if (panel_data->use_esd_check)
			queue_delayed_work(pd->esd_wq, &pd->esd_work,
					SP_ESD_CHECK_PERIOD);
	}

	return r;
}

static void sp_stop(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	cancel_delayed_work(&pd->esd_work);

    /*  Reason: Modify for disorder screen when wakeup  */
	cancel_work_sync(&pd->frame_work);

	
	dsi_bus_lock(dssdev);

	sp_power_off(dssdev);

	dsi_bus_unlock(dssdev);
}

static void sp_disable(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&pd->lock);
	
	sp_cancel_ulps_work(dssdev);
	sp_cancel_esd_work(dssdev);

	dsi_bus_lock(dssdev);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		int r;

		r = sp_wake_up(dssdev);
		if (!r)
			sp_power_off(dssdev);
	}

	dsi_bus_unlock(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

    gpio_direction_output(panel_data->reset_gpio, 0);
    mdelay(120);

	mutex_unlock(&pd->lock);
}

/*  Reason:Add for yellow leftline  */
static int sp_suspend_command(struct omap_dss_device *dssdev)
{

    struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
    int r=0;
    u8 data[8];

    cancel_delayed_work(&pd->esd_work);
    cancel_work_sync(&pd->frame_work);

    dsi_bus_lock(dssdev);


    data[0] = DCS_SLEEP_IN;

    r = dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, data, 1);
    if(r)
    {
        dev_err(&dssdev->dev, "Entry %s write DCS_SLEEP_IN failed!\n",__FUNCTION__);
        goto err;
    }
    msleep(120);



    omapdss_dsi_display_disable(dssdev,true,false);

    pd->enabled = 0;
err:
    dsi_bus_unlock(dssdev);
    return r;
}

/*  Reason: Add for system crashed when resume and suspend sys fast.  */
static int sp_suspend(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "suspend\n");
    /*reason: modified for ACL fouction */ 
	//g_suspendstate = true;
    /* END:   Added by meijinfang, 2011/12/22 */
	if (!boot_first)
	{
       /*reason: modified for ACL fouction */ 
	    g_suspendstate = true;
		mutex_lock(&pd->lock);
		frame_count++;

		if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
			mutex_unlock(&pd->lock);
			return -EINVAL;
		}

		//sp_stop(dssdev);
        sp_suspend_command(dssdev);
		dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

		mutex_unlock(&pd->lock);
	}
	else
	{
		u32 mask;
		u32 v;

        mask = CLK_FLAG& CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;
	    v = __raw_readl(OMAP4430_CM_DIV_M5_DPLL_PER);
		/* Clear the bit to allow gatectrl */
		v &= ~mask;
		__raw_writel(v,OMAP4430_CM_DIV_M5_DPLL_PER);
		boot_first = false;
	}

	return 0;
}

static int sp_panel_reset(struct omap_dss_device *dssdev)
{
	dev_err(&dssdev->dev, "performing LCD reset\n");

	sp_power_off(dssdev);
	sp_hw_reset(dssdev);
	return sp_power_on(dssdev);
}

static int sp_enable(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	mutex_lock(&pd->lock);

	
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	dsi_bus_lock(dssdev);

	r = sp_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r)
		goto err;

	sp_queue_esd_work(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&pd->lock);

	return 0;
err:
	mutex_unlock(&pd->lock);
	return r;
}

#define GRAM_W		600
#define GRAM_H		1024
#define BYTES_PER_TIME	120
#define BYTES_PER_PIXEL	3
static int sp_zero_gram(struct sp_data *pd)
{
	int i;
	int r = 0;
	int writeTimes;
	u8 buf[BYTES_PER_TIME + 1];

    /* set display window to the gram size: 600*1024 */
	buf[0]  = 0x2a;
	buf[1]  = 0x00;
	buf[2]  = 0x00;
	buf[3]  = 0x02;
	buf[4]  = 0x57;
	r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 5);
	if(r)
        return r;

	buf[0]  = 0x2b;
	buf[1]  = 0x00;
	buf[2]  = 0x00;
	buf[3]  = 0x03;
	buf[4]  = 0xff;
	r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 5);
	if(r)
        return r;

    /* reset to column and frame start */
	buf[0]=0x2c;
	r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
	if(r)
        return r;

	msleep(1);//add for BTA error
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x3c;
	writeTimes = GRAM_H * GRAM_W * BYTES_PER_PIXEL / BYTES_PER_TIME;

	omapdss_dsi_vc_enable_hs(pd->dssdev, pd->channel, true);
	for(i=0; i<writeTimes; i++)
	{
		dsi_vc_dcs_write_nosync(pd->dssdev, pd->channel, buf, BYTES_PER_TIME + 1);
	}
	omapdss_dsi_vc_enable_hs(pd->dssdev, pd->channel, false);

    return 0;
}
/* END:   Added by meijinfang, 2012/2/3 */

/*  Reason:Add for yellow leftline  */
static int sp_resume_command(struct omap_dss_device *dssdev)
{

    u8 buf[30];
    int r = 0;
    u8 pwr_mode=0,img_mode=0;
    struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

    r = omapdss_dsi_display_enable(dssdev);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) failed to enable DSI !\n",__FUNCTION__);
        goto err;
    }

    omapdss_dsi_vc_enable_hs(dssdev, pd->channel, false);

    /* Panel vendor suggested command sequence */
    msleep(25);


    buf[0]  = 0x22; //Pixoff
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
    if(r)
        r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write pixoff failed !\n",__FUNCTION__);
        goto err;
    }

    udelay(32);


    r = sp_dcs_write_0(pd, DCS_SLEEP_OUT);
    if (r)
    return r;

    /*check display image mode ,if not in black display try once*/
    sp_dcs_read_1(pd, DCS_READ_IMG_MODE, &img_mode);
    if(!(img_mode&PANEL_BLACK_DISPLAY))
    {
        buf[0]  = 0x22;
        r = dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
        if(r)
            return r;
        img_mode = 0;
        sp_dcs_read_1(pd, DCS_READ_IMG_MODE, &img_mode);
        if(!(img_mode&PANEL_BLACK_DISPLAY))
        {
            printk("%s:img_mode normal display\n",__FUNCTION__);
        }
    }

    /*if not in sleep out state ,try once*/
    sp_dcs_read_1(pd, DCS_READ_POWER_MODE, &pwr_mode);
    if(!(pwr_mode&PANEL_SLEEP_IN))
    {
        r = sp_dcs_write_0(pd, DCS_SLEEP_OUT);

        if (r)
             return r;
         pwr_mode = 0;
         r = sp_dcs_read_1(pd, DCS_READ_POWER_MODE, &pwr_mode);
         if(!(pwr_mode&PANEL_SLEEP_IN))
         {
             printk("%s:pwr_mode sleep in\n",__FUNCTION__);
         }
    }
    //msleep(120);
    msleep(20);
    #if 0
    /* memory window setting1 */
    buf[0]  = 0x2a;
    buf[1]  = 0x00;
    buf[2]  = 0x00;
    buf[3]  = 0x02;
    buf[4]  = 0x57;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 5);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write %x failed !\n",__FUNCTION__,buf[0]);
        goto err;
    }

    buf[0]  = 0x2b;
    buf[1]  = 0x00;
    buf[2]  = 0x00;
    buf[3]  = 0x03;
    buf[4]  = 0xff;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 5);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write %x failed !\n",__FUNCTION__,buf[0]);
        goto err;
    }
    #endif

    /*r = sp_zero_gram(pd);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) sp_zero_gram failed !\n",__FUNCTION__);
        goto err;
    }*/

    udelay(32);
    /* END:   Modified by meijinfang, 2012/2/3 */

    /* memory window setting2 */
    buf[0] = 0x35;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 1);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write %x failed !\n",__FUNCTION__,buf[0]);
        goto err;
    }

    buf[0]  = 0x2a;
    buf[1]  = 0x00;
    buf[2]  = 0x1e;
    buf[3]  = 0x02;
    buf[4]  = 0x39;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 5);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write %x failed !\n",__FUNCTION__,buf[0]);
        goto err;
    }

    buf[0]  = 0x2b;
    buf[1]  = 0x00;
    buf[2]  = 0x00;
    buf[3]  = 0x03;
    buf[4]  = 0xbf;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 5);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write %x failed !\n",__FUNCTION__,buf[0]);
        goto err;
    }

    buf[0] = 0xd1;
    buf[1] = 0x8a;
    r = dsi_vc_dcs_write(pd->dssdev, pd->channel,  buf, 2);
    if(r)
    {
        dev_err(&dssdev->dev, "(%s) write %x failed !\n",__FUNCTION__,buf[0]);
        goto err;
    }

    udelay(32);
    pd->enabled = 1;

err:
    omapdss_dsi_vc_enable_hs(dssdev, pd->channel, true);
    return r;

}

/*  Reason: Add code to handle resume faild  */
static int sp_resume(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int r = 0,count = 0;

	dev_dbg(&dssdev->dev, "resume\n");

	mutex_lock(&pd->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
        dev_err(&dssdev->dev,"^-^^-^HT: resum with a wrong state %d \n",dssdev->state);
        goto err;
	}

	dsi_bus_lock(dssdev);

    frame_count = 0;
    sp_frame_num = 0;
again:
	r = sp_resume_command(dssdev);//sp_power_on(dssdev);
    if(r && count<3)
    {
        count++;
        resume_failed_times++;
        dev_err(&dssdev->dev,"^-^^-^HT: resum failed %d times\n",count);
        goto again;
    }


	dsi_bus_unlock(dssdev);

	if (r) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		sp_queue_esd_work(dssdev);

        g_suspendstate = false;
        /* END:   Added by meijinfang, 2011/12/22 */
	}

	mutex_unlock(&pd->lock);

	return r;

err:
	mutex_unlock(&pd->lock);
	return r;
}

/*  Reason: Add for system crashed when resume and suspend sys fast.  */
static void sp_framedone_cb(int err, void *data)
{
    struct omap_dss_device *dssdev = data;

/*  Reason: Modify for disorder screen when wakeup  */
    struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

    if(frame_count == 0)
    {
        frame_count++;
        queue_work(pd->frame_wq, &pd->frame_work);
    }

	udelay(100);

    dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
    dsi_bus_unlock(dssdev);
}

static irqreturn_t sp_te_isr(int irq, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int old;
	int r;

	old = atomic_cmpxchg(&pd->do_update, 1, 0);

	if (old) {
		cancel_delayed_work(&pd->te_timeout_work);

		r = omap_dsi_update(dssdev, pd->channel,
				pd->update_region.x,
				pd->update_region.y,
				pd->update_region.w,
				pd->update_region.h,
				sp_framedone_cb, dssdev);
		if (r)
			goto err;
	}

	return IRQ_HANDLED;
err:
	dev_err(&dssdev->dev, "start update failed\n");
	dsi_bus_unlock(dssdev);
	return IRQ_HANDLED;
}

static void sp_te_timeout_work_callback(struct work_struct *work)
{
	struct sp_data *pd = container_of(work, struct sp_data,
					te_timeout_work.work);
	struct omap_dss_device *dssdev = pd->dssdev;

	dev_err(&dssdev->dev, "TE not received for 250ms!\n");

	atomic_set(&pd->do_update, 0);
	dsi_bus_unlock(dssdev);
}


static int sp_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{

	
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	/* If black screen appear, change the boot_skip value to avoid it. */
	if (boot_skip > 1)
	{
		boot_skip--;
		return 0;
	}
	else if (boot_skip == 1)
	{
		boot_skip--;
		mutex_lock(&pd->lock);
		dsi_bus_lock(dssdev);
		omapdss_dsi_vc_enable_hs(dssdev, pd->channel, false);

		msleep(25);
		//samsung_lcd_convert_window(dssdev);
		omapdss_dsi_vc_enable_hs(dssdev, pd->channel, true);
		dsi_bus_unlock(dssdev);
		mutex_unlock(&pd->lock);
		return 0;
	}
	mutex_lock(&pd->lock);
	dsi_bus_lock(dssdev);

	r = sp_wake_up(dssdev);
	if (r)
		goto err;

	if (!pd->enabled) {
		r = 0;
		goto err;
	}
	
	//PANEL_DBG("sp_update_locked________\n");
	
	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)

		goto err;

	r = sp_set_update_window(pd, x, y, w, h);
	if (r)
		goto err;

	if (pd->te_enabled && panel_data->use_ext_te) {
		pd->update_region.x = x;
		pd->update_region.y = y;
		pd->update_region.w = w;
		pd->update_region.h = h;
		barrier();
		schedule_delayed_work(&pd->te_timeout_work,
				msecs_to_jiffies(250));
		atomic_set(&pd->do_update, 1);
	} else {

		if(frame_count == 0)
		{
		   // msleep(200);
		    msleep(50);
		}

		/* We use VC(1) for VideoPort Data and VC(0) for L4 data */
		r = omap_dsi_update(dssdev, pd->channel, x, y, w, h,
				sp_framedone_cb, dssdev);
/*
		if (cpu_is_omap44xx())
			r = omap_dsi_update(dssdev, pd->channel, x, y, w, h,
				sp_framedone_cb, dssdev);
		else
			r = omap_dsi_update(dssdev, pd->channel, x, y, w, h,
				sp_framedone_cb, dssdev);
*/				
		if (r)
			goto err;
			
	}

#if 0
{
	u8 id1, id2, id3;

	PANEL_DBG("HARII:: Get Panel ID !!! \n");
	if (pd->enabled) {
		dsi_bus_lock(dssdev);
		r = sp_get_id(pd, &id1, &id2, &id3);
		dsi_bus_unlock(dssdev);
	} else {
		r = -ENODEV;
	}
	PANEL_DBG("HARII::: panel id: id1=%d,id2=%d,id3=%d \n",id1,id2,id3);
}
#endif

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&pd->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&pd->lock);
	return r;
}

/*  Reason:Add for yellow leftline  */
static int sp_double_update(struct omap_dss_device *dssdev,
                                u16 x, u16 y, u16 w, u16 h)
{
    int r;

    r = sp_update(dssdev,x,y,w,h);
    if(sp_frame_num < 5)
    {
        sp_frame_num++;
       // r = sp_update(dssdev,x,y,w,h);
    }
    return r;
}


/*  Reason: Add for system crashed when resume and suspend sys fast.  */

/*  Reason: Modify for disorder screen when wakeup  */
static void sp_frame_work(struct work_struct *work)
{
    u8 buf[20];
    int r;
    struct sp_data *pd = container_of(work, struct sp_data,frame_work);
    struct omap_dss_device *dssdev = pd->dssdev;

    mutex_lock(&pd->lock);
    if (1 == frame_count)
    {
        frame_count++;
    }
    else
    {
        mutex_unlock(&pd->lock);
        return;
    }

    dsi_bus_lock(dssdev);

    buf[0]  = 0x13;
    r=dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
    if(r)
    {
        r=dsi_vc_dcs_write(pd->dssdev, pd->channel, buf, 1);
        if(r)
        {
            frame_count = 0;
            dev_err(&dssdev->dev,"\n %s ================error recorvery\n",__func__);
        }
    }

    dsi_bus_unlock(dssdev);
    mutex_unlock(&pd->lock);
    return;
}



static int sp_sync(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "sync\n");

	mutex_lock(&pd->lock);
	dsi_bus_lock(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&pd->lock);

	dev_dbg(&dssdev->dev, "sync done\n");

	return 0;
}

static int _sp_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if (enable) {
		r = sp_dcs_write_1(pd, DCS_TEAR_ON, 0);;
		if (r)
			goto error;

	} else {
	        r = sp_dcs_write_0(pd, DCS_TEAR_OFF);
	
	        if (!panel_data->use_ext_te)
	                omapdss_dsi_enable_te(dssdev, enable);
	
	        if (pd->panel_config->sleep.enable_te)
	                msleep(pd->panel_config->sleep.enable_te);
        }
error:
        return r;
}

static int sp_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&pd->lock);

	if (pd->te_enabled == enable)
		goto end;

	dsi_bus_lock(dssdev);

	if (pd->enabled) {
		r = _sp_enable_te(dssdev, enable);
		if (r)
			goto err;
	}

	pd->te_enabled = enable;

	dsi_bus_unlock(dssdev);
end:
	mutex_unlock(&pd->lock);

	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&pd->lock);

	return r;
}

static int sp_get_te(struct omap_dss_device *dssdev)
{
	struct sp_data *pd = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&pd->lock);
	r = pd->te_enabled;
	mutex_unlock(&pd->lock);

	return r;
}

static int sp_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	return 0;
}

static u8 sp_get_rotate(struct omap_dss_device *dssdev)
{
	return 0;
}

static int sp_mirror(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static bool sp_get_mirror(struct omap_dss_device *dssdev)
{
	return 0;
}

static int sp_run_test(struct omap_dss_device *dssdev, int test_num)
{
	/* TODO */
	return -1;

}

static int sp_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h)
{
    int r;
    int first = 1;
    int plen = 0;
    unsigned buf_used = 0;
    struct sp_data *pd = dev_get_drvdata(&dssdev->dev);

    if (size < w * h * 3)
        return -ENOMEM;

    mutex_lock(&pd->lock);

    if (!pd->enabled) {
        r = -ENODEV;
        goto err1;
    }

    size = min(w * h * 3,
            dssdev->panel.timings.x_res *
            dssdev->panel.timings.y_res * 3);

    dsi_bus_lock(dssdev);

    r = sp_wake_up(dssdev);
    if (r)
        goto err2;

    /* plen 1 or 2 goes into short packet. until checksum error is fixed,
     * use short packets. plen 32 works, but bigger packets seem to cause
     * an error. */
    if (size % 2)
        plen = 1;
    else
        plen = 2;

    sp_set_update_window(pd, x, y, w, h);

    r = dsi_vc_set_max_rx_packet_size(dssdev, pd->channel, plen);
    if (r)
        goto err2;

    printk("size:%d\n",size);
    while (buf_used < size) {
        u8 dcs_cmd = first ? 0x2e : 0x3e;
        first = 0;

        r = dsi_vc_dcs_read(dssdev, pd->channel, dcs_cmd,
                buf + buf_used, size - buf_used);
        if (r < 0) {
            dev_err(&dssdev->dev, "read error\n");
            goto err3;
        }

        buf_used += r;
        printk("dcs read len:%d\n",buf_used);

        if (r < plen) {
            dev_err(&dssdev->dev, "short read\n");
            break;
        }
        msleep(1);

    }

    r = buf_used;

err3:
    dsi_vc_set_max_rx_packet_size(dssdev, pd->channel, 1);
err2:
    dsi_bus_unlock(dssdev);
err1:
    mutex_unlock(&pd->lock);
    return r;
}

static void sp_ulps_work(struct work_struct *work)
{
	struct sp_data *pd = container_of(work, struct sp_data,
			ulps_work.work);
	struct omap_dss_device *dssdev = pd->dssdev;

	mutex_lock(&pd->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE || !pd->enabled) {
		mutex_unlock(&pd->lock);
		return;
	}

	dsi_bus_lock(dssdev);

	sp_enter_ulps(dssdev);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&pd->lock);
}
static void sp_esd_work(struct work_struct *work)
{
#if 0
	struct sp_data *pd = container_of(work, struct sp_data,
			esd_work.work);
	struct omap_dss_device *dssdev = pd->dssdev;
	struct sp_dsi_panel_data *panel_data = get_panel_data(dssdev);
	u8 power_mode;
	int r;
	enum omap_dsi_index ix;
	ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD) ? DSI1 : DSI2;

	mutex_lock(&pd->lock);

	if (!pd->enabled) {
		mutex_unlock(&pd->lock);
		return;
	}

	dsi_bus_lock(ix);

	r = sp_dcs_read_1(ix,DCS_READ_POWER_MODE, &power_mode);
	if (r) {
		dev_err(&dssdev->dev, "failed to read panel power mode\n");
		goto err;
	}

#if 0
	if (atomic_read(&panel_data->state) == PANEL_ON)
		expected_mode = 0x9c;
	else
		expected_mode = 0x98;

	PANEL_DBG("ESD Check - read mode = 0x%02x, expected = 0x%02x\n", power_mode,
		expected_mode);

	if (power_mode != expected_mode) {
		dev_err(&dssdev->dev,
			"Power mode in incorrect state, "
			"mode = 0x%02x, expected = 0x%02x\n",
			power_mode, expected_mode);
		goto err;
	}
#endif
	dsi_bus_unlock(ix);

	queue_delayed_work(sp_data->esd_wq, &sp_data->esd_work,
			 SP_ESD_CHECK_PERIOD);

	mutex_unlock(&sp_data->lock);
	return;

err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	sp_power_off(dssdev);
	sp_hw_reset(dssdev);
	sp_power_on(dssdev);

	dsi_bus_unlock(ix);

	queue_delayed_work(pd->esd_wq, &pd->esd_work, SP_ESD_CHECK_PERIOD);

	mutex_unlock(&pd->lock);
#endif
}

static int sp_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	return 0;
}

static enum omap_dss_update_mode sp_get_update_mode(
		struct omap_dss_device *dssdev)
{
	return OMAP_DSS_UPDATE_MANUAL;
}

/*  Reason:Add for yellow leftline  */
static struct omap_dss_driver sp_driver = {
	.probe		= sp_probe,
	.remove		= sp_remove,

	.enable		= sp_enable,
	.disable	= sp_disable,
	.suspend	= sp_suspend,
	.resume		= sp_resume,

	.set_update_mode = sp_set_update_mode,
	.get_update_mode = sp_get_update_mode,

	.update		= sp_update,
	//.sched_update	= sp_sched_update,
	.sync		= sp_sync,

	.get_resolution	= sp_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.enable_te	= sp_enable_te,
	.get_te		= sp_get_te,

	.set_rotate	= sp_rotate,
	.get_rotate	= sp_get_rotate,
	.set_mirror	= sp_mirror,
	.get_mirror	= sp_get_mirror,
	.run_test	= sp_run_test,
	.memory_read	= sp_memory_read,

	.get_timings	= sp_get_timings,
	.set_timings	= sp_set_timings,
	.check_timings	= sp_check_timings,

	.driver         = {
		.name   = "spanel",
		.owner  = THIS_MODULE,
	},
};

/*  Reason: For the can't-wakeup issue  */

/*  Reason: lcd-compatible  */
static int __init sp_init(void)
{
	PANEL_DBG("%s \n",__func__);
	if(panel_is_the_panel_supported(sp_driver.driver.name))
	{
		PANEL_DBG(KERN_INFO"Function(%s): Read sunsamg config sucess!\n",__FUNCTION__);
		omap_dss_register_driver(&sp_driver);
	}
	else
	{
		PANEL_DBG(KERN_ERR"Function (%s) sunsamg panel is not configed\n",__FUNCTION__);
	}

	return 0;
}


static void __exit sp_exit(void)
{
	omap_dss_unregister_driver(&sp_driver);
}

module_init(sp_init);
module_exit(sp_exit);

MODULE_AUTHOR("Hari Nagalla <hnagalla@ti.com>");
MODULE_DESCRIPTION("Samsung S6E39A DSI Driver");
MODULE_LICENSE("GPL");
