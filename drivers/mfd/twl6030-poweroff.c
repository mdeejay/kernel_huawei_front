/*
 * /drivers/mfd/twl6030-poweroff.c
 *
 * Power off device
 *
 * Copyright (C) 2011 Texas Instruments Corporation
 *
 * Written by Rajeev Kulkarni <rajeevk@ti.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>

#define TWL6030_PHOENIX_DEV_ON	0x25
#define APP_DEVOFF	(1<<0)
#define CON_DEVOFF	(1<<1)
#define MOD_DEVOFF	(1<<2)

#include <linux/reboot.h>
#include <linux/syscalls.h>
#define SW_RESET      (1<<6)
#define CONTROLLER_STAT1		0x03
#define	VBUS_DET   (1<<2)

#define STS_HW_CONDITIONS		0x21
#define STS_USB_ID			(1<<2)

#define RUNMODE_FLAG_FACTORY 1
#define RUNMODE_FLAG_NORMAL 0
extern unsigned char runmode_is_factory(void);

static int is_charger_present(void)
{
	u8 vbus_state=0, hw_state=0;
	int charger_present =0;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &vbus_state,CONTROLLER_STAT1);
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
	vbus_state = vbus_state & VBUS_DET;
    /*check charger present*/
	if(vbus_state && !(hw_state & STS_USB_ID)){
		charger_present = 1;
	}
	else{
		charger_present = 0;
	}
	return charger_present;
}


void twl6030_poweroff(void)
{
	u8 val = 0;
	int err = 0;
       if(RUNMODE_FLAG_FACTORY != runmode_is_factory())
	{
	    if (is_charger_present()){
		pr_info("%s: charger present poweroff reboot\n", __func__);
		kernel_restart("charger_poweroff_reboot");
		return;
	    }
       }

	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &val,
				  TWL6030_PHOENIX_DEV_ON);
	if (err) {
		pr_warning("I2C error %d reading PHOENIX_DEV_ON\n", err);
		return;
	}

	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;

	err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,
				   TWL6030_PHOENIX_DEV_ON);

	if (err) {
		pr_warning("I2C error %d writing PHOENIX_DEV_ON\n", err);
		return;
	}

	return;
}

static int __init twl6030_poweroff_init(void)
{
	pm_power_off = twl6030_poweroff;

	return 0;
}

static void __exit twl6030_poweroff_exit(void)
{
	pm_power_off = NULL;
}

module_init(twl6030_poweroff_init);
module_exit(twl6030_poweroff_exit);

MODULE_DESCRIPTION("TLW6030 device power off");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajeev Kulkarni");
