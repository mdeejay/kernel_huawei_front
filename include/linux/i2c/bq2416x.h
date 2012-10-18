/*
 * Copyright (C) 2010 Texas Instruments
 * Author: Balaji T K
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

 
#ifndef _LINUX_BQ2416X_I2C_H
#define _LINUX_BQ2416X_I2C_H

#define BQ2416x_IC_THRESHOLD_VOLTAGE   4000
#define BQ2416x_NO_CHARGER_SOURCE	   0x00
#define BQ2416x_NOT_CHARGING     	   0x10
#define BQ2416x_START_CHARGING     	   0x20
#define BQ2416x_START_AC_CHARGING	   0x30
#define BQ2416x_START_USB_CHARGING	   0x40
#define BQ2416x_CHARGE_DONE		       0x50
#define BQ2416x_STOP_CHARGING		   0x60
#define POWER_SUPPLY_STATE_FAULT       0x70
#define POWER_SUPPLY_OVERVOLTAGE       0x80
#define POWER_SUPPLY_WEAKSOURCE        0x90
#define BATTERY_LOW_WARNING	   0x51
#define BATTERY_LOW_SHUTDOWN     0x52

#define BQ2416x_FAULT_THERMAL_SHUTDOWN	   0x71
#define BQ2416x_FAULT_BATTERY_TEMPERATURE  0x72
#define BQ2416x_FAULT_WATCHDOG_TIMER	   0x73
#define BQ2416x_FAULT_SAFETY_TIMER		   0x74
#define BQ2416x_FAULT_AC_SUPPLY		       0x75
#define BQ2416x_FAULT_USB_SUPPLY		   0x76
#define BQ2416x_FAULT_BATTERY		       0x77

#define BQ2416x_FAULT_VAC_OVP		      0x40
#define BQ2416x_FAULT_VAC_WEAK		0x80
#define BQ2416x_FAULT_VAC_VUVLO		0xC0

#define BQ2416x_FAULT_VBUS_OVP		0x10
#define BQ2416x_FAULT_VBUS_WEAK		0x20
#define BQ2416x_FAULT_VBUS_VUVLO	      0x30

#define BQ2416x_FAULT_BAT_OVP		      0x02
#define BQ2416x_FAULT_NO_BATTERY	      0x04

#define BQ2416x_FAULT_TS_COLD_OR_HOT		 0x02
#define BQ2416x_FAULT_TS_COLD_AND_COOL    0x04
#define BQ2416x_FAULT_TS_WARM_AND_HOT	 0x06

/* not a bq generated event,we use this to reset the
 * the timer from the twl driver.
 */
#define BQ2416x_RESET_TIMER		0x38

struct bq2416x_platform_data {
	int max_charger_currentmA;
	int max_charger_voltagemV;
	int termination_currentmA;
};

int bq2416x_register_notifier(struct notifier_block *nb, unsigned int events);
int bq2416x_unregister_notifier(struct notifier_block *nb, unsigned int events);

#endif
