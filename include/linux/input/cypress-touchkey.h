/*
 * Copyright 2006-2010, Cypress Semiconductor Corporation.
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#ifndef _CYPRESS_TOUCHKEY_H__
#define _CYPRESS_TOUCHKEY_H__

#define CYPRESS_TOUCHKEY_I2C_NAME "cypress_tk_i2c"

#define	CY8C20236A_I2C_SLAVER_ADD 25
#define	CY8C20236A_IRQ_GPIO_ID 151


#define CY8C20236A_TOUCH_SCL_IO 128 
#define CY8C20236A_TOUCH_SDA_IO 129 
#define CY8C20236A_TOUCH_RESET_IO 152

struct touchkey_platform_data {
	int irq_gpio_id;
	int i2c_slave_addr;
	int keycode_cnt;
	const int *keycode;
	void (*touchkey_onoff) (int);
};

enum {
	TOUCHKEY_OFF,
	TOUCHKEY_ON,
};

#endif /* _CYPRESS_TOUCHKEY_H__ */

