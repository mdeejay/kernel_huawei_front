/*
 * drivers/power/bq2416x_battery.c
 *
 * BQ24160/2 / BQ24161 battery charging driver
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>

#include <linux/i2c/bq27510_battery.h>
#include <linux/i2c/bq2416x.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <hsad/config_interface.h>

#define BQ2416X_GPIO_174  174

/* BQ24160 / BQ24161 / BQ24162 */
/* Status/Control Register */
#define REG_STATUS_CONTROL			       0x00
#define		TIMER_RST			             (1 << 7)     /* write "1" to reset the watchdog*/
#define		ENABLE_USB_SUPPLY_SEL	(1 << 3)     /* USB has precedence when both supplies are connected*/
#define		BQ2416x_USB_READY	            0x20 /*display charger is not charging*/
#define		BQ2416x_CHARGING_FROM_USB	    0x40 /*display charger is charging from USB interface*/
#define     ENABLE_ITERM      1
#define     DISABLE_ITERM     0
#define     DISABLE_CE        1
#define     ENABLE_CE         0
#define     BQ2416x_VUSB_SUPPLY_FAULT  0x30
#define     BQ2416x_VUSB_FAULT         0x70
#define     DISABLE_LOW_CHG    0

/* Battery/ Supply Status Register */
#define REG_BATTERY_AND_SUPPLY_STATUS	0x01

/* Control Register */
#define REG_CONTROL_REGISTER			      0x02
#define		INPUT_CURRENT_LIMIT_SHIFT	       4    
#define		ENABLE_ITERM_SHIFT		          2     /* Enable charge current termination*/
#define		ENABLE_STAT_PIN			 (1 << 3)        /* Enable STAT output to show charge status*/
#define		ENABLE_ICE_PIN		               0x7D  /* Charger enabled (default 0 for bq24160, 1 for bq24161/2)*/

/* Control/Battery Voltage Register */
#define REG_BATTERY_VOLTAGE			0x03
#define		VOLTAGE_SHIFT			           2

/* Vender/Part/Revision Register */
#define REG_PART_REVISION			       0x04

/* Battery Termination/Fast Charge Current Register */
#define REG_BATTERY_CURRENT			0x05
#define		BQ2416x_CURRENT_SHIFT           3 /* fast charge current shift bit*/

/* VIN-DPM Voltage/ DPPM Status Register */
#define REG_DPPM_VOLTAGE		            0x06
#define		USB_INPUT_DPPM_SHIFT		    3   /* USB input VIN-DPM voltage shift bit */

#define		CURRENT_USB_LIMIT_IN		500
#define		CURRENT_USB_CHARGE_IN	    550
#define		CURRENT_AC_LIMIT_IN		    1000
#define     CURRENT_AC_LIMIT_IN_900      900
#define     CURRENT_AC_LIMIT_IN_800		 800
#define		CURRENT_TERM_CHARGE_IN	    50

#define		VOLT_DPPM_ADJUST		    4520
#define		VOLT_DPPM_ADJUST_AC		    4200
/* Safety Timer/NTC Monitor Register */
#define REG_SAFETY_TIMER			    0x07
#define		ENABLE_TMR_PIN		   (1 << 7) /* Timer slowed by 2x when in thermal regulation*/
#define		TMR_X_6			       (1 << 5) /* 6 hour fast charge*/
#define		TMR_X_9			       (1 << 6) /* 7 hour fast charge*/
#define		ENABLE_TS_PIN		   (1 << 3) /* TS function enabled*/

/*preconditioning current is 100mA below 3.0V*/
#define     ENABLE_LOW_CHG		   (1 << 0)

#define 	BQ2416x_COLD_BATTERY_THRESHOLD	  -100 //(-10 *10) battery temperature is -10 degree
#define 	BQ2416x_COOL_BATTERY_THRESHOLD       0 //(  0 *10) battery temperature is 0 degree
#define 	BQ2416x_WARM_BATTERY_THRESHOLD     400 //( 40 *10) battery temperature is 50 degree
#define 	BQ2416x_HOT_BATTERY_THRESHOLD      500 //( 50 *10) battery temperature is 50 degree
#define     TEMPERATURE_OFFSET                 30 // 3 degree offset
#define     BQ2416x_PRECONDITIONING_BATVOLT_THRESHOLD  3000000 //(3.0V) battery preconditioning voltage is 3.0V
#define     BQ2416x_STARTUP_BATVOLT_THRESHOLD      3200000 //(3.2V) battery startup voltage is 3.0V
#define  	BQ2416x_LOW_TEMP_TERM_VOLTAGE  4000000  // low temperature charge termination voltage
#define BQ24161 (1 << 1)

//r00186667, 2011/08/02, add ac wake_lock.begin
#define BQ2416X_USE_WAKE_LOCK  1
//r00186667, 2011/08/02, add ac wake_lock.end

#define		EN_NOBATOP		   (1 << 0) /* Enable no battery operation*/

#define DRIVER_NAME			"bq2416x"
static int timer_fault;

struct charge_params {
	unsigned long		currentmA;
	unsigned long		voltagemV;
	unsigned long		term_currentmA;
	unsigned long		enable_iterm;
	bool			enable;
};

struct bq2416x_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct charge_params	params;
	struct delayed_work	bq2416x_charger_work;
//	struct notifier_block	nb;

	unsigned short		status_reg;
	unsigned short		control_reg;
	unsigned short		voltage_reg;
	unsigned short		bqchip_version;
	unsigned short		current_reg;
	unsigned short		special_charger_reg;

	unsigned int		cin_limit;
	unsigned int		currentmA;
	unsigned int		voltagemV;
	unsigned int		max_currentmA;
	unsigned int		max_voltagemV;
	unsigned int		term_currentmA;
	unsigned int        dppm_voltagemV;
  unsigned short		  usb_supply_sel_reg;
  unsigned short		  tmr_ts_reg;
	unsigned short	    dppm_reg;
	bool                    enable_ce;  
	bool                    hz_mode; 
	bool                    cd_active;
	int			charge_status;
	int			charger_source;

	bool			cfg_params;
	bool			enable_iterm;
	bool			active;
//r00186667, 2011/08/02, add for otg register.begin   
	unsigned long		event;
	struct otg_transceiver	*otg;
	struct notifier_block	nb_otg;
//r00186667, 2011/08/02, add for otg register.end   	
#if BQ2416X_USE_WAKE_LOCK
	struct wake_lock charger_wake_lock;
#endif
	bool   enable_low_chg;
	bool   factory_flag; // hand control charging process
	struct work_struct	usb_work;
     bool enable_hotcold_temp_charge;
	bool calling_limit;
};

/*exterm a bq27510 instance for reading battery info*/
extern struct bq27510_device_info *g_battery_measure_by_bq27510_device;
/*extern a notifier list for charging notification*/
extern struct blocking_notifier_head notifier_list;
extern u32 wakeup_timer_seconds;
#define BQ2416x_WATCHDOG_TIMEOUT 20000
#define set_one  1
#define set_zero 0
/**
 * bq2416x_write_block:
 * returns 0 if write successfully
 * This is API to check whether OMAP is waking up from device OFF mode.
 * There is no other status bit available for SW to read whether last state
 * entered was device OFF. To work around this, CORE PD, RFF context state
 * is used which is lost only when we hit device OFF state
 */
int bq2416x_write_block(struct bq2416x_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[1];
	int ret;

	*value		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= value;
	msg[0].len	= num_bytes + 1;
	
	ret = i2c_transfer(di->client->adapter, msg, 1);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

static int bq2416x_read_block(struct bq2416x_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[2];
	u8 buf;
	int ret;

	buf		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= &buf;
	msg[0].len	= 1;

	msg[1].addr	= di->client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= value;
	msg[1].len	= num_bytes;

	ret = i2c_transfer(di->client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

static int bq2416x_write_byte(struct bq2416x_device_info *di, u8 value, u8 reg)
{
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };

	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return bq2416x_write_block(di, temp_buffer, reg, 1);
}

static int bq2416x_read_byte(struct bq2416x_device_info *di, u8 *value, u8 reg)
{
	return bq2416x_read_block(di, value, reg, 1);
}

/*
 *config TMR_RST function to reset the watchdog
 * 
 */
static void bq2416x_config_usb_supply_sel_reg(struct bq2416x_device_info *di) 
{
	//di->usb_supply_sel_reg = (TIMER_RST | ENABLE_USB_SUPPLY_SEL);
	di->usb_supply_sel_reg = (TIMER_RST);
	bq2416x_write_byte(di, di->usb_supply_sel_reg, REG_STATUS_CONTROL);
	return;
}

/*
 *Enable STAT pin output to show charge status
 * 
 */
static void bq2416x_config_status_reg(struct bq2416x_device_info *di)
{
	bq2416x_config_usb_supply_sel_reg(di);
	bq2416x_write_byte(di, di->control_reg, REG_CONTROL_REGISTER);
	return;
}

/*
 *di->cin_limit:      set usb input limit current(100,150,500,800,900,1500)
 *di->enable_iterm:   Enable charge current termination
 *ENABLE_STAT_PIN: Enable STAT pin output to show charge status
 * di->enable_ce=0 : Charger enabled
 */
static void bq2416x_config_control_reg(struct bq2416x_device_info *di)
{
	u8 Iin_limit;

	if (di->cin_limit <= 100)
		Iin_limit = 0;
	else if (di->cin_limit > 100 && di->cin_limit <= 150)
		Iin_limit = 1;
	else if (di->cin_limit > 150 && di->cin_limit <= 500)
		Iin_limit = 2;
	else if (di->cin_limit > 500 && di->cin_limit <= 800)
		Iin_limit = 3;
	else if (di->cin_limit > 800 && di->cin_limit <= 900)
		Iin_limit = 4;	
	else if (di->cin_limit > 900 && di->cin_limit <= 1500)
		Iin_limit = 5;
	else
		Iin_limit = 6;

	di->control_reg = ((Iin_limit << INPUT_CURRENT_LIMIT_SHIFT)
				| (di->enable_iterm << ENABLE_ITERM_SHIFT) | ENABLE_STAT_PIN 
				|( di->enable_ce << 1) |di->hz_mode);
	bq2416x_write_byte(di, di->control_reg, REG_CONTROL_REGISTER);
	return;
}

/*
 * set Battery Regulation Voltage between 3.5V and 4.44V
 *
 */
static void bq2416x_config_voltage_reg(struct bq2416x_device_info *di)
{
	unsigned int voltagemV;
	u8 Voreg;

	voltagemV = di->voltagemV;
	if (voltagemV < 3500)
		voltagemV = 3500;
	else if (voltagemV > 4440)
		voltagemV = 4440;

	Voreg = (voltagemV - 3500)/20;
	di->voltage_reg = (Voreg << VOLTAGE_SHIFT);
	bq2416x_write_byte(di, di->voltage_reg, REG_BATTERY_VOLTAGE);
	return;
}

/*
 * set Battery charger current(550 ~1500mA) and Termination current(50~350mA)
 *
 */
static void bq2416x_config_current_reg(struct bq2416x_device_info *di)
{
	unsigned int currentmA = 0;
	unsigned int term_currentmA = 0;
	u8 Vichrg = 0;
	u8 shift = 0;
	u8 Viterm = 0;

	currentmA = di->currentmA;
	term_currentmA = di->term_currentmA;

	if (currentmA < 550)
		currentmA = 550;

//r00186667, 20110802 , removed begin
#if 0
	if ((di->bqchip_version & (BQ24153 | BQ24158))) {
		shift = BQ24153_CURRENT_SHIFT;
		if (currentmA > 1250)
			currentmA = 1250;
	}
#endif
//r00186667, 20110802 , removed end    

	if ((di->bqchip_version & BQ24161)) {
		shift = BQ2416x_CURRENT_SHIFT;
		if (currentmA > 2500)
			currentmA = 2500;
	}

	if (term_currentmA > 350)
		term_currentmA = 350;

	Vichrg = (currentmA - 550)/75;

	Viterm = (term_currentmA - 50)/50;

	di->current_reg = (Vichrg << shift | Viterm);
	bq2416x_write_byte(di, di->current_reg, REG_BATTERY_CURRENT);
	
	return;
}

/*
 * set USB input dppm voltage between 4.2V and 4.76V 
 *
 */
static void bq2416x_config_dppm_voltage_reg(struct bq2416x_device_info *di,
	                             unsigned int dppm_voltagemV) 
{
      u8 Vmreg;

	if (dppm_voltagemV < 4200)
		dppm_voltagemV = 4200;
	else if (dppm_voltagemV > 4760)
		dppm_voltagemV = 4760;
	
	Vmreg = (dppm_voltagemV - 4200)/80;
	
	di->dppm_reg =(Vmreg << USB_INPUT_DPPM_SHIFT);
	bq2416x_write_byte(di, di->dppm_reg,
					REG_DPPM_VOLTAGE);
	return;
}

/*
 * enable TMR_PIN and set Safety Timer Time Limit = 6h
 *
 */
static void bq2416x_config_safety_reg(struct bq2416x_device_info *di)
{
    di->tmr_ts_reg = (ENABLE_TMR_PIN |TMR_X_9)  | di->enable_low_chg; //safety time = 6h and ENABLE_LOW_CHG

	//di->tmr_ts_reg = (ENABLE_TMR_PIN); //safety time = 27 min
	bq2416x_write_byte(di, di->tmr_ts_reg, REG_SAFETY_TIMER);
	return;
}

/*di->enable_ce = 0,charger enable and display charging status
  di->enable_ce = 1 or USB supply fault,charger disable and display not charging status*/
static void
bq2416x__charge_status(struct bq2416x_device_info *di)
{
	long int events=BQ2416x_START_CHARGING;
	u8 read_reg[8] = {0};
    int battery_capacity = 0;

	bq2416x_read_block(di, &read_reg[0], 0, 2);

	if((read_reg[1] & BQ2416x_FAULT_VBUS_VUVLO) == BQ2416x_FAULT_VBUS_OVP){
		dev_err(di->dev, " POWER_SUPPLY_OVERVOLTAGE = %x\n",read_reg[1]);
		events = POWER_SUPPLY_OVERVOLTAGE;
		blocking_notifier_call_chain(&notifier_list, events, NULL);
		return;
	}

	if ((read_reg[0] & BQ2416x_VUSB_FAULT) == BQ2416x_CHARGING_FROM_USB){
				events = BQ2416x_START_CHARGING;
	}
	else if((di->enable_ce == DISABLE_CE)
		|| ((read_reg[0] & POWER_SUPPLY_STATE_FAULT) == POWER_SUPPLY_STATE_FAULT)){
		dev_err(di->dev, " BQ2416x_NOT_CHARGING \n");
		events = BQ2416x_NOT_CHARGING;
	}

	else if((read_reg[0] & BQ2416x_VUSB_FAULT) == BQ2416x_CHARGE_DONE){
        battery_capacity = bq27510_battery_capacity(g_battery_measure_by_bq27510_device);
        if((!is_bq27510_battery_full(g_battery_measure_by_bq27510_device))||(battery_capacity !=100)){
            dev_info(di->dev, "charge_done_battery_capacity=%d\n",battery_capacity);
            di->hz_mode = 1; /*enable bq2416x charger high impedance mode*/
            bq2416x_write_byte(di, di->control_reg | di->hz_mode, REG_CONTROL_REGISTER);
            di->hz_mode = 0; /*disable bq2416x charger high impedance mode*/
            msleep(700);
            bq2416x_config_control_reg(di);
            events = BQ2416x_START_CHARGING;
        } else{
            events = BQ2416x_CHARGE_DONE;
       }
     }
	else{
	     events=BQ2416x_START_CHARGING;
		dev_dbg(di->dev, "BQ2416x_START_CHARGING !\n");
	}
	if(di->charger_source == POWER_SUPPLY_TYPE_BATTERY){
		return;
	}
	blocking_notifier_call_chain(&notifier_list, events, NULL);
}

/*small current charging(100mA) when low battery voltage or low battery temprature*/
static void
bq2416x_low_current_charge(struct bq2416x_device_info *di)
{
	int battery_voltage = 0;
	int battery_temperature = 0;
	if (!is_bq27510_battery_exist(g_battery_measure_by_bq27510_device))
		return;

		battery_voltage = bq27510_battery_voltage(g_battery_measure_by_bq27510_device);
		battery_temperature = bq27510_battery_temperature(g_battery_measure_by_bq27510_device);

		if (battery_temperature < BQ2416x_COLD_BATTERY_THRESHOLD){
			dev_dbg(di->dev, "battery temp less than -10 degree,disable charging\n");
			di->enable_ce = DISABLE_CE;
			if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
			   if(di->calling_limit){
					di->cin_limit = CURRENT_AC_LIMIT_IN_800;
			   }
			   else{
					di->cin_limit = CURRENT_AC_LIMIT_IN;
			   }
			}

			else
				di->cin_limit = CURRENT_USB_LIMIT_IN;
		}
		else if ((battery_temperature >= BQ2416x_COLD_BATTERY_THRESHOLD)
			&& (battery_temperature < BQ2416x_COOL_BATTERY_THRESHOLD)){
			/*battery temp is between -10 and 0 degree,or battery voltage is less than 3.0V*/
			di->enable_low_chg = ENABLE_LOW_CHG;/*enable low charge,100mA charging*/
			bq2416x_config_safety_reg(di);
			if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
			   if(di->calling_limit){
					di->cin_limit = CURRENT_AC_LIMIT_IN_800;
			   }
			   else{
					di->cin_limit = CURRENT_AC_LIMIT_IN;
			   }
			}
			else
				di->cin_limit = CURRENT_USB_LIMIT_IN;
			if(battery_voltage < BQ2416x_LOW_TEMP_TERM_VOLTAGE){
				di->enable_ce = ENABLE_CE;
			}
			else{
				di->enable_ce = DISABLE_CE;
			}
		}
		else if ((battery_temperature >= BQ2416x_COOL_BATTERY_THRESHOLD)
		    && (battery_temperature < (BQ2416x_WARM_BATTERY_THRESHOLD - TEMPERATURE_OFFSET))){
			/*battery temp is between 00 and 50 degree,normal charge*/
			di->enable_low_chg = DISABLE_LOW_CHG;/*normal charge*/
			bq2416x_config_safety_reg(di);
			di->enable_ce = ENABLE_CE;
			if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
			   if(di->calling_limit){
					di->cin_limit = CURRENT_AC_LIMIT_IN_800;
			   }
			   else{
					di->cin_limit = CURRENT_AC_LIMIT_IN;
				}
			}
			else
				di->cin_limit = CURRENT_USB_LIMIT_IN;
		}
		else if ((battery_temperature >= BQ2416x_WARM_BATTERY_THRESHOLD)
		    && (battery_temperature < BQ2416x_HOT_BATTERY_THRESHOLD )){
			/*battery temp is between 00 and 50 degree,normal charge*/
			di->enable_low_chg = DISABLE_LOW_CHG;/*normal charge*/
			bq2416x_config_safety_reg(di);
			di->enable_ce = ENABLE_CE;
			if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
				di->cin_limit = CURRENT_AC_LIMIT_IN_800;
			else
				di->cin_limit = CURRENT_USB_LIMIT_IN;
		}
		else if (battery_temperature >= BQ2416x_HOT_BATTERY_THRESHOLD)
		{
			dev_dbg(di->dev, "battery temp more than 50 degree,disable charging\n");
			di->enable_ce = DISABLE_CE;
			if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
				di->cin_limit = CURRENT_AC_LIMIT_IN_800;
			else
				di->cin_limit = CURRENT_USB_LIMIT_IN;
		}
		else{
			di->cin_limit = di->cin_limit;

		}
		di->enable_ce = (di->enable_ce | di->factory_flag);
		bq2416x_config_control_reg(di);
	return;
}

void bq2416x_open_inner_fet(struct bq2416x_device_info *di)
{
    	u8 en_nobatop = 0;
		
		bq2416x_read_byte(di, &en_nobatop, REG_BATTERY_AND_SUPPLY_STATUS);
       if(g_battery_measure_by_bq27510_device &&
          is_bq27510_battery_exist(g_battery_measure_by_bq27510_device) ){
        	di->enable_iterm = ENABLE_ITERM;
        	en_nobatop = en_nobatop & (~EN_NOBATOP);
        }else {
        	di->enable_iterm = DISABLE_ITERM;
        	en_nobatop = en_nobatop | EN_NOBATOP;
        }
	bq2416x_config_control_reg(di);
	bq2416x_write_byte(di, en_nobatop, REG_BATTERY_AND_SUPPLY_STATUS);
}

static void bq2416x_start_usb_charger(struct bq2416x_device_info *di)
{
    long int  events;

      /*set gpio_174 low level for CD pin to enable bq24161 IC*/
	gpio_set_value(BQ2416X_GPIO_174, 0);
	events = BQ2416x_START_USB_CHARGING;
	blocking_notifier_call_chain(&notifier_list, events, NULL);
	di->enable_ce = ENABLE_CE;    /*enable charger*/
	di->enable_iterm = ENABLE_ITERM; /*enable charge current termination*/

    if(! g_battery_measure_by_bq27510_device)
        	return ;  
	di->charger_source = POWER_SUPPLY_TYPE_USB;
	di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	di->calling_limit = set_zero;
	di->dppm_voltagemV = VOLT_DPPM_ADJUST; 
	di->cin_limit = CURRENT_USB_LIMIT_IN;
	di->currentmA = CURRENT_USB_CHARGE_IN;
	bq2416x_config_control_reg(di);
	bq2416x_config_voltage_reg(di);
	bq2416x_config_current_reg(di);
       bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);			
	schedule_delayed_work(&di->bq2416x_charger_work,
						msecs_to_jiffies(0));
	dev_info(di->dev,"%s, ---->START USB CHARGING, \n"
	                  "battery current = %d mA\n"
	                  "battery voltage = %d mV\n"
	                  , __func__, di->currentmA, di->voltagemV);

	if (!is_bq27510_battery_exist(g_battery_measure_by_bq27510_device)){
		dev_dbg(di->dev, "BATTERY NOT DETECTED!\n");
		events = BQ2416x_NOT_CHARGING;
		blocking_notifier_call_chain(&notifier_list, events, NULL);
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		di->enable_low_chg = DISABLE_LOW_CHG;
		bq2416x_config_safety_reg(di);
	}
}

static void bq2416x_start_ac_charger(struct bq2416x_device_info *di)
{
       long int  events;
	   
      /*set gpio_174 low level for CD pin to enable bq24161 IC*/
	gpio_set_value(BQ2416X_GPIO_174, 0);
	events = BQ2416x_START_AC_CHARGING;
	blocking_notifier_call_chain(&notifier_list, events, NULL);
	di->enable_ce = ENABLE_CE;    /*enable charger*/
	di->enable_iterm = ENABLE_ITERM; /*enable charge current termination*/
	
	if(! g_battery_measure_by_bq27510_device)
        	return ;
	di->charger_source = POWER_SUPPLY_TYPE_MAINS;
	di->charge_status = POWER_SUPPLY_STATUS_CHARGING;

	di->calling_limit = set_zero;
	di->dppm_voltagemV = VOLT_DPPM_ADJUST_AC;
	di->cin_limit = CURRENT_AC_LIMIT_IN;
	di->currentmA = di->max_currentmA ;
	bq2416x_config_control_reg(di);
	bq2416x_config_voltage_reg(di);
	bq2416x_config_current_reg(di);
    bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);
#if BQ2416X_USE_WAKE_LOCK
	wake_lock(&di->charger_wake_lock);
#endif
	schedule_delayed_work(&di->bq2416x_charger_work,
						msecs_to_jiffies(0));

	dev_info(di->dev,"%s, ---->START AC CHARGING, \n"
	                  "battery current = %d mA\n"
	                  "battery voltage = %d mV\n"
	                  , __func__, di->currentmA, di->voltagemV);

	if (!is_bq27510_battery_exist(g_battery_measure_by_bq27510_device)){
		dev_dbg(di->dev, "BATTERY NOT DETECTED!\n");
		events = BQ2416x_NOT_CHARGING;
		blocking_notifier_call_chain(&notifier_list, events, NULL);
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		di->enable_low_chg = DISABLE_LOW_CHG;
		bq2416x_config_safety_reg(di);
	}
}

static void bq2416x_stop_charger(struct bq2416x_device_info *di)
{
       long int  events;

	dev_info(di->dev,"%s,---->STOP CHARGING\n", __func__);

	di->calling_limit = set_zero;
		di->enable_hotcold_temp_charge = set_one;
	di->factory_flag = set_zero; 
	di->enable_ce = DISABLE_CE;
	di->hz_mode = set_zero; /*not high impedance mode*/
	bq2416x_config_control_reg(di);
#if BQ2416X_USE_WAKE_LOCK
       if (POWER_SUPPLY_TYPE_MAINS == di->charger_source)
	    wake_unlock(&di->charger_wake_lock);
#endif
	di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
	di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	cancel_delayed_work_sync(&di->bq2416x_charger_work);
	events = BQ2416x_STOP_CHARGING;
	blocking_notifier_call_chain(&notifier_list, events, NULL);
	/*set gpio_174 high level for CD pin to disable bq24161 IC */
	gpio_set_value(BQ2416X_GPIO_174, 1);
}

//r00186667, 2011/08/02, removed event handler from pmic. begin	
#if 0
static int bq2416x_charger_event(struct notifier_block *nb, unsigned long event,
				void *_data)
{

	printk("%s,---->enter bq2416x_charger_event\n", __func__);

	struct bq2416x_device_info *di;
	struct charge_params *data;
	u8 read_reg[8] = {0};
	int ret = 0;

	di = container_of(nb, struct bq2416x_device_info, nb);
	data = &di->params;
	di->cfg_params = 1;

	if (event & BQ2416x_CHARGER_FAULT) {
		bq2416x_read_block(di, &read_reg[0], 0, 8);
		ret = read_reg[0] & 0x7F;
		return ret;
	}

	if (data->enable == 0) {
		di->currentmA = data->currentmA;
		di->voltagemV = data->voltagemV;
		di->enable_iterm = data->enable_iterm;
	}
       if ((event & BQ2416x_DEFAULT_USB_CHARGING ) && (di->active == 0)) {
		 di->cin_limit = 500;
	       bq2416x_config_control_reg(di);
	       bq2416x_config_voltage_reg(di);
	       bq2416x_config_current_reg(di);
		bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);	   	
		schedule_delayed_work(&di->bq2416x_charger_work,
						msecs_to_jiffies(0));
		di->active = 1;	   	
       }
	if ((event & BQ2416x_START_AC_CHARGING) && (di->active == 0)) {
		di->cin_limit = 1000;
	       bq2416x_config_control_reg(di);
	       bq2416x_config_voltage_reg(di);
	       bq2416x_config_current_reg(di);
		bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);	
		printk("%s, charging with VAC\n", __func__);	
		schedule_delayed_work(&di->bq2416x_charger_work,
						msecs_to_jiffies(0));
		di->active = 1;
	}
	
	if ((event & BQ2416x_START_USB_CHARGING) && (di->active == 0)) { 
		di->cin_limit = 500;
	       bq2416x_config_control_reg(di);
	       bq2416x_config_voltage_reg(di);
	       bq2416x_config_current_reg(di);
		bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);		
		printk("%s, charging with USB\n", __func__);		
		schedule_delayed_work(&di->bq2416x_charger_work,
						msecs_to_jiffies(0));
		di->active = 1;
	}

	if (event & BQ2416x_STOP_CHARGING) {
		cancel_delayed_work(&di->bq2416x_charger_work);
		di->active = 0;
	}

	if (event & BQ2416x_RESET_TIMER) {
		/* reset 32 second timer */
		bq2416x_config_status_reg(di);
	}

	return ret;
}
#endif
//r00186667, 2011/08/02, removed event handler from pmic. end	

static void
bq2416x_charger_update_status(struct bq2416x_device_info *di)
{
	u8 read_reg[8] = {0};

	timer_fault = 0;
	bq2416x_read_block(di, &read_reg[0], 0, 8); 

	if ((read_reg[0] & 0x70) == 0x50)   
		dev_dbg(di->dev, "CHARGE DONE\n");

	if ((read_reg[0] & 0x7) == 0x4) 
		timer_fault = 1;
     
	if (read_reg[0] & 0x7) {
		di->cfg_params = 1;
		dev_err(di->dev, "CHARGER STATUS = %x\n", read_reg[0]);
	}

    if ((read_reg[1] & 0x6) == 0x2) {
        di->hz_mode = 1;
        bq2416x_config_control_reg(di);
        bq2416x_write_byte(di, di->voltage_reg, REG_BATTERY_VOLTAGE);
        dev_err(di->dev, "battery ovp = %x,%x\n", read_reg[1],read_reg[3]);
        msleep(700);
        di->hz_mode = 0;
        bq2416x_config_control_reg(di);
    }

	if (is_bq27510_battery_exist(g_battery_measure_by_bq27510_device)){
		bq2416x__charge_status(di);
	}

	if ((timer_fault == 1) || (di->cfg_params == 1)) {
		bq2416x_write_byte(di, di->control_reg, REG_CONTROL_REGISTER);
		bq2416x_write_byte(di, di->voltage_reg, REG_BATTERY_VOLTAGE);
		bq2416x_write_byte(di, di->current_reg, REG_BATTERY_CURRENT);
		bq2416x_write_byte(di, di->dppm_reg, REG_DPPM_VOLTAGE);
		bq2416x_config_safety_reg(di); 
		di->cfg_params = 0;
	}

	/* reset 32 second timer */
	bq2416x_config_status_reg(di);

	return;
}

static void bq2416x_charger_work(struct work_struct *work)
{
	struct bq2416x_device_info *di = container_of(work,
		struct bq2416x_device_info, bq2416x_charger_work.work);

	bq2416x_open_inner_fet(di);
	if(di->enable_hotcold_temp_charge){
		bq2416x_low_current_charge(di);
	}
	bq2416x_charger_update_status(di);
	schedule_delayed_work(&di->bq2416x_charger_work,
						msecs_to_jiffies(BQ2416x_WATCHDOG_TIMEOUT));
}

static ssize_t bq2416x_set_enable_itermination(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
		return -EINVAL;
	di->enable_iterm = val;
	bq2416x_config_control_reg(di);

	return status;
}

static ssize_t bq2416x_show_enable_itermination(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->enable_iterm;
	return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2416x_set_cin_limit(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 100)
					|| (val > di->max_currentmA))
		return -EINVAL;
	di->cin_limit = val;
	bq2416x_config_control_reg(di);

	return status;
}

static ssize_t bq2416x_show_cin_limit(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->cin_limit;
	return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2416x_set_regulation_voltage(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
					|| (val > di->max_voltagemV))
		return -EINVAL;
	di->voltagemV = val;
	bq2416x_config_voltage_reg(di);

	return status;
}

static ssize_t bq2416x_show_regulation_voltage(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->voltagemV;
	return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2416x_set_charge_current(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 550)
					|| (val > di->max_currentmA))
		return -EINVAL;
	di->currentmA = val;
	bq2416x_config_current_reg(di);

	return status;
}

static ssize_t bq2416x_show_charge_current(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->currentmA;
	return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2416x_set_termination_current(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 350))
		return -EINVAL;
	di->term_currentmA = val;
	bq2416x_config_current_reg(di);

	return status;
}

static ssize_t bq2416x_show_termination_current(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->term_currentmA;
	return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2416x_set_dppm_voltage(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);
	if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
		return -EINVAL;
	di->dppm_voltagemV = val;
	bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);

	return status;
}

static ssize_t bq2416x_show_dppm_voltage(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->dppm_voltagemV;
	return sprintf(buf, "%lu\n", val);
}
/*
* set 1 --- enable_charger; 0 --- disable charger
*
*/
static ssize_t bq2416x_set_enable_charger(struct device *dev,	
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);
	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
		return -EINVAL;
	di->enable_ce = val ^ 0x1;
	bq2416x_config_control_reg(di);
	di->factory_flag = di->enable_ce;
	bq2416x__charge_status(di);
	return status;
}

static ssize_t bq2416x_show_enable_charger(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->enable_ce ^ 0x1;
	return sprintf(buf, "%lu\n", val);
}

/*
* set 1 --- hz_mode ; 0 --- not hz_mode 
*
*/
static ssize_t bq2416x_set_enable_hz_mode(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
		return -EINVAL;
	di->hz_mode= val;
	bq2416x_config_control_reg(di);

	return status;
}
static ssize_t bq2416x_show_enable_hz_mode(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->hz_mode;
	return sprintf(buf, "%lu\n", val);	
}

/*
* set 1 --- enable bq24161 IC; 0 --- disable bq24161 IC
*
*/
static ssize_t bq2416x_set_enable_cd(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
		return -EINVAL;
	di->cd_active =val ^ 0x1;
	gpio_set_value(BQ2416X_GPIO_174, di->cd_active);
	return status;
}
static ssize_t bq2416x_show_enable_cd(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->cd_active ^ 0x1;
	return sprintf(buf, "%lu\n", val);	
}
static ssize_t bq2416x_show_chargelog(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
        int i = 0;
        u8   read_reg[8] = {0};
        u8   buf_temp[20] = {0};
        struct bq2416x_device_info *di = dev_get_drvdata(dev);
        bq2416x_read_block(di, &read_reg[0], 0, 8);
        for(i=0;i<8;i++)
        {
            sprintf(buf_temp,"0x%-8.2x",read_reg[i]);
            strcat(buf,buf_temp);
        }
        strcat(buf,"\n");
	return strlen(buf);
}

static ssize_t bq2416x_set_calling_limit(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);
	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
		return -EINVAL;

	di->calling_limit = val;
	if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
		if(di->calling_limit){
			di->cin_limit = CURRENT_AC_LIMIT_IN_800;
			bq2416x_config_control_reg(di);
			dev_info(di->dev,"calling_limit_current = %d\n", di->cin_limit);
		}
	}
	else{
		di->calling_limit = 0;
	}

	return status;
}

static ssize_t bq2416x_show_calling_limit(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->calling_limit;

	return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2416x_set_enable_hotcold_temp_charge(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	int status = count;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
		return -EINVAL;
	di->enable_hotcold_temp_charge = val;

	return status;
}
static ssize_t bq2416x_show_enable_hotcold_temp_charge(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	unsigned long val;
	struct bq2416x_device_info *di = dev_get_drvdata(dev);

	val = di->enable_hotcold_temp_charge;
	return sprintf(buf, "%lu\n", val);
}

static DEVICE_ATTR(enable_hotcold_temp_charge, S_IWUSR | S_IRUGO,
				bq2416x_show_enable_hotcold_temp_charge,
				bq2416x_set_enable_hotcold_temp_charge);

static DEVICE_ATTR(enable_cd, S_IWUSR | S_IRUGO,
				bq2416x_show_enable_cd,
				bq2416x_set_enable_cd);

static DEVICE_ATTR(enable_itermination, S_IWUSR | S_IRUGO,
				bq2416x_show_enable_itermination,
				bq2416x_set_enable_itermination);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO,
				bq2416x_show_cin_limit,
				bq2416x_set_cin_limit);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
				bq2416x_show_regulation_voltage,
				bq2416x_set_regulation_voltage);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
				bq2416x_show_charge_current,
				bq2416x_set_charge_current);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
				bq2416x_show_termination_current,
				bq2416x_set_termination_current);
//r00186667, 2011/08/02, add test interface. begin				
static DEVICE_ATTR(enable_charger, S_IWUSR | S_IRUGO,
				bq2416x_show_enable_charger,
				bq2416x_set_enable_charger);

static DEVICE_ATTR(enable_hz_mode, S_IWUSR | S_IRUGO,
				bq2416x_show_enable_hz_mode,
				bq2416x_set_enable_hz_mode);

static DEVICE_ATTR(dppm_voltage, S_IWUSR | S_IRUGO,
				bq2416x_show_dppm_voltage,
				bq2416x_set_dppm_voltage);				
//r00186667, 2011/08/02, add test interface. end

static DEVICE_ATTR(chargelog, S_IWUSR | S_IRUGO,
				bq2416x_show_chargelog,
				NULL);

static DEVICE_ATTR(calling_limit, S_IWUSR | S_IRUGO,
				bq2416x_show_calling_limit,
				bq2416x_set_calling_limit);

static struct attribute *bq2416x_attributes[] = {
	&dev_attr_enable_itermination.attr,
	&dev_attr_cin_limit.attr,
	&dev_attr_regulation_voltage.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_termination_current.attr,
	&dev_attr_dppm_voltage.attr,    
	&dev_attr_enable_charger.attr,  
	&dev_attr_enable_hz_mode.attr, 
	&dev_attr_enable_cd.attr,
	&dev_attr_chargelog.attr,
	&dev_attr_enable_hotcold_temp_charge.attr,
	&dev_attr_calling_limit.attr,
	NULL,
};

static const struct attribute_group bq2416x_attr_group = {
	.attrs = bq2416x_attributes,
};

static void bq2416x_usb_charger_work(struct work_struct *work)
{
	struct bq2416x_device_info	*di =
		container_of(work, struct bq2416x_device_info, usb_work);
	switch (di->event) {
	case USB_EVENT_CHARGER:
		bq2416x_start_ac_charger(di);
		break;
	case USB_EVENT_VBUS:
		bq2416x_start_usb_charger(di);
		break;
	case USB_EVENT_NONE:
		bq2416x_stop_charger(di);
		break;
	case USB_EVENT_ENUMERATED:
		break;
	default:
		return;
	}
}
//r00186667, 2011/08/02, add usb notifier callback. begin
static int bq2416x_usb_notifier_call(struct notifier_block *nb_otg,
		unsigned long event, void *data)
{
	struct bq2416x_device_info *di = 
		container_of(nb_otg, struct bq2416x_device_info, nb_otg);

	di->event = event;
	switch (event) {	
	case USB_EVENT_VBUS:
		break;
	case USB_EVENT_ENUMERATED:
		break;
	case USB_EVENT_CHARGER:
	    	break;
	case USB_EVENT_NONE:
	    	break;
	case USB_EVENT_ID:
	    	break;
	default:
		return NOTIFY_OK;
	}
	schedule_work(&di->usb_work);
	return NOTIFY_OK;
}
//r00186667, 2011/08/02, add usb notifier callback. end

static int bq2416x_get_max_charge_voltage(struct bq2416x_device_info *di)
{
	bool ret = 0;

	ret = get_hw_config_int("gas_gauge/charge_voltage", &di->max_voltagemV , NULL);
	if(ret){
		if(di->max_voltagemV < 4200){
			di->max_voltagemV = 4200;
		}
		return true;
	}
	else{
		dev_err(di->dev, " bq2416x_get_max_charge_voltage from boardid fail \n");
		return false;
	}
}

static int bq2416x_get_max_charge_current(struct bq2416x_device_info *di)
{
	bool ret = 0;

	ret = get_hw_config_int("gas_gauge/charge_current", &di->max_currentmA , NULL);
	if(ret){
		if(di->max_currentmA < 1000){
			di->max_currentmA = 1000;
		}
		return true;
	}
	else{
		dev_err(di->dev, " bq2416x_get_max_charge_current from boardid fail \n");
		return false;
	}
}

static int __devinit bq2416x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2416x_device_info *di;
	struct bq2416x_platform_data *pdata = client->dev.platform_data;
	int ret;
	u8 read_reg = 0;
	enum plugin_status plugin_stat;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &client->dev;
	di->client = client;

	i2c_set_clientdata(client, di);

	ret = bq2416x_read_byte(di, &read_reg, REG_PART_REVISION);

	if (ret < 0) {
		dev_err(&client->dev, "chip not present at address %x\n",
								client->addr);
		ret = -EINVAL;
		goto err_kfree;
	}
#if 0
	if ((read_reg & 0x18) == 0x00 && (client->addr == 0x6a))
		di->bqchip_version = BQ24156;
#endif 
	
	if ((read_reg & 0x18) == 0x00 && (client->addr == 0x6b))
		di->bqchip_version = BQ24161;

	if (di->bqchip_version == 0) {
		dev_dbg(&client->dev, "unknown bq chip\n");
		dev_dbg(&client->dev, "Chip address %x", client->addr);
		dev_dbg(&client->dev, "bq chip version reg value %x", read_reg);
		ret = -EINVAL;
		goto err_kfree;
	}


	//	di->nb.notifier_call = bq2416x_charger_event;
	
#if 0
	 bq2415x_config_safety_reg(di, pdata->max_charger_currentmA,
						pdata->max_charger_voltagemV);
	 di->cin_limit = 900;
	 di->term_currentmA = pdata->termination_currentmA;
	 bq2415x_config_control_reg(di);
	 bq2415x_config_voltage_reg(di);
	 bq2415x_config_current_reg(di);
#endif 
       /*set gpio_174 to control CD pin to disable/enable bq24161 IC*/
	 gpio_request(BQ2416X_GPIO_174, "gpio_174_cd");
	 /* set charger CD pin to low level and enable it to supply power normally*/
	 gpio_direction_output(BQ2416X_GPIO_174, 0);
     ret = bq2416x_get_max_charge_voltage(di);
	 if(!ret){
		di->max_voltagemV = pdata->max_charger_voltagemV;
	 }

	 di->voltagemV = di->max_voltagemV;

	 ret = bq2416x_get_max_charge_current(di);
	 if(!ret){
		di->max_currentmA = pdata->max_charger_currentmA;
	 }

	 di->currentmA = CURRENT_USB_CHARGE_IN ;
        di->term_currentmA = CURRENT_TERM_CHARGE_IN;
	 di->dppm_voltagemV = VOLT_DPPM_ADJUST;
	 di->cin_limit = CURRENT_USB_LIMIT_IN; 


		di->enable_hotcold_temp_charge = set_one;
	di->enable_low_chg = DISABLE_LOW_CHG;/*set normally charge mode*/
	di->enable_iterm = ENABLE_ITERM; /*enable charge current termination*/
	di->factory_flag = 0;
	 di->enable_ce = ENABLE_CE;
	 di->hz_mode = 0;
	 di->cd_active = 0;
	 
	INIT_DELAYED_WORK_DEFERRABLE(&di->bq2416x_charger_work,
				bq2416x_charger_work);

#if BQ2416X_USE_WAKE_LOCK
	wake_lock_init(&di->charger_wake_lock, WAKE_LOCK_SUSPEND, "charger_wake_lock");
#endif

	//BLOCKING_INIT_NOTIFIER_HEAD(&notifier_list);

	di->active = 0;
	di->params.enable = 1;
	di->cfg_params = 1;
	
	 bq2416x_config_control_reg(di);
	 bq2416x_config_voltage_reg(di);
	 bq2416x_config_current_reg(di);
	 bq2416x_config_dppm_voltage_reg(di,di->dppm_voltagemV);	
	 bq2416x_config_safety_reg(di);
	
#if 0
	ret = bq2416x_read_byte(di, &read_reg, REG_SPECIAL_CHARGER_VOLTAGE);
	if (!(read_reg & 0x08)) {
		di->active = 1;
		schedule_delayed_work(&di->bq2415x_charger_work, 0);
	}
#endif

	ret = sysfs_create_group(&client->dev.kobj, &bq2416x_attr_group);
	if (ret)
		dev_dbg(&client->dev, "could not create sysfs files\n");


	//twl6030_register_notifier(&di->nb, 1);
	INIT_WORK(&di->usb_work, bq2416x_usb_charger_work);

//r00186667, 2011/08/02, otg register for receive USB/AC plugin event.begin   
	di->nb_otg.notifier_call = bq2416x_usb_notifier_call;
	di->otg = otg_get_transceiver();
	ret = otg_register_notifier(di->otg, &di->nb_otg);
	if (ret)
		dev_err(&client->dev, "otg register notifier failed %d\n", ret);
//r00186667, 2011/08/02, otg register for receive USB/AC plugin event.end  

//r00186667, 2011/08/02, get the boot event type.begin  
    plugin_stat = get_plugin_device_status();
    if( PLUGIN_USB_CHARGER == plugin_stat){
		di->event = USB_EVENT_VBUS;
    }else if (PLUGIN_AC_CHARGER == plugin_stat){
         di->event = USB_EVENT_CHARGER;
    }else{
		di->event = USB_EVENT_NONE;
	}
	schedule_work(&di->usb_work);
//r00186667, 2011/08/02, get the boot event type.end  
	return 0;

err_kfree:
	kfree(di);	
	
	return ret;
}

static int __devexit bq2416x_charger_remove(struct i2c_client *client)
{
	struct bq2416x_device_info *di = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &bq2416x_attr_group);
#if BQ2416X_USE_WAKE_LOCK
	wake_lock_destroy(&di->charger_wake_lock);
#endif
	cancel_delayed_work_sync(&di->bq2416x_charger_work);
	//flush_scheduled_work();
	//twl6030_unregister_notifier(&di->nb, 1);
	otg_unregister_notifier(di->otg, &di->nb_otg);
	kfree(di);

	return 0;
}

static const struct i2c_device_id bq2416x_id[] = {
	{ "bq24161", 0 },
	{},
};

#ifdef CONFIG_PM
static int bq2416x_charger_suspend(struct i2c_client *client,
	pm_message_t state)
{
	return 0;
}

static int bq2416x_charger_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define bq2416x_charger_suspend	NULL
#define bq2416x_charger_resume	NULL
#endif /* CONFIG_PM */

static struct i2c_driver bq2416x_charger_driver = {
	.probe		= bq2416x_charger_probe,
	.remove		= __devexit_p(bq2416x_charger_remove),
	.suspend	= bq2416x_charger_suspend,
	.resume		= bq2416x_charger_resume,
	.id_table	= bq2416x_id,
	.driver		= {
		.name	= "bq2416x_charger",
	},
};

static int __init bq2416x_charger_init(void)
{
	return i2c_add_driver(&bq2416x_charger_driver);
}
module_init(bq2416x_charger_init);

static void __exit bq2416x_charger_exit(void)
{
	i2c_del_driver(&bq2416x_charger_driver);
}
module_exit(bq2416x_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
