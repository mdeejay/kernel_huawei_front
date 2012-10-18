/*
 * Copyright 2010 HUAWEI Tech. Co., Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Optical Finger Navigation driver
 *
 */


#ifndef __BQ27510_BATTERY_H
 #define __BQ27510_BATTERY_H

#include <linux/power_supply.h>

 struct bq27510_device_info {
	struct device 		*dev;
	int			id;
	struct power_supply	bat;
	struct i2c_client	*client;
	struct delayed_work	notifier_work;
	unsigned long  timeout_jiffies;
};

/*external functions*/ 
extern int bq27510_battery_temperature(struct bq27510_device_info *di);
extern int bq27510_battery_voltage(struct bq27510_device_info *di);
//extern int bq27510_battery_current(struct bq27510_device_info *di);
extern short bq27510_battery_current(struct bq27510_device_info *di);
extern int bq27510_battery_tte(struct bq27510_device_info *di);
extern int bq27510_battery_ttf(struct bq27510_device_info *di);
extern int is_bq27510_battery_full(struct bq27510_device_info *di);
extern int is_bq27510_battery_exist(struct bq27510_device_info *di);

extern int bq27510_battery_status(struct bq27510_device_info *di);
extern int bq27510_battery_health(struct bq27510_device_info *di);
extern int bq27510_battery_capacity(struct bq27510_device_info *di);
extern int bq27510_battery_capacity_level(struct bq27510_device_info *di);
extern int bq27510_battery_technology(struct bq27510_device_info *di);

extern int bq27510_battery_rm(struct bq27510_device_info *di);
extern int bq27510_battery_fcc(struct bq27510_device_info *di);
extern int is_bq27510_battery_reach_threshold(struct bq27510_device_info *di);

extern const char* bq27510_battery_get_firmware_version(struct bq27510_device_info *di);

extern int bq27510_battery_check_firmware_version(struct bq27510_device_info *di);
#endif

