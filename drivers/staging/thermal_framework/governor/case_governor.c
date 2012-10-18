/*
 * drivers/thermal_framework/governor/case_governor.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Sebastien Sabatier <s-sabatier1@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/
#include <linux/err.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/thermal_framework.h>

/* System/Case Thermal thresholds */
#define INIT_COOLING_LEVEL    100
#define COMFORT_COOLING_LEVEL    102
#define WARM_COOLING_LEVEL    103
#define WARM_THRESHOLD_HOT  60000
#define WARM_THRESHOLD_COLD  57000
#define COMFORT_THRESHOLD_HOT 55000
#define COMFORT_THRESHOLD_COLD 52000
int sys_threshold_hot = COMFORT_THRESHOLD_HOT;
int sys_threshold_cold = COMFORT_THRESHOLD_COLD;
static uint32_t warm_flag = 0;
struct case_governor {
	struct thermal_dev *temp_sensor;
	int cooling_level;
};

static struct thermal_dev *therm_fw;
static struct case_governor *case_gov;

static LIST_HEAD(cooling_agents);

/**
 * DOC: Introduction
 * =================
 * The SYSTEM Thermal governor maintains the policy for the SYSTEM
 * temperature (PCB and/or case). The main goal of the governor is to
 * get the temperature from a thermal sensor and calculate the current
 * temperature of the case. When the case temperature is above the hot
 * limit, then the SYSTEM Thermal governor will cool the system by
 * throttling CPU frequency.
 * When the temperature is below the cold limit, then the SYSTEM Thermal
 * governor will remove any constraints about CPU frequency.
 * The SYSTEM Thermal governor may use 3 different sensors :
 * - CPU (OMAP) on-die temperature sensor (if there is no PCB sensor)
 * - PCB sensor located closed to CPU die
 * - Dedicated sensor for case temperature management
 * To take into account the response delay between the case temperature
 * and the temperature from one of these sensors, the sensor temperature
 * should be averaged.
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @temp:	Temperature (average on-die CPU temp or PCB temp sensor)
*/

static int case_thermal_manager(struct list_head *cooling_list, int temp)
{
	struct thermal_dev *cooling_dev, *tmp;
	/* TO DO: need to build an algo to find the right cooling agent */
	list_for_each_entry_safe(cooling_dev, tmp, cooling_list, node) {
		if (cooling_dev->dev_ops &&
			cooling_dev->dev_ops->cool_device) {
			/* TO DO: Add cooling agents to a list here */
			list_add(&cooling_dev->node, &cooling_agents);
			goto out;
		} else {
			pr_info("%s:Cannot find cool_device for %s\n",
				__func__, cooling_dev->name);
		}
	}

out:
	if (list_empty(&cooling_agents)) {
		pr_err("%s: No Cooling devices registered\n",
			__func__);
		return -ENODEV;
	} else {

		if ((temp >= WARM_THRESHOLD_HOT) ||((temp >= WARM_THRESHOLD_COLD) &&(warm_flag == 1))){
				sys_threshold_hot = WARM_THRESHOLD_HOT;
				sys_threshold_cold = WARM_THRESHOLD_COLD;
				warm_flag = 1;
		}else{
				sys_threshold_hot = COMFORT_THRESHOLD_HOT;
				sys_threshold_cold = COMFORT_THRESHOLD_COLD;
				warm_flag = 0;
		}

		thermal_update_temp_thresholds(case_gov->temp_sensor,
				sys_threshold_cold, sys_threshold_hot);
		if (temp >= sys_threshold_hot) {
				if (sys_threshold_hot == COMFORT_THRESHOLD_HOT ){
					if(case_gov->cooling_level < COMFORT_COOLING_LEVEL){
						case_gov->cooling_level++;
						thermal_cooling_set_level(&cooling_agents, case_gov->cooling_level);
					}

				}else if (sys_threshold_hot == WARM_THRESHOLD_HOT ){
					if(case_gov->cooling_level < WARM_COOLING_LEVEL){
						case_gov->cooling_level++;
						thermal_cooling_set_level(&cooling_agents, case_gov->cooling_level);
					}
				}

		} else if (temp <= sys_threshold_cold) {

					if (case_gov->cooling_level > INIT_COOLING_LEVEL){
						case_gov->cooling_level = INIT_COOLING_LEVEL;
						thermal_cooling_set_level(&cooling_agents, case_gov->cooling_level);
					}
		}

			list_del_init(&cooling_agents);

	}
	return 0;
}


static int case_process_temp(struct list_head *cooling_list,
				struct thermal_dev *temp_sensor,
				int temp)
{
	int ret;
	case_gov->temp_sensor = temp_sensor;
	ret = case_thermal_manager(cooling_list, temp);

	return ret;
}

static struct thermal_dev_ops case_gov_ops = {
	.process_temp = case_process_temp,
};

static int __init case_governor_init(void)
{
	struct thermal_dev *thermal_fw;
	case_gov = kzalloc(sizeof(struct case_governor), GFP_KERNEL);
	if (!case_gov) {
		pr_err("%s:Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}

	thermal_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (thermal_fw) {
		thermal_fw->name = "case_governor";
		thermal_fw->domain_name = "case";
		thermal_fw->dev_ops = &case_gov_ops;
		thermal_governor_dev_register(thermal_fw);
		therm_fw = thermal_fw;
	} else {
		pr_err("%s: Cannot allocate memory\n", __func__);
		kfree(case_gov);
		return -ENOMEM;
	}

	case_gov->cooling_level = INIT_COOLING_LEVEL;
	return 0;
}

static void __exit case_governor_exit(void)
{
	thermal_governor_dev_unregister(therm_fw);
	kfree(therm_fw);
	kfree(case_gov);
}

module_init(case_governor_init);
module_exit(case_governor_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("System/Case thermal governor");
MODULE_LICENSE("GPL");
