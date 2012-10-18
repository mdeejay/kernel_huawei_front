/*
 * Case Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/thermal_framework.h>
#include <linux/types.h>
#include <linux/mutex.h>

#include <plat/common.h>
#include <plat/case_temperature_sensor.h>
#include <linux/i2c/twl6030-gpadc.h>

#include <linux/suspend.h>
#define CASE_REPORT_DELAY_MS	250
#define AVERAGE_NUMBER	       10
#define TWL6030_VOLT_START_VALUE 166
#define TWL6030_VOLT_END_VALUE   1175
#define TWL6030_GPADC_CHANNEL     4
#define NTC_NUM 131
#define TEMP_ZERO_VOLT 1124
#define INIT_T_HOT		20000
#define INIT_T_COLD		19000
/*
 * case_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 * @sensor_mutex - Mutex for sysfs, irq and PM
 */
struct case_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
	struct mutex sensor_mutex;
	struct spinlock lock;
	struct thermal_dev *therm_fw;
	struct delayed_work case_sensor_work;
	int work_delay;
	int avg_is_valid;
	int threshold_hot;
	int threshold_cold;
	int hot_event;
};
static struct case_temp_sensor *case_temp_sensor_pm;

static unsigned suspend_state;

static int sensor_temp_table[AVERAGE_NUMBER];
/*
 * Temperature values in milli degrees celsius voltage values from 1175 to 166
 */

static int volt_to_temp[NTC_NUM][2] ={
{-25000,1175}, {-24000,1174}, {-23000,1172}, {-22000,1171}, {-21000,1170},
{-20000,1169}, {-19000,1167}, {-18000,1166}, {-17000,1165}, {-16000,1163},
{-15000,1161}, {-14000,1160}, {-13000,1158}, {-12000,1156}, {-11000,1154},
{-10000,1152}, {-9000,1149}, {-8000,1147}, {-7000,1145}, {-6000,1142},
{-5000,1139}, {-4000,1137}, {-3000,1134}, {-2000,1131}, {-1000,1128},
{0,1124}, {1000,1121}, {2000,1117}, {3000,1113}, {4000,1110},
{5000,1106}, {6000,1101}, {7000,1097}, {8000,1093}, {9000,1088},
{10000,1083}, {11000,1078}, {12000,1073}, {13000,1068}, {14000,1062},
{15000,1056}, {16000,1050}, {17000,1044}, {18000,1038}, {19000,1032},
{20000,1025}, {21000,1018}, {22000,1011}, {23000,1004}, {24000,997},
{25000,989}, {26000,982}, {27000,974}, {28000,966}, {29000,958},
{30000,949}, {31000,941}, {32000,932}, {33000,923}, {34000,914},
{35000,905}, {36000,896}, {37000,886}, {38000,877}, {39000,867},
{40000,857}, {41000,847}, {42000,837}, {43000,827}, {44000,816},
{45000,806}, {46000,796}, {47000,785}, {48000,775}, {49000,764},
{50000,753}, {51000,742}, {52000,732}, {53000,721}, {54000,710},
{55000,699}, {56000,688}, {57000,677}, {58000,667}, {59000,656},
{60000,645}, {61000,634}, {62000,624}, {63000,613}, {64000,602},
{65000,592}, {66000,581}, {67000,571}, {68000,560}, {69000,550},
{70000,540}, {71000,530}, {72000,520}, {73000,510}, {74000,500},
{75000,490}, {76000,481}, {77000,471}, {78000,462}, {79000,453},
{80000,444}, {81000,435}, {82000,426}, {83000,417}, {84000,408},
{85000,400}, {86000,392}, {87000,383}, {88000,375}, {89000,368},
{90000,360}, {91000,352}, {92000,345}, {93000,337}, {94000,330},
{95000,323}, {96000,316}, {97000,309}, {98000,302}, {99000,296},
{100000,289}, {105000,259}, {110000,232}, {115000,207}, {120000,186},
{125000,166}
};

static int volt_to_temp_conversion(int volt_val)
{
	int NTC_temp = 0;
	int count;
	int i ;

	if ((volt_val < TWL6030_VOLT_START_VALUE) ||
		(volt_val > TWL6030_VOLT_END_VALUE)) {
		pr_err("%s:Temp read is invalid %i\n", __func__, volt_val);
		return -EINVAL;
	}

	count = ARRAY_SIZE(volt_to_temp);

	for(i=0;i<count;i++)
	{
		if( volt_val < volt_to_temp[i][1] && volt_val >= volt_to_temp[i+1][1])
		{
			if(volt_val <= TEMP_ZERO_VOLT)
			{
				NTC_temp = volt_to_temp[i+1][0];
			}
			else
			{
				 NTC_temp = volt_to_temp[i][0];
			}
		break;
		}
	}
	return NTC_temp;

}

static int case_read_current_temp(struct case_temp_sensor *temp_sensor)
{
	int temp = 0;
	struct twl6030_gpadc_request req;
	int val;

	if (suspend_state)
		return  -EINVAL;

	req.channels = (1 << TWL6030_GPADC_CHANNEL);
	req.method = TWL6030_GPADC_SW2;
	req.func_cb = NULL;
	val = twl6030_gpadc_conversion(&req);
	if (val < 0) {
		pr_err("%s:TWL6030_GPADC conversion is invalid %d\n",
			__func__, val);
		return -EINVAL;
	}
	temp = volt_to_temp_conversion(req.rbuf[TWL6030_GPADC_CHANNEL]);

	return temp;
}

static int case_read_average_temp(struct case_temp_sensor *temp_sensor)
{
	int i;
	int sensor_temp;
	int acc_temp;
	/* Read current temperature */
	sensor_temp = case_read_current_temp(temp_sensor);

	/* if on-die sensor does not report a correct value, then return */
	if (sensor_temp == -EINVAL)
		return -EINVAL;

	/* Update historical buffer */
	for (i = 1; i < AVERAGE_NUMBER; i++) {
		sensor_temp_table[AVERAGE_NUMBER - i] =
		sensor_temp_table[AVERAGE_NUMBER - i - 1];
	}
	sensor_temp_table[0] = sensor_temp;

	if (sensor_temp_table[AVERAGE_NUMBER - 1] == 0)
		temp_sensor->avg_is_valid = 0;
	else
		temp_sensor->avg_is_valid = 1;

	/* Compute the new average value */
	acc_temp = 0;
	for (i = 0; i < AVERAGE_NUMBER; i++)
		acc_temp += sensor_temp_table[i];

	acc_temp = (acc_temp / AVERAGE_NUMBER);

	for (i = 0; i < AVERAGE_NUMBER; i++) {
		pr_debug("sensor_temp_table[%d] = %d\n", i,
			sensor_temp_table[i]);
	}

	return acc_temp;
}

static int case_report_temp(struct thermal_dev *tdev)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct case_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int current_temp;
	current_temp = case_read_average_temp(temp_sensor);
	temp_sensor->therm_fw->current_temp = current_temp;

	pr_debug("%s: case temp %d valid %d\n", __func__,
		temp_sensor->therm_fw->current_temp,
		temp_sensor->avg_is_valid);
	if ((temp_sensor->therm_fw->current_temp != -EINVAL) &&
	    (temp_sensor->avg_is_valid == 1)) {
		if ((current_temp >= temp_sensor->threshold_hot) ) {
			thermal_sensor_set_temp(temp_sensor->therm_fw);
			kobject_uevent(&temp_sensor->dev->kobj, KOBJ_CHANGE);
		}

               if ((current_temp <= temp_sensor->threshold_cold) &&
			(temp_sensor->hot_event == 1)) {
			thermal_sensor_set_temp(temp_sensor->therm_fw);
			kobject_uevent(&temp_sensor->dev->kobj, KOBJ_CHANGE);
			temp_sensor->hot_event = 0;
		}
	}

	return temp_sensor->therm_fw->current_temp;
}

static int case_set_temp_thresh(struct thermal_dev *tdev, int min, int max)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct case_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	if (max < min) {
		pr_err("%s:Min is greater then the max\n", __func__);
		return -EINVAL;
	}
	temp_sensor->threshold_cold = min;
	temp_sensor->threshold_hot = max;
	pr_debug("%s:threshold_cold %d, threshold_hot %d\n", __func__,
		min, max);
	temp_sensor->hot_event = 1;
	return 0;
}

/*
 * sysfs hook functions
 */
static int case_temp_sensor_read_temp(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct case_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int temp = 0;
	temp = case_read_average_temp(temp_sensor);

	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR(temp1_input, S_IRUGO, case_temp_sensor_read_temp,
			  NULL);

static struct attribute *case_temp_sensor_attributes[] = {
	&dev_attr_temp1_input.attr,
	NULL
};

static const struct attribute_group case_temp_sensor_group = {
	.attrs = case_temp_sensor_attributes,
};

static int omap_case_temp_sensor_pm_notifier_cb(struct notifier_block *notifier,
				unsigned long pm_event,  void *unused)
{
	int r = 0;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		suspend_state = 1;
		if (cancel_delayed_work_sync
				(&case_temp_sensor_pm->case_sensor_work)) {
			pr_err("%s: Failed to cancel delayed work!\n",
				       __func__);
			r = -EBUSY;
		}
		break;
	case PM_POST_SUSPEND:
		suspend_state = 0;
		r = schedule_delayed_work(&case_temp_sensor_pm->case_sensor_work,
				msecs_to_jiffies(0));
		if (r)
			pr_err("%s: Failed schedule delayed work task!\n",
					__func__);
		break;
	}

	return NOTIFY_DONE;
}
static struct thermal_dev_ops case_sensor_ops = {
	.set_temp_thresh = case_set_temp_thresh,
};

static struct notifier_block omap_case_temp_sensor_pm_notifier = {
        .notifier_call = omap_case_temp_sensor_pm_notifier_cb,
};

static void case_sensor_delayed_work_fn(struct work_struct *work)
{
	struct case_temp_sensor *temp_sensor =
				container_of(work, struct case_temp_sensor,
					     case_sensor_work.work);

	if (suspend_state)
		return;

	case_report_temp(temp_sensor->therm_fw);
	schedule_delayed_work(&temp_sensor->case_sensor_work,
				msecs_to_jiffies(temp_sensor->work_delay));
}

static int __devinit case_temp_sensor_probe(struct platform_device *pdev)
{
	struct omap_temp_sensor_pdata *pdata = pdev->dev.platform_data;
	struct case_temp_sensor *temp_sensor;
	int ret = 0, i;
	int r;
	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(struct case_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	case_temp_sensor_pm = temp_sensor;
	/* Init delayed work for Case sensor temperature */
	INIT_DELAYED_WORK(&temp_sensor->case_sensor_work,
			  case_sensor_delayed_work_fn);

	spin_lock_init(&temp_sensor->lock);
	mutex_init(&temp_sensor->sensor_mutex);

	temp_sensor->pdev = pdev;
	temp_sensor->dev = &pdev->dev;

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	platform_set_drvdata(pdev, temp_sensor);

	temp_sensor->therm_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (temp_sensor->therm_fw) {
		temp_sensor->therm_fw->name = "case_sensor";
		temp_sensor->therm_fw->domain_name = "case";
		temp_sensor->therm_fw->dev = temp_sensor->dev;
		temp_sensor->therm_fw->dev_ops = &case_sensor_ops;
		thermal_sensor_dev_register(temp_sensor->therm_fw);
	} else {
		dev_err(&pdev->dev, "%s:Cannot alloc memory for thermal fw\n",
			__func__);
		ret = -ENOMEM;
		goto therm_fw_alloc_err;
	}

	temp_sensor->threshold_hot = INIT_T_HOT;
	temp_sensor->threshold_cold = INIT_T_COLD;
	temp_sensor->hot_event = 0;
	ret = sysfs_create_group(&pdev->dev.kobj,
				 &case_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	for (i = 0; i < AVERAGE_NUMBER; i++)
		sensor_temp_table[i] = 0;
	temp_sensor->work_delay = CASE_REPORT_DELAY_MS;
	suspend_state = 0;
	schedule_delayed_work(&temp_sensor->case_sensor_work,
			msecs_to_jiffies(0));

/*
	r = register_pm_notifier(&omap_case_temp_sensor_pm_notifier);
	if (r)
		pr_err("%s: omap_case pm registration failed!\n", __func__);
*/
	return r;

sysfs_create_err:
	thermal_sensor_dev_unregister(temp_sensor->therm_fw);
	kfree(temp_sensor->therm_fw);
	platform_set_drvdata(pdev, NULL);
therm_fw_alloc_err:
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);
	return ret;
}

static int __devexit case_temp_sensor_remove(struct platform_device *pdev)
{
	struct case_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	thermal_sensor_dev_unregister(temp_sensor->therm_fw);
	kfree(temp_sensor->therm_fw);
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_REMOVE);
	sysfs_remove_group(&temp_sensor->dev->kobj, &case_temp_sensor_group);
	cancel_delayed_work_sync(&temp_sensor->case_sensor_work);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);

	return 0;
}

static int case_temp_sensor_runtime_suspend(struct device *dev)
{
	return 0;
}

static int case_temp_sensor_runtime_resume(struct device *dev)
{
	return 0;
}

static int case_temp_sensor_suspend(struct platform_device *pdev,
                                  pm_message_t state)
{
	unsigned long flags;
	struct case_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	spin_lock_irqsave(&temp_sensor->lock, flags);
	suspend_state = 1;
	__cancel_delayed_work(&case_temp_sensor_pm->case_sensor_work);
	spin_unlock_irqrestore(&temp_sensor->lock, flags);

	return 0;
}

static int case_temp_sensor_resume(struct platform_device *pdev)
{
	unsigned long flags;
	struct case_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	spin_lock_irqsave(&temp_sensor->lock, flags);
	suspend_state = 0;
	schedule_delayed_work(&case_temp_sensor_pm->case_sensor_work,
			msecs_to_jiffies(0));
	spin_unlock_irqrestore(&temp_sensor->lock, flags);

	return 0;
}

static const struct dev_pm_ops case_temp_sensor_dev_pm_ops = {
	.runtime_suspend = case_temp_sensor_runtime_suspend,
	.runtime_resume = case_temp_sensor_runtime_resume,
};

static struct platform_driver case_temp_sensor_driver = {
	.probe = case_temp_sensor_probe,
	.remove = case_temp_sensor_remove,
	.suspend = case_temp_sensor_suspend,
	.resume	= case_temp_sensor_resume,
	.driver = {
		.name = "case_temp_sensor",
		//.pm = &case_temp_sensor_dev_pm_ops,
	},
};

int __init case_temp_sensor_init(void)
{
	if (!cpu_is_omap446x())
		return 0;

	return platform_driver_register(&case_temp_sensor_driver);
}

static void __exit case_temp_sensor_exit(void)
{
	platform_driver_unregister(&case_temp_sensor_driver);
}

module_init(case_temp_sensor_init);
module_exit(case_temp_sensor_exit);

MODULE_DESCRIPTION("Case Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
