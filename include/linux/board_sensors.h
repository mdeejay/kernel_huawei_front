/************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  FileName: board_sensors.h
  Author: hantao(00185954)       Version : 0.1      Date:  2011-07-11
  Description:     // .h file for sensors       
  Version:         // 
  Function List:   // 
    1. -------
  History:         // 
      <author>  <time>   <version >   <desc>
      hantao    11/07/11     1.0     build this moudle  
***********************************************************/

#ifndef	__BOARD_SENSORS_H__
#define	__BOARD_SENSORS_H__

#ifdef __KERNEL__
#include <plat/Omap4-sensors.h>
#endif

#include <linux/akm8975.h>
#include <linux/lis3dh.h>
#include <linux/l3g4200d.h>

/*Device Id Name*/
#define M_I2C_NAME "akm8975"
#define	GS_I2C_NAME	"lis3dh_acc"
#define	GYRO_I2C_NAME	"l3g4200d_gyr"
#define	PL_I2C_NAME	"tmd2771"
/*Device Id Name*/

/*Slave Adress*/
#define	M_SA     0x0c
#define	GS_SA	0x18
/*Slave Adress*/

/*Input Device Name*/
#define	ACCL_INPUT_DEV_NAME	"input_accl"
#define	COMPASS_INPUT_DEV_NAME	"input_compass"
#define	GYRO_INPUT_DEV_NAME	"input_gyro"
/*Input Device Name*/

/*Input Device Power Name*/

#define	ADI_ACCL_POWER_NAME	"ADI_ACCELEROMETER_VDD_SENSOR"
#define	ST_ACCL_POWER_NAME	"ST_ACCELEROMETER_VDD_SENSOR"
#define	FS_ACCL_POWER_NAME	"FS_ACCELEROMETER_VDD_SENSOR"
#define	COMPASS_POWER_NAME	"COMPASS_VDD_SENSOR"
#define	GYRO_POWER_NAME	"GYROSCOPE_VDD_SENSOR"
#define	PL_POWER_NAME	"PROXIMITY_LIGHT_VDD_SENSOR"

#define DEV_POWER 2600000
/*Input Device Power Name*/

/*Special Array Size*/
#define ACCL_DATA_SIZE 6
/*Special Array Size*/

extern struct regulator *enable_power_for_device(struct device* dev , const char* id,int uV);

/*Platform Data Define*/
#ifdef __KERNEL__

#ifdef HUAWEI_SENSORS

#ifdef HUAWEI_SENSORS_GSENSOR_LIS3DH
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

};
#endif

#ifdef HUAWEI_SENSORS_COMPASS_AKM8975
static struct akm8975_platform_data compass_platform_data = {
    .gpio_DRDY = 25,
};
#endif

#ifdef HUAWEI_SENSORS_GYRO_L3G4200D
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
};
#endif

#endif //#ifdef HUAWEI_SENSORS

#endif //#ifdef __KERNEL__
/*Platform Data Define*/

#endif
