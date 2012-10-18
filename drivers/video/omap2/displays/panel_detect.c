
/************************************************************
FileName: panel_detect.c

Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.

Author: hantao(00185954)  | Version : 0.1 | Date: 2011-10-21

Description:     Compatibility of differents LCDs.

Function List:   


************************************************************/

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
//#include <plat/display.h>
#include <video/omapdss.h>
#include <linux/panel_detect.h>
#include <hsad/config_interface.h>

#define PANEL_LCD_NAME_LENGTH   50

#define PANEL_DETECT_DEBUG 
#define PANEL_TAG  "PANEL_DETECT: "

#ifdef PANEL_DETECT_DEBUG	    
#define HD_PRINT(fmt, ...) \
	printk(PANEL_TAG pr_fmt(fmt), ##__VA_ARGS__)	    
#else
#define HD_PRINT(fmt, ...) do{ \
	}while(0)
#endif


static int panel_get_lcd_number(void);
static void delete_others_unsupport_main_lcd(const char *name,struct omap_dss_board_info *sdp4430_dss_data);
static bool is_lcd_on(char *name);

static char current_lcd[PANEL_LCD_NAME_LENGTH] = "Default: NO LCD";
static char config_name[PANEL_LCD_NAME_LENGTH] = "Default: NO LCD";

static int panel_get_lcd_number(void)
{
    unsigned int num;

    if(get_hw_config("lcd/number",&num,sizeof(num),NULL))
    {
        return num;
    }
    else 
    {
        HD_PRINT(KERN_ERR"Get config faild Num = %d \n",num);
        return -1;
    }
}

static bool panel_get_lcd_config_name(void)
{
    if(get_hw_config("lcd/name",config_name,PANEL_LCD_NAME_LENGTH,NULL))
    {   
        HD_PRINT("Function (%s) (config_name=%s).\n",__FUNCTION__,config_name);
        return true;
    }
    else 
    {
        HD_PRINT(KERN_ERR"Get config name faild.\n");
        return false;
    }
}

bool  panel_is_the_panel_supported(const char *name)
{
    int num;
	
    num = panel_get_lcd_number();
    HD_PRINT("Num = %d \n",num);
    if(num <= 0)
        return false;

    HD_PRINT("Function(%s): driver_name=%s, current_name=%s.\n",__FUNCTION__,name,current_lcd);
    if(strcmp(name,current_lcd)==0)
    {
        return true;
    }

    return false;
}

bool panel_set_support_devices(struct omap_dss_board_info *sdp4430_dss_data)
{
    bool ret=false;
    int i;

    if (sdp4430_dss_data == NULL)
    {
        return false;
    }

    HD_PRINT("Entry %s\n",__FUNCTION__);
    if (!panel_get_lcd_config_name())
    {
        HD_PRINT("Get lcd config name failed.\n");
        return false;
    }

    if (sdp4430_dss_data->num_devices <= 0)
    {
        HD_PRINT("LCD device number is invalide.\n");
        return false;
    }

    for(i=0; i<sdp4430_dss_data->num_devices; i++)
    {
        if((sdp4430_dss_data->devices[i]->channel == OMAP_DSS_CHANNEL_LCD)
            &&(0 == strcmp(sdp4430_dss_data->devices[i]->driver_name,config_name))) 
        {
            HD_PRINT("We find the support main lcd: %s!\n",sdp4430_dss_data->devices[i]->driver_name);
            HD_PRINT("\nBefore copy, current_lcd (name=%s)\n",current_lcd);
            memcpy(current_lcd, sdp4430_dss_data->devices[i]->driver_name, PANEL_LCD_NAME_LENGTH);
            HD_PRINT("\nAfter copy, current_lcd (name=%s)\n",current_lcd);
            sdp4430_dss_data->default_device = sdp4430_dss_data->devices[i];
            delete_others_unsupport_main_lcd(sdp4430_dss_data->devices[i]->driver_name,sdp4430_dss_data);
            return true;
        }
    }

    return false;
}

static void delete_others_unsupport_main_lcd(const char *name,struct omap_dss_board_info *sdp4430_dss_data)
{
    int i,j;

    for(i=0; i<sdp4430_dss_data->num_devices; i++)
    {
        HD_PRINT("\nBefore delete, device %d name = %s\n",i,sdp4430_dss_data->devices[i]->driver_name);
    }

    for(i=0; i<sdp4430_dss_data->num_devices; i++)
    {
        if(sdp4430_dss_data->devices[i]->channel == OMAP_DSS_CHANNEL_LCD
           && strcmp(name,sdp4430_dss_data->devices[i]->driver_name))
        {
            HD_PRINT("\nIn cirle, device %d name = %s\n",i,sdp4430_dss_data->devices[i]->driver_name);
            for(j=i+1; j<sdp4430_dss_data->num_devices; j++)
            {
                sdp4430_dss_data->devices[j-1] = sdp4430_dss_data->devices[j];
            }
            sdp4430_dss_data->num_devices--;
        }
    }

    for(i=0; i<sdp4430_dss_data->num_devices; i++)
    {
        HD_PRINT("\nAfter delete, device %d name = %s\n",i,sdp4430_dss_data->devices[i]->driver_name);
    }
}


EXPORT_SYMBOL(panel_set_support_devices);
EXPORT_SYMBOL(panel_is_the_panel_supported);
