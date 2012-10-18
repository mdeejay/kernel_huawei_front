#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <hsad/config_interface.h>

/*gpio request by name*/
int gpio_request_by_name(char *name)
{
    int gpio_num = get_gpio_num_by_name(name);

    if(-1 == gpio_num)
    {
       HW_CONFIG_DEBUG(" return NULL\n");
        return -1;
    }

    return gpio_request(gpio_num, name);
}

/* get audio enhance_type type */
bool audio_get_enhance_type(char* pstring, size_t count)
{
    return get_hw_config_string("audio/enhance_type", pstring, count, NULL);
}

/* Viva T1, Front T1 and previous board return false(no xmd_ready gpio), after that return true */
int get_hw_config_xmd_ready(void)
{
	int xmd_ready_conf = false;
	bool ret = get_hw_config_int("xmd/xmd_ready_gpio_enable", &xmd_ready_conf, NULL);
	if( ret == true )
	{
		return xmd_ready_conf;
	}
	return false;
}
int get_mhl_ci2ca_value(void)
{
	int value = 0;
	bool ret = get_hw_config_int("mhl/ci2ca_is_pull_up", &value, NULL);
	if( ret == true)
		{
		 return value;
		}
	return 0;
}
int get_mhl_connect(void)
{
	int value = 0;
	bool ret = get_hw_config_int("mhl/mhl_connect", &value, NULL);
	if( ret == true)
		{
		 return value;
		}
	return 0;
}
int get_touchkey_light_boardId_value(void)
{
	int value = 0;
	bool ret = get_hw_config_int("tklight/touchkeylight", &value, NULL);
	if( ret == true)
		{
		 return value;
		}
	return 0;
}
int get_usb_trim_value(void)
{
	int value = 0;
	bool ret = get_hw_config_int("usbeye/trim_value", &value, NULL);
	if( ret == true)
	{
	    return value;
	}
	return 0x24;
}

int get_touchscreen_atmel_value(void)
{
	int value = 0;
	bool ret = get_hw_config_bool("touchscreen/front", &value, NULL);
	if( ret == true)
        {
            return value;
        }
	return 0;
}

int get_proximity_light_boardId_value(void)
{
	int value = 0;
	bool ret = get_hw_config_int("proximity/threshold", &value, NULL);
	if( ret == true)
       {
            return value;
        }
	return 0;
}
bool check_whether_test_version(char *pstr, size_t count)
{
	return get_hw_config_string("test_version/if_test_version",pstr, count, NULL);
}
bool get_sysen_value(void)
{
	int value = 0;
	get_hw_config_int("sysen/board_value", &value, NULL);
	if( 0x01 == value)
	{
	    return true;
	}
	return false;
}

bool is_command_mode_panel(void)
{
    char panel_name[20] = "unknown";
    if(get_hw_config("lcd/name",panel_name,20,NULL))
    {
        if(!strcmp(panel_name, "spanel"))
            return true;
    }

    if(get_hw_config("lcd/name",panel_name,20,NULL))
    {
        if(!strcmp(panel_name, "mdv20"))
            return false;
    }

    return false;
}
