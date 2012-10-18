/************************************************************************
  Copyright @ Huawei Technologies Co., Ltd. 1998-2011. All rights reserved.    
  Author:dongqiang	
  Description:    // sd detect for front board
*************************************************************************/
#include <linux/types.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/gpio.h>

#include <hsad/config_interface.h>


/*front return 0, not supported return -1 */
int get_sddetect_swfc(void)
{
	int swfc = 0;
	bool ret = get_hw_config_int("sdcard/sw_fc", &swfc, NULL);
	if( ret == true)
		{
		 return swfc;
		}
	return -1;
}
