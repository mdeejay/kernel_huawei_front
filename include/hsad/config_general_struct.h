#ifndef _CONFIG_GENERIC_STRUCT_H
#define _CONFIG_GENERIC_STRUCT_H
#include <linux/list.h>

#include "config_data.h"
#include "gpiomux.h"
#include "plat_power_mux.h"
#include "config_boardid.h"


struct board_id_general_struct
{
	char	name[32];
	int		board_id;
	union{
		gpiomux_setting *gpio_ptr;
		config_pair	*config_pair_ptr;
		power_tree_config *config_powerconf_ptr;
	}data_array;
	struct list_head list;
};


#endif
