#ifndef _CONFIG_MGR_H
#define _CONFIG_MGR_H
#include <linux/types.h>
#include "config_data.h"
#include "config_general_struct.h"

extern board_id_t	g_board_id;

extern int set_hw_config(board_id_t a_board_id);

/*
*get module struct pointer ,such as gpio,common
*/
extern struct board_id_general_struct * get_board_id_general_struct(char * module_name);

extern int get_hw_config_xmd_ready(void);	//if support return true, else return false */

#endif
