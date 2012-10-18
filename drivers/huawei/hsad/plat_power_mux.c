
#include <linux/string.h>
#include <hsad/config_general_struct.h>
#include <hsad/config_mgr.h>
#include <hsad/plat_power_mux.h>

/*Get PMIC data struct according to boardId*/
struct i2c_board_info*  get_board_powerconf(void)
{
    power_tree_config	*pPowerConfig = NULL;
    struct board_id_general_struct* pCurrentBoard = NULL;

	/*Get current boardid struct*/
    pCurrentBoard = get_board_id_general_struct(POWER_TREE_MODULE_NAME);
    if(NULL == pCurrentBoard)
    {
       pr_err("can not find  module:power\n");
       return NULL;
    }

    pr_info("current board_id is : 0x%x\n",pCurrentBoard->board_id);
    pPowerConfig = pCurrentBoard->data_array.config_powerconf_ptr;

    return pPowerConfig->config_power_ptr;
}

/*Get voltage value according to consummer name*/
int  get_consummer_voltage(const char* consummer)
{
    struct board_id_general_struct* pCurrentBoard = NULL;
    power_tree_config	*pPowerConfig = NULL;
    struct power_voltage_table	*pPowerTable = NULL;
    int ret = 0;

	/*Get current boardid struct*/
    pCurrentBoard = get_board_id_general_struct(POWER_TREE_MODULE_NAME);
    if(NULL == pCurrentBoard)
    {
       pr_err("can not find  module:power\n");
       return ret;
    }
	
    pr_info("current board_id is : 0x%x\n",pCurrentBoard->board_id);

    pPowerConfig = pCurrentBoard->data_array.config_powerconf_ptr;
    pPowerTable = pPowerConfig->power_voltage_ptr;
	while( 0 != pPowerTable->voltage_value) {
        if( !strcmp (pPowerTable->name , consummer) )
        {
               ret = pPowerTable->voltage_value;
               break;
        }else
                pPowerTable ++; 
    }

    return ret;
}
