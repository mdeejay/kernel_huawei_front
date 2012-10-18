
#ifndef PLAT_POWER_MUX_H
#define PLAT_POWER_MUX_H

/*Get PMIC data struct according to boardId*/
struct i2c_board_info*  get_board_powerconf(void);

/*Get voltage value according to consummer name*/
int  get_consummer_voltage(const char* consummer);


struct power_voltage_table
{
	char	name[32];
	int    voltage_value;
};

typedef struct
{
    struct i2c_board_info	*config_power_ptr;
    struct power_voltage_table	*power_voltage_ptr;
}power_tree_config;

#endif
