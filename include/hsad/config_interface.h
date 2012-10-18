#ifndef _CONFIG_INTERFACE_H
#define _CONFIG_INTERFACE_H
#include "config_data.h"
#include "config_general_struct.h"

extern int get_board_id_int(void);

/*
*request gpio by name
*/
extern int gpio_request_by_name(char *name);

/*
*get gpio struct pointer by name
*/
extern gpiomux_setting* get_gpio_struct_by_name(char * name);

/*
*get gpio num by name
*/
extern int get_gpio_num_by_name(char * name);


/* get audio enhance_type type */
extern bool audio_get_enhance_type(char* pstring, size_t count);
extern int  get_mhl_ci2ca_value(void);//add w00185212 for difference board
extern int  get_mhl_connect(void);
extern int  get_touchkey_light_boardId_value(void);  //add for touch key backlight identify boardid by z00190171

extern bool get_hw_config(const char* key, char* pbuf, size_t count, unsigned int *ptype);
extern bool get_hw_config_string(const char* key, char* pbuf, size_t count, unsigned int *ptype);
extern bool get_hw_config_int(const char* key, unsigned int* pbuf, unsigned int *ptype);
extern bool get_hw_config_bool(const char* key, bool* pbuf, unsigned int *ptype);
extern bool get_hw_config_enum(const char* key, char* pbuf, size_t count, unsigned int *ptype);
extern int get_usb_trim_value(void);

extern int get_touchscreen_atmel_value(void);

extern int get_proximity_light_boardId_value(void);
extern bool check_whether_test_version(char *pstr, size_t conut);
extern bool get_sysen_value(void);

/*True means command type, flase means video type*/
extern bool is_command_mode_panel(void);
#endif
