#ifndef _CONFIG_DATA_H
#define _CONFIG_DATA_H


#include <linux/kernel.h>

#define BOARD_ID0_MAX 4
#define BOARD_ID1_MAX 4
#define BOARD_ID2_MAX 4

#define DISABLE 	0x00
#define ENABLE  	0xFF

#define ISDEBUG  	(ENABLE)

#if (ISDEBUG == ENABLE)
#define HW_CONFIG_DEBUG(fmt, arg...) printk("[HW_CONFIG]: fun= %s: line=%d: " fmt, __FUNCTION__, __LINE__, ##arg)
#else
#define  HW_CONFIG_DEBUG(fmt, arg...)
#endif

typedef enum _bool_type
{
    FALSE = 0,
    TRUE
} bool_t;

typedef enum _config_data_type
{
    E_CONFIG_DATA_TYPE_INT = 0,
    E_CONFIG_DATA_TYPE_ENUM,
    E_CONFIG_DATA_TYPE_BOOL,
    E_CONFIG_DATA_TYPE_STRING,
    E_CONFIG_DATA_MAX
}config_data_type;

typedef struct _config_pair
{
    char* key;
    unsigned int data; /* data: 可以保存整数、字符串、枚举值 和布尔型 */
    config_data_type type; /* type: */
}config_pair;

typedef struct board_id_info
{
	unsigned int 	id0;
	unsigned int 	id1;
	unsigned int 	id2;
	unsigned int 	valid;

} board_id_t;

#endif
