/******************************************************************************

                  Copyright (C), 2001-2011, HUAWEI COP

******************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <asm/system.h>
//#include <../mach-omap2/control.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <hsad/gpiomux.h>

#include <hsad/config_general_struct.h>

struct board_id_general_struct *get_board_id_general_struct(char * module_name);

void board_muxconf_init(void)
{
    struct board_id_general_struct* pCurrentBoard = NULL;
    gpiomux_setting* pBoardInitTbl = NULL;
    void __iomem *ctrl_pad_base = NULL;
    void __iomem *wkup_pad_base = NULL;
    void __iomem *current_base = NULL;

    unsigned short usRegValue;

    ctrl_pad_base = ioremap(0x4a100000, SZ_4K);
    wkup_pad_base = ioremap(0x4a31e000, SZ_4K);

    current_base = ctrl_pad_base;
    pCurrentBoard = get_board_id_general_struct(GPIO_MODULE_NAME);

    if(NULL == pCurrentBoard)
    {
       pr_err("can not find module:gpio\n");
       return;
    }
	
    pr_info("current board_id is : 0x%x\n",pCurrentBoard->board_id);

    pBoardInitTbl = (gpiomux_setting *)pCurrentBoard->data_array.gpio_ptr;

    while(pBoardInitTbl->mux_pin != MUX_PIN_END ) 
    {

        if(pBoardInitTbl->mux_mode != GPIOMUX_M7)
        {
            usRegValue = __raw_readw(current_base+pBoardInitTbl->mux_pin);

            // config the normal mode
            if (pBoardInitTbl->init_pull_type != GPIOMUX_NONSET)
	    {
                usRegValue &= GPIO_NORMAL_MODE_UMASK;
                usRegValue |= ( ((u16)pBoardInitTbl->mux_mode) << GPIO_MUX_MODE_OFFSET |
                                ((u16)pBoardInitTbl->init_direction) << GPIO_INPUT_OFFSET |
                                ((u16)pBoardInitTbl->init_pull_type) << GPIO_PULL_TYPE_OFFSET );
            }

            // config the sleep mode
            if ( pBoardInitTbl->offmode_pull_type != GPIOMUX_NONSET)
            {
                usRegValue &= GPIO_OFF_MODE_UNMASK;
                if (pBoardInitTbl->offmode_pull_type != GPIOMUX_DISABLE)
                {
                    usRegValue |= ( 1 << GPIO_OFF_ENABLE_OFFSET |
                                    ((u16) (pBoardInitTbl->offmode_value) << GPIO_OFF_VAL_OFFSET) |
                                    ((u16) (pBoardInitTbl->offmode_pull_type) << GPIO_OFF_PULL_TYPE_OFFSET));
                }
            }
        } 
        else
        {
            usRegValue = GPIOMUX_M7;
        }
    
        __raw_writew(usRegValue,current_base+pBoardInitTbl->mux_pin);

	if(pBoardInitTbl->mux_pin == CSI22_DY2)
	{
		current_base = wkup_pad_base;
	}

	pBoardInitTbl++;
     }

     return;
}

#ifdef CONFIG_DEBUG_FS

static void read_current_mux_setting(void)
{
    void __iomem *ctrl_pad_base;
    void __iomem *wkup_pad_base;

	u32 ulOffset;
    u16 value;
	
    ctrl_pad_base = ioremap(0x4a100000, SZ_4K);
    wkup_pad_base = ioremap(0x4a31e000, SZ_4K);



    for ( ulOffset = 0x40 ; ulOffset <= 0x01D4; ulOffset+=2 )
    {
        value = __raw_readw(ctrl_pad_base + ulOffset);

        pr_info("offset[0x%x] : 0x%hx\n",ulOffset,value);
    }

    for (ulOffset = 0x40; ulOffset <= 0x74; ulOffset+=2)
    {
        value = __raw_readw(wkup_pad_base + ulOffset);
        pr_info("wkupoffset[0x%x] : 0x%hx\n",ulOffset,value);
    }
}

static int gpio_debug_read(void *data, u64 *val)
{
	printk("read current mux setting\n");
        read_current_mux_setting();
	return 0;
}

static int gpio_debug_write(void *data, u64 val)
{
	printk("\nwrite new mux setting,data:%p,val:0x%llx\n",data,val);
        board_muxconf_init();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gpio_dbg_fops,gpio_debug_read,gpio_debug_write,"%llu\n");

static int __init mux_dbg_init(void)
{
	struct dentry *gpio_mux_dbg = debugfs_create_dir("gpiomux_dbg",NULL);
	if(!gpio_mux_dbg)
	{
		printk("gpio_mux_dbg created failed!\n");
	}

	debugfs_create_file("test",S_IRUGO|S_IWUSR|S_IWGRP,gpio_mux_dbg,NULL,&gpio_dbg_fops);

	return 0;
}
late_initcall(mux_dbg_init);

#endif
