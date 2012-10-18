/*  linux/fs/proc/app_info.c
 *
 * nilongyu create for read u-boot version / hardware id / poweroff charger etc. .
 *
 * Copyright (C) 2009 Huawei, Inc.
 * Author: zhouzuohua <zhouzuohua@huawei.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



#include <linux/types.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>
#include <linux/pagemap.h>



extern unsigned int get_charge_flag(void);
extern unsigned int get_recovery_flag(void);

/* same as in proc_misc.c */
static int proc_calc_metrics(char *page, char **start, off_t off, int count, int *eof, int len)
{
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

static int app_tag_read_proc(char* page, char** start, off_t off, int count, int* eof, void *data)
{
    int len = 0;
    u32 charge_flag;
	u32 recovery_flag;  

	recovery_flag = get_recovery_flag();
    charge_flag = get_charge_flag();
    
    len = snprintf(page, PAGE_SIZE,
					"recovery_flag:\n%d\n"
					"charge_flag:\n%d\n",
					recovery_flag,
					charge_flag);
    return proc_calc_metrics(page, start, off, count, eof, len);
}

void __init proc_app_info_init ( void )
{
    static struct {
        char* name;
        int (* read_proc)(char*, char**, off_t, int, int*, void*);
    } *p, simple_ones[] = {
            {"app_info", app_tag_read_proc},
            {NULL,}
    };

    for (p = simple_ones; p->name; p++)
        create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);
}
