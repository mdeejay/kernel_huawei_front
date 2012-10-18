#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <hsad/config_mgr.h>
#include <hsad/config_interface.h>

struct dentry *boardid_debugfs_root;

static ssize_t config_boardid_read(struct file *filp,  char  __user *buffer,size_t count, loff_t *ppos)
{	

	int len = 0;
	char idarray[32];
	memset(idarray, 0, 32);
	
	len = sprintf(idarray, "board id: 0x%03x\n", get_board_id_int());
	
	return simple_read_from_buffer(buffer, count, ppos,(void *) idarray, len);

}


static ssize_t config_common_read(struct file *filp, char __user *buffer,size_t count, loff_t *ppos)
{	

	
	struct board_id_general_struct *config_pairs_ptr;
	config_pair* pconfig ;
	int len = 0,cnt=0;
	char *ptr,*arr_ptr_tmp;
	config_pairs_ptr = get_board_id_general_struct(COMMON_MODULE_NAME);
	
	ptr = (char*)kmalloc((PAGE_SIZE*sizeof(char)),GFP_KERNEL);

	if(!ptr)
		return -ENOMEM;
	
	memset(ptr,0,PAGE_SIZE);
	
	arr_ptr_tmp = ptr;
	
	if(NULL == config_pairs_ptr)
	{
		HW_CONFIG_DEBUG(" can not find  module:common\n");
		len = snprintf(ptr,PAGE_SIZE," can not find  module:common\n");
		len = simple_read_from_buffer(buffer, count, ppos, (void *)ptr, len);
		kfree(ptr);
		return len;
	}
	
	pconfig = (config_pair*)config_pairs_ptr->data_array.config_pair_ptr;
	len = snprintf(ptr,PAGE_SIZE,"%-20s\t%-20s\t%-10s\n\n","COMMON KEY","COMMON DATA","COMMON TYPE");
	ptr   += len;
	cnt += len;
	while(NULL != pconfig->key )
	{
			if( PAGE_SIZE - cnt <= 0)
			{
				HW_CONFIG_DEBUG("max size over one page");
				break;
			}
			if(0 == strlen(pconfig->key)){
				pconfig++;
				continue;
			}
			if(E_CONFIG_DATA_TYPE_STRING == pconfig->type)
			{
				len = snprintf(ptr,PAGE_SIZE-cnt,"%-20s\t%-20s\t%-3d(string)\n",pconfig->key,(char*)pconfig->data,pconfig->type);
			}
			else if(E_CONFIG_DATA_TYPE_INT == pconfig->type)
			{
				len = snprintf(ptr,PAGE_SIZE-cnt,"%-20s\t%-20d\t%-3d(int)\n",pconfig->key,pconfig->data,pconfig->type);
			}
			else if(E_CONFIG_DATA_TYPE_BOOL == pconfig->type)
			{
				len = snprintf(ptr,PAGE_SIZE-cnt,"%-20s\t%-20d\t%-3d(bool)\n",pconfig->key,pconfig->data,pconfig->type);
			}
			else if(E_CONFIG_DATA_TYPE_ENUM == pconfig->type)
			{
				len = snprintf(ptr,PAGE_SIZE-cnt,"%-20s\t%-20d\t%-3d(enum)\n",pconfig->key,pconfig->data,pconfig->type);
			}
			else
			{
								len = snprintf(ptr,PAGE_SIZE-cnt,"%-20s\t%-20d\t%-3d(unknow type)\n",pconfig->key,pconfig->data,pconfig->type);
			}
			
			ptr   += len;
			cnt += len;
			pconfig++;

	}
	
	*ptr = '\0';
	len = strlen(arr_ptr_tmp);
	
	len = simple_read_from_buffer(buffer, count, ppos, (void *) arr_ptr_tmp, len);
	kfree(arr_ptr_tmp);
	return len;

}

static ssize_t config_gpio_read(struct file *filp,  char  __user *buffer,size_t count, loff_t *ppos)
{	

	struct board_id_general_struct *gpios_ptr = get_board_id_general_struct(GPIO_MODULE_NAME);
	gpiomux_setting *gpio_ptr;
	char *ptr, *arr_ptr_tmp;
	int len = 0,cnt = 0;
	ptr = (char*)kmalloc(( PAGE_SIZE*sizeof(char)) ,GFP_KERNEL);

	if(!ptr)
		return -ENOMEM;
	
	memset( ptr, 0, PAGE_SIZE);
	
	arr_ptr_tmp = ptr;
	
	
	if( NULL == gpios_ptr )
	{
		HW_CONFIG_DEBUG(" can not find  module:gpio\n");
		len = snprintf(ptr,PAGE_SIZE," can not find  module:gpio\n");
		len=simple_read_from_buffer(buffer, count, ppos, (void *)ptr, len);
		kfree(ptr);
		return len;
	}
	
	gpio_ptr = (gpiomux_setting *) gpios_ptr->data_array.gpio_ptr;
	len = snprintf(ptr, PAGE_SIZE, "%-20s\t%-20s\t%-10s\n","NET NAME","GENERAL NAME","GPIO NUMBER");
	ptr += len;
	cnt += len;
	while( MUX_PIN_END != gpio_ptr->mux_pin )
	{
			if( PAGE_SIZE - cnt <= 0)
			{
				HW_CONFIG_DEBUG("max size over one page");
				break;
			}
			if( 0 == strlen(gpio_ptr->net_name)){
				gpio_ptr++;
				continue;
			}
			
			len = snprintf(ptr,PAGE_SIZE-cnt,"%-20s\t%-20s\t%-3d\n",gpio_ptr->net_name,gpio_ptr->general_name,gpio_ptr->gpio_num);
			
			ptr += len;
			cnt += len;
			gpio_ptr++;
	}
	
	*ptr = '\0';
	
	len = strlen(arr_ptr_tmp);
	len = simple_read_from_buffer(buffer, count, ppos, (void *) arr_ptr_tmp, len);
	kfree(arr_ptr_tmp);
	return len;

}



static const struct file_operations config_boardid_fops = {
        .read = config_boardid_read,
};

static const struct file_operations config_common_fops = {
        .read = config_common_read,
};

static const struct file_operations config_gpio_fops = {
        .read = config_gpio_read,
};



int  config_debugfs_init(void)
{
	struct dentry *common_file, *gpio_file, *boardid_file;

	boardid_debugfs_root = debugfs_create_dir("boardid", NULL);
	if(!boardid_debugfs_root)
		return -ENOENT;

	boardid_file = debugfs_create_file("id", 0444, boardid_debugfs_root, NULL, &config_boardid_fops);
	
	common_file = debugfs_create_file("common",0444,boardid_debugfs_root,NULL,&config_common_fops);

	gpio_file = debugfs_create_file("gpio",0444,boardid_debugfs_root,NULL,&config_gpio_fops);
	
	return 0;
	
}
