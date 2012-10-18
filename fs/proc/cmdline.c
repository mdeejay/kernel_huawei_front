#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <hsad/config_interface.h>

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	int len = 32;
	bool ret = true;
	char test_version[32]= {0};
	ret = check_whether_test_version(test_version,len);
	if(false == ret)
	{
		printk(KERN_INFO"cannot read test_version in function %s",__func__);
	}
	len = get_board_id_int();
	seq_printf(m, "%s hw_boardid=0x%03x hw_test_version=%s\n", saved_command_line, len, test_version);
	return 0;
}
static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_cmdline_init(void)
{
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
module_init(proc_cmdline_init);
