
#define PANEL_DBG_TAG "Panel debug : "


#ifdef __KERNEL__

#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#define PANEL_DEBUG(fmt, ...)	printk(PANEL_DBG_TAG fmt, ##__VA_ARGS__)

#define _PANEL_DEBUG(fmt, ...)	printk(fmt, ##__VA_ARGS__)
#define strtoul	simple_strtoul

#else

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#define LCD_PARAM_FILE	"panel_toshiba_init_param.txt"

#define PANEL_DEBUG(fmt, ...)	printf(PANEL_DBG_TAG fmt, ##__VA_ARGS__)
#define _PANEL_DEBUG(fmt, ...)	printf(fmt, ##__VA_ARGS__)



typedef unsigned int uint32_t;
typedef unsigned char uint8_t;

static char *skip_spaces(char *buf)
{
	char *p = buf;
	if(!p)
		return 0;
	while(isspace(*p))
		p++;

	return p;
}

#endif //__KERNEL__

#include "lcdd.h"


static struct params *_params=0;	//local pointer
static char *lcd_init_sequence = 0;

int set_param(uint32_t delay, uint8_t cmd, uint8_t *data, uint32_t data_size)
{
	struct lcd_param *param = _params->params[_params->line_nr];

	if(_params->line_nr >= MAX_LINE)	
	{
		//FIXME say sth.
		return -1;
	}

	if(param)
	{
		//FIXME warning.
		if(param->data)//Reuse the node. Free the old data kmalloc our own.
		{
#ifdef __KERNEL__
			kfree(param->data);
			if(data_size>0)
			{
				param->data = (uint8_t *)kzalloc(data_size, GFP_KERNEL);
				if(!param->data)
					goto err;
			}

#else

			free(param->data);
			if(data_size>0)
			{
				param->data = (uint8_t *)malloc(data_size);
				if(!param->data)
					goto err;
				memset(param->data, 0, data_size);
			}
#endif
		}
	}
	else
	{

#ifdef __KERNEL__
		param = (struct lcd_param *)kzalloc(sizeof(struct lcd_param), GFP_KERNEL);
		if(!param)
			goto err;

		if(data_size>0)
		{
			param->data = (uint8_t *)kzalloc(data_size, GFP_KERNEL);	
			if(!param->data)
				goto err;	
		}
		_params->params[_params->line_nr] = param;
#else

		param = (struct lcd_param *)malloc(sizeof(struct lcd_param));
		if(!param)
			goto err;
		memset(param, 0, sizeof(struct lcd_param));
		if(data_size>0)
		{
			param->data = (uint8_t *)malloc(data_size);	
			if(!param->data)
				goto err;	
			memset(param->data, 0, data_size);
		}
		_params->params[_params->line_nr] = param;

#endif
	}

	param->delay = delay;
        param->data_len = data_size;
	param->cmd = cmd;
	memcpy(param->data, data, data_size); 
#if 0	
	{
		int i;
		_PANEL_DEBUG("%d,0x%x,", delay, cmd);
		for(i = 0; i < data_size;i++)
			_PANEL_DEBUG("0x%x,", data[i]);
		_PANEL_DEBUG("\n");
	}
#endif
	_params->line_nr++;
	
	return 0;	

err:

#ifdef __KERNEL__

	if(param)
	{
		if(param->data)
		{
			kfree(param->data);
			param->data = 0;
		}
		kfree(param);
		_params->params[_params->line_nr] = 0;
	}
#else

	if(param)
	{
		if(param->data)
		{
			free(param->data);
			param->data = 0;
		}
		free(param);
		_params->params[_params->line_nr] = 0;
	}
#endif
	return -1;
}



static int parse_line(char **line)
{
	uint32_t _delay = 0;
	uint8_t _cmd = 0;
	uint8_t _data[128];
	char *endptr=0;

	unsigned int word_cnt=0;
	unsigned int val;
	int i;
	
	char *line_buf = *line;

	
        line_buf = skip_spaces(line_buf);

	
	if(*line_buf++ != '{')
	{
		PANEL_DEBUG("invalid line start/n");
		return -1;
	}

	for(i = 0; ; i++)
	{
		if(isspace(line_buf[i]))
			continue;

		if(line_buf[i]=='0' && (line_buf[i+1]=='x' || line_buf[i+1]=='X'))//hex
		{		
			
			val = strtoul(&line_buf[i], &endptr, 16);
			if(*endptr != ',' && *endptr != '}' && !isspace(*endptr))
				return -1;
			i += (endptr - (char *)&line_buf[i] - 1);	
		}
		else if(isdigit(line_buf[i]))					//decimal
		{
			val = strtoul(&line_buf[i], &endptr, 10);
			
			if(*endptr != ',' && *endptr != '}' && !isspace(*endptr))
				return -1;
			i += (endptr - (char *)&line_buf[i] - 1);	
		}
		else if(line_buf[i] == ',')
			continue;	
		else if(line_buf[i] == '}')
		{
			break;			
		}
		else
			return -1;
	
		word_cnt++;
		
		//save the values
		if(word_cnt == 1)
			_delay = val;
		else if(word_cnt == 2)
			_cmd = (uint8_t)val;
		else
		{	//Enter here when word_cnt is no less than 3
			_data[word_cnt - 3] = (uint8_t)val;
		}
	}


	if(word_cnt < 2)
		return -1;		

	set_param(_delay, _cmd, _data, word_cnt - 2);

	*line = &line_buf[i];

        return 0;
}

static int next_line(char **line)
{
	char *p = *line;
	while(*p != 0 && *p != '\n') //Neither EOF nor '\n'
		p++;
	if(*p)
		*line = ++p;	//Find a line end
	else
		return -1;	//EOF


	return 0;
}

static void cleanup(struct params *p)
{
	int i;
	if(!p)
		return;
        
	for(i = 0; i < p->line_nr; i++) 
        {    
                if(p->params[i])
                {    
                        if(p->params[i]->data)
                        {   
#ifdef __KERNEL__ 
                                kfree(p->params[i]->data);
#else
                                free(p->params[i]->data);
#endif
                                p->params[i]->data = 0; 
                        }    
                }    
        }    
		
#ifdef __KERNEL__ 
        kfree(p);
#else
        free(p);
#endif
	_params = 0;
}


int lcdd_parse(void)
{
	int r;
        char *line = lcd_init_sequence;

    if (!lcd_init_sequence)    
    {
        return 0;
    }
	
	PANEL_DEBUG("/**********  start parsing  **********/ \n");
	while(*line)
        {
		
                r = parse_line(&line);
		if(r < 0)
		{
			PANEL_DEBUG("encounter an inavlid line\n");
		}
		if(next_line(&line) < 0)//EOF
			break;	
        }

    /* BEGIN: Added by meijinfang, 2011/11/5   PN:viva_debug*/
    if(lcd_init_sequence)
    {
#ifdef __KERNEL__
        kfree(lcd_init_sequence);
#else
        
        free(lcd_init_sequence);
#endif
        lcd_init_sequence = 0;
    }
    /* END:   Added by meijinfang, 2011/11/5 */
	
	return 0;
}

void lcdd_parse_deinit(struct params *params)
{
	cleanup(params);
	if(lcd_init_sequence)
	{
#ifdef __KERNEL__
		kfree(lcd_init_sequence);
#else
		
		free(lcd_init_sequence);
#endif
		lcd_init_sequence = 0;
	}
}

/**
 * parse_init - Allocate space for saving params.
 * @params:	pointer in which the address of the allocated space is saved.
 * Returns 0 on sucess, -1 on allocation failure or no input sequence.
 */
int lcdd_parse_init(struct params **params)
{
	struct params *p = 0;		

    if (!lcd_init_sequence)
    {
        if (!_params)
        {
            /*use the default configuration.*/
            return -1;
        }
        else
        {
            /*use the current configuration which has been set last time.*/
            *params = _params;
            return 0;
        }
    }

    if (_params)
    {
        cleanup(_params);
    }

#ifdef __KERNEL__
	p = (struct params *)kzalloc(sizeof(struct params), GFP_KERNEL);
	if(!p)
	{
		PANEL_DEBUG("%s : Not enough memory", __func__);
		return -1;
	}
#else

	p = (struct params *)malloc(sizeof(struct params));
	if(!p)
	{
		PANEL_DEBUG("%s : Not enough memory", __func__);
		return -1;
	}

	memset(p, 0, sizeof(sizeof(struct params)));
#endif
	*params = p;
	_params = p;

	return 0;		
}


struct lcdd_attr{
	struct attribute attr;
	ssize_t (*show)(struct kobject *, struct attribute *, char *);
	ssize_t (*store)(struct kobject *, struct attribute *, const char *, size_t size);
};


static ssize_t lcdd_store(struct kobject *kobj, struct attribute *attr, const char *buffer, size_t size)
{
	if(lcd_init_sequence)
		kfree(lcd_init_sequence);	

	lcd_init_sequence = (char *)kzalloc(size, GFP_KERNEL);
	printk("%s:\n", __func__);	

	if(!lcd_init_sequence)
	{
		printk("%s : zmalloc failed, size = %d\n", __func__, size);
		return -1;
	}
	memcpy(lcd_init_sequence, buffer, size);

	return size;
}

static ssize_t lcdd_show(struct kobject *kobj, struct attribute *attr, char *buf)
{	
	ssize_t size = 0;
	if(lcd_init_sequence)
	{
		size = snprintf(buf, PAGE_SIZE, "%s\n", lcd_init_sequence);
	}
	
	return size;
}

static struct lcdd_attr lcdd_attr = __ATTR(lcdd_iface, S_IRWXU, lcdd_show, lcdd_store);


static struct kobject *lcdd_kobj = 0;

static int __init lcdd_init(void)
{
	int ret = 0;
 	lcdd_kobj = kobject_create_and_add("lcdd", 0);
	BUG_ON(!lcdd_kobj);

	ret = sysfs_create_file(lcdd_kobj, &lcdd_attr.attr);		
	if(ret)
		printk("%s : failed to create lcdd in sysfs.\n", __func__);
	
	return ret;
}

static void __exit lcdd_deinit(void)
{
	if(lcdd_kobj)
	{
		kobject_del(lcdd_kobj);	//This could be unnecessary.
		kobject_put(lcdd_kobj);		
		lcdd_kobj = 0;
	}

	if(lcd_init_sequence)
	{
		kfree(lcd_init_sequence);
		lcd_init_sequence = 0;
	}
}

module_init(lcdd_init);
module_exit(lcdd_deinit);

MODULE_AUTHOR("MadChris <s00194980@notesmail.huawei.com>");
////////////////////////////////  Applicaiton ////////////////////////
#if 0
static char *lcd_init_sequence = 0;
int parse_file(char *param_file)
{

	struct params *params;
        FILE *fd = fopen(param_file, "r");
	uint32_t file_size;
	char *buf;
	int i;	

        if(!fd)
        {
                PANEL_DEBUG("%s : Failed to open %s\n", __func__, param_file);
                return -1;
        }

	fseek(fd, 0, SEEK_END);
	file_size = ftell(fd);
	buf = (char *)malloc(file_size);
	fseek(fd, 0, SEEK_SET);
	fread(buf, file_size, 1, fd);
        fclose(fd);
	
	PANEL_DEBUG("Init sequence:\n%s\n", buf);


	parse_init(&params);
	parse_sequence(buf);

	PANEL_DEBUG("/**********  end of parse  **********/\n");	

	for(i = 0; i < params->line_nr; i++)
	{
		struct lcd_param *p;
		if(p = params->params[i])
		{
			int j;
			_PANEL_DEBUG("%d,0x%x,", p->delay, p->cmd);
			for(j = 0; j < p->data_len; j++)
				_PANEL_DEBUG("0x%x,", p->data[j]);
			_PANEL_DEBUG("\n");
		}

	}


	parse_deinit(params);


	
	return 0;
}

int main(int argc, char *argv[])
{
	
	parse_file(argv[1]);
	return 0;
}
#endif
