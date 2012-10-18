#ifndef __LCDD_H
#define __LCDD_H


#define MAX_LINE 256
#define mAX_DATA 256 

struct lcd_param
{
	u32 delay;
	u32 data_len;
	u8 cmd;
	u8 *data;
};


struct params
{
	u32 line_nr;
	struct lcd_param *params[MAX_LINE];
};

extern int lcdd_parse_init(struct params **params);
extern void lcdd_parse_deinit(struct params *params);
extern int lcdd_parse(void);

#endif
