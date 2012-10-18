/************************************************************
FileName: panel_detect.c

Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.

Author: hantao(00185954)  | Version : 0.1 | Date: 2011-10-21

Description:     Compatibility of differents LCDs.

Function List:   

History:   

     <author>  <time>   <version >   <desc> 

************************************************************/

#ifndef _PANEL_DETECT_H_
#define _PANEL_DETECT_H_


extern bool panel_is_the_panel_supported(const char *name);
extern bool panel_set_support_devices(struct omap_dss_board_info *sdp4430_dss_data );

#endif
