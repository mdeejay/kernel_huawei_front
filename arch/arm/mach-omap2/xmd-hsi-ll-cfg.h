/*
 * xmd-hsi-ll-cfg.h
 *
 * HSI Link Layer
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: ThippaReddy <thippareddy.dammur@intel.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __XMD_HSI_LL_CFG_H__
#define __XMD_HSI_LL_CFG_H__

#include <plat/omap_hsi.h>	//f00171359, fenghaiming modify£¬ kernel3.0, file omap_hsi.h has been moved to plat
#include <linux/sched.h>

/* Max channels supported */
#define HSI_LL_MAX_CHANNELS         HSI_CHANNELS_MAX

/* Receiver modes */
/* Synchronized Data Flow */
#define HSI_LL_SYNC_DATA_FLOW       HSI_FLOW_SYNCHRONIZED
/* Pipelined Data Flow    */
#define HSI_LL_PIPELINED_DATA_FLOW  HSI_FLOW_PIPELINED

/* Interface modes  */
#define HSI_LL_STREAM_MODE          HSI_MODE_STREAM
#define HSI_LL_FRAME_MODE           HSI_MODE_FRAME

/* Priority mode */
/* Round Robin Mode */
#define HSI_LL_ARBMODE_ROUNDROBIN   HSI_ARBMODE_ROUNDROBIN
/* Priority Mode */
#define HSI_LL_ARBMODE_PRIORITY     HSI_ARBMODE_PRIORITY

/* Default settings */
#define HSI_LL_INTERFACE_MODE       HSI_LL_FRAME_MODE
#define HSI_LL_RECEIVER_MODE        HSI_LL_SYNC_DATA_FLOW
#define HSI_LL_ARBMODE_MODE         HSI_LL_ARBMODE_ROUNDROBIN

/* Frame Size */
#define HSI_LL_MAX_FRAME_SIZE       HSI_FRAMESIZE_DEFAULT

/* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3)
    Divisor value => HSI CLK == HSI base CLK/(1+divisor value)
    Clock Change 48MHz => 96MHz */
/* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3) */
#define HSI_LL_DIVISOR_VALUE        HSI_DIVISOR_DEFAULT

/*To enable Power management */
#define HSI_LL_ENABLE_PM

#define HSI_LL_COUNTERS_VALUE      (HSI_COUNTERS_FT_DEFAULT | \
									HSI_COUNTERS_TB_DEFAULT | \
									HSI_COUNTERS_FB_DEFAULT)

/* Max Retries for OPEN_CONNECT_OCTECT */
#define HSI_LL_MAX_OPEN_CONN_RETRIES          200
/* Use this define if TX retry delay WQ has to be enabled.
   If MODEM has logic where it does not send NAK, then below define
   is not required.*/
/* #define HSI_LL_ENABLE_TX_RETRY_WQ */

/* Enable this to make sure that NAK is not sent to MODEM if buf is not
   available. When buf is not available AP does not send any response(NAK)
   instead waits for buffer and then sends NAK. Also it's necessarry that
   MODEM TX Timers should be disabled to avoid CP side TX timeouts.*/
#define HSI_LL_ENABLE_RX_BUF_RETRY_WQ

#define HSI_LL_REG_UNCOMPLETE   -1

 #define HSI_LL_ENABLE_DEBUG_LOG
 #define HSI_LL_ENABLE_CRITICAL_LOG
 #define HSI_LL_ENABLE_ERROR_LOG

#define HSI_LL_LOG_NECESSARY    1
#define HSI_LL_LOG_UNNECESSARY    0

/*To enable HSI switch sw reset */
//#define HSI_SWITCH_SWRESET_ENABEL


enum {
//Common level segment
	DYNADBG_EMERG         = 1U << 0,	//pr_emerg, default Y;
	DYNADBG_ALERT         = 1U << 1,	//pr_alert, default Y;
	DYNADBG_CRIT          = 1U << 2,	//pr_crit, default N;
	DYNADBG_ERR           = 1U << 3,	//pr_err, default Y;
	DYNADBG_WARN          = 1U << 4,	//pr_warn, default N;
	DYNADBG_NOTICE        = 1U << 5,	//pr_notice, default Y;
	DYNADBG_INFO          = 1U << 6,	//pr_info, default N;
	DYNADBG_DEBUG         = 1U << 7,	//pr_debug, default N;

//Special level segment
	DYNADBG_OPEN_CLOSE    = 1U << 8,	//default Y;
	DYNADBG_READ          = 1U << 9,	//default Y;
	DYNADBG_WRITE         = 1U << 10,	//default Y;
	DYNADBG_INIT_EXIT     = 1U << 11,	//default Y;
	DYNADBG_RX            = 1U << 12,	//default Y;
	DYNADBG_TX            = 1U << 13,	//default Y;
	DYNADBG_EVENT         = 1U << 14,	//default Y;
	DYNADBG_THREADS       = 1U << 15,	//default Y;

//Module segment
	DYNADBG_XMD_CMM_EN   = 1U << 20, //default Y;
	DYNADBG_XMD_BOOT_EN  = 1U << 21, //default Y;
	DYNADBG_XMD_TTY_EN   = 1U << 22, //default Y;
	DYNADBG_XMD_NET_EN   = 1U <<23, //default Y;
	DYNADBG_HSI_MCM_EN   = 1U << 24, //default Y;
	DYNADBG_HSI_LL_EN    = 1U << 25, //default Y;
	DYNADBG_HSI_IF_EN    = 1U << 26, //default Y;
	DYNADBG_HSI_PHL_EN   = 1U << 27, //default Y;

//Private segment, unrestricted to DYNADBG_GLOBAL_EN;
	DYNADBG_VERIFY_EN     = 1U << 29,   //default N;
	DYNADBG_PROBE_EN      = 1U << 30,   //default N;
//Global segment
	DYNADBG_GLOBAL_EN     = 1U << 31, //default Y;
};
//XMD_CMM, XMD_BOOT, XMD_TTY, XMD_NET, HSI_MCM, HSI_LL, HSI_IF, HSI_PHL;

extern uint32_t dynamic_debug_mask;
#define dynamic_debug(mask, x...) \
    do { \
        if( ( (dynamic_debug_mask)&((mask)|(DYNADBG_GLOBAL_EN)) ) == ((mask)|(DYNADBG_GLOBAL_EN)) ) \
            printk(KERN_INFO x); \
    } while (0)

#define dynamic_verify(mask, x...) \
    do { \
        if( ( (dynamic_debug_mask)&((mask)|(DYNADBG_VERIFY_EN)) ) == ((mask)|(DYNADBG_VERIFY_EN)) ) \
            printk(KERN_INFO x); \
    } while (0)



#endif /* __XMD_HSI_LL_CFG_H__ */
