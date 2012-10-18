/*
 * xmd-hsi-ll.c
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

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/hsi_driver_if.h>
#include "xmd-hsi-ll-if.h"
#include "xmd-hsi-ll-cfg.h"
#include "xmd-hsi-ll-internal.h"


uint32_t dynamic_debug_mask = DYNADBG_EMERG|DYNADBG_ALERT|DYNADBG_ERR|DYNADBG_NOTICE | \
                   DYNADBG_OPEN_CLOSE|DYNADBG_READ|DYNADBG_WRITE|DYNADBG_INIT_EXIT|DYNADBG_RX|DYNADBG_TX|DYNADBG_EVENT|DYNADBG_THREADS | \
                   DYNADBG_XMD_CMM_EN|DYNADBG_XMD_BOOT_EN|DYNADBG_XMD_TTY_EN|DYNADBG_XMD_NET_EN | \
                   DYNADBG_HSI_LL_EN|DYNADBG_HSI_IF_EN|DYNADBG_HSI_PHL_EN | \
                   DYNADBG_GLOBAL_EN;
module_param_named(debug_mask, dynamic_debug_mask, uint, S_IWUSR | S_IRUGO);

#define dynadbg_module(mask, x...)    dynamic_debug( ((mask)|(DYNADBG_HSI_LL_EN)), x)



#define HSI_LL_GET_SIZE_IN_WORDS(size)                       \
	if (size > 4)                                            \
		size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2); \
	else                                                     \
		size = 1;

#define HSI_LL_GET_SIZE_IN_BYTES(size)                       \
	if (size > 4)                                            \
		size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2); \
	else                                                     \
		size = 1;                                            \
	size<<=2;

DEFINE_MUTEX(hsi_ll_write_mutex);
DEFINE_MUTEX(hsi_ll_open_mutex);
DEFINE_MUTEX(hsi_ll_close_mutex);

static struct hsi_ll_data_struct hsi_ll_data;
static struct hsi_ll_if_struct   hsi_ll_if;

static struct hsi_device_driver  hsi_ll_iface =
{
	.ctrl_mask  = ANY_HSI_CONTROLLER,
	.ch_mask[0] = 0,
	.probe      = hsi_ll_probe_cb,
	.remove     = hsi_ll_remove_cb,
	.driver     =
	{
		.name = "HSI_LINK_LAYER",
	},
};

static hsi_ll_notify hsi_ll_cb;

static int hsi_ll_send_cmd_queue(
	unsigned int command,
	unsigned int channel)
{
	if (hsi_ll_data.tx_cmd.count >= HSI_LL_MAX_CMD_Q_SIZE) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: TX CMD Queue Overflow. %s %d",__func__,__LINE__);
//		printk("\nHSI_LL: TX CMD Queue Overflow. %s %d",__func__,__LINE__);
#endif
		return HSI_LL_RESULT_QUEUE_FULL;
	} else {
		hsi_ll_data.tx_cmd.count++;
	}

	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.write_index].command = command;
	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.write_index].channel = channel;

	hsi_ll_data.tx_cmd.write_index++;

	if (hsi_ll_data.tx_cmd.write_index >= HSI_LL_MAX_CMD_Q_SIZE) {
		hsi_ll_data.tx_cmd.write_index = 0;
	}

	hsi_ll_if.msg_avaliable_flag = 1;
	wake_up_interruptible(&hsi_ll_if.msg_avaliable);

	return HSI_LL_RESULT_SUCCESS;
}

static int hsi_ll_read_cmd_queue(
	unsigned int *command,
	unsigned int *channel)
{
	spin_lock_bh(&hsi_ll_if.wr_cmd_lock);

	if (hsi_ll_data.tx_cmd.count > 0) {
		hsi_ll_data.tx_cmd.count--;
	} else {
		spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
		return HSI_LL_RESULT_QUEUE_EMPTY;
	}

	*command = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.read_index].command;
	*channel = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.read_index].channel;

	hsi_ll_data.tx_cmd.read_index++;

	if (hsi_ll_data.tx_cmd.read_index >= HSI_LL_MAX_CMD_Q_SIZE) {
		hsi_ll_data.tx_cmd.read_index = 0;
	}

	spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
	return HSI_LL_RESULT_SUCCESS;
}

static int hsi_ll_command_decode(
	unsigned int* message,
	unsigned int* ll_msg_type,
	unsigned int* channel,
	unsigned int* param)
{
	int ret = 0;
	unsigned int msg = *message;

	*ll_msg_type = ((msg & 0xF0000000) >> 28);

	switch(*ll_msg_type) {
	case HSI_LL_MSG_BREAK:
		*channel = HSI_LL_INVALID_CHANNEL;
		break;
	case HSI_LL_MSG_OPEN_CONN:{
		char lcrCal, lcrAct;
		char val1,val2,val3;

		*channel = ((msg & 0x0F000000) >> 24);
		*param   = ((msg & 0x00FFFF00) >> 8 );

		val1 = ((msg & 0xFF000000) >> 24);
		val2 = ((msg & 0x00FF0000) >> 16);
		val3 = ((msg & 0x0000FF00) >>  8);

		lcrAct = (msg & 0x000000FF);
		lcrCal = val1 ^ val2 ^ val3;

		if (lcrCal != lcrAct)
			ret = -1;
		}
		break;
	case HSI_LL_MSG_CONN_READY:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CONN_CLOSED:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CANCEL_CONN:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_ACK:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_NAK:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CONF_RATE:
		*channel = ((msg & 0x0F000000) >> 24);
		*param   = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET:
		*channel = ((msg & 0x0F000000) >> 24);
		*param   = (msg & 0x00FFFFFF);
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
		if (*channel == 0) {
                       dynadbg_module(DYNADBG_CRIT|DYNADBG_RX,"\nHSI_LL: Unexpected case. Received CMD = 0x%x. %s %d\n",
                                 msg, __func__, __LINE__);
//                    printk("\nHSI_LL: Unexpected case. Received CMD = 0x%x. %s %d\n",
//                              msg, __func__, __LINE__);
		}
#endif
		break;
	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
	default:
		*ll_msg_type = HSI_LL_MSG_INVALID;
		*channel     = HSI_LL_INVALID_CHANNEL;
		ret = -1;
		break;
	}

	return ret;
}

static int hsi_ll_send_command(
	int cmd_type,
	unsigned int channel,
	void* arg)
{
	unsigned int command = 0;
	int ret = 0;

	spin_lock_bh(&hsi_ll_if.wr_cmd_lock);

	switch(cmd_type) {
	case HSI_LL_MSG_BREAK:
		command = 0;
		break;
	case HSI_LL_MSG_OPEN_CONN:{
		unsigned int size = *(unsigned int*)arg;
		unsigned int lcr  = 0;

		if (size > 4) {
			size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2);
		} else {
			size = 1;
		}

		command = ((HSI_LL_MSG_OPEN_CONN & 0x0000000F) << 28) |
				  ((channel              & 0x0000000F) << 24) |
				  ((size                 & 0x0000FFFF) << 8);

		lcr = ((command & 0xFF000000) >> 24) ^
			  ((command & 0x00FF0000) >> 16) ^
			  ((command & 0x0000FF00) >>  8);

		command = command | (lcr & 0x000000FF);
		}
		break;
	case HSI_LL_MSG_CONN_READY:
		command = ((HSI_LL_MSG_CONN_READY & 0x0000000F) << 28) |
				  ((channel               & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CONN_CLOSED:
		command = ((HSI_LL_MSG_CONN_CLOSED & 0x0000000F) << 28) |
				  ((channel                & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CANCEL_CONN: {
		unsigned int role = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_CANCEL_CONN & 0x0000000F) << 28) |
				  ((channel                & 0x0000000F) << 24) |
				  ((role                   & 0x000000FF) << 16);
		}
		break;
	case HSI_LL_MSG_ACK: {
		unsigned int echo_params = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_ACK & 0x0000000F) << 28) |
				  ((channel        & 0x0000000F) << 24) |
				  ((echo_params    & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_NAK:
		command = ((HSI_LL_MSG_NAK & 0x0000000F) << 28) |
				  ((channel        & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CONF_RATE: {
		unsigned int baud_rate = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_CONF_RATE & 0x0000000F) << 28) |
				  ((channel              & 0x0000000F) << 24) |
				  ((baud_rate            & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET: {
		unsigned int size = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F) << 28) |
				  ((channel                    & 0x0000000F) << 24) |
				  ((size                       & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
	default:
		ret = -1;
		break;
	}

	if (ret == 0) {
		ret = hsi_ll_send_cmd_queue(command, channel);
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		if(ret != 0) {
               dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL:hsi_ll_send_cmd_queue fail for Channel %d.%s %d\n",
					  channel, __func__, __LINE__);
//            printk("\nHSI_LL:hsi_ll_send_cmd_queue fail for Channel %d.%s %d\n",
//					  channel, __func__, __LINE__);
		}
	} else {
               dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL:Invalid command issued for Channel %d. %s %d\n",
				  channel, __func__, __LINE__);
//            printk("\nHSI_LL:Invalid command issued for Channel %d. %s %d\n",
//				  channel, __func__, __LINE__);
#endif
	}
	spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
	return ret;
}

/* Read callback for channel 1 to 15 */
static void hsi_ll_read_cb(struct hsi_device *dev, unsigned int size)
{
	spin_lock_bh(&hsi_ll_if.rd_cb_lock);
	/* Data Rx callback*/
	if (hsi_ll_data.ch[dev->n_ch].rx.state != HSI_LL_RX_STATE_RX) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL: Error. Invalid state for channel %d. %s %d",
				 dev->n_ch, __func__, __LINE__);
//            printk("\nHSI_LL: Error. Invalid state for channel %d. %s %d",
//				 dev->n_ch, __func__, __LINE__);
#endif
		spin_unlock_bh(&hsi_ll_if.rd_cb_lock);
		return;
	} else {
		if (hsi_ll_data.ch[dev->n_ch].rx.close_req == TRUE) {
			unsigned int role = HSI_LL_ROLE_RECEIVER;
			struct hsi_ll_rx_tx_data temp;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
               dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nHSI_LL:Read close requested for ch %d. %s %d\n",
					 dev->n_ch,__func__, __LINE__);
//            printk("\nHSI_LL:Read close requested for ch %d. %s %d\n",
//					 dev->n_ch,__func__, __LINE__);
#endif
			hsi_ll_send_command(HSI_LL_MSG_CANCEL_CONN,
								dev->n_ch,
								&role);

			temp.buffer = hsi_ll_data.ch[dev->n_ch].rx.buffer;
			temp.size   = hsi_ll_data.ch[dev->n_ch].rx.size;

			hsi_ll_cb(dev->n_ch,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_FREE_MEM,
					  &temp);

			hsi_ll_data.ch[dev->n_ch].rx.buffer    = NULL;
			hsi_ll_data.ch[dev->n_ch].rx.close_req = FALSE;
			hsi_ll_data.ch[dev->n_ch].rx.state     = HSI_LL_RX_STATE_CLOSED;
		} else {
			struct hsi_ll_rx_tx_data temp_data;

			temp_data.buffer = hsi_ll_data.ch[dev->n_ch].rx.buffer;
			temp_data.size   = hsi_ll_data.ch[dev->n_ch].rx.size;

			hsi_ll_cb(dev->n_ch,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_READ_COMPLETE,
					  &temp_data);

			hsi_ll_data.ch[dev->n_ch].rx.buffer = NULL;
			hsi_ll_send_command(HSI_LL_MSG_CONN_CLOSED,
								dev->n_ch,
								NULL);
			hsi_ll_data.ch[dev->n_ch].rx.state = HSI_LL_RX_STATE_IDLE;
		}
	}
	spin_unlock_bh(&hsi_ll_if.rd_cb_lock);
}

/* Read callback for control channel */
static void hsi_ll_read_complete_cb(struct hsi_device *dev, unsigned int size)
{
	int ret;
	unsigned int channel = 0, param = 0, ll_msg_type = 0;

	spin_lock_bh(&hsi_ll_if.rd_cmd_cb_lock);

	ret = hsi_ll_command_decode(&hsi_ll_data.rx_cmd,
								&ll_msg_type,
								&channel,
								&param);

#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
               dynadbg_module(DYNADBG_CRIT|DYNADBG_RX,"\nHSI_LL: CP => AP CMD = 0x%x.\n", hsi_ll_data.rx_cmd);
//            printk("\nHSI_LL: CP => AP CMD = 0x%x.\n", hsi_ll_data.rx_cmd);
#endif
	if (hsi_ll_if.rd_complete_flag != 1) {
		/* Raise an event */
		hsi_ll_if.rd_complete_flag = 1;
		wake_up_interruptible(&hsi_ll_if.rd_complete);
	} else {
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
		if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
               dynadbg_module(DYNADBG_CRIT|DYNADBG_RX,"\nHSI_LL: Recovery in progress "
					"so LL will drop current CP CMD\n");
//            printk("\nHSI_LL: Recovery in progress "
//					"so LL will drop current CP CMD\n");
		} else {
               dynadbg_module(DYNADBG_CRIT|DYNADBG_RX,"\nHSI_LL: Spurious Callback ???\n");
//            printk("\nHSI_LL: Spurious Callback ???\n");
		}
#endif
		goto quit_rd_cmd_cb;
	}

	if (ret != 0) {
	hsi_ll_send_command(HSI_LL_MSG_NAK, channel, NULL);
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL: Received Invalid MSG %d from CP"
				" for channel %d. %s %d\n",
				 ll_msg_type, channel, __func__, __LINE__);
//            printk("\nHSI_LL: Received Invalid MSG %d from CP"
//				" for channel %d. %s %d\n",
//				 ll_msg_type, channel, __func__, __LINE__);
#endif
		goto quit_rd_cmd_cb;
	}

	switch(ll_msg_type) {
	case HSI_LL_MSG_BREAK: {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Break received.%s %d\n", __func__, __LINE__);
//            printk("\nHSI_LL:Break received.%s %d\n", __func__, __LINE__);
#endif
		}
		break;
	case HSI_LL_MSG_ECHO: {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Error.Not supported.%s %d\n", __func__, __LINE__);
//            printk("\nHSI_LL:Error.Not supported.%s %d\n", __func__, __LINE__);
#endif
		}
		break;
	case HSI_LL_MSG_OPEN_CONN: {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:HSI_LL_MSG_OPEN_CONN Not supported.%s %d.\n",
					__func__, __LINE__);
//            printk("\nHSI_LL:HSI_LL_MSG_OPEN_CONN Not supported.%s %d.\n",
//					__func__, __LINE__);
#endif
		}
		break;
	case HSI_LL_MSG_CONN_CLOSED: {
		switch(hsi_ll_data.ch[channel].tx.state) {
		case HSI_LL_TX_STATE_TX:
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_WAIT_FOR_TX_COMPLETE;
			break;
		case HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED: {
			struct hsi_ll_rx_tx_data temp;
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;
			temp.buffer = hsi_ll_data.ch[channel].tx.buffer;
			temp.size   = hsi_ll_data.ch[channel].tx.size;
			hsi_ll_data.ch[channel].tx.buffer = NULL;
#if defined (HSI_LL_ENABLE_DEBUG_LOG) || defined (HSI_LL_ENABLE_CRITICAL_LOG)
               dynadbg_module(DYNADBG_WARN|DYNADBG_RX,"\nHSI_LL: Data transfer over channel %d completed.%s %d\n",
					  channel, __func__, __LINE__);
//            printk("\nHSI_LL: Data transfer over channel %d completed.%s %d\n",
//					  channel, __func__, __LINE__);
#endif
			hsi_ll_cb(channel,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_WRITE_COMPLETE,
					  &temp);
			}
			break;
		default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Invalid State. %s %d\n", __func__, __LINE__);
//            printk("\nHSI_LL:Invalid State. %s %d\n", __func__, __LINE__);
#endif
			break;
		}
		}
		break;
	case HSI_LL_MSG_CANCEL_CONN:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Invalid State. %s %d\n", __func__, __LINE__);
//          printk("\nHSI_LL:Invalid State. %s %d\n", __func__, __LINE__);
#endif
		break;
	case HSI_LL_MSG_ACK: {
		switch(hsi_ll_data.ch[channel].tx.state) {
		case HSI_LL_TX_STATE_OPEN_CONN: {
			int ret;
			unsigned int size = hsi_ll_data.ch[channel].tx.size;

			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TX;

			HSI_LL_GET_SIZE_IN_WORDS(size);

			ret = hsi_write(hsi_ll_data.dev[channel],
							hsi_ll_data.ch[channel].tx.buffer,
							size);

#if defined (HSI_LL_ENABLE_ERROR_LOG)
			if (ret!=0) {
             dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:hsi_write failed for channel %d"
					   " with error %d. %s %d\n",
						 channel, ret, __func__, __LINE__);
//          printk("\nHSI_LL:hsi_write failed for channel %d"
//					   " with error %d. %s %d\n",
//						 channel, ret, __func__, __LINE__);
			} else {
             dynadbg_module(DYNADBG_CRIT|DYNADBG_RX,"\nHSI_LL: Transferring data over channel %d. %s %d\n",
						 channel, __func__, __LINE__);
//          printk("\nHSI_LL: Transferring data over channel %d. %s %d\n",
//						 channel, __func__, __LINE__);
			}
#endif
		}
		break;
		case HSI_LL_TX_STATE_CLOSED: { /* ACK as response to CANCEL_CONN */
			if (hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK) {
				hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_CLOSED;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			} else {
                         dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL: Error. %s %d\n", __func__, __LINE__);
//				printk("\nHSI_LL: Error. %s %d\n", __func__, __LINE__);
#endif
			}
			}
			break;
		 /* ACK as response to CONF_RATE */
		case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK: {
			}
			break;
		default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Error.Invalid state. Current channel %d"
					" state %d. %s %d\n",
				channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
//			printk("\nHSI_LL:Error.Invalid state. Current channel %d"
//					" state %d. %s %d\n",
//				channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
#endif
			break;
			}
		}
		break;
	case HSI_LL_MSG_NAK:{
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nHSI_LL:NAK received for channel %d state %d. retry %d %s %d\n",
				 channel, hsi_ll_data.ch[channel].tx.state,
				 hsi_ll_data.ch[channel].tx.retry,  __func__, __LINE__);
//          printk("\nHSI_LL:NAK received for channel %d state %d. retry %d %s %d\n",
//				 channel, hsi_ll_data.ch[channel].tx.state,
//				 hsi_ll_data.ch[channel].tx.retry,  __func__, __LINE__);
#endif
		switch(hsi_ll_data.ch[channel].tx.state) {
			case HSI_LL_TX_STATE_OPEN_CONN: {
				hsi_ll_data.ch[channel].tx.retry++;

				if (hsi_ll_data.ch[channel].tx.retry == HSI_LL_MAX_OPEN_CONN_RETRIES) {
					struct hsi_ll_rx_tx_data temp;
					temp.buffer = hsi_ll_data.ch[channel].tx.buffer;
					temp.size   = hsi_ll_data.ch[channel].tx.size;
					hsi_ll_data.ch[channel].tx.state  = HSI_LL_TX_STATE_IDLE;
					hsi_ll_data.ch[channel].tx.buffer = NULL;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
                               dynadbg_module(DYNADBG_CRIT|DYNADBG_RX,"\nHSI_LL: %d retry attempts failed, "
							"droping packet for channel %d\n ",
							hsi_ll_data.ch[channel].tx.retry, channel);
//					printk("\nHSI_LL: %d retry attempts failed, "
//							"droping packet for channel %d\n ",
//							hsi_ll_data.ch[channel].tx.retry, channel);
#endif
					hsi_ll_cb(channel,
							  HSI_LL_RESULT_ERROR,
							  HSI_LL_EV_WRITE_COMPLETE,
							 &temp);
				} else {
#if !defined (HSI_LL_ENABLE_TX_RETRY_WQ)
					hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
										channel,
									   &hsi_ll_data.ch[channel].tx.size);
#else
					/* In NAK case Modem needs some delay  */
					PREPARE_WORK(&hsi_ll_data.ch[channel].tx.retry_work,
								  hsi_ll_retry_work);
					queue_work(hsi_ll_if.hsi_tx_retry_wq,
							  &hsi_ll_data.ch[channel].tx.retry_work);
#endif
					}
				}
				break;
			/* NAK as response to CONF_RATE */
			case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK: {
				hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;
				}
				break;
			default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                         dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Error. Invalid state. Current channel %d"
						" state %d. %s %d\n",channel, hsi_ll_data.ch[channel].tx.state, __func__, __LINE__);
//				printk("\nHSI_LL:Error. Invalid state. Current channel %d"
//						" state %d. %s %d\n",
//				channel, hsi_ll_data.ch[channel].tx.state, __func__, __LINE__);
#endif
				break;
		}
		}
		break;

	case HSI_LL_MSG_CONF_RATE:
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET: {
		switch(hsi_ll_data.ch[channel].rx.state) {
			case HSI_LL_RX_STATE_IDLE: {
				struct hsi_ll_rx_tx_data temp_data;
				temp_data.buffer  = NULL;
				temp_data.size    = hsi_ll_data.ch[channel].rx.size = param;

				/*Make size an multiple of word as data is always
					read in multiple of words */
				if(temp_data.size & 0x3) {
					temp_data.size += (4 - (temp_data.size & 0x3));
				}

				hsi_ll_cb(channel,
						  HSI_LL_RESULT_SUCCESS,
						  HSI_LL_EV_ALLOC_MEM,
						  &temp_data);

				if (temp_data.buffer == NULL) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                                dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL: Could not get mem (size=%d)for channel "
							"%d, so currently blocking this ch\n",temp_data.size,channel);
//					printk("\nHSI_LL: Could not get mem (size=%d)for channel "
//							"%d, so currently blocking this ch\n",
//							temp_data.size,channel);
#endif
					hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_BLOCKED;
					goto quit_rd_cmd_cb;
#else
					hsi_ll_send_command(HSI_LL_MSG_NAK,
										channel,
										NULL);
#endif
				} else {
					unsigned int size = hsi_ll_data.ch[channel].rx.size;
					unsigned int *buf = temp_data.buffer;
					hsi_ll_data.ch[channel].rx.buffer = temp_data.buffer;

					hsi_ll_send_command(HSI_LL_MSG_ACK,
										channel,
										&size);

					HSI_LL_GET_SIZE_IN_WORDS(size);

					hsi_read(hsi_ll_data.dev[channel], buf, size);
					hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_RX;
				}
			}
			break;
		case HSI_LL_RX_STATE_BLOCKED: {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
			hsi_ll_send_command(HSI_LL_MSG_NAK,
								channel,
								NULL);
			hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_SEND_NACK;
#endif
			}
			break;
         default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Error.Invalid state. Current channel %d "
					"state %d. %s %d\n",channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
//			printk("\nHSI_LL:Error.Invalid state. Current channel %d "
//					"state %d. %s %d\n",
//				channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
#endif
			break;
		}
		}
		break;
	default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL:Error.Invalid message encountered for "
				"channel %d. %s %d\n", channel, __func__, __LINE__);
//		printk("\nHSI_LL:Error.Invalid message encountered for "
//				"channel %d. %s %d\n", channel, __func__, __LINE__);
#endif
		break;
	}
quit_rd_cmd_cb:
	spin_unlock_bh(&hsi_ll_if.rd_cmd_cb_lock);
}

/* TX Retry Work Queue */
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
static void hsi_ll_retry_work(struct work_struct *work)
{
	struct hsi_ll_tx_ch *tx_ch= (struct hsi_ll_tx_ch*) container_of(work,
											struct hsi_ll_tx_ch,retry_work);
	unsigned int channel = tx_ch->channel;

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
		return;
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nHSI_LL:Retrying Open_connect for ch %d. Size %d. Retry count %d\n",
			channel, hsi_ll_data.ch[channel].tx.size,
			hsi_ll_data.ch[channel].tx.retry);
//	printk("\nHSI_LL:Retrying Open_connect for ch %d. Size %d. Retry count %d\n",
//			channel, hsi_ll_data.ch[channel].tx.size,
//			hsi_ll_data.ch[channel].tx.retry);
#endif
	udelay(100); /*If required Fine tune this delay*/
	hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
						channel,
					   &hsi_ll_data.ch[channel].tx.size);
	return;
}
#endif

/* Write callback for channels 1 to 15 */
static void hsi_ll_write_cb(struct hsi_device *dev, unsigned int size)
{
	spin_lock_bh(&hsi_ll_if.wr_cb_lock);

#if defined (HSI_LL_ENABLE_DEBUG_LOG) || defined (HSI_LL_ENABLE_CRITICAL_LOG)
      dynadbg_module(DYNADBG_WARN|DYNADBG_TX,"\nHSI_LL:Write complete for channel %d. %s %d\n",
			  dev->n_ch, __func__, __LINE__);
//	printk("\nHSI_LL:Write complete for channel %d. %s %d\n",
//			  dev->n_ch, __func__, __LINE__);
#endif
	switch(hsi_ll_data.ch[dev->n_ch].tx.state) {
	case HSI_LL_TX_STATE_TX:
		hsi_ll_data.ch[dev->n_ch].tx.state = HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED;
		break;
	case HSI_LL_TX_STATE_WAIT_FOR_TX_COMPLETE: {
		struct hsi_ll_rx_tx_data temp;

		hsi_ll_data.ch[dev->n_ch].tx.state = HSI_LL_TX_STATE_IDLE;
		temp.buffer = hsi_ll_data.ch[dev->n_ch].tx.buffer;
		temp.size   = hsi_ll_data.ch[dev->n_ch].tx.size;
		hsi_ll_data.ch[dev->n_ch].tx.buffer = NULL;
#if defined (HSI_LL_ENABLE_DEBUG_LOG) || defined (HSI_LL_ENABLE_CRITICAL_LOG)
             dynadbg_module(DYNADBG_WARN|DYNADBG_TX,"\nHSI_LL: Data transfer over channel %d completed. %s %d\n",
				  dev->n_ch, __func__, __LINE__);
//		printk("\nHSI_LL: Data transfer over channel %d completed. %s %d\n",
//				  dev->n_ch, __func__, __LINE__);
#endif
		hsi_ll_cb(dev->n_ch, HSI_LL_RESULT_SUCCESS,
				  HSI_LL_EV_WRITE_COMPLETE,
				 &temp);
		}
		break;
	default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL:Error.Invalid state for channel %d. %s %d\n",
				  dev->n_ch, __func__, __LINE__);
//		printk("\nHSI_LL:Error.Invalid state for channel %d. %s %d\n",
//				  dev->n_ch, __func__, __LINE__);
#endif
		break;
	}

	spin_unlock_bh(&hsi_ll_if.wr_cb_lock);
}

/* Write callback for control channel */
static void hsi_ll_write_complete_cb(struct hsi_device *dev, unsigned int size)
{
	spin_lock_bh(&hsi_ll_if.wr_cmd_cb_lock);

#if defined (HSI_LL_ENABLE_DEBUG_LOG) || defined (HSI_LL_ENABLE_CRITICAL_LOG)
         dynadbg_module(DYNADBG_WARN|DYNADBG_TX,"\nHSI_LL:Write Complete callback.%s %d\n", __func__, __LINE__);
//      printk("\nHSI_LL:Write Complete callback.%s %d\n", __func__, __LINE__);
#endif

	if (hsi_ll_if.wr_complete_flag != 1) {
		/* Raise an event */
		hsi_ll_if.wr_complete_flag = 1;
		hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_IDLE;
		wake_up_interruptible(&hsi_ll_if.wr_complete);
	}

	spin_unlock_bh(&hsi_ll_if.wr_cmd_cb_lock);
}

static int hsi_ll_create_rx_thread(void)
{
	hsi_ll_if.rd_th = kthread_run(hsi_ll_rd_ctrl_ch_th,
								  NULL,
								 "hsi_ll_rd_ctrlch");

	if (IS_ERR(hsi_ll_if.rd_th)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_THREADS,"\nHSI_LL:Cannot create read ctrl Thread.%s %d\n",
				  __func__, __LINE__);
//		printk("\nHSI_LL:Cannot create read ctrl Thread.%s %d\n",
//				  __func__, __LINE__);
#endif
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return -1;
	}

	return 0;
}

static int hsi_ll_config_bus(void)
{
	int ret = HSI_LL_RESULT_SUCCESS;

#if defined (HSI_LL_ENABLE_ERROR_LOG)
    dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"HSI_LL: %s( ) default to ([rx_divisor]%d, [tx_divisor]%d).\n",__func__,1,1);
// pr_info("HSI_LL: %s( ) default to ([rx_divisor]%d, [tx_divisor]%d).\n",__func__,1,1); // /*Add begin for Flashless switch to Norm_HSI AT block HSI */
#endif

	hsi_ll_data.rx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.rx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.rx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.rx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.rx_cfg.ctx.counters   = HSI_LL_COUNTERS_VALUE;
	hsi_ll_data.rx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_RX,
				    &hsi_ll_data.rx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:RX config error %d. %s %d\n", ret, __func__, __LINE__);
//		printk("\nHSI_LL:RX config error %d. %s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;

		return HSI_LL_RESULT_ERROR;
	}

	hsi_ll_data.tx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.tx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.tx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.tx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.tx_cfg.ctx.arb_mode   = HSI_LL_ARBMODE_MODE;
	hsi_ll_data.tx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;


	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_TX,
					&hsi_ll_data.tx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:TX config error %d.%s %d\n", ret, __func__, __LINE__);
//		printk("\nHSI_LL:TX config error %d.%s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return HSI_LL_RESULT_ERROR;
	}
	return ret;
}

static int hsi_ll_wr_ctrl_ch_th(void *data)
{
	int ret,i;
	wait_event_interruptible(hsi_ll_if.reg_complete,
							 hsi_ll_if.reg_complete_flag == 1 || kthread_should_stop() );
	if( kthread_should_stop() ) return 0;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
         dynadbg_module(DYNADBG_DEBUG|DYNADBG_THREADS,"\nHSI_LL:Write thread Started.%s %d\n", __func__, __LINE__);
//      printk("\nHSI_LL:Write thread Started.%s %d\n", __func__, __LINE__);
#endif

#if defined (HSI_SWITCH_SWRESET_ENABEL)
#if	defined(CONFIG_HSI_FLASHLESS_SUPPORT) && defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
        if( hsi_open(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL]) < 0 ) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_THREADS,"HSI_LL:%s. Could not open channel %d. \n", __func__, HSI_LL_CTRL_CHANNEL);
//         printk("HSI_LL:%s. Could not open channel %d. \n", __func__, HSI_LL_CTRL_CHANNEL);
#endif
        }
        if( hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_SW_RESET, NULL) < 0 )
            dynadbg_module(DYNADBG_ERR|DYNADBG_THREADS,"HSI_LL: %s. HSI PHY rest returned error. \n", __func__);
//         pr_err("HSI_LL: %s. HSI PHY rest returned error. \n", __func__);
#endif  //#if	defined(CONFIG_HSI_FLASHLESS_SUPPORT) && defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
#endif  //#if defined (HSI_SWITCH_SWRESET_ENABEL)

	ret = hsi_ll_open(HSI_LL_CTRL_CHANNEL);
	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_THREADS,"\nHSI_LL:Error while opening CTRL channel %d.%s %d",
				  ret, __func__, __LINE__);
//		printk("\nHSI_LL:Error while opening CTRL channel %d.%s %d",
//				  ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.initialized = FALSE;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;

		return -1;
	}

	for(i=1; i < HSI_LL_MAX_CHANNELS; i++) {
		/*Note: This is just a WA to receive packets on channels that
				are not opened by application, with this WA it is possible
				to receive and drop broadcast packets on channels
				close/not opened by application */
		ret = hsi_ll_open(i);
		if (ret != 0)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_THREADS,"HSI_LL: Error opening Channel %d,err=%d", i, ret);
//			printk("HSI_LL: Error opening Channel %d,err=%d", i, ret);
	}

	ret = hsi_ll_config_bus();
	if(ret) {
		return -1;
	}

	hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);

	ret = hsi_ll_create_rx_thread();
	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_THREADS,"\nHSI_LL: Failed to create RX thread. "
			"Initiating HSI Link Layer Shutdown. %s %d\n",
			__func__, __LINE__);
//		printk("\nHSI_LL: Failed to create RX thread. "
//			"Initiating HSI Link Layer Shutdown. %s %d\n",
//			__func__, __LINE__);
#endif
		hsi_ll_shutdown();
		return -1;
	}

	hsi_ll_if.msg_avaliable_flag = 0;

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
        hsi_ll_if.reg_complete_flag = 2;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
        dynadbg_module(DYNADBG_NOTICE|DYNADBG_THREADS,"\nHSI_LL: %s. wake_up_interruptible(&hsi_ll_if.reg_complete) to inform hsi_ll_phy_drv_init initialize over! \n",__func__);
//     printk("\nHSI_LL: %s. wake_up_interruptible(&hsi_ll_if.reg_complete) to inform hsi_ll_phy_drv_init initialize over! \n",__func__);
#endif
        wake_up_interruptible(&hsi_ll_if.reg_complete);
#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)

	wait_event_interruptible(hsi_ll_if.msg_avaliable,
							 hsi_ll_if.msg_avaliable_flag == 1 || kthread_should_stop() );
	if( kthread_should_stop() ) return 0;

	while(1) {
		unsigned int command, channel;

		hsi_ll_if.msg_avaliable_flag = 0;
		while(HSI_LL_RESULT_QUEUE_EMPTY == hsi_ll_read_cmd_queue(&command,
																 &channel)) {
#if defined (HSI_LL_ENABLE_PM)
			wait_event_interruptible_timeout(hsi_ll_if.msg_avaliable,
											 hsi_ll_if.msg_avaliable_flag == 1 || kthread_should_stop(),
											 10);  
			if( kthread_should_stop() ) return 0;
			if (hsi_ll_if.msg_avaliable_flag == 0) {
				hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_ENABLE;
				wake_up_interruptible(&hsi_ll_if.psv_event);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                         dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nHSI_LL: PSV enable requested.\n");
//				printk("\nHSI_LL: PSV enable requested.\n");
#endif
#endif
				wait_event_interruptible(hsi_ll_if.msg_avaliable,
										 hsi_ll_if.msg_avaliable_flag == 1 || kthread_should_stop() );
				if( kthread_should_stop() ) return 0;
#if defined (HSI_LL_ENABLE_PM)
			}
			hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
			hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_UNDEF;
#endif
			hsi_ll_if.msg_avaliable_flag = 0;
		}

		if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nHSI_LL: Stop CMD transfer till recovery completes.\n");
//			printk("\nHSI_LL: Stop CMD transfer till recovery completes.\n");
#endif
			wait_event_interruptible(hsi_ll_if.wr_complete,
									hsi_ll_if.wr_complete_flag == 2 || kthread_should_stop() );
			if( kthread_should_stop() ) return 0;
			hsi_ll_if.wr_complete_flag = 0;
			printk("HSI_LL: HSI LL recovery completed .Start CMD transfer\n");
			continue;
		}

		/* Wakeup Other Side */
		spin_lock_bh(&hsi_ll_if.psv_event_lock);
		if (hsi_ll_data.tx_cfg.ac_wake == HSI_LL_WAKE_LINE_LOW) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nHSI_LL: Requesting AC wake line High.\n");
//			printk("\nHSI_LL: Requesting AC wake line High.\n");
#endif
			spin_unlock_bh(&hsi_ll_if.psv_event_lock);
			msleep(10);
			spin_lock_bh(&hsi_ll_if.psv_event_lock);
			hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);
		}
		spin_unlock_bh(&hsi_ll_if.psv_event_lock);
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
             dynadbg_module(DYNADBG_CRIT|DYNADBG_TX,"\nHSI_LL: AP => CP CMD = 0x%x \n", command);
//		printk("\nHSI_LL: AP => CP CMD = 0x%x \n", command);
#endif
		hsi_ll_data.tx_cmd.channel = channel;
		hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_TX;

		ret = hsi_write(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						&command,
						1);

#if defined (HSI_LL_ENABLE_ERROR_LOG)
		if (ret != 0) {
                   dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: hsi_write(...) failed for ctrl chanel,"
					" err=%d. %s %d", ret, __func__, __LINE__);
//			printk("\nHSI_LL: hsi_write(...) failed for ctrl chanel,"
//					" err=%d. %s %d", ret, __func__, __LINE__);
		}
#endif
		wait_event_interruptible(hsi_ll_if.wr_complete,
								 hsi_ll_if.wr_complete_flag == 1 || kthread_should_stop() );
		if( kthread_should_stop() ) return 0;
		hsi_ll_if.wr_complete_flag = 0;
	}

	return 0;
}

static int hsi_ll_rd_ctrl_ch_th(void *data)
{
	int ret;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_THREADS,"\nHSI_LL:Read thread Started.%s %d\n", __func__, __LINE__);
//	printk("\nHSI_LL:Read thread Started.%s %d\n", __func__, __LINE__);
#endif

	while(1) {
		if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nHSI_LL: Stop CMD reception till recovery completes.\n");
//			printk("\nHSI_LL: Stop CMD reception till recovery completes.\n");
#endif
			wait_event_interruptible(hsi_ll_if.rd_complete,
						 hsi_ll_if.rd_complete_flag == 2 || kthread_should_stop() );
			if( kthread_should_stop() ) return 0;
			hsi_ll_if.rd_complete_flag = 0;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"HSI_LL: HSI LL recovery completed .Start CMD reception\n");
//			printk("HSI_LL: HSI LL recovery completed .Start CMD reception\n");
#endif
		}
		ret = hsi_read(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					  &hsi_ll_data.rx_cmd,
					  1);
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		if (ret) {
                dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nHSI_LL: hsi_read(...) failed for ctrl chanel,"
					" err=%d. %s %d", ret, __func__, __LINE__);
//             printk(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: hsi_read(...) failed for ctrl chanel,"
//					" err=%d. %s %d", ret, __func__, __LINE__);
		}
#endif
		wait_event_interruptible(hsi_ll_if.rd_complete,
								 hsi_ll_if.rd_complete_flag == 1 || kthread_should_stop() );
		if( kthread_should_stop() ) return 0;
		hsi_ll_if.rd_complete_flag = 0;
	}

	return 0;
}

/**
 * hsi_ll_write - HSI LL Write
 * @channel: Channel Number.
 * @char: pointer to buffer.
 * @size: Number of bytes to be transferred.
 */
int hsi_ll_write(int channel, unsigned char *buf, unsigned int size)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	if ((channel <= 0) || (channel >= HSI_LL_MAX_CHANNELS)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
               dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: write attempted for invalid channel %d. %s %d\n",
				  channel, __func__, __LINE__);
//            printk("\nHSI_LL: write attempted for invalid channel %d. %s %d\n",
//                           channel, __func__, __LINE__);
#endif
		return HSI_LL_RESULT_INVALID_PARAM;
	}

	mutex_lock(&hsi_ll_write_mutex);

	if (hsi_ll_data.ch[channel].open == FALSE) {
		ret = HSI_LL_RESULT_ERROR;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: write failed as channel %d is not opened. %s %d\n",
				  channel, __func__, __LINE__);
//		printk("\nHSI_LL: write failed as channel %d is not opened. %s %d\n",
//				  channel, __func__, __LINE__);
#endif
		goto quit_write;
	}

	if ((buf == NULL) || (size == 0)) {
		ret = HSI_LL_RESULT_INVALID_PARAM;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: write failed for channel %d due to invalid params."
				" %s %d\n", channel, __func__, __LINE__);
//		printk("\nHSI_LL: write failed for channel %d due to invalid params."
//				" %s %d\n", channel, __func__, __LINE__);
#endif
		goto quit_write;
	}

	if ((hsi_ll_data.ch[channel].tx.state == HSI_LL_TX_STATE_IDLE)  &&
		(hsi_ll_data.state				!= HSI_LL_IF_STATE_CONFIG)) {
                hsi_ll_data.ch[channel].tx.buffer = buf;
                hsi_ll_data.ch[channel].tx.size   = size;
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_OPEN_CONN;
		hsi_ll_data.ch[channel].tx.retry = 0;
		ret = hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
								  channel,
								  &size);

		if (ret != HSI_LL_RESULT_SUCCESS) {
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;
                        hsi_ll_data.ch[channel].tx.buffer = NULL;
                        hsi_ll_data.ch[channel].tx.size   = 0;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                  dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: write failed for channel %d. %s %d\n",
					  channel, __func__, __LINE__);
//			printk("\nHSI_LL: write failed for channel %d. %s %d\n",
//					  channel, __func__, __LINE__);
#endif
			goto quit_write;
		}
	} else {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL: write failed .channel %d busy. %s %d\n",
				  channel, __func__, __LINE__);
//		printk("\nHSI_LL: write failed .channel %d busy. %s %d\n",
//				  channel, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_ERROR;
		goto quit_write;
	}
quit_write:
	mutex_unlock(&hsi_ll_write_mutex);
	return ret;
}

/**
 * hsi_ll_open - HSI channel open.
 * @channel: Channel number.
 */
int hsi_ll_open(int channel)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	mutex_lock(& hsi_ll_open_mutex);

	if (hsi_ll_data.ch[channel].open == TRUE) {
		goto quit_open;
	}

	if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY ||
		hsi_ll_data.state == HSI_LL_IF_STATE_PERM_ERROR   ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL: Invalid state for channel %d. %s %d\n",
				  channel, __func__, __LINE__);
//		printk("\nHSI_LL: Invalid state for channel %d. %s %d\n",
//				  channel, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_ERROR;
		goto quit_open;
	}

	hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_IDLE;
	hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;

	if (0 > hsi_open(hsi_ll_data.dev[channel])) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Could not open channel %d.%s %d\n",
				  channel, __func__, __LINE__);
//		printk("\nHSI_LL:Could not open channel %d.%s %d\n",
//				  channel, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_ERROR;
	} else {
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		hsi_ll_data.ch[channel].tx.channel = channel;
		INIT_WORK(&hsi_ll_data.ch[channel].tx.retry_work, hsi_ll_retry_work);
#endif
		hsi_ll_data.ch[channel].open = TRUE;
	}

quit_open:
	mutex_unlock(&hsi_ll_open_mutex);
	return ret;
}

/**
 * hsi_ll_close - HSI channel close.
 * @channel: Channel number.
 */
int hsi_ll_close(int channel)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	mutex_lock(& hsi_ll_close_mutex);

	if (hsi_ll_data.state == HSI_LL_IF_STATE_PERM_ERROR ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT) {
		ret = HSI_LL_RESULT_ERROR;
		goto quit_close;
	}
#if 0
	/*NOTE: Below code block is commented as a WA. This is done to ensure that
			all hsi channels are kept open all the time so that
			broadcast messages from CP are not blocked */
	if ((hsi_ll_data.ch[channel].rx.state != HSI_LL_RX_STATE_IDLE)		||
		(hsi_ll_data.ch[channel].rx.state != HSI_LL_RX_STATE_POWER_DOWN)) {
		if (channel == 0) {
			/* wait until end of transmission */
			hsi_ll_data.ch[channel].rx.close_req = TRUE;
		} else if (hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_RX) {
			int role = HSI_LL_ROLE_RECEIVER;
			hsi_ll_send_command(HSI_LL_MSG_CANCEL_CONN,
								channel,
								&role);
			hsi_ll_cb(channel,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_FREE_MEM,
					  hsi_ll_data.ch[channel].rx.buffer);
			hsi_ll_data.ch[channel].rx.buffer    = NULL;
			hsi_ll_data.ch[channel].rx.close_req = FALSE;
			hsi_ll_data.ch[channel].rx.state     = HSI_LL_RX_STATE_SEND_CONN_CANCEL;
		} else if (hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_ACK    ||
			hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_NACK         ||
			hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_CONN_READY   ||
			hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_CONN_CLOSED) {
			hsi_ll_data.ch[channel].rx.close_req = TRUE;
		}
	} else {
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_CLOSED;
	}
	/* Upper layers should take care that noting is
		sent on this channel till it is opened again*/
	hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_CLOSED;
	hsi_close(hsi_ll_data.dev[channel]);
	hsi_ll_data.ch[channel].open = FALSE;
#endif

quit_close:
	mutex_unlock(&hsi_ll_close_mutex);
	return HSI_LL_RESULT_SUCCESS;
}

static void hsi_ll_wakeup_cp(unsigned int val)
{
	int ret = -1;

	if (val == HSI_LL_WAKE_LINE_HIGH) {
		hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_HIGH;
		ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						HSI_IOCTL_ACWAKE_UP,
						NULL);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"HSI_LL:Setting AC wake line to HIGH.\n");
//		printk("HSI_LL:Setting AC wake line to HIGH.\n");
#endif
	} else {
		hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_LOW;
		ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						HSI_IOCTL_ACWAKE_DOWN,
						NULL);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"HSI_LL:Setting AC wake line to LOW .\n");
//		printk("HSI_LL:Setting AC wake line to LOW .\n");
#endif
	}

#if defined (HSI_LL_ENABLE_ERROR_LOG)
	if (ret) {
             dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nHSI_LL:Error setting AC_WAKE line err=%d. %s %d\n",
				  ret, __func__, __LINE__);
//		printk("\nHSI_LL:Error setting AC_WAKE line err=%d. %s %d\n",
//				  ret, __func__, __LINE__);
	}
#endif

	return;
}

/*
 * Processes wakeup
 */
#if defined (HSI_LL_ENABLE_PM)
static int hsi_ll_psv_th(void *data)
{
	unsigned int psv_done = 0;
	unsigned int channel = 0;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG) || HSI_LL_LOG_NECESSARY
    static unsigned int count=0;    // /* Add for HSI exception handling */
#endif  //#if defined (HSI_LL_ENABLE_CRITICAL_LOG)

	while(1) {
		wait_event_interruptible(hsi_ll_if.psv_event,
					(hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE) || kthread_should_stop() );
		if( kthread_should_stop() ) return 0;
		psv_done = 0;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG) || HSI_LL_LOG_NECESSARY
        count = 0;  // /* Add for HSI exception handling */
#endif  //#if defined (HSI_LL_ENABLE_CRITICAL_LOG)

		do{
			if( kthread_should_stop() ) return 0;
			if (hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_DISABLE) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                         dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nHSI_LL:PSV_Enable Revoked.\n");
//				printk("\nHSI_LL:PSV_Enable Revoked.\n");
#endif
				break;
			}

			for(channel = 0; channel < HSI_LL_MAX_CHANNELS; channel++) {
				if (hsi_ll_data.ch[channel].tx.state != HSI_LL_TX_STATE_IDLE) {
#if defined (HSI_LL_ENABLE_CRITICAL_LOG) || HSI_LL_LOG_NECESSARY
                    if( ++count<=HSI_LL_MAX_CHANNELS ) // /* Add for HSI exception handling */
                         printk("\\HSI_LL:PSV_Enable - Channel[%d] is busy with"
                                                  " status %d, retry after 200ms.\n",
                                                       channel,hsi_ll_data.ch[channel].tx.state);
                    if( (count-1)==HSI_LL_MAX_CHANNELS )
                        printk("\\HSI_LL:PSV_Enable - Channel[%d] is busy with"
						   " status %d, retry for %d times.\n",
							channel,hsi_ll_data.ch[channel].tx.state, count);
#endif
					msleep_interruptible(100);
					break;
				}
			}

			if (channel >= HSI_LL_MAX_CHANNELS) {
				/* All channels are idle. Check if PSV can be Enabled */
				spin_lock_bh(&hsi_ll_if.psv_event_lock);
				if (hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                                dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nHSI_LL:Requesting to set AC wake line to low.\n");
//					printk("\nHSI_LL:Requesting to set AC wake line to low.\n");
#endif
					hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
					hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_LOW);
					psv_done = 1;
				}
				spin_unlock_bh(&hsi_ll_if.psv_event_lock);
			}
		} while(!psv_done);
	}

	return 1;
}
#endif

static int hsi_ll_events_init(void)
{
	init_waitqueue_head(&hsi_ll_if.wr_complete);
	init_waitqueue_head(&hsi_ll_if.rd_complete);
	init_waitqueue_head(&hsi_ll_if.msg_avaliable);
	init_waitqueue_head(&hsi_ll_if.reg_complete);
#if defined (HSI_LL_ENABLE_PM)
	init_waitqueue_head(&hsi_ll_if.psv_event);
#endif

	/* Initialize spin_lock */
	spin_lock_init(&hsi_ll_if.phy_cb_lock);
	spin_lock_init(&hsi_ll_if.wr_cmd_cb_lock);
	spin_lock_init(&hsi_ll_if.rd_cmd_cb_lock);
	spin_lock_init(&hsi_ll_if.wr_cb_lock);
	spin_lock_init(&hsi_ll_if.rd_cb_lock);
	spin_lock_init(&hsi_ll_if.wr_cmd_lock);
	spin_lock_init(&hsi_ll_if.psv_event_lock);
	return HSI_LL_RESULT_SUCCESS;
}

static int hsi_ll_probe_cb(struct hsi_device *dev)
{
	int port;

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (hsi_ll_iface.ch_mask[port])
			break;
	}

	if (port == HSI_MAX_PORTS)
		return -ENXIO;

	if (dev->n_ch >= HSI_LL_MAX_CHANNELS) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Invalid channel %d. %s %d.\n",
					dev->n_ch, __func__, __LINE__);
//		printk("\nHSI_LL:Invalid channel %d. %s %d.\n",
//					dev->n_ch, __func__, __LINE__);
#endif
		return -ENXIO;
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG) && (HSI_LL_LOG_UNNECESSARY)   // /* Add for HSI exception handling */
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\\HSI_LL: probe Channel ID = %d , Dev[%d] = 0x%x . %s %d.\n",
				dev->n_ch, dev->n_ch, (int)dev, __func__, __LINE__);
//	printk("\nHSI_LL: probe Channel ID = %d , Dev[%d] = 0x%x . %s %d.\n",
//				dev->n_ch, dev->n_ch, (int)dev, __func__, __LINE__);
#endif

	spin_lock_bh(&hsi_ll_if.phy_cb_lock);

	if (dev->n_ch == 0) {
		hsi_set_read_cb(dev, hsi_ll_read_complete_cb);
		hsi_set_write_cb(dev, hsi_ll_write_complete_cb);
	} else {
		hsi_set_read_cb(dev, hsi_ll_read_cb);
		hsi_set_write_cb(dev, hsi_ll_write_cb);
	}

	hsi_set_port_event_cb(dev, hsi_ll_port_event_cb);

	hsi_ll_data.dev[dev->n_ch] = dev;
	hsi_ll_data.ch[dev->n_ch].tx.state     = HSI_LL_TX_STATE_CLOSED;
	hsi_ll_data.ch[dev->n_ch].rx.state     = HSI_LL_RX_STATE_CLOSED;
	hsi_ll_data.ch[dev->n_ch].rx.close_req = FALSE;
	hsi_ll_data.ch[dev->n_ch].tx.pending   = FALSE;
	hsi_ll_data.ch[dev->n_ch].tx.data_rate = 0;
	hsi_ll_data.ch[dev->n_ch].tx.buffer    = NULL;
	hsi_ll_data.ch[dev->n_ch].rx.buffer    = NULL;

	hsi_ll_if.reg_complete_ch_count++;

	spin_unlock_bh(&hsi_ll_if.phy_cb_lock);

	if (hsi_ll_if.reg_complete_ch_count == HSI_LL_MAX_CHANNELS) {
		hsi_ll_if.reg_complete_flag = 1;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:Registration for %d channels completed.%s %d.\n",
				 (unsigned int)HSI_LL_MAX_CHANNELS, __func__, __LINE__);
//		printk("\nHSI_LL:Registration for %d channels completed.%s %d.\n",
//				 (unsigned int)HSI_LL_MAX_CHANNELS, __func__, __LINE__);
#endif
		wake_up_interruptible(&hsi_ll_if.reg_complete);
	}

	return 0;
}

static int hsi_ll_remove_cb(struct hsi_device *dev)
{
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Invoked %s.\n",__func__);
//	printk("\nHSI_LL: Invoked %s.\n",__func__);
#endif
	spin_lock_bh(&hsi_ll_if.phy_cb_lock);
	hsi_set_read_cb(dev, NULL);
	hsi_set_write_cb(dev, NULL);
    hsi_set_port_event_cb(dev, NULL);   //#y00185015#20111007#Or xmd-boot0 will call "hsi_ll_port_event_cb"
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
    dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"HSI_LL:%s. (dev=@0x%08X, hsi_ll_data.dev[dev->n_ch]=@0x%08X)==1?%d;\n",__func__,
                (int)dev,(int)hsi_ll_data.dev[dev->n_ch],(bool)(dev==hsi_ll_data.dev[dev->n_ch]) );
// printk("HSI_LL:%s. (dev=@0x%08X, hsi_ll_data.dev[dev->n_ch]=@0x%08X)==1?%d;\n",__func__,
//                (int)dev,(int)hsi_ll_data.dev[dev->n_ch],(bool)(dev==hsi_ll_data.dev[dev->n_ch]) );
#endif  //#if defined (HSI_LL_ENABLE_DEBUG_LOG)
    //dev = NULL; //dev is same with hsi_ll_data.dev[dev->n_ch]

	spin_unlock_bh(&hsi_ll_if.phy_cb_lock);

	return 0;
}

static void hsi_ll_port_event_cb(
	struct hsi_device *dev,
	unsigned int event,
	void *arg)
{
    if(dev->n_ch == HSI_LL_CTRL_CHANNEL) {  
	switch(event) {
	case HSI_EVENT_BREAK_DETECTED:
#if defined (HSI_LL_ENABLE_DEBUG_LOG) || HSI_LL_LOG_NECESSARY
		printk("\\HSI_LL:Break Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_ERROR:
#if defined (HSI_LL_ENABLE_DEBUG_LOG) || HSI_LL_LOG_NECESSARY
		printk("\\HSI_LL:Error Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_PRE_SPEED_CHANGE:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_EVENT,"\\HSI_LL:Pre Speed changer Event detected.%s %d\n",
				  __func__, __LINE__);
//		printk("\\HSI_LL:Pre Speed changer Event detected.%s %d\n",
//				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_POST_SPEED_CHANGE:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_EVENT,"\\HSI_LL:Post Speed changer Event detected.%s %d\n",
				 __func__, __LINE__);
//		printk("\\HSI_LL:Post Speed changer Event detected.%s %d\n",
//				 __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_CAWAKE_UP:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_EVENT,"\\HSI_LL:CA wakeup line UP detected.%s %d\n",
                                 __func__, __LINE__);
//		printk("\\HSI_LL:CA wakeup line UP detected.%s %d\n",
//				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_CAWAKE_DOWN:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_EVENT,"\\HSI_LL:CA wakeup line DOWN detected.%s %d\n",
                                 __func__, __LINE__);
//		printk("\\HSI_LL:CA wakeup line DOWN detected.%s %d\n",
//				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_HSR_DATAAVAILABLE:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_EVENT,"\\HSI_LL:HSR Event detected.%s %d\n",
				  __func__, __LINE__);
//		printk("\\HSI_LL:HSR Event detected.%s %d\n",
//				  __func__, __LINE__);
#endif
		break;
	default:
#if defined (HSI_LL_ENABLE_DEBUG_LOG) || HSI_LL_LOG_NECESSARY
		printk("\\HSI_LL:Invalid Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	}
    }
}


/**
 * hsi_ll_init - Initilizes all resources and registers with HSI PHY driver.
 * @port: Number of ports.
 * @cb: pointer to callback.
 */
int hsi_ll_init(int port, const hsi_ll_notify cb)
{
	int i;
	int ret = HSI_LL_RESULT_SUCCESS;

	if (!hsi_ll_data.initialized) {
		if ((port <= 0)           ||
		   (port > HSI_MAX_PORTS) ||
		   (cb == NULL)) {
			ret = HSI_LL_RESULT_INVALID_PARAM;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Invalid port or callback pointer. %s %d\n",
					__func__, __LINE__);
//			printk("\nHSI_LL:Invalid port or callback pointer. %s %d\n",
//					__func__, __LINE__);
#endif
			goto quit_init;
		}

		port-=1;

		for (i = port; i < HSI_MAX_PORTS; i++) {
			hsi_ll_iface.ch_mask[i] = (0xFFFFFFFF ^
									  (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));
		}

		if (HSI_LL_RESULT_SUCCESS != hsi_ll_events_init()) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Cannot create Events. %s %d\n",
					__func__, __LINE__);
//			printk("\nHSI_LL:Cannot create Events. %s %d\n",
//					__func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

		hsi_ll_cb = cb;
		hsi_ll_if.reg_complete_flag = 0;
#if	!defined(CONFIG_HSI_FLASHLESS_SUPPORT)
		ret = hsi_register_driver(&hsi_ll_iface);

		if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Error while registering with HSI PHY driver "
					"err=%d. %s %d\n", ret, __func__, __LINE__);
//			printk("\nHSI_LL:Error while registering with HSI PHY driver "
//					"err=%d. %s %d\n", ret, __func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_INIT_ERROR;
			goto quit_init;
		}
#endif
		hsi_ll_data.state = HSI_LL_IF_STATE_READY;

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:creating threads. %s %d.\n", __func__, __LINE__);
//		printk("\nHSI_LL:creating threads. %s %d.\n", __func__, __LINE__);
#endif
		hsi_ll_if.wr_th = kthread_run(hsi_ll_wr_ctrl_ch_th,
									  NULL,
									 "hsi_ll_wr_ctrlch");

		if (IS_ERR(hsi_ll_if.wr_th)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Cannot create write ctrl Thread. %s %d\n",
					  __func__, __LINE__);
//			printk("\nHSI_LL:Cannot create write ctrl Thread. %s %d\n",
//					  __func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

#if defined (HSI_LL_ENABLE_PM)
		hsi_ll_if.psv_th = kthread_run(hsi_ll_psv_th,
									   NULL,
									  "hsi_ll_psv_thread");

		if (IS_ERR(hsi_ll_if.psv_th)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Cannot create PSV Thread. %s %d\n",
					  __func__, __LINE__);
//			printk("\nHSI_LL:Cannot create PSV Thread. %s %d\n",
//					  __func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		hsi_ll_if.hsi_tx_retry_wq = create_workqueue("hsi_tx_retry_wq");
#endif
#if !defined(CONFIG_HSI_FLASHLESS_SUPPORT)
		hsi_ll_data.state = HSI_LL_IF_STATE_READY;
#else
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
#endif
	}

quit_init:
	return ret;
}

/**
 * hsi_ll_shutdown - Releases all resources.
 */
int hsi_ll_shutdown(void)
{
	if (hsi_ll_data.initialized) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:HSI Link Layer Shutdown initiated. %s %d\n",
				  __func__, __LINE__);
//		printk("\nHSI_LL:HSI Link Layer Shutdown initiated. %s %d\n",
//				  __func__, __LINE__);
#endif
		hsi_ll_close(HSI_LL_CTRL_CHANNEL);
		//hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_ACWAKE_DOWN, NULL);
            hsi_ll_wakeup_cp(HSI_IOCTL_ACWAKE_DOWN);

		kthread_stop(hsi_ll_if.rd_th);
		kthread_stop(hsi_ll_if.wr_th);
#if defined (HSI_LL_ENABLE_PM)
		kthread_stop(hsi_ll_if.psv_th);
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		destroy_workqueue(hsi_ll_if.hsi_tx_retry_wq);
#endif
		hsi_unregister_driver(&hsi_ll_iface);
		hsi_ll_data.initialized = FALSE;
	}
	return HSI_LL_RESULT_SUCCESS;
}

// hsi_ll_exit - Releases all resources and exit.   //#y00185015#20110923#Add for HSIswitch#
int hsi_ll_exit(void)
{
    int ch_i=0;

       dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nHSI_LL:HSI Link Layer Exit. %s hsi_ll_data.initialized=%d,hsi_ll_data.state=%d;\n",
                            __func__,hsi_ll_data.initialized,hsi_ll_data.state);
//    pr_info("\nHSI_LL:HSI Link Layer Exit. %s hsi_ll_data.initialized=%d,hsi_ll_data.state=%d;\n",
//                            __func__,hsi_ll_data.initialized,hsi_ll_data.state);

    {//if (hsi_ll_data.initialized) {
		//hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_ACWAKE_DOWN, NULL);
        hsi_ll_wakeup_cp(HSI_IOCTL_ACWAKE_DOWN);

//mutex_lock(& hsi_ll_close_mutex);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
       dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_ll_if.reg_complete_flag=%d;\n",__func__,hsi_ll_if.reg_complete_flag);
//    pr_info("\nHSI_LL:%s. hsi_ll_if.reg_complete_flag=%d;\n",__func__,hsi_ll_if.reg_complete_flag);

#endif
        if( hsi_ll_if.reg_complete_flag > 0 ) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. kthread_stop(hsi_ll_if.rd_th) start;\n",__func__);
//         pr_info("\nHSI_LL:%s. kthread_stop(hsi_ll_if.rd_th) start;\n",__func__);
#endif
    		kthread_stop(hsi_ll_if.rd_th);
        }
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. kthread_stop(hsi_ll_if.wr_th) start;\n",__func__);
//         pr_info("\nHSI_LL:%s. kthread_stop(hsi_ll_if.wr_th) start;\n",__func__);
#endif
		kthread_stop(hsi_ll_if.wr_th);

#if defined (HSI_LL_ENABLE_PM)
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
        dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. kthread_stop(hsi_ll_if.psv_th) start;\n",__func__);
//      pr_info("\nHSI_LL:%s. kthread_stop(hsi_ll_if.psv_th) start;\n",__func__);
#endif
        kthread_stop(hsi_ll_if.psv_th);
#endif

#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
        dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. destroy_workqueue(hsi_ll_if.hsi_tx_retry_wq) start;\n",__func__);
//     pr_info("\nHSI_LL:%s. destroy_workqueue(hsi_ll_if.hsi_tx_retry_wq) start;\n",__func__);
#endif
        destroy_workqueue(hsi_ll_if.hsi_tx_retry_wq);
#endif

    for(ch_i=HSI_LL_MAX_CHANNELS-1; ch_i >= HSI_LL_CTRL_CHANNEL; ch_i--) {
    	if (hsi_ll_data.ch[ch_i].open == TRUE) {
            hsi_ll_data.ch[ch_i].open = FALSE;
            
            hsi_ll_close(ch_i); /* < Modify for HSI driver soft_switch process > */
            hsi_write_cancel(hsi_ll_data.dev[ch_i]);  //#y00185015#20110919#Add for switch HSI#
            hsi_read_cancel(hsi_ll_data.dev[ch_i]);  //#y00185015#20110919#Add for switch HSI#

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. destroy_workqueue(hsi_ll_if.hsi_tx_retry_wq) start;\n",__func__);
//          pr_info("\nHSI_LL:%s. hsi_close(hsi_ll_data.dev[%d]);\n",__func__,ch_i);
#endif
                hsi_close(hsi_ll_data.dev[ch_i]);
        }
    }

#if defined (HSI_SWITCH_SWRESET_ENABEL)
        if( hsi_open(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL]) < 0 ) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL:%s. Could not open channel %d. \n", __func__, HSI_LL_CTRL_CHANNEL);
//         printk("HSI_LL:%s. Could not open channel %d. \n", __func__, HSI_LL_CTRL_CHANNEL);
#endif
        }
        if( hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_SW_RESET, NULL) < 0 )
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL: %s. HSI PHY rest returned error. \n",__func__);
//         pr_err("HSI_LL: %s. HSI PHY rest returned error. \n",__func__);
#endif   //#if defined (HSI_SWITCH_SWRESET_ENABEL)

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
        dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_unregister_driver(&hsi_ll_iface) start;\n",__func__);
//     pr_info("\nHSI_LL:%s. hsi_unregister_driver(&hsi_ll_iface) start;\n",__func__);
#endif
	hsi_unregister_driver(&hsi_ll_iface);
	hsi_ll_data.initialized = FALSE;
	hsi_ll_if.reg_complete_ch_count = 0;
	hsi_ll_if.reg_complete_flag = 0;
#if defined (HSI_LL_ENABLE_PM)
        hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
#endif
	hsi_ll_if.wr_complete_flag = 0;
	hsi_ll_if.rd_complete_flag = 0;
	}

//mutex_unlock(& hsi_ll_close_mutex);
	return HSI_LL_RESULT_SUCCESS;
}

/* Resets DLP */
int hsi_ll_reset(void)
{
	int ch_i = 0;
	struct hsi_ll_rx_tx_data temp;

	hsi_ll_data.initialized = FALSE;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
      dynadbg_module(DYNADBG_CRIT|DYNADBG_INIT_EXIT,"\nHSI_LL: DLP recovery started.\n");
//	printk("\nHSI_LL: DLP recovery started.\n");
#endif
	hsi_ll_data.state = HSI_LL_IF_STATE_ERR_RECOVERY;
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	flush_workqueue(hsi_ll_if.hsi_tx_retry_wq);
#endif
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
       dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Requesting read/write cancel.\n");
//    printk("\nHSI_LL: Requesting read/write cancel.\n");
#endif

	for(ch_i = 0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		hsi_ll_data.ch[ch_i].open = FALSE;
		hsi_write_cancel(hsi_ll_data.dev[ch_i]);
		hsi_read_cancel(hsi_ll_data.dev[ch_i]);
	}

	hsi_ll_if.wr_complete_flag = 1;
	wake_up_interruptible(&hsi_ll_if.wr_complete);
	hsi_ll_if.rd_complete_flag = 1;
	wake_up_interruptible(&hsi_ll_if.rd_complete);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Unblocking all read/write callbacks.\n");
//	printk("\nHSI_LL: Unblocking all read/write callbacks.\n");
#endif

	for(ch_i = 1; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		if (hsi_ll_data.ch[ch_i].tx.buffer != NULL) {
			temp.buffer = hsi_ll_data.ch[ch_i].tx.buffer;
			temp.size   = hsi_ll_data.ch[ch_i].tx.size;
                        hsi_ll_data.ch[ch_i].tx.buffer = NULL;
                        hsi_ll_data.ch[ch_i].tx.size = 0;
			hsi_ll_cb(ch_i,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_WRITE_COMPLETE,
					  &temp);
		}
		if (hsi_ll_data.ch[ch_i].rx.buffer != NULL) {
			temp.buffer = hsi_ll_data.ch[ch_i].rx.buffer;
			temp.size   = hsi_ll_data.ch[ch_i].rx.size;
                        hsi_ll_data.ch[ch_i].rx.buffer = NULL;
                        hsi_ll_data.ch[ch_i].rx.size = 0;
			hsi_ll_cb(ch_i,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_READ_COMPLETE,
					  &temp);
		}
		hsi_ll_data.ch[ch_i].tx.state = HSI_LL_TX_STATE_IDLE;
		hsi_ll_data.ch[ch_i].rx.state = HSI_LL_RX_STATE_IDLE;
		hsi_ll_data.ch[ch_i].open = TRUE;
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Revoking PSV enable.\n");
#endif

#if defined (HSI_LL_ENABLE_PM)
	hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
        spin_lock_bh(&hsi_ll_if.psv_event_lock);
        hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_LOW);
        spin_unlock_bh(&hsi_ll_if.psv_event_lock);
#endif

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Resetting HSI PHY Driver.\n");
//	printk("\nHSI_LL: Resetting HSI PHY Driver.\n");
#endif
	if(0 > hsi_ioctl(hsi_ll_data.dev[0], HSI_IOCTL_SW_RESET, NULL)) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"HSI_LL: HSI PHY rest returned error.\n");
//		printk("HSI_LL: HSI PHY rest returned error.\n");
#endif
	}

	hsi_ll_data.tx_cmd.count	   = 0;
	hsi_ll_data.tx_cmd.write_index = 0;
	hsi_ll_data.tx_cmd.read_index  = 0;
	hsi_ll_if.msg_avaliable_flag   = 0;

#if !defined(CONFIG_FLASHLESS_DEFAULT_BOOT_HSI)
	for(ch_i=0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		hsi_open(hsi_ll_data.dev[ch_i]);
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Reconfiguring HSI Buses.\n");
//	printk("\nHSI_LL: Reconfiguring HSI Buses.\n");
#endif
	if(0 > hsi_ll_config_bus()) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL: HSI bus reconfiguration failed. Recovery failed.\n");
//		printk("HSI_LL: HSI bus reconfiguration failed. Recovery failed.\n");
#endif
		return -1;
	}
	hsi_ll_data.state = HSI_LL_IF_STATE_READY;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Resuming read/write threads.\n");
//	printk("\nHSI_LL: Resuming read/write threads.\n");
#endif
	hsi_ll_if.wr_complete_flag = 2;
	hsi_ll_if.rd_complete_flag = 2;
	wake_up_interruptible(&hsi_ll_if.wr_complete);
	wake_up_interruptible(&hsi_ll_if.rd_complete);
	hsi_ll_data.initialized = TRUE;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
      dynadbg_module(DYNADBG_CRIT|DYNADBG_INIT_EXIT,"\nHSI_LL: DLP recovery completed.\n");
//	printk("\nHSI_LL: DLP recovery completed.\n");
#endif
#else
	hsi_unregister_driver(&hsi_ll_iface);
	hsi_ll_if.reg_complete_ch_count = 0;
	hsi_ll_if.reg_complete_flag = 0;
	hsi_ll_data.initialized = TRUE;
#endif /* CONFIG_FLASHLESS_DEFAULT_BOOT_HSI */

	return 0;
}

/**
 * hsi_ll_ioctl - HSI LL IOCTL
 * @channel: Channel Number.
 * @command: command to execute
 * @arg: pointer to a variable specific to command.
 */
int hsi_ll_ioctl(int channel, int command, void *arg)
{
	int ret;

	if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT)	// if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY)
		return HSI_LL_RESULT_ERROR;

	switch(command) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
	case HSI_LL_IOCTL_RX_RESUME: {
		if(hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_BLOCKED) {
			struct hsi_ll_rx_tx_data *tmp = (struct hsi_ll_rx_tx_data*)arg;
			if(arg == NULL) {
				return HSI_LL_RESULT_ERROR;
			}
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_READ,"\nHSI_LL: Got mem(size=%d), so unblocking ch %d.\n",
						tmp->size,channel);
//			printk("\nHSI_LL: Got mem(size=%d), so unblocking ch %d.\n",
//						tmp->size,channel);
#endif
			hsi_ll_data.ch[channel].rx.buffer = tmp->buffer;
			hsi_ll_data.ch[channel].rx.size   = tmp->size;
			hsi_ll_send_command(HSI_LL_MSG_ACK, channel,
								&tmp->size);
			HSI_LL_GET_SIZE_IN_WORDS(tmp->size);
			hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_RX;
			hsi_read(hsi_ll_data.dev[channel],
				tmp->buffer,
				tmp->size);
			ret = HSI_LL_RESULT_SUCCESS;
		} else {
			ret = HSI_LL_RESULT_ERROR;
		}
		}
		break;
#endif

	case HSI_LL_IOCTL_TX_RESUME: {  //HSI LL Write recovery
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            printk("\nHSI_LL: %s. HSI_LL_IOCTL_TX_RESUME - ch 0. txstate %d rxstate %d, ch %d. txstate %d rxstate %d\n", __func__,
                        hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state, hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].rx.state,
                        channel, hsi_ll_data.ch[channel].tx.state, hsi_ll_data.ch[channel].rx.state );
#endif
            if( hsi_ll_data.ch[channel].tx.state != HSI_LL_TX_STATE_IDLE ) {
                hsi_write_cancel(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL]);
                hsi_write_cancel(hsi_ll_data.dev[channel]);
                hsi_ll_data.ch[channel].tx.size = 0;
                hsi_ll_data.ch[channel].tx.buffer = NULL;
                hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;

                if( hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state != HSI_LL_TX_STATE_IDLE ) {
                    //hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].rx.state = HSI_LL_RX_STATE_IDLE;
                    hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_IDLE;

                    /* Raise an event */
                    hsi_ll_if.wr_complete_flag = 1;
                    wake_up_interruptible(&hsi_ll_if.wr_complete);
                }

                ret = HSI_LL_RESULT_SUCCESS;    //tx_resume success
            } else {
                ret = HSI_LL_RESULT_ERROR;  //no need tx_resume
            }
        }
        break;

	default:
		ret = HSI_LL_RESULT_ERROR;
		break;
	}

	return ret;
}

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
int hsi_ll_phy_drv_init(void)
{   //#y00185015#20110924#Add for distinguish HSIswh mode: FALSE,hsh; TRUE,ssh#
if(hsi_ll_data.initialized == FALSE)   //#y00185015#20110924#Add for distinguish HSIswh mode: FALSE,hsh; TRUE,ssh#
{//hsi_ll_data.initialized 未被显式初始化，默认为0，即: FALSE;
	int result;
	int ch_i;
	hsi_ll_iface.ch_mask[0] = (0xFFFFFFFF ^ 
							  (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));

       dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nHSI_LL:HSI physical Layer initiated. %s. hsi_ll_data.initialized=%d,hsi_ll_data.state=%d\n",
				__func__,hsi_ll_data.initialized,hsi_ll_data.state);
//    pr_info("\nHSI_LL:HSI physical Layer initiated. %s. hsi_ll_data.initialized=%d,hsi_ll_data.state=%d\n",
//				__func__,hsi_ll_data.initialized,hsi_ll_data.state);

	hsi_ll_data.state = HSI_LL_IF_STATE_READY;	
	result = hsi_register_driver(&hsi_ll_iface);
	if (result) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Error while registering with HSI PHY driver"
				" err=%d. %s %d\n", result, __func__, __LINE__);
//		printk("\nHSI_LL:Error while registering with HSI PHY driver"
//				" err=%d. %s %d\n", result, __func__, __LINE__);
#endif
		result = HSI_LL_RESULT_INIT_ERROR;
	}

    //wait_event_interruptible(hsi_ll_if.reg_complete, hsi_ll_if.reg_complete_flag == 1);
//wait wr_th, rd_th, and HSI controller initialize over!
    wait_event_interruptible(hsi_ll_if.reg_complete, hsi_ll_if.reg_complete_flag == 2);
    hsi_ll_if.reg_complete_flag = 1;

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
       dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\n HSI_LL: %s. hsi_ll_data.state=%d\n",__func__,hsi_ll_data.state);
//    printk("\n HSI_LL: %s. hsi_ll_data.state=%d\n",__func__,hsi_ll_data.state);
#endif
	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
		wait_event_interruptible(hsi_ll_if.reg_complete,
							 hsi_ll_if.reg_complete_flag == 1);
		for(ch_i=0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
			hsi_open(hsi_ll_data.dev[ch_i]);
		}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: %s. Reconfiguring HSI Buses.\n",__func__);
//		printk("\nHSI_LL: %s. Reconfiguring HSI Buses.\n",__func__);
#endif
		if(0 > hsi_ll_config_bus()) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL: %s. HSI bus reconfiguration failed. Recovery failed.\n",__func__);
//			printk("HSI_LL: %s. HSI bus reconfiguration failed. Recovery failed.\n",__func__);
#endif
			return -1;
		}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: %s. Resuming read/write threads.\n",__func__);
//		printk("\nHSI_LL: %s. Resuming read/write threads.\n",__func__);
#endif
		hsi_ll_if.rd_complete_flag = 2;
		hsi_ll_if.wr_complete_flag = 2;
		wake_up_interruptible(&hsi_ll_if.rd_complete);
		wake_up_interruptible(&hsi_ll_if.wr_complete);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: %s. DLP recovery completed.\n",__func__);
//		printk("\nHSI_LL: %s. DLP recovery completed.\n",__func__);
#endif
	}

	return result;
}

else    //if(hsi_ll_data.initialized == FALSE)   //#y00185015#20110924#Add for distinguish HSIswh mode: FALSE,hsh; TRUE,ssh#
//int hsi_ll_phy_drv_reinit(void)
{
	int result;
	int ch_i;
	hsi_ll_iface.ch_mask[0] = (0xFFFFFFFF ^ 
							  (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));

      dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nHSI_LL:HSI physical Layer reinitiated. %s. hsi_ll_data.initialized=%d,hsi_ll_data.state=%d\n",
                        __func__,hsi_ll_data.initialized,hsi_ll_data.state);
//   pr_info("\nHSI_LL:HSI physical Layer reinitiated. %s. hsi_ll_data.initialized=%d,hsi_ll_data.state=%d\n",
//                    __func__,hsi_ll_data.initialized,hsi_ll_data.state); // /* Add for Flashless HSI from boot to norm hsi_ll_open error!*/
/* < Add end for AP_Modem communication module dynamic log > */

    hsi_ll_data.state = HSI_LL_IF_STATE_READY;		// /* Modify for Flashless HSI from boot to norm hsi_ll_open error!*/

	result = hsi_register_driver(&hsi_ll_iface);
	if (result) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:Error while registering with HSI PHY driver"
				" err=%d. %s %d\n", result, __func__, __LINE__);
//		printk("\nHSI_LL:Error while registering with HSI PHY driver"
//				" err=%d. %s %d\n", result, __func__, __LINE__);
#endif
		result = HSI_LL_RESULT_INIT_ERROR;
	}
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
        dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\n HSI_LL:%s. hsi_ll_data.state=%d\n",__func__,hsi_ll_data.state);    //#20110822#y00185015#
//     printk("\n HSI_LL:%s. hsi_ll_data.state=%d\n",__func__,hsi_ll_data.state);    //#20110822#y00185015#
#endif

	{//if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {  //#y00185015#20110924#Add for distinguish HSIswh mode: FALSE,hsh; TRUE,ssh#
		wait_event_interruptible(hsi_ll_if.reg_complete,
							 hsi_ll_if.reg_complete_flag == 1);

#if defined (HSI_SWITCH_SWRESET_ENABEL)
        if( hsi_open(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL]) < 0 ) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL:%s. Could not open channel %d. \n",__func__, HSI_LL_CTRL_CHANNEL);
//         printk("HSI_LL:%s. Could not open channel %d. \n",__func__, HSI_LL_CTRL_CHANNEL);
#endif
        }
        if( hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_SW_RESET, NULL) < 0 )
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL: %s. HSI PHY rest returned error. \n", __func__);
//         pr_err("HSI_LL: %s. HSI PHY rest returned error. \n", __func__);
#endif   //#if defined (HSI_SWITCH_SWRESET_ENABEL)

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
        dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_ll_data.ch[%d].open=%d, hsi_ll_data.state=%d\n",__func__,
                HSI_LL_CTRL_CHANNEL,hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].open,hsi_ll_data.state);
//     pr_info("\nHSI_LL:%s. hsi_ll_data.ch[%d].open=%d, hsi_ll_data.state=%d\n",__func__,
//                HSI_LL_CTRL_CHANNEL,hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].open,hsi_ll_data.state);
#endif

	result = hsi_ll_open(HSI_LL_CTRL_CHANNEL);
	if (result) {
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_ll_open(%d) fail. err=%d",__func__,HSI_LL_CTRL_CHANNEL,result);
//         pr_err("\nHSI_LL:%s. hsi_ll_open(%d) fail. err=%d",__func__,HSI_LL_CTRL_CHANNEL,result);

		//result = HSI_LL_RESULT_INIT_ERROR;
//		hsi_ll_data.initialized = FALSE;
//		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		//return -1;
	}

	for(ch_i=1; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		/*Note: This is just a WA to receive packets on channels that
				are not opened by application, with this WA it is possible
				to receive and drop broadcast packets on channels
				close/not opened by application */
		result = hsi_ll_open(ch_i);
		if (result != 0)
                dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_ll_open(%d) fail. err=%d",__func__,ch_i,result);
//             pr_err("\nHSI_LL:%s. hsi_ll_open(%d) fail. err=%d",__func__,ch_i,result);
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: %s. Reconfiguring HSI Buses.\n",__func__);
//		printk("\nHSI_LL: %s. Reconfiguring HSI Buses.\n",__func__);
#endif
		if(0 > hsi_ll_config_bus()) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                   dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL: %s. HSI bus reconfiguration failed. Recovery failed.\n",__func__);
//			printk("HSI_LL: %s. HSI bus reconfiguration failed. Recovery failed.\n",__func__);
#endif
			return -1;
		}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH=%d) start!\n",__func__,HSI_LL_WAKE_LINE_HIGH);
//         printk("\nHSI_LL:%s. hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH=%d) start!\n",__func__,HSI_LL_WAKE_LINE_HIGH);
#endif
            hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: %s. Resuming read/write threads.\n",__func__);
//		printk("\nHSI_LL: %s. Resuming read/write threads.\n",__func__);
#endif
		hsi_ll_if.wr_complete_flag = 2;
		//hsi_ll_if.rd_complete_flag = 2;
        
            hsi_ll_if.rd_complete_flag = 1; //#y00185015#20110926#Add for HSIswh.ssw, else error#
		wake_up_interruptible(&hsi_ll_if.rd_complete);
		wake_up_interruptible(&hsi_ll_if.wr_complete);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: %s. DLP recovery completed.\n",__func__);
//		printk("\nHSI_LL: %s. DLP recovery completed.\n",__func__);
#endif
	}
	return result;
}
}   //#y00185015#20110924#Add for distinguish HSIswh mode: FALSE,hsh; TRUE,ssh#

int hsi_ll_phy_drv_exit(void)
{
	int ch_i=0;

        dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nHSI_LL:HSI physical Layer Exit. %s hsi_ll_data.initialized=%d,hsi_ll_data.state=%d;\n",
                        __func__,hsi_ll_data.initialized,hsi_ll_data.state);    //#20110822#y00185015#
//     pr_err("\nHSI_LL:HSI physical Layer Exit. %s hsi_ll_data.initialized=%d,hsi_ll_data.state=%d;\n",
//                        __func__,hsi_ll_data.initialized,hsi_ll_data.state);    //#20110822#y00185015#

#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
                flush_workqueue(hsi_ll_if.hsi_tx_retry_wq);
#endif

        hsi_ll_data.tx_cmd.count       = 0;
        hsi_ll_data.tx_cmd.write_index = 0;
        hsi_ll_data.tx_cmd.read_index  = 0;
        hsi_ll_if.msg_avaliable_flag   = 0;

        if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Stop CMD transfer till recovery completes.\n");
//             printk("\nHSI_LL: Stop CMD transfer till recovery completes.\n");
#endif
            hsi_ll_if.wr_complete_flag = 2;
            hsi_ll_if.reg_complete_flag = HSI_LL_REG_UNCOMPLETE;
            wake_up_interruptible(&hsi_ll_if.wr_complete);
        }
        //hsi_ll_if.wr_complete_flag = 1;
        //wake_up_interruptible(&hsi_ll_if.wr_complete);
#if defined (HSI_LL_ENABLE_PM)
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL: Revoking PSV enable.\n");
//             printk("\nHSI_LL: Revoking PSV enable.\n");
#endif
        hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
#endif

    {//if (hsi_ll_data.initialized) {   //#y00185015#20110919#Add for switch HSI#
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
       dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_ll_data.tx_cfg.ac_wake=%d, start hsi_ll_wakeup_cp(%d);\n",__func__,
                           hsi_ll_data.tx_cfg.ac_wake,HSI_IOCTL_ACWAKE_DOWN);
//    pr_info("\nHSI_LL:%s. hsi_ll_data.tx_cfg.ac_wake=%d, start hsi_ll_wakeup_cp(%d);\n",__func__,
//                        hsi_ll_data.tx_cfg.ac_wake,HSI_IOCTL_ACWAKE_DOWN);
#endif
		//hsi_ll_close(HSI_LL_CTRL_CHANNEL);
		//hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_ACWAKE_DOWN, NULL);
        hsi_ll_wakeup_cp(HSI_IOCTL_ACWAKE_DOWN);

    for(ch_i=HSI_LL_MAX_CHANNELS-1; ch_i >= HSI_LL_CTRL_CHANNEL; ch_i--) {
        hsi_ll_close(ch_i);

        if (hsi_ll_data.ch[ch_i].open == TRUE) {
                hsi_ll_data.ch[ch_i].open = FALSE;

            hsi_ll_data.ch[ch_i].rx.close_req = TRUE;//FALSE;
            hsi_ll_data.ch[ch_i].tx.close_req = TRUE;//FALSE;
            hsi_ll_data.ch[ch_i].rx.state = HSI_LL_RX_STATE_CLOSED;
            /* Upper layers should take care that noting is sent on this channel till it is opened again */
            hsi_ll_data.ch[ch_i].tx.state = HSI_LL_TX_STATE_CLOSED;
            //hsi_close(hsi_ll_data.dev[ch_i]);
            //hsi_ll_data.ch[ch_i].open = FALSE;

            hsi_write_cancel(hsi_ll_data.dev[ch_i]);  //#y00185015#20110919#Add for switch HSI#
            hsi_read_cancel(hsi_ll_data.dev[ch_i]);  //#y00185015#20110919#Add for switch HSI#

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_close(hsi_ll_data.dev[%d]);\n",__func__,ch_i);
//         pr_info("\nHSI_LL:%s. hsi_close(hsi_ll_data.dev[%d]);\n",__func__,ch_i);
#endif
            hsi_close(hsi_ll_data.dev[ch_i]);
        }
    }

#if defined (HSI_SWITCH_SWRESET_ENABEL)
        if( hsi_open(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL]) < 0 ) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL:%s. Could not open channel %d. \n", __func__, HSI_LL_CTRL_CHANNEL);
//         printk("HSI_LL:%s. Could not open channel %d. \n", __func__, HSI_LL_CTRL_CHANNEL);
#endif
        }
        if( hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_SW_RESET, NULL) < 0 )
            dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"HSI_LL: %s. HSI PHY rest returned error. \n",__func__);
//         pr_err("HSI_LL: %s. HSI PHY rest returned error. \n",__func__);
#endif   //#if defined (HSI_SWITCH_SWRESET_ENABEL)

        dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nHSI_LL:%s. hsi_unregister_driver(&hsi_ll_iface) start;\n",__func__);
//     pr_info("\nHSI_LL:%s. hsi_unregister_driver(&hsi_ll_iface) start;\n",__func__);
		hsi_unregister_driver(&hsi_ll_iface);
		hsi_ll_data.initialized = FALSE;
        
            hsi_ll_if.reg_complete_ch_count = 0;
            hsi_ll_if.reg_complete_flag = 0;
#if defined (HSI_LL_ENABLE_PM)
		hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
#endif
            hsi_ll_if.wr_complete_flag = 0;
            hsi_ll_if.rd_complete_flag = 0;

            hsi_ll_data.initialized = TRUE; //#y00185015#20110924#Add for distinguish HSIswh mode: FALSE,hsh; TRUE,ssh#
    }
	return HSI_LL_RESULT_SUCCESS;
}
#endif



// /* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3) */
static int xmd_hsi_ll_config_bus(int rx_divisor, int tx_divisor)
{
	int ret = HSI_LL_RESULT_SUCCESS;

        dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"HSI_LL: %s([rx_divisor]%d,[tx_divisor]%d).\n",__func__,rx_divisor,tx_divisor);
//     pr_info("HSI_LL: %s([rx_divisor]%d,[tx_divisor]%d).\n",__func__,rx_divisor,tx_divisor); // /* Add begin for Flashless switch to Norm_HSI AT block HSI communication */

	hsi_ll_data.rx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.rx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.rx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.rx_cfg.ctx.divisor    = rx_divisor; //HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.rx_cfg.ctx.counters   = HSI_LL_COUNTERS_VALUE;
	hsi_ll_data.rx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_RX,
				    &hsi_ll_data.rx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:RX config error %d. %s %d\n", ret, __func__, __LINE__);
//		printk("\nHSI_LL:RX config error %d. %s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;

		return HSI_LL_RESULT_ERROR;
	}

	hsi_ll_data.tx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.tx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.tx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.tx_cfg.ctx.divisor    = tx_divisor; //HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.tx_cfg.ctx.arb_mode   = HSI_LL_ARBMODE_MODE;
	hsi_ll_data.tx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;


	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_TX,
					&hsi_ll_data.tx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nHSI_LL:TX config error %d.%s %d\n", ret, __func__, __LINE__);
//		printk("\nHSI_LL:TX config error %d.%s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return HSI_LL_RESULT_ERROR;
	}
	return ret;
}

#include "xmd_boot_tty.h"

static DEFINE_MUTEX(xmd_ll_hsimode_mutex);

int hsi_ll_get_hsimode(void)
{
int hsimode=0;

    mutex_lock(&xmd_ll_hsimode_mutex);
//norm_hsimode, HSI模式配置值;
    hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_GET_HSI_MODE, (void*)&hsimode);    
    mutex_unlock(&xmd_ll_hsimode_mutex);

    return hsimode;
}
int hsi_ll_set_hsimode(int hsimode)
{
//    int port=1; //pport->hsi_channel[channel].dev->ch->hsi_port->port_number
int result=0, norm_hsimode=-1;

//HSI_IOCTL_GET_HSI_MODE,   /* 16, Get HSI hsi_mode value for all channel */
    hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_GET_HSI_MODE, (void*)&norm_hsimode);

    dynadbg_module(DYNADBG_NOTICE|DYNADBG_WRITE,"\nHSI_LL: %s. begin! [hsimode]0x%08x -> 0x%08x;\n",__func__,norm_hsimode,hsimode);
// printk("\nHSI_LL: %s. begin! [hsimode]0x%08x -> 0x%08x;\n",__func__,norm_hsimode,hsimode);
    
    mutex_lock(&xmd_ll_hsimode_mutex);
    
    if( norm_hsimode==hsimode ) {
        dynadbg_module(DYNADBG_ALERT|DYNADBG_WRITE,"HSI_LL: %s. wrong hsimode shift, but continue!\n",__func__);
//     pr_warn("HSI_LL: %s. wrong hsimode shift, but continue!\n",__func__);
        //mutex_unlock(&xmd_ll_hsimode_mutex);
        //return status;
    }

//HSI_IOCTL_SET_HSI_MODE,	/* 15, Set HSI hsi_mode value for all channel */
    hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL], HSI_IOCTL_SET_HSI_MODE, (void*)&hsimode);

//XMM_HSI_SOFT_RESET, HSI SOFT_RESET标志;
        if( (bool)(hsimode&XMM_HSI_SOFT_RESET)==true ) {  //XMM_HSI_SOFT_RESET, HSI SOFT_RESET标志;
            result = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
                              HSI_IOCTL_SW_RESET,
                              NULL);
            if (result)
                dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\n HSI_LL: XMM_HSI_SOFT_RESET error %d. %s %d\n", result, __func__, __LINE__);
//             printk("\n HSI_LL: XMM_HSI_SOFT_RESET error %d. %s %d\n", result, __func__, __LINE__);
            //result = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],HSI_IOCTL_SW_RESET,NULL);
        }
//XMM_HSI_OPEN_AGAIN, HSI hsi_open again标志;
        //HSI_IOCTL_SW_RESET -> hsi_open(hsi_ll_data.dev[ch_i])
        if( (bool)(hsimode&XMM_HSI_OPEN_AGAIN)==true )  //XMM_HSI_OPEN_AGAIN, HSI hsi_open again标志;
        {   int ch_i=HSI_LL_CTRL_CHANNEL;
                for(ch_i=HSI_LL_CTRL_CHANNEL; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
                    if (hsi_ll_data.ch[ch_i].open == TRUE) {
                        dynadbg_module(DYNADBG_DEBUG|DYNADBG_WRITE,"\nHSI_LL: %s. ch %d hsi_open start.\n",__func__,ch_i);
//                     printk("\nHSI_LL: %s. ch %d hsi_open start.\n",__func__,ch_i);
                        hsi_open(hsi_ll_data.dev[ch_i]);  //#y00185015#20110919#Add for switch HSI#
                    }
                }
        }
//XMM_HSI_RX_CLK_SET; //XMM_HSI_TX_CLK_SET; //XMM_HSI_RX_CLK,XMM_HSI_TX_CLK;
        if( (bool)(hsimode&XMM_HSI_RX_CLK_SET)==true && (bool)(hsimode&XMM_HSI_TX_CLK_SET)==true ) {  //XMM_HSI_RX_CLK_SET; //XMM_HSI_TX_CLK_SET;
         // /* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3) */
             result = xmd_hsi_ll_config_bus(XMM_MODE_GET_DIV_RX(hsimode),XMM_MODE_GET_DIV_TX(hsimode));
             if (result)
                 dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\n HSI_LL: result error %d.%s %d\n", result, __func__, __LINE__);
//              printk("\n HSI_LL: result error %d.%s %d\n", result, __func__, __LINE__);
        }

//XMM_HSI_FLUSH_RX, HSI FLUSH_RX标志;
        if( (bool)(hsimode&XMM_HSI_FLUSH_RX)==true ) {  //XMM_HSI_FLUSH_RX, HSI FLUSH_RX标志;
        //#20110913#y00185015#Add for hsr_fifo_flush.patch#
             result = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
                                  HSI_IOCTL_FLUSH_RX,
                                  NULL);
             if (result)
                 dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\n HSI_LL: HSI_IOCTL_FLUSH_RX error %d. %s %d\n", result, __func__, __LINE__);
//              printk("\n HSI_LL: HSI_IOCTL_FLUSH_RX error %d. %s %d\n", result, __func__, __LINE__);
        }
//XMM_HSI_FLUSH_TX, HSI FLUSH_TX标志;
        if( (bool)(hsimode&XMM_HSI_FLUSH_TX)==true ) {  //XMM_HSI_FLUSH_TX, HSI FLUSH_TX标志;
            result = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
                              HSI_IOCTL_FLUSH_TX,
                              NULL);
            if (result)
                dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\n HSI_LL: HSI_IOCTL_FLUSH_TX error %d. %s %d\n", result, __func__, __LINE__);
//             printk("\n HSI_LL: HSI_IOCTL_FLUSH_TX error %d. %s %d\n", result, __func__, __LINE__);
        }

//XMM_HSI_FLUSH_RXSTATE, HSI FLUSH_RXSTATE标志;
        if( (bool)(hsimode&XMM_HSI_FLUSH_RXSTATE)==true ) {  //XMM_HSI_FLUSH_RXSTATE, HSI FLUSH_RXSTATE标志;
        //#20110913#y00185015#Add for hsr_fifo_flush.patch#
             result = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
                                  HSI_IOCTL_FLUSH_RXSTATE,
                                  NULL);
             if (result)
                 dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\n HSI_LL: HSI_IOCTL_FLUSH_RXSTATE error %d. %s %d\n", result, __func__, __LINE__);
//              printk("\n HSI_LL: HSI_IOCTL_FLUSH_RXSTATE error %d. %s %d\n", result, __func__, __LINE__);
        }
//XMM_HSI_FLUSH_TXSTATE, HSI FLUSH_TXSTATE标志;
        if( (bool)(hsimode&XMM_HSI_FLUSH_TXSTATE)==true ) {  //XMM_HSI_FLUSH_TXSTATE, HSI FLUSH_TXSTATE标志;
            result = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
                              HSI_IOCTL_FLUSH_TXSTATE,
                              NULL);
            if (result)
                dynadbg_module(DYNADBG_ERR|DYNADBG_WRITE,"\n HSI_LL: HSI_IOCTL_FLUSH_TXSTATE error %d. %s %d\n", result, __func__, __LINE__);
//             printk("\n HSI_LL: HSI_IOCTL_FLUSH_TXSTATE error %d. %s %d\n", result, __func__, __LINE__);
        }
//XMM_HSI_OPEN_AGAIN, HSI hsi_open again标志;
        if( (bool)(hsimode&XMM_HSI_OPEN_AGAIN)==true ) {  //XMM_HSI_OPEN_AGAIN, HSI hsi_open again标志;
            hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);
            dynadbg_module(DYNADBG_NOTICE|DYNADBG_WRITE,"\nHSI_LL: %s. ch %d hsi_read start.\n",__func__,HSI_LL_CTRL_CHANNEL);
//         printk("\nHSI_LL: %s. ch %d hsi_read start.\n",__func__,HSI_LL_CTRL_CHANNEL);
            hsi_read(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],&hsi_ll_data.rx_cmd,1);
        }

        //norm_hsimode = hsimode;    //boot_hsimode, HSI模式配置值;
        mutex_unlock(&xmd_ll_hsimode_mutex);

    dynadbg_module(DYNADBG_NOTICE|DYNADBG_WRITE,"\n HSI_LL: %s. end!\n",__func__); //y00185015
// printk("\n HSI_LL: %s. end!\n",__func__); //y00185015
    
    return( result );
}


