/*
 * xmd-hsi-mcm.c
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
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


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "xmd-ch.h"
#include "xmd-hsi-mcm.h"
#include "xmd-hsi-ll-if.h"
#include "xmd_hsi_mem.h"
#include <linux/delay.h>
#include <linux/slab.h>


#define MCM_DBG_LOG 1   //#define MCM_DBG_LOG 0
#define MCM_DBG_ERR_LOG 1   //#define MCM_DBG_ERR_LOG 0
#define MCM_DBG_ERR_RECOVERY_LOG 1   //#define MCM_DBG_ERR_RECOVERY_LOG 0

#include "xmd-hsi-ll-cfg.h"
extern uint32_t dynamic_debug_mask;
#define dynadbg_module(mask, x...)    dynamic_debug( ((mask)|(DYNADBG_HSI_MCM_EN)), x)


#define MCM_DBG_ENABLE

#define XMD_TTY_WRITE_TIMEOUT_DEFAULT		(3000)      //xmd_tty_write wait_event timeout default value (ms)

static struct hsi_chn hsi_all_channels[MAX_HSI_CHANNELS] = {
	{"CONTROL",  HSI_CH_NOT_USED},
	{"CHANNEL1", HSI_CH_FREE},
	{"CHANNEL2", HSI_CH_FREE},
	{"CHANNEL3", HSI_CH_FREE},
	{"CHANNEL4", HSI_CH_FREE},
	{"CHANNEL5", HSI_CH_FREE},
	{"CHANNEL6", HSI_CH_FREE},
	{"CHANNEL7", HSI_CH_FREE},
	{"CHANNEL8", HSI_CH_FREE},
	{"CHANNEL9", HSI_CH_FREE},
	{"CHANNEL10",HSI_CH_FREE},
	{"CHANNEL11",HSI_CH_FREE},
	{"CHANNEL12",HSI_CH_FREE},
	{"CHANNEL13",HSI_CH_FREE},
	{"CHANNEL14",HSI_CH_FREE},
	{"CHANNEL15",HSI_CH_FREE},
};

static unsigned int hsi_mcm_state;
static int is_dlp_reset_in_progress;

static struct work_struct XMD_DLP_RECOVERY_wq;
static struct hsi_channel hsi_channels[MAX_HSI_CHANNELS];
static struct workqueue_struct *hsi_read_wq;
static struct workqueue_struct *hsi_write_wq;
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
static struct workqueue_struct *hsi_buf_retry_wq;
#endif

void (*xmd_boot_cb)(void);
static void hsi_read_work(struct work_struct *work);
static void hsi_write_work(struct work_struct *work);

static void xmd_dlp_recovery(void);
static void xmd_ch_reinit(void);
static void xmd_dlp_recovery_wq(struct work_struct *cp_crash_wq);
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
static void hsi_buf_retry_work(struct work_struct *work);
#endif
extern void ifx_pmu_reset(void);
extern void rmnet_restart_queue(int chno);

void init_q(int chno)
{
	int i;

	hsi_channels[chno].rx_q.head  = 0;
	hsi_channels[chno].rx_q.tail  = 0;
	hsi_channels[chno].tx_q.head  = 0;
	hsi_channels[chno].tx_q.tail  = 0;
	hsi_channels[chno].tx_blocked = 0;
	hsi_channels[chno].pending_rx_msgs = 0;
	hsi_channels[chno].pending_tx_msgs = 0;

	for (i=0; i<NUM_X_BUF; i++)
		hsi_channels[chno].rx_q.data[i].being_used = HSI_FALSE;
}

/* Head grows on reading from q. "data=q[head];head++;" */
struct x_data* read_q(int chno, struct xq* q)
{
	struct x_data *data = NULL;

	if (!q) {
#if MCM_DBG_ERR_LOG
            dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"mcm: NULL Q instance");
//	      printk("mcm: NULL Q instance");
#endif
		return NULL;
	}

#if MCM_DBG_LOG
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nmcm: [read_q]  head = %d, tail = %d\n",q->head,q->tail);
//	printk("\nmcm: [read_q]  head = %d, tail = %d\n",q->head,q->tail);
#endif

	spin_lock_bh(&hsi_channels[chno].lock);

	if (q->head == q->tail) {
		spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nmcm: Q empty [read] \n");
//		printk("\nmcm: Q empty [read] \n");
#endif
		return NULL;
	}

	data = q->data + q->head;
	q->head = (q->head + 1) % NUM_X_BUF;

	spin_unlock_bh(&hsi_channels[chno].lock);

	return data;
}

/* Tail grows on writing in q. "q[tail]=data;tail++;" */
int write_q(struct xq* q, char *buf, int size, struct x_data **data)
{
	int temp = 0;

	if (!q) {
#if MCM_DBG_ERR_LOG
             dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"mcm: NULL q instance");
//		printk("mcm: NULL q instance");
#endif
		return 0;
	}

	temp = (q->tail + 1) % NUM_X_BUF;

	if (temp != q->head) {
		q->data[q->tail].buf  = buf;
		q->data[q->tail].size = size;
		if (data) {
			*data = q->data + q->tail;
		}
		q->tail = temp;
	} else {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm:Q full [write], head = %d, tail = %d\n",q->head,q->tail);
//          printk("\nmcm:Q full [write], head = %d, tail = %d\n",q->head,q->tail);
#endif
		return 0;
	}

		return q->tail > q->head ? q->tail - q->head:q->tail - q->head + NUM_X_BUF;
}

#ifdef  MCM_DBG_ENABLE
module_param_named(xmd_tty_write_timeout_0, hsi_channels[1].write_timeout, uint, S_IWUSR | S_IRUGO);
module_param_named(xmd_tty_write_timeout_1, hsi_channels[2].write_timeout, uint, S_IWUSR | S_IRUGO);
#endif  //#ifdef  MCM_DBG_ENABLE

/**
 * hsi_ch_ioctl - HSI CH IOCTL
 * @channel: Channel Number.
 * @command: command to execute
 * @arg: pointer to a variable specific to command.
 */
int hsi_ch_ioctl(int channel, int command, void *arg)
{
	int ret;

	if ((channel < 0) || (channel >= MAX_HSI_CHANNELS))
		return HSI_CH_RESULT_INVALID_PARAM;

	switch(command) {
	case HSI_CH_IOCTL_GET_WRITE_TIMEOUT: {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
                dynadbg_module(DYNADBG_DEBUG|DYNADBG_READ,"\nmcm: Get ch %d write_timeout. %d;\n",
                                        channel,hsi_channels[channel].write_timeout );
#endif
                if(arg == NULL) {
                    return HSI_CH_RESULT_ERROR;
                }
                *(int *)arg = hsi_channels[channel].write_timeout;
                ret = HSI_CH_RESULT_SUCCESS;
		}
		break;

	case HSI_CH_IOCTL_SET_WRITE_TIMEOUT: {
                if(arg == NULL) {
                    return HSI_CH_RESULT_ERROR;
                }
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
               dynadbg_module(DYNADBG_DEBUG|DYNADBG_READ,"\nmcm: Set ch %d write_timeout from %d to %d;\n",
					channel,hsi_channels[channel].write_timeout,*(int *)arg );
#endif
                hsi_channels[channel].write_timeout = *(int *)arg;
                ret = HSI_CH_RESULT_SUCCESS;
		}
		break;

	default:
		ret = HSI_CH_RESULT_FAIL;
		break;
	}

	return ret;
}

static int hsi_ch_net_write(int chno, void *data, int len)
{
	/* Non blocking write */
	void *buf = NULL;
	static struct x_data *d = NULL;
	int n = 0;
	int flag = 1;
	int ret = 0;
#ifdef XMD_TX_MULTI_PACKET
	if (d && hsi_channels[chno].write_queued == HSI_TRUE) {
		if (d->being_used == HSI_FALSE && (d->size + len) < HSI_MEM_LARGE_BLOCK_SIZE) {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: Adding in the queued buffer for ch %d\n",chno);
//			printk("\nmcm: Adding in the queued buffer for ch %d\n",chno);
#endif
			buf = d->buf + d->size;
			d->size += len;
			flag = 0;
		} else {
			flag = 1;
		}
	}
#endif
	if (flag) {
#ifdef XMD_TX_MULTI_PACKET
		buf = hsi_mem_alloc(HSI_MEM_LARGE_BLOCK_SIZE);
#else
		buf = hsi_mem_alloc(len);
#endif
		flag = 1;
	}

	if (!buf || !data) {
#if MCM_DBG_ERR_LOG
         dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nmcm: Failed to alloc memory So Cannot transfer packet.\n");
//		printk("\nmcm: Failed to alloc memory So Cannot transfer packet.\n");
#endif
		return -ENOMEM;
	}

	memcpy(buf, data, len);

	if (flag) {
		d = NULL;
		n = write_q(&hsi_channels[chno].tx_q, buf, len, &d);

		if (n != 0) {
			hsi_channels[chno].pending_tx_msgs++;
		}
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: n = %d\n",n);
//		printk("\nmcm: n = %d\n",n);
#endif
		if (n == 0) {
#if MCM_DBG_LOG
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: rmnet TX queue is full for channel %d,"
					" So cannot transfer this packet.\n",chno);
//			printk("\nmcm: rmnet TX queue is full for channel %d,"
//					" So cannot transfer this packet.\n",chno);
#endif
			hsi_channels[chno].tx_blocked = 1;
			hsi_mem_free(buf);
			PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
			queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
			ret = -EBUSY;
		} else if (n == 1) {
			PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
			queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
			ret = 0;
		}
	}

	return ret;
}

/**
 * hsi_ch_tty_write - HSI CH TTY WRITE
 * @channel: Channel Number.
 * @data: write data address
 * @len: data length.
 * return - HSI_LL_RESULT_SUCCESS(0), success; <0, failed;
 */
static int hsi_ch_tty_write(int chno, void *data, int len)
{
	void *buf = NULL;
	int err;

	buf = hsi_mem_alloc(len);

	if (!buf) {
		return -ENOMEM;
	}

	memcpy(buf, data, len);

	hsi_channels[chno].write_happening = HSI_TRUE;

	err = hsi_ll_write(chno, (unsigned char *)buf, len);
	if (err < 0) {
#if MCM_DBG_ERR_LOG
             dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nmcm: hsi_ll_write(...) failed. err=%d\n",err);
//		printk("\nmcm: hsi_ll_write(...) failed. err=%d\n",err);
#endif

		hsi_mem_free(buf);
		hsi_channels[chno].write_happening = HSI_FALSE;
	} else {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm:locking mutex for ch: %d\n",chno);
//		printk("\nmcm:locking mutex for ch: %d\n",chno);
#endif

        if( hsi_channels[chno].write_timeout<=0 ) {
    		wait_event (hsi_channels[chno].write_wait,
    					hsi_channels[chno].write_happening == HSI_FALSE);
        } else {
            wait_event_timeout (hsi_channels[chno].write_wait,
                        hsi_channels[chno].write_happening == HSI_FALSE, msecs_to_jiffies(hsi_channels[chno].write_timeout) );
            if( hsi_channels[chno].write_happening != HSI_FALSE ) { /* Time out */
                if( HSI_LL_RESULT_SUCCESS != hsi_ll_ioctl(chno, HSI_LL_IOCTL_TX_RESUME, NULL) ) {
                    dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nmcm: wait_event_timeout %d ms, hsi_ll_ioctl no need tx_resume\n",
                                                    hsi_channels[chno].write_timeout);
                } else {
                    hsi_mem_free(buf);
                    err = -EREMOTEIO;
                }
            }
        }
    }

	return err;
}

static void* hsi_ch_net_read(int chno, int* len)
{
	struct x_data *data = hsi_channels[chno].curr;
	*len = data->size;
	return data->buf;
}

static void* hsi_ch_tty_read(int chno, int* len)
{
	struct x_data *data = hsi_channels[chno].curr;
	*len = data->size;
	return data->buf;
}

int xmd_ch_write(int chno, void *data, int len)
{
	int err;

#if MCM_DBG_LOG
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: write entering, ch %d\n",chno);
//	printk("\nmcm: write entering, ch %d\n",chno);
#endif

    // "/dev/xmd-tty5"(ch6) for USB switch
    if( (6!=chno)||(( (dynamic_debug_mask)&((DYNADBG_PROBE_EN)|(DYNADBG_GLOBAL_EN)) )!=((DYNADBG_PROBE_EN)|(DYNADBG_GLOBAL_EN)) ) ) {
        if( is_cp_ready()!=true ) {
            dynadbg_module(DYNADBG_NOTICE|DYNADBG_WRITE,"mcm: CP is not ready now!\n");
            return -EBUSY;
        }
    }

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT) && 0  //No need check here, is_cp_ready() will do the same thing;
      if( get_hsi_drvmode()!=XMD_HSI_DRVMODE_NORM ) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
	    dynadbg_module(DYNADBG_DEBUG|DYNADBG_WRITE,"\nmcm: Invalid state for hsi_drvmode, XMD_HSI_DRVMODE_NORM now!\n");
//    printk("\nmcm: Invalid state for hsi_drvmode, XMD_HSI_DRVMODE_NORM now!\n");
#endif
	    return -ENODEV;
      }
#endif  //#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)

	if (!hsi_channels[chno].write) {
#if MCM_DBG_ERR_LOG
             dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nmcm:write func NULL for ch: %d\n",chno);
//		printk("\nmcm:write func NULL for ch: %d\n",chno);
#endif
		return -EINVAL;
	}

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
                   dynadbg_module(DYNADBG_NOTICE|DYNADBG_TX,"\nmcm:Dropping packets of channel %d as "
					"error recovery is in progress\n", chno);
//			printk("\nmcm:Dropping packets of channel %d as "
//					"error recovery is in progress\n", chno);
#endif
			return -ENOTBLK;
	}

	err = hsi_channels[chno].write(chno, data, len);

#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: write returning, ch %d\n",chno);
//          printk("\nmcm: write returning, ch %d\n",chno);
#endif
	return err;
}

void* xmd_ch_read(int chno, int* len)
{
	return hsi_channels[chno].read(chno,len);
}

void xmd_ch_close(int chno)
{
      dynadbg_module(DYNADBG_WARN|DYNADBG_OPEN_CLOSE,"\nmcm:closing channel %d.\n",chno);
//	printk("\nmcm:closing channel %d.\n",chno);

	if(chno == XMD_RIL_RECOVERY_CHANNEL) {
#if MCM_DBG_ERR_RECOVERY_LOG
             dynadbg_module(DYNADBG_NOTICE|DYNADBG_OPEN_CLOSE,"\nmcm: Ch %d closed so starting Recovery.\n",chno);
//		printk("\nmcm: Ch %d closed so starting Recovery.\n",chno);
#endif
		xmd_dlp_recovery();
	}
	if (hsi_channels[chno].read_happening == HSI_TRUE) {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_OPEN_CLOSE,"\nmcm:locking read mutex for ch: %d\n",chno);
//		printk("\nmcm:locking read mutex for ch: %d\n",chno);
#endif
		wait_event(hsi_channels[chno].read_wait,
					hsi_channels[chno].read_happening == HSI_FALSE);
	}
	hsi_ll_close(chno);
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].state = HSI_CH_FREE;
	spin_unlock_bh(&hsi_channels[chno].lock);
}

int xmd_ch_open(struct xmd_ch_info* info, void (*notify_cb)(int chno))
{
	int i;
	int size = ARRAY_SIZE(hsi_channels);

	for (i=0; i<size; i++) {
		if (hsi_channels[i].name)
			if (!strcmp(info->name, hsi_channels[i].name)) {
				if (hsi_channels[i].state == HSI_CH_BUSY ||

					hsi_channels[i].state == HSI_CH_NOT_USED) {
#if MCM_DBG_ERR_LOG
                                dynadbg_module(DYNADBG_ERR|DYNADBG_OPEN_CLOSE,"\nmcm:Channel state not suitable %d\n",i);
//					printk("\nmcm:Channel state not suitable %d\n",i);
#endif
					return -EINVAL;
				}

				if ((i == XMD_RIL_RECOVERY_CHANNEL) &&
					(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY)) {
#if MCM_DBG_ERR_RECOVERY_LOG
                                dynadbg_module(DYNADBG_NOTICE|DYNADBG_OPEN_CLOSE,"\nmcm: Recovery completed.\n");
//					printk("\nmcm: Recovery completed.\n");
#endif
					xmd_ch_reinit();
				}
				if (0 != hsi_ll_open(i)) {
#if MCM_DBG_ERR_LOG
                                dynadbg_module(DYNADBG_ERR|DYNADBG_OPEN_CLOSE,"\nmcm:hsi_ll_open failed for channel %d\n",i);
//					printk("\nmcm:hsi_ll_open failed for channel %d\n",i);
#endif
					return -EINVAL;
				}

				hsi_channels[i].info = info;

				spin_lock_bh(&hsi_channels[i].lock);
				hsi_channels[i].state = HSI_CH_BUSY;
				spin_unlock_bh(&hsi_channels[i].lock);

				hsi_channels[i].notify = notify_cb;
				switch(info->user)
				{
				case XMD_TTY:
					hsi_channels[i].read = hsi_ch_tty_read;
					hsi_channels[i].write = hsi_ch_tty_write;
				break;
				case XMD_NET:
					hsi_channels[i].read = hsi_ch_net_read;
					hsi_channels[i].write = hsi_ch_net_write;
				break;
				default:
#if MCM_DBG_ERR_LOG
                                dynadbg_module(DYNADBG_ERR|DYNADBG_OPEN_CLOSE,"\nmcm:Neither TTY nor NET \n");
//					printk("\nmcm:Neither TTY nor NET \n");
#endif
					return -EINVAL;
				}
				INIT_WORK(&hsi_channels[i].read_work, hsi_read_work);
				INIT_WORK(&hsi_channels[i].write_work, hsi_write_work);
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
				INIT_WORK(&hsi_channels[i].buf_retry_work, hsi_buf_retry_work);
#endif
				return i;
			}
	}
#if MCM_DBG_ERR_LOG
      dynadbg_module(DYNADBG_ERR|DYNADBG_OPEN_CLOSE,"\n Channel name not proper \n");
//	printk("\n Channel name not proper \n");
#endif
	return -EINVAL;
}

void hsi_read_work(struct work_struct *work)
{
	/* function registered with read work q */
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													read_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;

	if (hsi_channels[chno].read_queued == HSI_TRUE) {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nmcm: read wq already in progress\n");
//		printk("\nmcm: read wq already in progress\n");
#endif
		return;
	}

	hsi_channels[chno].read_queued = HSI_TRUE;

	while ((data = read_q(chno, &hsi_channels[chno].rx_q)) != NULL) {
		char *buf = data->buf;
		hsi_channels[chno].curr = data;

		if (hsi_mcm_state != HSI_MCM_STATE_ERR_RECOVERY) {
			hsi_channels[chno].notify(chno);
#if MCM_DBG_ERR_RECOVERY_LOG
		} else {
             dynadbg_module(DYNADBG_NOTICE|DYNADBG_RX,"\nmcm:Dropping RX packets of channel %d from WQ as "
					"error recovery is in progress\n", chno);
//			printk("\nmcm:Dropping RX packets of channel %d from WQ as "
//					"error recovery is in progress\n", chno);
#endif
		}

		hsi_mem_free(buf);
		if(chno >= 13) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
			if(hsi_channels[chno].rx_blocked) {
				hsi_channels[chno].rx_blocked = 0;
				spin_lock_bh(&hsi_channels[chno].lock);
				hsi_channels[chno].pending_rx_msgs++;
				spin_unlock_bh(&hsi_channels[chno].lock);
				PREPARE_WORK(&hsi_channels[chno].buf_retry_work,
							  hsi_buf_retry_work);
				queue_work(hsi_buf_retry_wq,
						  &hsi_channels[chno].buf_retry_work);
			}
#endif
			hsi_channels[chno].pending_rx_msgs--;
		}
	}
	hsi_channels[chno].read_queued = HSI_FALSE;
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].read_happening = HSI_FALSE;
	spin_unlock_bh(&hsi_channels[chno].lock);

	wake_up(&hsi_channels[chno].read_wait);
}

void hsi_write_work(struct work_struct *work)
{
	/* function registered with write work q */
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													write_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;
	int err;

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
             dynadbg_module(DYNADBG_NOTICE|DYNADBG_TX,"\nmcm:Dropping packets of channel %d from WQ as "
				" error recovery is in progress\n", chno);
//		printk("\nmcm:Dropping packets of channel %d from WQ as "
//				" error recovery is in progress\n", chno);
#endif
		goto quit_write_wq;
	}

	if (hsi_channels[chno].write_queued == HSI_TRUE) {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: write wq already in progress\n");
//		printk("\nmcm: write wq already in progress\n");
#endif
		return;
	}

	hsi_channels[chno].write_queued = HSI_TRUE;

	while ((data = read_q(chno, &hsi_channels[chno].tx_q)) != NULL) {
		hsi_channels[chno].write_happening = HSI_TRUE;
		data->being_used = HSI_TRUE;
		err = hsi_ll_write(chno, (unsigned char *)data->buf, data->size);
		if (err < 0) {
#if MCM_DBG_ERR_LOG
                   dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nmcm: hsi_ll_write failed\n");
//			printk("\nmcm: hsi_ll_write failed\n");
#endif

			hsi_mem_free(data->buf);

			hsi_channels[chno].write_happening = HSI_FALSE;
		} else {
#if MCM_DBG_LOG
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm:locking mutex for ch: %d\n",chno);
//			printk("\nmcm:locking mutex for ch: %d\n",chno);
#endif
			wait_event(hsi_channels[chno].write_wait,
						hsi_channels[chno].write_happening == HSI_FALSE);
		}
		hsi_channels[chno].pending_tx_msgs--;
		data->being_used = HSI_FALSE;
		if (hsi_channels[chno].tx_blocked == 1) {
			hsi_channels[chno].tx_blocked = 0;
#if MCM_DBG_LOG
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: Channel queue free , "
					"restarting TX queue for ch %d \n",chno);
//                printk("\nmcm: Channel queue free , "
//					"restarting TX queue for ch %d \n",chno);
#endif
			rmnet_restart_queue(chno);
		}
	}

quit_write_wq:

	hsi_channels[chno].write_queued = HSI_FALSE;
}

#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
static void hsi_buf_retry_work(struct work_struct *work)
{
	struct hsi_ll_rx_tx_data temp_data;
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													buf_retry_work);
	int chno = ch->info->chno;
	temp_data.size = ch->pending_rx_size;
	temp_data.buffer = NULL;

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
			return;
	}
	/*GFP_NOFAIL not available so switching to while loop*/
	while(temp_data.buffer == NULL) {
		temp_data.buffer = kmalloc(temp_data.size,
								   GFP_DMA | GFP_KERNEL);
	}
#if MCM_DBG_LOG
      dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nHSI_LL: Allocating mem(size=%d) in retry Q for ch %d\n",
			temp_data.size,chno);
//	printk("\nHSI_LL: Allocating mem(size=%d) in retry Q for ch %d\n",
//			temp_data.size,chno);
#endif
	if(0 > hsi_ll_ioctl(chno, HSI_LL_IOCTL_RX_RESUME, &temp_data)) {
		kfree(temp_data.buffer);
	}
	ch->pending_rx_size = 0;
}
#endif

void hsi_ch_cb(unsigned int chno, int result, int event, void* arg)
{
	ll_rx_tx_data *data = (ll_rx_tx_data *) arg;

	if (!(chno <= MAX_HSI_CHANNELS && chno >= 0) ||
		hsi_channels[chno].state == HSI_CH_NOT_USED) {
#if MCM_DBG_ERR_LOG
             dynadbg_module(DYNADBG_ERR|DYNADBG_INIT_EXIT,"\nmcm: Wrong channel number or channel not used\n");
//		printk("\nmcm: Wrong channel number or channel not used\n");
#endif
		return;
	}

	switch(event) {
	case HSI_LL_EV_ALLOC_MEM: {
		if(chno >= 13) {
			if (hsi_channels[chno].pending_rx_msgs >= NUM_X_BUF) {
				data->buffer = 0;
#if !defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
#if MCM_DBG_ERR_LOG
                         dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nmcm:Channel %d RX queue is full so sending NAK to CP\n",
						chno);
//				printk("\nmcm:Channel %d RX queue is full so sending NAK to CP\n",
						chno);
#endif
#else
				hsi_channels[chno].pending_rx_size = data->size;
				hsi_channels[chno].rx_blocked = 1;
#endif
				break;
			} else {
				hsi_channels[chno].pending_rx_msgs++;
			}
		}

#if MCM_DBG_LOG
                 dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nmcm: Allocating read memory of size %d to channel %d \n",
					data->size, chno);
//              printk("\nmcm: Allocating read memory of size %d to channel %d \n",
//					data->size, chno);
#endif
		/* MODEM can't handle NAK so we allocate memory and
			drop the packet after recieving from MODEM */
#if 0
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
                   dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nmcm: channel not yet opened so not allocating memory\n");
//			printk("\nmcm: channel not yet opened so not allocating memory\n");
#endif
			data->buffer = NULL;
			break;
		}
		spin_unlock_bh(&hsi_channels[chno].lock);
#endif
		data->buffer = (char *)hsi_mem_alloc(data->size);

#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
		if(data->buffer == NULL) {
			hsi_channels[chno].pending_rx_size = data->size;
			PREPARE_WORK(&hsi_channels[chno].buf_retry_work,
						 hsi_buf_retry_work);
			queue_work(hsi_buf_retry_wq,
					   &hsi_channels[chno].buf_retry_work);
		}
#endif
		}
		break;

	case HSI_LL_EV_FREE_MEM: {
#if MCM_DBG_LOG
            dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: Freeing memory for channel %d, ptr = 0x%p \n",
					chno,data->buffer);
//         printk("\nmcm: Freeing memory for channel %d, ptr = 0x%p \n",
//					chno,data->buffer);
#endif
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
                   dynadbg_module(DYNADBG_ERR|DYNADBG_TX,"\nmcm: channel not yet opened so cant free mem\n");
//		      printk("\nmcm: channel not yet opened so cant free mem\n");
#endif
			break;
			}
		spin_unlock_bh(&hsi_channels[chno].lock);
		hsi_mem_free(data->buffer);
		}
		break;

	case HSI_LL_EV_RESET_MEM:
		/* if event is break, handle it somehow. */
		break;

	case HSI_LL_EV_WRITE_COMPLETE: {
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm:unlocking mutex for ch: %d\n",chno);
//		printk("\nmcm:unlocking mutex for ch: %d\n",chno);
#endif

		hsi_mem_free(data->buffer);
		hsi_channels[chno].write_happening = HSI_FALSE;
		wake_up(&hsi_channels[chno].write_wait);
		//hsi_mem_free(data->buffer);

#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_TX,"\nmcm: write complete cb, ch %d\n",chno);
//		printk("\nmcm: write complete cb, ch %d\n",chno);
#endif
		}
		break;

	case HSI_LL_EV_READ_COMPLETE: {
		int n = 0;
#if MCM_DBG_LOG
             dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nmcm: Read complete... size %d, channel %d, ptr = 0x%p \n",
					data->size, chno,data->buffer);
//		printk("\nmcm: Read complete... size %d, channel %d, ptr = 0x%p \n",
//					data->size, chno,data->buffer);
#endif
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			if(chno >= 13) {
				hsi_channels[chno].pending_rx_msgs--;
			}
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
                   dynadbg_module(DYNADBG_DEBUG|DYNADBG_RX,"\nmcm: channel %d not yet opened so dropping the packet\n",chno);
//			printk("\nmcm: channel %d not yet opened so dropping the packet\n",chno);
#endif
			hsi_mem_free(data->buffer);
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
			if(hsi_channels[chno].rx_blocked) {
				hsi_channels[chno].rx_blocked = 0;
				spin_lock_bh(&hsi_channels[chno].lock);
				hsi_channels[chno].pending_rx_msgs++;
				spin_unlock_bh(&hsi_channels[chno].lock);
				PREPARE_WORK(&hsi_channels[chno].buf_retry_work,
							  hsi_buf_retry_work);
				queue_work(hsi_buf_retry_wq,
						  &hsi_channels[chno].buf_retry_work);
			}
#endif
			break;
		}

		n = write_q(&hsi_channels[chno].rx_q, data->buffer, data->size, NULL);

		spin_unlock_bh(&hsi_channels[chno].lock);

		if (n == 0) {
#if MCM_DBG_ERR_LOG
                   dynadbg_module(DYNADBG_ERR|DYNADBG_RX,"\nmcm: Dropping the packet as channel %d is "
					"busy sending already read data\n",chno);
//			printk("\nmcm: Dropping the packet as channel %d is "
//					"busy sending already read data\n",chno);
#endif
			hsi_mem_free(data->buffer);
			/* Schedule work Q to send data to upper layers */
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		} else if (n == 1) {
			if (hsi_channels[chno].read_happening == HSI_FALSE) {
				hsi_channels[chno].read_happening = HSI_TRUE;
			}
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		}
		/* if n > 1, no need to schdule the wq again. */
		}
		break;
	default:
		/* Wrong event. */
#if MCM_DBG_ERR_LOG
            dynadbg_module(DYNADBG_ERR,"\nmcm:Wrong event.ch %d event %d", chno, event);
//         printk("\nmcm:Wrong event.ch %d event %d", chno, event);
#endif
		break;
	}
}

void xmd_ch_register_xmd_boot_cb(void (*fn)(void))
{
	xmd_boot_cb = fn;
}

void xmd_ch_init(void)	//void __init xmd_ch_init(void)
{
	int i;
	int size = ARRAY_SIZE(hsi_all_channels);

#if MCM_DBG_LOG
      dynadbg_module(DYNADBG_DEBUG,"\nmcm: xmd_ch_init++\n");
//	printk("\nmcm: xmd_ch_init++\n");
#endif

	for (i=0; i<size; i++) {
		hsi_channels[i].state = hsi_all_channels[i].state;
		hsi_channels[i].name = hsi_all_channels[i].name;
		hsi_channels[i].write_happening = HSI_FALSE;
		hsi_channels[i].write_timeout = XMD_TTY_WRITE_TIMEOUT_DEFAULT;  //xmd_tty_write timeout value(ms)
		hsi_channels[i].write_queued = HSI_FALSE;
		hsi_channels[i].read_queued = HSI_FALSE;
		hsi_channels[i].read_happening = HSI_FALSE;
		spin_lock_init(&hsi_channels[i].lock);
		init_waitqueue_head(&hsi_channels[i].write_wait);
		init_waitqueue_head(&hsi_channels[i].read_wait);
		init_q(i);
	}

	hsi_mem_init();
	hsi_ll_init(1, hsi_ch_cb);

	/* Create and initialize work q */
	hsi_read_wq = create_workqueue("hsi-read-wq");
	hsi_write_wq = create_workqueue("hsi-write-wq");
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
	hsi_buf_retry_wq = create_workqueue("hsi_buf_retry_wq");
#endif
	INIT_WORK(&XMD_DLP_RECOVERY_wq, xmd_dlp_recovery_wq);

	hsi_mcm_state = HSI_MCM_STATE_INITIALIZED;
}

void xmd_ch_exit(void)
{
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
	flush_workqueue(hsi_buf_retry_wq);
#endif
	flush_workqueue(hsi_read_wq);
	destroy_workqueue(hsi_read_wq);
	flush_workqueue(hsi_write_wq);
	destroy_workqueue(hsi_write_wq);
#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
	hsi_ll_exit();	//Add for Flashless HSIswitch
#else
	hsi_ll_shutdown();
#endif	//#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)

	hsi_mcm_state = HSI_MCM_STATE_UNDEF;
}

int xmd_ch_reset(void)
{
	int ch_i;
	int size = ARRAY_SIZE(hsi_all_channels);

	if(	hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
		return -1;
	}

	hsi_mcm_state = HSI_MCM_STATE_ERR_RECOVERY;

#if MCM_DBG_ERR_RECOVERY_LOG
      dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nmcm: HSI DLP Error Recovery initiated.\n");
//	printk("\nmcm: HSI DLP Error Recovery initiated.\n");
#endif

#if 0 /*TODO: Replace below API with the one specific to this project*/
	ifx_pmu_reset();
#endif

	for (ch_i=0; ch_i < size; ch_i++) {
		if (hsi_channels[ch_i].write_happening == HSI_TRUE) {
			hsi_channels[ch_i].write_happening = HSI_FALSE;
			wake_up(&hsi_channels[ch_i].write_wait);
		}
	}
	flush_workqueue(hsi_write_wq);
	hsi_ll_reset();
	flush_workqueue(hsi_read_wq);

	for (ch_i=0; ch_i < size; ch_i++) {
		init_q(ch_i);
	}

	/*TODO: Fine tune Modem Reset delay to required value.*/
	msleep(5000);

#if MCM_DBG_ERR_RECOVERY_LOG
      dynadbg_module(DYNADBG_NOTICE|DYNADBG_INIT_EXIT,"\nmcm: HSI DLP Error Recovery completed waiting for CP "
			"ready indication from RIL.\n");
//	printk("\nmcm: HSI DLP Error Recovery completed waiting for CP "
//			"ready indication from RIL.\n");
#endif
	/* Change MCM state to initilized when CP ready
		indication from tty ctrl channel is issued */

	return 0;
}

static void xmd_ch_reinit(void)
{
	if(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY)
		hsi_mcm_state = HSI_MCM_STATE_INITIALIZED;
}

void xmd_dlp_recovery(void)
{
	if(!is_dlp_reset_in_progress) {
		is_dlp_reset_in_progress = 1;
		schedule_work(&XMD_DLP_RECOVERY_wq);
	}
}

static void xmd_dlp_recovery_wq(struct work_struct *cp_crash_wq)
{
	xmd_ch_reset(); /* Start MCM/DLP cleanup */
	is_dlp_reset_in_progress = 0;
}
