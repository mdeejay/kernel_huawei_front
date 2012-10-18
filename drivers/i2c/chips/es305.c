/* drivers/i2c/chips/es305.c - es305 voice processor driver
 *
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/es305.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include "../../../arch/arm/mach-omap2/mux.h"
#include <hsad/config_general_struct.h>
#include <hsad/config_interface.h>

#define ES305_CONFIG_NAME_LENGTH 10
#define ES305_UART1_WAKEUP       1
#define ES305_UART3_WAKEUP       3
#define ES305_UNKNOWN            -1

#define DEBUG            (0)
#define ENABLE_DIAG_IOCTLS      (0)
#define BAUDRATE 400


static struct i2c_client *this_client;
static struct es305_platform_data *pdata;

static int execute_cmdmsg(unsigned int);

static struct mutex es305_lock;
static int es305_opened;
static int es305_suspended = 0 ;
static unsigned int es305_NS_state = ES305_NS_STATE_AUTO;
static int es305_current_config = ES305_PATH_SUSPEND;
static int es305_param_ID;
static int gpio_es305_reset;
static int es305_clk_enable = 0;
static struct clk *es305_clk;
struct vp_ctxt {
    unsigned char *data;
    unsigned int img_size;
};
struct vp_ctxt the_vp;

static char config_name[ES305_CONFIG_NAME_LENGTH] = "unknown";
static int wakeup_id = ES305_UNKNOWN;

static bool get_audience_config_name(void)
{
    if(get_hw_config("audio/audience_type",config_name,ES305_CONFIG_NAME_LENGTH,NULL))
    {
        pr_info("%s:(config_name=%s).\n",__FUNCTION__,config_name);
        return true;
    }
    else
    {
        pr_err("%s:Get config name faild.\n",__FUNCTION__);
        return false;
    }
}

static ssize_t audience_type_read(struct file *filp,char __user *buffer,size_t count, loff_t *ppos)
{
    int len = 0;
    char idarray[ES305_CONFIG_NAME_LENGTH];
    char type_menu[ES305_CONFIG_NAME_LENGTH];

    memset(idarray, 0, ES305_CONFIG_NAME_LENGTH);
    memset(type_menu,0,ES305_CONFIG_NAME_LENGTH);
    if(!get_audience_config_name()){
                pr_err("Get audinece config name fail\n");
    }
    strcpy(type_menu,config_name);
    len = sprintf(idarray, "%s\n",type_menu);
    return simple_read_from_buffer(buffer, count, ppos,(void *) idarray, len);
}

static const struct file_operations audience_type_fops = {
    .read = audience_type_read,
};
static int  audience_debugfs(void)
{
    struct dentry *audience_debugfs_dir;
    struct dentry *audience_file;

    audience_debugfs_dir = debugfs_create_dir("audience", NULL);
    if(!audience_debugfs_dir)
        return -ENOENT;
    audience_file = debugfs_create_file("audience_type", 0444, audience_debugfs_dir, NULL, &audience_type_fops);
    return 0;
}

static void  es305_clock_open(void)
{
    /*TURN ON AUXCLK4 OUTPUT 26Mhz*/
        es305_clk = clk_get(NULL, "auxclk4_ck");
        clk_set_rate(es305_clk, 26000000);
        clk_enable(es305_clk);
        es305_clk_enable = 1;
    pr_info("%s :es305_clk_enable =%d\n", __func__,es305_clk_enable);
}
static void  es305_clock_off(void)
{
         clk_disable(es305_clk);
    es305_clk_enable = 0;
    pr_info("%s :es305_clk_enable =%d\n", __func__,es305_clk_enable);
}
static int es305_i2c_read(char *rxData, int length)
{
    int rc;
    struct i2c_msg msgs[] = {
        {
            .addr = this_client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxData,
        },
    };

    rc = i2c_transfer(this_client->adapter, msgs, 1);
    if (rc < 0) {
        pr_err("%s: transfer error %d\n", __func__, rc);
        return rc;
    }

#if DEBUG
    {
        int i = 0;
        for (i = 0; i < length; i++)
            pr_info("%s: rx[%d] = %2x\n", __func__, i, rxData[i]);
    }
#endif

    return 0;
}

static int es305_i2c_write(char *txData, int length)
{
    int rc;
    struct i2c_msg msg[] = {
        {
            .addr = this_client->addr,
            .flags = 0,
            .len = length,
            .buf = txData,
        },
    };

    rc = i2c_transfer(this_client->adapter, msg, 1);
    if (rc < 0) {
        pr_err("%s: transfer error %d\n", __func__, rc);
        return rc;
    }

#if DEBUG
    {
        int i = 0;
        for (i = 0; i < length; i++)
            pr_info("%s: tx[%d] = %2x\n", __func__, i, txData[i]);
    }
#endif

    return 0;
}

static int es305_open(struct inode *inode, struct file *file)
{
    int rc = 0;
    struct vp_ctxt *vp = &the_vp;

    mutex_lock(&es305_lock);

    if (es305_opened) {
        pr_err("%s: busy\n", __func__);
        rc = -EBUSY;
        goto done;
    }

    file->private_data = vp;
    vp->img_size = 0;
    es305_opened = 1;
done:
    mutex_unlock(&es305_lock);
    return rc;
}

static int es305_release(struct inode *inode, struct file *file)
{
    mutex_lock(&es305_lock);
    es305_opened = 0;
    mutex_unlock(&es305_lock);

    return 0;
}

static void es305_i2c_sw_reset(unsigned int reset_cmd)
{
    int rc = 0;
    unsigned char msgbuf[4];

    msgbuf[0] = (reset_cmd >> 24) & 0xFF;
    msgbuf[1] = (reset_cmd >> 16) & 0xFF;
    msgbuf[2] = (reset_cmd >> 8) & 0xFF;
    msgbuf[3] = reset_cmd & 0xFF;

    pr_info("%s: %08x\n", __func__, reset_cmd);

    rc = es305_i2c_write(msgbuf, 4);
    if (!rc)
        msleep(20);
}

static ssize_t es305_bootup_init(struct file *file, struct es305img *img)
{
    struct vp_ctxt *vp = file->private_data;
    int rc, pass = 0;
    int remaining;
    int retry = RETRY_CNT;
    unsigned char *index;
    char buf[2];

    if (img->img_size > ES305_MAX_FW_SIZE) {
        pr_err("%s: invalid es305 image size %d\n", __func__,
            img->img_size);
        return -EINVAL;
    }

    vp->data = kmalloc(img->img_size, GFP_KERNEL);
    if (!vp->data) {
        pr_err("%s: out of memory\n", __func__);
        return -ENOMEM;
    }
    vp->img_size = img->img_size;
    if (copy_from_user(vp->data, img->buf, img->img_size)) {
        pr_err("%s: copy from user failed\n", __func__);
        kfree(vp->data);
        return -EFAULT;
    }

    while (retry--) {
        if(!es305_clk_enable)
            es305_clock_open();
        gpio_direction_output(gpio_es305_reset , 0);
                mdelay(1);
        /* Take out of reset */
        gpio_direction_output(gpio_es305_reset , 1);

        msleep(50); /* Delay before send I2C command */
        /* Boot Cmd to ES305 */
        buf[0] = ES305_msg_BOOT >> 8;
        buf[1] = ES305_msg_BOOT & 0xff;

        rc = es305_i2c_write(buf, 2);
        if (rc < 0) {
            pr_err("%s: set boot mode error (%d retries left)\n",
                __func__, retry);
            continue;
        }

        mdelay(1); /* use polling */
        rc = es305_i2c_read(buf, 2);
        if (rc < 0) {
            pr_err("%s: boot mode ack error (%d retries left)\n",
                __func__, retry);
            continue;
        }

        if (buf[0] != ES305_msg_BOOT_ACK) {
            pr_err("%s: not a boot-mode ack (%d retries left)\n",
                __func__, retry);
            continue;
        }
        if (buf[1] != ES305_msg_BOOT_ACK) {
            pr_err("%s: not a boot-mode ack  2222  (%d retries left)\n",
                __func__, retry);
            continue;
        }

        remaining = vp->img_size / BAUDRATE;
        index = vp->data;

        for (; remaining; remaining--, index += BAUDRATE) {
            rc = es305_i2c_write(index, BAUDRATE);
            if (rc < 0)
                break;
        }

        if (rc >= 0 && vp->img_size % BAUDRATE)
            rc = es305_i2c_write(index, vp->img_size % BAUDRATE);

        if (rc < 0) {
            pr_err("%s: fw load error %d (%d retries left) remaining %d\n",
                __func__, rc, retry, remaining);
            continue;
        }
        pr_info("%s: firmware loaded successfully\n", __func__);
        pass = 1;
        break;
    }

    msleep(100);
    rc = execute_cmdmsg(A100_msg_Sync);
    if (rc < 0) {
        pr_err("%s: sync command error %d (%d retries left)\n",
                __func__, rc, retry);
    }
    /* Put ES305 into sleep mode */
    rc = execute_cmdmsg(A100_msg_Sleep);
    if(rc<0){
        pr_err("%s:suspend error! \n",__func__);
        goto set_suspend_err;
    }

    es305_suspended = 1;
    es305_current_config = ES305_PATH_SUSPEND;
    /* Disable ES305 clock */
    msleep(20);
    if(es305_clk_enable)
        es305_clock_off();
set_suspend_err:
    if (pass && !rc)
        pr_info("%s: initialized!\n", __func__);
    else
        pr_err("%s: initialization failed\n", __func__);

    kfree(vp->data);
    return rc;
}

unsigned char phonecall_receiver[] = {

    0x80,0x31,0x00,0x00,
};

unsigned char phonecall_headset[] = {

    0x80,0x31,0x00,0x01,
};

unsigned char phonecall_headphone[] = {

	0x80,0x31,0x00,0x02,
};

unsigned char phonecall_speaker[] = {

    0x80,0x31, 0x00,0x03,
};

unsigned char phonecall_bt[] = {
    0x80,0x31, 0x00,0x04,

};

unsigned char phonecall_receiver_wb[] = {
    /*BT call,audience is in passthrough mode.there are two passthrough modes
        by hardware different,in FRONTT1P5 connect the Data lines only.*/
    0x80,0x31,0x00,0x06,
};

unsigned char phonecall_headset_wb[] = {
   0x80,0x31,0x00,0x07,
};

unsigned char phonecall_headphone_wb[] = {
   0x80,0x31,0x00,0x08,
};

unsigned char phonecall_speaker_wb[] = {
   0x80,0x31,0x00,0x09,
};

unsigned char phonecall_bt_wb[] = {
   0x80,0x31,0x00,0x0a,
};

unsigned char INT_MIC_recording_receiver[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char EXT_MIC_recording[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char INT_MIC_recording_speaker[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char BACK_MIC_recording[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_no_ns_receiver[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_no_ns_headset[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_no_ns_speaker[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_no_ns_bt[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_ns_receiver[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_ns_headset[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_ns_speaker[] = {

    0x80,0x52,0x00,0xE2,
};

unsigned char vr_ns_bt[] = {

    0x80,0x52,0x00,0xE2,
};
unsigned char passthrough_mode[] = {

    0x80,0x52,0x00,0xE2,
};
unsigned char suspend_mode[] = {
    0x80,0x10,0x00,0x01,
};
static void board_type_check(void)
{
       char wakeup_type[3][10] = {"unknown","i2c","uart"};

        if(!get_audience_config_name()){
                pr_err("Get audinece config name fail\n");
        }

       if(strncmp(wakeup_type[2], config_name, 4) == 0){
            wakeup_id =  ES305_UART1_WAKEUP;
        }
       else if(strncmp(wakeup_type[1], config_name, 3) == 0){
            wakeup_id =  ES305_UART3_WAKEUP;
        }
       else
            wakeup_id =  ES305_UNKNOWN;
}
static ssize_t chk_wakeup_es305(void)
{
    int rc = 0, retry = 3;
    if (es305_suspended == 1) {
        if(!es305_clk_enable)
              es305_clock_open();
        switch(wakeup_id){
            case ES305_UART3_WAKEUP:
                gpio_request(pdata->gpio_es305_wakeup_uart3,"gpio_17");
                omap_mux_init_signal("dpm_emu6.gpio_17", OMAP_MUX_MODE3);
                gpio_direction_output(pdata->gpio_es305_wakeup_uart3 , 1);
                mdelay(1);
                gpio_direction_output(pdata->gpio_es305_wakeup_uart3, 0);
                gpio_free(pdata->gpio_es305_wakeup_uart3);
                break;
            case ES305_UART1_WAKEUP:
                gpio_request(pdata->gpio_es305_wakeup_uart1,"gpio_141");
                omap_mux_init_signal("uart3_cts_rctx.gpio_141", OMAP_MUX_MODE3);
                gpio_direction_output(pdata->gpio_es305_wakeup_uart1 , 1);
                mdelay(1);
                gpio_direction_output(pdata->gpio_es305_wakeup_uart1, 0);
                gpio_free(pdata->gpio_es305_wakeup_uart1);
                break;
            default:
                 return ES305_UNKNOWN;
                 break;
                }
        msleep(30); // 120

        do {
            rc = execute_cmdmsg(A100_msg_Sync);
        } while ((rc < 0) && --retry);
        if (rc < 0) {
            pr_err("%s: failed (%d)\n", __func__, rc);
            goto wakeup_sync_err;
        }

        es305_suspended = 0;
    }
wakeup_sync_err:
    switch(wakeup_id){
        case ES305_UART3_WAKEUP:
            omap_mux_init_signal("dpm_emu6.uart3_tx_irtx", OMAP_MUX_MODE2);
            break;
        case ES305_UART1_WAKEUP:
            omap_mux_init_signal("uart3_cts_rctx.uart1_tx", OMAP_MUX_MODE1);
            break;
        default:
            return ES305_UNKNOWN;
            break;
    }
    return rc;
}

/* Filter commands according to noise suppression state forced by
 * ES305_SET_NS_STATE ioctl.
 *
 * For this function to operate properly, all configurations must include
 * both A100_msg_Bypass and Mic_Config commands even if default values
 * are selected or if Mic_Config is useless because VP is off
 */
int es305_filter_vp_cmd(int cmd, int mode)
{
    int msg = (cmd >> 16) & 0xFFFF;
    int filtered_cmd = cmd;

    if (es305_NS_state == ES305_NS_STATE_AUTO)
        return cmd;

    switch (msg) {
    case A100_msg_Bypass:
        if (es305_NS_state == ES305_NS_STATE_OFF)
            filtered_cmd = ES305_msg_VP_OFF;
        else
            filtered_cmd = ES305_msg_VP_ON;
        break;
    case A100_msg_SetAlgorithmParmID:
        es305_param_ID = cmd & 0xFFFF;
        break;
    case A100_msg_SetAlgorithmParm:
        if (es305_param_ID == Mic_Config) {
            if (es305_NS_state == ES305_NS_STATE_CT)
                filtered_cmd = (msg << 16);
            else if (es305_NS_state == ES305_NS_STATE_FT)
                filtered_cmd = (msg << 16) | 0x0002;
        }
        break;
    default:
        if (mode == ES305_CONFIG_VP)
            filtered_cmd = -1;
        break;
    }

    pr_info("%s: %x filtered = %x, es305_NS_state %d, mode %d\n", __func__,
            cmd, filtered_cmd, es305_NS_state, mode);

    return filtered_cmd;
}

int es305_set_config(char newid, int mode)
{
    int rc = 0, size = 0;
    int number_of_cmd_sets, rd_retry_cnt;
    unsigned int sw_reset = 0;
    unsigned char *i2c_cmds;
    unsigned char *index = 0;
    unsigned char ack_buf[ES305_CMD_FIFO_DEPTH * 4];
    unsigned char rdbuf[4];

    if ((es305_suspended) && (newid == ES305_PATH_SUSPEND))
    {
        return rc;
    }


    rc = chk_wakeup_es305();
    if (rc < 0)
        return rc;

    sw_reset = ((A100_msg_Reset << 16) | RESET_IMMEDIATE);

    switch (newid) {
    case ES305_PATH_PASSTHROUGH:
        i2c_cmds = (unsigned char *)passthrough_mode;
        size = sizeof(passthrough_mode);
        break;
    case ES305_PATH_INCALL_RECEIVER:
        i2c_cmds = phonecall_receiver;
        size = sizeof(phonecall_receiver);
        break;
    case ES305_PATH_INCALL_HEADSET:
        i2c_cmds = phonecall_headset;
        size = sizeof(phonecall_headset);
        break;
	case ES305_PATH_INCALL_HEADPHONE:
        i2c_cmds = phonecall_headphone;
        size = sizeof(phonecall_headphone);
        break;
    case ES305_PATH_INCALL_SPEAKER:
        i2c_cmds = phonecall_speaker;
        size = sizeof(phonecall_speaker);
        break;
    case ES305_PATH_INCALL_BT:
        i2c_cmds = phonecall_bt;
        size = sizeof(phonecall_bt);
        break;
    case ES305_PATH_VR_NO_NS_RECEIVER:
        i2c_cmds = vr_no_ns_receiver;
        size = sizeof(vr_no_ns_receiver);
        break;
    case ES305_PATH_VR_NO_NS_HEADSET:
        i2c_cmds = vr_no_ns_headset;
        size = sizeof(vr_no_ns_headset);
        break;
    case ES305_PATH_VR_NO_NS_SPEAKER:
        i2c_cmds = vr_no_ns_speaker;
        size = sizeof(vr_no_ns_speaker);
        break;
    case ES305_PATH_VR_NO_NS_BT:
        i2c_cmds = vr_no_ns_bt;
        size = sizeof(vr_no_ns_bt);
        break;
    case ES305_PATH_VR_NS_RECEIVER:
        i2c_cmds = vr_ns_receiver;
        size = sizeof(vr_ns_receiver);
        break;
    case ES305_PATH_VR_NS_HEADSET:
        i2c_cmds = vr_ns_headset;
        size = sizeof(vr_ns_headset);
        break;
    case ES305_PATH_VR_NS_SPEAKER:
        i2c_cmds = vr_ns_speaker;
        size = sizeof(vr_ns_speaker);
        break;
    case ES305_PATH_VR_NS_BT:
        i2c_cmds = vr_ns_bt;
        size = sizeof(vr_ns_bt);
        break;
    case ES305_PATH_RECORD_RECEIVER:
        i2c_cmds = INT_MIC_recording_receiver;
        size = sizeof(INT_MIC_recording_receiver);
        break;
    case ES305_PATH_RECORD_HEADSET:
        i2c_cmds = EXT_MIC_recording;
        size = sizeof(EXT_MIC_recording);
        break;
    case ES305_PATH_RECORD_SPEAKER:
        i2c_cmds = INT_MIC_recording_speaker;
        size = sizeof(INT_MIC_recording_speaker);
        break;
    case ES305_PATH_RECORD_BT:
        i2c_cmds = phonecall_bt;
        size = sizeof(phonecall_bt);
        break;
    case ES305_PATH_SUSPEND:
        i2c_cmds = (unsigned char *)suspend_mode;
        size = sizeof(suspend_mode);
        break;
    case ES305_PATH_CAMCORDER:
        i2c_cmds = BACK_MIC_recording;
        size = sizeof(BACK_MIC_recording);
        break;
   case ES305_PATH_INCALL_RECEIVER_WB:
        i2c_cmds = phonecall_receiver_wb;
        size = sizeof(phonecall_receiver_wb);
        break;
    case ES305_PATH_INCALL_HEADSET_WB:
          i2c_cmds = phonecall_headset_wb;
	   size = sizeof(phonecall_headset_wb);
       break;
    case ES305_PATH_INCALL_HEADPHONE_WB:
          i2c_cmds = phonecall_headphone_wb;
          size = sizeof(phonecall_headphone_wb);
       break;
    case ES305_PATH_INCALL_SPEAKER_WB:
           i2c_cmds = phonecall_speaker_wb;
	   size = sizeof(phonecall_speaker_wb);
       break;
    case ES305_PATH_INCALL_BT_WB:
          i2c_cmds = phonecall_bt_wb;
	   size = sizeof(phonecall_bt_wb);
       break;
    default:
        pr_err("%s: invalid cmd %d\n", __func__, newid);
        rc = -1;
        goto input_err;
        break;
    }

    es305_current_config = newid;
    pr_info("%s: change to mode %d\n", __func__, newid);

    pr_info("%s: block write start (size = %d)\n", __func__, size);
#if DEBUG
        int i = 0;
        for (i = 1; i <= size; i++) {
                pr_info("%x ", *(i2c_cmds + i - 1));
                if ( !(i % 4))
                        pr_info("\n");
        }
#endif

    rc = es305_i2c_write(i2c_cmds, size);
    if (rc < 0) {
        pr_err("ES305 CMD block write error!\n");
        es305_i2c_sw_reset(sw_reset);
        return rc;
    }
    pr_info("%s: block write end\n", __func__);

    /* Don't need to get Ack after sending out a suspend command */
    if (*i2c_cmds == 0x80 && *(i2c_cmds + 1) == 0x10
        && *(i2c_cmds + 2) == 0x00 && *(i2c_cmds + 3) == 0x01) {
        es305_suspended = 1;
        /* Disable ES305 clock */
        msleep(120);
        if(es305_clk_enable)
            es305_clock_off();
        return rc;
    }

    memset(ack_buf, 0, sizeof(ack_buf));
    msleep(20);
    pr_info("%s: CMD ACK block read start\n", __func__);
    rc = es305_i2c_read(ack_buf, size);
    if (rc < 0) {
        pr_err("%s: CMD ACK block read error\n", __func__);
        es305_i2c_sw_reset(sw_reset);
        return rc;
    } else {
        pr_info("%s: CMD ACK block read end\n", __func__);
#if DEBUG
        for (i = 1; i <= size; i++) {
            pr_info("%x ", ack_buf[i-1]);
            if ( !(i % 4))
                pr_info("\n");
        }
#endif
        index = ack_buf;
        number_of_cmd_sets = size / 4;
        do {
            if (*index == 0x00) {
                rd_retry_cnt = POLLING_RETRY_CNT;
rd_retry:
                if (rd_retry_cnt--) {
                    memset(rdbuf, 0, sizeof(rdbuf));
                    rc = es305_i2c_read(rdbuf, 4);
                    if (rc < 0)
                        return rc;
#if DEBUG
                    for (i = 0; i < sizeof(rdbuf); i++) {
                        pr_info("0x%x\n", rdbuf[i]);
                    }
                    pr_info("-----------------\n");
#endif
                    if (rdbuf[0] == 0x00) {
                        msleep(20);
                        goto rd_retry;
                    }
                } else {
                    pr_err("%s: CMD ACK Not Ready\n",
                        __func__);
                    return -EBUSY;
                }
            } else if (*index == 0xff) { /* illegal cmd */
                return -ENOEXEC;
            } else if (*index == 0x80) {
                index += 4;
            }
        } while (--number_of_cmd_sets);
    }
input_err:
    return rc;
}

int execute_cmdmsg(unsigned int msg)
{
    int rc = 0;
    int retries, pass = 0;
    unsigned char msgbuf[4];
    unsigned char chkbuf[4];
    unsigned int sw_reset = 0;

    sw_reset = ((A100_msg_Reset << 16) | RESET_IMMEDIATE);

    msgbuf[0] = (msg >> 24) & 0xFF;
    msgbuf[1] = (msg >> 16) & 0xFF;
    msgbuf[2] = (msg >> 8) & 0xFF;
    msgbuf[3] = msg & 0xFF;

    memcpy(chkbuf, msgbuf, 4);

    rc = es305_i2c_write(msgbuf, 4);
    if (rc < 0) {
        pr_err("%s: error %d\n", __func__, rc);
        es305_i2c_sw_reset(sw_reset);
        return rc;
    }

    /* We don't need to get Ack after sending out a suspend command */
    if (msg == A100_msg_Sleep)
        return rc;

    retries = POLLING_RETRY_CNT;
    while (retries--) {
        rc = 0;
        memset(msgbuf, 0, sizeof(msgbuf));
        msleep(20); /* use polling */
        rc = es305_i2c_read(msgbuf, 4);
        if (rc < 0) {
            pr_err("%s: ack-read error %d (%d retries)\n", __func__,
                rc, retries);
            continue;
        }
        if (msgbuf[0] == 0x80  && msgbuf[1] == chkbuf[1]) {
            pass = 1;
            break;
        } else if (msgbuf[0] == 0xff && msgbuf[1] == 0xff) {
            pr_err("%s: illegal cmd %08x\n", __func__, msg);
            rc = -EINVAL;
            break;
        } else if ( msgbuf[0] == 0x00 && msgbuf[1] == 0x00 ) {
            pr_info("%s: not ready (%d retries)\n", __func__,
                retries);
            rc = -EBUSY;
        } else {
            pr_info("%s: cmd/ack mismatch: (%d retries left)\n",
                __func__,
                retries);
#if DEBUG
            pr_info("%s: msgbuf[0] = %x\n", __func__, msgbuf[0]);
            pr_info("%s: msgbuf[1] = %x\n", __func__, msgbuf[1]);
            pr_info("%s: msgbuf[2] = %x\n", __func__, msgbuf[2]);
            pr_info("%s: msgbuf[3] = %x\n", __func__, msgbuf[3]);
#endif
            rc = -EBUSY;
        }
    }

    if (!pass) {
        pr_err("%s: failed execute cmd %08x (%d)\n", __func__,
            msg, rc);
        es305_i2c_sw_reset(sw_reset);
    }
    return rc;
}

#if ENABLE_DIAG_IOCTLS
static int es305_set_mic_state(char miccase)
{
    int rc = 0;
    unsigned int cmd_msg = 0;

    switch (miccase) {
    case 1: /* Mic-1 ON / Mic-2 OFF */
        cmd_msg = 0x80260007;
        break;
    case 2: /* Mic-1 OFF / Mic-2 ON */
        cmd_msg = 0x80260015;
        break;
    case 3: /* both ON */
        cmd_msg = 0x80260001;
        break;
    case 4: /* both OFF */
        cmd_msg = 0x80260006;
        break;
    default:
        pr_info("%s: invalid input %d\n", __func__, miccase);
        rc = -EINVAL;
        break;
    }
    rc = execute_cmdmsg(cmd_msg);
    return rc;
}

static int exe_cmd_in_file(unsigned char *incmd)
{
    int rc = 0;
    int i = 0;
    unsigned int cmd_msg = 0;
    unsigned char tmp = 0;

    for (i = 0; i < 4; i++) {
        tmp = *(incmd + i);
        cmd_msg |= (unsigned int)tmp;
        if (i != 3)
            cmd_msg = cmd_msg << 8;
    }
    rc = execute_cmdmsg(cmd_msg);
    if (rc < 0)
        pr_err("%s: cmd %08x error %d\n", __func__, cmd_msg, rc);
    return rc;
}
#endif /* ENABLE_DIAG_IOCTLS */
static long es305_ioctl(struct file *file, unsigned int cmd, unsigned long arg)

{
    void __user *argp = (void __user *)arg;
    struct es305img img;
    int rc = 0;
#if ENABLE_DIAG_IOCTLS
    int mic_cases = 0;
    int mic_sel = 0;
#endif
    int pathid = 0;
    unsigned int ns_state;
    char msg[4];

    switch (cmd) {
    case ES305_SYNC_CMD:
        msleep(100);
        rc = execute_cmdmsg(A100_msg_Sync);
        if (rc < 0) {
            pr_err("%s: sync command error %d \n",
                        __func__, rc);
            return rc;
        }
        break;

    case ES305_RESET_CMD:
                  if(!es305_clk_enable)
              es305_clock_open();
        gpio_direction_output(gpio_es305_reset , 0);
                     mdelay(1);
        gpio_direction_output(gpio_es305_reset , 1);
        msleep(50);
        break;

    case ES305_BOOTUP_INIT:
        img.buf = 0;
        img.img_size = 0;
        if (copy_from_user(&img, argp, sizeof(img)))
            return -EFAULT;
        rc = es305_bootup_init(file, &img);
        break;
    case ES305_WRITE_MSG:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
           if (copy_from_user(msg, argp, sizeof(msg)))
            return -EFAULT;
         rc = es305_i2c_write(msg,4);
         if(rc < 0)
             pr_err("%s: ES305_WRITE_MSG (%s) error %d!\n",
                __func__, msg, rc);
         break;
    case ES305_READ_DATA:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
        rc = es305_i2c_read(msg, 4);
        if (copy_to_user(argp, &msg, 4))
            return -EFAULT;
        break;

    case ES305_SET_CONFIG:
        if (copy_from_user(&pathid, argp, sizeof(pathid)))
            return -EFAULT;
        rc = es305_set_config(pathid, ES305_CONFIG_FULL);
        if (rc < 0)
            pr_err("%s: ES305_SET_CONFIG (%d) error %d!\n",
                __func__, pathid, rc);
        break;


    case ES305_SET_NS_STATE:
        if (copy_from_user(&ns_state, argp, sizeof(ns_state)))
            return -EFAULT;
        pr_info("%s: set noise suppression %d\n", __func__, ns_state);
        if (ns_state < 0 || ns_state >= ES305_NS_NUM_STATES)
            return -EINVAL;
        es305_NS_state = ns_state;
        if (!es305_suspended)
            es305_set_config(es305_current_config,ES305_CONFIG_VP);
        break;

#if ENABLE_DIAG_IOCTLS
    case ES305_SET_MIC_ONOFF:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
        if (copy_from_user(&mic_cases, argp, sizeof(mic_cases)))
            return -EFAULT;
        rc = es305_set_mic_state(mic_cases);
        if (rc < 0)
            pr_err("%s: ES305_SET_MIC_ONOFF %d error %d!\n",
                __func__, mic_cases, rc);
        break;
    /*
    case ES305_SET_MICSEL_ONOFF:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
        if (copy_from_user(&mic_sel, argp, sizeof(mic_sel)))
            return -EFAULT;
        gpio_set_value(pdata->gpio_es305_micsel, !!mic_sel);
        rc = 0;
        break;
    */
    case ES305_READ_DATA:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
        rc = es305_i2c_read(msg, 4);
        if (copy_to_user(argp, &msg, 4))
            return -EFAULT;
        break;
    case ES305_SYNC_CMD:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
        msg[0] = 0x80;
        msg[1] = 0x00;
        msg[2] = 0x00;
        msg[3] = 0x00;
        rc = es305_i2c_write(msg, 4);
        break;
    case ES305_SET_CMD_FILE:
        rc = chk_wakeup_es305();
        if (rc < 0)
            return rc;
        if (copy_from_user(msg, argp, sizeof(msg)))
            return -EFAULT;
        rc = exe_cmd_in_file(msg);
        break;
#endif /* ENABLE_DIAG_IOCTLS */
    default:
        pr_err("%s: invalid command %d\n", __func__, _IOC_NR(cmd));
        rc = -EINVAL;
        break;
    }

    return rc;
}

static const struct file_operations es305_fops = {
    .owner = THIS_MODULE,
    .open = es305_open,
    .release = es305_release,
    .unlocked_ioctl = es305_ioctl,
};

static struct miscdevice es305_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "audience_es305",
    .fops = &es305_fops,
};

static int es305_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;

    pdata = client->dev.platform_data;

    if (pdata == NULL) {
        pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
        if (pdata == NULL) {
            rc = -ENOMEM;
            pr_err("%s: platform data is NULL\n", __func__);
            goto err_alloc_data_failed;
        }
    }

    this_client = client;
    if(!es305_clk_enable)
        es305_clock_open();
    gpio_es305_reset = get_gpio_num_by_name("GPIO_ES305_RST");
        if(gpio_es305_reset < 0)
        {
            rc = -EINVAL;
            pr_err("%s: get GPIO_ES305_RST number failed\n",__func__);
            goto err_alloc_data_failed;
        }

    rc = gpio_request(gpio_es305_reset, "es305");
    if (rc < 0) {
        pr_err("%s: gpio request reset pin failed\n", __func__);
        goto err_free_gpio_all;
    }

    rc = gpio_direction_output(gpio_es305_reset, 1);
    if (rc < 0) {
        pr_err("%s: request reset gpio direction failed\n", __func__);
        goto err_free_gpio_all;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s: i2c check functionality error\n", __func__);
        rc = -ENODEV;
        goto err_free_gpio_all;
    }

    rc = misc_register(&es305_device);
    if (rc) {
        pr_err("%s: es305_device register failed\n", __func__);
        goto err_free_gpio_all;
    }
         if(es305_clk_enable)
            es305_clock_off();
    return 0;

err_free_gpio_all:
    gpio_free(gpio_es305_reset);
err_alloc_data_failed:
         if(es305_clk_enable)
            es305_clock_off();
    return rc;
}

static int es305_remove(struct i2c_client *client)
{
    struct es305_platform_data *p305data = i2c_get_clientdata(client);
    kfree(p305data);
    return 0;
}

static int es305_suspend(struct i2c_client *client, pm_message_t mesg)
{
    return 0;
}

static int es305_resume(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id es305_id[] = {
    { "audience_es305", 0 },
    { }
};

static struct i2c_driver es305_driver = {
    .probe = es305_probe,
    .remove = es305_remove,
    .suspend = es305_suspend,
    .resume    = es305_resume,
    .id_table = es305_id,
    .driver = {
        .name = "audience_es305",
    },
};

static int __init es305_init(void)
{
    pr_info("%s\n", __func__);
    board_type_check();
    audience_debugfs();
    mutex_init(&es305_lock);

    return i2c_add_driver(&es305_driver);
}

static void __exit es305_exit(void)
{
    i2c_del_driver(&es305_driver);
}

module_init(es305_init);
module_exit(es305_exit);

MODULE_DESCRIPTION("ES305 voice processor driver");
MODULE_LICENSE("GPL");
