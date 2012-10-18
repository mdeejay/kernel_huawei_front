/******************************************************************************
 *
 * TTY Driver for the MIPI HSI based IPC.
 *
 * Copyright (C) 2011 Intel Mobile Communications
 * Thippa Reddy <thippareddy.dammur@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA
 *
 *****************************************************************************/
/******************************************************************************
  Copyright (C), 1988-2012, Huawei Tech. Co., Ltd.
  File name:     xmd_boot_tty.h
  Author:       Version:        Date: 
  y00185015     1.0             20110822
  Description:  AP侧Flashless功能实现集合头文件，xmd-boot0驱动(tty类)头文件
  Others:
******************************************************************************/


#ifndef _XMD_BOOT_TTY_H
#define _XMD_BOOT_TTY_H

#include <linux/hsi_driver_if.h>    // /* Add begin for Flashless switch to Norm_HSI AT block */


#define XMM_HSI_MODE_FILE "/sys/devices/platform/xmm_boot/hsimode"


#define XMM_HSI_PSI_STATE   (1<<0)  //bit0,XMM_HSI_PSI_STATE, HSI PSI_STATE标志;
#define XMM_HSI_EBL_START   (1<<1)  //bit1,XMM_HSI_EBL_START, HSI EBL_START标志;
#define XMM_HSI_IMG_STATE   (1<<2)  //bit2,XMM_HSI_IMG_STATE, HSI IMG_STATE标志;
#define XMM_HSI_CODE_DOWN   (1<<3)  //bit3,XMM_HSI_CODE_DOWN, HSI CODE_DOWN标志;

#define XMM_HSI_3_WIRE_TYPE (1<<4)  //bit4,XMM_HSI_3_WIRE_TYPE, HSI线模式标志;
#define XMM_HSI_READY_LVL   (1<<5)  //bit5,XMM_HSI_READY_LVL, HSI READY线默认值;    //目前不支持,值同 XMM_HSI_3_WIRE_TYPE;
#define XMM_HSI_EXHSI_TYPE  (1<<6)  //bit6,XMM_HSI_EXHSI_TYPE, HSI ExHSI模式标志;
#define XMM_HSI_CAWAKE_LVL  (1<<7)  //bit7,XMM_HSI_CAWAKE_LVL, HSI CAWAKE线默认值;

#define XMM_HSI_DIV_96M     (0) //bit8~10,bit12~14,XMM_HSI_DIV, HSI(RX/TX)分频系数值;
#define XMM_HSI_DIV_48M     (1) //bit8~10,bit12~14,XMM_HSI_DIV, HSI(RX/TX)分频系数值;
#define XMM_HSI_DIV_24M     (3) //bit8~10,bit12~14,XMM_HSI_DIV, HSI(RX/TX)分频系数值;
#define XMM_MODE_GET_DIV_RX(mode)   ( (mode>>8 )&0x07 ) //bit8~10,XMM_HSI_DIV_RX, XMM_MODE_GET_DIV_RX(mode);
#define XMM_MODE_GET_DIV_TX(mode)   ( (mode>>12)&0x07 ) //bit12~14,XMM_HSI_DIV_TX, XMM_MODE_GET_DIV_TX(mode);
#define XMM_HSI_RX_CLK(div) (((div)&0x07)<<8)   //bit8~10,XMM_HSI_RX_CLK(div), HSI RX分频系数值;
#define XMM_HSI_RX_CLK_SET  (1<<11) //bit11,XMM_HSI_RX_CLK_SET, HSI RX分频系数设置标志位;   //值须同 XMM_HSI_TX_CLK_SET;
#define XMM_HSI_TX_CLK(div) (((div)&0x07)<<12)  //bit12~14,XMM_HSI_TX_CLK(div), HSI TX分频系数值;
#define XMM_HSI_TX_CLK_SET  (1<<15) //bit15,XMM_HSI_TX_CLK_SET, HSI TX分频系数设置标志位;   //值须同 XMM_HSI_RX_CLK_SET;

#define XMM_HSI_FLUSH_RX    (1<<16) //bit16,XMM_HSI_FLUSH_RX, HSI FLUSH_RX标志;
#define XMM_HSI_FLUSH_TX    (1<<17) //bit17,XMM_HSI_FLUSH_TX, HSI FLUSH_TX标志;
#define XMM_HSI_CLK_UNDIS   (1<<18) //bit18,XMM_HSI_CLK_UNDIS, HSI时钟取消关闭标志;
#define XMM_HSI_PM_UNDFT    (1<<19) //bit19,XMM_HSI_PM_UNDFT, HSI PM_unset_default标志;
#define XMM_HSI_DISINT_CAWAKE   (1<<20) //bit20,XMM_HSI_DISINT_CAWAKE, HSI CAWAKE中断禁止标志;
#define XMM_HSI_RD_MDELAY   (1<<21) //bit21,XMM_HSI_RD_MDELAY, HSI hsi_read后mdelay标志;    //##调试而设#
#define XMM_HSI_RDDN_LOG    (1<<22) //bit22,XMM_HSI_RDDN_LOG, HSI read_done LOG标志;    //##调试而设#
#define XMM_HSI_WRDN_LOG    (1<<23) //bit23,XMM_HSI_WRDN_LOG, HSI write_done LOG标志;   //##调试而设#

#define XMM_HSI_FLUSH_RXSTATE   (1<<24) //bit24,XMM_HSI_FLUSH_RXSTATE, HSI FLUSH_RXSTATE标志;
#define XMM_HSI_FLUSH_TXSTATE   (1<<25) //bit25,XMM_HSI_FLUSH_TXSTATE, HSI FLUSH_TXSTATE标志;    //##调试而设#
#define XMM_HSI_SOFT_RESET    (1<<26) //bit26,XMM_HSI_SOFT_RESET, HSI SOFT_RESET LOG标志;    //##调试而设#
#define XMM_HSI_OPEN_AGAIN    (1<<27) //bit27,XMM_HSI_OPEN_AGAIN, HSI hsi_open again标志;   //##调试而设#

#define XMM_HSI_HEAD_RD (1<<28) //bit28,XMM_HSI_HEAD_RD, HSI MIPI_Head读标志;
#define XMM_HSI_HEAD_WR (1<<29) //bit29,XMM_HSI_HEAD_WR, HSI MIPI_Head写标志;   //目前不支持，效果同'1';
#define XMM_HSI_USE_BOOT    (1<<30) //bit30,XMM_HSI_USE_BOOT, HSI使用模式标志;
#define XMM_HSI_Modem_Flashless (1<<31) //bit31,XMM_HSI_Modem_Flashless, Modem存储类型;

#define XMM_HSI_MODE_PSI_3_WIRE(rx_div,tx_div)  ( 0x00|(XMM_HSI_PSI_STATE)&(~XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)|(XMM_HSI_CODE_DOWN) | \
            (XMM_HSI_3_WIRE_TYPE)|(XMM_HSI_READY_LVL) & (~XMM_HSI_EXHSI_TYPE)&(~XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))|(XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))|(XMM_HSI_TX_CLK_SET) | \
            (XMM_HSI_FLUSH_RX)|(XMM_HSI_FLUSH_TX) | (XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)|(XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) | \
            (XMM_HSI_HEAD_RD)|(XMM_HSI_HEAD_WR) | (XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_PSI_3_WIRE_FLW(rx_div,tx_div)  ( 0x00|(XMM_HSI_PSI_STATE)&(~XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)|(XMM_HSI_CODE_DOWN) | \
            (XMM_HSI_3_WIRE_TYPE)|(XMM_HSI_READY_LVL) & (~XMM_HSI_EXHSI_TYPE)&(~XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))|(XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))|(XMM_HSI_TX_CLK_SET) | \
            (XMM_HSI_FLUSH_RX)|(XMM_HSI_FLUSH_TX) | (XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)|(XMM_HSI_DISINT_CAWAKE) | \
            (XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) | \
            (XMM_HSI_HEAD_RD)|(XMM_HSI_HEAD_WR) | (XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_PSI_3_WIRE_ACT(rx_div,tx_div)  ( 0x00|(XMM_HSI_PSI_STATE)&(~XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)|(XMM_HSI_CODE_DOWN) | \
            (XMM_HSI_3_WIRE_TYPE)|(XMM_HSI_READY_LVL) & (~XMM_HSI_EXHSI_TYPE)&(~XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))|(XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))|(XMM_HSI_TX_CLK_SET) | \
            (XMM_HSI_FLUSH_RX)|(XMM_HSI_FLUSH_TX) | (XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)&(~XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) | \
            (XMM_HSI_HEAD_RD)|(XMM_HSI_HEAD_WR) | (XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_PSI_EXHSI(rx_div,tx_div)   ( 0x00|(XMM_HSI_PSI_STATE)&(~XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)|(XMM_HSI_CODE_DOWN) & \
            (~XMM_HSI_3_WIRE_TYPE)&(~XMM_HSI_READY_LVL) | (XMM_HSI_EXHSI_TYPE)|(XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))|(XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))|(XMM_HSI_TX_CLK_SET) | \
            (XMM_HSI_FLUSH_RX)|(XMM_HSI_FLUSH_TX) & (~XMM_HSI_CLK_UNDIS)|(XMM_HSI_PM_UNDFT)&(~XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) | \
            (XMM_HSI_HEAD_RD)|(XMM_HSI_HEAD_WR) | (XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_EBL_EXHSI(rx_div,tx_div)   ( 0x00&(~XMM_HSI_PSI_STATE)|(XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)|(XMM_HSI_CODE_DOWN) & \
            (~XMM_HSI_3_WIRE_TYPE)&(~XMM_HSI_READY_LVL) | (XMM_HSI_EXHSI_TYPE)|(XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))|(XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))|(XMM_HSI_TX_CLK_SET) & \
            (~XMM_HSI_FLUSH_RX)&(~XMM_HSI_FLUSH_TX) & (~XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)&(~XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) | \
            (XMM_HSI_HEAD_RD)|(XMM_HSI_HEAD_WR) | (XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_EBL_4_WIRE(rx_div,tx_div)  ( 0x00&(~XMM_HSI_PSI_STATE)|(XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)|(XMM_HSI_CODE_DOWN) & \
            (~XMM_HSI_3_WIRE_TYPE)&(~XMM_HSI_READY_LVL) & (~XMM_HSI_EXHSI_TYPE)&(~XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))|(XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))|(XMM_HSI_TX_CLK_SET) & \
            (~XMM_HSI_FLUSH_RX)&(~XMM_HSI_FLUSH_TX) & (~XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)&(~XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) | \
            (XMM_HSI_HEAD_RD)|(XMM_HSI_HEAD_WR) | (XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_HSI_NORM(rx_div,tx_div)    ( 0x00&(~XMM_HSI_PSI_STATE)&(~XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)&(~XMM_HSI_CODE_DOWN) & \
            (~XMM_HSI_3_WIRE_TYPE)&(~XMM_HSI_READY_LVL) & (~XMM_HSI_EXHSI_TYPE)&(~XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))&(~XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))&(~XMM_HSI_TX_CLK_SET) & \
            (~XMM_HSI_FLUSH_RX)&(~XMM_HSI_FLUSH_TX) & (~XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)&(~XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) & \
            (~XMM_HSI_HEAD_RD)&(~XMM_HSI_HEAD_WR) & (~XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )
#define XMM_HSI_MODE_EXHSI_NORM(rx_div,tx_div)      ( 0x00&(~XMM_HSI_PSI_STATE)&(~XMM_HSI_EBL_START)&(~XMM_HSI_IMG_STATE)&(~XMM_HSI_CODE_DOWN) & \
            (~XMM_HSI_3_WIRE_TYPE)&(~XMM_HSI_READY_LVL) | (XMM_HSI_EXHSI_TYPE)|(XMM_HSI_CAWAKE_LVL) | \
            (XMM_HSI_RX_CLK(rx_div))&(~XMM_HSI_RX_CLK_SET) | (XMM_HSI_TX_CLK(tx_div))&(~XMM_HSI_TX_CLK_SET) & \
            (~XMM_HSI_FLUSH_RX)&(~XMM_HSI_FLUSH_TX) & (~XMM_HSI_CLK_UNDIS)&(~XMM_HSI_PM_UNDFT)&(~XMM_HSI_DISINT_CAWAKE) & \
            (~XMM_HSI_RD_MDELAY) & (~XMM_HSI_RDDN_LOG)&(~XMM_HSI_WRDN_LOG) & \
            (~XMM_HSI_HEAD_RD)&(~XMM_HSI_HEAD_WR) & (~XMM_HSI_USE_BOOT) | (XMM_HSI_Modem_Flashless) )

int xmd_set_hsimode(int hsimode);


#define XMD_BOOT_TTY_WRITE_TIMEOUT   msecs_to_jiffies(200)  //xmd_boot_tty_write wait_event timeout value, timeout for 200 ms

struct xmd_boot_if_struct {
	int wr_flag;
	unsigned int rx_size;
	unsigned int rx_state; /*0=>ACK, 1=>DATA*/ 
	void* rx_ptr;
	wait_queue_head_t  wr_ev;
	struct tty_driver *tty_drv;
	struct tty_struct *tty_ptr;
	struct hsi_device *hsi_dev;
	struct work_struct read_work;
	struct workqueue_struct *read_wq;

	struct hst_ctx    c_tx;	/* Add for Flashless */
	struct hsr_ctx    c_rx;	/* Add for Flashless */
};

#endif /* _XMD_BOOT_TTY_H */
