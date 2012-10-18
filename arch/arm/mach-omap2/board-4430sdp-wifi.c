/*
 * Board support file for containing WiFi specific details for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Pradeep Gurumath <pradeepgurumath@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* linux/arch/arm/mach-omap2/board-4430sdp-wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include<linux/random.h>
#include <linux/skbuff.h>
#include <generated/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>
#include <asm/mach-types.h>

#include "board-4430sdp-wifi.h"
#include "mux.h"

#ifndef __NVE_INTERFACE_H__
#include "../../../include/linux/nve_interface.h"
#endif
static int sdp4430_wifi_cd;             /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS        4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS         160
#define PREALLOC_WLAN_SECTION_HEADER            24

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM        16

#define WLAN_MAC_LEN            6
#define NV_WLAN_NUM             193
#define NV_WLAN_VALID_SIZE      12
#define MAC_ADDRESS_FILE  "/data/misc/wifi/macaddr"
unsigned char g_wifimac[WLAN_MAC_LEN] = {0};

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
        void *mem_ptr;
        unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
        { NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
        { NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
        { NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
        { NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

/*kmalloc memory for wifi*/
void *wifi_mem_prealloc(int section, unsigned long size)
{
        if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
                return wlan_static_skb;
        if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
                return NULL;
        if (wifi_mem_array[section].size < size)
                return NULL;
        return wifi_mem_array[section].mem_ptr;
}

#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wifi_mem_prealloc);
#endif



/*init wifi buf*/
int init_wifi_mem(void)
{
        int i;

        for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
                if (i < (WLAN_SKB_BUF_NUM/2))
                        wlan_static_skb[i] = dev_alloc_skb(4096);
                else
                        wlan_static_skb[i] = dev_alloc_skb(8192);
        }
        for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
                wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
                                                        GFP_KERNEL);
                if (wifi_mem_array[i].mem_ptr == NULL)
                        return -ENOMEM;
        }
        return 0;
}

void config_wlan_mux(void)
{
        omap_mux_init_signal("kpd_col1.gpio_0", OMAP_PIN_INPUT |
                                OMAP_PIN_OFF_WAKEUPENABLE);
        omap_mux_init_signal("gpmc_nwp.gpio_54", OMAP_PIN_OUTPUT);

        omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
                                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLUP);
        omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
                                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLUP);
        omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
                                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLUP);
        omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
                                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLUP);
        omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
                                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLUP);
        omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
                                OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLUP);
}

int omap_wifi_status_register(void (*callback)(int card_present,
                                                void *dev_id), void *dev_id)
{
        if (wifi_status_cb)
                return -EAGAIN;
        wifi_status_cb = callback;

        wifi_status_cb_devid = dev_id;

        return 0;
}

int omap_wifi_status(struct device *dev, int slot)
{
        return sdp4430_wifi_cd;
}

int sdp4430_wifi_set_carddetect(int val)
{
        printk(KERN_WARNING"%s: %d\n", __func__, val);
        detect_flag = 1;
        printk(KERN_WARNING "%s:detect_flag = %d\n",__func__,detect_flag);
        sdp4430_wifi_cd = val;
        if (wifi_status_cb)
                wifi_status_cb(val, wifi_status_cb_devid);
        else
                printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
        return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(sdp4430_wifi_set_carddetect);
#endif

static int sdp4430_wifi_power_state;
extern void twl_suspend_power_control(int device,int on);  
int sdp4430_wifi_power(int on)
{
        printk(KERN_WARNING"%s: %d\n", __func__, on);
        if (machine_is_omap4_panda())
                //gpio_set_value(PANDA_WIFI_PMENA_GPIO, on); 
                ;
        else
                if(0 == on)
                {
                        gpio_set_value(SDP4430_WIFI_PMENA_GPIO, on);
                        twl_suspend_power_control(1,on);  
                }
                else
                {
                        twl_suspend_power_control(1,on);    
                        gpio_set_value(SDP4430_WIFI_PMENA_GPIO, 1);
                        mdelay(100);
                        gpio_set_value(SDP4430_WIFI_PMENA_GPIO, 0);
                        mdelay(100);
                        gpio_set_value(SDP4430_WIFI_PMENA_GPIO, 1);
                        mdelay(200);
                }
                sdp4430_wifi_power_state = on;
        return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(sdp4430_wifi_power);
#endif

static int sdp4430_wifi_reset_state;
int sdp4430_wifi_reset(int on)
{
        printk(KERN_WARNING"%s: %d\n", __func__, on);
        sdp4430_wifi_reset_state = on;
        return 0;
}
static int char2byte( char* strori, char* outbuf )
{
    int i = 0;
    int temp = 0;
    int sum = 0;
    for( i = 0; i < 12; i++ )
    {
         switch (strori[i]) {
             case '0' ... '9':
                 temp = strori[i] - '0';
                 break;
             case 'a' ... 'f':
                 temp = strori[i] - 'a' + 10;
                 break;
             case 'A' ... 'F':
                 temp = strori[i] - 'A' + 10;
                 break;
             default:
                 break;
        }
        sum += temp;
        if( i % 2 == 0 ){
            outbuf[i/2] |= temp << 4;
        }
        else{
            outbuf[i/2] |= temp;
        }
    }
    return sum;
}

static void read_from_global_buf(unsigned char * buf)
{
    memcpy(buf,g_wifimac,WLAN_MAC_LEN);
    printk("get MAC from g_wifimac: mac=%02x:%02x:%02x:%02x:%02x:%02x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
    return;
}

static int read_from_file(unsigned char * buf)
{
    struct file* filp = NULL;
    mm_segment_t old_fs;
    int result = 0;
    filp = filp_open(MAC_ADDRESS_FILE, O_CREAT|O_RDWR, 0666);
    if(IS_ERR(filp))
    {
        printk("open mac address file error\n");
        return -1;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    filp->f_pos = 0;
    result = filp->f_op->read(filp,buf,WLAN_MAC_LEN,&filp->f_pos);
    if(WLAN_MAC_LEN == result)
    {
        printk("get MAC from the file!\n");
        memcpy(g_wifimac,buf,WLAN_MAC_LEN);
        set_fs(old_fs);
        filp_close(filp,NULL);
        return 0;
    }
    //random mac
    get_random_bytes(buf,WLAN_MAC_LEN);
    buf[0] = 0x0;
    printk("get MAC from Random: mac=%02x:%02x:%02x:%02x:%02x:%02x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
    memcpy(g_wifimac,buf,WLAN_MAC_LEN);

    //update mac -file
    filp->f_pos = 0;
    result = filp->f_op->write(filp,buf,WLAN_MAC_LEN,&filp->f_pos);
    if(WLAN_MAC_LEN != result )
    {
        printk("update NV mac to file error\n");
        set_fs(old_fs);
        filp_close(filp,NULL);
        return -1;
    }
    set_fs(old_fs);
    filp_close(filp,NULL);
    return 0;
}
int sdp4430_wifi_get_mac_addr(unsigned char * buf)
{
    int ret = -1;
    int sum = 0;

    if(NULL == buf)
    {
        printk("sdp4430_wifi_get_mac_addr is error.\r");
        return -1;
    }
    memset(buf, 0, WLAN_MAC_LEN );

    //virable initialize
    struct nve_info_user  info;
    memset( &info, 0, sizeof(info) );
    info.nv_read    = TEL_HUAWEI_NV_READ;
    info.nv_number  = NV_WLAN_NUM;   //nve item
    strcpy( info.nv_name, "MACWLAN" );
    info.valid_size = NV_WLAN_VALID_SIZE;

    if (0 != g_wifimac[0] || 0 != g_wifimac[1] || 0 != g_wifimac[2] || 0 != g_wifimac[3]
       || 0 != g_wifimac[4] || 0 != g_wifimac[5])
    {
        read_from_global_buf(buf);
        return 0;
    }
    //read from nv
    ret = nve_direct_access( &info );
    if (!ret)
    {
        sum = char2byte(info.nv_data, buf );
        if (0 != sum)
        {
            printk("get MAC from NV: mac=%02x:%02x:%02x:%02x:%02x:%02x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
            memcpy(g_wifimac,buf,WLAN_MAC_LEN);
            return 0;
        }
    }
    //read buf from mac-file or random mac
    return read_from_file(buf);
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(sdp4430_wifi_reset);
#endif
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(sdp4430_wifi_get_mac_addr);
#endif
struct wifi_platform_data sdp4430_wifi_control = {
        .set_power      = sdp4430_wifi_power,
        .set_reset      = sdp4430_wifi_reset,
        .set_carddetect = sdp4430_wifi_set_carddetect,
        .get_mac_addr   = sdp4430_wifi_get_mac_addr,
        .mem_prealloc   = wifi_mem_prealloc,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource sdp4430_wifi_resources[] = {
        [0] = {
                .name           = "bcm4329_wlan_irq",   
                .start          = OMAP_GPIO_IRQ(SDP4430_WIFI_IRQ_GPIO),
                .end            = OMAP_GPIO_IRQ(SDP4430_WIFI_IRQ_GPIO),
                .flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL, 
        },
};

static struct platform_device sdp4430_wifi_device = {
    .name           = "bcmdhd_wlan", 
        .id             = 1,
        .num_resources  = ARRAY_SIZE(sdp4430_wifi_resources),
        .resource       = sdp4430_wifi_resources,
        .dev            = {
                .platform_data = &sdp4430_wifi_control,
        },
};
#endif

static int __init sdp4430_wifi_init(void)
{
        int ret = 0;

        if (machine_is_omap_4430sdp()) {
                printk(KERN_WARNING"%s: start\n", __func__);
                init_wifi_mem();     
                ret = gpio_request(SDP4430_WIFI_PMENA_GPIO, "wifi_pmena");
                if (ret < 0) {
                        pr_err("%s: can't reserve GPIO: %d\n", __func__,
                                SDP4430_WIFI_PMENA_GPIO);
                        goto out;
                }
                gpio_direction_output(SDP4430_WIFI_PMENA_GPIO, 0);

                ret = gpio_request(SDP4430_WIFI_IRQ_GPIO, "wifi_irq");
                if (ret < 0) {
                        printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
                                SDP4430_WIFI_IRQ_GPIO);
                        goto out;
                }
                gpio_direction_input(SDP4430_WIFI_IRQ_GPIO);
#ifdef CONFIG_WIFI_CONTROL_FUNC
                ret = platform_device_register(&sdp4430_wifi_device);
#endif
        }
out:
        return ret;
}

device_initcall(sdp4430_wifi_init);
