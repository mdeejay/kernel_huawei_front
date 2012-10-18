/*
 * arch/arm/plat-omap/include/xmd.h
 *
 * Adaptation to IFX XMM IPC.  GPIO config info.
 *
 * Copyright (C) 2010 Infineon Technologies AG. All rights reserved.
 *
 * Author: Khened Chaitanya <Chaitanya.Khened@infineon.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef XMD_H
#define XMD_H

// struct ethhdr definition
#include <linux/if_ether.h>
#include <mach/msm_smd.h>
 /*
  * Exported definitions
  */

#if defined(CONFIG_ARCH_OMAP4)
struct xmm_pad_mux_config {
	char *muxname;
	int  muxval;
};
#endif

#if defined(CONFIG_ARCH_OMAP3)
struct xmm_pad_mux_config {
	int  muxval;
};
#endif

#ifdef CONFIG_OMAP4_XMM_SPI
struct xmd_spi_platform_data {
	int mrdy_gpio;
	int srdy_gpio;

	struct xmm_pad_mux_config mrdy_pinmux;
	struct xmm_pad_mux_config srdy_pinmux;
	struct xmm_pad_mux_config spi_clk_pinmux;
	struct xmm_pad_mux_config spi_simo_pinmux;
	struct xmm_pad_mux_config spi_somi_pinmux;

	struct xmm_pad_mux_config mrdy_safe_pinmux;
	struct xmm_pad_mux_config srdy_safe_pinmux;
	struct xmm_pad_mux_config spi_clk_safe_pinmux;
	struct xmm_pad_mux_config spi_simo_safe_pinmux;
	struct xmm_pad_mux_config spi_somi_safe_pinmux;

	int spi_protocol_version;
};
#endif

#ifdef CONFIG_OMAP4_XMM_HSI
struct xmd_hsi_platform_data {
	struct xmm_pad_mux_config hsi_cawake_pinmux;
	struct xmm_pad_mux_config hsi_cadata_pinmux;
	struct xmm_pad_mux_config hsi_caflag_pinmux;
	struct xmm_pad_mux_config hsi_acready_pinmux;
	struct xmm_pad_mux_config hsi_acwake_pinmux;
	struct xmm_pad_mux_config hsi_acdata_pinmux;
	struct xmm_pad_mux_config hsi_acflag_pinmux;
	struct xmm_pad_mux_config hsi_caready_pinmux;

	struct xmm_pad_mux_config hsi_cawake_safe_pinmux;
	struct xmm_pad_mux_config hsi_cadata_safe_pinmux;
	struct xmm_pad_mux_config hsi_caflag_safe_pinmux;
	struct xmm_pad_mux_config hsi_acready_safe_pinmux;
	struct xmm_pad_mux_config hsi_acwake_safe_pinmux;
	struct xmm_pad_mux_config hsi_acdata_safe_pinmux;
	struct xmm_pad_mux_config hsi_acflag_safe_pinmux;
	struct xmm_pad_mux_config hsi_caready_safe_pinmux;
};
#endif

#ifdef CONFIG_OMAP4_XMM_C2C
struct xmd_c2c_platform_data {
	/* C2C_DATAIN [0..7] */
	struct xmm_pad_mux_config c2c_datain0_pinmux;
	struct xmm_pad_mux_config c2c_datain1_pinmux;
	struct xmm_pad_mux_config c2c_datain2_pinmux;
	struct xmm_pad_mux_config c2c_datain3_pinmux;
	struct xmm_pad_mux_config c2c_datain4_pinmux;
	struct xmm_pad_mux_config c2c_datain5_pinmux;
	struct xmm_pad_mux_config c2c_datain6_pinmux;
	struct xmm_pad_mux_config c2c_datain7_pinmux;

	struct xmm_pad_mux_config c2c_datain0_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain1_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain2_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain3_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain4_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain5_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain6_safe_pinmux;
	struct xmm_pad_mux_config c2c_datain7_safe_pinmux;

	/* C2C_DATAOUT [0..7] */
	struct xmm_pad_mux_config c2c_dataout0_pinmux;
	struct xmm_pad_mux_config c2c_dataout1_pinmux;
	struct xmm_pad_mux_config c2c_dataout2_pinmux;
	struct xmm_pad_mux_config c2c_dataout3_pinmux;
	struct xmm_pad_mux_config c2c_dataout4_pinmux;
	struct xmm_pad_mux_config c2c_dataout5_pinmux;
	struct xmm_pad_mux_config c2c_dataout6_pinmux;
	struct xmm_pad_mux_config c2c_dataout7_pinmux;

	struct xmm_pad_mux_config c2c_dataout0_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout1_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout2_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout3_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout4_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout5_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout6_safe_pinmux;
	struct xmm_pad_mux_config c2c_dataout7_safe_pinmux;

	/* C2C_DATA [8..15] */
	struct xmm_pad_mux_config c2c_data8_pinmux;
	struct xmm_pad_mux_config c2c_data9_pinmux;
	struct xmm_pad_mux_config c2c_data10_pinmux;
	struct xmm_pad_mux_config c2c_data11_pinmux;
	struct xmm_pad_mux_config c2c_data12_pinmux;
	struct xmm_pad_mux_config c2c_data13_pinmux;
	struct xmm_pad_mux_config c2c_data14_pinmux;
	struct xmm_pad_mux_config c2c_data15_pinmux;

	struct xmm_pad_mux_config c2c_data8_safe_pinmux;
	struct xmm_pad_mux_config c2c_data9_safe_pinmux;
	struct xmm_pad_mux_config c2c_data10_safe_pinmux;
	struct xmm_pad_mux_config c2c_data11_safe_pinmux;
	struct xmm_pad_mux_config c2c_data12_safe_pinmux;
	struct xmm_pad_mux_config c2c_data13_safe_pinmux;
	struct xmm_pad_mux_config c2c_data14_safe_pinmux;
	struct xmm_pad_mux_config c2c_data15_safe_pinmux;

	/* C2C_CLK */
	struct xmm_pad_mux_config c2c_clkout0_pinmux;
	struct xmm_pad_mux_config c2c_clkout1_pinmux;
	struct xmm_pad_mux_config c2c_clkin0_pinmux;
	struct xmm_pad_mux_config c2c_clkin1_pinmux;

	struct xmm_pad_mux_config c2c_clkout0_safe_pinmux;
	struct xmm_pad_mux_config c2c_clkout1_safe_pinmux;
	struct xmm_pad_mux_config c2c_clkin0_safe_pinmux;
	struct xmm_pad_mux_config c2c_clkin1_safe_pinmux;

	/* C2C_WAKE_REQ */
	struct xmm_pad_mux_config c2c_wakereqout_pinmux;
	struct xmm_pad_mux_config c2c_wakereqin_pinmux;

	struct xmm_pad_mux_config c2c_wakereqout_safe_pinmux;
	struct xmm_pad_mux_config c2c_wakereqin_safe_pinmux;
};
#endif

struct xmd_boot_platform_data {
	int reset_pmu_req_gpio;
	int baseband_reset_gpio;
	int on_off_gpio;
	int pwr_supply_gpio;
	int reset2_gpio;
	int xmd_ready_gpio;

	struct xmm_pad_mux_config reset_pmu_req_pinmux;
	struct xmm_pad_mux_config baseband_reset_pinmux;
	struct xmm_pad_mux_config on_off_pinmux;
	struct xmm_pad_mux_config pwr_supply_pinmux;
	struct xmm_pad_mux_config reset2_pinmux;
	struct xmm_pad_mux_config xmd_ready_pinmux;

	struct xmm_pad_mux_config reset_pmu_req_safe_pinmux;
	struct xmm_pad_mux_config baseband_reset_safe_pinmux;
	struct xmm_pad_mux_config on_off_safe_pinmux;
	struct xmm_pad_mux_config pwr_supply_safe_pinmux;
	struct xmm_pad_mux_config reset2_safe_pinmux;
	struct xmm_pad_mux_config xmd_ready_safe_pinmux;
};

struct xmd_platform_data {
	struct xmd_boot_platform_data boot_platform_data;
#ifdef CONFIG_OMAP4_XMM_C2C
	struct xmd_c2c_platform_data c2c_platform_data;
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	struct xmd_hsi_platform_data hsi_platform_data;
#endif
#ifdef CONFIG_OMAP4_XMM_SPI
	struct xmd_spi_platform_data spi_platform_data;
#endif
};

struct mux_header_desc {
	int dlci;
	int cr_flag;
	int cmd;
	int pf_flag;
};

struct mux_seg_desc {
	char *buff;
	int len;
};

struct mux_ch_desc {
	volatile struct smd_half_channel *send;
	volatile struct smd_half_channel *recv;
	unsigned char *send_data;
	unsigned char *recv_data;
	unsigned fifo_mask;
	unsigned fifo_size;

	int state;
	int dlci;
	char msc[5];
	int msc_len;
	int packet;
	int eth_hdr_valid;
	struct ethhdr eth_hdr;
};

struct trx_frame_desc {
	/* TX params (from MUX) */
	int tx_len;
	int tx_more_data;
	int tx_flags;
};


#define XMD_SPI_RX_BUFFERS 3

struct xmd_spi_rx_frame_buffer {
	char* address;
	char* payload;
	int free;
};

typedef int (*trx_init_cb_t)(	char *,			// tx_addr
				int, 			// tx_len_max,
				struct trx_frame_desc * // trx_frm_desc
				);

typedef int (*trx_complete_cb_t)(	char *, // rx_addr
					int, 	// rx_len
					int, 	// rx_flags
					int *	// free
				);

enum _XMD_STATE_FLAG_ {
    XMD_STATE_FREE,  /* unsolicited power off modem, free monitor */
    XMD_STATE_OFF,
    XMD_STATE_POWER,
    XMD_STATE_READY,
    XMD_STATE_INVALID,
};

struct xmd_cp_ready_data {
    wait_queue_head_t cp_ready_wait;
    //char cp_ready_flag;
    int cp_ready_flag;   //spinlock protection  /* _XMD_STATE_FLAG_: XMD_STATE_FREE,XMD_STATE_OFF,XMD_STATE_POWER,XMD_STATE_READY,XMD_STATE_INVALID; */
    spinlock_t lock;
};

struct xmd_boot {
	struct xmd_boot_platform_data *boot_platform_data;
	int boot_state;
	int reset2_irq_registered;
	struct work_struct reset2_work;
	int xmd_ready_irq_registered;
	struct work_struct xmd_ready_work;
	struct xmd_cp_ready_data cp_ready;
};

struct xmd_spi {
	/* Frame transfer context */
	int spi_protocol_version;
	int next_frame_size;
	int current_frame_size;
	int tx_more_flag;
	int tx_next_data_size;
	int rx_current_data_size;
	int rx_more_flag;
	int rx_next_data_size;
	int rx_flags;

	/* Driver data */
	trx_init_cb_t trx_init_cb;
	trx_complete_cb_t trx_complete_cb;

	struct xmd_spi_rx_frame_buffer rx_buffer_pool[XMD_SPI_RX_BUFFERS];
	char* rx_spi_buff;
	char* tx_spi_buff;
	char* rx_spi_data;
	char* tx_spi_data;

	int header_size;
	int max_buffer_size;
	int def_buffer_size;
	int user_data_pending;
	struct work_struct spi_work;
	struct workqueue_struct *xmd_spi_wq;
	struct spi_device *spi_dev;

	struct xmd_spi_platform_data *spi_platform_data;
};

struct xmd_mux {
	/* CMUX param */
	int n1;

	/* RX parsing */
	int parse_state;
	int header_count;
	char rx_header[5];
	int data_len;
	int ds_count;
	int fcs;
	struct mux_seg_desc ds[2];

	/* RX parser SPI buffer retention */
	int retained_rx_spi_frame;
	int *retained_buff_done_flag;
	struct mux_seg_desc retained_seg;

	/* SMD interfacing */
	int ch_count;
	struct mux_ch_desc *mux_ch;
	int mux_state;
	void (*notif_upper_layer)(void);
	smd_channel_t* mux_smd;

	/* Driver Data */
	struct work_struct mux_work_smd_notify;
	struct work_struct mux_work_modem_on;
	struct work_struct mux_work_modem_off;
	struct workqueue_struct *xmd_mux_wq;
};


struct xmd_smd {
	void (*notify_low_layer)(void);
};

struct xmd_hsi {
	struct xmd_hsi_platform_data *hsi_platform_data;
};

struct xmd_c2c {
	struct xmd_c2c_platform_data *c2c_platform_data;
	struct device *dev;
	void __iomem *mem_va;
	phys_addr_t mem_pa;
	unsigned long mem_size;
	struct work_struct geno_work;
};

struct xmd_data {
	struct xmd_boot boot_data;
#ifdef CONFIG_OMAP4_XMM_SPI
	struct xmd_spi  spi_data;
	struct xmd_mux  mux_data;
	struct xmd_smd  smd_data;
#endif
#ifdef CONFIG_OMAP4_XMM_HSI
	struct xmd_hsi hsi_data;
#endif
#ifdef CONFIG_OMAP4_XMM_C2C
	struct xmd_c2c c2c_data;
#endif
};

extern struct xmd_data *xmd;


/*
 * XMD internal functions
 */

#ifdef CONFIG_OMAP4_XMM_SPI
void xmd_spi_kick(void);
int xmd_spi_register(trx_init_cb_t, trx_complete_cb_t);
int xmd_spi_register_force(	trx_init_cb_t trx_init,
				trx_complete_cb_t trx_complete,
				int protocol_version,
				int header_size,
				int max_buff_size,
				int def_buff_size);

typedef struct smd_shared smd_shared_t;
typedef struct smd_alloc_elm smd_alloc_elm_t;

void xmd_mux_register ( struct smd_shared_v1 **,
			smd_alloc_elm_t*,
			int,
			void (*)(void),
			void (**)(void));

int xmd_spi_init(void);
void xmd_spi_exit(void);
int xmd_spi_board_init(struct xmd_spi_platform_data *);
int xmd_spi_power_on_reset(void);
int xmd_spi_power_off(void);

int xmd_mux_init(void);
void xmd_mux_exit(void);
int xmd_mux_enable(void);
int xmd_mux_disable(void);


int xmd_smd_init(void);
void xmd_smd_exit(void);
#endif

int xmd_c2c_init(void);
void xmd_c2c_exit(void);
int xmd_c2c_board_init(struct xmd_c2c_platform_data *);
int xmd_c2c_power_on_reset(void);
int xmd_c2c_power_off(void);

int xmd_c2c_dd_init(void);
void xmd_c2c_dd_exit(void);


int xmd_omap_mux_init_signal(struct xmm_pad_mux_config muxcfg);


/*
 * Exported public functions
 */

int xmd_board_init(struct xmd_platform_data *);

int xmd_board_power_on_reset(void);
int xmd_board_power_off(void);

int xmd_board_pci_adapter(void);

#define   XMD_BOOT_TIMEOUT_MS_FLASH      12000
#define   XMD_BOOT_TIMEOUT_MS_FLASHLESS  3000

int wait_for_xmd_ack_timeout(unsigned int ms);
void wake_up_xmd_ack(void);
int get_xmd_ack_cp_ready(void);
void set_xmd_ack_cp_ready(int val);
int is_cp_ready(void);

#endif
