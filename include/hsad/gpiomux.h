#ifndef OMAP_GPIO_MUX_H
#define OMAP_GPIO_MUX_H
/*Copyright @ Huawei Technologies Co., Ltd. 2011. All rights reserved.
*/
#define GPMC_AD0        0x0040
#define GPMC_AD1        0x0042
#define GPMC_AD2        0x0044
#define GPMC_AD3        0x0046
#define GPMC_AD4        0x0048
#define GPMC_AD5        0x004A
#define GPMC_AD6        0x004C
#define GPMC_AD7        0x004E
#define GPMC_AD8        0x0050
#define GPMC_AD9        0x0052
#define GPMC_AD10        0x0054
#define GPMC_AD11        0x0056
#define GPMC_AD12        0x0058
#define GPMC_AD13        0x005A
#define GPMC_AD14        0x005C
#define GPMC_AD15        0x005E
#define GPMC_A16        0x0060
#define GPMC_A17        0x0062
#define GPMC_A18        0x0064
#define GPMC_A19        0x0066
#define GPMC_A20        0x0068
#define GPMC_A21        0x006A
#define GPMC_A22        0x006C
#define GPMC_A23        0x006E
#define GPMC_A24        0x0070
#define GPMC_A25        0x0072
#define GPMC_NCS0        0x0074
#define GPMC_NCS1        0x0076
#define GPMC_NCS2        0x0078
#define GPMC_NCS3        0x007A
#define GPMC_NWP        0x007C
#define GPMC_CLK        0x007E
#define GPMC_NADV_ALE        0x0080
#define GPMC_NOE        0x0082
#define GPMC_NWE        0x0084
#define GPMC_NBE0_CLE        0x0086
#define GPMC_NBE1        0x0088
#define GPMC_WAIT0        0x008A
#define GPMC_WAIT1        0x008C
#define C2C_DATA11        0x008E
#define C2C_DATA12        0x0090
#define C2C_DATA13        0x0092
#define C2C_DATA14        0x0094
#define C2C_DATA15        0x0096
#define HDMI_HPD        0x0098
#define HDMI_CEC        0x009A
#define HDMI_DDC_SCL        0x009C
#define HDMI_DDC_SDA        0x009E
#define CSI21_DX0        0x00A0
#define CSI21_DY0        0x00A2
#define CSI21_DX1        0x00A4
#define CSI21_DY1        0x00A6
#define CSI21_DX2        0x00A8
#define CSI21_DY2        0x00AA
#define CSI21_DX3        0x00AC
#define CSI21_DY3        0x00AE
#define CSI21_DX4        0x00B0
#define CSI21_DY4        0x00B2
#define CSI22_DX0        0x00B4
#define CSI22_DY0        0x00B6
#define CSI22_DX1        0x00B8
#define CSI22_DY1        0x00BA
#define CAM_SHUTTER        0x00BC
#define CAM_STROBE        0x00BE
#define CAM_GLOBALRESET        0x00C0
#define USBB1_ULPITLL_CLK    0x00C2
#define USBB1_ULPITLL_STP    0x00C4
#define USBB1_ULPITLL_DIR    0x00C6
#define USBB1_ULPITLL_NXT    0x00C8
#define USBB1_ULPITLL_DAT0    0x00CA
#define USBB1_ULPITLL_DAT1    0x00CC
#define USBB1_ULPITLL_DAT2    0x00CE
#define USBB1_ULPITLL_DAT3    0x00D0
#define USBB1_ULPITLL_DAT4    0x00D2
#define USBB1_ULPITLL_DAT5    0x00D4
#define USBB1_ULPITLL_DAT6    0x00D6
#define USBB1_ULPITLL_DAT7    0x00D8
#define USBB1_HSIC_DATA        0x00DA
#define USBB1_HSIC_STROBE    0x00DC
#define USBC1_ICUSB_DP        0x00DE
#define USBC1_ICUSB_DM        0x00E0
#define SDMMC1_CLK        0x00E2
#define SDMMC1_CMD        0x00E4
#define SDMMC1_DAT0        0x00E6
#define SDMMC1_DAT1        0x00E8
#define SDMMC1_DAT2        0x00EA
#define SDMMC1_DAT3        0x00EC
#define SDMMC1_DAT4        0x00EE
#define SDMMC1_DAT5        0x00F0
#define SDMMC1_DAT6        0x00F2
#define SDMMC1_DAT7        0x00F4
#define ABE_MCBSP2_CLKX        0x00F6
#define ABE_MCBSP2_DR        0x00F8
#define ABE_MCBSP2_DX        0x00FA
#define ABE_MCBSP2_FSX        0x00FC
#define ABE_MCBSP1_CLKX        0x00FE
#define ABE_MCBSP1_DR        0x0100
#define ABE_MCBSP1_DX        0x0102
#define ABE_MCBSP1_FSX        0x0104
#define ABE_PDM_UL_DATA        0x0106
#define ABE_PDM_DL_DATA        0x0108
#define ABE_PDM_FRAME        0x010A
#define ABE_PDM_LB_CLK        0x010C
#define ABE_CLKS        0x010E
#define ABE_DMIC_CLK1        0x0110
#define ABE_DMIC_DIN1        0x0112
#define ABE_DMIC_DIN2        0x0114
#define ABE_DMIC_DIN3        0x0116
#define UART2_CTS        0x0118
#define UART2_RTS        0x011A
#define UART2_RX        0x011C
#define UART2_TX        0x011E
#define HDQ_SIO            0x0120
#define I2C1_SCL        0x0122
#define I2C1_SDA        0x0124
#define I2C2_SCL        0x0126
#define I2C2_SDA        0x0128
#define I2C3_SCL        0x012A
#define I2C3_SDA        0x012C
#define I2C4_SCL        0x012E
#define I2C4_SDA        0x0130
#define MCSPI1_CLK        0x0132
#define MCSPI1_SOMI        0x0134
#define MCSPI1_SIMO        0x0136
#define MCSPI1_CS0        0x0138
#define MCSPI1_CS1        0x013A
#define MCSPI1_CS2        0x013C
#define MCSPI1_CS3        0x013E
#define UART3_CTS_RCTX        0x0140
#define UART3_RTS_SD        0x0142
#define UART3_RX_IRRX        0x0144
#define UART3_TX_IRTX        0x0146
#define SDMMC5_CLK        0x0148
#define SDMMC5_CMD        0x014A
#define SDMMC5_DAT0        0x014C
#define SDMMC5_DAT1        0x014E
#define SDMMC5_DAT2        0x0150
#define SDMMC5_DAT3        0x0152
#define MCSPI4_CLK        0x0154
#define MCSPI4_SIMO        0x0156
#define MCSPI4_SOMI        0x0158
#define MCSPI4_CS0        0x015A
#define UART4_RX        0x015C
#define UART4_TX        0x015E
#define USBB2_ULPITLL_CLK    0x0160
#define USBB2_ULPITLL_STP    0x0162
#define USBB2_ULPITLL_DIR    0x0164
#define USBB2_ULPITLL_NXT    0x0166
#define USBB2_ULPITLL_DAT0    0x0168
#define USBB2_ULPITLL_DAT1    0x016A
#define USBB2_ULPITLL_DAT2    0x016C
#define USBB2_ULPITLL_DAT3    0x016E
#define USBB2_ULPITLL_DAT4    0x0170
#define USBB2_ULPITLL_DAT5    0x0172
#define USBB2_ULPITLL_DAT6    0x0174
#define USBB2_ULPITLL_DAT7    0x0176
#define USBB2_HSIC_DATA        0x0178
#define USBB2_HSIC_STROBE    0x017A
#define UNIPRO_TX0        0x017C
#define UNIPRO_TY0        0x017E
#define UNIPRO_TX1        0x0180
#define UNIPRO_TY1        0x0182
#define UNIPRO_TX2        0x0184
#define UNIPRO_TY2        0x0186
#define UNIPRO_RX0        0x0188
#define UNIPRO_RY0        0x018A
#define UNIPRO_RX1        0x018C
#define UNIPRO_RY1        0x018E
#define UNIPRO_RX2        0x0190
#define UNIPRO_RY2        0x0192
#define USBA0_OTG_CE        0x0194
#define USBA0_OTG_DP        0x0196
#define USBA0_OTG_DM        0x0198
#define FREF_CLK1_OUT        0x019A
#define FREF_CLK2_OUT        0x019C
#define SYS_NIRQ1        0x019E
#define SYS_NIRQ2        0x01A0
#define SYS_BOOT0        0x01A2
#define SYS_BOOT1        0x01A4
#define SYS_BOOT2        0x01A6
#define SYS_BOOT3        0x01A8
#define SYS_BOOT4        0x01AA
#define SYS_BOOT5        0x01AC
#define DPM_EMU0        0x01AE
#define DPM_EMU1        0x01B0
#define DPM_EMU2        0x01B2
#define DPM_EMU3        0x01B4
#define DPM_EMU4        0x01B6
#define DPM_EMU5        0x01B8
#define DPM_EMU6        0x01BA
#define DPM_EMU7        0x01BC
#define DPM_EMU8        0x01BE
#define DPM_EMU9        0x01C0
#define DPM_EMU10        0x01C2
#define DPM_EMU11        0x01C4
#define DPM_EMU12        0x01C6
#define DPM_EMU13        0x01C8
#define DPM_EMU14        0x01CA
#define DPM_EMU15        0x01CC
#define DPM_EMU16        0x01CE
#define DPM_EMU17        0x01D0
#define DPM_EMU18        0x01D2
#define DPM_EMU19        0x01D4
#define CSI22_DX2        0x01D6
#define CSI22_DY2        0x01F4

#define PAD0_SIM_IO             0x0040
#define PAD1_SIM_CLK            0x0042
#define PAD0_SIM_RESET          0x0044
#define PAD1_SIM_CD             0x0046
#define PAD0_SIM_PWRCTRL        0x0048
#define PAD1_SR_SCL             0x004A
#define PAD0_SR_SDA             0x004C
#define PAD1_FREF_XTAL_IN       0x004E
#define PAD0_FREF_SLICER_IN     0x0050
#define PAD1_FREF_CLK_IOREQ     0x0052
#define PAD0_FREF_CLK0_OUT      0x0054
#define PAD1_FREF_CLK3_REQ      0x0056
#define PAD0_FREF_CLK3_OUT      0x0058
#define PAD1_FREF_CLK4_REQ      0x005A
#define PAD0_FREF_CLK4_OUT      0x005C
#define PAD1_SYS_32K            0x005E
#define PAD0_SYS_NRESPWRON      0x0060
#define PAD1_SYS_NRESWARM       0x0062
#define PAD0_SYS_PWR_REQ        0x0064
#define PAD1_SYS_PWRON_RESET    0x0066
#define PAD0_SYS_BOOT6          0x0068
#define PAD1_SYS_BOOT7          0x006A
#define PAD0_JTAG_NTRST         0x006C
#define PAD1_JTAG_TCK           0x006E
#define PAD0_JTAG_RTCK          0x0070
#define PAD1_JTAG_TMS_TMSC      0x0072
#define PAD0_JTAG_TDI           0x0074
#define PAD1_JTAG_TDO           0x0076

#define GPIOMUX_NONSET 0x0ff4
#define GPIO_NORMAL_MODE_UMASK  0xfe00 //bit0~bit8 for normal mode
#define GPIO_MUX_MODE_OFFSET       0
#define GPIO_PULL_TYPE_OFFSET      3
#define GPIO_INPUT_OFFSET          8
#define GPIO_OFF_MODE_UNMASK    0xc1ff //bit9~bit13 for offmode
#define GPIO_OFF_ENABLE_OFFSET     9
#define GPIO_OFF_VAL_OFFSET        10
#define GPIO_OFF_PULL_TYPE_OFFSET  12
#define MUX_PIN_END  0xffffffff
#define GPIO_INVALID_NUM 0xffffffff

enum gpiomux_mode {
    GPIOMUX_M0 = 0,
    GPIOMUX_M1,
    GPIOMUX_M2,
    GPIOMUX_M3,
    GPIOMUX_M4,
    GPIOMUX_M5,
    GPIOMUX_M6,
    GPIOMUX_M7,
};

enum gpiomux_pull {
    GPIOMUX_NOPULL = 0,
    GPIOMUX_PULL_DOWN,
    GPIOMUX_PULL_UP = 3,
    GPIOMUX_DISABLE,
};

enum gpiomux_dir {
    GPIOMUX_OUT = 0,
    GPIOMUX_IN,
};

enum gpiomux_out_value {
    GPIOMUX_OUT_LOW = 0,
    GPIOMUX_OUT_HIGH,
    MUX_NO_OUT,
};

enum gpiomux_off_value {
    MUX_OFF_OUTPTD = 0,
    MUX_OFF_IN,
    MUX_OFF_OUTPTU,
};

enum gpiomux_wkup {
    WAKE_UP_DISABLE = 0,
    WAKE_UP_ENABLE,
};

typedef struct {
    char net_name[32];  //not include gpio num
    char general_name[32]; //include gpio num
    int gpio_num;
    unsigned int mux_pin;
    enum gpiomux_mode  mux_mode;
    enum gpiomux_pull  init_pull_type;
    enum gpiomux_dir   init_direction;
    enum gpiomux_pull  offmode_pull_type;
    enum gpiomux_off_value offmode_value;
    enum gpiomux_wkup  wakeup_enable;
}gpiomux_setting;

typedef struct {
    unsigned int boardid;
    unsigned int inittbl;
}board_init_entry;

extern void board_muxconf_init(void);

#endif
