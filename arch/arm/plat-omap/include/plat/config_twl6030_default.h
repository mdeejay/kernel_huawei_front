
#ifndef CONFIG_TWL6030_DEFAULT_H
#define CONFIG_TWL6030_DEFAULT_H

#include <linux/i2c/twl.h>
#include <plat/usb.h>

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};


static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
	.vddhf_uV	= 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
	.voltage_raise_speed = 0x26,
};

static int twl6040_init(void)
{
/*
        u8 rev = 0;
        int ret;

        ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
                                &rev, TWL6040_REG_ASICREV);
        if (ret)
                return ret;
*/
        /*
         * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
         * when AUDPWRON = 0, which causes current drain on this pin's
         * pull-down on OMAP side. The workaround consists of disabling
         * pull-down resistor of ABE_PDM_UL_DATA pin
         * Impacted revisions: ES1.1 and ES1.2 (both share same ASICREV value)
         */
//      if (rev == TWL6040_REV_1_1)
//              omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
//                      OMAP_PIN_INPUT);

        return 0;
}


static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
	.init		= twl6040_init,
};

static struct regulator_init_data twl6030_clk32kg = {
       .constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on		= true,
       },
};


static struct twl4030_madc_platform_data twl6030_madc = {
	.irq_line	= 1,
};

static int sdp4430_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data twl6030_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};

#endif
