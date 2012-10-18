#ifndef __ARCH_ARM_PLAT_OMAP_HUAWEI_DSI_PANEL_H
#define __ARCH_ARM_PLAT_OMAP_HUAWEI_DSI_PANEL_H


//#include "display.h"
#include <video/omapdss.h>

/**
 * struct huawei_dsi_panel_data - huawei samsung DSI panel driver configuration
 * @name: panel name
 * @use_ext_te: use external TE
 * @ext_te_gpio: external TE GPIO
 * @use_esd_check: perform ESD checks
 * @max_backlight_level: maximum backlight level
 * @set_backlight: pointer to backlight set function
 * @get_backlight: pointer to backlight get function
 */

struct sp_dsi_panel_data {
	const char *name;

	int reset_gpio;

	bool use_ext_te;
	int ext_te_gpio;

	bool use_esd_check;
	unsigned esd_interval;
	unsigned ulps_timeout;

	int max_backlight_level;
	int (*set_backlight)(struct omap_dss_device *dssdev, int level);
	int (*get_backlight)(struct omap_dss_device *dssdev);
};

#endif /* __ARCH_ARM_PLAT_OMAP_NOKIA_DSI_PANEL_H */
