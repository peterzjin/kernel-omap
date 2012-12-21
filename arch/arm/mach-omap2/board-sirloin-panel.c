/*
 * linux/arch/arm/mach-omap2/board-sirloin-panel.c
 *
 * Copyright (C) 2013 Peter
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/omapfb.h>

#include <plat/display.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/resource.h>
#include <plat/board-sirloin.h>

#define SIRLOIN_DISPLAY_RESET_GPIO	68

static struct omapfb_platform_data sirloin_fb_data = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			{
				.format = OMAPFB_COLOR_ARGB32,
				.format_used = 1,
				.paddr = SIRLOIN_FBMEM_START,
				.size = SIRLOIN_FBMEM_SIZE,
			},
		},
	},
};

static struct omap_dss_device sirloin_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_SDI,
	.name = "lcd",
	.driver_name = "panel-acx567akm",
	/*
	.phy.dsi.clk_lane = 1,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 2,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 3,
	.phy.dsi.data2_pol = 0,
	.phy.dsi.div.regn = 12,
	.phy.dsi.div.regm = 160,
	.phy.dsi.div.regm3 = 5,
	.phy.dsi.div.regm4 = 5,
	.phy.dsi.div.lck_div = 1,
	.phy.dsi.div.pck_div = 4,
	.phy.dsi.div.lp_clk_div = 5,
	*/
	.reset_gpio = SIRLOIN_DISPLAY_RESET_GPIO,
};

static struct omap_dss_device *sirloin_dss_devices[] = {
	&sirloin_lcd_device,
};

static struct omap_dss_board_info sirloin_dss_data = {
	.num_devices = ARRAY_SIZE(sirloin_dss_devices),
	.devices = sirloin_dss_devices,
	.default_device = &sirloin_lcd_device,
};

struct platform_device sirloin_dss_device = {
        .name          = "omapdss",
        .id            = -1,
        .dev            = {
                .platform_data = &sirloin_dss_data,
        },
};

void __init sirloin_panel_init(void)
{
	int ret;

#if 0
	omap_cfg_reg(AG22_34XX_DSI_DX0);
	omap_cfg_reg(AH22_34XX_DSI_DY0);
	omap_cfg_reg(AG23_34XX_DSI_DX1);
	omap_cfg_reg(AH23_34XX_DSI_DY1);
	omap_cfg_reg(AG24_34XX_DSI_DX2);
	omap_cfg_reg(AH24_34XX_DSI_DY2);
	/* disp reset b */
	omap_cfg_reg(AE4_34XX_GPIO136_OUT);
#endif

	omapfb_set_platform_data(&sirloin_fb_data);

	ret = gpio_request(SIRLOIN_DISPLAY_RESET_GPIO, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		goto error;
	}

	platform_device_register(&sirloin_dss_device);
	return;

error:
	gpio_free(SIRLOIN_DISPLAY_RESET_GPIO);
}
