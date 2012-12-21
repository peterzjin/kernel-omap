/*
 * arch/arm/mach-omap2/board-sirloin-spi.c
 *
 * Copyright (C) 2013 Peter
 *
 * arch/arm/mach-omap2/board-sholes-spi.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/resource.h>
#include <plat/omap34xx.h>

static struct omap2_mcspi_device_config lcdpanel_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};

static struct spi_board_info sirloin_spi_board_info[] __initdata = {
	{
		.modalias        = "acx567akm",
		.bus_num         = 3,
		.chip_select     = 0,
		.max_speed_hz    = 1500000,
		.controller_data = &lcdpanel_mcspi_config,
	},
};

void __init sirloin_spi_init(void)
{
	omap_cfg_reg(H26_34XX_McSPI3_CLK);
	omap_cfg_reg(H25_34XX_McSPI3_O);
	omap_cfg_reg(E28_34XX_McSPI3_I);
	omap_cfg_reg(J26_34XX_McSPI3_CS0);

	spi_register_board_info(sirloin_spi_board_info,
			       ARRAY_SIZE(sirloin_spi_board_info));

}
