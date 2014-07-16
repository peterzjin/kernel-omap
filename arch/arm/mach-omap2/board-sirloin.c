/*
 * linux/arch/arm/mach-omap2/board-sirloin.c
 *
 * Copyright (C) 2013 Peter Jin
 *
 * linux/arch/arm/mach-omap2/board-rx51.c
 *
 * Copyright (C) 2007, 2008 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/usb.h>

#include "sdram-qimonda-hyb18m512160af-6.h"
#include "mux.h"
#include "pm.h"
#include "prm-regbits-34xx.h"

/*
 * cpuidle C-states definition override from the default values.
 * The 'exit_latency' field is the sum of sleep and wake-up latencies.
 */
static struct cpuidle_params sirloin_cpuidle_params[] = {
	/* C1 */
	{110 + 162, 5 , 1},
	/* C2 */
	{106 + 180, 309, 1},
	/* C3 */
	{107 + 410, 46057, 0},
	/* C4 */
	{121 + 3374, 46057, 0},
	/* C5 */
	{855 + 1146, 46057, 1},
	/* C6 */
	{7580 + 4134, 484329, 0},
	/* C7 */
	{7505 + 15274, 484329, 1},
};

void debug_rst()
{
	writel(0x4, OMAP3430_PRM_RSTCTRL);
}

static void __init sirloin_init_early(void)
{
	struct omap_sdrc_params *sdrc_params;

	omap2_init_common_infrastructure();
	omap2_init_common_devices(hyb18m512160af6_sdrc_params, NULL);
}

extern void __init sirloin_peripherals_init(void);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
	.power			= 0,
};

static void __init sirloin_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_pm_init_cpuidle(sirloin_cpuidle_params);
	omap_serial_init();
	usb_musb_init(&musb_board_data);
	//sirloin_peripherals_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
}

static void __init sirloin_map_io(void)
{
	omap2_set_globals_3xxx();
	omap34xx_map_common_io();
}

static void __init sirloin_reserve(void)
{
	//sirloin_video_mem_init();
	omap_reserve();
}

MACHINE_START(SIRLOIN, "Palm Pre Plus")
	/* Maintainer: Peter Jin <peter.zjin@gmail.com> */
	.boot_params	= 0x80000100,
	.reserve	= sirloin_reserve,
	.map_io		= sirloin_map_io,
	.init_early	= sirloin_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= sirloin_init,
	.timer		= &omap_timer,
MACHINE_END
