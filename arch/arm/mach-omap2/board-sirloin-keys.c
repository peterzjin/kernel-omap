/*
 * arch/arm/mach-omap2/board-sirloin-keys.c
 *
 * Copyright (C) 2009 Google, Inc.
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>

#include <plat/mux.h>
#include <plat/gpio.h>
#include <linux/gpio_keys.h>
#include <plat/board-sirloin.h>

/* GPIO Keys */
#define VOL_UP_GPIO     (24)
#define VOL_DN_GPIO     (25)
#define SLIDER_GPIO     (27)
#define RING_GPIO       (28)
#define POWER_GPIO      (29)


static struct gpio_keys_button sirloin_gpio_keys[] = {
	{
		.code        = KEY_VOLUMEUP,
		.gpio        = VOL_UP_GPIO,
		.active_low  = 1,
		.desc        = "volume up",
		.type        = EV_KEY,
		.wakeup      = 1,
		.debounce_interval    = 20,
	},
	{
		.code        = KEY_VOLUMEDOWN,
		.gpio        = VOL_DN_GPIO,
		.active_low  = 1,
		.desc        = "volume down",
		.type        = EV_KEY,
		.wakeup      = 1,
		.debounce_interval    = 20,
	},
	/*
	{
		.code        = SW_SLIDER,
		.gpio        = SLIDER_GPIO,
		.active_low  = 1,
		.desc        = "slider",
		.type        = EV_SW,
		.wakeup      = 1,
		.debounce_interval    = 20,
	},
	{
		.code        = SW_RINGER,
		.gpio        = RING_GPIO,
		.active_low  = 1,
		.desc        = "ring silence",
		.type        = EV_SW,
		.wakeup      = 1,
		.debounce_interval    = 20,
	},
	*/
	{
		.code        = KEY_END,
		.gpio        = POWER_GPIO,
		.active_low  = 1,
		.desc        = "power",
		.type        = EV_KEY,
		.wakeup      = 1,
		.debounce_interval    = 20,
	},
};

static struct gpio_keys_platform_data sirloin_gpio_keys_data = {
	.buttons = sirloin_gpio_keys,
	.nbuttons = ARRAY_SIZE(sirloin_gpio_keys),
};

static struct platform_device sirloin_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data= &sirloin_gpio_keys_data,
	},
};

static void __init sirloin_gpio_keys_init(void)
{
	platform_device_register(&sirloin_gpio_keys_device);
}





void __init sirloin_keys_init(void)
{
	sirloin_gpio_keys_init();
}
