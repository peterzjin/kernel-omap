/*
 * linux/arch/arm/mach-omap2/board-sirloin-peripherals.c
 *
 * Copyright (C) 2014 Peter Jin
 *
 * linux/arch/arm/mach-omap2/board-rx51-peripherals.c
 *
 * Copyright (C) 2008-2009 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mmc/host.h>

#include <plat/mcspi.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/gpmc.h>

#include "mux.h"
#include "hsmmc.h"
#include "common-board-devices.h"

#define SYSTEM_REV_B_USES_VAUX3	0x1699
#define SYSTEM_REV_S_USES_VAUX3 0x8

#define SIRLOIN_USB_TRANSCEIVER_RST_GPIO	67

/* list all spi devices here */
enum {
	SIRLOIN_SPI_MIPID,		/* LCD panel */
};

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

static struct spi_board_info sirloin_peripherals_spi_board_info[] __initdata = {
	[SIRLOIN_SPI_MIPID] = {
		.modalias		= "acx565akm",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 6000000,
		.controller_data	= &mipid_mcspi_config,
	},
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)

#define SIRLOIN_GPIO_CAMERA_LENS_COVER	110
#define SIRLOIN_GPIO_CAMERA_FOCUS		68
#define SIRLOIN_GPIO_CAMERA_CAPTURE	69
#define SIRLOIN_GPIO_KEYPAD_SLIDE		71
#define SIRLOIN_GPIO_LOCK_BUTTON		113
#define SIRLOIN_GPIO_PROXIMITY		89

#define SIRLOIN_GPIO_DEBOUNCE_TIMEOUT	10

static struct gpio_keys_button sirloin_gpio_keys[] = {
	{
		.desc			= "Camera Lens Cover",
		.type			= EV_SW,
		.code			= SW_CAMERA_LENS_COVER,
		.gpio			= SIRLOIN_GPIO_CAMERA_LENS_COVER,
		.active_low		= 1,
		.debounce_interval	= SIRLOIN_GPIO_DEBOUNCE_TIMEOUT,
	}, {
		.desc			= "Camera Focus",
		.type			= EV_KEY,
		.code			= KEY_CAMERA_FOCUS,
		.gpio			= SIRLOIN_GPIO_CAMERA_FOCUS,
		.active_low		= 1,
		.debounce_interval	= SIRLOIN_GPIO_DEBOUNCE_TIMEOUT,
	}, {
		.desc			= "Camera Capture",
		.type			= EV_KEY,
		.code			= KEY_CAMERA,
		.gpio			= SIRLOIN_GPIO_CAMERA_CAPTURE,
		.active_low		= 1,
		.debounce_interval	= SIRLOIN_GPIO_DEBOUNCE_TIMEOUT,
	}, {
		.desc			= "Lock Button",
		.type			= EV_KEY,
		.code			= KEY_SCREENLOCK,
		.gpio			= SIRLOIN_GPIO_LOCK_BUTTON,
		.active_low		= 1,
		.debounce_interval	= SIRLOIN_GPIO_DEBOUNCE_TIMEOUT,
	}, {
		.desc			= "Keypad Slide",
		.type			= EV_SW,
		.code			= SW_KEYPAD_SLIDE,
		.gpio			= SIRLOIN_GPIO_KEYPAD_SLIDE,
		.active_low		= 1,
		.debounce_interval	= SIRLOIN_GPIO_DEBOUNCE_TIMEOUT,
	}, {
		.desc			= "Proximity Sensor",
		.type			= EV_SW,
		.code			= SW_FRONT_PROXIMITY,
		.gpio			= SIRLOIN_GPIO_PROXIMITY,
		.active_low		= 0,
		.debounce_interval	= SIRLOIN_GPIO_DEBOUNCE_TIMEOUT,
	}
};

static struct gpio_keys_platform_data sirloin_gpio_keys_data = {
	.buttons	= sirloin_gpio_keys,
	.nbuttons	= ARRAY_SIZE(sirloin_gpio_keys),
};

static struct platform_device sirloin_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &sirloin_gpio_keys_data,
	},
};

static void __init sirloin_add_gpio_keys(void)
{
	platform_device_register(&sirloin_gpio_keys_device);
}
#else
static void __init sirloin_add_gpio_keys(void)
{
}
#endif /* CONFIG_KEYBOARD_GPIO || CONFIG_KEYBOARD_GPIO_MODULE */

static uint32_t board_keymap[] = {
	/*
	 * Note that KEY(x, 8, KEY_XXX) entries represent "entrire row
	 * connected to the ground" matrix state.
	 */
	KEY(0, 0, KEY_Q),
	KEY(0, 1, KEY_O),
	KEY(0, 2, KEY_P),
	KEY(0, 3, KEY_COMMA),
	KEY(0, 4, KEY_BACKSPACE),
	KEY(0, 6, KEY_A),
	KEY(0, 7, KEY_S),

	KEY(1, 0, KEY_W),
	KEY(1, 1, KEY_D),
	KEY(1, 2, KEY_F),
	KEY(1, 3, KEY_G),
	KEY(1, 4, KEY_H),
	KEY(1, 5, KEY_J),
	KEY(1, 6, KEY_K),
	KEY(1, 7, KEY_L),

	KEY(2, 0, KEY_E),
	KEY(2, 1, KEY_DOT),
	KEY(2, 2, KEY_UP),
	KEY(2, 3, KEY_ENTER),
	KEY(2, 5, KEY_Z),
	KEY(2, 6, KEY_X),
	KEY(2, 7, KEY_C),
	KEY(2, 8, KEY_F9),

	KEY(3, 0, KEY_R),
	KEY(3, 1, KEY_V),
	KEY(3, 2, KEY_B),
	KEY(3, 3, KEY_N),
	KEY(3, 4, KEY_M),
	KEY(3, 5, KEY_SPACE),
	KEY(3, 6, KEY_SPACE),
	KEY(3, 7, KEY_LEFT),

	KEY(4, 0, KEY_T),
	KEY(4, 1, KEY_DOWN),
	KEY(4, 2, KEY_RIGHT),
	KEY(4, 4, KEY_LEFTCTRL),
	KEY(4, 5, KEY_RIGHTALT),
	KEY(4, 6, KEY_LEFTSHIFT),
	KEY(4, 8, KEY_F10),

	KEY(5, 0, KEY_Y),
	KEY(5, 8, KEY_F11),

	KEY(6, 0, KEY_U),

	KEY(7, 0, KEY_I),
	KEY(7, 1, KEY_F7),
	KEY(7, 2, KEY_F8),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data sirloin_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
	.rep		= 1,
};

static struct twl4030_madc_platform_data sirloin_madc_data = {
	.irq_line		= 1,
};

/* Enable input logic and pull all lines up when eMMC is on. */
static struct omap_board_mux sirloin_mmc2_on_mux[] = {
	OMAP3_MUX(SDMMC2_CMD, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT4, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT5, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT6, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT7, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

/* Disable input logic and pull all lines down when eMMC is off. */
static struct omap_board_mux sirloin_mmc2_off_mux[] = {
	OMAP3_MUX(SDMMC2_CMD, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT4, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT5, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT6, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT7, OMAP_PULL_ENA | OMAP_MUX_MODE0),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_mux_partition *partition;

/*
 * Current flows to eMMC when eMMC is off and the data lines are pulled up,
 * so pull them down. N.B. we pull 8 lines because we are using 8 lines.
 */
static void sirloin_mmc2_remux(struct device *dev, int slot, int power_on)
{
	if (power_on)
		omap_mux_write_array(partition, sirloin_mmc2_on_mux);
	else
		omap_mux_write_array(partition, sirloin_mmc2_off_mux);
}

static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.cover_only	= true,
		.gpio_cd	= 160,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
						/* See also sirloin_mmc2_remux */
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
		.remux		= sirloin_mmc2_remux,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply sirloin_vmmc1_supply =
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0");

static struct regulator_consumer_supply sirloin_vaux3_supply =
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1");

static struct regulator_consumer_supply sirloin_vsim_supply =
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.1");

static struct regulator_consumer_supply sirloin_vmmc2_supplies[] = {
	/* tlv320aic3x analog supplies */
	REGULATOR_SUPPLY("AVDD", "2-0018"),
	REGULATOR_SUPPLY("DRVDD", "2-0018"),
	REGULATOR_SUPPLY("AVDD", "2-0019"),
	REGULATOR_SUPPLY("DRVDD", "2-0019"),
	/* tpa6130a2 */
	REGULATOR_SUPPLY("Vdd", "2-0060"),
	/* Keep vmmc as last item. It is not iterated for newer boards */
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};

static struct regulator_consumer_supply sirloin_vio_supplies[] = {
	/* tlv320aic3x digital supplies */
	REGULATOR_SUPPLY("IOVDD", "2-0018"),
	REGULATOR_SUPPLY("DVDD", "2-0018"),
	REGULATOR_SUPPLY("IOVDD", "2-0019"),
	REGULATOR_SUPPLY("DVDD", "2-0019"),
	/* Si4713 IO supply */
	REGULATOR_SUPPLY("vio", "2-0063"),
};

static struct regulator_consumer_supply sirloin_vaux1_consumers[] = {
	REGULATOR_SUPPLY("vdds_sdi", "omapdss"),
	/* Si4713 supply */
	REGULATOR_SUPPLY("vdd", "2-0063"),
};

static struct regulator_consumer_supply sirloin_vdac_supply[] = {
	REGULATOR_SUPPLY("vdda_dac", "omapdss_venc"),
};

static struct regulator_init_data sirloin_vaux1 = {
	.constraints = {
		.name			= "V28",
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.always_on		= true, /* due battery cover sensor */
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(sirloin_vaux1_consumers),
	.consumer_supplies	= sirloin_vaux1_consumers,
};

static struct regulator_init_data sirloin_vaux2 = {
	.constraints = {
		.name			= "VCSI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VAUX3 - adds more power to VIO_18 rail */
static struct regulator_init_data sirloin_vaux3_cam = {
	.constraints = {
		.name			= "VCAM_DIG_18",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sirloin_vaux3_mmc = {
	.constraints = {
		.name			= "VMMC2_30",
		.min_uV			= 2800000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sirloin_vaux3_supply,
};

static struct regulator_init_data sirloin_vaux4 = {
	.constraints = {
		.name			= "VCAM_ANA_28",
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sirloin_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sirloin_vmmc1_supply,
};

static struct regulator_init_data sirloin_vmmc2 = {
	.constraints = {
		.name			= "V28_A",
		.min_uV			= 2800000,
		.max_uV			= 3000000,
		.always_on		= true, /* due VIO leak to AIC34 VDDs */
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(sirloin_vmmc2_supplies),
	.consumer_supplies	= sirloin_vmmc2_supplies,
};

static struct regulator_init_data sirloin_vsim = {
	.constraints = {
		.name			= "VMMC2_IO_18",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sirloin_vsim_supply,
};

static struct regulator_init_data sirloin_vdac = {
	.constraints = {
		.name			= "VDAC",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= sirloin_vdac_supply,
};

static struct regulator_init_data sirloin_vio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(sirloin_vio_supplies),
	.consumer_supplies	= sirloin_vio_supplies,
};

static int sirloin_twlgpio_setup(struct device *dev, unsigned gpio, unsigned n)
{
	/* FIXME this gpio setup is just a placeholder for now */
	gpio_request_one(gpio + 6, GPIOF_OUT_INIT_LOW, "backlight_pwm");
	gpio_request_one(gpio + 7, GPIOF_OUT_INIT_LOW, "speaker_en");

	return 0;
}

static struct twl4030_gpio_platform_data sirloin_gpio_data = {
	.gpio_base		= OMAP_MAX_GPIO_LINES,
	.irq_base		= TWL4030_GPIO_IRQ_BASE,
	.irq_end		= TWL4030_GPIO_IRQ_END,
	.pulldowns		= BIT(0) | BIT(1) | BIT(2) | BIT(3)
				| BIT(4) | BIT(5)
				| BIT(8) | BIT(9) | BIT(10) | BIT(11)
				| BIT(12) | BIT(13) | BIT(14) | BIT(15)
				| BIT(16) | BIT(17) ,
	.setup			= sirloin_twlgpio_setup,
};

static struct twl4030_usb_data sirloin_usb_data = {
	.usb_mode		= T2_USB_MODE_ULPI,
};

static struct twl4030_ins sleep_on_seq[] __initdata = {
/*
 * Turn off everything
 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 1, 0, RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script = sleep_on_seq,
	.size   = ARRAY_SIZE(sleep_on_seq),
	.flags  = TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_seq[] __initdata = {
/*
 * Reenable everything
 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 1, 0, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_script __initdata = {
	.script	= wakeup_seq,
	.size	= ARRAY_SIZE(wakeup_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] __initdata = {
/*
 * Reenable everything
 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 1, 0, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script	= wakeup_p3_seq,
	.size	= ARRAY_SIZE(wakeup_p3_seq),
	.flags	= TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 0, 1, RES_STATE_ACTIVE),
		0x13},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_PP, 0, 3, RES_STATE_OFF), 0x13},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VDD1, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VDD2, RES_STATE_WRST), 0x13},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VPLL1, RES_STATE_WRST), 0x35},
	{MSG_SINGULAR(DEV_GRP_P3, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	/* wakeup12 script should be loaded before sleep script, otherwise a
	   board might hit retention before loading of wakeup script is
	   completed. This can cause boot failures depending on timing issues.
	*/
	&wakeup_script,
	&sleep_on_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] __initdata = {
	{ .resource = RES_VDD1, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = RES_STATE_OFF,
	  .remap_sleep = RES_STATE_OFF
	},
	{ .resource = RES_VDD2, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = RES_STATE_OFF,
	  .remap_sleep = RES_STATE_OFF
	},
	{ .resource = RES_VPLL1, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = RES_STATE_OFF,
	  .remap_sleep = RES_STATE_OFF
	},
	{ .resource = RES_VPLL2, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX1, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX2, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX3, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VAUX4, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VMMC1, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VMMC2, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VDAC, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VSIM, .devgroup = -1,
	  .type = -1, .type2 = 3, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = -1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = -1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_VIO, .devgroup = DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1 , .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_P1 | DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_32KCLKOUT, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_RESET, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ .resource = RES_MAIN_REF, .devgroup = -1,
	  .type = 1, .type2 = -1, .remap_off = -1, .remap_sleep = -1
	},
	{ 0, 0},
};

static struct twl4030_power_data sirloin_t2scripts_data __initdata = {
	.scripts        = twl4030_scripts,
	.num = ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

struct twl4030_codec_vibra_data sirloin_vibra_data __initdata = {
	.coexist	= 0,
};

struct twl4030_codec_data sirloin_codec_data __initdata = {
	.audio_mclk	= 26000000,
	.vibra		= &sirloin_vibra_data,
};

static struct twl4030_platform_data sirloin_twldata __initdata = {
	.irq_base		= TWL4030_IRQ_BASE,
	.irq_end		= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	//.gpio			= &sirloin_gpio_data,
	//.keypad			= &sirloin_kp_data,
	//.madc			= &sirloin_madc_data,
	//.usb			= &sirloin_usb_data,
	//.power			= &sirloin_t2scripts_data,
	//.codec			= &sirloin_codec_data,

	//.vaux1			= &sirloin_vaux1,
	//.vaux2			= &sirloin_vaux2,
	//.vaux4			= &sirloin_vaux4,
	//.vmmc1			= &sirloin_vmmc1,
	//.vsim			= &sirloin_vsim,
	//.vdac			= &sirloin_vdac,
	//.vio			= &sirloin_vio,
};

static int __init sirloin_i2c_init(void)
{
#if 0
	if ((system_rev >= SYSTEM_REV_S_USES_VAUX3 && system_rev < 0x100) ||
	    system_rev >= SYSTEM_REV_B_USES_VAUX3) {
		sirloin_twldata.vaux3 = &sirloin_vaux3_mmc;
		/* Only older boards use VMMC2 for internal MMC */
		sirloin_vmmc2.num_consumer_supplies--;
	} else {
		sirloin_twldata.vaux3 = &sirloin_vaux3_cam;
	}
	sirloin_twldata.vmmc2 = &sirloin_vmmc2;
#endif
	omap_pmic_init(1, 400, "twl5030", INT_34XX_SYS_NIRQ, &sirloin_twldata);
	/*
	omap_register_i2c_bus(2, 100, sirloin_peripherals_i2c_board_info_2,
			      ARRAY_SIZE(sirloin_peripherals_i2c_board_info_2));
	omap_register_i2c_bus(3, 400, NULL, 0);
	*/
	return 0;
}

void __init sirloin_peripherals_init(void)
{
	sirloin_i2c_init();
	/*
	sirloin_add_gpio_keys();
	spi_register_board_info(sirloin_peripherals_spi_board_info,
				ARRAY_SIZE(sirloin_peripherals_spi_board_info));

	partition = omap_mux_get("core");
	if (partition)
		omap2_hsmmc_init(mmc);

	sirloin_charger_init();
	*/
}

