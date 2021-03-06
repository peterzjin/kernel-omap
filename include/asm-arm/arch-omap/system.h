/*
 * Copied from linux/include/asm-arm/arch-sa1100/system.h
 * Copyright (c) 1999 Nicolas Pitre <nico@cam.org>
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/hardware.h>

#ifndef CONFIG_MACH_VOICEBLUE
#define voiceblue_reset()		do {} while (0)
#endif

extern void omap_prcm_arch_reset(char mode);

static inline void arch_idle(void)
{
	/* This option is used for debugging with JTAG. cpu_do_idle() will
	 * eventually execute a WFI instruction which will cause a JTAG
	 * connection to break. Therefore we disable the cpu_do_idle() if we
	 * are debugging with JTAG.
	 */
#ifndef CONFIG_DISABLE_WFI
	cpu_do_idle();
#endif
}

static inline void omap1_arch_reset(char mode)
{
	/*
	 * Workaround for 5912/1611b bug mentioned in sprz209d.pdf p. 28
	 * "Global Software Reset Affects Traffic Controller Frequency".
	 */
	if (cpu_is_omap5912()) {
		omap_writew(omap_readw(DPLL_CTL) & ~(1 << 4),
				 DPLL_CTL);
		omap_writew(0x8, ARM_RSTCT1);
	}

	if (machine_is_voiceblue())
		voiceblue_reset();
	else
		omap_writew(1, ARM_RSTCT1);
}

static inline void arch_reset(char mode)
{
	if (!cpu_class_is_omap2())
		omap1_arch_reset(mode);
	else
		omap_prcm_arch_reset(mode);
}

#endif
