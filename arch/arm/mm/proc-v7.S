/*
 *  linux/arch/arm/mm/proc-v7.S
 *
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is the "shell" of the ARMv7 processor support.
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/elf.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>

#include "proc-macros.S"

#define TTB_C		(1 << 0)
#define TTB_S		(1 << 1)
#define TTB_RGN_OC_WT	(2 << 3)
#define TTB_RGN_OC_WB	(3 << 3)

ENTRY(cpu_v7_proc_init)
	mov	pc, lr

ENTRY(cpu_v7_proc_fin)
	mov	pc, lr

/*
 *	cpu_v7_reset(loc)
 *
 *	Perform a soft reset of the system.  Put the CPU into the
 *	same state as it would be if it had been reset, and branch
 *	to what would be the reset vector.
 *
 *	- loc   - location to jump to for soft reset
 *
 *	It is assumed that:
 */
	.align	5
ENTRY(cpu_v7_reset)
	mov	pc, r0

/*
 *	cpu_v7_do_idle()
 *
 *	Idle the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_v7_do_idle)
	.long	0xe320f003			@ ARM V7 WFI instruction
	mov	pc, lr

ENTRY(cpu_v7_dcache_clean_area)
#ifndef TLB_CAN_READ_FROM_L1_CACHE
	dcache_line_size r2, r3
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, r2
	subs	r1, r1, r2
	bhi	1b
	dsb
#endif
	mov	pc, lr

/*
 *	cpu_v7_switch_mm(pgd_phys, tsk)
 *
 *	Set the translation table base pointer to be pgd_phys
 *
 *	- pgd_phys - physical address of new TTB
 *
 *	It is assumed that:
 *	- we are not using split page tables
 */
ENTRY(cpu_v7_switch_mm)
#ifdef CONFIG_MMU
	mov	r2, #0
	ldr	r1, [r1, #MM_CONTEXT_ID]	@ get mm->context.id
	orr	r0, r0, #TTB_RGN_OC_WB		@ mark PTWs outer cacheable, WB
	mcr	p15, 0, r2, c13, c0, 1		@ set reserved context ID
	isb
1:	mcr	p15, 0, r0, c2, c0, 0		@ set TTB 0
	isb
	mcr	p15, 0, r1, c13, c0, 1		@ set context ID
	isb
#endif
	mov	pc, lr

/*
 *	cpu_v7_set_pte_ext(ptep, pte)
 *
 *	Set a level 2 translation table entry.
 *
 *	- ptep  - pointer to level 2 translation table entry
 *		  (hardware version is stored at -1024 bytes)
 *	- pte   - PTE value to store
 *	- ext	- value for extended PTE bits
 *
 *	Permissions:
 *	  YUWD  APX AP1 AP0	SVC	User
 *	  0xxx   0   0   0	no acc	no acc
 *	  100x   1   0   1	r/o	no acc
 *	  10x0   1   0   1	r/o	no acc
 *	  1011   0   0   1	r/w	no acc
 *	  110x   0   1   0	r/w	r/o
 *	  11x0   0   1   0	r/w	r/o
 *	  1111   0   1   1	r/w	r/w
 */
ENTRY(cpu_v7_set_pte_ext)
#ifdef CONFIG_MMU
	str	r1, [r0], #-2048		@ linux version

	bic	r3, r1, #0x000003f0
	bic	r3, r3, #0x00000003
	orr	r3, r3, r2
	orr	r3, r3, #PTE_EXT_AP0 | 2

	tst	r1, #L_PTE_WRITE
	tstne	r1, #L_PTE_DIRTY
	orreq	r3, r3, #PTE_EXT_APX

	tst	r1, #L_PTE_USER
	orrne	r3, r3, #PTE_EXT_AP1
	tstne	r3, #PTE_EXT_APX
	bicne	r3, r3, #PTE_EXT_APX | PTE_EXT_AP0

	tst	r1, #L_PTE_YOUNG
	biceq	r3, r3, #PTE_EXT_APX | PTE_EXT_AP_MASK

	tst	r1, #L_PTE_EXEC
	orreq	r3, r3, #PTE_EXT_XN

	tst	r1, #L_PTE_PRESENT
	moveq	r3, #0

	str	r3, [r0]
	mcr	p15, 0, r0, c7, c10, 1		@ flush_pte
#endif
	mov	pc, lr

cpu_v7_name:
	.ascii	"ARMv7 Processor"
	.align

	.section ".text.init", #alloc, #execinstr

/*
 *	__v7_setup
 *
 *	Initialise TLB, Caches, and MMU state ready to switch the MMU
 *	on.  Return in r0 the new CP15 C1 control register setting.
 *
 *	We automatically detect if we have a Harvard cache, and use the
 *	Harvard cache control instructions insead of the unified cache
 *	control instructions.
 *
 *	This should be able to cover all ARMv7 cores.
 *
 *	It is assumed that:
 *	- cache type register is implemented
 */
__v7_setup:
	adr	r12, __v7_setup_stack		@ the local stack
	stmia	r12, {r0-r5, r7, r9, r11, lr}
	bl	v7_flush_dcache_all
	ldmia	r12, {r0-r5, r7, r9, r11, lr}
	mov	r10, #0
#ifdef HARVARD_CACHE
	mcr	p15, 0, r10, c7, c5, 0		@ I+BTB cache invalidate
#endif
	dsb
#ifdef CONFIG_MMU
	mcr	p15, 0, r10, c8, c7, 0		@ invalidate I + D TLBs
	mcr	p15, 0, r10, c2, c0, 2		@ TTB control register
	orr	r4, r4, #TTB_RGN_OC_WB		@ mark PTWs outer cacheable, WB
	mcr	p15, 0, r4, c2, c0, 0		@ load TTB0
	mcr	p15, 0, r4, c2, c0, 1		@ load TTB1
	mov	r10, #0x1f			@ domains 0, 1 = manager
	mcr	p15, 0, r10, c3, c0, 0		@ load domain access register
#endif

	@ set up for allowing performance counting registers in user space
	mov r10, #1
	mcr p15, 0, r10, c9, c14, 0		@ allow user access in the USEREN reg
	mrc p15, 0, r10, c9, c12, 0

	@ The following conflicts with oprofile.  We'll leave it out
	@ as you can set these in userspace.
/**	bic r10, #(1<<3)				@ cycle count every cycle
	orr r10, #(1<<2)|(1<<0)			@ clear and enable cycle counter
	mcr p15, 0, r10, c9, c12, 0		@ PMNC
	mov	r10, #(1<<31)
	mcr p15, 0, r10, c9, c12, 1		@ enable cycle counter
 **/

#if defined(CONFIG_ARCH_OMAP3)
	@ L2 cache is enabled in the aux control register
	mrc	p15, 0, r0, c1, c0, 1
	orr	r0, r0, #0x11		@ speculative+no-alais protection
#ifdef CONFIG_CPU_L2CACHE_DISABLE
	bic	r0, r0, #0x2		@ disable L2 Cache.
#else
	orr	r0, r0, #0x2		@ enaable L2 Cache.
#endif

/* On 3430 ES2.0 ZeBu and silicon, Aux Ctrl Reg can be written outside
 * Secure mode also
 */
#if defined(CONFIG_TRUST_ZONE_OFF) || defined(CONFIG_OMAP_VIRTIO) || \
    defined(CONFIG_OMAP_ZEBU)      || defined(CONFIG_ARCH_OMAP34XX)
	mcr     p15, 0, r0, c1, c0, 1
#else
	mov r10, r12		@ r12 is this function's sp. back it up.

	mov r12, #0x3           @ AUXCR service
	.word 0xE1600070        @ Call OMAP SMI monitor service

	mov r12, r10		@ restore r12. 
#endif

#ifdef ARCH_OMAP34XX
#ifdef CONFIG_CPU_LOCKDOWN_TO_64K_L2
	mov	r10, #0xfc
	mcr     p15, 1, r10, c9, c0, 0
#endif

#ifdef CONFIG_CPU_LOCKDOWN_TO_128K_L2
	mov	r10, #0xf0
	mcr     p15, 1, r10, c9, c0, 0
#endif

#ifdef CONFIG_CPU_LOCKDOWN_TO_256K_L2
	mov	r10, #0x00
	mcr     p15, 1, r10, c9, c0, 0
#endif
#endif

	adr	r5, v7_crval
	ldmia	r5, {r5, r6}
   	mrc	p15, 0, r0, c1, c0, 0		@ read control register
	bic	r0, r0, r5			@ clear bits them
	orr	r0, r0, r6			@ set them
	mov	pc, lr			@ return to head.S:__ret

	/*
	 *  TAT N EV   F	H   R
	 * .EFR M.EE .UI. ..A. .RVI Z... B... .CAM
	 * 0xxx x0xx 11x0 01x1 0xxx x000 0111 1xxx < forced typical
	 * r	rr   rr r rr r r	 rrr rrrr r	< always read only
	 * .000 ..00 ..0. ..0. .011 1... .... .101 < we want
	 */
	.type   v7_crval, #object
v7_crval:
	crval   clear=0x7322f006, mmuset=0x00003805, ucset=0x00001804
#else

#ifndef CONFIG_CPU_L2CACHE_DISABLE
	@ L2 cache configuration in the L2 aux control register
	mrc	p15, 1, r10, c9, c0, 2
	bic	r10, r10, #(1 << 16)		@ L2 outer cache
	mcr	p15, 1, r10, c9, c0, 2
	@ L2 cache is enabled in the aux control register
	mrc	p15, 0, r10, c1, c0, 1
	orr	r10, r10, #2
	mcr	p15, 0, r10, c1, c0, 1
#endif
	mrc	p15, 0, r0, c1, c0, 0		@ read control register
	ldr	r10, cr1_clear			@ get mask for bits to clear
	bic	r0, r0, r10			@ clear bits them
	ldr	r10, cr1_set			@ get mask for bits to set
	orr	r0, r0, r10			@ set them
	mov	pc, lr				@ return to head.S:__ret

	/*
	 *         V X F   I D LR
	 * .... ...E PUI. .T.T 4RVI ZFRS BLDP WCAM
	 * rrrr rrrx xxx0 0101 xxxx xxxx x111 xxxx < forced
	 *         0 110       0011 1.00 .111 1101 < we want
	 */
	.type	cr1_clear, #object
	.type	cr1_set, #object
cr1_clear:
	.word	0x0120c302
cr1_set:
	.word	0x00c0387d
#endif

__v7_setup_stack:
	.space	4 * 11				@ 11 registers

	.type	v7_processor_functions, #object
ENTRY(v7_processor_functions)
	.word	v7_early_abort
	.word	cpu_v7_proc_init
	.word	cpu_v7_proc_fin
	.word	cpu_v7_reset
	.word	cpu_v7_do_idle
	.word	cpu_v7_dcache_clean_area
	.word	cpu_v7_switch_mm
	.word	cpu_v7_set_pte_ext
	.word	pabort_ifar
	.size	v7_processor_functions, . - v7_processor_functions

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv7"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v7"
	.size	cpu_elf_name, . - cpu_elf_name
	.align

	.section ".proc.info.init", #alloc, #execinstr

	/*
	 * Match any ARMv7 processor core.
	 */
	.type	__v7_proc_info, #object
__v7_proc_info:
	.long	0x000f0000		@ Required ID value
	.long	0x000f0000		@ Mask for ID
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_XN | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__v7_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_v7_name
	.long	v7_processor_functions
	.long	v7wbi_tlb_fns
	.long	v6_user_fns
	.long	v7_cache_fns
	.size	__v7_proc_info, . - __v7_proc_info
