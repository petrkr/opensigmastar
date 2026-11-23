/* SPDX-License-Identifier: GPL-2.0 */
/*
 * 32-bit ARM EABI unsigned division used by vendor modules.  The vendor
 * kernel patches __aeabi_uidiv to the hardware udiv instruction
 * (ARM_PATCH_IDIV), so a zero denominator silently returns 0.  Do the same:
 * some vendor code divides by zero at stream start (see README).
 */
#include <linux/linkage.h>

	.arch_extension idiv

ENTRY(osm___aeabi_uidiv)
	udiv	r0, r0, r1
	bx	lr
ENDPROC(osm___aeabi_uidiv)
