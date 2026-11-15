/* SPDX-License-Identifier: GPL-2.0 */
/*
 * 64 bit division helpers of the ARM EABI, used by the vendor modules but
 * not exported by the current kernel.
 * Arguments: n in r0:r1, d in r2:r3. Result: q in r0:r1, r in r2:r3.
 */
#include <linux/linkage.h>

	.macro	divmod, name, helper
ENTRY(\name)
	push	{r4, lr}
	sub	sp, sp, #16
	add	r4, sp, #8
	str	r4, [sp]		@ remainder pointer, third argument
	bl	\helper
	ldrd	r2, r3, [sp, #8]
	add	sp, sp, #16
	pop	{r4, pc}
ENDPROC(\name)
	.endm

	divmod	osm___aeabi_uldivmod, div64_u64_rem
	divmod	osm___aeabi_ldivmod, osm_div64_s64_rem
