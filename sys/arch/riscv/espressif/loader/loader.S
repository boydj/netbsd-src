/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * esp32s31-loader: get a NetBSD kernel started from Espressif's U-Boot.
 *
 * Espressif's ESP32-S31 BSP boots Linux with
 *
 *	booti 0x40400000 - 0x40200000
 *
 * i.e. a RISC-V "Image" executed in place from the flash XIP window, with
 * the device tree read from flash.  NetBSD can't run from flash, so this
 * position independent stub, prepended to the raw kernel, carries the
 * Image header and copies the kernel to PSRAM (KERNEL_LOAD), moves the
 * device tree just past the kernel's BSS, has OpenSBI synchronize the
 * (non-coherent, outside-of-the-hart) caches and jumps to the kernel with
 * the usual a0 = hartid, a1 = dtb.
 *
 * Runs in S-mode with the MMU off.  Build-time constants:
 *	KERNEL_LOAD	physical address the kernel was linked for
 *	KERNEL_SIZE	size of the raw kernel that follows this stub
 *	KERNEL_MEMSZ	size of the kernel including its BSS
 */

#define	SBI_EID_BASE			0x10
#define	SBI_FID_BASE_GETMVENDORID	4
#define	SBI_LEGACY_PUTCHAR		1
#define	S31_SBI_EXT_VENDOR_START	0x09000000
#define	S31_SBI_ICACHE_SYNC		3

	.option	norvc
	.text
	.globl	_start
_start:
	/* Linux RISC-V Image header (Documentation/riscv/boot-image-header) */
	j	entry			/* code0 */
	.word	0			/* code1 */
	.dword	0			/* text_offset */
	.dword	image_end - _start	/* image_size */
	.dword	0			/* flags: little endian */
	.word	0x00000002		/* version 0.2 */
	.word	0			/* res1 */
	.dword	0			/* res2 */
	.ascii	"RISCV\0\0\0"		/* magic (deprecated) */
	.ascii	"RSC\x05"		/* magic2 */
	.word	0			/* res3 */

entry:
	mv	s0, a0			/* hartid */
	mv	s1, a1			/* dtb */

	la	a0, banner
	call	puts

	/*
	 * 1. Move the device tree to just past the kernel's BSS.  Do this
	 *    first: U-Boot may have put it where the kernel is going.
	 */
	li	s2, KERNEL_LOAD + ((KERNEL_MEMSZ + 0xfff) & ~0xfff)
	lbu	t0, 4(s1)		/* fdt totalsize, big endian */
	lbu	t1, 5(s1)
	lbu	t2, 6(s1)
	lbu	t3, 7(s1)
	slli	t0, t0, 24
	slli	t1, t1, 16
	slli	t2, t2, 8
	or	t0, t0, t1
	or	t0, t0, t2
	or	s3, t0, t3		/* s3 = dtb size */
	mv	a0, s2
	mv	a1, s1
	mv	a2, s3
	call	memmove

	/* 2. Copy the kernel. */
	li	a0, KERNEL_LOAD
	la	a1, payload
	li	a2, KERNEL_SIZE
	call	memmove

	/*
	 * 3. Write back the D-cache and invalidate the I-cache.  On the
	 *    ESP32-S31 fence.i doesn't reach the caches; Espressif's OpenSBI
	 *    does it through a vendor extension.  Errors are ignored (other
	 *    SBIs, QEMU).
	 */
	li	a7, SBI_EID_BASE
	li	a6, SBI_FID_BASE_GETMVENDORID
	ecall
	bnez	a0, 1f
	li	t0, 0x00ffffff
	and	a1, a1, t0
	li	a7, S31_SBI_EXT_VENDOR_START
	add	a7, a7, a1
	li	a6, S31_SBI_ICACHE_SYNC
	li	a0, 0
	li	a1, 0
	ecall
1:
	fence	rw, rw
	fence.i

	la	a0, jumping
	call	puts

	/* 4. Off we go. */
	mv	a0, s0
	mv	a1, s2
	li	t0, KERNEL_LOAD
	jr	t0

/*
 * memmove(a0 = dst, a1 = src, a2 = len), overlap safe, byte at a time
 * unless both are word aligned.
 */
memmove:
	beqz	a2, 9f
	or	t0, a0, a1
	or	t0, t0, a2
	andi	t0, t0, 3
	bltu	a1, a0, 5f		/* src below dst: copy backwards */
	/* forwards */
	bnez	t0, 2f
1:	lw	t1, 0(a1)
	sw	t1, 0(a0)
	addi	a0, a0, 4
	addi	a1, a1, 4
	addi	a2, a2, -4
	bnez	a2, 1b
	ret
2:	lbu	t1, 0(a1)
	sb	t1, 0(a0)
	addi	a0, a0, 1
	addi	a1, a1, 1
	addi	a2, a2, -1
	bnez	a2, 2b
	ret
5:	/* backwards */
	add	a0, a0, a2
	add	a1, a1, a2
	bnez	t0, 7f
6:	addi	a0, a0, -4
	addi	a1, a1, -4
	lw	t1, 0(a1)
	sw	t1, 0(a0)
	addi	a2, a2, -4
	bnez	a2, 6b
	ret
7:	addi	a0, a0, -1
	addi	a1, a1, -1
	lbu	t1, 0(a1)
	sb	t1, 0(a0)
	addi	a2, a2, -1
	bnez	a2, 7b
9:	ret

/* puts(a0) via the legacy SBI console */
puts:
	mv	t2, a0
1:	lbu	a0, 0(t2)
	beqz	a0, 2f
	li	a7, SBI_LEGACY_PUTCHAR
	ecall
	addi	t2, t2, 1
	j	1b
2:	ret

banner:
	.asciz	"\r\nesp32s31-loader: copying NetBSD kernel to PSRAM\r\n"
jumping:
	.asciz	"esp32s31-loader: starting kernel\r\n"

	.balign	64
payload:
	.incbin	KERNEL_BIN
	.balign	16
image_end:
