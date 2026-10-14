/*	$NetBSD: fpu.c,v 1.3 2023/05/07 12:41:49 skrll Exp $	*/

/*-
 * Copyright (c) 2014 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: fpu.c,v 1.3 2023/05/07 12:41:49 skrll Exp $");

#include "opt_multiprocessor.h"

#include <sys/param.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/cpu.h>
#include <sys/proc.h>
#include <sys/lwp.h>
#include <sys/pcu.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <riscv/locore.h>
#include <riscv/userret.h>
#include <riscv/frame.h>

/*
 * Width of the FP registers: 64 with the D extension (the default), 32 with
 * F only (e.g. the ESP32-S31), 0 without an FPU.  Set from the boot hart's
 * "riscv,isa" by riscv_fdt_cpu_bootstrap().
 */
u_int riscv_fpu_flen = 64;

/*
 * Lazy switching relies on FP instructions trapping while sstatus.FS is
 * Off.  If they don't (seen in an emulator), switch eagerly instead: give
 * every lwp its own FP state whenever it returns to user mode.
 */
bool riscv_fpu_eager;
static volatile bool fpu_probe_trapped;
extern const char fpu_probe_insn[];

static void fpu_state_save(lwp_t *);
static void fpu_state_load(lwp_t *, u_int);
static void fpu_state_release(lwp_t *);

const pcu_ops_t pcu_fpu_ops = {
	.pcu_id = PCU_FPU,
	.pcu_state_save = fpu_state_save,
	.pcu_state_load = fpu_state_load,
	.pcu_state_release = fpu_state_release
};

static void
fpu_disable(void)
{
	csr_sstatus_clear(SR_FS);
}

static void
fpu_enable(void)
{
	csr_sstatus_clear(SR_FS);
	csr_sstatus_set(__SHIFTIN(SR_FS_CLEAN, SR_FS));
}

void
fpu_state_save(lwp_t *l)
{
//	struct trapframe * const tf = l->l_md.md_utf;
	struct pcb * const pcb = lwp_getpcb(l);
	struct fpreg * const fp = &pcb->pcb_fpregs;

	KASSERT(l->l_pcu_cpu[PCU_FPU] == curcpu());

	curcpu()->ci_ev_fpu_saves.ev_count++;

	// Enable FPU to save FP state
	fpu_enable();

	// Save FCSR
	fp->r_fcsr = fcsr_read();

	// Save FP register values: 32 or 64 bits wide (each in a 64-bit slot).
	if (riscv_fpu_flen == 32) {
		__asm(".option push\n\t.option arch, +f\n\t"
		    "fsw\tf0, (0*%1)(%0)\n\t"
		    "fsw\tf1, (1*%1)(%0)\n\t"
		    "fsw\tf2, (2*%1)(%0)\n\t"
		    "fsw\tf3, (3*%1)(%0)\n\t"
		    "fsw\tf4, (4*%1)(%0)\n\t"
		    "fsw\tf5, (5*%1)(%0)\n\t"
		    "fsw\tf6, (6*%1)(%0)\n\t"
		    "fsw\tf7, (7*%1)(%0)\n\t"
		    "fsw\tf8, (8*%1)(%0)\n\t"
		    "fsw\tf9, (9*%1)(%0)\n\t"
		    "fsw\tf10, (10*%1)(%0)\n\t"
		    "fsw\tf11, (11*%1)(%0)\n\t"
		    "fsw\tf12, (12*%1)(%0)\n\t"
		    "fsw\tf13, (13*%1)(%0)\n\t"
		    "fsw\tf14, (14*%1)(%0)\n\t"
		    "fsw\tf15, (15*%1)(%0)\n\t"
		    "fsw\tf16, (16*%1)(%0)\n\t"
		    "fsw\tf17, (17*%1)(%0)\n\t"
		    "fsw\tf18, (18*%1)(%0)\n\t"
		    "fsw\tf19, (19*%1)(%0)\n\t"
		    "fsw\tf20, (20*%1)(%0)\n\t"
		    "fsw\tf21, (21*%1)(%0)\n\t"
		    "fsw\tf22, (22*%1)(%0)\n\t"
		    "fsw\tf23, (23*%1)(%0)\n\t"
		    "fsw\tf24, (24*%1)(%0)\n\t"
		    "fsw\tf25, (25*%1)(%0)\n\t"
		    "fsw\tf26, (26*%1)(%0)\n\t"
		    "fsw\tf27, (27*%1)(%0)\n\t"
		    "fsw\tf28, (28*%1)(%0)\n\t"
		    "fsw\tf29, (29*%1)(%0)\n\t"
		    "fsw\tf30, (30*%1)(%0)\n\t"
		    "fsw\tf31, (31*%1)(%0)\n\t"
		    ".option pop"
		    ::	"r"(fp->r_fpreg),
			"i"(sizeof(fp->r_fpreg[0])) : "memory");
	} else {
		__asm(".option push\n\t.option arch, +d\n\t"
		    "fsd\tf0, (0*%1)(%0)\n\t"
		    "fsd\tf1, (1*%1)(%0)\n\t"
		    "fsd\tf2, (2*%1)(%0)\n\t"
		    "fsd\tf3, (3*%1)(%0)\n\t"
		    "fsd\tf4, (4*%1)(%0)\n\t"
		    "fsd\tf5, (5*%1)(%0)\n\t"
		    "fsd\tf6, (6*%1)(%0)\n\t"
		    "fsd\tf7, (7*%1)(%0)\n\t"
		    "fsd\tf8, (8*%1)(%0)\n\t"
		    "fsd\tf9, (9*%1)(%0)\n\t"
		    "fsd\tf10, (10*%1)(%0)\n\t"
		    "fsd\tf11, (11*%1)(%0)\n\t"
		    "fsd\tf12, (12*%1)(%0)\n\t"
		    "fsd\tf13, (13*%1)(%0)\n\t"
		    "fsd\tf14, (14*%1)(%0)\n\t"
		    "fsd\tf15, (15*%1)(%0)\n\t"
		    "fsd\tf16, (16*%1)(%0)\n\t"
		    "fsd\tf17, (17*%1)(%0)\n\t"
		    "fsd\tf18, (18*%1)(%0)\n\t"
		    "fsd\tf19, (19*%1)(%0)\n\t"
		    "fsd\tf20, (20*%1)(%0)\n\t"
		    "fsd\tf21, (21*%1)(%0)\n\t"
		    "fsd\tf22, (22*%1)(%0)\n\t"
		    "fsd\tf23, (23*%1)(%0)\n\t"
		    "fsd\tf24, (24*%1)(%0)\n\t"
		    "fsd\tf25, (25*%1)(%0)\n\t"
		    "fsd\tf26, (26*%1)(%0)\n\t"
		    "fsd\tf27, (27*%1)(%0)\n\t"
		    "fsd\tf28, (28*%1)(%0)\n\t"
		    "fsd\tf29, (29*%1)(%0)\n\t"
		    "fsd\tf30, (30*%1)(%0)\n\t"
		    "fsd\tf31, (31*%1)(%0)\n\t"
		    ".option pop"
		    ::	"r"(fp->r_fpreg),
			"i"(sizeof(fp->r_fpreg[0])) : "memory");
	}

	// Disable the FPU
	fpu_disable();
}

void
fpu_state_load(lwp_t *l, u_int flags)
{
	struct trapframe * const tf = l->l_md.md_utf;
	struct pcb * const pcb = lwp_getpcb(l);
	struct fpreg * const fp = &pcb->pcb_fpregs;

	KASSERT(l == curlwp);
//	KASSERT(l->l_pcu_cpu[PCU_FPU] == curcpu());

	// If this is the first time the state is being loaded, zero it first.
	if (__predict_false((flags & PCU_VALID) == 0)) {
		memset(fp, 0, sizeof(*fp));
	}

	// Enable the FP when this lwp return to userspace.
	tf->tf_sr &= ~SR_FS;
	tf->tf_sr |= __SHIFTIN(SR_FS_CLEAN, SR_FS);

	// If this is a simple reenable, set the FPU enable flag and return
	if (flags & PCU_REENABLE) {
		curcpu()->ci_ev_fpu_reenables.ev_count++;
		return;
	}

	curcpu()->ci_ev_fpu_loads.ev_count++;


	// Enabling to load FP state.  Interrupts will remain on.
	fpu_enable();

	// load FP registers and establish processes' FP context.
	if (riscv_fpu_flen == 32) {
		__asm(".option push\n\t.option arch, +f\n\t"
		    "flw\tf0, (0*%1)(%0)\n\t"
		    "flw\tf1, (1*%1)(%0)\n\t"
		    "flw\tf2, (2*%1)(%0)\n\t"
		    "flw\tf3, (3*%1)(%0)\n\t"
		    "flw\tf4, (4*%1)(%0)\n\t"
		    "flw\tf5, (5*%1)(%0)\n\t"
		    "flw\tf6, (6*%1)(%0)\n\t"
		    "flw\tf7, (7*%1)(%0)\n\t"
		    "flw\tf8, (8*%1)(%0)\n\t"
		    "flw\tf9, (9*%1)(%0)\n\t"
		    "flw\tf10, (10*%1)(%0)\n\t"
		    "flw\tf11, (11*%1)(%0)\n\t"
		    "flw\tf12, (12*%1)(%0)\n\t"
		    "flw\tf13, (13*%1)(%0)\n\t"
		    "flw\tf14, (14*%1)(%0)\n\t"
		    "flw\tf15, (15*%1)(%0)\n\t"
		    "flw\tf16, (16*%1)(%0)\n\t"
		    "flw\tf17, (17*%1)(%0)\n\t"
		    "flw\tf18, (18*%1)(%0)\n\t"
		    "flw\tf19, (19*%1)(%0)\n\t"
		    "flw\tf20, (20*%1)(%0)\n\t"
		    "flw\tf21, (21*%1)(%0)\n\t"
		    "flw\tf22, (22*%1)(%0)\n\t"
		    "flw\tf23, (23*%1)(%0)\n\t"
		    "flw\tf24, (24*%1)(%0)\n\t"
		    "flw\tf25, (25*%1)(%0)\n\t"
		    "flw\tf26, (26*%1)(%0)\n\t"
		    "flw\tf27, (27*%1)(%0)\n\t"
		    "flw\tf28, (28*%1)(%0)\n\t"
		    "flw\tf29, (29*%1)(%0)\n\t"
		    "flw\tf30, (30*%1)(%0)\n\t"
		    "flw\tf31, (31*%1)(%0)\n\t"
		    ".option pop"
		    ::	"r"(fp->r_fpreg),
			"i"(sizeof(fp->r_fpreg[0])) : "memory");
	} else {
		__asm(".option push\n\t.option arch, +d\n\t"
		    "fld\tf0, (0*%1)(%0)\n\t"
		    "fld\tf1, (1*%1)(%0)\n\t"
		    "fld\tf2, (2*%1)(%0)\n\t"
		    "fld\tf3, (3*%1)(%0)\n\t"
		    "fld\tf4, (4*%1)(%0)\n\t"
		    "fld\tf5, (5*%1)(%0)\n\t"
		    "fld\tf6, (6*%1)(%0)\n\t"
		    "fld\tf7, (7*%1)(%0)\n\t"
		    "fld\tf8, (8*%1)(%0)\n\t"
		    "fld\tf9, (9*%1)(%0)\n\t"
		    "fld\tf10, (10*%1)(%0)\n\t"
		    "fld\tf11, (11*%1)(%0)\n\t"
		    "fld\tf12, (12*%1)(%0)\n\t"
		    "fld\tf13, (13*%1)(%0)\n\t"
		    "fld\tf14, (14*%1)(%0)\n\t"
		    "fld\tf15, (15*%1)(%0)\n\t"
		    "fld\tf16, (16*%1)(%0)\n\t"
		    "fld\tf17, (17*%1)(%0)\n\t"
		    "fld\tf18, (18*%1)(%0)\n\t"
		    "fld\tf19, (19*%1)(%0)\n\t"
		    "fld\tf20, (20*%1)(%0)\n\t"
		    "fld\tf21, (21*%1)(%0)\n\t"
		    "fld\tf22, (22*%1)(%0)\n\t"
		    "fld\tf23, (23*%1)(%0)\n\t"
		    "fld\tf24, (24*%1)(%0)\n\t"
		    "fld\tf25, (25*%1)(%0)\n\t"
		    "fld\tf26, (26*%1)(%0)\n\t"
		    "fld\tf27, (27*%1)(%0)\n\t"
		    "fld\tf28, (28*%1)(%0)\n\t"
		    "fld\tf29, (29*%1)(%0)\n\t"
		    "fld\tf30, (30*%1)(%0)\n\t"
		    "fld\tf31, (31*%1)(%0)\n\t"
		    ".option pop"
		    ::	"r"(fp->r_fpreg),
			"i"(sizeof(fp->r_fpreg[0])) : "memory");
	}

	// load FPCSR and disable FPU again
	fcsr_write(fp->r_fcsr);

	fpu_disable();
}

void
fpu_state_release(lwp_t *l)
{
	l->l_md.md_utf->tf_sr &= ~SR_FS;
	fpu_disable();
}

/*
 * Called from trap_instruction() for illegal instructions in kernel mode:
 * step over the probe in fpu_attach().
 */
bool
fpu_probe_trap(struct trapframe *tf)
{
	if (tf->tf_pc != (register_t)fpu_probe_insn)
		return false;
	fpu_probe_trapped = true;
	tf->tf_pc += 4;
	return true;
}

/*
 * Report the FPU and find out whether FP instructions trap with the FPU
 * off: run one with sstatus.FS = Off.
 */
void __noinline
fpu_attach(device_t dv)
{
	if (riscv_fpu_flen == 0) {
		aprint_normal_dev(dv, "no FPU\n");
		return;
	}

	fpu_probe_trapped = false;
	fpu_disable();
	__asm volatile(
		".option push\n\t"
		".option arch, +f\n\t"
		".option norvc\n\t"
		".globl fpu_probe_insn\n"
	"fpu_probe_insn:\n\t"
		"fmv.x.w\tt0, f0\n\t"
		".option pop"
		::: "t0", "memory");
	riscv_fpu_eager = !fpu_probe_trapped;

	aprint_normal_dev(dv, "FPU: %s precision (%s), %s context switching\n",
	    riscv_fpu_flen == 32 ? "single" : "double",
	    riscv_fpu_flen == 32 ? "F" : "D",
	    riscv_fpu_eager ? "eager (FS=Off does not trap)" : "lazy");
}

/*
 * Eager switching, from userret(): make sure the FPU holds this lwp's
 * state before it runs in user mode.
 */
void
fpu_eager_load(lwp_t *l)
{
	if (curcpu()->ci_pcu_curlwp[PCU_FPU] != l ||
	    __SHIFTOUT(l->l_md.md_utf->tf_sr, SR_FS) == SR_FS_OFF)
		pcu_load(&pcu_fpu_ops);
}
