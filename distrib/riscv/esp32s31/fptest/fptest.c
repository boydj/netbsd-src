/* $NetBSD$ */

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
 * fptest: check the single-precision FPU and that the kernel keeps every
 * process's FP registers (f0-f31, fcsr) across context switches and signal
 * handlers.  Built with F instructions under the soft-float ilp32 ABI, so
 * it links against the ordinary libc.
 */

#include <err.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>

#define	NPROC	4
#define	ROUNDS	200

#define	FREGS(op)							\
	op "\tf0, 0(%0)\n\t"	op "\tf1, 4(%0)\n\t"			\
	op "\tf2, 8(%0)\n\t"	op "\tf3, 12(%0)\n\t"			\
	op "\tf4, 16(%0)\n\t"	op "\tf5, 20(%0)\n\t"			\
	op "\tf6, 24(%0)\n\t"	op "\tf7, 28(%0)\n\t"			\
	op "\tf8, 32(%0)\n\t"	op "\tf9, 36(%0)\n\t"			\
	op "\tf10, 40(%0)\n\t"	op "\tf11, 44(%0)\n\t"			\
	op "\tf12, 48(%0)\n\t"	op "\tf13, 52(%0)\n\t"			\
	op "\tf14, 56(%0)\n\t"	op "\tf15, 60(%0)\n\t"			\
	op "\tf16, 64(%0)\n\t"	op "\tf17, 68(%0)\n\t"			\
	op "\tf18, 72(%0)\n\t"	op "\tf19, 76(%0)\n\t"			\
	op "\tf20, 80(%0)\n\t"	op "\tf21, 84(%0)\n\t"			\
	op "\tf22, 88(%0)\n\t"	op "\tf23, 92(%0)\n\t"			\
	op "\tf24, 96(%0)\n\t"	op "\tf25, 100(%0)\n\t"			\
	op "\tf26, 104(%0)\n\t"	op "\tf27, 108(%0)\n\t"			\
	op "\tf28, 112(%0)\n\t"	op "\tf29, 116(%0)\n\t"			\
	op "\tf30, 120(%0)\n\t"	op "\tf31, 124(%0)"

#define	FCLOBBER							\
	"f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",			\
	"f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",		\
	"f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",		\
	"f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31"

/* Load a pattern derived from seed into f0-f31 and the rounding mode. */
static void
fill(uint32_t seed)
{
	uint32_t v[32];

	for (int i = 0; i < 32; i++)
		v[i] = 0x3f800000u + (seed << 12) + (uint32_t)i;
	__asm volatile(FREGS("flw") :: "r"(v) : "memory", FCLOBBER);
	__asm volatile("csrw\tfrm, %0" :: "r"(seed % 5));
}

/* Check f0-f31 and the rounding mode still hold the pattern for seed. */
static int
check(uint32_t seed)
{
	uint32_t v[32], frm;
	int bad = 0;

	__asm volatile(FREGS("fsw") :: "r"(v) : "memory");
	__asm volatile("csrr\t%0, frm" : "=r"(frm));
	for (int i = 0; i < 32; i++) {
		if (v[i] != 0x3f800000u + (seed << 12) + (uint32_t)i) {
			fprintf(stderr, "fptest: f%d = %#x, expected %#x\n",
			    i, v[i], 0x3f800000u + (seed << 12) + (uint32_t)i);
			bad++;
		}
	}
	if (frm != seed % 5) {
		fprintf(stderr, "fptest: frm = %u, expected %u\n", frm,
		    seed % 5);
		bad++;
	}
	return bad;
}

static void
sigill(int sig)
{
	static const char msg[] = "fptest: FP instruction trapped: no FPU\n";

	(void)sig;
	(void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(2);
}

static volatile sig_atomic_t handled;

static void
clobber(int sig)
{
	(void)sig;
	fill(99);			/* the handler's own FP use */
	handled = check(99) == 0 ? 1 : -1;
}

static int
arith(void)
{
	volatile float one = 1.0f, two = 2.0f, zero = 0.0f;
	uint32_t flags;
	float r;

	/* Basic arithmetic, in hardware (fsqrt.s, fdiv.s, fmadd.s). */
	r = __builtin_sqrtf(two);
	if (r * r < 1.9999f || r * r > 2.0001f) {
		fprintf(stderr, "fptest: sqrtf(2)^2 = %f\n", (double)r);
		return 1;
	}
	r = __builtin_fmaf(two, 3.0f, one);
	if (r != 7.0f) {
		fprintf(stderr, "fptest: fma(2, 3, 1) = %f\n", (double)r);
		return 1;
	}

	/* Exception flags: 1/0 raises divide-by-zero. */
	__asm volatile("csrw\tfflags, zero");
	r = one / zero;
	__asm volatile("csrr\t%0, fflags" : "=r"(flags));
	if ((flags & 0x08) == 0 || r <= 1e30f) {
		fprintf(stderr, "fptest: 1/0 = %f, fflags %#x\n", (double)r,
		    flags);
		return 1;
	}
	return 0;
}

int
main(void)
{
	int status, bad = 0;
	pid_t pid;

	signal(SIGILL, sigill);

	if (arith() != 0)
		errx(1, "arithmetic: FAILED");
	printf("fptest: arithmetic and exception flags ok\n");

	/* Several processes with different FP state, switching often. */
	for (int n = 0; n < NPROC; n++) {
		if ((pid = fork()) == -1)
			err(1, "fork");
		if (pid == 0) {
			fill((uint32_t)n + 1);
			for (int r = 0; r < ROUNDS; r++) {
				sched_yield();
				if (r % 50 == 0)
					usleep(1000);
				if (check((uint32_t)n + 1) != 0)
					_exit(1);
			}
			_exit(0);
		}
	}
	fill(42);
	for (int n = 0; n < NPROC; n++) {
		if (wait(&status) == -1)
			err(1, "wait");
		if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
			bad++;
	}
	bad += check(42) != 0;
	if (bad)
		errx(1, "context switch: FAILED");
	printf("fptest: FP state kept across %d processes x %d switches\n",
	    NPROC + 1, ROUNDS);

	/* A signal handler that uses the FPU must not disturb us. */
	signal(SIGUSR1, clobber);
	fill(7);
	raise(SIGUSR1);
	if (handled != 1 || check(7) != 0)
		errx(1, "signal handler: FAILED");
	printf("fptest: FP state restored after a signal handler\n");

	printf("fptest: ok\n");
	return 0;
}
