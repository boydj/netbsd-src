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
 * Espressif ESP32-S31 SYSTIMER: timecounter, hardclock and delay(9).
 *
 * The RISC-V time CSR is not usable on this chip: S-mode reads trap to
 * OpenSBI, and the CLINT MTIME behind it is CPU-clocked and stops in wfi.
 * The SYSTIMER is an always-on 52-bit counter clocked from the crystal
 * (16MHz), which keeps running in wfi.  OpenSBI has already enabled its
 * clock.  We own UNIT0 and the TARGET0 alarm (interrupt matrix source 33).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/timetc.h>

#include <dev/fdt/fdtvar.h>

#include <machine/machdep.h>
#include <machine/sysreg.h>

#include <riscv/espressif/esp32s31_reg.h>
#include <riscv/espressif/esp32s31_var.h>

/* Minimum distance between "now" and a newly programmed alarm. */
#define	SYSTIMER_MIN_DELTA	32

struct esp_systimer {
	bus_space_tag_t		st_bst;
	bus_space_handle_t	st_bsh;
	uint32_t		st_freq;
	uint32_t		st_ticks_per_hz;
	uint32_t		st_ticks_per_usec;
	uint64_t		st_next;	/* next alarm */
	struct evcnt		st_ev_missed;
	void			*st_ih;
	device_t		st_dev;
	int			st_phandle;
};

static struct esp_systimer esp_systimer;

#define	ST_READ(st, reg)	\
	bus_space_read_4((st)->st_bst, (st)->st_bsh, (reg))
#define	ST_WRITE(st, reg, val)	\
	bus_space_write_4((st)->st_bst, (st)->st_bsh, (reg), (val))

/*
 * Read the 52-bit counter.  The snapshot latch (UNIT0_OP) is shared, so
 * keep interrupts off while using it.
 */
static uint64_t
esp_systimer_count(struct esp_systimer *st)
{
	const register_t sstatus = csr_sstatus_read();
	uint32_t hi, lo;
	u_int guard = 100000;

	csr_sstatus_clear(SR_SIE);
	ST_WRITE(st, SYSTIMER_UNIT0_OP,
	    ST_READ(st, SYSTIMER_UNIT0_OP) | SYSTIMER_UNIT0_UPDATE);
	while ((ST_READ(st, SYSTIMER_UNIT0_OP) & SYSTIMER_UNIT0_VALID) == 0 &&
	    --guard != 0)
		continue;
	hi = ST_READ(st, SYSTIMER_UNIT0_VALUE_HI) & SYSTIMER_HI_MASK;
	lo = ST_READ(st, SYSTIMER_UNIT0_VALUE_LO);
	if (sstatus & SR_SIE)
		csr_sstatus_set(SR_SIE);

	return ((uint64_t)hi << 32) | lo;
}

static void
esp_systimer_delay(unsigned long us)
{
	struct esp_systimer * const st = &esp_systimer;
	const uint64_t finish = esp_systimer_count(st) +
	    (uint64_t)us * st->st_ticks_per_usec + 1;

	while (esp_systimer_count(st) < finish)
		continue;
}

static u_int
esp_systimer_get_timecount(struct timecounter *tc)
{
	return (u_int)esp_systimer_count(&esp_systimer);
}

static struct timecounter esp_systimer_tc = {
	.tc_get_timecount = esp_systimer_get_timecount,
	.tc_counter_mask = ~0u,
	.tc_frequency = SYSTIMER_FREQ,
	.tc_name = "esp32s31-systimer",
	.tc_quality = 500,
};

static void
esp_systimer_arm(struct esp_systimer *st, uint64_t target)
{
	ST_WRITE(st, SYSTIMER_CONF,
	    ST_READ(st, SYSTIMER_CONF) & ~SYSTIMER_CONF_TARGET0_WORK_EN);
	ST_WRITE(st, SYSTIMER_TARGET0_HI,
	    (uint32_t)(target >> 32) & SYSTIMER_HI_MASK);
	ST_WRITE(st, SYSTIMER_TARGET0_LO, (uint32_t)target);
	ST_WRITE(st, SYSTIMER_TARGET0_CONF,
	    ST_READ(st, SYSTIMER_TARGET0_CONF) &
	    ~(SYSTIMER_TARGET0_UNIT_SEL | SYSTIMER_TARGET0_PERIOD_MODE));
	ST_WRITE(st, SYSTIMER_COMP0_LOAD, 1);
	ST_WRITE(st, SYSTIMER_CONF,
	    ST_READ(st, SYSTIMER_CONF) | SYSTIMER_CONF_TARGET0_WORK_EN);
}

/*
 * Schedule the next tick, one period after the previous one.  Ticks that
 * are already in the past are counted as missed and skipped.
 */
static void
esp_systimer_schedule(struct esp_systimer *st)
{
	const uint64_t now = esp_systimer_count(st);

	st->st_next += st->st_ticks_per_hz;
	while (st->st_next < now + SYSTIMER_MIN_DELTA) {
		st->st_ev_missed.ev_count++;
		st->st_next += st->st_ticks_per_hz;
	}
	esp_systimer_arm(st, st->st_next);
}

static int
esp_systimer_intr(void *arg)
{
	struct esp_systimer * const st = &esp_systimer;
	struct clockframe * const cf = arg;
	struct cpu_info * const ci = curcpu();

	/* One-shot: disarm before clearing so the level can't re-assert. */
	ST_WRITE(st, SYSTIMER_CONF,
	    ST_READ(st, SYSTIMER_CONF) & ~SYSTIMER_CONF_TARGET0_WORK_EN);
	ST_WRITE(st, SYSTIMER_INT_CLR, SYSTIMER_INT_TARGET0);

	ci->ci_ev_timer.ev_count++;
	esp_systimer_schedule(st);

	hardclock(cf);

	return 1;
}

/*
 * Called from cpu_initclocks() via riscv_timer_register().
 */
static void
esp_systimer_initclocks(void)
{
	struct esp_systimer * const st = &esp_systimer;
	struct cpu_info * const ci = curcpu();

	st->st_ticks_per_hz = st->st_freq / hz;

	evcnt_attach_dynamic(&ci->ci_ev_timer, EVCNT_TYPE_INTR,
	    NULL, device_xname(ci->ci_dev), "timer");
	evcnt_attach_dynamic(&st->st_ev_missed, EVCNT_TYPE_MISC,
	    NULL, device_xname(st->st_dev), "missed ticks");

	st->st_ih = fdtbus_intr_establish_xname(st->st_phandle, 0, IPL_SCHED,
	    FDT_INTR_MPSAFE, esp_systimer_intr, NULL, "clock");
	if (st->st_ih == NULL)
		panic("%s: couldn't establish timer interrupt", __func__);

	ST_WRITE(st, SYSTIMER_INT_CLR, SYSTIMER_INT_TARGET0);
	ST_WRITE(st, SYSTIMER_INT_ENA,
	    ST_READ(st, SYSTIMER_INT_ENA) | SYSTIMER_INT_TARGET0);

	st->st_next = esp_systimer_count(st);
	esp_systimer_schedule(st);

	esp_systimer_tc.tc_frequency = st->st_freq;
	tc_init(&esp_systimer_tc);
}

/*
 * Make delay(9) work before autoconfiguration, using the devmap'ed
 * registers.
 */
void
esp_systimer_early_init(bus_space_tag_t bst, bus_addr_t addr)
{
	struct esp_systimer * const st = &esp_systimer;

	st->st_bst = bst;
	if (bus_space_map(bst, addr, 0x100, 0, &st->st_bsh) != 0)
		return;
	st->st_freq = SYSTIMER_FREQ;
	st->st_ticks_per_usec = st->st_freq / 1000000;

	/* UNIT0 free-running (OpenSBI clocked and un-stalled it). */
	ST_WRITE(st, SYSTIMER_CONF,
	    ST_READ(st, SYSTIMER_CONF) | SYSTIMER_CONF_UNIT0_WORK_EN);

	riscv_delay_register(esp_systimer_delay);
}

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-systimer" },
	DEVICE_COMPAT_EOL
};

static int
esp_systimer_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_systimer_attach(device_t parent, device_t self, void *aux)
{
	struct esp_systimer * const st = &esp_systimer;
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	uint32_t freq;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	st->st_dev = self;
	st->st_phandle = phandle;
	st->st_bst = faa->faa_bst;
	if (bus_space_map(st->st_bst, addr, size, 0, &st->st_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (of_getprop_uint32(phandle, "clock-frequency", &freq) != 0)
		freq = SYSTIMER_FREQ;
	st->st_freq = freq;
	st->st_ticks_per_usec = freq / 1000000;

	ST_WRITE(st, SYSTIMER_CONF,
	    ST_READ(st, SYSTIMER_CONF) | SYSTIMER_CONF_UNIT0_WORK_EN);

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr)))
		strlcpy(intrstr, "?", sizeof(intrstr));

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 system timer, %u.%03u MHz, %s\n",
	    freq / 1000000, (freq / 1000) % 1000, intrstr);

	riscv_delay_register(esp_systimer_delay);
	riscv_timer_register(esp_systimer_initclocks);
}

CFATTACH_DECL_NEW(esp_systimer, 0,
	esp_systimer_match, esp_systimer_attach, NULL, NULL);
