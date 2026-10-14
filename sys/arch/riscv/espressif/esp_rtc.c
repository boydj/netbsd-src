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
 * ESP32-S31 RTC timer as a time-of-day clock (todr(9)).
 *
 * The LP domain's 48-bit RTC timer counts the slow clock (the ~136 kHz RC
 * oscillator, or a 32 kHz crystal) from power-up, through resets of the
 * HP side (reboot, watchdog) and deep sleep; there's no battery, so a
 * power cycle starts it again.  When the time is set, the counter value
 * and the time go into LP store registers that nothing else on the chip
 * uses (LP_SYS STORE10-15); the time is then that plus the ticks since,
 * at the slow clock's rate, measured against the XTAL at attach with the
 * calibration engine of timer group 0 (ESP-IDF's rtc_clk_cal()).  The RC
 * oscillator is only good to a few percent: the time is kept in sync
 * with NTP, but it's what makes a reboot come up at about the right time.
 *
 *	rtc@20800000 {
 *		compatible = "espressif,esp32s31-rtc-timer";
 *		reg = <0x20800000 0x100>,	(RTC timer)
 *		      <0x20700054 0x18>,	(LP_SYS STORE10..15)
 *		      <0x20701000 0x4>,		(LP_AON ROOT_CLK_CONF)
 *		      <0x20580000 0x100>,	(TIMG0, calibration)
 *		      <0x20587114 0x8>;		(HP_SYS_CLKRST TIMG0, TGRT)
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <dev/clock_subr.h>
#include <dev/fdt/fdtvar.h>

/* RTC timer */
#define	RTC_UPDATE		0x10
#define	 RTC_MAIN_TIMER_UPDATE	__BIT(27)
#define	RTC_MAIN_BUF0_LOW	0x14
#define	RTC_MAIN_BUF0_HIGH	0x18

/* LP store registers, from STORE10 */
#define	ST_TICKS_LO		0x00
#define	ST_TICKS_HI		0x04
#define	ST_SEC_LO		0x08
#define	ST_SEC_HI		0x0c
#define	ST_USEC			0x10
#define	ST_MAGIC		0x14
#define	 ESP_RTC_MAGIC		0x4e425344	/* "NBSD" */

/* LP_AON ROOT_CLK_CONF */
#define	SLOW_CLK_SEL		__BITS(25, 23)
#define	 SLOW_CLK_RC_SLOW	0
#define	 SLOW_CLK_XTAL32K	1

/* TIMG0 RTC calibration */
#define	TIMG_RTCCALICFG		0x68
#define	 RTCCALICFG_START_CYCLING	__BIT(12)
#define	 RTCCALICFG_RDY		__BIT(15)
#define	 RTCCALICFG_MAX		__BITS(30, 16)
#define	 RTCCALICFG_START	__BIT(31)
#define	TIMG_RTCCALICFG1	0x6c
#define	 RTCCALICFG1_VALUE	__BITS(31, 7)
#define	TIMG_RTCCALICFG2	0x80
#define	 RTCCALICFG2_TIMEOUT	__BIT(0)
#define	 RTCCALICFG2_THRES	__BITS(31, 7)

/* HP_SYS_CLKRST, from TIMERGRP0_CTRL0 */
#define	CLK_TIMG0		0x00
#define	 TIMG0_APB_CLK_EN	__BIT(0)
#define	CLK_TGRT		0x04
#define	 TGRT_CLK_EN		__BIT(0)
#define	 TGRT_CLK_SRC_SEL	__BITS(4, 1)
#define	  TGRT_SRC_RC_SLOW	8
#define	  TGRT_SRC_XTAL32K	10
#define	 TGRT_CLK_DIV_NUM	__BITS(20, 5)

#define	ESP_RTC_CAL_CYCLES	1024
#define	ESP_RTC_XTAL_MHZ	40
#define	ESP_RTC_FRACT		19	/* cal: us per tick, Q13.19 */

enum { R_TIMER, R_STORE, R_AON, R_TIMG0, R_CLK, NREGS };

struct esp_rtc_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh[NREGS];
	uint32_t		sc_cal;		/* us per tick, Q13.19 */
	struct todr_chip_handle	sc_todr;
};

#define	RD(sc, m, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh[m], (r))
#define	WR(sc, m, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh[m], (r), (v))

static int	esp_rtc_match(device_t, cfdata_t, void *);
static void	esp_rtc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_rtc, sizeof(struct esp_rtc_softc),
	esp_rtc_match, esp_rtc_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-rtc-timer" },
	DEVICE_COMPAT_EOL
};

static uint64_t
esp_rtc_ticks(struct esp_rtc_softc *sc)
{
	uint32_t lo, hi;

	WR(sc, R_TIMER, RTC_UPDATE,
	    RD(sc, R_TIMER, RTC_UPDATE) | RTC_MAIN_TIMER_UPDATE);
	DELAY(1);
	lo = RD(sc, R_TIMER, RTC_MAIN_BUF0_LOW);
	hi = RD(sc, R_TIMER, RTC_MAIN_BUF0_HIGH) & 0xffff;
	return ((uint64_t)hi << 32) | lo;
}

/* Ticks to microseconds, as esp_rtc_get_time_us(). */
static uint64_t
esp_rtc_ticks_to_us(struct esp_rtc_softc *sc, uint64_t ticks)
{
	const uint64_t cal = sc->sc_cal;

	return (((ticks & 0xffffffff) * cal) >> ESP_RTC_FRACT) +
	    (((ticks >> 32) * cal) << (32 - ESP_RTC_FRACT));
}

/*
 * Measure the slow clock against the XTAL: count XTAL cycles over
 * ESP_RTC_CAL_CYCLES slow clock cycles (rtc_clk_cal_internal()).
 */
static uint32_t
esp_rtc_calibrate(struct esp_rtc_softc *sc, bool xtal32k)
{
	const u_int n = ESP_RTC_CAL_CYCLES;
	uint32_t cfg, tgrt, x;
	u_int guard;

	WR(sc, R_CLK, CLK_TIMG0, RD(sc, R_CLK, CLK_TIMG0) | TIMG0_APB_CLK_EN);
	tgrt = RD(sc, R_CLK, CLK_TGRT);
	WR(sc, R_CLK, CLK_TGRT, (tgrt & ~(TGRT_CLK_SRC_SEL | TGRT_CLK_DIV_NUM)) |
	    TGRT_CLK_EN | __SHIFTIN(xtal32k ? TGRT_SRC_XTAL32K :
	    TGRT_SRC_RC_SLOW, TGRT_CLK_SRC_SEL));

	cfg = RD(sc, R_TIMG0, TIMG_RTCCALICFG);
	if (cfg & RTCCALICFG_START_CYCLING) {
		WR(sc, R_TIMG0, TIMG_RTCCALICFG2,
		    __SHIFTIN(1, RTCCALICFG2_THRES));
		for (guard = 10000; guard > 0 &&
		    (RD(sc, R_TIMG0, TIMG_RTCCALICFG) & RTCCALICFG_RDY) == 0 &&
		    (RD(sc, R_TIMG0, TIMG_RTCCALICFG2) &
		     RTCCALICFG2_TIMEOUT) == 0; guard--)
			DELAY(1);
	}
	cfg &= ~(RTCCALICFG_START_CYCLING | RTCCALICFG_MAX | RTCCALICFG_START);
	cfg |= __SHIFTIN(n, RTCCALICFG_MAX);
	WR(sc, R_TIMG0, TIMG_RTCCALICFG, cfg);
	WR(sc, R_TIMG0, TIMG_RTCCALICFG2,
	    __SHIFTIN(xtal32k ? n << 12 : n << 10, RTCCALICFG2_THRES));
	WR(sc, R_TIMG0, TIMG_RTCCALICFG, cfg | RTCCALICFG_START);

	/* 1024 cycles of a 136 kHz clock: 7.5 ms; give it 100. */
	for (guard = 100000; guard > 0; guard--) {
		if (RD(sc, R_TIMG0, TIMG_RTCCALICFG) & RTCCALICFG_RDY)
			break;
		if (RD(sc, R_TIMG0, TIMG_RTCCALICFG2) & RTCCALICFG2_TIMEOUT) {
			guard = 0;
			break;
		}
		DELAY(1);
	}
	x = __SHIFTOUT(RD(sc, R_TIMG0, TIMG_RTCCALICFG1), RTCCALICFG1_VALUE);
	WR(sc, R_TIMG0, TIMG_RTCCALICFG, cfg);
	WR(sc, R_CLK, CLK_TGRT, tgrt);
	if (guard == 0 || x == 0)
		return 0;

	return (((uint64_t)x << ESP_RTC_FRACT) + (ESP_RTC_XTAL_MHZ * n) / 2 - 1) /
	    (ESP_RTC_XTAL_MHZ * n);
}

static int
esp_rtc_gettime(todr_chip_handle_t tch, struct timeval *tv)
{
	struct esp_rtc_softc * const sc = device_private(tch->todr_dev);
	uint64_t ref, now, us, sec;

	if (RD(sc, R_STORE, ST_MAGIC) != ESP_RTC_MAGIC)
		return EINVAL;		/* never set since power-up */
	ref = ((uint64_t)RD(sc, R_STORE, ST_TICKS_HI) << 32) |
	    RD(sc, R_STORE, ST_TICKS_LO);
	now = esp_rtc_ticks(sc);
	if (now < ref)
		return EINVAL;
	us = esp_rtc_ticks_to_us(sc, now - ref) + RD(sc, R_STORE, ST_USEC);
	sec = ((uint64_t)RD(sc, R_STORE, ST_SEC_HI) << 32) |
	    RD(sc, R_STORE, ST_SEC_LO);
	tv->tv_sec = sec + us / 1000000;
	tv->tv_usec = us % 1000000;
	return 0;
}

static int
esp_rtc_settime(todr_chip_handle_t tch, struct timeval *tv)
{
	struct esp_rtc_softc * const sc = device_private(tch->todr_dev);
	const uint64_t now = esp_rtc_ticks(sc);
	const uint64_t sec = tv->tv_sec;

	WR(sc, R_STORE, ST_MAGIC, 0);
	WR(sc, R_STORE, ST_TICKS_LO, (uint32_t)now);
	WR(sc, R_STORE, ST_TICKS_HI, (uint32_t)(now >> 32));
	WR(sc, R_STORE, ST_SEC_LO, (uint32_t)sec);
	WR(sc, R_STORE, ST_SEC_HI, (uint32_t)(sec >> 32));
	WR(sc, R_STORE, ST_USEC, tv->tv_usec);
	WR(sc, R_STORE, ST_MAGIC, ESP_RTC_MAGIC);
	return 0;
}

static int
esp_rtc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_rtc_attach(device_t parent, device_t self, void *aux)
{
	struct esp_rtc_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	uint64_t t0, t1;
	bool xtal32k;
	u_int freq;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	for (u_int i = 0; i < NREGS; i++) {
		if (fdtbus_get_reg(phandle, i, &addr, &size) != 0 ||
		    bus_space_map(sc->sc_bst, addr, size, 0,
			&sc->sc_bsh[i]) != 0) {
			aprint_error(": couldn't map registers\n");
			return;
		}
	}

	t0 = esp_rtc_ticks(sc);
	DELAY(1000);
	t1 = esp_rtc_ticks(sc);
	if (t1 == t0) {
		aprint_naive("\n");
		aprint_normal(": RTC timer not running\n");
		return;
	}

	xtal32k = __SHIFTOUT(RD(sc, R_AON, 0), SLOW_CLK_SEL) ==
	    SLOW_CLK_XTAL32K;
	sc->sc_cal = esp_rtc_calibrate(sc, xtal32k);
	if (sc->sc_cal == 0) {
		/* Nominal: 136 kHz RC, or 32768 Hz. */
		sc->sc_cal = xtal32k ? 16000000 :
		    ((1000000ULL << ESP_RTC_FRACT) / 136000);
	}
	freq = (1000000ULL << ESP_RTC_FRACT) / sc->sc_cal;

	aprint_naive("\n");
	aprint_normal(": RTC timer, %s at %u Hz, %s\n",
	    xtal32k ? "32 kHz crystal" : "RC oscillator", freq,
	    RD(sc, R_STORE, ST_MAGIC) == ESP_RTC_MAGIC ?
	    "time kept since the last reset" : "time not set");

	sc->sc_todr.todr_dev = self;
	sc->sc_todr.todr_gettime = esp_rtc_gettime;
	sc->sc_todr.todr_settime = esp_rtc_settime;
	todr_attach(&sc->sc_todr);
}
