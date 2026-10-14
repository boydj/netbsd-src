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
 * ESP32-S31 PWM: the two LED controllers (espledc, 8 channels each) and
 * the four motor control PWM blocks (espmcpwm, 6 outputs each), as pwm(9)
 * controllers for device tree consumers ("pwms = <&ledc0 3 1000000 0>":
 * channel, period in ns, flags; 1 = inverted) and, through sysctl, for
 * userland:
 *
 *	sysctl -w hw.espledc0.ch0.pin=5		route channel 0 to GPIO5
 *	sysctl -w hw.espledc0.ch0.period=1000000	1 kHz (0: off)
 *	sysctl -w hw.espledc0.ch0.duty=250000	25%
 *	sysctl -w hw.espledc0.ch0.inverted=1	active low
 *	sysctl hw.espledc0.ch0.actual		what the hardware does
 *
 * Both count the 40 MHz XTAL.  LEDC: each channel runs off one of four
 * timers (shared by channels with the same period), a counter of up to 20
 * bits behind a fractional divider, so from 0.04 Hz to 20 MHz with the
 * resolution falling as the frequency rises.  MCPWM: output pairs (A, B)
 * share an operator and its timer, so the period; 16 bits behind an 8-bit
 * prescaler, so 2.4 Hz up.  Polarity is the GPIO matrix's output inversion;
 * a channel that's off drives its inactive level.
 *
 *	ledc0: pwm@20392000 {
 *		compatible = "espressif,esp32s31-ledc";
 *		reg = <0x20392000 0x800>;
 *		clocks = <&clkrst 21>;
 *		resets = <&clkrst 13>;
 *		#pwm-cells = <3>;
 *	};
 *	mcpwm0: pwm@20381000 {
 *		compatible = "espressif,esp32s31-mcpwm";
 *		reg = <0x20381000 0x1000>;
 *		clocks = <&clkrst 35>;
 *		resets = <&clkrst 26>;
 *		#pwm-cells = <3>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bitops.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_pwm.h>

#include <riscv/espressif/esp32s31_var.h>

/* LEDC */
#define	LEDC_BASE0		0x20392000
#define	LEDC_BASE1		0x2039c000
#define	LEDC_NCHAN		8
#define	LEDC_NTIMER		4
#define	LEDC_CH_CONF0(n)	(0x00 + 0x14 * (n))
#define	 CONF0_TIMER_SEL	__BITS(1, 0)
#define	 CONF0_SIG_OUT_EN	__BIT(2)
#define	 CONF0_IDLE_LV		__BIT(3)
#define	 CONF0_PARA_UP		__BIT(4)
#define	LEDC_CH_HPOINT(n)	(0x04 + 0x14 * (n))
#define	LEDC_CH_DUTY(n)		(0x08 + 0x14 * (n))	/* 4 fraction bits */
#define	LEDC_CH_CONF1(n)	(0x0c + 0x14 * (n))
#define	 CONF1_DUTY_START	__BIT(31)
#define	LEDC_TIMER_CONF(t)	(0xa0 + 8 * (t))
#define	 TIMER_DUTY_RES		__BITS(4, 0)
#define	 TIMER_CLK_DIV		__BITS(22, 5)		/* 10.8 fixed point */
#define	 TIMER_PAUSE		__BIT(23)
#define	 TIMER_RST		__BIT(24)
#define	 TIMER_PARA_UP		__BIT(26)
#define	LEDC_INT_ENA		0xc8
#define	LEDC_INT_CLR		0xcc
#define	LEDC_CH_GAMMA_CONF(n)	(0x100 + 4 * (n))
#define	 GAMMA_ENTRY_NUM	__BITS(4, 0)
#define	LEDC_CH_POWER_UP	0x174
#define	LEDC_TIMER_POWER_UP	0x178
#define	LEDC_GAMMA_RANGE(n, r)	(0x400 + 0x40 * (n) + 4 * (r))
#define	 GAMMA_FIXED		0x00200003	/* one step: a fixed duty */
#define	LEDC_MAX_RES		20
#define	LEDC_MAX_DIV		0x3ffff

/* MCPWM */
#define	MCPWM_BASE0		0x20381000
#define	MCPWM_NCHAN		6
#define	MCPWM_TIMER_CFG0(t)	(0x00 + 0x10 * (t))
#define	 CFG0_PRESCALE		__BITS(7, 0)
#define	 CFG0_PERIOD		__BITS(23, 8)
#define	 CFG0_PERIOD_UPMETHOD	__BITS(25, 24)		/* 1: at zero */
#define	MCPWM_TIMER_CFG1(t)	(0x04 + 0x10 * (t))
#define	 CFG1_START		__BITS(2, 0)
#define	  START_RUN		2
#define	 CFG1_MOD		__BITS(4, 3)
#define	  MOD_FREEZE		0
#define	  MOD_UP		1
#define	MCPWM_OPERATOR_TIMERSEL	0x34
#define	MCPWM_GEN_STMP_CFG(o)	(0x38 + 0x38 * (o))
#define	 STMP_A_UPMETHOD	__BITS(3, 0)		/* 1: at zero */
#define	 STMP_B_UPMETHOD	__BITS(7, 4)
#define	MCPWM_GEN_TSTMP(o, g)	(0x3c + 0x38 * (o) + 4 * (g))
#define	MCPWM_GEN_CFG0(o)	(0x44 + 0x38 * (o))
#define	MCPWM_GEN_FORCE(o)	(0x48 + 0x38 * (o))
#define	 FORCE_UPMETHOD		__BITS(5, 0)
#define	 FORCE_A_MODE		__BITS(7, 6)
#define	 FORCE_B_MODE		__BITS(9, 8)
#define	  FORCE_OFF		0
#define	  FORCE_LOW		1
#define	  FORCE_HIGH		2
#define	MCPWM_GEN_ACT(o, g)	(0x4c + 0x38 * (o) + 4 * (g))
#define	 ACT_A			0x12	/* high at zero, low at compare A */
#define	 ACT_B			0x42	/* high at zero, low at compare B */
#define	MCPWM_UPDATE_CFG	0x108
#define	 UPDATE_GLOBAL_UP_EN	__BIT(0)
#define	 UPDATE_GLOBAL_FORCE_UP	__BIT(1)
#define	 UPDATE_OP_UP_EN	(__BIT(2) | __BIT(4) | __BIT(6))
#define	MCPWM_INT_ENA		0x10c
#define	MCPWM_INT_CLR		0x118
#define	MCPWM_MAX_PERIOD	65535
#define	MCPWM_MAX_PRESCALE	256

#define	ESP_PWM_MAXCHAN		8

struct esp_pwm_softc;

struct esp_pwm_chan {
	struct pwm_controller	ch_pwm;
	struct esp_pwm_softc	*ch_sc;
	u_int			ch_index;
	struct pwm_config	ch_conf;
	bool			ch_enabled;	/* by pwm_enable() */
	bool			ch_running;	/* in hardware */
	int			ch_pin;		/* GPIO, or -1 */
	int			ch_timer;	/* LEDC: timer, or -1 */
	/* what the hardware does */
	uint32_t		ch_div;		/* LEDC divider, MCPWM prescale */
	uint32_t		ch_count;	/* counts a period */
};

struct esp_pwm_type {
	const char	*pt_prefix;	/* GPIO matrix signal names */
	u_int		pt_nchan;
	int		(*pt_init)(struct esp_pwm_softc *);
	int		(*pt_apply)(struct esp_pwm_softc *,
			    struct esp_pwm_chan *, bool);
};

struct esp_pwm_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	kmutex_t		sc_lock;
	const struct esp_pwm_type *sc_type;
	u_int			sc_unit;
	uint64_t		sc_rate;	/* counter clock, Hz */
	struct esp_pwm_chan	sc_chan[ESP_PWM_MAXCHAN];
	struct {			/* LEDC timers */
		u_int		refs;
		uint32_t	div;
		u_int		res;
	}			sc_ltimer[LEDC_NTIMER];
	struct sysctllog	*sc_sysctllog;
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static void
esp_pwm_rmw(struct esp_pwm_softc *sc, bus_size_t r, uint32_t clr,
    uint32_t set)
{
	WR(sc, r, (RD(sc, r) & ~clr) | set);
}

/*
 * LEDC
 */

static int
esp_ledc_init(struct esp_pwm_softc *sc)
{
	WR(sc, LEDC_INT_ENA, 0);
	WR(sc, LEDC_INT_CLR, 0xffffffff);
	for (u_int n = 0; n < LEDC_NCHAN; n++) {
		WR(sc, LEDC_CH_CONF0(n), CONF0_PARA_UP);	/* off, low */
		for (u_int r = 0; r < 16; r++)
			WR(sc, LEDC_GAMMA_RANGE(n, r), 0);
	}
	WR(sc, LEDC_CH_POWER_UP, 0);
	WR(sc, LEDC_TIMER_POWER_UP, 0);
	return 0;
}

/*
 * Divider (10.8) and resolution for a period: the most bits that get
 * within 500 ppm of it (the divider's rounding), else the closest.
 */
static int
esp_ledc_timing(struct esp_pwm_softc *sc, u_int period, uint32_t *divp,
    u_int *resp)
{
	/* counts of the source clock a period, times 256 */
	const uint64_t t256 = sc->sc_rate * period * 32 / 125000000;
	uint64_t besterr = UINT64_MAX;
	int res, maxres;

	if (t256 < 2 * 256)
		return EINVAL;			/* faster than src/2 */
	maxres = MIN(fls64(t256) - 1 - 8, LEDC_MAX_RES);
	for (res = maxres; res >= 1; res--) {
		const uint64_t div = (t256 + (1ULL << (res - 1))) >> res;
		const uint64_t got = div << res;
		const uint64_t err = got > t256 ? got - t256 : t256 - got;

		if (div < 0x100 || div > LEDC_MAX_DIV)
			continue;
		if (err < besterr) {
			besterr = err;
			*divp = div;
			*resp = res;
		}
		if (err * 2000 <= t256)
			break;
	}
	return besterr == UINT64_MAX ? EINVAL : 0;
}

static void
esp_ledc_release(struct esp_pwm_softc *sc, u_int t)
{
	if (--sc->sc_ltimer[t].refs == 0) {
		esp_pwm_rmw(sc, LEDC_TIMER_CONF(t), 0, TIMER_PAUSE);
		esp_pwm_rmw(sc, LEDC_TIMER_POWER_UP, __BIT(t), 0);
	}
}

static void
esp_ledc_off(struct esp_pwm_softc *sc, struct esp_pwm_chan *ch)
{
	const u_int n = ch->ch_index;

	esp_pwm_rmw(sc, LEDC_CH_CONF0(n), CONF0_SIG_OUT_EN | CONF0_IDLE_LV,
	    CONF0_PARA_UP);
	if (ch->ch_timer >= 0) {
		esp_ledc_release(sc, ch->ch_timer);
		ch->ch_timer = -1;
	}
	esp_pwm_rmw(sc, LEDC_CH_POWER_UP, __BIT(n), 0);
	ch->ch_running = false;
}

static int
esp_ledc_apply(struct esp_pwm_softc *sc, struct esp_pwm_chan *ch, bool on)
{
	const struct pwm_config * const conf = &ch->ch_conf;
	const u_int n = ch->ch_index;
	uint32_t div, duty;
	u_int res;
	int t, error;

	if (!on || conf->period == 0) {
		esp_ledc_off(sc, ch);
		return 0;
	}
	if ((error = esp_ledc_timing(sc, conf->period, &div, &res)) != 0)
		return error;

	/* A timer at this rate already, or a free one (ours, if alone). */
	if (ch->ch_timer >= 0 && sc->sc_ltimer[ch->ch_timer].div == div &&
	    sc->sc_ltimer[ch->ch_timer].res == res) {
		t = ch->ch_timer;
	} else {
		for (t = 0; t < LEDC_NTIMER; t++)
			if (sc->sc_ltimer[t].refs > 0 &&
			    sc->sc_ltimer[t].div == div &&
			    sc->sc_ltimer[t].res == res)
				break;
		if (t == LEDC_NTIMER && ch->ch_timer >= 0 &&
		    sc->sc_ltimer[ch->ch_timer].refs == 1)
			t = ch->ch_timer;
		if (t == LEDC_NTIMER)
			for (t = 0; t < LEDC_NTIMER; t++)
				if (sc->sc_ltimer[t].refs == 0)
					break;
		if (t == LEDC_NTIMER)
			return EBUSY;		/* four periods in use */
		if (t != ch->ch_timer) {
			sc->sc_ltimer[t].refs++;
			if (ch->ch_timer >= 0)
				esp_ledc_release(sc, ch->ch_timer);
			ch->ch_timer = t;
		}
		if (sc->sc_ltimer[t].refs == 1 &&
		    (sc->sc_ltimer[t].div != div ||
		     sc->sc_ltimer[t].res != res ||
		     !(RD(sc, LEDC_TIMER_POWER_UP) & __BIT(t)))) {
			/* (Re)start the timer. */
			sc->sc_ltimer[t].div = div;
			sc->sc_ltimer[t].res = res;
			esp_pwm_rmw(sc, LEDC_TIMER_POWER_UP, 0, __BIT(t));
			WR(sc, LEDC_TIMER_CONF(t), __SHIFTIN(div, TIMER_CLK_DIV) |
			    __SHIFTIN(res, TIMER_DUTY_RES) | TIMER_PARA_UP |
			    TIMER_RST);
			WR(sc, LEDC_TIMER_CONF(t), __SHIFTIN(div, TIMER_CLK_DIV) |
			    __SHIFTIN(res, TIMER_DUTY_RES));
		}
	}

	/* Duty: high from 0 to duty counts of 2^res. */
	duty = MIN(((uint64_t)conf->duty_cycle << res) + conf->period / 2,
	    (uint64_t)conf->period << res) / conf->period;
	esp_pwm_rmw(sc, LEDC_CH_POWER_UP, 0, __BIT(n));
	WR(sc, LEDC_CH_HPOINT(n), 0);
	WR(sc, LEDC_CH_DUTY(n), duty << 4);
	WR(sc, LEDC_GAMMA_RANGE(n, 0), GAMMA_FIXED);
	WR(sc, LEDC_CH_GAMMA_CONF(n), __SHIFTIN(1, GAMMA_ENTRY_NUM));
	esp_pwm_rmw(sc, LEDC_CH_CONF0(n), CONF0_TIMER_SEL | CONF0_IDLE_LV,
	    __SHIFTIN(t, CONF0_TIMER_SEL) | CONF0_SIG_OUT_EN);
	WR(sc, LEDC_CH_CONF1(n), CONF1_DUTY_START);
	esp_pwm_rmw(sc, LEDC_CH_CONF0(n), 0, CONF0_PARA_UP);

	/* What it does: from the registers. */
	ch->ch_div = __SHIFTOUT(RD(sc, LEDC_TIMER_CONF(t)), TIMER_CLK_DIV);
	ch->ch_count = 1U << __SHIFTOUT(RD(sc, LEDC_TIMER_CONF(t)),
	    TIMER_DUTY_RES);
	ch->ch_running = true;
	return 0;
}

static const struct esp_pwm_type esp_ledc_type = {
	.pt_prefix = "ledc",
	.pt_nchan = LEDC_NCHAN,
	.pt_init = esp_ledc_init,
	.pt_apply = esp_ledc_apply,
};

/*
 * MCPWM: channel 2o + g is operator o's generator g (A, B), on timer o.
 */

static int
esp_mcpwm_init(struct esp_pwm_softc *sc)
{
	WR(sc, MCPWM_UPDATE_CFG, UPDATE_GLOBAL_UP_EN | UPDATE_OP_UP_EN);
	WR(sc, MCPWM_UPDATE_CFG, UPDATE_GLOBAL_UP_EN | UPDATE_OP_UP_EN |
	    UPDATE_GLOBAL_FORCE_UP);
	WR(sc, MCPWM_UPDATE_CFG, UPDATE_GLOBAL_UP_EN | UPDATE_OP_UP_EN);
	WR(sc, MCPWM_INT_ENA, 0);
	WR(sc, MCPWM_INT_CLR, 0xffffffff);
	WR(sc, MCPWM_OPERATOR_TIMERSEL, 0 | (1 << 2) | (2 << 4));
	for (u_int o = 0; o < 3; o++) {
		WR(sc, MCPWM_TIMER_CFG1(o), __SHIFTIN(MOD_FREEZE, CFG1_MOD));
		WR(sc, MCPWM_GEN_CFG0(o), 0);
		WR(sc, MCPWM_GEN_STMP_CFG(o), __SHIFTIN(1, STMP_A_UPMETHOD) |
		    __SHIFTIN(1, STMP_B_UPMETHOD));
		WR(sc, MCPWM_GEN_ACT(o, 0), ACT_A);
		WR(sc, MCPWM_GEN_ACT(o, 1), ACT_B);
		WR(sc, MCPWM_GEN_FORCE(o), __SHIFTIN(FORCE_LOW, FORCE_A_MODE) |
		    __SHIFTIN(FORCE_LOW, FORCE_B_MODE));
	}
	return 0;
}

static void
esp_mcpwm_force(struct esp_pwm_softc *sc, u_int o, u_int g, u_int mode)
{
	esp_pwm_rmw(sc, MCPWM_GEN_FORCE(o),
	    FORCE_UPMETHOD | (g ? FORCE_B_MODE : FORCE_A_MODE),
	    g ? __SHIFTIN(mode, FORCE_B_MODE) : __SHIFTIN(mode, FORCE_A_MODE));
}

static int
esp_mcpwm_apply(struct esp_pwm_softc *sc, struct esp_pwm_chan *ch, bool on)
{
	const struct pwm_config * const conf = &ch->ch_conf;
	const u_int o = ch->ch_index / 2, g = ch->ch_index % 2;
	struct esp_pwm_chan * const sib = &sc->sc_chan[ch->ch_index ^ 1];
	uint64_t ticks;
	uint32_t prescale, peak, duty;

	if (!on || conf->period == 0) {
		esp_mcpwm_force(sc, o, g, FORCE_LOW);
		ch->ch_running = false;
		if (!sib->ch_running)
			WR(sc, MCPWM_TIMER_CFG1(o),
			    __SHIFTIN(MOD_FREEZE, CFG1_MOD));
		return 0;
	}

	ticks = (sc->sc_rate * conf->period + 500000000) / 1000000000;
	prescale = howmany(ticks, MCPWM_MAX_PERIOD);
	if (ticks < 2 || prescale > MCPWM_MAX_PRESCALE)
		return EINVAL;
	prescale = MAX(prescale, 1);
	peak = (ticks + prescale / 2) / prescale;
	if (sib->ch_running &&
	    (sib->ch_div != prescale || sib->ch_count != peak))
		return EBUSY;		/* A and B share the period */

	if (!sib->ch_running) {
		const uint32_t cfg0 = __SHIFTIN(prescale - 1, CFG0_PRESCALE) |
		    __SHIFTIN(peak - 1, CFG0_PERIOD);

		if (__SHIFTOUT(RD(sc, MCPWM_TIMER_CFG1(o)), CFG1_MOD) ==
		    MOD_FREEZE) {
			WR(sc, MCPWM_TIMER_CFG0(o), cfg0);
			WR(sc, MCPWM_TIMER_CFG0(o), cfg0 |
			    __SHIFTIN(1, CFG0_PERIOD_UPMETHOD));
			WR(sc, MCPWM_TIMER_CFG1(o),
			    __SHIFTIN(START_RUN, CFG1_START) |
			    __SHIFTIN(MOD_UP, CFG1_MOD));
		} else {
			/* New period at the next zero. */
			WR(sc, MCPWM_TIMER_CFG0(o), cfg0 |
			    __SHIFTIN(1, CFG0_PERIOD_UPMETHOD));
		}
	}

	duty = MIN(((uint64_t)conf->duty_cycle * peak + conf->period / 2) /
	    conf->period, peak);
	if (duty == 0) {
		esp_mcpwm_force(sc, o, g, FORCE_LOW);
	} else if (duty >= peak) {
		esp_mcpwm_force(sc, o, g, FORCE_HIGH);
	} else {
		WR(sc, MCPWM_GEN_TSTMP(o, g), duty);
		esp_mcpwm_force(sc, o, g, FORCE_OFF);
	}

	ch->ch_div = __SHIFTOUT(RD(sc, MCPWM_TIMER_CFG0(o)), CFG0_PRESCALE) + 1;
	ch->ch_count = __SHIFTOUT(RD(sc, MCPWM_TIMER_CFG0(o)), CFG0_PERIOD) + 1;
	ch->ch_running = true;
	return 0;
}

static const struct esp_pwm_type esp_mcpwm_type = {
	.pt_prefix = "pwm",
	.pt_nchan = MCPWM_NCHAN,
	.pt_init = esp_mcpwm_init,
	.pt_apply = esp_mcpwm_apply,
};

/*
 * Common: pwm(9), pins, sysctl
 */

static void
esp_pwm_signal(struct esp_pwm_softc *sc, struct esp_pwm_chan *ch, char *buf,
    size_t len)
{
	if (sc->sc_type == &esp_ledc_type)
		snprintf(buf, len, "ledc%u_%u", sc->sc_unit, ch->ch_index);
	else
		snprintf(buf, len, "pwm%u_%u%c", sc->sc_unit,
		    ch->ch_index / 2, "ab"[ch->ch_index % 2]);
}

/* Route the channel to its pin, with its polarity. */
static int
esp_pwm_route(struct esp_pwm_softc *sc, struct esp_pwm_chan *ch, int pin)
{
	char sig[16];
	int error;

	if (pin < 0)
		return 0;
	esp_pwm_signal(sc, ch, sig, sizeof(sig));
	if ((error = esp_gpio_set_function(pin, sig)) != 0)
		return error;
	return esp_gpio_set_output_invert(pin,
	    ch->ch_conf.polarity == PWM_ACTIVE_LOW);
}

static int
esp_pwm_enable(pwm_tag_t pwm, bool enable)
{
	struct esp_pwm_chan * const ch = pwm->pwm_priv;
	struct esp_pwm_softc * const sc = ch->ch_sc;
	int error;

	mutex_enter(&sc->sc_lock);
	error = sc->sc_type->pt_apply(sc, ch, enable);
	if (error == 0)
		ch->ch_enabled = enable;
	mutex_exit(&sc->sc_lock);
	return error;
}

static int
esp_pwm_get_config(pwm_tag_t pwm, struct pwm_config *conf)
{
	struct esp_pwm_chan * const ch = pwm->pwm_priv;
	struct esp_pwm_softc * const sc = ch->ch_sc;

	mutex_enter(&sc->sc_lock);
	*conf = ch->ch_conf;
	mutex_exit(&sc->sc_lock);
	return 0;
}

static int
esp_pwm_set_config_locked(struct esp_pwm_softc *sc, struct esp_pwm_chan *ch,
    const struct pwm_config *conf, bool on)
{
	const struct pwm_config old = ch->ch_conf;
	int error;

	KASSERT(mutex_owned(&sc->sc_lock));

	ch->ch_conf = *conf;
	ch->ch_conf.duty_cycle = MIN(conf->duty_cycle, conf->period);
	if ((error = sc->sc_type->pt_apply(sc, ch, on)) != 0) {
		ch->ch_conf = old;
		(void)sc->sc_type->pt_apply(sc, ch, ch->ch_enabled);
		return error;
	}
	if (old.polarity != ch->ch_conf.polarity && ch->ch_pin >= 0)
		(void)esp_gpio_set_output_invert(ch->ch_pin,
		    ch->ch_conf.polarity == PWM_ACTIVE_LOW);
	return 0;
}

static int
esp_pwm_set_config(pwm_tag_t pwm, const struct pwm_config *conf)
{
	struct esp_pwm_chan * const ch = pwm->pwm_priv;
	struct esp_pwm_softc * const sc = ch->ch_sc;
	int error;

	mutex_enter(&sc->sc_lock);
	error = esp_pwm_set_config_locked(sc, ch, conf, ch->ch_enabled);
	mutex_exit(&sc->sc_lock);
	return error;
}

/* pwms = <&ledc0 channel period flags> */
static pwm_tag_t
esp_pwm_get_tag(device_t dev, const void *data, size_t len)
{
	struct esp_pwm_softc * const sc = device_private(dev);
	const u_int *cells = data;
	struct esp_pwm_chan *ch;

	if (len != 16)
		return NULL;
	const u_int index = be32toh(cells[1]);
	if (index >= sc->sc_type->pt_nchan)
		return NULL;
	ch = &sc->sc_chan[index];
	mutex_enter(&sc->sc_lock);
	if (!ch->ch_enabled) {
		ch->ch_conf.period = be32toh(cells[2]);
		ch->ch_conf.polarity = (be32toh(cells[3]) & 1) ?
		    PWM_ACTIVE_LOW : PWM_ACTIVE_HIGH;
	}
	mutex_exit(&sc->sc_lock);
	return &ch->ch_pwm;
}

static const struct fdtbus_pwm_controller_func esp_pwm_funcs = {
	.get_tag = esp_pwm_get_tag,
};

enum { NODE_PERIOD, NODE_DUTY, NODE_PIN, NODE_INVERTED, NODE_ACTUAL };

static int
esp_pwm_sysctl(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_pwm_chan * const ch = node.sysctl_data;
	struct esp_pwm_softc * const sc = ch->ch_sc;
	const int which = node.sysctl_num - 100;
	struct pwm_config conf;
	char buf[48];
	int64_t val;
	int ival, error;

	mutex_enter(&sc->sc_lock);
	conf = ch->ch_conf;
	switch (which) {
	case NODE_PERIOD:
		val = ch->ch_enabled ? conf.period : 0;
		break;
	case NODE_DUTY:
		val = conf.duty_cycle;
		break;
	case NODE_PIN:
		val = ch->ch_pin;
		break;
	case NODE_INVERTED:
		val = conf.polarity == PWM_ACTIVE_LOW;
		break;
	case NODE_ACTUAL:
		if (!ch->ch_running) {
			strlcpy(buf, "off", sizeof(buf));
		} else {
			/*
			 * A period is div x count source clocks; LEDC's
			 * divider has 8 fraction bits.
			 */
			const uint64_t scale =
			    sc->sc_type == &esp_ledc_type ? 256 : 1;
			const uint64_t clocks = (uint64_t)ch->ch_div *
			    ch->ch_count;
			/* ns = clocks x 1e9 / (rate x scale), reduced */
			uint64_t num = 1000000000, den = sc->sc_rate * scale;
			uint64_t a = num, b = den;

			while (b != 0) {
				const uint64_t r = a % b;

				a = b;
				b = r;
			}
			num /= a;
			den /= a;
			const uint64_t pns = (clocks * num + den / 2) / den;
			const uint64_t hz1000 = (sc->sc_rate * scale * 1000 +
			    clocks / 2) / clocks;

			snprintf(buf, sizeof(buf),
			    "period %" PRIu64 " ns (%" PRIu64 ".%03" PRIu64
			    " Hz), %u steps", pns, hz1000 / 1000, hz1000 % 1000,
			    ch->ch_count);
		}
		break;
	}
	mutex_exit(&sc->sc_lock);

	if (which == NODE_ACTUAL) {
		node.sysctl_data = buf;
		node.sysctl_size = strlen(buf) + 1;
		return sysctl_lookup(SYSCTLFN_CALL(&node));
	}
	if (which == NODE_PERIOD || which == NODE_DUTY) {
		node.sysctl_data = &val;
	} else {
		ival = val;
		node.sysctl_data = &ival;
	}
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;
	if (which != NODE_PERIOD && which != NODE_DUTY)
		val = ival;

	mutex_enter(&sc->sc_lock);
	switch (which) {
	case NODE_PERIOD:
		if (val < 0 || val > UINT_MAX) {
			error = EINVAL;
			break;
		}
		conf.period = val;
		error = esp_pwm_set_config_locked(sc, ch, &conf, val > 0);
		if (error == 0)
			ch->ch_enabled = val > 0;
		break;
	case NODE_DUTY:
		if (val < 0 || val > UINT_MAX) {
			error = EINVAL;
			break;
		}
		conf.duty_cycle = val;
		error = esp_pwm_set_config_locked(sc, ch, &conf,
		    ch->ch_enabled);
		break;
	case NODE_INVERTED:
		conf.polarity = val ? PWM_ACTIVE_LOW : PWM_ACTIVE_HIGH;
		error = esp_pwm_set_config_locked(sc, ch, &conf,
		    ch->ch_enabled);
		break;
	case NODE_PIN:
		if (val < -1) {
			error = EINVAL;
			break;
		}
		if (val >= 0 && (error = esp_pwm_route(sc, ch, val)) != 0)
			break;
		if (ch->ch_pin >= 0 && ch->ch_pin != val) {
			(void)esp_gpio_set_output_invert(ch->ch_pin, false);
			(void)esp_gpio_set_function(ch->ch_pin, "gpio");
		}
		ch->ch_pin = val;
		break;
	}
	mutex_exit(&sc->sc_lock);
	return error;
}

static void
esp_pwm_sysctl_init(struct esp_pwm_softc *sc)
{
	static const struct {
		const char *name, *descr;
	} nodes[] = {
		[NODE_PERIOD] = { "period", "period, ns (0: off)" },
		[NODE_DUTY] = { "duty", "active time a period, ns" },
		[NODE_PIN] = { "pin", "GPIO driven, or -1" },
		[NODE_INVERTED] = { "inverted", "active low" },
		[NODE_ACTUAL] = { "actual", "what the hardware does" },
	};
	const struct sysctlnode *top, *cnode;

	if (sysctl_createv(&sc->sc_sysctllog, 0, NULL, &top, 0,
	    CTLTYPE_NODE, device_xname(sc->sc_dev),
	    SYSCTL_DESCR("PWM channels"), NULL, 0, NULL, 0,
	    CTL_HW, CTL_CREATE, CTL_EOL) != 0)
		return;
	for (u_int n = 0; n < sc->sc_type->pt_nchan; n++) {
		struct esp_pwm_chan * const ch = &sc->sc_chan[n];
		char name[8];

		snprintf(name, sizeof(name), "ch%u", n);
		if (sysctl_createv(&sc->sc_sysctllog, 0, &top, &cnode, 0,
		    CTLTYPE_NODE, name, SYSCTL_DESCR("channel"), NULL, 0,
		    NULL, 0, CTL_CREATE, CTL_EOL) != 0)
			continue;
		for (u_int i = 0; i < __arraycount(nodes); i++) {
			if (i == NODE_ACTUAL)
				sysctl_createv(&sc->sc_sysctllog, 0, &cnode,
				    NULL, CTLFLAG_READONLY, CTLTYPE_STRING,
				    nodes[i].name, SYSCTL_DESCR(nodes[i].descr),
				    esp_pwm_sysctl, 0, (void *)ch, 0, 100 + i,
				    CTL_EOL);
			else if (i == NODE_PERIOD || i == NODE_DUTY)
				sysctl_createv(&sc->sc_sysctllog, 0, &cnode,
				    NULL, CTLFLAG_READWRITE, CTLTYPE_QUAD,
				    nodes[i].name, SYSCTL_DESCR(nodes[i].descr),
				    esp_pwm_sysctl, 0, (void *)ch, 0, 100 + i,
				    CTL_EOL);
			else
				sysctl_createv(&sc->sc_sysctllog, 0, &cnode,
				    NULL, CTLFLAG_READWRITE, CTLTYPE_INT,
				    nodes[i].name, SYSCTL_DESCR(nodes[i].descr),
				    esp_pwm_sysctl, 0, (void *)ch, 0, 100 + i,
				    CTL_EOL);
		}
	}
}

static const struct device_compatible_entry ledc_compat[] = {
	{ .compat = "espressif,esp32s31-ledc",	.data = &esp_ledc_type },
	DEVICE_COMPAT_EOL
};

static const struct device_compatible_entry mcpwm_compat[] = {
	{ .compat = "espressif,esp32s31-mcpwm",	.data = &esp_mcpwm_type },
	DEVICE_COMPAT_EOL
};

static int
esp_ledc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, ledc_compat);
}

static int
esp_mcpwm_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, mcpwm_compat);
}

static void
esp_pwm_attach(device_t parent, device_t self, void *aux)
{
	struct esp_pwm_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	sc->sc_type = of_compatible_match(phandle, ledc_compat) ?
	    &esp_ledc_type : &esp_mcpwm_type;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	if (sc->sc_type == &esp_ledc_type)
		sc->sc_unit = addr == LEDC_BASE1 ? 1 : 0;
	else
		sc->sc_unit = (addr - MCPWM_BASE0) / 0x1000;

	/* The core clock first: the registers live in its domain. */
	if ((clk = fdtbus_clock_get_index(phandle, 0)) == NULL ||
	    clk_enable(clk) != 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}
	sc->sc_rate = clk_get_rate(clk);
	if (sc->sc_rate == 0) {
		aprint_error(": no clock rate\n");
		return;
	}
	if ((rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		fdtbus_reset_assert(rst);
		fdtbus_reset_deassert(rst);
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	aprint_naive("\n");
	aprint_normal(": %s %u, %u channels, %" PRIu64 " MHz\n",
	    sc->sc_type == &esp_ledc_type ? "LED PWM controller" :
	    "motor control PWM", sc->sc_unit, sc->sc_type->pt_nchan,
	    sc->sc_rate / 1000000);

	sc->sc_type->pt_init(sc);
	for (u_int n = 0; n < sc->sc_type->pt_nchan; n++) {
		struct esp_pwm_chan * const ch = &sc->sc_chan[n];

		ch->ch_sc = sc;
		ch->ch_index = n;
		ch->ch_pin = -1;
		ch->ch_timer = -1;
		ch->ch_pwm.pwm_enable = esp_pwm_enable;
		ch->ch_pwm.pwm_get_config = esp_pwm_get_config;
		ch->ch_pwm.pwm_set_config = esp_pwm_set_config;
		ch->ch_pwm.pwm_dev = self;
		ch->ch_pwm.pwm_priv = ch;
	}

	fdtbus_register_pwm_controller(self, phandle, &esp_pwm_funcs);
	esp_pwm_sysctl_init(sc);
}

CFATTACH_DECL_NEW(esp_ledc, sizeof(struct esp_pwm_softc),
	esp_ledc_match, esp_pwm_attach, NULL, NULL);
CFATTACH_DECL_NEW(esp_mcpwm, sizeof(struct esp_pwm_softc),
	esp_mcpwm_match, esp_pwm_attach, NULL, NULL);
