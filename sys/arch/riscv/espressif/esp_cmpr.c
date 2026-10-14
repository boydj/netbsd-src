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
 * ESP32-S31 analog comparator ("zero detect"): three channels, each
 * comparing one of four fixed pads (GPIO37-40) against a reference, as
 * envsys(4) indicators ("on": above the reference).
 *
 * The reference is internal, n x 10% of VDD (n = 0..7), with optional
 * hysteresis, or external, on another of the four pads.  This version of
 * the block only compares when told to, so every refresh starts a scan.
 * Nothing is enabled until asked for: the pads are strapping pins and
 * SD slot 1's.
 *
 *	sysctl -w hw.espcmpr0.ch1=37		compare GPIO37 (-1: off)
 *	sysctl -w hw.espcmpr0.reference=5	against 50% of VDD
 *	sysctl -w hw.espcmpr0.reference=40	or against GPIO40
 *	envstat -d espcmpr0
 *
 *	comparator@2039a000 {
 *		compatible = "espressif,esp32s31-ana-cmpr";
 *		reg = <0x2039a000 0x400>;
 *		clocks = <&clkrst 42>;
 *		resets = <&clkrst 34>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>
#include <dev/sysmon/sysmonvar.h>

#include <riscv/espressif/esp32s31_var.h>

#define	CMPR_CONF		0x00
#define	 CONF_VREF_SEL		__BITS(3, 0)	/* one-hot pad masks */
#define	 CONF_CH3_SEL		__BITS(7, 4)
#define	 CONF_CH2_SEL		__BITS(11, 8)
#define	 CONF_CH1_SEL		__BITS(15, 12)
#define	 CONF_LIMIT_CNT		__BITS(27, 20)
#define	 CONF_POLL_MASK		__BITS(30, 28)	/* bit 0: channel 1 */
#define	 CONF_POLL_MODE		__BIT(31)	/* 0: scan all per start */
#define	CMPR_FILTER_CNT		0x04
#define	CMPR_POLL_PERIOD	0x08
#define	CMPR_INT_ENA		0x10
#define	CMPR_INT_CLR		0x18
#define	CMPR_CHANNEL_STATUS	0x38	/* CH3 bit 0, CH2 bit 1, CH1 bit 2 */
#define	CMPR_PAD_CFG		0x3c
#define	 PAD_HYS		__BITS(2, 0)
#define	 PAD_HYS_EN		__BIT(3)
#define	 PAD_DREF		__BITS(6, 4)
#define	 PAD_MODE_EXT		__BIT(7)
#define	 PAD_XPD		__BIT(8)
#define	CMPR_START		0x40

#define	CMPR_NCHAN		3
#define	CMPR_NPADS		4
#define	CMPR_PAD0_GPIO		37
#define	CMPR_REF_DEFAULT	5		/* 50% of VDD */

struct esp_cmpr_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	kmutex_t		sc_lock;
	int			sc_chan[CMPR_NCHAN];	/* pad, or -1 */
	int			sc_ref;		/* 0..7, or 8 + pad */
	int			sc_hys;		/* 0..3 */
	struct sysmon_envsys	*sc_sme;
	envsys_data_t		sc_sensor[CMPR_NCHAN];
	struct sysctllog	*sc_sysctllog;
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	esp_cmpr_match(device_t, cfdata_t, void *);
static void	esp_cmpr_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_cmpr, sizeof(struct esp_cmpr_softc),
	esp_cmpr_match, esp_cmpr_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-ana-cmpr" },
	DEVICE_COMPAT_EOL
};

static const uint32_t esp_cmpr_chan_sel[CMPR_NCHAN] = {
	CONF_CH1_SEL, CONF_CH2_SEL, CONF_CH3_SEL
};

/*
 * Program the selection.  Each pad may be in one selector only (reset
 * gives each its own), so unused selectors take a pad nobody uses.
 */
static int
esp_cmpr_program(struct esp_cmpr_softc *sc)
{
	const int refpad = sc->sc_ref >= 8 ? sc->sc_ref - 8 : -1;
	u_int used = 0, mask = 0, spare, poll = 0;
	uint32_t conf, pad;

	KASSERT(mutex_owned(&sc->sc_lock));

	for (u_int c = 0; c < CMPR_NCHAN; c++) {
		if (sc->sc_chan[c] < 0)
			continue;
		if (used & __BIT(sc->sc_chan[c]))
			return EBUSY;
		used |= __BIT(sc->sc_chan[c]);
		poll |= __BIT(c);
	}
	if (refpad >= 0) {
		if (used & __BIT(refpad))
			return EBUSY;
		used |= __BIT(refpad);
	}

	conf = RD(sc, CMPR_CONF) & ~(CONF_VREF_SEL | CONF_CH1_SEL |
	    CONF_CH2_SEL | CONF_CH3_SEL | CONF_POLL_MASK | CONF_POLL_MODE);
	mask = used;
	for (u_int c = 0; c < CMPR_NCHAN + 1; c++) {
		const uint32_t field = c < CMPR_NCHAN ?
		    esp_cmpr_chan_sel[c] : CONF_VREF_SEL;
		int p = c < CMPR_NCHAN ? sc->sc_chan[c] : refpad;

		if (p < 0) {
			spare = ~mask & (__BIT(CMPR_NPADS) - 1);
			KASSERT(spare != 0);
			p = ffs(spare) - 1;
			mask |= __BIT(p);
		}
		conf |= __SHIFTIN(__BIT(p), field);
	}
	conf |= __SHIFTIN(poll, CONF_POLL_MASK);

	pad = RD(sc, CMPR_PAD_CFG) & ~(PAD_HYS | PAD_HYS_EN | PAD_DREF |
	    PAD_MODE_EXT | PAD_XPD);
	if (refpad >= 0) {
		pad |= PAD_MODE_EXT;
	} else {
		static const uint8_t hys[4] = { 0, 1, 2, 4 };

		pad |= __SHIFTIN(sc->sc_ref, PAD_DREF);
		if (sc->sc_hys != 0)
			pad |= __SHIFTIN(hys[sc->sc_hys], PAD_HYS) |
			    PAD_HYS_EN;
	}
	if (poll != 0)
		pad |= PAD_XPD;

	/* The pads: analog. */
	for (u_int p = 0; p < CMPR_NPADS; p++) {
		if ((used & __BIT(p)) != 0 &&
		    esp_gpio_set_function(CMPR_PAD0_GPIO + p, "analog") != 0)
			return EBUSY;
	}

	WR(sc, CMPR_CONF, conf);
	WR(sc, CMPR_PAD_CFG, pad);
	return 0;
}

static void
esp_cmpr_refresh(struct sysmon_envsys *sme, envsys_data_t *edata)
{
	struct esp_cmpr_softc * const sc = sme->sme_cookie;
	const u_int c = edata->sensor;
	uint32_t st;

	mutex_enter(&sc->sc_lock);
	if (sc->sc_chan[c] < 0) {
		mutex_exit(&sc->sc_lock);
		edata->state = ENVSYS_SINVALID;
		return;
	}
	/* One scan of the enabled channels; a few cycles each. */
	WR(sc, CMPR_START, 1);
	DELAY(20);
	st = RD(sc, CMPR_CHANNEL_STATUS);
	mutex_exit(&sc->sc_lock);

	edata->value_cur = (st & __BIT(2 - c)) != 0;
	edata->state = ENVSYS_SVALID;
}

static int
esp_cmpr_sysctl_chan(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_cmpr_softc * const sc = node.sysctl_data;
	const u_int c = node.sysctl_num - 100;
	int val, old, error;

	val = sc->sc_chan[c] < 0 ? -1 : CMPR_PAD0_GPIO + sc->sc_chan[c];
	node.sysctl_data = &val;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;
	if (val != -1 &&
	    (val < CMPR_PAD0_GPIO || val >= CMPR_PAD0_GPIO + CMPR_NPADS))
		return EINVAL;

	mutex_enter(&sc->sc_lock);
	old = sc->sc_chan[c];
	sc->sc_chan[c] = val < 0 ? -1 : val - CMPR_PAD0_GPIO;
	if ((error = esp_cmpr_program(sc)) != 0) {
		sc->sc_chan[c] = old;
		(void)esp_cmpr_program(sc);
	}
	mutex_exit(&sc->sc_lock);
	return error;
}

static int
esp_cmpr_sysctl_ref(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_cmpr_softc * const sc = node.sysctl_data;
	int val, old, error;

	val = sc->sc_ref >= 8 ? CMPR_PAD0_GPIO + sc->sc_ref - 8 : sc->sc_ref;
	node.sysctl_data = &val;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;
	if (val >= CMPR_PAD0_GPIO && val < CMPR_PAD0_GPIO + CMPR_NPADS)
		val = 8 + val - CMPR_PAD0_GPIO;
	else if (val < 0 || val > 7)
		return EINVAL;

	mutex_enter(&sc->sc_lock);
	old = sc->sc_ref;
	sc->sc_ref = val;
	if ((error = esp_cmpr_program(sc)) != 0) {
		sc->sc_ref = old;
		(void)esp_cmpr_program(sc);
	}
	mutex_exit(&sc->sc_lock);
	return error;
}

static int
esp_cmpr_sysctl_hys(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_cmpr_softc * const sc = node.sysctl_data;
	int val, error;

	val = sc->sc_hys;
	node.sysctl_data = &val;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;
	if (val < 0 || val > 3)
		return EINVAL;

	mutex_enter(&sc->sc_lock);
	sc->sc_hys = val;
	error = esp_cmpr_program(sc);
	mutex_exit(&sc->sc_lock);
	return error;
}

static void
esp_cmpr_sysctl_init(struct esp_cmpr_softc *sc)
{
	const struct sysctlnode *node;

	if (sysctl_createv(&sc->sc_sysctllog, 0, NULL, &node, 0,
	    CTLTYPE_NODE, device_xname(sc->sc_dev),
	    SYSCTL_DESCR("analog comparator"), NULL, 0, NULL, 0,
	    CTL_HW, CTL_CREATE, CTL_EOL) != 0)
		return;
	for (u_int c = 0; c < CMPR_NCHAN; c++) {
		char name[4];

		snprintf(name, sizeof(name), "ch%u", c + 1);
		sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL,
		    CTLFLAG_READWRITE, CTLTYPE_INT, name,
		    SYSCTL_DESCR("GPIO compared (37-40), or -1: off"),
		    esp_cmpr_sysctl_chan, 0, (void *)sc, 0, 100 + c, CTL_EOL);
	}
	sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL,
	    CTLFLAG_READWRITE, CTLTYPE_INT, "reference",
	    SYSCTL_DESCR("0-7: n x 10% of VDD; 37-40: that GPIO"),
	    esp_cmpr_sysctl_ref, 0, (void *)sc, 0, CTL_CREATE, CTL_EOL);
	sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL,
	    CTLFLAG_READWRITE, CTLTYPE_INT, "hysteresis",
	    SYSCTL_DESCR("0-3, internal reference only"),
	    esp_cmpr_sysctl_hys, 0, (void *)sc, 0, CTL_CREATE, CTL_EOL);
}

static int
esp_cmpr_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_cmpr_attach(device_t parent, device_t self, void *aux)
{
	struct esp_cmpr_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	if ((clk = fdtbus_clock_get_index(phandle, 0)) == NULL ||
	    clk_enable(clk) != 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}
	if ((rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		fdtbus_reset_assert(rst);
		fdtbus_reset_deassert(rst);
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	aprint_naive("\n");
	aprint_normal(": analog comparator, GPIO%u-%u\n", CMPR_PAD0_GPIO,
	    CMPR_PAD0_GPIO + CMPR_NPADS - 1);

	/* Off, no interrupts; one scan per start, a short debounce. */
	WR(sc, CMPR_INT_ENA, 0);
	WR(sc, CMPR_INT_CLR, 0xfff);
	WR(sc, CMPR_FILTER_CNT, 1);
	WR(sc, CMPR_POLL_PERIOD, 40);		/* 1 us at 40 MHz */
	WR(sc, CMPR_CONF, (RD(sc, CMPR_CONF) & ~CONF_LIMIT_CNT) |
	    __SHIFTIN(1, CONF_LIMIT_CNT));
	for (u_int c = 0; c < CMPR_NCHAN; c++)
		sc->sc_chan[c] = -1;
	sc->sc_ref = CMPR_REF_DEFAULT;
	mutex_enter(&sc->sc_lock);
	(void)esp_cmpr_program(sc);
	mutex_exit(&sc->sc_lock);

	sc->sc_sme = sysmon_envsys_create();
	sc->sc_sme->sme_name = device_xname(self);
	sc->sc_sme->sme_cookie = sc;
	sc->sc_sme->sme_refresh = esp_cmpr_refresh;
	for (u_int c = 0; c < CMPR_NCHAN; c++) {
		envsys_data_t * const s = &sc->sc_sensor[c];

		s->units = ENVSYS_INDICATOR;
		s->state = ENVSYS_SINVALID;
		snprintf(s->desc, sizeof(s->desc), "ch%u above reference",
		    c + 1);
		if (sysmon_envsys_sensor_attach(sc->sc_sme, s) != 0)
			break;
	}
	if (sysmon_envsys_register(sc->sc_sme) != 0) {
		aprint_error_dev(self, "couldn't register with envsys\n");
		sysmon_envsys_destroy(sc->sc_sme);
		sc->sc_sme = NULL;
	}

	esp_cmpr_sysctl_init(sc);
}
