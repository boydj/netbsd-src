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
 * ESP32-S31 watchdog: the main system watchdog (MWDT) of timer group 1,
 * for sysmon_wdog(9) / wdogctl(8).
 *
 * One stage, "reset system" (the HP side: both harts and the HP
 * peripherals; the RTC timer and the LP store registers live on).  The
 * MWDT counts the 40 MHz XTAL divided by 20000, 500 us a tick.  Its
 * registers are write-protected (a key in WPROTECT), and configuration
 * changes take effect on CONF_UPDATE_EN.  Timer group 0 is left alone:
 * ESP-IDF on hart 1 calibrates the slow clock with it.
 *
 *	watchdog@20581000 {
 *		compatible = "espressif,esp32s31-mwdt";
 *		reg = <0x20581000 0x100>;
 *		clocks = <&clkrst 31>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/wdog.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>
#include <dev/sysmon/sysmonvar.h>

#define	TIMG_WDTCONFIG0		0x48
#define	 WDTCONFIG0_FLASHBOOT_MOD_EN	__BIT(14)
#define	 WDTCONFIG0_SYS_RESET_LENGTH	__BITS(17, 15)
#define	 WDTCONFIG0_CPU_RESET_LENGTH	__BITS(20, 18)
#define	 WDTCONFIG0_CONF_UPDATE_EN	__BIT(22)
#define	 WDTCONFIG0_STG3		__BITS(24, 23)
#define	 WDTCONFIG0_STG2		__BITS(26, 25)
#define	 WDTCONFIG0_STG1		__BITS(28, 27)
#define	 WDTCONFIG0_STG0		__BITS(30, 29)
#define	  STG_OFF		0
#define	  STG_RESET_SYSTEM	3
#define	 WDTCONFIG0_EN			__BIT(31)
#define	TIMG_WDTCONFIG1		0x4c
#define	 WDTCONFIG1_CLK_PRESCALE	__BITS(31, 16)
#define	TIMG_WDTCONFIG2		0x50	/* stage 0 timeout, ticks */
#define	TIMG_WDTFEED		0x60
#define	TIMG_WDTWPROTECT	0x64
#define	 WDT_WKEY		0x50d83aa1
#define	TIMG_INT_ENA		0x70
#define	TIMG_INT_CLR		0x7c
#define	 INT_WDT		__BIT(2)

#define	ESP_WDT_PRESCALE	20000		/* 40 MHz -> 2 kHz */
#define	ESP_WDT_TICKS_PER_SEC	2000
#define	ESP_WDT_DEFAULT_PERIOD	15		/* seconds */
#define	ESP_WDT_MAX_PERIOD	3600

struct esp_wdt_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	struct sysmon_wdog	sc_smw;
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	esp_wdt_match(device_t, cfdata_t, void *);
static void	esp_wdt_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_wdt, sizeof(struct esp_wdt_softc),
	esp_wdt_match, esp_wdt_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-mwdt" },
	DEVICE_COMPAT_EOL
};

static void
esp_wdt_unlock(struct esp_wdt_softc *sc)
{
	WR(sc, TIMG_WDTWPROTECT, WDT_WKEY);
}

static void
esp_wdt_lock(struct esp_wdt_softc *sc)
{
	WR(sc, TIMG_WDTWPROTECT, 0);
}

static void
esp_wdt_conf(struct esp_wdt_softc *sc, uint32_t conf0)
{
	WR(sc, TIMG_WDTCONFIG0, conf0 | WDTCONFIG0_CONF_UPDATE_EN);
}

static void
esp_wdt_stop(struct esp_wdt_softc *sc)
{
	esp_wdt_unlock(sc);
	esp_wdt_conf(sc, RD(sc, TIMG_WDTCONFIG0) &
	    ~(WDTCONFIG0_EN | WDTCONFIG0_FLASHBOOT_MOD_EN | WDTCONFIG0_STG0 |
	    WDTCONFIG0_STG1 | WDTCONFIG0_STG2 | WDTCONFIG0_STG3));
	esp_wdt_lock(sc);
}

static int
esp_wdt_tickle(struct sysmon_wdog *smw)
{
	struct esp_wdt_softc * const sc = smw->smw_cookie;

	esp_wdt_unlock(sc);
	WR(sc, TIMG_WDTFEED, 1);
	esp_wdt_lock(sc);
	return 0;
}

static int
esp_wdt_setmode(struct sysmon_wdog *smw)
{
	struct esp_wdt_softc * const sc = smw->smw_cookie;
	uint32_t conf0;

	if ((smw->smw_mode & WDOG_MODE_MASK) == WDOG_MODE_DISARMED) {
		esp_wdt_stop(sc);
		return 0;
	}

	if (smw->smw_period == WDOG_PERIOD_DEFAULT)
		smw->smw_period = ESP_WDT_DEFAULT_PERIOD;
	if (smw->smw_period == 0 || smw->smw_period > ESP_WDT_MAX_PERIOD)
		return EINVAL;

	esp_wdt_unlock(sc);
	conf0 = RD(sc, TIMG_WDTCONFIG0) & ~(WDTCONFIG0_EN |
	    WDTCONFIG0_FLASHBOOT_MOD_EN | WDTCONFIG0_STG0 | WDTCONFIG0_STG1 |
	    WDTCONFIG0_STG2 | WDTCONFIG0_STG3 | WDTCONFIG0_SYS_RESET_LENGTH |
	    WDTCONFIG0_CPU_RESET_LENGTH);
	esp_wdt_conf(sc, conf0);
	WR(sc, TIMG_WDTCONFIG1, (RD(sc, TIMG_WDTCONFIG1) &
	    ~WDTCONFIG1_CLK_PRESCALE) |
	    __SHIFTIN(ESP_WDT_PRESCALE, WDTCONFIG1_CLK_PRESCALE));
	WR(sc, TIMG_WDTCONFIG2, smw->smw_period * ESP_WDT_TICKS_PER_SEC);
	conf0 |= __SHIFTIN(STG_RESET_SYSTEM, WDTCONFIG0_STG0) |
	    __SHIFTIN(7, WDTCONFIG0_SYS_RESET_LENGTH) |
	    __SHIFTIN(7, WDTCONFIG0_CPU_RESET_LENGTH);
	esp_wdt_conf(sc, conf0);
	WR(sc, TIMG_WDTFEED, 1);
	esp_wdt_conf(sc, conf0 | WDTCONFIG0_EN);
	esp_wdt_lock(sc);
	return 0;
}

static int
esp_wdt_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_wdt_attach(device_t parent, device_t self, void *aux)
{
	struct esp_wdt_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
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
	if ((clk = fdtbus_clock_get_index(phandle, 0)) != NULL &&
	    clk_enable(clk) != 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}

	/* Whatever the boot chain left running, stop it. */
	esp_wdt_stop(sc);
	WR(sc, TIMG_INT_ENA, RD(sc, TIMG_INT_ENA) & ~INT_WDT);
	WR(sc, TIMG_INT_CLR, INT_WDT);

	aprint_naive("\n");
	aprint_normal(": watchdog (timer group 1)\n");

	sc->sc_smw.smw_name = device_xname(self);
	sc->sc_smw.smw_cookie = sc;
	sc->sc_smw.smw_setmode = esp_wdt_setmode;
	sc->sc_smw.smw_tickle = esp_wdt_tickle;
	sc->sc_smw.smw_period = ESP_WDT_DEFAULT_PERIOD;
	if (sysmon_wdog_register(&sc->sc_smw) != 0)
		aprint_error_dev(self, "couldn't register with sysmon\n");
}
