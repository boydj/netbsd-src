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
 * Espressif ESP32-S31 true random number generator.
 *
 * The TRNG block samples an analog noise source into a 32-bit data
 * register.  There is no "data ready" indication: the register can be
 * read at any time, and reading faster than the noise source refills it
 * returns correlated values.  ESP-IDF leaves at least 16 APB cycles per
 * output byte between reads; we wait 1us per 32-bit word, which is more.
 *
 * Nothing is published about the min-entropy of the noise source, so it
 * is credited conservatively at one bit per byte.  A start-up test and a
 * continuous repetition test stop all crediting if the output looks stuck
 * (e.g. an unclocked block or an emulator without a TRNG model, both of
 * which read back constant words).
 *
 * Bindings (same as Espressif's Linux BSP):
 *	compatible = "espressif,esp32s31-trng";
 *	reg = <TRNG base>, <RNG bus clock/reset register>;
 *	reg-names = "trng", "clkrst";
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/rndsource.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#define	TRNG_CONF			0x00
#define	 TRNG_CONF_SAMPLE_ENABLE	__BIT(31)
#define	 TRNG_CONF_NOISE_CRC_EN		__BIT(30)
#define	TRNG_DATA			0x48
#define	TRNG_DATE			0xfc
#define	 TRNG_DATE_CLK_EN		__BIT(28)

#define	CLKRST_RNG_BUS_RST_EN		__BIT(31)
#define	CLKRST_RNG_BUS_CLK_EN		__BIT(30)

#define	TRNG_WORD_DELAY_US		1
#define	TRNG_SELFTEST_WORDS		64
#define	TRNG_BITS_PER_BYTE		1	/* entropy credited per byte */

struct esp_trng_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	bus_space_handle_t	sc_clkrst_bsh;
	uint32_t		sc_last;
	bool			sc_failed;
	krndsource_t		sc_rndsource;
};

#define	TRNG_READ(sc, reg)	\
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	TRNG_WRITE(sc, reg, val)	\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static int	esp_trng_match(device_t, cfdata_t, void *);
static void	esp_trng_attach(device_t, device_t, void *);
static void	esp_trng_get(size_t, void *);

CFATTACH_DECL_NEW(esp_trng, sizeof(struct esp_trng_softc),
	esp_trng_match, esp_trng_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-trng" },
	DEVICE_COMPAT_EOL
};

static int
esp_trng_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_trng_enable(struct esp_trng_softc *sc)
{
	const bus_space_handle_t ch = sc->sc_clkrst_bsh;
	uint32_t val;

	/* Ungate the bus clock and pulse the block reset. */
	val = bus_space_read_4(sc->sc_bst, ch, 0);
	bus_space_write_4(sc->sc_bst, ch, 0,
	    val | CLKRST_RNG_BUS_CLK_EN | CLKRST_RNG_BUS_RST_EN);
	bus_space_write_4(sc->sc_bst, ch, 0,
	    (val | CLKRST_RNG_BUS_CLK_EN) & ~CLKRST_RNG_BUS_RST_EN);

	TRNG_WRITE(sc, TRNG_DATE, TRNG_READ(sc, TRNG_DATE) | TRNG_DATE_CLK_EN);
	TRNG_WRITE(sc, TRNG_CONF, TRNG_READ(sc, TRNG_CONF) |
	    TRNG_CONF_SAMPLE_ENABLE | TRNG_CONF_NOISE_CRC_EN);
}

static uint32_t
esp_trng_word(struct esp_trng_softc *sc)
{
	delay(TRNG_WORD_DELAY_US);
	return TRNG_READ(sc, TRNG_DATA);
}

/*
 * Start-up test: no two consecutive words may be equal.  A healthy 32-bit
 * source fails this with probability ~2^-26; a stuck or absent one always
 * does.
 */
static bool
esp_trng_selftest(struct esp_trng_softc *sc)
{
	uint32_t prev, cur;
	u_int i;

	prev = esp_trng_word(sc);
	for (i = 1; i < TRNG_SELFTEST_WORDS; i++) {
		cur = esp_trng_word(sc);
		if (cur == prev)
			return false;
		prev = cur;
	}
	sc->sc_last = prev;
	return true;
}

static void
esp_trng_attach(device_t parent, device_t self, void *aux)
{
	struct esp_trng_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr, caddr;
	bus_size_t size, csize;

	if (fdtbus_get_reg_byname(phandle, "trng", &addr, &size) != 0 &&
	    fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (fdtbus_get_reg_byname(phandle, "clkrst", &caddr, &csize) != 0 &&
	    fdtbus_get_reg(phandle, 1, &caddr, &csize) != 0) {
		aprint_error(": couldn't get clock/reset register\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0 ||
	    bus_space_map(sc->sc_bst, caddr, csize, 0,
		&sc->sc_clkrst_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 TRNG\n");

	esp_trng_enable(sc);
	if (!esp_trng_selftest(sc)) {
		aprint_error_dev(self,
		    "output is stuck, not using it as an entropy source\n");
		return;
	}

	rndsource_setcb(&sc->sc_rndsource, esp_trng_get, sc);
	rnd_attach_source(&sc->sc_rndsource, device_xname(self), RND_TYPE_RNG,
	    RND_FLAG_COLLECT_VALUE | RND_FLAG_HASCB);
}

/*
 * entropy(9) callback.  Calls are serialized by the entropy subsystem.
 * bytes_wanted is in bytes of entropy, so deliver enough data to cover
 * it at our (low) credit rate.
 */
static void
esp_trng_get(size_t bytes_wanted, void *priv)
{
	struct esp_trng_softc * const sc = priv;
	uint32_t buf[8];
	size_t bits_wanted = bytes_wanted * NBBY;
	u_int i;

	while (bits_wanted > 0 && !sc->sc_failed) {
		for (i = 0; i < __arraycount(buf); i++) {
			buf[i] = esp_trng_word(sc);
			/* Continuous test: a repeated word means trouble. */
			if (buf[i] == sc->sc_last) {
				sc->sc_failed = true;
				device_printf(sc->sc_dev, "output is stuck, "
				    "no longer crediting entropy\n");
			}
			sc->sc_last = buf[i];
		}
		const size_t credit = sc->sc_failed ? 0 :
		    sizeof(buf) * TRNG_BITS_PER_BYTE;
		rnd_add_data_sync(&sc->sc_rndsource, buf, sizeof(buf), credit);
		bits_wanted -= MIN(bits_wanted, credit);
	}
	explicit_memset(buf, 0, sizeof(buf));
}
