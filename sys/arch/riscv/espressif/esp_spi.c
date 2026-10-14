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
 * ESP32-S31 general-purpose SPI masters (GPSPI2, GPSPI3), for spi(4).
 *
 * Full-duplex transfers of up to 64 bytes through the controller's data
 * buffer (W0-W15), polled; longer ones are split with the chip select
 * held (MISC.CS_KEEP_ACTIVE).  Hardware chip selects CS0..CS2.  The
 * clock is the 40 MHz XTAL, divided as ESP-IDF's spi_ll_master_cal_clock.
 * Pins go through the GPIO matrix (spiN_sck, spiN_mosi, spiN_miso,
 * spiN_csM).
 *
 *	spi2: spi@2038f000 {
 *		compatible = "espressif,esp32s31-spi";
 *		reg = <0x2038f000 0x1000>;
 *		clocks = <&clkrst 19>;
 *		resets = <&clkrst 11>;
 *		num-cs = <3>;
 *		#address-cells = <1>;
 *		#size-cells = <0>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>
#include <dev/spi/spivar.h>

#define	SPI_CMD			0x00
#define	 CMD_UPDATE		__BIT(23)
#define	 CMD_USR		__BIT(24)
#define	SPI_ADDR		0x04
#define	SPI_CTRL		0x08
#define	 CTRL_LINE_MODES	0x0001c7e0	/* dual/quad/oct bits */
#define	 CTRL_RD_BIT_ORDER	__BITS(24, 23)
#define	 CTRL_WR_BIT_ORDER	__BITS(26, 25)
#define	SPI_CLOCK		0x0c
#define	 CLOCK_CLKCNT_L		__BITS(5, 0)
#define	 CLOCK_CLKCNT_H		__BITS(11, 6)
#define	 CLOCK_CLKCNT_N		__BITS(17, 12)
#define	 CLOCK_CLKDIV_PRE	__BITS(21, 18)
#define	 CLOCK_EQU_SYSCLK	__BIT(31)
#define	SPI_USER		0x10
#define	 USER_DOUTDIN		__BIT(0)
#define	 USER_CS_HOLD		__BIT(6)
#define	 USER_CS_SETUP		__BIT(7)
#define	 USER_CK_OUT_EDGE	__BIT(9)
#define	 USER_FWRITE_MODES	0x00007000
#define	 USER_SIO		__BIT(17)
#define	 USER_USR_MOSI		__BIT(27)
#define	 USER_USR_MISO		__BIT(28)
#define	 USER_USR_DUMMY		__BIT(29)
#define	 USER_USR_ADDR		__BIT(30)
#define	 USER_USR_COMMAND	__BIT(31)
#define	SPI_USER1		0x14
#define	 USER1_MST_WFULL_ERR_END_EN __BIT(16)
#define	 USER1_CS_SETUP_TIME	__BITS(21, 17)
#define	 USER1_CS_HOLD_TIME	__BITS(26, 22)
#define	SPI_USER2		0x18
#define	 USER2_MST_REMPTY_ERR_END_EN __BIT(27)
#define	SPI_MS_DLEN		0x1c
#define	SPI_MISC		0x20
#define	 MISC_CS_DIS(n)		__BIT(n)	/* CS0..CS5 */
#define	 MISC_CS_DIS_ALL	__BITS(5, 0)
#define	 MISC_MASTER_CS_POL	__BITS(12, 7)
#define	 MISC_CK_IDLE_EDGE	__BIT(29)
#define	 MISC_CS_KEEP_ACTIVE	__BIT(30)
#define	SPI_DMA_CONF		0x30
#define	 DMA_CONF_SLV_RX_SEG_TRANS_CLR_EN __BIT(19)
#define	 DMA_CONF_SLV_TX_SEG_TRANS_CLR_EN __BIT(20)
#define	SPI_DMA_INT_ENA		0x34
#define	SPI_DMA_INT_CLR		0x38
#define	SPI_DMA_INT_RAW		0x3c
#define	 DMA_INT_TRANS_DONE	__BIT(12)
#define	SPI_W(n)		(0x98 + 4 * (n))
#define	SPI_SLAVE		0xe0
#define	 SLAVE_MODE		__BIT(26)
#define	 SLAVE_MST_FD_WAIT_DMA_TX_DATA __BIT(29)

#define	ESP_SPI_BUF		64
#define	ESP_SPI_MAXCS		6

struct esp_spi_slave {
	int		ss_mode;
	uint32_t	ss_clock;	/* CLOCK register */
	bool		ss_configured;
};

struct esp_spi_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	kmutex_t		sc_lock;
	struct spi_controller	sc_spi;
	u_int			sc_fapb;	/* module clock, Hz */
	struct esp_spi_slave	sc_slaves[ESP_SPI_MAXCS];
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	esp_spi_match(device_t, cfdata_t, void *);
static void	esp_spi_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_spi, sizeof(struct esp_spi_softc),
	esp_spi_match, esp_spi_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-spi" },
	DEVICE_COMPAT_EOL
};

/* Latch the configuration into the SPI clock domain. */
static int
esp_spi_update(struct esp_spi_softc *sc)
{
	u_int guard;

	WR(sc, SPI_CMD, CMD_UPDATE);
	for (guard = 100000; (RD(sc, SPI_CMD) & CMD_UPDATE) != 0; guard--) {
		if (guard == 0)
			return ETIMEDOUT;
		DELAY(1);
	}
	return 0;
}

/* CLOCK register for an SCK of freq Hz, as spi_ll_master_cal_clock(). */
static uint32_t
esp_spi_clock(u_int fapb, u_int freq, u_int *actual)
{
	u_int bestn = 0, bestpre = 0, besterr = UINT_MAX;

	if (freq >= fapb / 4 * 3) {
		*actual = fapb;
		return CLOCK_EQU_SYSCLK;
	}
	for (u_int n = 2; n <= 64; n++) {
		u_int pre = ((fapb / n) + freq / 2) / freq;
		u_int err;

		pre = MAX(pre, 1);
		pre = MIN(pre, 16);
		err = abs((int)(fapb / (pre * n)) - (int)freq);
		if (err <= besterr) {
			besterr = err;
			bestn = n;
			bestpre = pre;
		}
	}
	const u_int h = MAX(1, (128 * bestn + 127) / 256);
	*actual = fapb / (bestpre * bestn);
	return __SHIFTIN(bestn - 1, CLOCK_CLKCNT_L) |
	    __SHIFTIN(h - 1, CLOCK_CLKCNT_H) |
	    __SHIFTIN(bestn - 1, CLOCK_CLKCNT_N) |
	    __SHIFTIN(bestpre - 1, CLOCK_CLKDIV_PRE);
}

static int
esp_spi_configure(void *cookie, int slave, int mode, int speed)
{
	struct esp_spi_softc * const sc = cookie;
	struct esp_spi_slave *ss;
	u_int actual;

	if (slave < 0 || slave >= sc->sc_spi.sct_nslaves)
		return EINVAL;
	if (mode < SPI_MODE_0 || mode > SPI_MODE_3 || speed <= 0)
		return EINVAL;

	ss = &sc->sc_slaves[slave];
	mutex_enter(&sc->sc_lock);
	ss->ss_mode = mode;
	ss->ss_clock = esp_spi_clock(sc->sc_fapb, speed, &actual);
	ss->ss_configured = true;
	mutex_exit(&sc->sc_lock);
	return 0;
}

/* Select a slave: clock, mode and chip select. */
static void
esp_spi_select(struct esp_spi_softc *sc, int slave)
{
	const struct esp_spi_slave * const ss = &sc->sc_slaves[slave];
	uint32_t user, misc;

	WR(sc, SPI_CLOCK, ss->ss_clock);
	WR(sc, SPI_CTRL, RD(sc, SPI_CTRL) &
	    ~(CTRL_LINE_MODES | CTRL_RD_BIT_ORDER | CTRL_WR_BIT_ORDER));

	user = USER_DOUTDIN | USER_CS_HOLD;
	misc = RD(sc, SPI_MISC) & ~(MISC_CS_DIS_ALL | MISC_MASTER_CS_POL |
	    MISC_CK_IDLE_EDGE | MISC_CS_KEEP_ACTIVE);
	switch (ss->ss_mode) {
	case SPI_MODE_0:
		break;
	case SPI_MODE_1:
		user |= USER_CK_OUT_EDGE;
		break;
	case SPI_MODE_2:
		user |= USER_CK_OUT_EDGE;
		misc |= MISC_CK_IDLE_EDGE;
		break;
	case SPI_MODE_3:
		misc |= MISC_CK_IDLE_EDGE;
		break;
	}
	for (int cs = 0; cs < ESP_SPI_MAXCS; cs++)
		if (cs != slave)
			misc |= MISC_CS_DIS(cs);
	WR(sc, SPI_USER, user);
	WR(sc, SPI_USER1, __SHIFTIN(1, USER1_CS_HOLD_TIME));
	WR(sc, SPI_MISC, misc);
}

/* One full-duplex transfer of n <= 64 bytes. */
static int
esp_spi_xfer(struct esp_spi_softc *sc, const uint8_t *wp, uint8_t *rp,
    size_t n, bool keepcs)
{
	uint32_t user, w;
	u_int guard;
	int error;

	KASSERT(n > 0 && n <= ESP_SPI_BUF);

	WR(sc, SPI_DMA_INT_CLR, DMA_INT_TRANS_DONE);
	for (u_int i = 0; i < howmany(n, 4); i++) {
		w = 0xffffffff;
		if (wp != NULL)
			memcpy(&w, wp + 4 * i, MIN(4, n - 4 * i));
		WR(sc, SPI_W(i), le32toh(w));
	}
	WR(sc, SPI_MS_DLEN, n * 8 - 1);
	user = RD(sc, SPI_USER) & ~(USER_USR_COMMAND | USER_USR_ADDR |
	    USER_USR_DUMMY | USER_USR_MISO | USER_FWRITE_MODES | USER_SIO);
	user |= USER_USR_MOSI | (rp != NULL ? USER_USR_MISO : 0);
	WR(sc, SPI_USER, user);
	WR(sc, SPI_SLAVE, (RD(sc, SPI_SLAVE) & ~SLAVE_MODE) |
	    SLAVE_MST_FD_WAIT_DMA_TX_DATA);
	if (keepcs)
		WR(sc, SPI_MISC, RD(sc, SPI_MISC) | MISC_CS_KEEP_ACTIVE);
	else
		WR(sc, SPI_MISC, RD(sc, SPI_MISC) & ~MISC_CS_KEEP_ACTIVE);

	if ((error = esp_spi_update(sc)) != 0)
		return error;
	WR(sc, SPI_CMD, CMD_USR);

	/* 64 bytes at 100 kHz is ~5 ms. */
	for (guard = 100000; (RD(sc, SPI_DMA_INT_RAW) & DMA_INT_TRANS_DONE) == 0;
	    guard--) {
		if (guard == 0)
			return ETIMEDOUT;
		DELAY(1);
	}

	if (rp != NULL) {
		for (u_int i = 0; i < howmany(n, 4); i++) {
			w = htole32(RD(sc, SPI_W(i)));
			memcpy(rp + 4 * i, &w, MIN(4, n - 4 * i));
		}
	}
	return 0;
}

static int
esp_spi_transfer(void *cookie, struct spi_transfer *st)
{
	struct esp_spi_softc * const sc = cookie;
	struct spi_chunk *chunk;
	int error = 0;

	mutex_enter(&sc->sc_lock);
	if (!sc->sc_slaves[st->st_slave].ss_configured) {
		mutex_exit(&sc->sc_lock);
		spi_done(st, EINVAL);
		return 0;
	}
	esp_spi_select(sc, st->st_slave);

	for (chunk = st->st_chunks; chunk != NULL && error == 0;
	    chunk = chunk->chunk_next) {
		const uint8_t *wp = chunk->chunk_write;
		uint8_t *rp = chunk->chunk_read;
		size_t left = chunk->chunk_count;

		while (left > 0 && error == 0) {
			const size_t n = MIN(left, ESP_SPI_BUF);
			const bool keep = n < left || chunk->chunk_next != NULL;

			error = esp_spi_xfer(sc, wp, rp, n, keep);
			if (wp != NULL)
				wp += n;
			if (rp != NULL)
				rp += n;
			left -= n;
		}
	}
	if (error) {
		/* Let go of the chip select. */
		WR(sc, SPI_MISC, RD(sc, SPI_MISC) & ~MISC_CS_KEEP_ACTIVE);
		esp_spi_update(sc);
	}
	mutex_exit(&sc->sc_lock);

	spi_done(st, error);
	return 0;
}

static int
esp_spi_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_spi_attach(device_t parent, device_t self, void *aux)
{
	struct esp_spi_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct clk *clk, *pclk;
	bus_addr_t addr;
	bus_size_t size;
	u_int ncs;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	if ((clk = fdtbus_clock_get_index(phandle, 0)) == NULL ||
	    clk_enable(clk) != 0 || (pclk = clk_get_parent(clk)) == NULL) {
		aprint_error(": couldn't enable clock\n");
		return;
	}
	/* XTAL, undivided (the module clock must stay at or below 80 MHz) */
	clk_set_rate(clk, clk_get_rate(pclk));
	sc->sc_fapb = clk_get_rate(clk);
	if (sc->sc_fapb == 0 || sc->sc_fapb > 80000000) {
		aprint_error(": bad module clock %u Hz\n", sc->sc_fapb);
		return;
	}
	if (of_getprop_uint32(phandle, "num-cs", &ncs) != 0)
		ncs = 3;
	ncs = MIN(ncs, ESP_SPI_MAXCS);

	fdtbus_pinctrl_set_config(phandle, "default");
	if ((rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		fdtbus_reset_assert(rst);
		fdtbus_reset_deassert(rst);
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	/* Master, as spi_ll_master_init(). */
	WR(sc, SPI_SLAVE, 0);
	WR(sc, SPI_USER, 0);
	WR(sc, SPI_USER1, RD(sc, SPI_USER1) & ~(USER1_CS_SETUP_TIME |
	    USER1_CS_HOLD_TIME | USER1_MST_WFULL_ERR_END_EN));
	WR(sc, SPI_USER2, RD(sc, SPI_USER2) & ~USER2_MST_REMPTY_ERR_END_EN);
	WR(sc, SPI_DMA_CONF, DMA_CONF_SLV_TX_SEG_TRANS_CLR_EN |
	    DMA_CONF_SLV_RX_SEG_TRANS_CLR_EN);
	WR(sc, SPI_DMA_INT_ENA, 0);
	WR(sc, SPI_DMA_INT_CLR, 0xffffffff);
	WR(sc, SPI_MISC, RD(sc, SPI_MISC) | MISC_CS_DIS_ALL);
	if (esp_spi_update(sc) != 0) {
		aprint_error(": not responding\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": SPI master, %u chip selects\n", ncs);

	sc->sc_spi.sct_cookie = sc;
	sc->sc_spi.sct_configure = esp_spi_configure;
	sc->sc_spi.sct_transfer = esp_spi_transfer;
	sc->sc_spi.sct_nslaves = ncs;
	spibus_attach(self, &sc->sc_spi);
}
