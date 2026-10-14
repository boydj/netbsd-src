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
 * ESP32-S31 I2C controllers (I2C0, I2C1), master mode, for iic(4).
 *
 * The controller runs a list of up to eight commands (RSTART, WRITE n,
 * READ n, STOP, END) on bytes in a 32-byte transmit FIFO and fills a
 * 32-byte receive FIFO.  Longer transfers are done in rounds: a round
 * that doesn't finish the transfer ends with END, which holds the bus
 * until the next round is loaded and started.  Transfers are polled.
 *
 * Timing as ESP-IDF's i2c_ll_master_cal_bus_clk(), from the 40 MHz XTAL.
 * Pins go through the GPIO matrix (pinctrl, or sysctl hw.espgpio0.pinN
 * with i2cN_scl/i2cN_sda).
 *
 *	i2c0: i2c@20385000 {
 *		compatible = "espressif,esp32s31-i2c";
 *		reg = <0x20385000 0x200>;
 *		clocks = <&clkrst 17>;
 *		resets = <&clkrst 9>;
 *		clock-frequency = <100000>;
 *		#address-cells = <1>;
 *		#size-cells = <0>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bitops.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>
#include <dev/i2c/i2cvar.h>

#define	I2C_SCL_LOW_PERIOD	0x00
#define	I2C_CTR			0x04
#define	 CTR_SDA_FORCE_OUT	__BIT(0)
#define	 CTR_SCL_FORCE_OUT	__BIT(1)
#define	 CTR_RX_FULL_ACK_LEVEL	__BIT(3)
#define	 CTR_MS_MODE		__BIT(4)
#define	 CTR_TRANS_START	__BIT(5)
#define	 CTR_TX_LSB_FIRST	__BIT(6)
#define	 CTR_RX_LSB_FIRST	__BIT(7)
#define	 CTR_ARBITRATION_EN	__BIT(9)
#define	 CTR_FSM_RST		__BIT(10)
#define	 CTR_CONF_UPGATE	__BIT(11)
#define	I2C_SR			0x08
#define	 SR_RESP_REC		__BIT(0)
#define	 SR_BUS_BUSY		__BIT(4)
#define	 SR_RXFIFO_CNT		__BITS(13, 8)
#define	I2C_TO			0x0c
#define	 TO_TIME_OUT_VALUE	__BITS(4, 0)
#define	 TO_TIME_OUT_EN		__BIT(5)
#define	I2C_FIFO_CONF		0x18
#define	 FIFO_CONF_NONFIFO_EN	__BIT(10)
#define	 FIFO_CONF_RX_FIFO_RST	__BIT(12)
#define	 FIFO_CONF_TX_FIFO_RST	__BIT(13)
#define	I2C_DATA		0x1c
#define	I2C_INT_RAW		0x20
#define	I2C_INT_CLR		0x24
#define	I2C_INT_ENA		0x28
#define	 INT_END_DETECT		__BIT(3)
#define	 INT_ARBITRATION_LOST	__BIT(5)
#define	 INT_TRANS_COMPLETE	__BIT(7)
#define	 INT_TIME_OUT		__BIT(8)
#define	 INT_NACK		__BIT(10)
#define	 INT_SCL_ST_TO		__BIT(13)
#define	 INT_SCL_MAIN_ST_TO	__BIT(14)
#define	 INT_ALL		0x7fff
#define	I2C_SDA_HOLD		0x30
#define	I2C_SDA_SAMPLE		0x34
#define	I2C_SCL_HIGH_PERIOD	0x38
#define	 SCL_HIGH_PERIOD	__BITS(8, 0)
#define	 SCL_WAIT_HIGH_PERIOD	__BITS(15, 9)
#define	I2C_SCL_START_HOLD	0x40
#define	I2C_SCL_RSTART_SETUP	0x44
#define	I2C_SCL_STOP_HOLD	0x48
#define	I2C_SCL_STOP_SETUP	0x4c
#define	I2C_FILTER_CFG		0x50
#define	 FILTER_SCL_THRES	__BITS(3, 0)
#define	 FILTER_SDA_THRES	__BITS(7, 4)
#define	 FILTER_SCL_EN		__BIT(8)
#define	 FILTER_SDA_EN		__BIT(9)
#define	I2C_COMD(n)		(0x58 + 4 * (n))
#define	 COMD_BYTE_NUM		__BITS(7, 0)
#define	 COMD_ACK_EN		__BIT(8)
#define	 COMD_ACK_EXP		__BIT(9)
#define	 COMD_ACK_VAL		__BIT(10)	/* 1: NACK */
#define	 COMD_OP		__BITS(13, 11)
#define	  OP_WRITE		1
#define	  OP_STOP		2
#define	  OP_READ		3
#define	  OP_END		4
#define	  OP_RSTART		6
#define	 COMD_DONE		__BIT(31)
#define	I2C_SCL_SP_CONF		0x80
#define	 SCL_RST_SLV_EN		__BIT(0)
#define	 SCL_RST_SLV_NUM	__BITS(5, 1)

#define	ESP_I2C_FIFO		32
#define	ESP_I2C_NCMD		8

struct esp_i2c_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	struct i2c_controller	sc_ic;
	u_int			sc_freq;
	uint64_t		sc_srate;	/* source clock, Hz */
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	esp_i2c_match(device_t, cfdata_t, void *);
static void	esp_i2c_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_i2c, sizeof(struct esp_i2c_softc),
	esp_i2c_match, esp_i2c_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-i2c" },
	DEVICE_COMPAT_EOL
};

static void
esp_i2c_update(struct esp_i2c_softc *sc)
{
	WR(sc, I2C_CTR, RD(sc, I2C_CTR) | CTR_CONF_UPGATE);
}

static void
esp_i2c_fifo_reset(struct esp_i2c_softc *sc)
{
	const uint32_t v = RD(sc, I2C_FIFO_CONF) & ~FIFO_CONF_NONFIFO_EN;

	WR(sc, I2C_FIFO_CONF,
	    v | FIFO_CONF_TX_FIFO_RST | FIFO_CONF_RX_FIFO_RST);
	WR(sc, I2C_FIFO_CONF,
	    v & ~(FIFO_CONF_TX_FIFO_RST | FIFO_CONF_RX_FIFO_RST));
}

/* Bus timing for sc_freq, as i2c_ll_master_cal_bus_clk(). */
static void
esp_i2c_set_timing(struct esp_i2c_softc *sc)
{
	const uint64_t src = sc->sc_srate;
	const u_int f = sc->sc_freq;
	const u_int clkm_div = src / (f * 1024) + 1;
	const u_int sclk = src / clkm_div;
	const u_int half = sclk / f / 2;
	const u_int wait_high = f >= 80000 ? half / 2 - 2 : half / 4;
	const u_int high = half - wait_high;
	u_int tout;

	/* Our clock's divider is clkm_div; it's 1 at bus speeds above 39k. */
	(void)clkm_div;

	WR(sc, I2C_SCL_LOW_PERIOD, half - 1);
	WR(sc, I2C_SCL_HIGH_PERIOD, __SHIFTIN(high, SCL_HIGH_PERIOD) |
	    __SHIFTIN(wait_high, SCL_WAIT_HIGH_PERIOD));
	WR(sc, I2C_SDA_HOLD, half / 4 - 1);
	WR(sc, I2C_SDA_SAMPLE, half / 2 - 1);
	WR(sc, I2C_SCL_RSTART_SETUP, half - 1);
	WR(sc, I2C_SCL_STOP_SETUP, half - 1);
	WR(sc, I2C_SCL_START_HOLD, half - 1);
	WR(sc, I2C_SCL_STOP_HOLD, half - 1);
	/* SCL held for more than ~2.5 ms: time out. */
	tout = fls32((uint32_t)(src / 1000000 * 2500));
	WR(sc, I2C_TO, __SHIFTIN(MIN(tout, 0x1f), TO_TIME_OUT_VALUE) |
	    TO_TIME_OUT_EN);
	WR(sc, I2C_FILTER_CFG, __SHIFTIN(7, FILTER_SCL_THRES) |
	    __SHIFTIN(7, FILTER_SDA_THRES) | FILTER_SCL_EN | FILTER_SDA_EN);
	esp_i2c_update(sc);
}

/* Unwedge the controller and the bus: FSM reset and nine SCL pulses. */
static void
esp_i2c_recover(struct esp_i2c_softc *sc)
{
	u_int guard;

	WR(sc, I2C_CTR, RD(sc, I2C_CTR) | CTR_FSM_RST);
	WR(sc, I2C_SCL_SP_CONF, __SHIFTIN(9, SCL_RST_SLV_NUM) |
	    SCL_RST_SLV_EN);
	esp_i2c_update(sc);
	for (guard = 5000; guard > 0 &&
	    (RD(sc, I2C_SCL_SP_CONF) & SCL_RST_SLV_EN) != 0; guard--)
		DELAY(10);
	WR(sc, I2C_SCL_SP_CONF, 0);
	esp_i2c_update(sc);
	esp_i2c_fifo_reset(sc);
	WR(sc, I2C_INT_CLR, INT_ALL);
}

static uint32_t
esp_i2c_cmd(u_int op, u_int n, uint32_t flags)
{
	return __SHIFTIN(op, COMD_OP) | __SHIFTIN(n, COMD_BYTE_NUM) | flags;
}

/* Run the loaded commands; wait for END or STOP. */
static int
esp_i2c_run(struct esp_i2c_softc *sc, size_t nbytes)
{
	/* 10 bit times per byte, plus slack for stretching. */
	u_int timo = 10000 + nbytes * 10 * 1000000 / sc->sc_freq * 2;
	uint32_t st;

	WR(sc, I2C_INT_CLR, INT_ALL);
	esp_i2c_update(sc);
	WR(sc, I2C_CTR, RD(sc, I2C_CTR) | CTR_TRANS_START);

	for (;;) {
		st = RD(sc, I2C_INT_RAW);
		if (st & (INT_NACK | INT_TIME_OUT | INT_ARBITRATION_LOST |
		    INT_SCL_ST_TO | INT_SCL_MAIN_ST_TO))
			break;
		if (st & (INT_TRANS_COMPLETE | INT_END_DETECT))
			return 0;
		if (timo-- == 0)
			return ETIMEDOUT;
		DELAY(1);
	}

	if (st & INT_NACK) {
		/* The controller sends STOP itself; let it finish. */
		for (u_int guard = 2000; guard > 0 &&
		    (RD(sc, I2C_SR) & SR_BUS_BUSY) != 0; guard--)
			DELAY(5);
		if ((RD(sc, I2C_SR) & SR_BUS_BUSY) != 0)
			esp_i2c_recover(sc);
		return EIO;
	}
	esp_i2c_recover(sc);
	return (st & INT_ARBITRATION_LOST) ? EAGAIN : ETIMEDOUT;
}

static int
esp_i2c_exec(void *priv, i2c_op_t op, i2c_addr_t addr, const void *cmdbuf,
    size_t cmdlen, void *buf, size_t len, int flags)
{
	struct esp_i2c_softc * const sc = priv;
	const uint8_t *cmd = cmdbuf, *wdata = buf;
	uint8_t *rdata = buf;
	const bool rd = I2C_OP_READ_P(op);
	/* Write phase: address, command bytes, and data when writing. */
	size_t wtotal = (rd && cmdlen == 0) ? 0 : 1 + cmdlen + (rd ? 0 : len);
	size_t wdone = 0, rdone = 0;
	bool raddr = false;	/* read address sent */
	bool first = true;
	int error;

	if (addr > 0x7f)
		return ENOTSUP;
	if (rd && len == 0)
		return EINVAL;

	if (RD(sc, I2C_SR) & SR_BUS_BUSY)
		esp_i2c_recover(sc);
	esp_i2c_fifo_reset(sc);

	while (wdone < wtotal || (rd && rdone < len)) {
		u_int ncmd = 0, tx = 0, rx = 0;
		bool more = false;

		if (first) {
			WR(sc, I2C_COMD(ncmd++), esp_i2c_cmd(OP_RSTART, 0, 0));
			first = false;
		}

		/* Writes */
		if (wdone < wtotal) {
			const size_t n = MIN(wtotal - wdone, ESP_I2C_FIFO);

			for (size_t i = wdone; i < wdone + n; i++) {
				uint8_t b;

				if (i == 0)
					b = addr << 1;
				else if (i <= cmdlen)
					b = cmd[i - 1];
				else
					b = wdata[i - 1 - cmdlen];
				WR(sc, I2C_DATA, b);
			}
			WR(sc, I2C_COMD(ncmd++),
			    esp_i2c_cmd(OP_WRITE, n, COMD_ACK_EN));
			wdone += n;
			tx += n;
			if (wdone < wtotal)
				more = true;
		}

		/* Reads: (repeated) start, address, then the bytes */
		if (!more && rd) {
			if (!raddr) {
				if (tx + 1 > ESP_I2C_FIFO) {
					more = true;
					goto load;
				}
				if (tx > 0 || wtotal > 0)
					WR(sc, I2C_COMD(ncmd++),
					    esp_i2c_cmd(OP_RSTART, 0, 0));
				WR(sc, I2C_DATA, (addr << 1) | 1);
				WR(sc, I2C_COMD(ncmd++),
				    esp_i2c_cmd(OP_WRITE, 1, COMD_ACK_EN));
				tx++;
				raddr = true;
			}
			const size_t left = len - rdone;
			if (left <= ESP_I2C_FIFO) {
				if (left > 1)
					WR(sc, I2C_COMD(ncmd++),
					    esp_i2c_cmd(OP_READ, left - 1, 0));
				WR(sc, I2C_COMD(ncmd++),
				    esp_i2c_cmd(OP_READ, 1, COMD_ACK_VAL));
				rx = left;
			} else {
				rx = ESP_I2C_FIFO;
				WR(sc, I2C_COMD(ncmd++),
				    esp_i2c_cmd(OP_READ, rx, 0));
				more = true;
			}
		}
load:
		if (more || !I2C_OP_STOP_P(op))
			WR(sc, I2C_COMD(ncmd++), esp_i2c_cmd(OP_END, 0, 0));
		else
			WR(sc, I2C_COMD(ncmd++), esp_i2c_cmd(OP_STOP, 0, 0));
		KASSERT(ncmd <= ESP_I2C_NCMD);

		if ((error = esp_i2c_run(sc, tx + rx)) != 0)
			return error;

		for (u_int i = 0; i < rx; i++) {
			if (__SHIFTOUT(RD(sc, I2C_SR), SR_RXFIFO_CNT) == 0)
				return EIO;
			rdata[rdone++] = RD(sc, I2C_DATA) & 0xff;
		}
	}
	return 0;
}

static int
esp_i2c_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_i2c_attach(device_t parent, device_t self, void *aux)
{
	struct esp_i2c_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct clk *clk, *pclk;
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
	    clk_enable(clk) != 0 ||
	    (pclk = clk_get_parent(clk)) == NULL ||
	    (sc->sc_srate = clk_get_rate(pclk)) == 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}
	if (of_getprop_uint32(phandle, "clock-frequency", &sc->sc_freq) != 0)
		sc->sc_freq = 100000;
	sc->sc_freq = MAX(sc->sc_freq, 40000);
	sc->sc_freq = MIN(sc->sc_freq, 1000000);
	/* Undivided source clock; the timing registers are 9 bits. */
	clk_set_rate(clk, sc->sc_srate);

	fdtbus_pinctrl_set_config(phandle, "default");
	if ((rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		fdtbus_reset_assert(rst);
		fdtbus_reset_deassert(rst);
	}

	/* Master, open-drain lines, no arbitration, MSB first. */
	WR(sc, I2C_CTR, (RD(sc, I2C_CTR) & ~(CTR_SDA_FORCE_OUT |
	    CTR_SCL_FORCE_OUT | CTR_RX_FULL_ACK_LEVEL | CTR_TX_LSB_FIRST |
	    CTR_RX_LSB_FIRST | CTR_ARBITRATION_EN)) | CTR_MS_MODE);
	esp_i2c_fifo_reset(sc);
	esp_i2c_update(sc);
	WR(sc, I2C_INT_ENA, 0);
	WR(sc, I2C_INT_CLR, INT_ALL);
	esp_i2c_set_timing(sc);

	aprint_naive("\n");
	aprint_normal(": I2C controller, %u kHz\n", sc->sc_freq / 1000);

	iic_tag_init(&sc->sc_ic);
	sc->sc_ic.ic_cookie = sc;
	sc->sc_ic.ic_exec = esp_i2c_exec;
	iicbus_attach(self, &sc->sc_ic);
}
