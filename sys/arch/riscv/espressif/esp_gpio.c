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
 * ESP32-S31 GPIO and pin multiplexing.
 *
 * Each of the 62 pads has an IO_MUX word selecting one of eight functions
 * (function 1 = the GPIO matrix) plus its electrical settings.  Through the
 * GPIO matrix any peripheral signal can reach any pad: an output pad selects
 * its source signal, an input signal selects its pad.  The RGMII receive and
 * clock pads (GPIO13-19) and the SD slot 0 pads (GPIO20-25) can instead be
 * handed to the EMAC or the SD host directly (the "CNNT" dedicated path),
 * with their own pad registers.
 *
 * This driver provides pin configurations for the device tree ("pinmux"
 * cells as in Espressif's bindings: pin << 16 | function), GPIO lines for
 * other drivers ("gpios" properties) and gpio(4).  Pins listed in
 * "gpio-reserved-ranges" (flash, PSRAM, USB PHY, console) are left alone.
 *
 * Peripheral signals that go through the GPIO matrix (UART1-3, I2C, SPI,
 * LEDC and MCPWM outputs, TWAI, I2S) can be put on any free pin by name,
 * either in the device tree:
 *
 *		uart1_pins: uart1-pins {
 *			espressif,pins = <10 11>;
 *			espressif,functions = "uart1_txd", "uart1_rxd";
 *		};
 *
 * or at run time, with sysctl:
 *
 *	sysctl -w hw.espgpio0.pin10=uart1_txd
 *	sysctl hw.espgpio0.functions		(the names)
 *
 *	gpio: pinctrl@20582000 {
 *		compatible = "espressif,esp32s31-gpio";
 *		reg = <0x20582000 0x100>,	(IO_MUX)
 *		      <0x20583000 0xe00>,	(GPIO and GPIO matrix)
 *		      <0x20588000 0x400>;	(CNNT IO_MUX)
 *		gpio-controller;
 *		#gpio-cells = <2>;
 *		gpio-reserved-ranges = <26 7>, <33 2>, <58 2>;
 *
 *		emac_pins: emac-pins {
 *			pinmux = <0x50005>, ...;
 *			input-enable;
 *			bias-disable;
 *			drive-strength = <20>;
 *		};
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bitops.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/gpio.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>
#include <dev/gpio/gpiovar.h>

#include <riscv/espressif/esp32s31_var.h>

#define	ESP_GPIO_NPINS		62
#define	CTL_CREATE_PIN_BASE	1000	/* sysctl node numbers of pinN */

/* IO_MUX pad word, one per pad */
#define	IOMUX_PAD(pin)		((pin) * 4)
#define	 IOMUX_MCU_SEL		__BITS(14, 12)	/* function */
#define	 IOMUX_FUN_DRV		__BITS(11, 10)	/* 5, 10, 20, 40 mA */
#define	 IOMUX_FUN_IE		__BIT(9)
#define	 IOMUX_FUN_WPU		__BIT(8)
#define	 IOMUX_FUN_WPD		__BIT(7)
#define	 IOMUX_FUNC_GPIO	1
#define	 IOMUX_FUNC_GMAC	2
#define	 IOMUX_FUNC_SD2		4	/* SD slot 1, GPIO35-40 */

/* GPIO block */
#define	GPIO_OUT_W1TS(bank)	((bank) ? 0x14 : 0x08)
#define	GPIO_OUT_W1TC(bank)	((bank) ? 0x18 : 0x0c)
#define	GPIO_OUT(bank)		((bank) ? 0x10 : 0x04)
#define	GPIO_ENABLE(bank)	((bank) ? 0x40 : 0x34)
#define	GPIO_ENABLE_W1TS(bank)	((bank) ? 0x44 : 0x38)
#define	GPIO_ENABLE_W1TC(bank)	((bank) ? 0x48 : 0x3c)
#define	GPIO_IN(bank)		((bank) ? 0x68 : 0x64)
#define	GPIO_PIN(pin)		(0xf4 + (pin) * 4)
#define	 GPIO_PIN_PAD_DRIVER	__BIT(2)	/* open drain */
#define	GPIO_FUNC_IN_SEL(sig)	(0x2f4 + (sig) * 4)
#define	 GPIO_SIG_IN_SEL	__BIT(9)	/* through the matrix */
#define	GPIO_FUNC_OUT_SEL(pin)	(0xaf4 + (pin) * 4)
#define	 GPIO_OUT_SEL		__BITS(8, 0)
#define	 GPIO_OUT_INV_SEL	__BIT(9)
#define	 GPIO_OE_SEL		__BIT(10)	/* OE from GPIO_ENABLE */
#define	 SIG_GPIO_OUT		256		/* drive from GPIO_OUT */

/* CNNT IO_MUX: dedicated pads GPIO13-25 */
#define	CNNT_PAD(pin)		(((pin) - 13) * 4)
#define	 CNNT_FUN_DRV		__BITS(10, 9)
#define	 CNNT_FUN_IE		__BIT(8)
#define	 CNNT_FUN_WPU		__BIT(7)
#define	 CNNT_FUN_WPD		__BIT(6)
#define	CNNT_CTRL		0x3f4
#define	 CNNT_SDIO_DED_SEL	__BIT(0)
#define	 CNNT_GMAC_DED_SEL	__BIT(1)
#define	CNNT_GMAC_FIRST		13
#define	CNNT_GMAC_LAST		19
#define	CNNT_SDIO_FIRST		20
#define	CNNT_SDIO_LAST		25

/* Matrix signal numbers */
#define	SIG_GMII_MDI_IN		107
#define	SIG_GMII_MDC_OUT	108
#define	SIG_GMII_MDO_OUT	109
#define	SIG_SD_CARD_DETECT_N_1	126	/* slot 0 */
#define	SIG_SD_CARD_WRITE_PRT_1	130
#define	SIG_SD_CARD_DETECT_N_2	127	/* slot 1 */
#define	SIG_SD_CARD_WRITE_PRT_2	131
#define	GPIO_MATRIX_CONST_ZERO	0xc0	/* IN_SEL value: tie the signal low */
#define	GPIO_MATRIX_CONST_ONE	0x80	/* ... high */

/* Function ids of the "pinmux" cells (Espressif dt-bindings) */
#define	FUNC_GMAC		4
#define	FUNC_GMII_MDC		5
#define	FUNC_GMII_MDIO		6
#define	FUNC_SDIO		7	/* not in Espressif's bindings yet */
#define	FUNC_SDIO2		8	/* nor this: SD slot 1's pads */


struct esp_pin_function {
	int	pf_iomux;	/* IO_MUX function, or -1 */
	int	pf_in;		/* matrix input signal, or -1 */
	int	pf_out;		/* matrix output signal, or -1 */
	int	pf_ded;		/* CNNT dedicated path, or 0 */
};

static const struct esp_pin_function esp_pin_functions[] = {
	[FUNC_GMAC] = { IOMUX_FUNC_GMAC, -1, -1, CNNT_GMAC_DED_SEL },
	[FUNC_GMII_MDC] = { IOMUX_FUNC_GPIO, -1, SIG_GMII_MDC_OUT, 0 },
	[FUNC_GMII_MDIO] = { IOMUX_FUNC_GPIO, SIG_GMII_MDI_IN,
			     SIG_GMII_MDO_OUT, 0 },
	/* SD slot 0: CLK, CMD and DAT0-3 on their dedicated pads */
	[FUNC_SDIO] = { IOMUX_FUNC_GPIO, -1, -1, CNNT_SDIO_DED_SEL },
	/* SD slot 1: CLK, CMD and DAT0-3 are IO_MUX function 4 */
	[FUNC_SDIO2] = { IOMUX_FUNC_SD2, -1, -1, 0 },
};

/*
 * Peripheral signals routed through the GPIO matrix, by name (signal
 * numbers from ESP-IDF's gpio_sig_map.h).  Outputs drive the pad all the
 * time (output enable from GPIO_ENABLE), open drain where marked.
 */
#define	MF_OD		0x01	/* open drain */
#define	MF_PU		0x02	/* pull-up */
#define	MF_IDLE0	0x04	/* input reads 0 when not routed */

struct esp_matrix_func {
	const char	*mf_name;
	int16_t		mf_in;		/* input signal, or -1 */
	int16_t		mf_out;		/* output signal, or -1 */
	uint8_t		mf_flags;
};

static const struct esp_matrix_func esp_matrix_funcs[] = {
	{ "gpio",	-1, SIG_GPIO_OUT, 0 },
	/* input buffer, pulls and output off: for the ADC and comparator */
	{ "analog",	-1,  -1, 0 },
	{ "uart1_txd",	-1,  13, 0 },
	{ "uart1_rxd",	13,  -1, MF_PU },
	{ "uart1_rts",	-1,  14, 0 },
	{ "uart1_cts",	14,  -1, MF_IDLE0 },
	{ "uart2_txd",	-1,  16, 0 },
	{ "uart2_rxd",	16,  -1, MF_PU },
	{ "uart2_rts",	-1,  17, 0 },
	{ "uart2_cts",	17,  -1, MF_IDLE0 },
	{ "uart3_txd",	-1, 137, 0 },
	{ "uart3_rxd",	137, -1, MF_PU },
	{ "uart3_rts",	-1, 138, 0 },
	{ "uart3_cts",	138, -1, MF_IDLE0 },
	{ "i2c0_scl",	68,  68, MF_OD | MF_PU },
	{ "i2c0_sda",	69,  69, MF_OD | MF_PU },
	{ "i2c1_scl",	70,  70, MF_OD | MF_PU },
	{ "i2c1_sda",	71,  71, MF_OD | MF_PU },
	{ "spi2_sck",	-1,  53, 0 },
	{ "spi2_mosi",	-1,  55, 0 },
	{ "spi2_miso",	54,  -1, 0 },
	{ "spi2_cs0",	-1,  62, 0 },
	{ "spi2_cs1",	-1,  63, 0 },
	{ "spi2_cs2",	-1,  64, 0 },
	{ "spi2_cs3",	-1,  65, 0 },
	{ "spi3_sck",	-1,  47, 0 },
	{ "spi3_mosi",	-1,  49, 0 },
	{ "spi3_miso",	48,  -1, 0 },
	{ "spi3_cs0",	-1,  52, 0 },
	{ "spi3_cs1",	-1,  46, 0 },
	{ "spi3_cs2",	-1,  45, 0 },
	{ "ledc0_0",	-1, 126, 0 },
	{ "ledc0_1",	-1, 127, 0 },
	{ "ledc0_2",	-1, 128, 0 },
	{ "ledc0_3",	-1, 129, 0 },
	{ "ledc0_4",	-1, 130, 0 },
	{ "ledc0_5",	-1, 131, 0 },
	{ "ledc0_6",	-1, 132, 0 },
	{ "ledc0_7",	-1, 133, 0 },
	{ "ledc1_0",	-1, 230, 0 },
	{ "ledc1_1",	-1, 231, 0 },
	{ "ledc1_2",	-1, 232, 0 },
	{ "ledc1_3",	-1, 233, 0 },
	{ "ledc1_4",	-1, 234, 0 },
	{ "ledc1_5",	-1, 235, 0 },
	{ "ledc1_6",	-1, 236, 0 },
	{ "ledc1_7",	-1, 237, 0 },
	{ "pwm0_0a",	-1,  89, 0 },
	{ "pwm0_0b",	-1,  90, 0 },
	{ "pwm0_1a",	-1,  91, 0 },
	{ "pwm0_1b",	-1,  92, 0 },
	{ "pwm0_2a",	-1,  93, 0 },
	{ "pwm0_2b",	-1,  94, 0 },
	{ "pwm1_0a",	-1,  95, 0 },
	{ "pwm1_0b",	-1,  96, 0 },
	{ "pwm1_1a",	-1,  97, 0 },
	{ "pwm1_1b",	-1,  98, 0 },
	{ "pwm1_2a",	-1,  99, 0 },
	{ "pwm1_2b",	-1, 100, 0 },
	{ "pwm2_0a",	-1,  86, 0 },
	{ "pwm2_0b",	-1,  87, 0 },
	{ "pwm2_1a",	-1,  88, 0 },
	{ "pwm2_1b",	-1, 110, 0 },
	{ "pwm2_2a",	-1, 111, 0 },
	{ "pwm2_2b",	-1, 112, 0 },
	{ "pwm3_0a",	-1, 113, 0 },
	{ "pwm3_0b",	-1, 114, 0 },
	{ "pwm3_1a",	-1, 115, 0 },
	{ "pwm3_1b",	-1, 134, 0 },
	{ "pwm3_2a",	-1, 135, 0 },
	{ "pwm3_2b",	-1, 136, 0 },
	{ "twai0_tx",	-1,  80, 0 },
	{ "twai0_rx",	80,  -1, MF_PU },
	{ "twai1_tx",	-1,  83, 0 },
	{ "twai1_rx",	81,  -1, MF_PU },
	{ "i2s0_mclk",	-1,  26, 0 },
	{ "i2s0_bck",	-1,  25, 0 },
	{ "i2s0_ws",	-1,  27, 0 },
	{ "i2s0_dout",	-1,  28, 0 },
	{ "i2s0_din",	28,  -1, 0 },
	{ "i2s1_mclk",	-1,  32, 0 },
	{ "i2s1_bck",	-1,  31, 0 },
	{ "i2s1_ws",	-1,  33, 0 },
	{ "i2s1_dout",	-1,  34, 0 },
	{ "i2s1_din",	34,  -1, 0 },
};

/* Is the pin one of the dedicated pads of this path? */
static bool
esp_gpio_dedicated(int ded, u_int pin)
{
	switch (ded) {
	case CNNT_GMAC_DED_SEL:
		return pin >= CNNT_GMAC_FIRST && pin <= CNNT_GMAC_LAST;
	case CNNT_SDIO_DED_SEL:
		return pin >= CNNT_SDIO_FIRST && pin <= CNNT_SDIO_LAST;
	default:
		return false;
	}
}

struct esp_gpio_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_iomux;
	bus_space_handle_t	sc_gpio;
	bus_space_handle_t	sc_cnnt;
	kmutex_t		sc_lock;
	uint64_t		sc_reserved;	/* bit per pin */

	struct gpio_chipset_tag	sc_gc;
	gpio_pin_t		sc_pins[ESP_GPIO_NPINS];

	struct sysctllog	*sc_sysctllog;
	char			*sc_funclist;
	size_t			sc_funclistlen;
};

static struct esp_gpio_softc *esp_gpio_sc;


struct esp_gpio_pin {
	struct esp_gpio_softc	*pin_sc;
	u_int			pin_no;
	bool			pin_actlo;
};

#define	RD(sc, h, o)	bus_space_read_4((sc)->sc_bst, (sc)->h, (o))
#define	WR(sc, h, o, v)	bus_space_write_4((sc)->sc_bst, (sc)->h, (o), (v))

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-gpio" },
	DEVICE_COMPAT_EOL
};

static void
esp_gpio_rmw(struct esp_gpio_softc *sc, bus_space_handle_t h, bus_size_t o,
    uint32_t clr, uint32_t set)
{
	const uint32_t v = bus_space_read_4(sc->sc_bst, h, o);

	bus_space_write_4(sc->sc_bst, h, o, (v & ~clr) | set);
}

static bool
esp_gpio_usable(struct esp_gpio_softc *sc, u_int pin)
{
	return pin < ESP_GPIO_NPINS && (sc->sc_reserved & (1ULL << pin)) == 0;
}

/* Make a pad a plain GPIO, input enabled, output per "output". */
static void
esp_gpio_set_mode(struct esp_gpio_softc *sc, u_int pin, int flags)
{
	const u_int bank = pin / 32, bit = 1U << (pin % 32);

	KASSERT(mutex_owned(&sc->sc_lock));

	esp_gpio_rmw(sc, sc->sc_iomux, IOMUX_PAD(pin),
	    IOMUX_MCU_SEL | IOMUX_FUN_WPU | IOMUX_FUN_WPD,
	    __SHIFTIN(IOMUX_FUNC_GPIO, IOMUX_MCU_SEL) | IOMUX_FUN_IE |
	    ((flags & GPIO_PIN_PULLUP) ? IOMUX_FUN_WPU : 0) |
	    ((flags & GPIO_PIN_PULLDOWN) ? IOMUX_FUN_WPD : 0));
	WR(sc, sc_gpio, GPIO_FUNC_OUT_SEL(pin), SIG_GPIO_OUT);
	esp_gpio_rmw(sc, sc->sc_gpio, GPIO_PIN(pin), GPIO_PIN_PAD_DRIVER,
	    (flags & GPIO_PIN_OPENDRAIN) ? GPIO_PIN_PAD_DRIVER : 0);
	if (flags & GPIO_PIN_OUTPUT)
		WR(sc, sc_gpio, GPIO_ENABLE_W1TS(bank), bit);
	else
		WR(sc, sc_gpio, GPIO_ENABLE_W1TC(bank), bit);
}

static int
esp_gpio_read_pin(struct esp_gpio_softc *sc, u_int pin)
{
	const u_int bank = pin / 32, bit = 1U << (pin % 32);

	return (RD(sc, sc_gpio, GPIO_IN(bank)) & bit) != 0;
}

static void
esp_gpio_write_pin(struct esp_gpio_softc *sc, u_int pin, int val)
{
	const u_int bank = pin / 32, bit = 1U << (pin % 32);

	WR(sc, sc_gpio, val ? GPIO_OUT_W1TS(bank) : GPIO_OUT_W1TC(bank), bit);
}

/*
 * Matrix functions
 */

static const struct esp_matrix_func *
esp_gpio_func_lookup(const char *name)
{
	for (u_int i = 0; i < __arraycount(esp_matrix_funcs); i++)
		if (strcmp(esp_matrix_funcs[i].mf_name, name) == 0)
			return &esp_matrix_funcs[i];
	return NULL;
}

/* Put a pin to a function; "gpio" makes it a plain input again. */
static void
esp_gpio_route(struct esp_gpio_softc *sc, u_int pin,
    const struct esp_matrix_func *mf)
{
	const u_int bank = pin / 32, bit = 1U << (pin % 32);

	KASSERT(mutex_owned(&sc->sc_lock));
	KASSERT(esp_gpio_usable(sc, pin));

	/* Let go of the input signals the pin feeds now. */
	for (u_int i = 0; i < __arraycount(esp_matrix_funcs); i++) {
		const struct esp_matrix_func * const f = &esp_matrix_funcs[i];

		if (f->mf_in < 0 || f == mf)
			continue;
		if (RD(sc, sc_gpio, GPIO_FUNC_IN_SEL(f->mf_in)) ==
		    (pin | GPIO_SIG_IN_SEL))
			WR(sc, sc_gpio, GPIO_FUNC_IN_SEL(f->mf_in),
			    ((f->mf_flags & MF_IDLE0) ? GPIO_MATRIX_CONST_ZERO :
			    GPIO_MATRIX_CONST_ONE) | GPIO_SIG_IN_SEL);
	}

	if (mf->mf_out == SIG_GPIO_OUT) {
		esp_gpio_set_mode(sc, pin, GPIO_PIN_INPUT);
		return;
	}

	esp_gpio_rmw(sc, sc->sc_iomux, IOMUX_PAD(pin),
	    IOMUX_MCU_SEL | IOMUX_FUN_IE | IOMUX_FUN_WPU | IOMUX_FUN_WPD,
	    __SHIFTIN(IOMUX_FUNC_GPIO, IOMUX_MCU_SEL) |
	    (mf->mf_in >= 0 ? IOMUX_FUN_IE : 0) |
	    ((mf->mf_flags & MF_PU) ? IOMUX_FUN_WPU : 0));
	esp_gpio_rmw(sc, sc->sc_gpio, GPIO_PIN(pin), GPIO_PIN_PAD_DRIVER,
	    (mf->mf_flags & MF_OD) ? GPIO_PIN_PAD_DRIVER : 0);
	if (mf->mf_out >= 0) {
		WR(sc, sc_gpio, GPIO_FUNC_OUT_SEL(pin),
		    __SHIFTIN(mf->mf_out, GPIO_OUT_SEL) | GPIO_OE_SEL);
		WR(sc, sc_gpio, GPIO_ENABLE_W1TS(bank), bit);
	} else {
		WR(sc, sc_gpio, GPIO_FUNC_OUT_SEL(pin), SIG_GPIO_OUT);
		WR(sc, sc_gpio, GPIO_ENABLE_W1TC(bank), bit);
	}
	if (mf->mf_in >= 0)
		WR(sc, sc_gpio, GPIO_FUNC_IN_SEL(mf->mf_in),
		    pin | GPIO_SIG_IN_SEL);
}

/* What the pin does now. */
static const char *
esp_gpio_func_name(struct esp_gpio_softc *sc, u_int pin)
{
	const u_int out = __SHIFTOUT(RD(sc, sc_gpio, GPIO_FUNC_OUT_SEL(pin)),
	    GPIO_OUT_SEL);
	const u_int fn = __SHIFTOUT(RD(sc, sc_iomux, IOMUX_PAD(pin)),
	    IOMUX_MCU_SEL);

	if (!esp_gpio_usable(sc, pin))
		return "reserved";
	if (fn != IOMUX_FUNC_GPIO)
		return "iomux";
	for (u_int i = 0; i < __arraycount(esp_matrix_funcs); i++) {
		const struct esp_matrix_func * const f = &esp_matrix_funcs[i];

		if (out != SIG_GPIO_OUT && f->mf_out == (int)out)
			return f->mf_name;
		if (out == SIG_GPIO_OUT && f->mf_out < 0 && f->mf_in >= 0 &&
		    RD(sc, sc_gpio, GPIO_FUNC_IN_SEL(f->mf_in)) ==
		    (pin | GPIO_SIG_IN_SEL))
			return f->mf_name;
	}
	if ((RD(sc, sc_iomux, IOMUX_PAD(pin)) & IOMUX_FUN_IE) == 0 &&
	    (RD(sc, sc_gpio, GPIO_ENABLE(pin / 32)) & (1U << (pin % 32))) == 0)
		return "analog";
	return "gpio";
}

/*
 * For other drivers: give a pin a function by name ("analog" for the
 * comparator's pads).
 */
int
esp_gpio_set_function(u_int pin, const char *name)
{
	struct esp_gpio_softc * const sc = esp_gpio_sc;
	const struct esp_matrix_func *mf;

	if (sc == NULL)
		return ENXIO;
	if (!esp_gpio_usable(sc, pin))
		return EBUSY;
	if ((mf = esp_gpio_func_lookup(name)) == NULL)
		return EINVAL;
	mutex_enter(&sc->sc_lock);
	esp_gpio_route(sc, pin, mf);
	mutex_exit(&sc->sc_lock);
	return 0;
}

/* Invert a pin's matrix output (for active-low PWM). */
int
esp_gpio_set_output_invert(u_int pin, bool inv)
{
	struct esp_gpio_softc * const sc = esp_gpio_sc;

	if (sc == NULL)
		return ENXIO;
	if (!esp_gpio_usable(sc, pin))
		return EBUSY;
	mutex_enter(&sc->sc_lock);
	esp_gpio_rmw(sc, sc->sc_gpio, GPIO_FUNC_OUT_SEL(pin), GPIO_OUT_INV_SEL,
	    inv ? GPIO_OUT_INV_SEL : 0);
	mutex_exit(&sc->sc_lock);
	return 0;
}

static int
esp_gpio_sysctl_pin(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_gpio_softc * const sc = node.sysctl_data;
	const u_int pin = node.sysctl_num - CTL_CREATE_PIN_BASE;
	const struct esp_matrix_func *mf;
	char buf[16];
	int error;

	mutex_enter(&sc->sc_lock);
	strlcpy(buf, esp_gpio_func_name(sc, pin), sizeof(buf));
	mutex_exit(&sc->sc_lock);

	node.sysctl_data = buf;
	node.sysctl_size = sizeof(buf);
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;

	if (!esp_gpio_usable(sc, pin))
		return EPERM;
	if ((mf = esp_gpio_func_lookup(buf)) == NULL)
		return EINVAL;
	mutex_enter(&sc->sc_lock);
	esp_gpio_route(sc, pin, mf);
	mutex_exit(&sc->sc_lock);
	return 0;
}

static void
esp_gpio_sysctl_init(struct esp_gpio_softc *sc)
{
	const struct sysctlnode *node;
	char *p;
	size_t len = 1;

	for (u_int i = 0; i < __arraycount(esp_matrix_funcs); i++)
		len += strlen(esp_matrix_funcs[i].mf_name) + 1;
	sc->sc_funclistlen = len;
	sc->sc_funclist = p = kmem_zalloc(len, KM_SLEEP);
	for (u_int i = 0; i < __arraycount(esp_matrix_funcs); i++) {
		const size_t n = strlen(esp_matrix_funcs[i].mf_name);

		memcpy(p, esp_matrix_funcs[i].mf_name, n);
		p[n] = (i + 1 < __arraycount(esp_matrix_funcs)) ? ' ' : '\0';
		p += n + 1;
	}

	if (sysctl_createv(&sc->sc_sysctllog, 0, NULL, &node, 0,
	    CTLTYPE_NODE, device_xname(sc->sc_dev),
	    SYSCTL_DESCR("pin functions"), NULL, 0, NULL, 0,
	    CTL_HW, CTL_CREATE, CTL_EOL) != 0)
		return;
	sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL, CTLFLAG_READONLY,
	    CTLTYPE_STRING, "functions",
	    SYSCTL_DESCR("functions a pin can be given"), NULL, 0,
	    sc->sc_funclist, 0, CTL_CREATE, CTL_EOL);
	for (u_int pin = 0; pin < ESP_GPIO_NPINS; pin++) {
		char name[8];

		if (!esp_gpio_usable(sc, pin))
			continue;
		snprintf(name, sizeof(name), "pin%u", pin);
		sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL,
		    CTLFLAG_READWRITE, CTLTYPE_STRING, name,
		    SYSCTL_DESCR("pin function"), esp_gpio_sysctl_pin, 0,
		    (void *)sc, 16, CTL_CREATE_PIN_BASE + pin, CTL_EOL);
	}
}

/*
 * Pin configuration ("pinctrl-0 = <&emac_pins>").
 */

static int
esp_gpio_drive_field(int phandle)
{
	u_int mA;

	if (of_getprop_uint32(phandle, "drive-strength", &mA) != 0)
		return -1;
	return mA <= 5 ? 0 : mA <= 10 ? 1 : mA <= 20 ? 2 : 3;
}

/* espressif,pins = <pin ...>; espressif,functions = "name", ... */
static int
esp_gpio_set_config_matrix(struct esp_gpio_softc *sc, int phandle)
{
	const uint32_t *pins;
	const char *fn;
	int plen, i;

	pins = fdtbus_get_prop(phandle, "espressif,pins", &plen);
	for (i = 0; pins != NULL && plen >= 4; i++, plen -= 4, pins++) {
		const u_int pin = be32toh(*pins);
		const struct esp_matrix_func *mf;

		fn = fdtbus_get_string_index(phandle,
		    "espressif,functions", i);
		mf = fn != NULL ? esp_gpio_func_lookup(fn) : NULL;
		if (mf == NULL || !esp_gpio_usable(sc, pin)) {
			aprint_error_dev(sc->sc_dev, "%s: bad pin %u (%s)\n",
			    fdtbus_get_string(phandle, "name"), pin,
			    fn != NULL ? fn : "?");
			continue;
		}
		mutex_enter(&sc->sc_lock);
		esp_gpio_route(sc, pin, mf);
		mutex_exit(&sc->sc_lock);
	}
	return 0;
}

static int
esp_gpio_set_config(device_t dev, const void *data, size_t len)
{
	struct esp_gpio_softc * const sc = device_private(dev);
	const int phandle = fdtbus_get_phandle_from_native(be32dec(data));
	const uint32_t *pinmux;
	int plen;

	if (of_hasprop(phandle, "espressif,pins"))
		return esp_gpio_set_config_matrix(sc, phandle);

	pinmux = fdtbus_get_prop(phandle, "pinmux", &plen);
	if (pinmux == NULL)
		return ENOENT;

	const bool ie = of_hasprop(phandle, "input-enable");
	const bool pu = of_hasprop(phandle, "bias-pull-up");
	const bool pd = of_hasprop(phandle, "bias-pull-down");
	const int drv = esp_gpio_drive_field(phandle);

	mutex_enter(&sc->sc_lock);
	for (; plen >= 4; plen -= 4, pinmux++) {
		const uint32_t cell = be32toh(*pinmux);
		const u_int pin = cell >> 16, fn = cell & 0xffff;
		const struct esp_pin_function *pf;

		if (fn >= __arraycount(esp_pin_functions) ||
		    esp_pin_functions[fn].pf_iomux == 0 ||
		    !esp_gpio_usable(sc, pin)) {
			aprint_error_dev(dev, "%s: bad pinmux %#x\n",
			    fdtbus_get_string(phandle, "name"), cell);
			continue;
		}
		pf = &esp_pin_functions[fn];

		if (pf->pf_in >= 0)
			WR(sc, sc_gpio, GPIO_FUNC_IN_SEL(pf->pf_in),
			    pin | GPIO_SIG_IN_SEL);
		if (pf->pf_out >= 0)
			WR(sc, sc_gpio, GPIO_FUNC_OUT_SEL(pin), pf->pf_out);

		if (esp_gpio_dedicated(pf->pf_ded, pin)) {
			/* The peripheral takes the pad: CNNT pad register. */
			esp_gpio_rmw(sc, sc->sc_cnnt, CNNT_CTRL, 0, pf->pf_ded);
			if (pf->pf_ded == CNNT_SDIO_DED_SEL) {
				/*
				 * Slot 0 has no card-detect or write-protect
				 * pads: say "card present, writable", as
				 * Espressif's driver does.
				 */
				WR(sc, sc_gpio,
				    GPIO_FUNC_IN_SEL(SIG_SD_CARD_DETECT_N_1),
				    GPIO_MATRIX_CONST_ZERO | GPIO_SIG_IN_SEL);
				WR(sc, sc_gpio,
				    GPIO_FUNC_IN_SEL(SIG_SD_CARD_WRITE_PRT_1),
				    GPIO_MATRIX_CONST_ZERO | GPIO_SIG_IN_SEL);
			}
			esp_gpio_rmw(sc, sc->sc_cnnt, CNNT_PAD(pin),
			    CNNT_FUN_IE | CNNT_FUN_WPU | CNNT_FUN_WPD |
			    (drv >= 0 ? CNNT_FUN_DRV : 0),
			    (ie ? CNNT_FUN_IE : 0) | (pu ? CNNT_FUN_WPU : 0) |
			    (pd ? CNNT_FUN_WPD : 0) |
			    (drv >= 0 ? __SHIFTIN(drv, CNNT_FUN_DRV) : 0));
		} else {
			if (fn == FUNC_SDIO2) {
				/* No CD or WP pads routed: as slot 0. */
				WR(sc, sc_gpio,
				    GPIO_FUNC_IN_SEL(SIG_SD_CARD_DETECT_N_2),
				    GPIO_MATRIX_CONST_ZERO | GPIO_SIG_IN_SEL);
				WR(sc, sc_gpio,
				    GPIO_FUNC_IN_SEL(SIG_SD_CARD_WRITE_PRT_2),
				    GPIO_MATRIX_CONST_ZERO | GPIO_SIG_IN_SEL);
			}
			esp_gpio_rmw(sc, sc->sc_iomux, IOMUX_PAD(pin),
			    IOMUX_MCU_SEL | IOMUX_FUN_IE | IOMUX_FUN_WPU |
			    IOMUX_FUN_WPD | (drv >= 0 ? IOMUX_FUN_DRV : 0),
			    __SHIFTIN(pf->pf_iomux, IOMUX_MCU_SEL) |
			    (ie ? IOMUX_FUN_IE : 0) | (pu ? IOMUX_FUN_WPU : 0) |
			    (pd ? IOMUX_FUN_WPD : 0) |
			    (drv >= 0 ? __SHIFTIN(drv, IOMUX_FUN_DRV) : 0));
		}
	}
	mutex_exit(&sc->sc_lock);

	return 0;
}

static const struct fdtbus_pinctrl_controller_func esp_gpio_pinctrl_funcs = {
	.set_config = esp_gpio_set_config,
};

/*
 * GPIO lines for other drivers ("reset-gpios = <&gpio 7 GPIO_ACTIVE_LOW>").
 */

static void *
esp_gpio_fdt_acquire(device_t dev, const void *data, size_t len, int flags)
{
	struct esp_gpio_softc * const sc = device_private(dev);
	const uint32_t * const cells = data;
	struct esp_gpio_pin *gpin;

	if (len != 12)
		return NULL;
	const u_int pin = be32toh(cells[1]);
	if (!esp_gpio_usable(sc, pin))
		return NULL;

	gpin = kmem_zalloc(sizeof(*gpin), KM_SLEEP);
	gpin->pin_sc = sc;
	gpin->pin_no = pin;
	gpin->pin_actlo = (be32toh(cells[2]) & 1) != 0;

	mutex_enter(&sc->sc_lock);
	esp_gpio_set_mode(sc, pin, flags);
	mutex_exit(&sc->sc_lock);

	return gpin;
}

static void
esp_gpio_fdt_release(device_t dev, void *priv)
{
	struct esp_gpio_pin * const gpin = priv;
	struct esp_gpio_softc * const sc = gpin->pin_sc;

	mutex_enter(&sc->sc_lock);
	esp_gpio_set_mode(sc, gpin->pin_no, GPIO_PIN_INPUT);
	mutex_exit(&sc->sc_lock);
	kmem_free(gpin, sizeof(*gpin));
}

static int
esp_gpio_fdt_read(device_t dev, void *priv, bool raw)
{
	struct esp_gpio_pin * const gpin = priv;
	int val = esp_gpio_read_pin(gpin->pin_sc, gpin->pin_no);

	if (!raw && gpin->pin_actlo)
		val = !val;
	return val;
}

static void
esp_gpio_fdt_write(device_t dev, void *priv, int val, bool raw)
{
	struct esp_gpio_pin * const gpin = priv;

	if (!raw && gpin->pin_actlo)
		val = !val;
	esp_gpio_write_pin(gpin->pin_sc, gpin->pin_no, val);
}

static const struct fdtbus_gpio_controller_func esp_gpio_fdt_funcs = {
	.acquire = esp_gpio_fdt_acquire,
	.release = esp_gpio_fdt_release,
	.read = esp_gpio_fdt_read,
	.write = esp_gpio_fdt_write,
};

/*
 * gpio(4)
 */

static int
esp_gpio_gc_read(void *priv, int pin)
{
	return esp_gpio_read_pin(priv, pin);
}

static void
esp_gpio_gc_write(void *priv, int pin, int val)
{
	esp_gpio_write_pin(priv, pin, val);
}

static void
esp_gpio_gc_ctl(void *priv, int pin, int flags)
{
	struct esp_gpio_softc * const sc = priv;

	if (!esp_gpio_usable(sc, pin))
		return;
	mutex_enter(&sc->sc_lock);
	esp_gpio_set_mode(sc, pin, flags);
	mutex_exit(&sc->sc_lock);
}

static void
esp_gpio_attach_ports(struct esp_gpio_softc *sc)
{
	struct gpio_chipset_tag * const gc = &sc->sc_gc;
	struct gpiobus_attach_args gba;

	gc->gp_cookie = sc;
	gc->gp_pin_read = esp_gpio_gc_read;
	gc->gp_pin_write = esp_gpio_gc_write;
	gc->gp_pin_ctl = esp_gpio_gc_ctl;

	for (u_int pin = 0; pin < ESP_GPIO_NPINS; pin++) {
		gpio_pin_t * const gp = &sc->sc_pins[pin];

		gp->pin_num = pin;
		if (!esp_gpio_usable(sc, pin))
			continue;	/* no caps: unusable */
		gp->pin_caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
		    GPIO_PIN_OPENDRAIN | GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN;
		gp->pin_state = esp_gpio_read_pin(sc, pin);
	}

	memset(&gba, 0, sizeof(gba));
	gba.gba_gc = gc;
	gba.gba_pins = sc->sc_pins;
	gba.gba_npins = ESP_GPIO_NPINS;
	config_found(sc->sc_dev, &gba, NULL, CFARGS_NONE);
}

static int
esp_gpio_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_gpio_attach(device_t parent, device_t self, void *aux)
{
	struct esp_gpio_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_space_handle_t *bshs[] = { &sc->sc_iomux, &sc->sc_gpio,
	    &sc->sc_cnnt };
	const uint32_t *rr;
	bus_addr_t addr;
	bus_size_t size;
	int len;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	for (u_int i = 0; i < __arraycount(bshs); i++) {
		if (fdtbus_get_reg(phandle, i, &addr, &size) != 0 ||
		    bus_space_map(sc->sc_bst, addr, size, 0, bshs[i]) != 0) {
			aprint_error(": couldn't map registers\n");
			return;
		}
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_VM);

	/* gpio-reserved-ranges = <first count>, ... */
	rr = fdtbus_get_prop(phandle, "gpio-reserved-ranges", &len);
	for (; rr != NULL && len >= 8; len -= 8, rr += 2) {
		const u_int first = be32toh(rr[0]), count = be32toh(rr[1]);

		for (u_int pin = first; pin < first + count &&
		    pin < ESP_GPIO_NPINS; pin++)
			sc->sc_reserved |= 1ULL << pin;
	}

	esp_gpio_sc = sc;

	/* The IO_MUX itself must be clocked and out of reset. */
	fdtbus_clock_enable(phandle, "apb", false);
	fdtbus_clock_enable(phandle, "iomux", false);

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 GPIO, %d pins (%d reserved)\n",
	    ESP_GPIO_NPINS, popcount64(sc->sc_reserved));

	fdtbus_register_gpio_controller(self, phandle, &esp_gpio_fdt_funcs);
	for (int child = OF_child(phandle); child; child = OF_peer(child)) {
		if (of_hasprop(child, "pinmux") ||
		    of_hasprop(child, "espressif,pins"))
			fdtbus_register_pinctrl_config(self, child,
			    &esp_gpio_pinctrl_funcs);
	}

	esp_gpio_sysctl_init(sc);
	esp_gpio_attach_ports(sc);
}

CFATTACH_DECL_NEW(esp_gpio, sizeof(struct esp_gpio_softc),
	esp_gpio_match, esp_gpio_attach, NULL, NULL);
