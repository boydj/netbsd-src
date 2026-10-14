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
 * ESP32-S31 peripheral clock gates and resets.
 *
 * The bits live in two register blocks, HP_SYS_CLKRST and CNNT_SYS (the
 * "connectivity" block: EMAC, USB OTG-HS, SDIO).  Clock and reset ids are
 * those of Espressif's device tree bindings (dt-bindings/clock/esp32s31.h,
 * dt-bindings/reset/esp32s31.h), so device trees can use the same cells.
 * Espressif's bindings have no SD/MMC host yet; its clocks and reset use
 * the next free ids (clocks 11 and 12, reset 5).
 *
 *	clkrst: clock-controller@20587000 {
 *		compatible = "espressif,esp32s31-clkrst";
 *		reg = <0x20587000 0x400>,	(HP_SYS_CLKRST)
 *		      <0x20359000 0x100>;	(CNNT_SYS)
 *		clocks = <&xtal_clk>, <&mpll_clk>;
 *		clock-names = "xtal", "mpll";
 *		#clock-cells = <1>;
 *		#reset-cells = <1>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <dev/clk/clk_backend.h>
#include <dev/fdt/fdtvar.h>

/* Clock ids (Espressif dt-bindings) */
#define	CLK_IOMUX_APB		0
#define	CLK_IOMUX		1
#define	CLK_EMAC_SYS		6
#define	CLK_EMAC_REF		7
#define	CLK_USB_OTGHS_BUS	8
#define	CLK_USB_OTGHS_PHYREF	9
#define	CLK_EMAC_PTP		10
#define	CLK_SDMMC_SYS		11	/* SD/MMC host bus clock ("biu") */
#define	CLK_SDIO_LS		12	/* SD/MMC card clock source ("ciu") */
#define	CLK_NUM			13

/* Reset ids */
#define	RST_IOMUX		0
#define	RST_EMAC		3
#define	RST_USB_OTGHS		4
#define	RST_SDMMC		5
#define	RST_NUM			6

/* HP_SYS_CLKRST */
#define	HP_USB_OTGHS_CTRL0	0x0ac
#define	 USB_OTGHS_APB_CLK_EN	__BIT(0)
#define	 USB_OTGHS_SYS_CLK_EN	__BIT(1)
#define	HP_SDIO_HOST_CTRL0	0x0c0
#define	 SDMMC_SYS_CLK_EN	__BIT(0)
#define	 SDIO_HS_MODE		__BIT(1)
#define	 SDIO_LS_CLK_SRC_SEL	__BIT(2)	/* 0: MPLL */
#define	 SDIO_LS_CLK_EN		__BIT(3)
#define	HP_SDIO_HOST_FUNC_CTRL0	0x0c4
#define	 SDIO_LS_CLK_EDGE_CFG_UPDATE __BIT(0)
#define	 SDIO_LS_CLK_EDGE_L	__BITS(4, 1)
#define	 SDIO_LS_CLK_EDGE_H	__BITS(8, 5)
#define	 SDIO_LS_CLK_EDGE_N	__BITS(12, 9)
#define	 SDIO_LS_SLF_CLK_EDGE_SEL __BITS(15, 13)
#define	 SDIO_LS_DRV_CLK_EDGE_SEL __BITS(18, 16)
#define	 SDIO_LS_SAM_CLK_EDGE_SEL __BITS(21, 19)
#define	 SDIO_LS_SLF_CLK_EN	__BIT(22)
#define	 SDIO_LS_DRV_CLK_EN	__BIT(23)
#define	 SDIO_LS_SAM_CLK_EN	__BIT(24)
#define	HP_EMAC_CTRL0		0x0c8
#define	 EMAC_SYS_CLK_EN	__BIT(0)
#define	HP_IOMUX_CTRL0		0x15c
#define	 IOMUX_APB_CLK_EN	__BIT(0)
#define	 IOMUX_RST_EN		__BIT(1)
#define	 IOMUX_CLK_EN		__BIT(4)

/* CNNT_SYS */
#define	CNNT_USB_OTG20_CTRL	0x030
#define	 USB_OTG20_PHYREF_CLK_EN __BIT(27)
#define	 USB_OTGHS_RST_EN	(__BIT(29) | __BIT(30) | __BIT(31))
#define	CNNT_HP_SDMMC_CTRL	0x038
#define	 SDMMC_RST_EN		__BIT(30)
#define	CNNT_HP_EMAC_CTRL	0x03c
#define	 EMAC_RST_EN		__BIT(1)
#define	CNNT_HP_EMAC_REF_CTRL	0x040
#define	 EMAC_REF_CLK_SEL	__BITS(1, 0)	/* 0: MPLL */
#define	 EMAC_REF_CLK_EN	__BIT(2)
#define	 EMAC_REF_CLK_DIV	__BITS(15, 8)	/* divider - 1 */
#define	CNNT_HP_EMAC_PTP_CTRL	0x054
#define	 EMAC_PTP_REF_CLK_EN	__BIT(0)

enum { MAP_HP, MAP_CNNT, NMAPS };
enum { PARENT_XTAL, PARENT_MPLL, PARENT_12M };

struct esp_clk {
	struct clk	ec_clk;
	uint8_t		ec_map;
	uint8_t		ec_parent;
	bus_size_t	ec_reg;
	uint32_t	ec_gate;
	uint32_t	ec_div;		/* divider field (value = div - 1) */
	uint32_t	ec_mux;		/* parent select field, set to 0 */
	bool		ec_sdio;	/* the SD card clock edge generator */
};

struct esp_rst {
	uint8_t		er_map;
	bus_size_t	er_reg;
	uint32_t	er_mask;	/* set: in reset */
};

struct esp_clkrst_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh[NMAPS];
	kmutex_t		sc_lock;
	struct clk_domain	sc_clkdom;
	u_int			sc_parent_rate[3];
	struct esp_clk		sc_clks[CLK_NUM];
};

#define	CLK(_id, _name, _map, _parent, _reg, _gate, _div, _mux)		\
	[_id] = { .ec_clk = { .name = (_name) }, .ec_map = (_map),	\
		  .ec_parent = (_parent), .ec_reg = (_reg),		\
		  .ec_gate = (_gate), .ec_div = (_div), .ec_mux = (_mux) }

static const struct esp_clk esp_clks[CLK_NUM] = {
	CLK(CLK_IOMUX_APB, "iomux_apb", MAP_HP, PARENT_XTAL, HP_IOMUX_CTRL0,
	    IOMUX_APB_CLK_EN, 0, 0),
	CLK(CLK_IOMUX, "iomux", MAP_HP, PARENT_XTAL, HP_IOMUX_CTRL0,
	    IOMUX_CLK_EN, 0, 0),
	CLK(CLK_EMAC_SYS, "emac_sys", MAP_HP, PARENT_XTAL, HP_EMAC_CTRL0,
	    EMAC_SYS_CLK_EN, 0, 0),
	CLK(CLK_EMAC_REF, "emac_ref", MAP_CNNT, PARENT_MPLL,
	    CNNT_HP_EMAC_REF_CTRL, EMAC_REF_CLK_EN, EMAC_REF_CLK_DIV,
	    EMAC_REF_CLK_SEL),
	CLK(CLK_USB_OTGHS_BUS, "usb_otghs_bus", MAP_HP, PARENT_XTAL,
	    HP_USB_OTGHS_CTRL0, USB_OTGHS_APB_CLK_EN | USB_OTGHS_SYS_CLK_EN,
	    0, 0),
	CLK(CLK_USB_OTGHS_PHYREF, "usb_otghs_phyref", MAP_CNNT, PARENT_12M,
	    CNNT_USB_OTG20_CTRL, USB_OTG20_PHYREF_CLK_EN, 0, 0),
	CLK(CLK_EMAC_PTP, "emac_ptp", MAP_CNNT, PARENT_XTAL,
	    CNNT_HP_EMAC_PTP_CTRL, EMAC_PTP_REF_CLK_EN, 0, 0),
	CLK(CLK_SDMMC_SYS, "sdmmc_sys", MAP_HP, PARENT_XTAL,
	    HP_SDIO_HOST_CTRL0, SDMMC_SYS_CLK_EN, 0, 0),
	[CLK_SDIO_LS] = { .ec_clk = { .name = "sdio_ls" }, .ec_map = MAP_HP,
			  .ec_parent = PARENT_MPLL, .ec_reg = HP_SDIO_HOST_CTRL0,
			  .ec_gate = SDIO_LS_CLK_EN,
			  .ec_mux = SDIO_LS_CLK_SRC_SEL | SDIO_HS_MODE,
			  .ec_sdio = true },
};

static const struct esp_rst esp_rsts[RST_NUM] = {
	[RST_IOMUX] = { MAP_HP, HP_IOMUX_CTRL0, IOMUX_RST_EN },
	[RST_EMAC] = { MAP_CNNT, CNNT_HP_EMAC_CTRL, EMAC_RST_EN },
	[RST_USB_OTGHS] = { MAP_CNNT, CNNT_USB_OTG20_CTRL, USB_OTGHS_RST_EN },
	[RST_SDMMC] = { MAP_CNNT, CNNT_HP_SDMMC_CTRL, SDMMC_RST_EN },
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-clkrst" },
	DEVICE_COMPAT_EOL
};

static void
esp_clkrst_update(struct esp_clkrst_softc *sc, u_int map, bus_size_t reg,
    uint32_t clr, uint32_t set)
{
	uint32_t val;

	mutex_enter(&sc->sc_lock);
	val = bus_space_read_4(sc->sc_bst, sc->sc_bsh[map], reg);
	val = (val & ~clr) | set;
	bus_space_write_4(sc->sc_bst, sc->sc_bsh[map], reg, val);
	mutex_exit(&sc->sc_lock);
}

static uint32_t
esp_clkrst_read(struct esp_clkrst_softc *sc, u_int map, bus_size_t reg)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh[map], reg);
}

/* clk(9) backend */

static struct clk *
esp_clk_get(void *priv, const char *name)
{
	struct esp_clkrst_softc * const sc = priv;

	for (u_int id = 0; id < CLK_NUM; id++) {
		if (sc->sc_clks[id].ec_clk.name != NULL &&
		    strcmp(sc->sc_clks[id].ec_clk.name, name) == 0)
			return &sc->sc_clks[id].ec_clk;
	}
	return NULL;
}

static void
esp_clk_put(void *priv, struct clk *clk)
{
}

static u_int
esp_clk_get_rate(void *priv, struct clk *clk)
{
	struct esp_clkrst_softc * const sc = priv;
	struct esp_clk * const ec = (struct esp_clk *)clk;
	const u_int prate = sc->sc_parent_rate[ec->ec_parent];

	if (ec->ec_sdio) {
		const uint32_t f = esp_clkrst_read(sc, MAP_HP,
		    HP_SDIO_HOST_FUNC_CTRL0);

		if ((esp_clkrst_read(sc, MAP_HP, HP_SDIO_HOST_CTRL0) &
		    SDIO_HS_MODE) != 0)
			return prate;
		return prate / (__SHIFTOUT(f, SDIO_LS_CLK_EDGE_N) + 1);
	}
	if (ec->ec_div == 0)
		return prate;
	return prate / (__SHIFTOUT(esp_clkrst_read(sc, ec->ec_map, ec->ec_reg),
	    ec->ec_div) + 1);
}

/*
 * The SD card clock comes from an edge generator dividing MPLL by 2..16;
 * the host's own CLKDIV divides further.  Pick the smallest divider that
 * doesn't exceed the rate asked for.
 */
static int
esp_clk_sdio_set_rate(struct esp_clkrst_softc *sc, u_int prate, u_int rate)
{
	u_int div;

	div = howmany(prate, rate);
	div = MAX(div, 2);
	div = MIN(div, 16);
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_CTRL0, SDIO_HS_MODE, 0);
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_FUNC_CTRL0,
	    SDIO_LS_CLK_EDGE_H | SDIO_LS_CLK_EDGE_N | SDIO_LS_CLK_EDGE_L,
	    __SHIFTIN(div / 2 - 1, SDIO_LS_CLK_EDGE_H) |
	    __SHIFTIN(div - 1, SDIO_LS_CLK_EDGE_N) |
	    __SHIFTIN(div - 1, SDIO_LS_CLK_EDGE_L));
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_FUNC_CTRL0, 0,
	    SDIO_LS_CLK_EDGE_CFG_UPDATE);
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_FUNC_CTRL0,
	    SDIO_LS_CLK_EDGE_CFG_UPDATE, 0);
	return 0;
}

static void
esp_clk_sdio_enable(struct esp_clkrst_softc *sc)
{
	/* Drive on the falling edge, sample and feed back on the rising. */
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_FUNC_CTRL0,
	    SDIO_LS_DRV_CLK_EDGE_SEL | SDIO_LS_SAM_CLK_EDGE_SEL |
	    SDIO_LS_SLF_CLK_EDGE_SEL,
	    SDIO_LS_DRV_CLK_EN | SDIO_LS_SAM_CLK_EN | SDIO_LS_SLF_CLK_EN |
	    __SHIFTIN(1, SDIO_LS_DRV_CLK_EDGE_SEL));
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_FUNC_CTRL0, 0,
	    SDIO_LS_CLK_EDGE_CFG_UPDATE);
	esp_clkrst_update(sc, MAP_HP, HP_SDIO_HOST_FUNC_CTRL0,
	    SDIO_LS_CLK_EDGE_CFG_UPDATE, 0);
}

static int
esp_clk_set_rate(void *priv, struct clk *clk, u_int rate)
{
	struct esp_clkrst_softc * const sc = priv;
	struct esp_clk * const ec = (struct esp_clk *)clk;
	const u_int prate = sc->sc_parent_rate[ec->ec_parent];
	const u_int maxdiv = __SHIFTOUT_MASK(ec->ec_div) + 1;
	u_int div;

	if (ec->ec_sdio && rate != 0)
		return esp_clk_sdio_set_rate(sc, prate, rate);
	if (ec->ec_div == 0 || rate == 0)
		return rate == esp_clk_get_rate(priv, clk) ? 0 : EINVAL;

	div = (prate + rate / 2) / rate;
	if (div < 1 || div > maxdiv)
		return EINVAL;
	esp_clkrst_update(sc, ec->ec_map, ec->ec_reg, ec->ec_div,
	    __SHIFTIN(div - 1, ec->ec_div));
	return 0;
}

static int
esp_clk_enable(void *priv, struct clk *clk)
{
	struct esp_clkrst_softc * const sc = priv;
	struct esp_clk * const ec = (struct esp_clk *)clk;

	/* Select the (only) parent, then ungate. */
	esp_clkrst_update(sc, ec->ec_map, ec->ec_reg, ec->ec_mux, 0);
	if (ec->ec_sdio)
		esp_clk_sdio_enable(sc);
	esp_clkrst_update(sc, ec->ec_map, ec->ec_reg, 0, ec->ec_gate);
	return 0;
}

static int
esp_clk_disable(void *priv, struct clk *clk)
{
	struct esp_clkrst_softc * const sc = priv;
	struct esp_clk * const ec = (struct esp_clk *)clk;

	esp_clkrst_update(sc, ec->ec_map, ec->ec_reg, ec->ec_gate, 0);
	return 0;
}

static const struct clk_funcs esp_clk_funcs = {
	.get = esp_clk_get,
	.put = esp_clk_put,
	.get_rate = esp_clk_get_rate,
	.set_rate = esp_clk_set_rate,
	.enable = esp_clk_enable,
	.disable = esp_clk_disable,
};

static struct clk *
esp_clkrst_clock_decode(device_t dev, int cc_phandle, const void *data,
    size_t len)
{
	struct esp_clkrst_softc * const sc = device_private(dev);
	u_int id;

	if (len != 4)
		return NULL;
	id = be32dec(data);
	if (id >= CLK_NUM || sc->sc_clks[id].ec_clk.name == NULL)
		return NULL;
	return &sc->sc_clks[id].ec_clk;
}

static const struct fdtbus_clock_controller_func esp_clkrst_clock_funcs = {
	.decode = esp_clkrst_clock_decode,
};

/* Resets */

static void *
esp_clkrst_reset_acquire(device_t dev, const void *data, size_t len)
{
	u_int id;

	if (len != 4)
		return NULL;
	id = be32dec(data);
	if (id >= RST_NUM || esp_rsts[id].er_mask == 0)
		return NULL;
	return __UNCONST(&esp_rsts[id]);
}

static void
esp_clkrst_reset_release(device_t dev, void *priv)
{
}

static int
esp_clkrst_reset_assert(device_t dev, void *priv)
{
	struct esp_clkrst_softc * const sc = device_private(dev);
	const struct esp_rst * const er = priv;

	esp_clkrst_update(sc, er->er_map, er->er_reg, 0, er->er_mask);
	return 0;
}

static int
esp_clkrst_reset_deassert(device_t dev, void *priv)
{
	struct esp_clkrst_softc * const sc = device_private(dev);
	const struct esp_rst * const er = priv;

	esp_clkrst_update(sc, er->er_map, er->er_reg, er->er_mask, 0);
	return 0;
}

static const struct fdtbus_reset_controller_func esp_clkrst_reset_funcs = {
	.acquire = esp_clkrst_reset_acquire,
	.release = esp_clkrst_reset_release,
	.reset_assert = esp_clkrst_reset_assert,
	.reset_deassert = esp_clkrst_reset_deassert,
};

static u_int
esp_clkrst_parent_rate(int phandle, const char *name, u_int def)
{
	struct clk * const clk = fdtbus_clock_get(phandle, name);
	const u_int rate = clk != NULL ? clk_get_rate(clk) : 0;

	return rate != 0 ? rate : def;
}

static int
esp_clkrst_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_clkrst_attach(device_t parent, device_t self, void *aux)
{
	struct esp_clkrst_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	for (u_int map = 0; map < NMAPS; map++) {
		if (fdtbus_get_reg(phandle, map, &addr, &size) != 0 ||
		    bus_space_map(sc->sc_bst, addr, size, 0,
			&sc->sc_bsh[map]) != 0) {
			aprint_error(": couldn't map registers\n");
			return;
		}
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_HIGH);

	sc->sc_parent_rate[PARENT_XTAL] =
	    esp_clkrst_parent_rate(phandle, "xtal", 40000000);
	sc->sc_parent_rate[PARENT_MPLL] =
	    esp_clkrst_parent_rate(phandle, "mpll", 500000000);
	sc->sc_parent_rate[PARENT_12M] = 12000000;

	sc->sc_clkdom.name = device_xname(self);
	sc->sc_clkdom.funcs = &esp_clk_funcs;
	sc->sc_clkdom.priv = sc;
	memcpy(sc->sc_clks, esp_clks, sizeof(sc->sc_clks));
	for (u_int id = 0; id < CLK_NUM; id++) {
		if (sc->sc_clks[id].ec_clk.name == NULL)
			continue;
		sc->sc_clks[id].ec_clk.domain = &sc->sc_clkdom;
		clk_attach(&sc->sc_clks[id].ec_clk);
	}

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 clocks and resets\n");

	fdtbus_register_clock_controller(self, phandle,
	    &esp_clkrst_clock_funcs);
	fdtbus_register_reset_controller(self, phandle,
	    &esp_clkrst_reset_funcs);
}

CFATTACH_DECL_NEW(esp_clkrst, sizeof(struct esp_clkrst_softc),
	esp_clkrst_match, esp_clkrst_attach, NULL, NULL);
