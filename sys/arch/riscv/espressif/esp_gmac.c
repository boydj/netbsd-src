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
 * ESP32-S31 Ethernet: a Synopsys DesignWare GMAC 3.70a (dwc_gmac(4))
 * behind RGMII, with the SoC glue -- clocks, reset, pads, RGMII clocking,
 * PHY reset and the factory MAC address in eFuse -- done here.  The
 * register sequence follows Espressif's Linux glue (dwmac-esp32s31).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/rndsource.h>
#include <sys/systm.h>

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>

#include <dev/mii/miivar.h>

#include <dev/ic/dwc_gmac_reg.h>
#include <dev/ic/dwc_gmac_var.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/dwc_gmac_fdt_subr.h>

/* CNNT_SYS: RGMII pad clocking and PHY interface selection */
#define	CNNT_HP_EMAC_RMII_PAD_CTRL	0x044
#define	 EMAC_RMII_PAD_CLK_EN		__BIT(1)
#define	 EMAC_RMII_PAD_CLK_INV_EN	__BIT(2)
#define	CNNT_HP_EMAC_RMII_CTRL		0x048
#define	 EMAC_RMII_CLK_SEL		__BIT(0)
#define	 EMAC_RMII_CLK_EN		__BIT(1)
#define	 EMAC_RMII_PAD_OUT_CLK		__BIT(2)
#define	CNNT_HP_EMAC_RX_CTRL		0x04c
#define	 EMAC_RX_PAD_CLK_EN		__BIT(0)
#define	 EMAC_RX_PAD_CLK_INV_EN		__BIT(1)
#define	 EMAC_RX_CLK_SEL		__BIT(2)
#define	 EMAC_RX_180_CLK_EN		__BIT(3)
#define	CNNT_HP_EMAC_TX_CTRL		0x050
#define	 EMAC_TX_PAD_CLK_EN		__BIT(0)
#define	 EMAC_TX_CLK_SEL		__BIT(2)
#define	 EMAC_TX_180_CLK_EN		__BIT(3)
#define	CNNT_GMAC_CTRL0			0x060
#define	 GMAC_PHY_INTF_SEL		__BITS(4, 2)
#define	 GMAC_PHY_INTF_SEL_RGMII	1

/* eFuse: factory MAC address, byte-reversed, in BLK1 */
#define	EFUSE_MAC_FACTORY_ADDR		0x20715050

struct esp_gmac_softc {
	struct dwc_gmac_softc	sc_base;
	bus_space_handle_t	sc_cnnt;
	struct clk		*sc_clk_ref;
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-dwmac" },
	{ .compat = "esp,esp32s31-dwmac" },
	DEVICE_COMPAT_EOL
};

static void
esp_gmac_cnnt_update(struct esp_gmac_softc *esc, bus_size_t reg,
    uint32_t mask, uint32_t val)
{
	bus_space_tag_t bst = esc->sc_base.sc_bst;
	uint32_t v;

	v = bus_space_read_4(bst, esc->sc_cnnt, reg);
	bus_space_write_4(bst, esc->sc_cnnt, reg, (v & ~mask) | val);
}

/* RGMII: transmit clock of 2.5, 25 or 125 MHz for 10, 100, 1000 Mbit/s. */
static void
esp_gmac_set_speed(struct dwc_gmac_softc *sc, int speed)
{
	struct esp_gmac_softc * const esc = (struct esp_gmac_softc *)sc;
	u_int rate;

	switch (speed) {
	case IFM_10_T:
		rate = 2500000;
		break;
	case IFM_100_TX:
		rate = 25000000;
		break;
	case IFM_1000_T:
		rate = 125000000;
		break;
	default:
		return;
	}
	if (esc->sc_clk_ref != NULL && clk_set_rate(esc->sc_clk_ref, rate) != 0)
		aprint_error_dev(sc->sc_dev, "couldn't set TX clock to %u Hz\n",
		    rate);
}

/*
 * Factory MAC address from eFuse, or a fixed local one.  As in ESP-IDF, the
 * eFuse address is the base of four: Wi-Fi station, soft-AP, Bluetooth,
 * then Ethernet (the radio co-processor's interfaces use the first ones).
 */
static void
esp_gmac_get_enaddr(struct dwc_gmac_softc *sc, uint8_t enaddr[ETHER_ADDR_LEN])
{
	bus_space_handle_t bsh;
	uint8_t raw[8];

	memset(raw, 0, sizeof(raw));
	if (bus_space_map(sc->sc_bst, EFUSE_MAC_FACTORY_ADDR, 8, 0, &bsh) == 0) {
		const uint32_t lo = bus_space_read_4(sc->sc_bst, bsh, 0);
		const uint32_t hi = bus_space_read_4(sc->sc_bst, bsh, 4);

		le32enc(raw, lo);
		le32enc(raw + 4, hi);
		bus_space_unmap(sc->sc_bst, bsh, 8);
	}
	for (u_int i = 0; i < ETHER_ADDR_LEN; i++)
		enaddr[i] = raw[ETHER_ADDR_LEN - 1 - i];

	const bool zero = (enaddr[0] | enaddr[1] | enaddr[2] | enaddr[3] |
	    enaddr[4] | enaddr[5]) == 0;
	if (zero || ETHER_IS_MULTICAST(enaddr)) {
		/*
		 * Every chip has a factory address, so this is an emulator
		 * (the random number generators aren't up yet anyway): use
		 * a fixed, locally administered one.
		 */
		static const uint8_t fallback[ETHER_ADDR_LEN] =
		    { 0x02, 0xe5, 0x53, 0x31, 0x00, 0x01 };

		memcpy(enaddr, fallback, ETHER_ADDR_LEN);
		aprint_normal_dev(sc->sc_dev,
		    "no factory MAC address in eFuse, using a local one\n");
		return;
	}
	enaddr[ETHER_ADDR_LEN - 1] += 3;
}

/* The PHY address from the device tree's mdio node, or MII_PHY_ANY. */
static int
esp_gmac_phy_addr(int phandle)
{
	int mdio, phy;
	u_int addr;

	if ((mdio = of_find_firstchild_byname(phandle, "mdio")) <= 0)
		return MII_PHY_ANY;
	for (phy = OF_child(mdio); phy > 0; phy = OF_peer(phy)) {
		if (of_getprop_uint32(phy, "reg", &addr) == 0 && addr < 32)
			return addr;
	}
	return MII_PHY_ANY;
}

/* Pulse the PHY's reset line from the mdio child node, if it has one. */
static void
esp_gmac_phy_reset(int phandle)
{
	struct fdtbus_gpio_pin *pin;
	u_int assert_us = 10000, deassert_us = 100000;
	int mdio, phy;

	if ((mdio = of_find_firstchild_byname(phandle, "mdio")) <= 0)
		return;
	for (phy = OF_child(mdio); phy > 0; phy = OF_peer(phy)) {
		pin = fdtbus_gpio_acquire(phy, "reset-gpios", GPIO_PIN_OUTPUT);
		if (pin == NULL)
			continue;
		of_getprop_uint32(phy, "reset-assert-us", &assert_us);
		of_getprop_uint32(phy, "reset-deassert-us", &deassert_us);
		fdtbus_gpio_write(pin, 1);	/* asserted */
		delay(assert_us);
		fdtbus_gpio_write(pin, 0);
		delay(deassert_us);
		/* keep the pin: releasing it would float the reset line */
	}
}

static int
esp_gmac_intr(void *arg)
{
	return dwc_gmac_intr(arg);
}

static int
esp_gmac_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_gmac_attach(device_t parent, device_t self, void *aux)
{
	struct esp_gmac_softc * const esc = device_private(self);
	struct dwc_gmac_softc * const sc = &esc->sc_base;
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	uint8_t enaddr[ETHER_ADDR_LEN];
	struct fdtbus_reset *rst;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	int cnnt;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	sc->sc_dmat = faa->faa_dmat;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	cnnt = fdtbus_get_phandle(phandle, "espressif,cnnt-sys");
	if (cnnt <= 0 || fdtbus_get_reg(cnnt, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &esc->sc_cnnt) != 0) {
		aprint_error(": couldn't map CNNT_SYS\n");
		return;
	}

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": couldn't decode interrupt\n");
		return;
	}

	/* Clocks: system (register interface), PTP, RGMII reference. */
	if (fdtbus_clock_enable(phandle, "stmmaceth", true) != 0 ||
	    fdtbus_clock_enable(phandle, "ptp_ref", false) != 0) {
		aprint_error(": couldn't enable clocks\n");
		return;
	}
	esc->sc_clk_ref = fdtbus_clock_get(phandle, "ref");
	if (esc->sc_clk_ref == NULL || clk_enable(esc->sc_clk_ref) != 0) {
		aprint_error(": couldn't enable the reference clock\n");
		return;
	}
	clk_set_rate(esc->sc_clk_ref, 25000000);

	rst = fdtbus_reset_get(phandle, "stmmaceth");
	if (rst != NULL) {
		fdtbus_reset_assert(rst);
		delay(10);
		fdtbus_reset_deassert(rst);
	}

	fdtbus_pinctrl_set_config(phandle, "default");

	/* RGMII, with the receive clock from the pad, as Espressif sets it. */
	esp_gmac_cnnt_update(esc, CNNT_GMAC_CTRL0, GMAC_PHY_INTF_SEL,
	    __SHIFTIN(GMAC_PHY_INTF_SEL_RGMII, GMAC_PHY_INTF_SEL));
	esp_gmac_cnnt_update(esc, CNNT_HP_EMAC_RMII_PAD_CTRL,
	    EMAC_RMII_PAD_CLK_EN | EMAC_RMII_PAD_CLK_INV_EN, 0);
	esp_gmac_cnnt_update(esc, CNNT_HP_EMAC_RMII_CTRL,
	    EMAC_RMII_CLK_SEL | EMAC_RMII_CLK_EN | EMAC_RMII_PAD_OUT_CLK,
	    EMAC_RMII_PAD_OUT_CLK);
	esp_gmac_cnnt_update(esc, CNNT_HP_EMAC_RX_CTRL,
	    EMAC_RX_PAD_CLK_INV_EN | EMAC_RX_PAD_CLK_EN | EMAC_RX_CLK_SEL |
	    EMAC_RX_180_CLK_EN,
	    EMAC_RX_PAD_CLK_EN | EMAC_RX_CLK_SEL | EMAC_RX_180_CLK_EN);
	esp_gmac_cnnt_update(esc, CNNT_HP_EMAC_TX_CTRL,
	    EMAC_TX_PAD_CLK_EN | EMAC_TX_CLK_SEL | EMAC_TX_180_CLK_EN,
	    EMAC_TX_180_CLK_EN);

	esp_gmac_phy_reset(phandle);

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 Ethernet (DesignWare GMAC)\n");

	esp_gmac_get_enaddr(sc, enaddr);
	prop_dictionary_set_data(device_properties(self), "mac-address",
	    enaddr, ETHER_ADDR_LEN);

	sc->sc_set_speed = esp_gmac_set_speed;
	dwc_gmac_preattach_fdt(phandle, sc);
	/*
	 * The S31's GMAC is built with enhanced descriptors.  Like
	 * Espressif's Linux glue, don't rely on the HW feature register
	 * to say so.
	 */
	sc->sc_flags |= DWC_GMAC_ENH_DESC;

	/*
	 * MDC must stay at or below 2.5MHz; the largest CSR clock divider is
	 * safe whatever the register interface runs at.
	 */
	if (dwc_gmac_attach(sc, esp_gmac_phy_addr(phandle),
	    GMAC_MII_CLK_250_300M_DIV124) != 0)
		return;

	if (fdtbus_intr_establish_xname(phandle, 0, IPL_NET, FDT_INTR_MPSAFE,
	    esp_gmac_intr, sc, device_xname(self)) == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);
}

CFATTACH_DECL_NEW(esp_gmac, sizeof(struct esp_gmac_softc),
	esp_gmac_match, esp_gmac_attach, NULL, NULL);
