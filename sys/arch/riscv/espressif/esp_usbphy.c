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
 * ESP32-S31 USB 2.0 OTG-HS PHY: an FDT PHY provider for the dwc2
 * controller.  Powering it on selects the 12 MHz reference, takes the
 * PHY out of forced suspend with its PLL running, and pulls D+/D- down
 * for host mode.  The sequence follows Espressif's Linux driver
 * (phy-esp32s31-usb).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

/* HP_ALIVE_SYS */
#define	ALIVE_USB_CTRL			0x024
#define	 USB_PHY_DM_PULLDOWN		__BIT(2)
#define	 USB_PHY_DP_PULLDOWN		__BIT(3)
#define	ALIVE_USB_OTGHS_CTRL		0x0b0
#define	 USB_PHY_PLL_FORCE_EN		__BIT(0)
#define	 USB_PHY_PLL_EN			__BIT(1)
#define	 USB_PHY_SUSPENDM_FORCE_EN	__BIT(2)
#define	 USB_PHY_SUSPENDM		__BIT(3)
#define	 USB_PHY_OTG_SUSPENDM		__BIT(7)
#define	 USB_PHY_REFCLK_MODE		__BIT(8)

/* CNNT_SYS */
#define	CNNT_USB_OTG20_CTRL		0x030
#define	 USB_PHYREF_CLK_SRC_SEL		__BITS(26,25)
#define	 USB_PHYREF_CLK_SRC_12M		0

/* The PHY's own window: low-speed support in FC register 6. */
#define	UTMI_FC_06			0x018
#define	 UTMI_LS_PAR_EN			__BIT(0)
#define	 UTMI_LS_KPALV_EN		__BIT(3)

/* The PHY PLL needs a moment to lock before the controller resets. */
#define	USB_PHY_PLL_LOCK_US		50

struct esp_usbphy_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_utmi;
	bus_space_handle_t	sc_alive;
	bus_space_handle_t	sc_cnnt;
	struct clk		*sc_ref;
	bool			sc_host;
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "esp,esp32s31-usb-phy" },
	{ .compat = "espressif,esp32s31-usb-phy" },
	DEVICE_COMPAT_EOL
};

static void
esp_usbphy_update(struct esp_usbphy_softc *sc, bus_space_handle_t bsh,
    bus_size_t reg, uint32_t mask, uint32_t val)
{
	uint32_t v;

	v = bus_space_read_4(sc->sc_bst, bsh, reg);
	bus_space_write_4(sc->sc_bst, bsh, reg, (v & ~mask) | (val & mask));
}

static void *
esp_usbphy_acquire(device_t dev, const void *data, size_t len)
{
	struct esp_usbphy_softc * const sc = device_private(dev);

	if (len != 0)
		return NULL;
	return sc;
}

static void
esp_usbphy_release(device_t dev, void *priv)
{
}

static int
esp_usbphy_enable(device_t dev, void *priv, bool enable)
{
	struct esp_usbphy_softc * const sc = device_private(dev);
	const uint32_t run = USB_PHY_PLL_FORCE_EN | USB_PHY_PLL_EN |
	    USB_PHY_SUSPENDM_FORCE_EN | USB_PHY_SUSPENDM | USB_PHY_OTG_SUSPENDM;
	const uint32_t pulldown = USB_PHY_DP_PULLDOWN | USB_PHY_DM_PULLDOWN;
	int error;

	if (!enable) {
		/* Re-suspend, keeping the force enables so the state is ours. */
		esp_usbphy_update(sc, sc->sc_alive, ALIVE_USB_OTGHS_CTRL,
		    USB_PHY_SUSPENDM | USB_PHY_OTG_SUSPENDM | USB_PHY_PLL_EN,
		    0);
		esp_usbphy_update(sc, sc->sc_alive, ALIVE_USB_CTRL, pulldown,
		    0);
		if (sc->sc_ref != NULL)
			clk_disable(sc->sc_ref);
		return 0;
	}

	/* The 12 MHz reference, on both sides that describe it. */
	esp_usbphy_update(sc, sc->sc_cnnt, CNNT_USB_OTG20_CTRL,
	    USB_PHYREF_CLK_SRC_SEL,
	    __SHIFTIN(USB_PHYREF_CLK_SRC_12M, USB_PHYREF_CLK_SRC_SEL));
	esp_usbphy_update(sc, sc->sc_alive, ALIVE_USB_OTGHS_CTRL,
	    USB_PHY_REFCLK_MODE, USB_PHY_REFCLK_MODE);

	if (sc->sc_ref != NULL && (error = clk_enable(sc->sc_ref)) != 0)
		return error;

	/*
	 * Out of reset the PHY is force-suspended with its PLL under
	 * hardware control; take over both.  SUSPENDM is active low.
	 */
	esp_usbphy_update(sc, sc->sc_alive, ALIVE_USB_OTGHS_CTRL, run, run);
	delay(USB_PHY_PLL_LOCK_US);

	/* D+/D- pulled down: the host's idle line state. */
	esp_usbphy_update(sc, sc->sc_alive, ALIVE_USB_CTRL, pulldown,
	    sc->sc_host ? pulldown : 0);

	esp_usbphy_update(sc, sc->sc_utmi, UTMI_FC_06,
	    UTMI_LS_PAR_EN | UTMI_LS_KPALV_EN,
	    UTMI_LS_PAR_EN | UTMI_LS_KPALV_EN);

	return 0;
}

static const struct fdtbus_phy_controller_func esp_usbphy_funcs = {
	.acquire = esp_usbphy_acquire,
	.release = esp_usbphy_release,
	.enable = esp_usbphy_enable,
};

/* Map the first register window of the node a phandle property names. */
static int
esp_usbphy_map_syscon(struct esp_usbphy_softc *sc, int phandle,
    const char *prop, const char *altprop, bus_space_handle_t *bshp)
{
	bus_addr_t addr;
	bus_size_t size;
	int node;

	node = fdtbus_get_phandle(phandle, prop);
	if (node <= 0)
		node = fdtbus_get_phandle(phandle, altprop);
	if (node <= 0 || fdtbus_get_reg(node, 0, &addr, &size) != 0)
		return ENOENT;
	return bus_space_map(sc->sc_bst, addr, size, 0, bshp);
}

static int
esp_usbphy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_usbphy_attach(device_t parent, device_t self, void *aux)
{
	struct esp_usbphy_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	/*
	 * The controller's dr_mode decides the line state; only host mode
	 * is supported by NetBSD's dwc2, so pull down unless told otherwise.
	 */
	sc->sc_host = true;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_utmi) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	if (esp_usbphy_map_syscon(sc, phandle, "esp,hp-alive-sys",
	    "espressif,hp-alive-sys", &sc->sc_alive) != 0) {
		aprint_error(": couldn't map HP_ALIVE_SYS\n");
		return;
	}
	if (esp_usbphy_map_syscon(sc, phandle, "esp,cnnt-sys",
	    "espressif,cnnt-sys", &sc->sc_cnnt) != 0) {
		aprint_error(": couldn't map CNNT_SYS\n");
		return;
	}
	sc->sc_ref = fdtbus_clock_get(phandle, "ref");

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 USB 2.0 PHY\n");

	fdtbus_register_phy_controller(self, phandle, &esp_usbphy_funcs);
}

CFATTACH_DECL_NEW(esp_usbphy, sizeof(struct esp_usbphy_softc),
    esp_usbphy_match, esp_usbphy_attach, NULL, NULL);
