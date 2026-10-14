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
 * the next free ids (clocks 11 and 12, reset 5), and so do the rest of the
 * peripherals here (clocks 13 and up, resets 6 and up; see esp32s31.dts).
 *
 * Enabling a clock also selects its parent (XTAL, for everything that can
 * have it: the PLL-derived 80 and 160 MHz references get gated off by
 * ESP-IDF, the RC oscillator is imprecise) and, for blocks with their own
 * RAM (UART FIFOs, CAN buffers, LEDC gamma RAM), powers that RAM up in
 * HP_SYSTEM.  Clocks with a selectable parent report the rate of the one
 * selected, so a console UART left on another source by the boot firmware
 * keeps its baud rate.
 *
 *	clkrst: clock-controller@20587000 {
 *		compatible = "espressif,esp32s31-clkrst";
 *		reg = <0x20587000 0x400>,	(HP_SYS_CLKRST)
 *		      <0x20359000 0x100>,	(CNNT_SYS)
 *		      <0x20586000 0x400>,	(HP_SYSTEM: memory power)
 *		      <0x20710000 0x100>;	(LP_PERI_CLKRST)
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
#define	CLK_UART0		13
#define	CLK_UART1		14
#define	CLK_UART2		15
#define	CLK_UART3		16
#define	CLK_I2C0		17
#define	CLK_I2C1		18
#define	CLK_GPSPI2		19
#define	CLK_GPSPI3		20
#define	CLK_LEDC0		21
#define	CLK_LEDC1		22
#define	CLK_TWAI0		23
#define	CLK_TWAI1		24
#define	CLK_CRYPTO		25	/* crypto bus + security clocks */
#define	CLK_AES			26
#define	CLK_SHA			27
#define	CLK_RSA			28
#define	CLK_ECC			29
#define	CLK_TIMG0		30
#define	CLK_TIMG1		31
#define	CLK_TSENS		32
#define	CLK_ADC			33
#define	CLK_LP_I2C		34
#define	CLK_MCPWM0		35
#define	CLK_MCPWM1		36
#define	CLK_MCPWM2		37
#define	CLK_MCPWM3		38
#define	CLK_DS			39
#define	CLK_HMAC		40
#define	CLK_ECDSA		41
#define	CLK_ZERO_DET		42	/* analog comparator */
#define	CLK_I2S0		43	/* bus clocks; I2S does its own dividers */
#define	CLK_I2S1		44
#define	CLK_AHB_PDMA		45
#define	CLK_NUM			46

/* Reset ids */
#define	RST_IOMUX		0
#define	RST_EMAC		3
#define	RST_USB_OTGHS		4
#define	RST_SDMMC		5
#define	RST_UART1		6
#define	RST_UART2		7
#define	RST_UART3		8
#define	RST_I2C0		9
#define	RST_I2C1		10
#define	RST_GPSPI2		11
#define	RST_GPSPI3		12
#define	RST_LEDC0		13
#define	RST_LEDC1		14
#define	RST_TWAI0		15
#define	RST_TWAI1		16
#define	RST_CRYPTO		17
#define	RST_AES			18
#define	RST_SHA			19
#define	RST_RSA			20
#define	RST_ECC			21
#define	RST_TIMG1		22
#define	RST_TSENS		23
#define	RST_ADC			24
#define	RST_LP_I2C		25
#define	RST_MCPWM0		26
#define	RST_MCPWM1		27
#define	RST_MCPWM2		28
#define	RST_MCPWM3		29
#define	RST_SEC			30
#define	RST_DS			31
#define	RST_HMAC		32
#define	RST_ECDSA		33
#define	RST_ZERO_DET		34
#define	RST_I2S0		35
#define	RST_I2S1		36
#define	RST_NUM			37

/* HP_SYS_CLKRST */
#define	HP_CRYPTO_CTRL0		0x058
#define	 CRYPTO_SYS_CLK_EN	__BIT(0)
#define	 CRYPTO_RST_EN		__BIT(1)
#define	 CRYPTO_SEC_CLK_EN	__BIT(2)
#define	 CRYPTO_SEC_RST_EN	__BIT(3)
#define	 CRYPTO_AES_CLK_EN	__BIT(4)
#define	 CRYPTO_AES_RST_EN	__BIT(5)
#define	 CRYPTO_SHA_CLK_EN	__BIT(6)
#define	 CRYPTO_SHA_RST_EN	__BIT(7)
#define	 CRYPTO_RSA_CLK_EN	__BIT(8)
#define	 CRYPTO_RSA_RST_EN	__BIT(9)
#define	 CRYPTO_DS_CLK_EN	__BIT(10)
#define	 CRYPTO_DS_RST_EN	__BIT(11)
#define	 CRYPTO_ECC_CLK_EN	__BIT(12)
#define	 CRYPTO_ECC_RST_EN	__BIT(13)
#define	 CRYPTO_HMAC_CLK_EN	__BIT(14)
#define	 CRYPTO_HMAC_RST_EN	__BIT(15)
#define	 CRYPTO_ECDSA_CLK_EN	__BIT(16)
#define	 CRYPTO_ECDSA_RST_EN	__BIT(17)
#define	 CRYPTO_CLK_SRC_SEL	__BITS(21, 20)	/* XTAL, RC_FAST, PLL_F240M */
#define	HP_GPSPI2_CTRL0		0x06c
#define	HP_GPSPI3_CTRL0		0x070
#define	 GPSPI_SYS_CLK_EN	__BIT(0)
#define	 GPSPI_APB_CLK_EN	__BIT(1)
#define	 GPSPI_RST_EN		__BIT(2)
#define	 GPSPI_CLK_SRC_SEL	__BITS(5, 4)	/* XTAL, RC_FAST, BBPLL */
#define	 GPSPI_HS_CLK_EN	__BIT(6)
#define	 GPSPI_HS_CLK_DIV_NUM	__BITS(14, 7)
#define	 GPSPI_MST_CLK_EN	__BIT(23)
#define	HP_UART_CTRL0(n)	(0x088 + 4 * (n))
#define	 UART_SYS_CLK_EN	__BIT(0)
#define	 UART_APB_CLK_EN	__BIT(1)
#define	 UART_APB_RST_EN	__BIT(3)
#define	 UART_CLK_SRC_SEL	__BITS(6, 5)	/* XTAL, RC_FAST, REF_F80M */
#define	 UART_CLK_EN		__BIT(7)
#define	 UART_SCLK_DIV_NUM	__BITS(15, 8)
#define	HP_I2C_CTRL0(n)		(0x0dc + 4 * (n))
#define	 I2C_APB_CLK_EN		__BIT(0)
#define	 I2C_RST_EN		__BIT(1)
#define	 I2C_CLK_SRC_SEL	__BIT(3)	/* XTAL, RC_FAST */
#define	 I2C_CLK_EN		__BIT(4)
#define	 I2C_CLK_DIV_NUM	__BITS(12, 5)
#define	HP_TWAI_CTRL0(n)	(0x10c + 4 * (n))
#define	 TWAI_APB_CLK_EN	__BIT(0)
#define	 TWAI_CLK_SRC_SEL	__BITS(2, 1)	/* XTAL, -, REF_F80M */
#define	 TWAI_CLK_EN		__BIT(3)
#define	 TWAI_RST_EN		__BIT(4)
#define	HP_TIMERGRP1_CTRL0	0x11c
#define	HP_TIMERGRP0_CTRL0	0x114
#define	 TIMERGRP_APB_CLK_EN	__BIT(0)
#define	 TIMERGRP_RST_EN	__BIT(1)
#define	 TIMERGRP_WDT_CLK_EN	__BIT(11)
#define	HP_MCPWM_CTRL0(n)	(0x124 + 4 * (n))
#define	HP_AHB_PDMA_CTRL0	0x07c
#define	 AHB_PDMA_SYS_CLK_EN	__BIT(0)
#define	HP_I2S_CTRL0(n)		(0x0e4 + 0x14 * (n))
#define	 I2S_APB_CLK_EN		__BIT(0)
#define	 I2S_APB_RST_EN		__BIT(1)
#define	HP_ZERO_DET_CTRL0	0x1b8
#define	 ZERO_DET_APB_CLK_EN	__BIT(0)
#define	 ZERO_DET_APB_RST_EN	__BIT(1)
#define	 ZERO_DET_CLK_EN	__BIT(2)
#define	 ZERO_DET_CORE_RST_EN	__BIT(3)
#define	 ZERO_DET_CLK_SRC_SEL	__BITS(6, 5)
#define	 ZERO_DET_CLK_DIV_NUM	__BITS(14, 7)
#define	 MCPWM_APB_CLK_EN	__BIT(0)
#define	 MCPWM_RST_EN		__BIT(1)
#define	 MCPWM_CLK_SRC_SEL	__BITS(4, 3)	/* XTAL, RC_FAST, PLL_F160M */
#define	 MCPWM_CLK_EN		__BIT(5)
#define	 MCPWM_CLK_DIV_NUM	__BITS(13, 6)
#define	HP_LEDC_CTRL0		0x148
#define	 LEDC0_APB_CLK_EN	__BIT(0)
#define	 LEDC0_RST_EN		__BIT(1)
#define	 LEDC0_CLK_SRC_SEL	__BITS(4, 3)	/* XTAL, RC_FAST, REF_F80M */
#define	 LEDC0_CLK_EN		__BIT(5)
#define	 LEDC1_APB_CLK_EN	__BIT(6)
#define	 LEDC1_RST_EN		__BIT(7)
#define	 LEDC1_CLK_SRC_SEL	__BITS(10, 9)
#define	 LEDC1_CLK_EN		__BIT(11)
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

/* HP_SYSTEM: peripheral memory power (lp_mode [1:0], lp_en [2], force [3]) */
#define	HPSYS_UART_MEM(n)	((n) == 3 ? 0x1f0 : 0x28c + 4 * (n))
#define	HPSYS_CAN0_MEM		0x228
#define	HPSYS_CAN1_MEM		0x2a4
#define	HPSYS_LEDC0_MEM		0x208
#define	HPSYS_LEDC1_MEM		0x298
#define	 HPSYS_MEM_ON		0x2	/* lp_mode 2, no power-down */

/* LP_PERI_CLKRST */
#define	LPPERI_I2C_CTRL		0x014
#define	LPPERI_ADC_CTRL		0x020
#define	LPPERI_TSENS_CTRL	0x030
#define	 LPPERI_DIV_NUM		__BITS(7, 0)	/* ADC */
#define	 LPPERI_I2C_DIV_NUM	__BITS(27, 20)
#define	 LPPERI_CLK_SEL		__BITS(29, 28)
#define	 LPPERI_CLK_EN		__BIT(30)
#define	 LPPERI_RST_EN		__BIT(31)

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

enum { MAP_HP, MAP_CNNT, MAP_HPSYS, MAP_LPPERI, NMAPS };
#define	NMAPS_REQUIRED	2		/* older device trees: HP and CNNT */
enum {
	PARENT_XTAL, PARENT_MPLL, PARENT_12M, PARENT_RC_FAST, PARENT_F80M,
	PARENT_F160M, PARENT_F240M, PARENT_PLL480, NPARENTS,
	PARENT_NONE = 0xff
};

/* Parent selected by each value of a mux field */
static const uint8_t mux_xtal_rcfast_f80m[4] =
    { PARENT_XTAL, PARENT_RC_FAST, PARENT_F80M, PARENT_NONE };
static const uint8_t mux_xtal_rcfast[4] =
    { PARENT_XTAL, PARENT_RC_FAST, PARENT_NONE, PARENT_NONE };
static const uint8_t mux_xtal_rcfast_pll480[4] =
    { PARENT_XTAL, PARENT_RC_FAST, PARENT_PLL480, PARENT_NONE };
static const uint8_t mux_xtal_rcfast_f160m[4] =
    { PARENT_XTAL, PARENT_RC_FAST, PARENT_F160M, PARENT_NONE };
static const uint8_t mux_xtal_rcfast_f240m[4] =
    { PARENT_XTAL, PARENT_RC_FAST, PARENT_F240M, PARENT_NONE };
static const uint8_t mux_twai[4] =
    { PARENT_XTAL, PARENT_NONE, PARENT_F80M, PARENT_NONE };
static const uint8_t mux_lp_rcfast_xtal[4] =
    { PARENT_RC_FAST, PARENT_XTAL, PARENT_NONE, PARENT_NONE };

struct esp_clk {
	struct clk	ec_clk;
	uint8_t		ec_map;
	uint8_t		ec_parent;	/* the one enabling selects */
	bus_size_t	ec_reg;
	uint32_t	ec_gate;
	uint32_t	ec_div;		/* divider field (value = div - 1) */
	uint32_t	ec_mux;		/* parent select field */
	const uint8_t	*ec_muxsel;	/* parents by mux value; NULL: mux 0 */
	uint16_t	ec_mem;		/* HP_SYSTEM memory power reg, or 0 */
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
	bool			sc_mapped[NMAPS];
	struct clk_domain	sc_clkdom;
	u_int			sc_parent_rate[NPARENTS];
	struct clk		sc_pclks[NPARENTS];
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

	/* Peripherals with a selectable source (enabling picks XTAL) */
#define	MCLK(_id, _name, _map, _reg, _gate, _div, _mux, _muxsel, _mem)	\
	[_id] = { .ec_clk = { .name = (_name) }, .ec_map = (_map),	\
		  .ec_parent = PARENT_XTAL, .ec_reg = (_reg),		\
		  .ec_gate = (_gate), .ec_div = (_div), .ec_mux = (_mux),	\
		  .ec_muxsel = (_muxsel), .ec_mem = (_mem) }
#define	UART_GATES	(UART_SYS_CLK_EN | UART_APB_CLK_EN | UART_CLK_EN)
	MCLK(CLK_UART0, "uart0", MAP_HP, HP_UART_CTRL0(0), UART_GATES,
	    UART_SCLK_DIV_NUM, UART_CLK_SRC_SEL, mux_xtal_rcfast_f80m,
	    HPSYS_UART_MEM(0)),
	MCLK(CLK_UART1, "uart1", MAP_HP, HP_UART_CTRL0(1), UART_GATES,
	    UART_SCLK_DIV_NUM, UART_CLK_SRC_SEL, mux_xtal_rcfast_f80m,
	    HPSYS_UART_MEM(1)),
	MCLK(CLK_UART2, "uart2", MAP_HP, HP_UART_CTRL0(2), UART_GATES,
	    UART_SCLK_DIV_NUM, UART_CLK_SRC_SEL, mux_xtal_rcfast_f80m,
	    HPSYS_UART_MEM(2)),
	MCLK(CLK_UART3, "uart3", MAP_HP, HP_UART_CTRL0(3), UART_GATES,
	    UART_SCLK_DIV_NUM, UART_CLK_SRC_SEL, mux_xtal_rcfast_f80m,
	    HPSYS_UART_MEM(3)),
	MCLK(CLK_I2C0, "i2c0", MAP_HP, HP_I2C_CTRL0(0),
	    I2C_APB_CLK_EN | I2C_CLK_EN, I2C_CLK_DIV_NUM, I2C_CLK_SRC_SEL,
	    mux_xtal_rcfast, 0),
	MCLK(CLK_I2C1, "i2c1", MAP_HP, HP_I2C_CTRL0(1),
	    I2C_APB_CLK_EN | I2C_CLK_EN, I2C_CLK_DIV_NUM, I2C_CLK_SRC_SEL,
	    mux_xtal_rcfast, 0),
#define	GPSPI_GATES	(GPSPI_SYS_CLK_EN | GPSPI_APB_CLK_EN | GPSPI_HS_CLK_EN | \
			 GPSPI_MST_CLK_EN)
	MCLK(CLK_GPSPI2, "gpspi2", MAP_HP, HP_GPSPI2_CTRL0, GPSPI_GATES,
	    GPSPI_HS_CLK_DIV_NUM, GPSPI_CLK_SRC_SEL, mux_xtal_rcfast_pll480, 0),
	MCLK(CLK_GPSPI3, "gpspi3", MAP_HP, HP_GPSPI3_CTRL0, GPSPI_GATES,
	    GPSPI_HS_CLK_DIV_NUM, GPSPI_CLK_SRC_SEL, mux_xtal_rcfast_pll480, 0),
	MCLK(CLK_LEDC0, "ledc0", MAP_HP, HP_LEDC_CTRL0,
	    LEDC0_APB_CLK_EN | LEDC0_CLK_EN, 0, LEDC0_CLK_SRC_SEL,
	    mux_xtal_rcfast_f80m, HPSYS_LEDC0_MEM),
	MCLK(CLK_LEDC1, "ledc1", MAP_HP, HP_LEDC_CTRL0,
	    LEDC1_APB_CLK_EN | LEDC1_CLK_EN, 0, LEDC1_CLK_SRC_SEL,
	    mux_xtal_rcfast_f80m, HPSYS_LEDC1_MEM),
	MCLK(CLK_TWAI0, "twai0", MAP_HP, HP_TWAI_CTRL0(0),
	    TWAI_APB_CLK_EN | TWAI_CLK_EN, 0, TWAI_CLK_SRC_SEL, mux_twai,
	    HPSYS_CAN0_MEM),
	MCLK(CLK_TWAI1, "twai1", MAP_HP, HP_TWAI_CTRL0(1),
	    TWAI_APB_CLK_EN | TWAI_CLK_EN, 0, TWAI_CLK_SRC_SEL, mux_twai,
	    HPSYS_CAN1_MEM),
	MCLK(CLK_CRYPTO, "crypto", MAP_HP, HP_CRYPTO_CTRL0,
	    CRYPTO_SYS_CLK_EN | CRYPTO_SEC_CLK_EN, 0, CRYPTO_CLK_SRC_SEL,
	    mux_xtal_rcfast_f240m, 0),
	CLK(CLK_AES, "aes", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_AES_CLK_EN, 0, 0),
	CLK(CLK_SHA, "sha", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_SHA_CLK_EN, 0, 0),
	CLK(CLK_RSA, "rsa", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_RSA_CLK_EN, 0, 0),
	CLK(CLK_ECC, "ecc", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_ECC_CLK_EN, 0, 0),
	CLK(CLK_DS, "ds", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_DS_CLK_EN, 0, 0),
	CLK(CLK_HMAC, "hmac", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_HMAC_CLK_EN, 0, 0),
	CLK(CLK_ECDSA, "ecdsa", MAP_HP, PARENT_XTAL, HP_CRYPTO_CTRL0,
	    CRYPTO_ECDSA_CLK_EN, 0, 0),
	CLK(CLK_TIMG0, "timg0", MAP_HP, PARENT_XTAL, HP_TIMERGRP0_CTRL0,
	    TIMERGRP_APB_CLK_EN | TIMERGRP_WDT_CLK_EN, 0, 0),
	CLK(CLK_TIMG1, "timg1", MAP_HP, PARENT_XTAL, HP_TIMERGRP1_CTRL0,
	    TIMERGRP_APB_CLK_EN | TIMERGRP_WDT_CLK_EN, 0, 0),
	CLK(CLK_TSENS, "tsens", MAP_LPPERI, PARENT_XTAL, LPPERI_TSENS_CTRL,
	    LPPERI_CLK_EN, 0, 0),
	MCLK(CLK_ADC, "adc", MAP_LPPERI, LPPERI_ADC_CTRL, LPPERI_CLK_EN,
	    LPPERI_DIV_NUM, LPPERI_CLK_SEL, mux_lp_rcfast_xtal, 0),
	MCLK(CLK_LP_I2C, "lp_i2c", MAP_LPPERI, LPPERI_I2C_CTRL, LPPERI_CLK_EN,
	    LPPERI_I2C_DIV_NUM, LPPERI_CLK_SEL, mux_lp_rcfast_xtal, 0),
#define	MCPWM_GATES	(MCPWM_APB_CLK_EN | MCPWM_CLK_EN)
	MCLK(CLK_MCPWM0, "mcpwm0", MAP_HP, HP_MCPWM_CTRL0(0), MCPWM_GATES,
	    MCPWM_CLK_DIV_NUM, MCPWM_CLK_SRC_SEL, mux_xtal_rcfast_f160m, 0),
	MCLK(CLK_MCPWM1, "mcpwm1", MAP_HP, HP_MCPWM_CTRL0(1), MCPWM_GATES,
	    MCPWM_CLK_DIV_NUM, MCPWM_CLK_SRC_SEL, mux_xtal_rcfast_f160m, 0),
	MCLK(CLK_MCPWM2, "mcpwm2", MAP_HP, HP_MCPWM_CTRL0(2), MCPWM_GATES,
	    MCPWM_CLK_DIV_NUM, MCPWM_CLK_SRC_SEL, mux_xtal_rcfast_f160m, 0),
	MCLK(CLK_MCPWM3, "mcpwm3", MAP_HP, HP_MCPWM_CTRL0(3), MCPWM_GATES,
	    MCPWM_CLK_DIV_NUM, MCPWM_CLK_SRC_SEL, mux_xtal_rcfast_f160m, 0),
	CLK(CLK_I2S0, "i2s0", MAP_HP, PARENT_XTAL, HP_I2S_CTRL0(0),
	    I2S_APB_CLK_EN, 0, 0),
	CLK(CLK_I2S1, "i2s1", MAP_HP, PARENT_XTAL, HP_I2S_CTRL0(1),
	    I2S_APB_CLK_EN, 0, 0),
	CLK(CLK_AHB_PDMA, "ahb_pdma", MAP_HP, PARENT_XTAL, HP_AHB_PDMA_CTRL0,
	    AHB_PDMA_SYS_CLK_EN, 0, 0),
	MCLK(CLK_ZERO_DET, "zero_det", MAP_HP, HP_ZERO_DET_CTRL0,
	    ZERO_DET_APB_CLK_EN | ZERO_DET_CLK_EN, ZERO_DET_CLK_DIV_NUM,
	    ZERO_DET_CLK_SRC_SEL, mux_xtal_rcfast_f80m, 0),
};

static const struct esp_rst esp_rsts[RST_NUM] = {
	[RST_IOMUX] = { MAP_HP, HP_IOMUX_CTRL0, IOMUX_RST_EN },
	[RST_EMAC] = { MAP_CNNT, CNNT_HP_EMAC_CTRL, EMAC_RST_EN },
	[RST_USB_OTGHS] = { MAP_CNNT, CNNT_USB_OTG20_CTRL, USB_OTGHS_RST_EN },
	[RST_SDMMC] = { MAP_CNNT, CNNT_HP_SDMMC_CTRL, SDMMC_RST_EN },
	[RST_UART1] = { MAP_HP, HP_UART_CTRL0(1), UART_APB_RST_EN },
	[RST_UART2] = { MAP_HP, HP_UART_CTRL0(2), UART_APB_RST_EN },
	[RST_UART3] = { MAP_HP, HP_UART_CTRL0(3), UART_APB_RST_EN },
	[RST_I2C0] = { MAP_HP, HP_I2C_CTRL0(0), I2C_RST_EN },
	[RST_I2C1] = { MAP_HP, HP_I2C_CTRL0(1), I2C_RST_EN },
	[RST_GPSPI2] = { MAP_HP, HP_GPSPI2_CTRL0, GPSPI_RST_EN },
	[RST_GPSPI3] = { MAP_HP, HP_GPSPI3_CTRL0, GPSPI_RST_EN },
	[RST_LEDC0] = { MAP_HP, HP_LEDC_CTRL0, LEDC0_RST_EN },
	[RST_LEDC1] = { MAP_HP, HP_LEDC_CTRL0, LEDC1_RST_EN },
	[RST_TWAI0] = { MAP_HP, HP_TWAI_CTRL0(0), TWAI_RST_EN },
	[RST_TWAI1] = { MAP_HP, HP_TWAI_CTRL0(1), TWAI_RST_EN },
	[RST_CRYPTO] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_RST_EN },
	[RST_SEC] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_SEC_RST_EN },
	[RST_AES] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_AES_RST_EN },
	[RST_SHA] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_SHA_RST_EN },
	[RST_RSA] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_RSA_RST_EN },
	[RST_ECC] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_ECC_RST_EN },
	[RST_DS] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_DS_RST_EN },
	[RST_HMAC] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_HMAC_RST_EN },
	[RST_ECDSA] = { MAP_HP, HP_CRYPTO_CTRL0, CRYPTO_ECDSA_RST_EN },
	[RST_TIMG1] = { MAP_HP, HP_TIMERGRP1_CTRL0, TIMERGRP_RST_EN },
	[RST_TSENS] = { MAP_LPPERI, LPPERI_TSENS_CTRL, LPPERI_RST_EN },
	[RST_ADC] = { MAP_LPPERI, LPPERI_ADC_CTRL, LPPERI_RST_EN },
	[RST_LP_I2C] = { MAP_LPPERI, LPPERI_I2C_CTRL, LPPERI_RST_EN },
	[RST_MCPWM0] = { MAP_HP, HP_MCPWM_CTRL0(0), MCPWM_RST_EN },
	[RST_MCPWM1] = { MAP_HP, HP_MCPWM_CTRL0(1), MCPWM_RST_EN },
	[RST_MCPWM2] = { MAP_HP, HP_MCPWM_CTRL0(2), MCPWM_RST_EN },
	[RST_MCPWM3] = { MAP_HP, HP_MCPWM_CTRL0(3), MCPWM_RST_EN },
	[RST_ZERO_DET] = { MAP_HP, HP_ZERO_DET_CTRL0,
	    ZERO_DET_APB_RST_EN | ZERO_DET_CORE_RST_EN },
	[RST_I2S0] = { MAP_HP, HP_I2S_CTRL0(0), I2S_APB_RST_EN },
	[RST_I2S1] = { MAP_HP, HP_I2S_CTRL0(1), I2S_APB_RST_EN },
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

static const char * const esp_parent_names[NPARENTS] = {
	[PARENT_XTAL] = "xtal", [PARENT_MPLL] = "mpll", [PARENT_12M] = "usb_12m",
	[PARENT_RC_FAST] = "rc_fast", [PARENT_F80M] = "ref_f80m",
	[PARENT_F160M] = "pll_f160m", [PARENT_F240M] = "pll_f240m",
	[PARENT_PLL480] = "bbpll",
};

/* The parents are fixed-rate clocks; which one is this? */
static int
esp_clk_pindex(struct esp_clkrst_softc *sc, struct clk *clk)
{
	if (clk >= &sc->sc_pclks[0] && clk < &sc->sc_pclks[NPARENTS])
		return clk - &sc->sc_pclks[0];
	return -1;
}

static struct clk *
esp_clk_get(void *priv, const char *name)
{
	struct esp_clkrst_softc * const sc = priv;

	for (u_int id = 0; id < CLK_NUM; id++) {
		if (sc->sc_clks[id].ec_clk.name != NULL &&
		    strcmp(sc->sc_clks[id].ec_clk.name, name) == 0)
			return &sc->sc_clks[id].ec_clk;
	}
	for (u_int p = 0; p < NPARENTS; p++) {
		if (strcmp(esp_parent_names[p], name) == 0)
			return &sc->sc_pclks[p];
	}
	return NULL;
}

static void
esp_clk_put(void *priv, struct clk *clk)
{
}

/* The parent the mux selects now. */
static u_int
esp_clk_parent(struct esp_clkrst_softc *sc, const struct esp_clk *ec)
{
	if (ec->ec_muxsel == NULL)
		return ec->ec_parent;
	return ec->ec_muxsel[__SHIFTOUT(esp_clkrst_read(sc, ec->ec_map,
	    ec->ec_reg), ec->ec_mux)];
}

static u_int
esp_clk_parent_rate(struct esp_clkrst_softc *sc, const struct esp_clk *ec)
{
	const u_int p = esp_clk_parent(sc, ec);

	return p < NPARENTS ? sc->sc_parent_rate[p] : 0;
}

static struct clk *
esp_clk_get_parent(void *priv, struct clk *clk)
{
	struct esp_clkrst_softc * const sc = priv;
	u_int p;

	if (esp_clk_pindex(sc, clk) >= 0)
		return NULL;
	p = esp_clk_parent(sc, (struct esp_clk *)clk);
	return p < NPARENTS ? &sc->sc_pclks[p] : NULL;
}

static u_int
esp_clk_get_rate(void *priv, struct clk *clk)
{
	struct esp_clkrst_softc * const sc = priv;
	struct esp_clk * const ec = (struct esp_clk *)clk;
	const int pi = esp_clk_pindex(sc, clk);

	if (pi >= 0)
		return sc->sc_parent_rate[pi];

	const u_int prate = esp_clk_parent_rate(sc, ec);

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

	if (esp_clk_pindex(sc, clk) >= 0)
		return rate == esp_clk_get_rate(priv, clk) ? 0 : EINVAL;

	const u_int prate = esp_clk_parent_rate(sc, ec);
	const u_int maxdiv = __SHIFTOUT_MASK(ec->ec_div) + 1;
	u_int div;

	if (ec->ec_sdio && rate != 0)
		return esp_clk_sdio_set_rate(sc, prate, rate);
	if (ec->ec_div == 0 || rate == 0)
		return rate == esp_clk_get_rate(priv, clk) ? 0 : EINVAL;

	if (prate == 0)
		return EINVAL;
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

	if (esp_clk_pindex(sc, clk) >= 0)
		return 0;
	if (!sc->sc_mapped[ec->ec_map])
		return ENXIO;

	/* Select the parent, power up the block's memory, then ungate. */
	if (ec->ec_muxsel != NULL) {
		u_int sel;

		for (sel = 0; sel < 4; sel++)
			if (ec->ec_muxsel[sel] == ec->ec_parent)
				break;
		KASSERT(sel < 4);
		esp_clkrst_update(sc, ec->ec_map, ec->ec_reg, ec->ec_mux,
		    __SHIFTIN(sel, ec->ec_mux));
	} else {
		esp_clkrst_update(sc, ec->ec_map, ec->ec_reg, ec->ec_mux, 0);
	}
	if (ec->ec_mem != 0 && sc->sc_mapped[MAP_HPSYS])
		esp_clkrst_update(sc, MAP_HPSYS, ec->ec_mem, 0xf,
		    HPSYS_MEM_ON);
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

	if (esp_clk_pindex(sc, clk) >= 0)
		return 0;
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
	.get_parent = esp_clk_get_parent,
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

	if (!sc->sc_mapped[er->er_map])
		return ENXIO;
	esp_clkrst_update(sc, er->er_map, er->er_reg, 0, er->er_mask);
	return 0;
}

static int
esp_clkrst_reset_deassert(device_t dev, void *priv)
{
	struct esp_clkrst_softc * const sc = device_private(dev);
	const struct esp_rst * const er = priv;

	if (!sc->sc_mapped[er->er_map])
		return ENXIO;
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
		if (fdtbus_get_reg(phandle, map, &addr, &size) != 0 &&
		    map >= NMAPS_REQUIRED)
			continue;
		if (fdtbus_get_reg(phandle, map, &addr, &size) != 0 ||
		    bus_space_map(sc->sc_bst, addr, size, 0,
			&sc->sc_bsh[map]) != 0) {
			aprint_error(": couldn't map registers\n");
			return;
		}
		sc->sc_mapped[map] = true;
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_HIGH);

	sc->sc_parent_rate[PARENT_XTAL] =
	    esp_clkrst_parent_rate(phandle, "xtal", 40000000);
	sc->sc_parent_rate[PARENT_MPLL] =
	    esp_clkrst_parent_rate(phandle, "mpll", 500000000);
	sc->sc_parent_rate[PARENT_12M] = 12000000;
	sc->sc_parent_rate[PARENT_RC_FAST] = 17500000;	/* approximate */
	sc->sc_parent_rate[PARENT_F80M] = 80000000;
	sc->sc_parent_rate[PARENT_F160M] = 160000000;
	sc->sc_parent_rate[PARENT_F240M] = 240000000;
	sc->sc_parent_rate[PARENT_PLL480] = 480000000;

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
	for (u_int p = 0; p < NPARENTS; p++) {
		sc->sc_pclks[p].name = esp_parent_names[p];
		sc->sc_pclks[p].domain = &sc->sc_clkdom;
		clk_attach(&sc->sc_pclks[p]);
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
