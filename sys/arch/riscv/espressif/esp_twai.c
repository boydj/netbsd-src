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
 * ESP32-S31 TWAI: two CAN controllers, each a CTU CAN FD core, as CAN
 * interfaces (netcan, canconfig(8)).
 *
 * NetBSD's CAN sockets carry classic frames only, so that's what goes out;
 * FD frames on the bus are understood (no error frames for them) but
 * dropped, and counted.  One frame is in flight at a time, in TXT buffer
 * 1.  The core counts the 40 MHz XTAL; bit timing is canconfig's
 * (prescaler, propagation, phase 1, phase 2, SJW, in time quanta), or
 * sysctl hw.esptwaiN.bitrate picks one the way ESP-IDF does (500 kbit/s
 * at attach).  A bus-off controller recovers on its own after 128 x 11
 * recessive bits.  Pins: sysctl hw.espgpio0.pinN=twai0_tx / twai0_rx, or
 * pinctrl, to a transceiver.
 *
 *	twai0: can@20394000 {
 *		compatible = "espressif,esp32s31-twai";
 *		reg = <0x20394000 0x1000>;
 *		interrupt-parent = <&intmtx>;
 *		interrupts = <18 4>;
 *		clocks = <&clkrst 23>;
 *		resets = <&clkrst 15>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/endian.h>
#include <sys/evcnt.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/rndsource.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <net/if.h>
#include <net/if_types.h>
#include <net/bpf.h>

#include <netcan/can.h>
#include <netcan/can_var.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>

#define	CTU_DEVICE_ID		0x00
#define	 DEVICE_ID_CTU		0xcafd
#define	CTU_MODE		0x04
#define	 MODE_RST		__BIT(0)
#define	 MODE_BMM		__BIT(1)	/* bus monitoring */
#define	 MODE_STM		__BIT(2)	/* self test: no ACK needed */
#define	 MODE_FDE		__BIT(4)
#define	 MODE_ROM		__BIT(6)	/* restricted operation */
#define	 MODE_RXBAM		__BIT(9)
#define	 MODE_ILBP		__BIT(21)	/* internal loopback */
#define	 MODE_ENA		__BIT(22)
#define	 MODE_TBFBO		__BIT(25)
#define	CTU_STATUS		0x08
#define	CTU_COMMAND		0x0c
#define	 COMMAND_CDO		__BIT(3)
#define	 COMMAND_ERCRST		__BIT(4)
#define	CTU_INT_STAT		0x10
#define	CTU_INT_ENA_SET		0x14
#define	CTU_INT_ENA_CLR		0x18
#define	CTU_INT_MASK_SET	0x1c
#define	CTU_INT_MASK_CLR	0x20
#define	 INT_RXI		__BIT(0)
#define	 INT_TXI		__BIT(1)
#define	 INT_EWLI		__BIT(2)
#define	 INT_DOI		__BIT(3)
#define	 INT_FCSI		__BIT(4)
#define	 INT_ALI		__BIT(5)
#define	 INT_BEI		__BIT(6)
#define	 INT_RBNEI		__BIT(10)
#define	 INT_TXBHCI		__BIT(11)
#define	 INT_ALL		__BITS(11, 0)
#define	CTU_BTR			0x24
#define	 BTR_PROP		__BITS(6, 0)
#define	 BTR_PH1		__BITS(12, 7)
#define	 BTR_PH2		__BITS(18, 13)
#define	 BTR_BRP		__BITS(26, 19)
#define	 BTR_SJW		__BITS(31, 27)
#define	CTU_FAULT_STATE		0x2c
#define	 FAULT_ERA		__BIT(16)
#define	 FAULT_ERP		__BIT(17)
#define	 FAULT_BOF		__BIT(18)
#define	CTU_REC_TEC		0x30
#define	 REC_VAL		__BITS(8, 0)
#define	 TEC_VAL		__BITS(24, 16)
#define	CTU_RX_STATUS		0x68
#define	 RX_STATUS_RXFRC	__BITS(14, 4)
#define	CTU_RX_DATA		0x6c
#define	CTU_TX_STATUS		0x70
#define	 TXT_STATE(s, n)	(((s) >> (4 * (n))) & 0xf)
#define	 TXT_READY		1
#define	 TXT_TX		2
#define	 TXT_ABORTING		3
#define	 TXT_OK			4
#define	 TXT_FAILED		6
#define	 TXT_ABORTED		7
#define	 TXT_EMPTY		8
#define	CTU_TX_COMMAND		0x74
#define	 TXCMD_EMPTY		__BIT(0)
#define	 TXCMD_READY		__BIT(1)
#define	 TXCMD_ABORT		__BIT(2)
#define	 TXCMD_BUF(n)		__BIT(8 + (n))
#define	CTU_TX_PRIORITY		0x78
#define	CTU_TXTB(n)		(0x100 * ((n) + 1))

/* frame format word */
#define	FFW_DLC			__BITS(3, 0)
#define	FFW_RTR			__BIT(5)
#define	FFW_IDE			__BIT(6)
#define	FFW_FDF			__BIT(7)
#define	FFW_RWCNT		__BITS(15, 11)
/* identifier word */
#define	IDW_BASE_SHIFT		18

#define	ESP_TWAI_INTS		(INT_RBNEI | INT_TXBHCI | INT_DOI | INT_FCSI | \
				 INT_EWLI)
#define	ESP_TWAI_TXB		0		/* the TXT buffer used */

struct esp_twai_softc {
	struct canif_softc	sc_cansc;	/* first */
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	kmutex_t		sc_lock;
	void			*sc_ih;
	struct ifnet		*sc_ifp;
	krndsource_t		sc_rnd;
	struct mbuf		*sc_txm;	/* in the TXT buffer */
	u_int			sc_clkfreq;
	const char		*sc_state;
	bool			sc_busoff;
	struct evcnt		sc_ev_fd;
	struct evcnt		sc_ev_busoff;
	struct evcnt		sc_ev_overrun;
	struct sysctllog	*sc_sysctllog;
};
#define	sc_dev		sc_cansc.csc_dev
#define	sc_timecaps	sc_cansc.csc_timecaps
#define	sc_timings	sc_cansc.csc_timings
#define	sc_linkmodes	sc_cansc.csc_linkmodes

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	esp_twai_match(device_t, cfdata_t, void *);
static void	esp_twai_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_twai, sizeof(struct esp_twai_softc),
	esp_twai_match, esp_twai_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-twai" },
	DEVICE_COMPAT_EOL
};

static int
esp_twai_reset(struct esp_twai_softc *sc)
{
	WR(sc, CTU_MODE, MODE_RST);
	for (u_int i = 0; i < 100; i++) {
		if ((RD(sc, CTU_DEVICE_ID) & 0xffff) == DEVICE_ID_CTU)
			return 0;
		DELAY(100);
	}
	return ETIMEDOUT;
}

/*
 * Bit timing for a bit rate, as ESP-IDF works it out: the fewest time
 * quanta a bit of at most 254, sampling at 75% (>= 800 kbit/s), 80%
 * (>= 500 kbit/s) or 87.5%.
 */
static int
esp_twai_calc_timing(u_int clk, u_int rate, struct can_link_timings *t)
{
	u_int total, brp, tseg, sp, tseg1, ph1, ph2;

	if (rate == 0)
		return EINVAL;
	total = (clk + rate / 2) / rate;
	for (brp = 1; brp <= 255; brp++)
		if (total % brp == 0 && total / brp >= 2 && total / brp <= 254)
			break;
	if (brp > 255 || (clk + total / 2) / total != rate)
		return EINVAL;
	tseg = total / brp;
	sp = rate >= 800000 ? 750 : rate >= 500000 ? 800 : 875;
	tseg1 = tseg * sp / 1000 - 1;
	ph2 = tseg - tseg1 - 1;
	ph1 = MIN(tseg1 * 3 / 4, 63);
	if (tseg1 - ph1 < 1 || ph2 < 1 || ph2 > 63)
		return EINVAL;
	t->clt_brp = brp;
	t->clt_prop = tseg1 - ph1;
	t->clt_ps1 = ph1;
	t->clt_ps2 = ph2;
	t->clt_sjw = MAX(1, MIN(ph2 / 2, 31));
	return 0;
}

static int
esp_twai_check_timing(struct esp_twai_softc *sc,
    const struct can_link_timings *t)
{
	const struct can_link_timecaps * const c = &sc->sc_timecaps;

	if (t->clt_brp < c->cltc_brp_min || t->clt_brp > c->cltc_brp_max ||
	    t->clt_prop < c->cltc_prop_min || t->clt_prop > c->cltc_prop_max ||
	    t->clt_ps1 < c->cltc_ps1_min || t->clt_ps1 > c->cltc_ps1_max ||
	    t->clt_ps2 < c->cltc_ps2_min || t->clt_ps2 > c->cltc_ps2_max ||
	    t->clt_sjw < 1 || t->clt_sjw > c->cltc_sjw_max)
		return EINVAL;
	return 0;
}

static void
esp_twai_update_state(struct esp_twai_softc *sc)
{
	const uint32_t fs = RD(sc, CTU_FAULT_STATE);
	const bool was_busoff = sc->sc_busoff;

	KASSERT(mutex_owned(&sc->sc_lock));

	sc->sc_busoff = (fs & FAULT_BOF) != 0;
	if (sc->sc_busoff) {
		if (!was_busoff) {
			sc->sc_ev_busoff.ev_count++;
			aprint_error_dev(sc->sc_dev, "bus-off, recovering\n");
			WR(sc, CTU_COMMAND, COMMAND_ERCRST);
		}
		sc->sc_state = "bus-off";
	} else if (fs & FAULT_ERP) {
		sc->sc_state = "error-passive";
	} else {
		sc->sc_state = "error-active";
	}
}

static void
esp_twai_rx(struct esp_twai_softc *sc)
{
	struct ifnet * const ifp = sc->sc_ifp;

	KASSERT(mutex_owned(&sc->sc_lock));

	while (__SHIFTOUT(RD(sc, CTU_RX_STATUS), RX_STATUS_RXFRC) != 0) {
		const uint32_t ffw = RD(sc, CTU_RX_DATA);
		const u_int rwcnt = __SHIFTOUT(ffw, FFW_RWCNT);
		uint32_t idw = 0, data[16];
		u_int len;
		struct can_frame *cf;
		struct mbuf *m;

		if (rwcnt == 0)
			break;			/* not a frame: give up */
		/* identifier, two timestamp words, then the data */
		for (u_int i = 0; i < rwcnt; i++) {
			const uint32_t w = RD(sc, CTU_RX_DATA);

			if (i == 0)
				idw = w;
			else if (i >= 3 && i - 3 < __arraycount(data))
				data[i - 3] = w;
		}

		if (ffw & FFW_FDF) {
			sc->sc_ev_fd.ev_count++;
			if_statinc(ifp, if_iqdrops);
			continue;
		}
		if ((m = m_gethdr(M_NOWAIT, MT_HEADER)) == NULL) {
			if_statinc(ifp, if_ierrors);
			continue;
		}
		cf = mtod(m, struct can_frame *);
		memset(cf, 0, sizeof(*cf));
		if (ffw & FFW_IDE)
			cf->can_id = (idw & CAN_EFF_MASK) | CAN_EFF_FLAG;
		else
			cf->can_id = (idw >> IDW_BASE_SHIFT) & CAN_SFF_MASK;
		/* classic: DLC 9-15 mean 8 bytes */
		len = MIN(__SHIFTOUT(ffw, FFW_DLC), CAN_MAX_DLEN);
		cf->can_dlc = len;
		if (ffw & FFW_RTR) {
			cf->can_id |= CAN_RTR_FLAG;
		} else {
			for (u_int i = 0; i < len; i++)
				cf->data[i] = data[i / 4] >> (8 * (i % 4));
		}
		m->m_len = m->m_pkthdr.len = CAN_MTU;
		m_set_rcvif(m, ifp);
		can_bpf_mtap(ifp, m, true);
		can_input(ifp, m);
	}
	/* Cleared once drained; it comes back while frames remain. */
	WR(sc, CTU_INT_STAT, INT_RBNEI);
}

static void
esp_twai_txdone(struct esp_twai_softc *sc)
{
	struct ifnet * const ifp = sc->sc_ifp;
	const u_int st = TXT_STATE(RD(sc, CTU_TX_STATUS), ESP_TWAI_TXB);
	struct mbuf *m;

	KASSERT(mutex_owned(&sc->sc_lock));

	if (st != TXT_OK && st != TXT_FAILED && st != TXT_ABORTED)
		return;
	WR(sc, CTU_TX_COMMAND, TXCMD_EMPTY | TXCMD_BUF(ESP_TWAI_TXB));
	if ((m = sc->sc_txm) == NULL)
		return;
	sc->sc_txm = NULL;
	ifp->if_timer = 0;
	if (st == TXT_OK) {
		if_statadd2(ifp, if_obytes, m->m_len, if_opackets, 1);
		/* local echo, for the other sockets on this interface */
		can_mbuf_tag_clean(m);
		m_set_rcvif(m, ifp);
		can_input(ifp, m);
	} else {
		if_statinc(ifp, if_oerrors);
		m_freem(m);
	}
	if_schedule_deferred_start(ifp);
}

static int
esp_twai_intr(void *arg)
{
	struct esp_twai_softc * const sc = arg;
	uint32_t st;
	int rv = 0;

	mutex_enter(&sc->sc_lock);
	while ((st = RD(sc, CTU_INT_STAT) & ESP_TWAI_INTS) != 0) {
		rv = 1;
		rnd_add_uint32(&sc->sc_rnd, st);
		/* RBNEI is cleared after the drain, DOI after CDO. */
		WR(sc, CTU_INT_STAT, st & ~(INT_RBNEI | INT_DOI));
		if (st & INT_DOI) {
			sc->sc_ev_overrun.ev_count++;
			if_statinc(sc->sc_ifp, if_ierrors);
			WR(sc, CTU_COMMAND, COMMAND_CDO);
			WR(sc, CTU_INT_STAT, INT_DOI);
		}
		if (st & INT_RBNEI)
			esp_twai_rx(sc);
		if (st & INT_TXBHCI)
			esp_twai_txdone(sc);
		if (st & (INT_FCSI | INT_EWLI))
			esp_twai_update_state(sc);
		if ((sc->sc_ifp->if_flags & IFF_RUNNING) == 0)
			break;
	}
	mutex_exit(&sc->sc_lock);
	return rv;
}

static void
esp_twai_ifstart(struct ifnet *ifp)
{
	struct esp_twai_softc * const sc = ifp->if_softc;
	const bus_size_t tb = CTU_TXTB(ESP_TWAI_TXB);
	struct can_frame *cf;
	struct mbuf *m;
	uint32_t ffw, idw, d[2];
	u_int st;

	mutex_enter(&sc->sc_lock);
	if (sc->sc_txm != NULL || (ifp->if_flags & IFF_RUNNING) == 0)
		goto out;
	st = TXT_STATE(RD(sc, CTU_TX_STATUS), ESP_TWAI_TXB);
	if (st == TXT_READY || st == TXT_TX || st == TXT_ABORTING)
		goto out;
	IF_DEQUEUE(&ifp->if_snd, m);
	if (m == NULL)
		goto out;
	MCLAIM(m, ifp->if_mowner);
	KASSERT((m->m_flags & M_PKTHDR) != 0);
	if (m->m_len < CAN_MTU) {
		if_statinc(ifp, if_oerrors);
		m_freem(m);
		goto out;
	}

	cf = mtod(m, struct can_frame *);
	ffw = __SHIFTIN(MIN(cf->can_dlc, CAN_MAX_DLC), FFW_DLC);
	if (cf->can_id & CAN_EFF_FLAG) {
		ffw |= FFW_IDE;
		idw = cf->can_id & CAN_EFF_MASK;
	} else {
		idw = (cf->can_id & CAN_SFF_MASK) << IDW_BASE_SHIFT;
	}
	if (cf->can_id & CAN_RTR_FLAG)
		ffw |= FFW_RTR;
	d[0] = le32dec(&cf->data[0]);
	d[1] = le32dec(&cf->data[4]);

	WR(sc, tb + 0x00, ffw);
	WR(sc, tb + 0x04, idw);
	WR(sc, tb + 0x08, 0);
	WR(sc, tb + 0x0c, 0);
	WR(sc, tb + 0x10, d[0]);
	WR(sc, tb + 0x14, d[1]);
	sc->sc_txm = m;
	WR(sc, CTU_TX_COMMAND, TXCMD_READY | TXCMD_BUF(ESP_TWAI_TXB));
	ifp->if_timer = 5;
	can_bpf_mtap(ifp, m, false);
out:
	mutex_exit(&sc->sc_lock);
}

static int
esp_twai_ifup(struct esp_twai_softc *sc)
{
	const struct can_link_timings * const t = &sc->sc_timings;
	uint32_t mode;
	int error;

	KASSERT(mutex_owned(&sc->sc_lock));

	if ((error = esp_twai_check_timing(sc, t)) != 0)
		return error;
	if ((error = esp_twai_reset(sc)) != 0) {
		aprint_error_dev(sc->sc_dev, "doesn't come out of reset\n");
		return error;
	}

	/* All frames (no acceptance filtering), FD understood. */
	mode = MODE_TBFBO | MODE_RXBAM | MODE_FDE;
	if (sc->sc_linkmodes & CAN_LINKMODE_LISTENONLY)
		mode |= MODE_BMM | MODE_ROM;
	if (sc->sc_linkmodes & CAN_LINKMODE_LOOPBACK)
		mode |= MODE_ILBP | MODE_STM;
	if (sc->sc_linkmodes & CAN_LINKMODE_PRESUME_ACK)
		mode |= MODE_STM;
	WR(sc, CTU_MODE, mode);
	WR(sc, CTU_BTR, __SHIFTIN(t->clt_prop, BTR_PROP) |
	    __SHIFTIN(t->clt_ps1, BTR_PH1) | __SHIFTIN(t->clt_ps2, BTR_PH2) |
	    __SHIFTIN(t->clt_brp, BTR_BRP) | __SHIFTIN(t->clt_sjw, BTR_SJW));
	WR(sc, CTU_TX_PRIORITY, 7 << (4 * ESP_TWAI_TXB));

	WR(sc, CTU_INT_ENA_CLR, INT_ALL);
	WR(sc, CTU_INT_MASK_SET, INT_ALL & ~ESP_TWAI_INTS);
	WR(sc, CTU_INT_MASK_CLR, ESP_TWAI_INTS);
	WR(sc, CTU_INT_STAT, INT_ALL);
	WR(sc, CTU_INT_ENA_SET, ESP_TWAI_INTS);

	WR(sc, CTU_MODE, mode | MODE_ENA);
	sc->sc_state = "error-active";
	sc->sc_busoff = false;
	sc->sc_ifp->if_flags |= IFF_RUNNING;
	return 0;
}

static void
esp_twai_ifdown(struct esp_twai_softc *sc)
{
	struct ifnet * const ifp = sc->sc_ifp;

	KASSERT(mutex_owned(&sc->sc_lock));

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
	WR(sc, CTU_INT_ENA_CLR, INT_ALL);
	WR(sc, CTU_MODE, RD(sc, CTU_MODE) & ~MODE_ENA);
	WR(sc, CTU_INT_STAT, INT_ALL);
	if (sc->sc_txm != NULL) {
		m_freem(sc->sc_txm);
		sc->sc_txm = NULL;
	}
	sc->sc_state = "stopped";
}

static int
esp_twai_ifioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct esp_twai_softc * const sc = ifp->if_softc;
	struct ifreq * const ifr = data;
	int error = 0;

	mutex_enter(&sc->sc_lock);
	switch (cmd) {
	case SIOCINITIFADDR:
		error = EAFNOSUPPORT;
		break;
	case SIOCSIFMTU:
		if ((u_int)ifr->ifr_mtu != sizeof(struct can_frame))
			error = EINVAL;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = EAFNOSUPPORT;
		break;
	default:
		error = ifioctl_common(ifp, cmd, data);
		if (error != 0)
			break;
		if ((ifp->if_flags & IFF_UP) != 0 &&
		    (ifp->if_flags & IFF_RUNNING) == 0) {
			if ((error = esp_twai_ifup(sc)) != 0)
				ifp->if_flags &= ~IFF_UP;
		} else if ((ifp->if_flags & IFF_UP) == 0 &&
		    (ifp->if_flags & IFF_RUNNING) != 0) {
			esp_twai_ifdown(sc);
		}
		break;
	}
	mutex_exit(&sc->sc_lock);
	return error;
}

static void
esp_twai_ifwatchdog(struct ifnet *ifp)
{
	struct esp_twai_softc * const sc = ifp->if_softc;

	mutex_enter(&sc->sc_lock);
	if (sc->sc_txm != NULL) {
		/* No ACK, or no bus: give the frame up. */
		WR(sc, CTU_TX_COMMAND, TXCMD_ABORT | TXCMD_BUF(ESP_TWAI_TXB));
		for (u_int i = 0; i < 100 && TXT_STATE(RD(sc, CTU_TX_STATUS),
		    ESP_TWAI_TXB) == TXT_ABORTING; i++)
			DELAY(10);
		WR(sc, CTU_TX_COMMAND, TXCMD_EMPTY | TXCMD_BUF(ESP_TWAI_TXB));
		m_freem(sc->sc_txm);
		sc->sc_txm = NULL;
		if_statinc(ifp, if_oerrors);
	}
	mutex_exit(&sc->sc_lock);
	if_schedule_deferred_start(ifp);
}

static int
esp_twai_sysctl_bitrate(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_twai_softc * const sc = node.sysctl_data;
	const struct can_link_timings * const t = &sc->sc_timings;
	struct can_link_timings nt;
	int rate, error;

	mutex_enter(&sc->sc_lock);
	rate = esp_twai_check_timing(sc, t) != 0 ? 0 :
	    sc->sc_clkfreq / (t->clt_brp *
	    (1 + t->clt_prop + t->clt_ps1 + t->clt_ps2));
	mutex_exit(&sc->sc_lock);

	node.sysctl_data = &rate;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;
	if (rate <= 0 || esp_twai_calc_timing(sc->sc_clkfreq, rate, &nt) != 0)
		return EINVAL;

	mutex_enter(&sc->sc_lock);
	if (sc->sc_ifp->if_flags & IFF_RUNNING)
		error = EBUSY;		/* as canconfig: down first */
	else
		sc->sc_timings = nt;
	mutex_exit(&sc->sc_lock);
	return error;
}

static int
esp_twai_sysctl_state(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct esp_twai_softc * const sc = node.sysctl_data;
	char buf[64];

	mutex_enter(&sc->sc_lock);
	if (sc->sc_ifp->if_flags & IFF_RUNNING) {
		const uint32_t rt = RD(sc, CTU_REC_TEC);

		esp_twai_update_state(sc);
		snprintf(buf, sizeof(buf), "%s, tx errors %u, rx errors %u",
		    sc->sc_state, (u_int)__SHIFTOUT(rt, TEC_VAL),
		    (u_int)__SHIFTOUT(rt, REC_VAL));
	} else {
		strlcpy(buf, "stopped", sizeof(buf));
	}
	mutex_exit(&sc->sc_lock);

	node.sysctl_data = buf;
	node.sysctl_size = strlen(buf) + 1;
	return sysctl_lookup(SYSCTLFN_CALL(&node));
}

static void
esp_twai_sysctl_init(struct esp_twai_softc *sc)
{
	const struct sysctlnode *node;

	if (sysctl_createv(&sc->sc_sysctllog, 0, NULL, &node, 0,
	    CTLTYPE_NODE, device_xname(sc->sc_dev),
	    SYSCTL_DESCR("CAN controller"), NULL, 0, NULL, 0,
	    CTL_HW, CTL_CREATE, CTL_EOL) != 0)
		return;
	sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL, CTLFLAG_READWRITE,
	    CTLTYPE_INT, "bitrate",
	    SYSCTL_DESCR("bit rate (sets the timing; interface down)"),
	    esp_twai_sysctl_bitrate, 0, (void *)sc, 0, CTL_CREATE, CTL_EOL);
	sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL, CTLFLAG_READONLY,
	    CTLTYPE_STRING, "state", SYSCTL_DESCR("bus state, error counts"),
	    esp_twai_sysctl_state, 0, (void *)sc, 0, CTL_CREATE, CTL_EOL);
}

static int
esp_twai_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_twai_attach(device_t parent, device_t self, void *aux)
{
	struct esp_twai_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct ifnet *ifp;
	struct clk *clk;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	uint32_t id;

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
	sc->sc_clkfreq = clk_get_rate(clk);
	if ((rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		fdtbus_reset_assert(rst);
		fdtbus_reset_deassert(rst);
	}
	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": couldn't decode interrupt\n");
		return;
	}
	if (esp_twai_reset(sc) != 0) {
		aprint_error(": no CTU CAN FD core\n");
		return;
	}
	id = RD(sc, CTU_DEVICE_ID);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NET);

	aprint_naive("\n");
	aprint_normal(": CAN controller (CTU CAN FD %u.%u), %u MHz\n",
	    (u_int)(id >> 24), (u_int)((id >> 16) & 0xff),
	    sc->sc_clkfreq / 1000000);

	sc->sc_timecaps.cltc_prop_min = 1;
	sc->sc_timecaps.cltc_prop_max = 127;
	sc->sc_timecaps.cltc_ps1_min = 1;
	sc->sc_timecaps.cltc_ps1_max = 63;
	sc->sc_timecaps.cltc_ps2_min = 1;
	sc->sc_timecaps.cltc_ps2_max = 63;
	sc->sc_timecaps.cltc_sjw_max = 31;
	sc->sc_timecaps.cltc_brp_min = 1;
	sc->sc_timecaps.cltc_brp_max = 255;
	sc->sc_timecaps.cltc_brp_inc = 1;
	sc->sc_timecaps.cltc_clock_freq = sc->sc_clkfreq;
	sc->sc_timecaps.cltc_linkmode_caps = CAN_LINKMODE_LOOPBACK |
	    CAN_LINKMODE_LISTENONLY | CAN_LINKMODE_PRESUME_ACK;
	can_ifinit_timings(&sc->sc_cansc);
	(void)esp_twai_calc_timing(sc->sc_clkfreq, 500000, &sc->sc_timings);
	sc->sc_state = "stopped";

	evcnt_attach_dynamic(&sc->sc_ev_fd, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "CAN FD frames dropped");
	evcnt_attach_dynamic(&sc->sc_ev_busoff, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "bus-off");
	evcnt_attach_dynamic(&sc->sc_ev_overrun, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "RX overruns");

	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_NET,
	    FDT_INTR_MPSAFE, esp_twai_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	ifp = if_alloc(IFT_OTHER);
	sc->sc_ifp = ifp;
	strlcpy(ifp->if_xname, device_xname(self), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_flags = 0;
	ifp->if_start = esp_twai_ifstart;
	ifp->if_ioctl = esp_twai_ifioctl;
	ifp->if_watchdog = esp_twai_ifwatchdog;
	can_ifattach(ifp);
	if_deferred_start_init(ifp, NULL);
	bpf_mtap_softint_init(ifp);
	rnd_attach_source(&sc->sc_rnd, device_xname(self), RND_TYPE_NET,
	    RND_FLAG_DEFAULT);

	esp_twai_sysctl_init(sc);
}
