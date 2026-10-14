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
 * espwifi(4): the ESP32-S31's Wi-Fi, in station mode, through the radio
 * co-processor (espcp(4)).
 *
 * The firmware does scanning, association and WPA2/WPA3 itself, so this is
 * an Ethernet interface whose link is up while associated.  espwifictl(8)
 * scans and picks the network through driver-specific ioctls; the
 * interface going up starts the radio and (re)joins the chosen network.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/syslog.h>
#include <sys/systm.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_ether.h>
#include <net/if_media.h>

#include <riscv/espressif/esp_cpvar.h>

struct espwifi_softc {
	device_t		sc_dev;
	device_t		sc_cp;
	struct ethercom		sc_ec;
	uint8_t			sc_enaddr[ETHER_ADDR_LEN];

	kmutex_t		sc_lock;
	struct espcp_wifi_config sc_cfg;
	bool			sc_cfgset;
	struct espcp_wifi_status sc_status;
};

static int	espwifi_match(device_t, cfdata_t, void *);
static void	espwifi_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(espwifi, sizeof(struct espwifi_softc),
	espwifi_match, espwifi_attach, NULL, NULL);

static int	espwifi_init(struct ifnet *);
static void	espwifi_stop(struct ifnet *, int);
static void	espwifi_start(struct ifnet *);
static int	espwifi_ioctl(struct ifnet *, u_long, void *);
static void	espwifi_rx(void *, uint8_t, const void *, size_t);
static void	espwifi_space(void *);
static void	espwifi_event(void *, uint8_t, const void *, size_t);

#define	ESPWIFI_CTL_TIMO	5000	/* ms */
#define	ESPWIFI_SCAN_TIMO	20000

static int
espwifi_match(device_t parent, cfdata_t cf, void *aux)
{
	const struct espcp_attach_args * const ea = aux;

	return strcmp(ea->ea_name, "espwifi") == 0;
}

static void
espwifi_attach(device_t parent, device_t self, void *aux)
{
	struct espwifi_softc * const sc = device_private(self);
	const struct espcp_attach_args * const ea = aux;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;

	sc->sc_dev = self;
	sc->sc_cp = parent;
	memcpy(sc->sc_enaddr, ea->ea_addr, ETHER_ADDR_LEN);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);
	sc->sc_status.ws_state = ESPCP_WIFI_OFF;

	aprint_naive("\n");
	aprint_normal(": Wi-Fi station, address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	strlcpy(ifp->if_xname, device_xname(self), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = espwifi_init;
	ifp->if_stop = espwifi_stop;
	ifp->if_start = espwifi_start;
	ifp->if_ioctl = espwifi_ioctl;
	IFQ_SET_MAXLEN(&ifp->if_snd, 128);
	IFQ_SET_READY(&ifp->if_snd);

	if_initialize(ifp);
	ifp->if_percpuq = if_percpuq_create(ifp);
	ifp->if_link_state = LINK_STATE_DOWN;
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);
	if_register(ifp);

	espcp_register(parent, ESPCP_CHAN_WIFI, espwifi_rx, espwifi_space, sc);
	espcp_register_events(parent, espwifi_event, sc);
}

/* ------------------------------------------------------------------------ */

static void
espwifi_event(void *arg, uint8_t type, const void *data, size_t len)
{
	struct espwifi_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct espcp_wifi_status ws;
	char ssid[sizeof(ws.ws_ssid) + 1];

	if (type != ESPCP_EVT_WIFI_LINK || len < sizeof(ws))
		return;
	memcpy(&ws, data, sizeof(ws));
	const bool was = sc->sc_status.ws_state == ESPCP_WIFI_CONNECTED;
	const bool is = ws.ws_state == ESPCP_WIFI_CONNECTED;
	sc->sc_status = ws;

	memcpy(ssid, ws.ws_ssid, MIN(ws.ws_ssid_len, sizeof(ws.ws_ssid)));
	ssid[MIN(ws.ws_ssid_len, sizeof(ws.ws_ssid))] = '\0';
	if (is && !was) {
		log(LOG_INFO, "%s: associated with \"%s\" (%s), channel %u\n",
		    ifp->if_xname, ssid, ether_sprintf(ws.ws_bssid),
		    ws.ws_channel);
		if_link_state_change(ifp, LINK_STATE_UP);
	} else if (!is && was) {
		log(LOG_INFO, "%s: disassociated, reason %u\n",
		    ifp->if_xname, ws.ws_reason);
		if_link_state_change(ifp, LINK_STATE_DOWN);
	}
}

static void
espwifi_rx(void *arg, uint8_t type, const void *data, size_t len)
{
	struct espwifi_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct mbuf *m;

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;
	if (len < ETHER_HDR_LEN || len > ETHER_ALIGN + MCLBYTES ||
	    len > ETHER_MAX_LEN + ETHER_VLAN_ENCAP_LEN) {
		if_statinc(ifp, if_ierrors);
		return;
	}
	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m != NULL && len + ETHER_ALIGN > MHLEN) {
		MCLGET(m, M_DONTWAIT);
		if ((m->m_flags & M_EXT) == 0) {
			m_freem(m);
			m = NULL;
		}
	}
	if (m == NULL) {
		if_statinc(ifp, if_ierrors);
		return;
	}
	m->m_data += ETHER_ALIGN;
	memcpy(mtod(m, void *), data, len);
	m->m_len = m->m_pkthdr.len = len;
	m_set_rcvif(m, ifp);
	if_percpuq_enqueue(ifp->if_percpuq, m);
}

static void
espwifi_space(void *arg)
{
	struct espwifi_softc * const sc = arg;

	if_schedule_deferred_start(&sc->sc_ec.ec_if);
}

static void
espwifi_start(struct ifnet *ifp)
{
	struct espwifi_softc * const sc = ifp->if_softc;
	struct mbuf *m;
	int error;

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;
	for (;;) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;
		if (sc->sc_status.ws_state != ESPCP_WIFI_CONNECTED) {
			IFQ_DEQUEUE(&ifp->if_snd, m);
			if_statinc(ifp, if_oerrors);
			m_freem(m);
			continue;
		}
		error = espcp_send(sc->sc_cp, ESPCP_CHAN_WIFI, 0, NULL, 0, m);
		if (error == ENOBUFS)
			break;		/* espwifi_space() restarts us */
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (error != 0) {
			if_statinc(ifp, if_oerrors);
		} else {
			if_statinc(ifp, if_opackets);
			if_statadd(ifp, if_obytes, m->m_pkthdr.len);
			bpf_mtap(ifp, m, BPF_D_OUT);
		}
		m_freem(m);
	}
}

/* ------------------------------------------------------------------------ */

static int
espwifi_join(struct espwifi_softc *sc)
{
	struct espcp_wifi_config cfg;

	mutex_enter(&sc->sc_lock);
	cfg = sc->sc_cfg;
	const bool set = sc->sc_cfgset;
	mutex_exit(&sc->sc_lock);
	if (!set)
		return 0;
	return espcp_ctl(sc->sc_cp, ESPCP_CTL_WIFI_CONNECT, &cfg, sizeof(cfg),
	    NULL, NULL, ESPWIFI_CTL_TIMO);
}

static int
espwifi_init(struct ifnet *ifp)
{
	struct espwifi_softc * const sc = ifp->if_softc;
	int error;

	error = espcp_ctl(sc->sc_cp, ESPCP_CTL_WIFI_START, NULL, 0, NULL, NULL,
	    ESPWIFI_CTL_TIMO);
	if (error != 0)
		return error;
	if (sc->sc_status.ws_state == ESPCP_WIFI_OFF)
		sc->sc_status.ws_state = ESPCP_WIFI_IDLE;
	ifp->if_flags |= IFF_RUNNING;
	error = espwifi_join(sc);
	if (error != 0)
		log(LOG_WARNING, "%s: can't join: error %d\n", ifp->if_xname,
		    error);
	return 0;
}

static void
espwifi_stop(struct ifnet *ifp, int disable)
{
	struct espwifi_softc * const sc = ifp->if_softc;

	ifp->if_flags &= ~IFF_RUNNING;
	(void)espcp_ctl(sc->sc_cp, ESPCP_CTL_WIFI_STOP, NULL, 0, NULL, NULL,
	    ESPWIFI_CTL_TIMO);
	sc->sc_status.ws_state = ESPCP_WIFI_OFF;
	if_link_state_change(ifp, LINK_STATE_DOWN);
}

static int
espwifi_drvspec(struct espwifi_softc *sc, u_long cmd, struct ifdrv *ifd)
{
	const bool set = cmd == SIOCSDRVSPEC;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	void *buf;
	size_t len;
	int error;

	switch (ifd->ifd_cmd) {
	case ESPWIFI_SCAN:
		if (set)
			return EINVAL;
		len = sizeof(struct espcp_wifi_scan) +
		    ESPCP_SCAN_MAX * sizeof(struct espcp_wifi_ap);
		buf = kmem_zalloc(len, KM_SLEEP);
		error = espcp_ctl(sc->sc_cp, ESPCP_CTL_WIFI_SCAN, NULL, 0, buf,
		    &len, ESPWIFI_SCAN_TIMO);
		if (error == 0) {
			error = copyout(buf, ifd->ifd_data,
			    MIN(len, ifd->ifd_len));
			ifd->ifd_len = len;
		}
		kmem_free(buf, sizeof(struct espcp_wifi_scan) +
		    ESPCP_SCAN_MAX * sizeof(struct espcp_wifi_ap));
		return error;

	case ESPWIFI_STATUS: {
		struct espcp_wifi_status ws;

		if (set)
			return EINVAL;
		len = sizeof(ws);
		error = espcp_ctl(sc->sc_cp, ESPCP_CTL_WIFI_STATUS, NULL, 0,
		    &ws, &len, ESPWIFI_CTL_TIMO);
		if (error == ENETDOWN) {
			memset(&ws, 0, sizeof(ws));
			ws.ws_state = ESPCP_WIFI_OFF;
			error = 0;
		}
		if (error == 0)
			error = copyout(&ws, ifd->ifd_data,
			    MIN(sizeof(ws), ifd->ifd_len));
		return error;
	}

	case ESPWIFI_JOIN: {
		struct espcp_wifi_config cfg;

		if (!set || ifd->ifd_len != sizeof(cfg))
			return EINVAL;
		error = copyin(ifd->ifd_data, &cfg, sizeof(cfg));
		if (error != 0)
			return error;
		cfg.wc_ssid[sizeof(cfg.wc_ssid) - 1] = '\0';
		cfg.wc_password[sizeof(cfg.wc_password) - 1] = '\0';
		if (cfg.wc_ssid[0] == '\0')
			return EINVAL;
		mutex_enter(&sc->sc_lock);
		sc->sc_cfg = cfg;
		sc->sc_cfgset = true;
		mutex_exit(&sc->sc_lock);
		explicit_memset(&cfg, 0, sizeof(cfg));
		if ((ifp->if_flags & IFF_RUNNING) == 0)
			return 0;	/* on the way up */
		return espwifi_join(sc);
	}

	case ESPWIFI_LEAVE:
		if (!set)
			return EINVAL;
		mutex_enter(&sc->sc_lock);
		explicit_memset(&sc->sc_cfg, 0, sizeof(sc->sc_cfg));
		sc->sc_cfgset = false;
		mutex_exit(&sc->sc_lock);
		if ((ifp->if_flags & IFF_RUNNING) == 0)
			return 0;
		return espcp_ctl(sc->sc_cp, ESPCP_CTL_WIFI_DISCONNECT, NULL, 0,
		    NULL, NULL, ESPWIFI_CTL_TIMO);

	default:
		return EINVAL;
	}
}

static int
espwifi_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct espwifi_softc * const sc = ifp->if_softc;
	int s, error;

	switch (cmd) {
	case SIOCGDRVSPEC:
	case SIOCSDRVSPEC:
		error = espwifi_drvspec(sc, cmd, data);
		break;
	default:
		s = splnet();
		error = ether_ioctl(ifp, cmd, data);
		splx(s);
		if (error == ENETRESET)
			error = 0;	/* the firmware takes all multicast */
		break;
	}
	return error;
}
