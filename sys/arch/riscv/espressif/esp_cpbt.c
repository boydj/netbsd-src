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
 * espbt(4): the ESP32-S31's Bluetooth controller (BR/EDR and LE), which
 * runs in the radio co-processor's firmware (espcp(4)), as an HCI
 * transport for bluetooth(4).  Packets go both ways with their H4 type
 * byte, as the stack keeps them.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <netbt/bluetooth.h>
#include <netbt/hci.h>

#include <riscv/espressif/esp_cpvar.h>

struct espbt_softc {
	device_t		sc_dev;
	device_t		sc_cp;
	struct hci_unit		*sc_unit;

	kmutex_t		sc_lock;
	bool			sc_enabled;
	MBUFQ_HEAD()		sc_cmdq;
	MBUFQ_HEAD()		sc_aclq;
	MBUFQ_HEAD()		sc_scoq;
	struct bt_stats		sc_stats;
};

static int	espbt_match(device_t, cfdata_t, void *);
static void	espbt_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(espbt, sizeof(struct espbt_softc),
	espbt_match, espbt_attach, NULL, NULL);

static int	espbt_enable(device_t);
static void	espbt_disable(device_t);
static void	espbt_output_cmd(device_t, struct mbuf *);
static void	espbt_output_acl(device_t, struct mbuf *);
static void	espbt_output_sco(device_t, struct mbuf *);
static void	espbt_stats(device_t, struct bt_stats *, int);
static void	espbt_rx(void *, uint8_t, const void *, size_t);
static void	espbt_space(void *);

static const struct hci_if espbt_hci = {
	.enable =	espbt_enable,
	.disable =	espbt_disable,
	.output_cmd =	espbt_output_cmd,
	.output_acl =	espbt_output_acl,
	.output_sco =	espbt_output_sco,
	.get_stats =	espbt_stats,
	.ipl =		IPL_SOFTNET,
};

static int
espbt_match(device_t parent, cfdata_t cf, void *aux)
{
	const struct espcp_attach_args * const ea = aux;

	return strcmp(ea->ea_name, "espbt") == 0;
}

static void
espbt_attach(device_t parent, device_t self, void *aux)
{
	struct espbt_softc * const sc = device_private(self);
	const struct espcp_attach_args * const ea = aux;

	sc->sc_dev = self;
	sc->sc_cp = parent;
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_SOFTNET);
	MBUFQ_INIT(&sc->sc_cmdq);
	MBUFQ_INIT(&sc->sc_aclq);
	MBUFQ_INIT(&sc->sc_scoq);

	aprint_naive("\n");
	aprint_normal(": Bluetooth BR/EDR + LE controller, address "
	    "%02x:%02x:%02x:%02x:%02x:%02x\n",
	    ea->ea_addr[0], ea->ea_addr[1], ea->ea_addr[2],
	    ea->ea_addr[3], ea->ea_addr[4], ea->ea_addr[5]);

	espcp_register(parent, ESPCP_CHAN_HCI, espbt_rx, espbt_space, sc);
	sc->sc_unit = hci_attach_pcb(&espbt_hci, self, 0);
	if (sc->sc_unit == NULL)
		aprint_error_dev(self, "can't attach to bluetooth(4)\n");
}

/*
 * Push queued packets into the ring until it is full.  SCO packets that
 * the stack wants to hear about are handed back once they are gone.
 */
static void
espbt_kick(struct espbt_softc *sc)
{
	MBUFQ_HEAD() done;
	struct mbuf *m;
	int error;

	MBUFQ_INIT(&done);
	mutex_enter(&sc->sc_lock);
	while (sc->sc_enabled) {
		uint32_t *cnt;

		if ((m = MBUFQ_FIRST(&sc->sc_cmdq)) != NULL)
			cnt = &sc->sc_stats.cmd_tx;
		else if ((m = MBUFQ_FIRST(&sc->sc_scoq)) != NULL)
			cnt = &sc->sc_stats.sco_tx;
		else if ((m = MBUFQ_FIRST(&sc->sc_aclq)) != NULL)
			cnt = &sc->sc_stats.acl_tx;
		else
			break;
		error = espcp_send(sc->sc_cp, ESPCP_CHAN_HCI,
		    *mtod(m, uint8_t *), NULL, 0, m);
		if (error == ENOBUFS)
			break;		/* espbt_space() calls us again */
		if (cnt == &sc->sc_stats.cmd_tx)
			MBUFQ_DEQUEUE(&sc->sc_cmdq, m);
		else if (cnt == &sc->sc_stats.sco_tx)
			MBUFQ_DEQUEUE(&sc->sc_scoq, m);
		else
			MBUFQ_DEQUEUE(&sc->sc_aclq, m);
		if (error != 0) {
			sc->sc_stats.err_tx++;
		} else {
			(*cnt)++;
			sc->sc_stats.byte_tx += m->m_pkthdr.len;
		}
		MBUFQ_ENQUEUE(&done, m);
	}
	mutex_exit(&sc->sc_lock);

	while ((m = MBUFQ_FIRST(&done)) != NULL) {
		MBUFQ_DEQUEUE(&done, m);
		if (*mtod(m, uint8_t *) == HCI_SCO_DATA_PKT &&
		    M_GETCTX(m, void *) != NULL)
			(void)hci_complete_sco(sc->sc_unit, m);
		else
			m_freem(m);
	}
}

static void
espbt_space(void *arg)
{
	espbt_kick(arg);
}

static struct mbuf *
espbt_mbuf(const void *data, size_t len)
{
	struct mbuf *m;

	if (len > MCLBYTES)
		return NULL;
	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return NULL;
	if (len > MHLEN) {
		MCLGET(m, M_DONTWAIT);
		if ((m->m_flags & M_EXT) == 0) {
			m_freem(m);
			return NULL;
		}
	}
	memcpy(mtod(m, void *), data, len);
	m->m_len = m->m_pkthdr.len = len;
	return m;
}

static void
espbt_rx(void *arg, uint8_t type, const void *data, size_t len)
{
	struct espbt_softc * const sc = arg;
	struct mbuf *m;
	bool ok;

	if (!sc->sc_enabled || len < 2)
		return;
	m = espbt_mbuf(data, len);
	if (m == NULL) {
		sc->sc_stats.err_rx++;
		return;
	}
	sc->sc_stats.byte_rx += len;
	switch (type) {
	case HCI_EVENT_PKT:
		sc->sc_stats.evt_rx++;
		ok = hci_input_event(sc->sc_unit, m);
		break;
	case HCI_ACL_DATA_PKT:
		sc->sc_stats.acl_rx++;
		ok = hci_input_acl(sc->sc_unit, m);
		break;
	case HCI_SCO_DATA_PKT:
		sc->sc_stats.sco_rx++;
		ok = hci_input_sco(sc->sc_unit, m);
		break;
	default:
		m_freem(m);
		ok = false;
		break;
	}
	if (!ok)
		sc->sc_stats.err_rx++;
}

static int
espbt_enable(device_t self)
{
	struct espbt_softc * const sc = device_private(self);
	int error;

	if (sc->sc_enabled)
		return 0;
	/*
	 * Turn the controller on.  The reply comes back through the ring
	 * before anything the stack sends after this, so don't wait for it.
	 */
	error = espcp_ctl_async(sc->sc_cp, ESPCP_CTL_BT_ENABLE, NULL, 0);
	if (error != 0)
		return error;
	mutex_enter(&sc->sc_lock);
	sc->sc_enabled = true;
	mutex_exit(&sc->sc_lock);
	return 0;
}

static void
espbt_disable(device_t self)
{
	struct espbt_softc * const sc = device_private(self);

	mutex_enter(&sc->sc_lock);
	sc->sc_enabled = false;
	MBUFQ_DRAIN(&sc->sc_cmdq);
	MBUFQ_DRAIN(&sc->sc_aclq);
	MBUFQ_DRAIN(&sc->sc_scoq);
	mutex_exit(&sc->sc_lock);
	(void)espcp_ctl_async(sc->sc_cp, ESPCP_CTL_BT_DISABLE, NULL, 0);
}

static void
espbt_output(struct espbt_softc *sc, struct mbuf *m, int q)
{
	mutex_enter(&sc->sc_lock);
	if (!sc->sc_enabled) {
		mutex_exit(&sc->sc_lock);
		m_freem(m);
		return;
	}
	switch (q) {
	case HCI_CMD_PKT:
		MBUFQ_ENQUEUE(&sc->sc_cmdq, m);
		break;
	case HCI_ACL_DATA_PKT:
		MBUFQ_ENQUEUE(&sc->sc_aclq, m);
		break;
	default:
		MBUFQ_ENQUEUE(&sc->sc_scoq, m);
		break;
	}
	mutex_exit(&sc->sc_lock);
	espbt_kick(sc);
}

static void
espbt_output_cmd(device_t self, struct mbuf *m)
{
	M_SETCTX(m, NULL);
	espbt_output(device_private(self), m, HCI_CMD_PKT);
}

static void
espbt_output_acl(device_t self, struct mbuf *m)
{
	M_SETCTX(m, NULL);
	espbt_output(device_private(self), m, HCI_ACL_DATA_PKT);
}

static void
espbt_output_sco(device_t self, struct mbuf *m)
{
	espbt_output(device_private(self), m, HCI_SCO_DATA_PKT);
}

static void
espbt_stats(device_t self, struct bt_stats *dest, int flush)
{
	struct espbt_softc * const sc = device_private(self);

	mutex_enter(&sc->sc_lock);
	memcpy(dest, &sc->sc_stats, sizeof(*dest));
	if (flush)
		memset(&sc->sc_stats, 0, sizeof(sc->sc_stats));
	mutex_exit(&sc->sc_lock);
}
