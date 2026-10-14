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
 * ESP32-S31 USB-Serial/JTAG controller: a fixed-function USB CDC-ACM
 * device on the chip's full-speed USB pins (the dev board's "USB" port).
 * It shows up on the host as /dev/ttyACM* or /dev/cu.usbmodem*, with no
 * USB stack needed here: endpoint 1's 64-byte FIFOs are all there is.
 *
 * It can be the console (stdout-path in the device tree), or an extra
 * terminal (ttyUSJ0) with a login on it.  With no host attached nothing
 * drains the transmit FIFO, so console output gives up after a short
 * wait instead of hanging the kernel.
 */

#include "opt_console.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#define cn_trap()			\
	do {				\
		console_debugger();	\
		cn_trapped = 1;		\
	} while (/* CONSTCOND */ 0)

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/fcntl.h>
#include <sys/intr.h>
#include <sys/kauth.h>
#include <sys/lwp.h>
#include <sys/systm.h>
#include <sys/termios.h>
#include <sys/tty.h>

#include <ddb/db_active.h>

#include <dev/cons.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_console.h>

#include <riscv/espressif/esp32s31_reg.h>

static int	esp_usj_match(device_t, cfdata_t, void *);
static void	esp_usj_attach(device_t, device_t, void *);

static int	esp_usj_intr(void *);
static void	esp_usj_softintr(void *);

static int	esp_usj_cngetc(dev_t);
static void	esp_usj_cnputc(dev_t, int);
static void	esp_usj_cnpollc(dev_t, int);

static void	esp_usj_start(struct tty *);
static int	esp_usj_param(struct tty *, struct termios *);

extern struct cfdriver espusj_cd;

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "esp,esp32s31-usb-serial-jtag" },
	{ .compat = "esp,esp32s3-acm" },
	DEVICE_COMPAT_EOL
};

#define	USJ_RBUFSZ	512		/* must be 2^n */

/*
 * Send at most this much per packet.  A short packet ends the host's
 * transfer at once; a full 64-byte one could sit in the host's buffer
 * until more data arrives.
 */
#define	USJ_PKT_MAX	(USJ_FIFO_SIZE - 1)

/* How long console output waits for a host to take a packet. */
#define	USJ_CN_SPIN	200000

struct esp_usj_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	kmutex_t		sc_lock;
	void			*sc_ih;
	void			*sc_sih;

	struct tty		*sc_tty;

	/* receive ring, filled by the hard interrupt */
	volatile u_int		sc_rbuf_w;
	volatile u_int		sc_rbuf_r;
	u_char			sc_rbuf[USJ_RBUFSZ];

	/* transmit state */
	u_char			*sc_tba;	/* next byte to send */
	u_int			sc_tbc;		/* bytes left */
	volatile bool		sc_tx_done;
	bool			sc_tx_busy;
};

static int esp_usj_console_phandle = -1;
static bus_space_tag_t esp_usj_cn_bst;
static bus_space_handle_t esp_usj_cn_bsh;
static struct cnm_state esp_usj_cnm_state;
static u_int esp_usj_cn_pending;	/* bytes in the FIFO, not yet sent */
static bool esp_usj_cn_stalled;		/* no host is reading */

static struct consdev esp_usj_consdev = {
	.cn_getc = esp_usj_cngetc,
	.cn_putc = esp_usj_cnputc,
	.cn_pollc = esp_usj_cnpollc,
	.cn_dev = NODEV,
	.cn_pri = CN_NORMAL,
};

static dev_type_open(esp_usj_open);
static dev_type_close(esp_usj_close);
static dev_type_read(esp_usj_read);
static dev_type_write(esp_usj_write);
static dev_type_ioctl(esp_usj_ioctl);
static dev_type_tty(esp_usj_tty);
static dev_type_poll(esp_usj_poll);
static dev_type_stop(esp_usj_stop);

const struct cdevsw espusj_cdevsw = {
	.d_open = esp_usj_open,
	.d_close = esp_usj_close,
	.d_read = esp_usj_read,
	.d_write = esp_usj_write,
	.d_ioctl = esp_usj_ioctl,
	.d_stop = esp_usj_stop,
	.d_tty = esp_usj_tty,
	.d_poll = esp_usj_poll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard = nodiscard,
	.d_flag = D_TTY
};

#define	RD4(sc, reg)		\
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR4(sc, reg, val)	\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static inline bool
esp_usj_tx_free(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	return (bus_space_read_4(bst, bsh, USJ_EP1_CONF) &
	    USJ_EP1_CONF_IN_FREE) != 0;
}

static inline bool
esp_usj_rx_avail(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	return (bus_space_read_4(bst, bsh, USJ_EP1_CONF) &
	    USJ_EP1_CONF_OUT_AVAIL) != 0;
}

CFATTACH_DECL_NEW(esp_usj, sizeof(struct esp_usj_softc),
	esp_usj_match, esp_usj_attach, NULL, NULL);

static int
esp_usj_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_usj_attach(device_t parent, device_t self, void *aux)
{
	struct esp_usj_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	struct tty *tp;
	int major;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_SERIAL);

	WR4(sc, USJ_INT_ENA, 0);
	WR4(sc, USJ_INT_CLR, 0xffffffff);

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr)))
		strlcpy(intrstr, "?", sizeof(intrstr));

	sc->sc_sih = softint_establish(SOFTINT_SERIAL, esp_usj_softintr, sc);
	if (sc->sc_sih == NULL) {
		aprint_error(": failed to establish softint\n");
		return;
	}

	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_SERIAL,
	    FDT_INTR_MPSAFE, esp_usj_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error(": failed to establish interrupt on %s\n",
		    intrstr);
		return;
	}

	major = cdevsw_lookup_major(&espusj_cdevsw);

	tp = sc->sc_tty = tty_alloc();
	tp->t_oproc = esp_usj_start;
	tp->t_param = esp_usj_param;
	tp->t_dev = makedev(major, device_unit(self));
	tp->t_sc = sc;
	tty_attach(tp);

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 USB-Serial/JTAG");
	if (esp_usj_console_phandle == phandle) {
		cn_tab->cn_dev = tp->t_dev;
		aprint_normal(", console");
	}
	aprint_normal("\n");
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	/* Always listen, so the console gets ddb and early input. */
	WR4(sc, USJ_INT_ENA, USJ_INT_OUT_RECV_PKT);
}

/*
 * Console.  Bytes are collected in the FIFO and sent at a newline, when
 * the FIFO is full, or when the console turns to reading.
 */
static void
esp_usj_cnflush(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	if (esp_usj_cn_pending > 0) {
		bus_space_write_4(bst, bsh, USJ_EP1_CONF, USJ_EP1_CONF_WR_DONE);
		esp_usj_cn_pending = 0;
	}
}

static int
esp_usj_cngetc(dev_t dev)
{
	const bus_space_tag_t bst = esp_usj_cn_bst;
	const bus_space_handle_t bsh = esp_usj_cn_bsh;
	int s, c;

	s = splserial();
	esp_usj_cnflush(bst, bsh);
	if (!esp_usj_rx_avail(bst, bsh)) {
		splx(s);
		return -1;
	}
	c = bus_space_read_4(bst, bsh, USJ_EP1) & 0xff;
	if (!db_active) {
		int cn_trapped __unused = 0;
		cn_check_magic(dev, c, esp_usj_cnm_state);
	}
	splx(s);

	return c;
}

static void
esp_usj_cnputc(dev_t dev, int c)
{
	const bus_space_tag_t bst = esp_usj_cn_bst;
	const bus_space_handle_t bsh = esp_usj_cn_bsh;
	u_int spin;
	int s;

	s = splserial();
	/*
	 * Wait for room.  If no host has taken a packet for a while, stop
	 * waiting at all until one does, and drop what doesn't fit.
	 */
	for (spin = esp_usj_cn_stalled ? 1 : USJ_CN_SPIN; spin > 0; spin--) {
		if (esp_usj_tx_free(bst, bsh))
			break;
	}
	if (spin == 0) {
		esp_usj_cn_stalled = true;
		esp_usj_cn_pending = 0;
		splx(s);
		return;
	}
	esp_usj_cn_stalled = false;
	bus_space_write_4(bst, bsh, USJ_EP1, c & 0xff);
	if (++esp_usj_cn_pending >= USJ_PKT_MAX || c == '\n')
		esp_usj_cnflush(bst, bsh);
	splx(s);
}

static void
esp_usj_cnpollc(dev_t dev, int on)
{
	int s;

	s = splserial();
	esp_usj_cnflush(esp_usj_cn_bst, esp_usj_cn_bsh);
	splx(s);
}

static int
esp_usj_open(dev_t dev, int flag, int mode, lwp_t *l)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));
	struct tty *tp;

	if (sc == NULL || sc->sc_tty == NULL)
		return ENXIO;
	tp = sc->sc_tty;

	if (kauth_authorize_device_tty(l->l_cred,
	    KAUTH_DEVICE_TTY_OPEN, tp) != 0)
		return EBUSY;

	if ((tp->t_state & TS_ISOPEN) == 0 && tp->t_wopen == 0) {
		tp->t_dev = dev;
		ttychars(tp);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_cflag = TTYDEF_CFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		tp->t_ispeed = tp->t_ospeed = 115200;
		ttsetwater(tp);
	}
	tp->t_state |= TS_CARR_ON;

	int error = ttyopen(tp, 0, ISSET(flag, O_NONBLOCK));
	if (error)
		return error;

	return tp->t_linesw->l_open(dev, tp);
}

static int
esp_usj_close(dev_t dev, int flag, int mode, lwp_t *l)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	tp->t_linesw->l_close(tp, flag);
	ttyclose(tp);

	return 0;
}

static int
esp_usj_read(dev_t dev, struct uio *uio, int flag)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	return tp->t_linesw->l_read(tp, uio, flag);
}

static int
esp_usj_write(dev_t dev, struct uio *uio, int flag)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	return tp->t_linesw->l_write(tp, uio, flag);
}

static int
esp_usj_poll(dev_t dev, int events, lwp_t *l)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	return tp->t_linesw->l_poll(tp, events, l);
}

static int
esp_usj_ioctl(dev_t dev, u_long cmd, void *data, int flag, lwp_t *l)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;
	int error;

	error = tp->t_linesw->l_ioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	return ttioctl(tp, cmd, data, flag, l);
}

static struct tty *
esp_usj_tty(dev_t dev)
{
	struct esp_usj_softc * const sc =
	    device_lookup_private(&espusj_cd, minor(dev));

	return sc->sc_tty;
}

static void
esp_usj_stop(struct tty *tp, int flag)
{
	struct esp_usj_softc * const sc = tp->t_sc;

	mutex_spin_enter(&sc->sc_lock);
	if (sc->sc_tx_busy) {
		sc->sc_tbc = 0;
		if (!ISSET(tp->t_state, TS_TTSTOP))
			SET(tp->t_state, TS_FLUSH);
	}
	mutex_spin_exit(&sc->sc_lock);
}

/*
 * Put the next packet of the current chunk in the FIFO and send it.
 * Called with sc_lock held, when the FIFO has room.
 */
static void
esp_usj_txfill(struct esp_usj_softc *sc)
{
	u_int n = 0;

	while (n < USJ_PKT_MAX && sc->sc_tbc > 0 &&
	    esp_usj_tx_free(sc->sc_bst, sc->sc_bsh)) {
		WR4(sc, USJ_EP1, *sc->sc_tba);
		sc->sc_tba++;
		sc->sc_tbc--;
		n++;
	}
	/* The console may have left bytes behind too; send them all. */
	if (n > 0 || esp_usj_cn_pending > 0) {
		WR4(sc, USJ_EP1_CONF, USJ_EP1_CONF_WR_DONE);
		esp_usj_cn_pending = 0;
	}
}

static void
esp_usj_start(struct tty *tp)
{
	struct esp_usj_softc * const sc = tp->t_sc;
	int s;

	s = spltty();
	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP))
		goto out;
	if (!ttypull(tp))
		goto out;

	u_char *tba = tp->t_outq.c_cf;
	u_int tbc = ndqb(&tp->t_outq, 0);
	if (tbc == 0)
		goto out;

	SET(tp->t_state, TS_BUSY);

	mutex_spin_enter(&sc->sc_lock);
	sc->sc_tba = tba;
	sc->sc_tbc = tbc;
	sc->sc_tx_busy = true;
	sc->sc_tx_done = false;
	/*
	 * Clear a stale "IN empty" before sending, so the one that counts
	 * is the host taking this packet; the interrupt sends the rest.
	 */
	WR4(sc, USJ_INT_CLR, USJ_INT_IN_EMPTY);
	if (esp_usj_tx_free(sc->sc_bst, sc->sc_bsh))
		esp_usj_txfill(sc);
	WR4(sc, USJ_INT_ENA, RD4(sc, USJ_INT_ENA) | USJ_INT_IN_EMPTY);
	mutex_spin_exit(&sc->sc_lock);
out:
	splx(s);
}

static int
esp_usj_param(struct tty *tp, struct termios *t)
{
	/* There is no line: the host's settings don't matter. */
	tp->t_ispeed = t->c_ispeed;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	return 0;
}

static int
esp_usj_intr(void *priv)
{
	struct esp_usj_softc * const sc = priv;
	struct tty * const tp = sc->sc_tty;
	uint32_t st;
	bool sched = false;

	mutex_spin_enter(&sc->sc_lock);
	st = RD4(sc, USJ_INT_ST);
	if (st == 0) {
		mutex_spin_exit(&sc->sc_lock);
		return 0;
	}
	WR4(sc, USJ_INT_CLR, st);

	while (esp_usj_rx_avail(sc->sc_bst, sc->sc_bsh)) {
		int cn_trapped = 0;
		const int c = RD4(sc, USJ_EP1) & 0xff;

		if (tp != NULL) {
			cn_check_magic(tp->t_dev, c, esp_usj_cnm_state);
			if (cn_trapped)
				continue;
		}
		if (sc->sc_rbuf_w - sc->sc_rbuf_r >= USJ_RBUFSZ - 1)
			continue;	/* overflow: drop */
		sc->sc_rbuf[sc->sc_rbuf_w & (USJ_RBUFSZ - 1)] = c;
		sc->sc_rbuf_w++;
		sched = true;
	}

	if ((st & USJ_INT_IN_EMPTY) != 0 &&
	    esp_usj_tx_free(sc->sc_bst, sc->sc_bsh)) {
		if (sc->sc_tbc > 0) {
			esp_usj_txfill(sc);
		} else {
			WR4(sc, USJ_INT_ENA,
			    RD4(sc, USJ_INT_ENA) & ~USJ_INT_IN_EMPTY);
			if (sc->sc_tx_busy) {
				sc->sc_tx_busy = false;
				sc->sc_tx_done = true;
				sched = true;
			}
		}
	}
	mutex_spin_exit(&sc->sc_lock);

	if (sched)
		softint_schedule(sc->sc_sih);

	return 1;
}

static void
esp_usj_softintr(void *priv)
{
	struct esp_usj_softc * const sc = priv;
	struct tty * const tp = sc->sc_tty;

	if (tp == NULL)
		return;

	while (sc->sc_rbuf_w != sc->sc_rbuf_r) {
		const int c = sc->sc_rbuf[sc->sc_rbuf_r & (USJ_RBUFSZ - 1)];

		sc->sc_rbuf_r++;
		if (ISSET(tp->t_state, TS_ISOPEN))
			(void)tp->t_linesw->l_rint(c, tp);
	}

	if (sc->sc_tx_done) {
		sc->sc_tx_done = false;

		ttylock(tp);
		CLR(tp->t_state, TS_BUSY);
		if (ISSET(tp->t_state, TS_FLUSH))
			CLR(tp->t_state, TS_FLUSH);
		else
			ndflush(&tp->t_outq,
			    (int)(sc->sc_tba - tp->t_outq.c_cf));
		ttyunlock(tp);

		(*tp->t_linesw->l_start)(tp);
	}
}

static int
esp_usj_console_match(int phandle)
{
	return of_compatible_match(phandle, compat_data);
}

static void
esp_usj_console_consinit(struct fdt_attach_args *faa, u_int uart_freq)
{
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0)
		panic("esp_usj: no console registers");

	esp_usj_cn_bst = faa->faa_bst;
	if (bus_space_map(esp_usj_cn_bst, addr, size, 0,
	    &esp_usj_cn_bsh) != 0)
		panic("esp_usj: failed to map console");

	cn_tab = &esp_usj_consdev;
	cn_init_magic(&esp_usj_cnm_state);
	cn_set_magic("+++++");		/* there is no BREAK on USB */

	esp_usj_console_phandle = phandle;
}

static const struct fdt_console esp_usj_console = {
	.match = esp_usj_console_match,
	.consinit = esp_usj_console_consinit,
};

FDT_CONSOLE(esp_usj, &esp_usj_console);
