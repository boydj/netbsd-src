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
 * Espressif UART (ESP32-S3 style register layout, as found on the
 * ESP32-S31).  The line settings programmed by the boot firmware
 * (115200 8N1 from the 40MHz crystal) are left alone.
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

static int	esp_uart_match(device_t, cfdata_t, void *);
static void	esp_uart_attach(device_t, device_t, void *);

static int	esp_uart_intr(void *);
static void	esp_uart_softintr(void *);

static int	esp_uart_cngetc(dev_t);
static void	esp_uart_cnputc(dev_t, int);
static void	esp_uart_cnpollc(dev_t, int);

static void	esp_uart_start(struct tty *);
static int	esp_uart_param(struct tty *, struct termios *);

extern struct cfdriver espuart_cd;

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "esp,esp32s3-uart" },
	{ .compat = "espressif,esp32s31-uart" },
	{ .compat = "esp,uart0" },		/* U-Boot's binding */
	DEVICE_COMPAT_EOL
};

#define	ESP_RBUFSZ	512		/* must be 2^n */

struct esp_uart_softc {
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
	u_char			sc_rbuf[ESP_RBUFSZ];

	/* transmit state */
	u_char			*sc_tba;	/* next byte to send */
	u_int			sc_tbc;		/* bytes left */
	volatile bool		sc_tx_done;
	bool			sc_tx_busy;
};

static int esp_uart_console_phandle = -1;
static bus_space_tag_t esp_uart_cn_bst;
static bus_space_handle_t esp_uart_cn_bsh;
static struct cnm_state esp_uart_cnm_state;

static struct consdev esp_uart_consdev = {
	.cn_getc = esp_uart_cngetc,
	.cn_putc = esp_uart_cnputc,
	.cn_pollc = esp_uart_cnpollc,
	.cn_dev = NODEV,
	.cn_pri = CN_NORMAL,
};

static dev_type_open(esp_uart_open);
static dev_type_close(esp_uart_close);
static dev_type_read(esp_uart_read);
static dev_type_write(esp_uart_write);
static dev_type_ioctl(esp_uart_ioctl);
static dev_type_tty(esp_uart_tty);
static dev_type_poll(esp_uart_poll);
static dev_type_stop(esp_uart_stop);

const struct cdevsw espuart_cdevsw = {
	.d_open = esp_uart_open,
	.d_close = esp_uart_close,
	.d_read = esp_uart_read,
	.d_write = esp_uart_write,
	.d_ioctl = esp_uart_ioctl,
	.d_stop = esp_uart_stop,
	.d_tty = esp_uart_tty,
	.d_poll = esp_uart_poll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard = nodiscard,
	.d_flag = D_TTY
};

static int esp_uart_cmajor = -1;

#define	RD4(sc, reg)		\
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR4(sc, reg, val)	\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static inline u_int
esp_uart_txcnt(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	return __SHIFTOUT(bus_space_read_4(bst, bsh, UART_STATUS),
	    UART_STATUS_TXFIFO_CNT);
}

static inline u_int
esp_uart_rxcnt(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	return __SHIFTOUT(bus_space_read_4(bst, bsh, UART_STATUS),
	    UART_STATUS_RXFIFO_CNT);
}

/*
 * Configuration registers live in the APB clock domain; latch them into
 * the UART core.
 */
static void
esp_uart_sync_regs(bus_space_tag_t bst, bus_space_handle_t bsh)
{
	u_int guard = 100000;

	bus_space_write_4(bst, bsh, UART_REG_UPDATE, UART_REG_UPDATE_BUSY);
	while ((bus_space_read_4(bst, bsh, UART_REG_UPDATE) &
	    UART_REG_UPDATE_BUSY) != 0 && --guard != 0)
		continue;
}

CFATTACH_DECL_NEW(esp_uart, sizeof(struct esp_uart_softc),
	esp_uart_match, esp_uart_attach, NULL, NULL);

static int
esp_uart_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_uart_attach(device_t parent, device_t self, void *aux)
{
	struct esp_uart_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	struct tty *tp;
	int major, minor, error;

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

	/* Quiesce the interrupt sources until somebody opens us. */
	WR4(sc, UART_INT_ENA, 0);
	WR4(sc, UART_INT_CLR, 0xffffffff);

	/*
	 * Interrupt on every received byte, and when the transmit FIFO
	 * drains below a few bytes.
	 */
	WR4(sc, UART_CONF1,
	    __SHIFTIN(1, UART_CONF1_RXFIFO_FULL_THRHD) |
	    __SHIFTIN(8, UART_CONF1_TXFIFO_EMPTY_THRHD));
	esp_uart_sync_regs(sc->sc_bst, sc->sc_bsh);

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr)))
		strlcpy(intrstr, "?", sizeof(intrstr));

	sc->sc_sih = softint_establish(SOFTINT_SERIAL, esp_uart_softintr, sc);
	if (sc->sc_sih == NULL) {
		aprint_error(": failed to establish softint\n");
		return;
	}

	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_SERIAL,
	    FDT_INTR_MPSAFE, esp_uart_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error(": failed to establish interrupt on %s\n",
		    intrstr);
		return;
	}

	/* The major is static in kernels that list it in majors.riscv. */
	if (esp_uart_cmajor == -1 &&
	    cdevsw_lookup_major(&espuart_cdevsw) == NODEVMAJOR) {
		int bmajor = -1, cmajor = -1;

		error = devsw_attach("espuart", NULL, &bmajor,
		    &espuart_cdevsw, &cmajor);
		if (error) {
			aprint_error(": couldn't allocate major number\n");
			return;
		}
		esp_uart_cmajor = cmajor;
	}

	major = cdevsw_lookup_major(&espuart_cdevsw);
	minor = device_unit(self);

	tp = sc->sc_tty = tty_alloc();
	tp->t_oproc = esp_uart_start;
	tp->t_param = esp_uart_param;
	tp->t_dev = makedev(major, minor);
	tp->t_sc = sc;
	tty_attach(tp);

	aprint_naive("\n");
	aprint_normal(": ESP32 UART");
	if (esp_uart_console_phandle == phandle) {
		cn_tab->cn_dev = tp->t_dev;
		aprint_normal(", console");
	}
	aprint_normal("\n");
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	/* Always listen, so the console gets break/ddb and early input. */
	WR4(sc, UART_INT_ENA, UART_INT_RXFIFO_FULL | UART_INT_BRK_DET);
}

static int
esp_uart_cngetc(dev_t dev)
{
	const bus_space_tag_t bst = esp_uart_cn_bst;
	const bus_space_handle_t bsh = esp_uart_cn_bsh;
	int s, c;

	s = splserial();
	if (esp_uart_rxcnt(bst, bsh) == 0) {
		splx(s);
		return -1;
	}
	c = bus_space_read_4(bst, bsh, UART_FIFO) & 0xff;
	if (!db_active) {
		int cn_trapped __unused = 0;
		cn_check_magic(dev, c, esp_uart_cnm_state);
	}
	splx(s);

	return c;
}

static void
esp_uart_cnputc(dev_t dev, int c)
{
	const bus_space_tag_t bst = esp_uart_cn_bst;
	const bus_space_handle_t bsh = esp_uart_cn_bsh;
	int s;

	s = splserial();
	while (esp_uart_txcnt(bst, bsh) >= UART_TX_FIFO_MAX)
		continue;
	bus_space_write_4(bst, bsh, UART_FIFO, c & 0xff);
	splx(s);
}

static void
esp_uart_cnpollc(dev_t dev, int on)
{
}

static int
esp_uart_open(dev_t dev, int flag, int mode, lwp_t *l)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));
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
esp_uart_close(dev_t dev, int flag, int mode, lwp_t *l)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	tp->t_linesw->l_close(tp, flag);
	ttyclose(tp);

	return 0;
}

static int
esp_uart_read(dev_t dev, struct uio *uio, int flag)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	return tp->t_linesw->l_read(tp, uio, flag);
}

static int
esp_uart_write(dev_t dev, struct uio *uio, int flag)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	return tp->t_linesw->l_write(tp, uio, flag);
}

static int
esp_uart_poll(dev_t dev, int events, lwp_t *l)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;

	return tp->t_linesw->l_poll(tp, events, l);
}

static int
esp_uart_ioctl(dev_t dev, u_long cmd, void *data, int flag, lwp_t *l)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));
	struct tty * const tp = sc->sc_tty;
	int error;

	error = tp->t_linesw->l_ioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	return ttioctl(tp, cmd, data, flag, l);
}

static struct tty *
esp_uart_tty(dev_t dev)
{
	struct esp_uart_softc * const sc =
	    device_lookup_private(&espuart_cd, minor(dev));

	return sc->sc_tty;
}

static void
esp_uart_stop(struct tty *tp, int flag)
{
	struct esp_uart_softc * const sc = tp->t_sc;

	mutex_spin_enter(&sc->sc_lock);
	if (sc->sc_tx_busy) {
		/* Stop sending what's left of the current chunk. */
		sc->sc_tbc = 0;
		if (!ISSET(tp->t_state, TS_TTSTOP))
			SET(tp->t_state, TS_FLUSH);
	}
	mutex_spin_exit(&sc->sc_lock);
}

/*
 * Fill the transmit FIFO from the current chunk.  Called with sc_lock.
 */
static void
esp_uart_txfill(struct esp_uart_softc *sc)
{
	u_int room = UART_TX_FIFO_MAX -
	    esp_uart_txcnt(sc->sc_bst, sc->sc_bsh);

	while (room-- > 0 && sc->sc_tbc > 0) {
		WR4(sc, UART_FIFO, *sc->sc_tba);
		sc->sc_tba++;
		sc->sc_tbc--;
	}
}

static void
esp_uart_start(struct tty *tp)
{
	struct esp_uart_softc * const sc = tp->t_sc;
	int s;

	s = spltty();
	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP))
		goto out;
	if (!ttypull(tp))
		goto out;

	/* Send the first contiguous run in the output queue. */
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
	esp_uart_txfill(sc);
	/* Let the interrupt handler feed the rest / report completion. */
	WR4(sc, UART_INT_CLR, UART_INT_TXFIFO_EMPTY);
	WR4(sc, UART_INT_ENA, RD4(sc, UART_INT_ENA) | UART_INT_TXFIFO_EMPTY);
	mutex_spin_exit(&sc->sc_lock);
out:
	splx(s);
}

static int
esp_uart_param(struct tty *tp, struct termios *t)
{
	/* Line settings are fixed by the firmware. */
	tp->t_ispeed = t->c_ispeed;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	return 0;
}

static int
esp_uart_intr(void *priv)
{
	struct esp_uart_softc * const sc = priv;
	struct tty * const tp = sc->sc_tty;
	uint32_t st;
	bool sched = false;

	mutex_spin_enter(&sc->sc_lock);
	st = RD4(sc, UART_INT_ST);
	if (st == 0) {
		mutex_spin_exit(&sc->sc_lock);
		return 0;
	}

	/* Drain the receive FIFO. */
	while (esp_uart_rxcnt(sc->sc_bst, sc->sc_bsh) > 0) {
		int cn_trapped = 0;
		const int c = RD4(sc, UART_FIFO) & 0xff;

		if (tp != NULL) {
			cn_check_magic(tp->t_dev, c, esp_uart_cnm_state);
			if (cn_trapped)
				continue;
		}
		if (sc->sc_rbuf_w - sc->sc_rbuf_r >= ESP_RBUFSZ - 1)
			continue;	/* overflow: drop */
		sc->sc_rbuf[sc->sc_rbuf_w & (ESP_RBUFSZ - 1)] = c;
		sc->sc_rbuf_w++;
		sched = true;
	}
	if (st & UART_INT_BRK_DET) {
		int cn_trapped = 0;
		if (tp != NULL)
			cn_check_magic(tp->t_dev, CNC_BREAK,
			    esp_uart_cnm_state);
		(void)cn_trapped;
	}

	if (st & UART_INT_TXFIFO_EMPTY) {
		if (sc->sc_tbc > 0) {
			esp_uart_txfill(sc);
		} else {
			WR4(sc, UART_INT_ENA,
			    RD4(sc, UART_INT_ENA) & ~UART_INT_TXFIFO_EMPTY);
			if (sc->sc_tx_busy) {
				sc->sc_tx_busy = false;
				sc->sc_tx_done = true;
				sched = true;
			}
		}
	}

	WR4(sc, UART_INT_CLR, st);
	mutex_spin_exit(&sc->sc_lock);

	if (sched)
		softint_schedule(sc->sc_sih);

	return 1;
}

static void
esp_uart_softintr(void *priv)
{
	struct esp_uart_softc * const sc = priv;
	struct tty * const tp = sc->sc_tty;

	if (tp == NULL)
		return;

	while (sc->sc_rbuf_w != sc->sc_rbuf_r) {
		const int c = sc->sc_rbuf[sc->sc_rbuf_r & (ESP_RBUFSZ - 1)];

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
esp_uart_console_match(int phandle)
{
	return of_compatible_match(phandle, compat_data);
}

static void
esp_uart_console_consinit(struct fdt_attach_args *faa, u_int uart_freq)
{
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0)
		panic("esp_uart: no console registers");

	esp_uart_cn_bst = faa->faa_bst;
	if (bus_space_map(esp_uart_cn_bst, addr, size, 0,
	    &esp_uart_cn_bsh) != 0)
		panic("esp_uart: failed to map console");

	cn_tab = &esp_uart_consdev;
	cn_init_magic(&esp_uart_cnm_state);
	cn_set_magic("\047\001");	/* default magic is BREAK */

	esp_uart_console_phandle = phandle;
}

static const struct fdt_console esp_uart_console = {
	.match = esp_uart_console_match,
	.consinit = esp_uart_console_consinit,
};

FDT_CONSOLE(esp_uart, &esp_uart_console);
