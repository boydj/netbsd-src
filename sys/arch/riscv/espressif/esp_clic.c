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
 * Espressif ESP32-S31 interrupt controllers.
 *
 * Peripherals drive "sources" into the interrupt matrix (INTMTX), which
 * routes each source to one of the core's CLIC inputs.  The CLIC runs in
 * CLIC mode: every input traps with scause.exccode equal to its id; there
 * is no claim/complete register and no sie/sip.  OpenSBI delegates ids
 * 16-47 to S-mode (mode = supervisor, maximum level, masked).
 *
 * espclic   - the S-mode CLIC bank; owns trap dispatch.
 * espintmtx - the matrix; the interrupt controller devices refer to.
 *
 * IPL handling: all inputs share one CLIC level, so masking by priority is
 * not possible in hardware.  IPL_VM and above are implemented by
 * sstatus.SIE (see spl.S, RISCV_CLIC), so when an interrupt is taken the
 * current IPL is always below IPL_VM.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>
#include <sys/evcnt.h>
#include <sys/intr.h>
#include <sys/kmem.h>
#include <sys/queue.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#include <machine/frame.h>
#include <machine/machdep.h>
#include <machine/sysreg.h>

#include <riscv/espressif/esp32s31_reg.h>
#include <riscv/espressif/esp32s31_var.h>

/*
 * Generic peripherals use CLIC ids 16..31 (the same pool Espressif's
 * Linux uses); 32..47 are left for local sources.
 */
#define	CLIC_ALLOC_FIRST	16
#define	CLIC_ALLOC_LAST		31

struct esp_clic_ih {
	TAILQ_ENTRY(esp_clic_ih) ih_next;
	struct esp_clic_slot	*ih_slot;
	int			(*ih_func)(void *);
	void			*ih_arg;
};

struct esp_clic_slot {
	TAILQ_HEAD(, esp_clic_ih) cs_handlers;
	int			cs_ipl;
	bool			cs_mpsafe;
	u_int			cs_id;
	struct evcnt		cs_ev;
	char			cs_name[32];
};

struct esp_clic_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	struct esp_clic_slot	sc_slots[CLIC_NIDS];
	struct evcnt		sc_ev_stray;
};

static struct esp_clic_softc *esp_clic_sc;

#define	CLIC_READ1(sc, reg)	\
	bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	CLIC_WRITE1(sc, reg, val)	\
	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static void
esp_clic_intr(struct trapframe *tf, register_t epc, register_t status,
    register_t cause)
{
	struct esp_clic_softc * const sc = esp_clic_sc;
	struct cpu_info * const ci = curcpu();
	const u_int id = CAUSE_CODE(cause) & 0xfff;
	const int opl = ci->ci_cpl;

	KASSERT(CAUSE_INTERRUPT_P(cause));

	ci->ci_intr_depth++;
	ci->ci_data.cpu_nintr++;

	struct esp_clic_slot * const cs =
	    (id < CLIC_NIDS) ? &sc->sc_slots[id] : NULL;

	if (cs == NULL || TAILQ_EMPTY(&cs->cs_handlers)) {
		/* Nobody wants it: mask it so we don't loop forever. */
		if (id < CLIC_NIDS)
			CLIC_WRITE1(sc, CLIC_INTIE(id), 0);
		sc->sc_ev_stray.ev_count++;
	} else {
		struct clockframe cf = {
			.cf_epc = epc,
			.cf_status = status,
			.cf_intr_depth = ci->ci_intr_depth,
		};
		struct esp_clic_ih *ih;

		(void)splraise(cs->cs_ipl);
		cs->cs_ev.ev_count++;

		if (!cs->cs_mpsafe)
			KERNEL_LOCK(1, NULL);
		TAILQ_FOREACH(ih, &cs->cs_handlers, ih_next) {
			if ((*ih->ih_func)(ih->ih_arg ? ih->ih_arg : &cf))
				break;
		}
		if (!cs->cs_mpsafe)
			KERNEL_UNLOCK_ONE(NULL);
	}

	ci->ci_intr_depth--;
	splx(opl);
}

bool
esp_clic_attached(void)
{
	return esp_clic_sc != NULL;
}

void *
esp_clic_establish(u_int id, int ipl, int flags, int (*func)(void *),
    void *arg, const char *xname)
{
	struct esp_clic_softc * const sc = esp_clic_sc;

	if (sc == NULL || id < CLIC_SMODE_FIRST || id > CLIC_SMODE_LAST)
		return NULL;
	if (ipl < IPL_VM) {
		/* sstatus.SIE can't mask below IPL_VM; see spl.S */
		ipl = IPL_VM;
	}

	struct esp_clic_slot * const cs = &sc->sc_slots[id];
	const bool mpsafe = (flags & FDT_INTR_MPSAFE) != 0;
	struct esp_clic_ih * const ih = kmem_zalloc(sizeof(*ih), KM_SLEEP);

	ih->ih_slot = cs;
	ih->ih_func = func;
	ih->ih_arg = arg;

	const int s = splhigh();
	if (TAILQ_EMPTY(&cs->cs_handlers)) {
		cs->cs_ipl = ipl;
		cs->cs_mpsafe = mpsafe;
		snprintf(cs->cs_name, sizeof(cs->cs_name), "clic %u%s%s", id,
		    xname ? " " : "", xname ? xname : "");
	} else if (cs->cs_ipl != ipl || cs->cs_mpsafe != mpsafe) {
		splx(s);
		kmem_free(ih, sizeof(*ih));
		device_printf(sc->sc_dev,
		    "can't share id %u with different ipl/mpsafe\n", id);
		return NULL;
	}
	TAILQ_INSERT_TAIL(&cs->cs_handlers, ih, ih_next);

	/* Level triggered, maximum level, clear stale pending, enable. */
	uint8_t attr = CLIC_READ1(sc, CLIC_INTATTR(id));
	attr &= ~CLIC_INTATTR_TRIG;
	attr |= __SHIFTIN(CLIC_INTATTR_TRIG_LEVEL, CLIC_INTATTR_TRIG);
	attr &= ~CLIC_INTATTR_SHV;
	CLIC_WRITE1(sc, CLIC_INTATTR(id), attr);
	CLIC_WRITE1(sc, CLIC_INTCTL(id), CLIC_INTCTL_MAX);
	CLIC_WRITE1(sc, CLIC_INTIP(id), 0);
	CLIC_WRITE1(sc, CLIC_INTIE(id), 1);
	splx(s);

	return ih;
}

void
esp_clic_disestablish(void *cookie)
{
	struct esp_clic_softc * const sc = esp_clic_sc;
	struct esp_clic_ih * const ih = cookie;
	struct esp_clic_slot * const cs = ih->ih_slot;

	const int s = splhigh();
	TAILQ_REMOVE(&cs->cs_handlers, ih, ih_next);
	if (TAILQ_EMPTY(&cs->cs_handlers))
		CLIC_WRITE1(sc, CLIC_INTIE(cs->cs_id), 0);
	splx(s);

	kmem_free(ih, sizeof(*ih));
}

static const struct device_compatible_entry clic_compat_data[] = {
	{ .compat = "espressif,esp32s31-clic" },
	DEVICE_COMPAT_EOL
};

static int
esp_clic_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, clic_compat_data);
}

static void
esp_clic_attach(device_t parent, device_t self, void *aux)
{
	struct esp_clic_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(faa->faa_phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	for (u_int id = 0; id < CLIC_NIDS; id++) {
		struct esp_clic_slot * const cs = &sc->sc_slots[id];

		TAILQ_INIT(&cs->cs_handlers);
		cs->cs_id = id;
	}

	/* Mask everything we own until a driver asks for it. */
	for (u_int id = CLIC_SMODE_FIRST; id <= CLIC_SMODE_LAST; id++)
		CLIC_WRITE1(sc, CLIC_INTIE(id), 0);

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 CLIC (S-mode ids %u-%u)\n",
	    CLIC_SMODE_FIRST, CLIC_SMODE_LAST);

	evcnt_attach_dynamic(&sc->sc_ev_stray, EVCNT_TYPE_INTR, NULL,
	    device_xname(self), "stray");

	esp_clic_sc = sc;
	riscv_intr_set_handler(esp_clic_intr);
}

CFATTACH_DECL_NEW(esp_clic, sizeof(struct esp_clic_softc),
	esp_clic_match, esp_clic_attach, NULL, NULL);


/*
 * Interrupt matrix.
 */

struct esp_intmtx_irq {
	void			*ii_ih;
	u_int			ii_src;
	u_int			ii_id;
};

struct esp_intmtx_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	uint8_t			sc_src_id[INTMTX_NSRC];	/* 0 = unrouted */
	uint8_t			sc_id_src[CLIC_NIDS];
	bool			sc_id_used[CLIC_NIDS];
};

static u_int
esp_intmtx_find_id(struct esp_intmtx_softc *sc, u_int src)
{
	if (sc->sc_src_id[src] != 0)
		return sc->sc_src_id[src];	/* share the existing route */

	for (u_int id = CLIC_ALLOC_FIRST; id <= CLIC_ALLOC_LAST; id++) {
		if (!sc->sc_id_used[id])
			return id;
	}
	return 0;
}

static void *
esp_intmtx_fdt_establish(device_t dev, u_int *specifier, int ipl, int flags,
    int (*func)(void *), void *arg, const char *xname)
{
	struct esp_intmtx_softc * const sc = device_private(dev);
	const u_int src = be32toh(specifier[0]);

	if (src >= INTMTX_NSRC) {
		device_printf(dev, "source %u out of range\n", src);
		return NULL;
	}

	const u_int id = esp_intmtx_find_id(sc, src);
	if (id == 0) {
		device_printf(dev, "no free CLIC id for source %u\n", src);
		return NULL;
	}

	void *ih = esp_clic_establish(id, ipl, flags, func, arg, xname);
	if (ih == NULL)
		return NULL;

	struct esp_intmtx_irq * const ii = kmem_zalloc(sizeof(*ii), KM_SLEEP);
	ii->ii_ih = ih;
	ii->ii_src = src;
	ii->ii_id = id;

	sc->sc_src_id[src] = id;
	sc->sc_id_src[id] = src;
	sc->sc_id_used[id] = true;
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, INTMTX_SRC(src),
	    __SHIFTIN(id, INTMTX_SRC_SLOT));

	return ii;
}

static void
esp_intmtx_fdt_disestablish(device_t dev, void *cookie)
{
	struct esp_intmtx_irq * const ii = cookie;

	esp_clic_disestablish(ii->ii_ih);
	kmem_free(ii, sizeof(*ii));
}

static bool
esp_intmtx_fdt_intrstr(device_t dev, u_int *specifier, char *buf,
    size_t buflen)
{
	const u_int src = be32toh(specifier[0]);

	snprintf(buf, buflen, "intmtx source %u", src);
	return true;
}

static struct fdtbus_interrupt_controller_func esp_intmtx_fdt_funcs = {
	.establish = esp_intmtx_fdt_establish,
	.disestablish = esp_intmtx_fdt_disestablish,
	.intrstr = esp_intmtx_fdt_intrstr,
};

static const struct device_compatible_entry intmtx_compat_data[] = {
	{ .compat = "espressif,esp32s31-intmtx" },
	DEVICE_COMPAT_EOL
};

static int
esp_intmtx_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, intmtx_compat_data);
}

static void
esp_intmtx_attach(device_t parent, device_t self, void *aux)
{
	struct esp_intmtx_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;

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

	/*
	 * Unroute everything the firmware left pointing at the CLIC ids we
	 * own.  Routes to M-mode ids (e.g. an OpenSBI timer) are left alone.
	 */
	for (u_int src = 0; src < INTMTX_NSRC; src++) {
		const u_int id = __SHIFTOUT(bus_space_read_4(sc->sc_bst,
		    sc->sc_bsh, INTMTX_SRC(src)), INTMTX_SRC_SLOT);

		if (id >= CLIC_SMODE_FIRST && id <= CLIC_SMODE_LAST)
			bus_space_write_4(sc->sc_bst, sc->sc_bsh,
			    INTMTX_SRC(src), 0);
	}

	aprint_naive("\n");
	aprint_normal(": ESP32-S31 interrupt matrix\n");

	if (!esp_clic_attached())
		aprint_error_dev(self, "CLIC not attached!\n");

	int error = fdtbus_register_interrupt_controller(self, phandle,
	    &esp_intmtx_fdt_funcs);
	if (error)
		aprint_error_dev(self, "couldn't register with fdtbus: %d\n",
		    error);
}

CFATTACH_DECL_NEW(esp_intmtx, sizeof(struct esp_intmtx_softc),
	esp_intmtx_match, esp_intmtx_attach, NULL, NULL);
