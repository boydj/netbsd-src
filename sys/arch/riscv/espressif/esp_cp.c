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
 * espcp(4): the ESP32-S31's radio co-processor.
 *
 * The ESP32-S31 has two harts.  NetBSD runs on hart 0; hart 1 runs a
 * firmware built from ESP-IDF with Espressif's (closed) Wi-Fi and Bluetooth
 * stacks, which this driver loads from its flash partition and starts
 * through OpenSBI.  The two talk through rings in a block of PSRAM, with
 * FROM_CPU interrupts as doorbells; esp_cpproto.h has the details.
 * espwifi(4) and espbt(4) attach to this driver.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/condvar.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/reboot.h>
#include <sys/syslog.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#include <riscv/espressif/esp32s31_reg.h>
#include <riscv/espressif/esp32s31_var.h>
#include <riscv/espressif/esp_cpvar.h>

#define	ESPCP_NCHAN	4

#define	fence()		__asm volatile("fence rw, rw" ::: "memory")

struct espcp_client {
	espcp_rx_fn	cl_rx;
	espcp_space_fn	cl_space;
	void		*cl_arg;
	bool		cl_wait;	/* waiting for room in the ring */
};

struct espcp_ctlreq {
	TAILQ_ENTRY(espcp_ctlreq) cr_link;
	uint32_t	cr_seq;
	void		*cr_buf;
	size_t		cr_buflen;
	size_t		cr_len;
	int		cr_status;
	bool		cr_done;
};

struct espcp_region {
	bus_addr_t		r_addr;
	bus_size_t		r_size;
	bus_space_handle_t	r_bsh;
	void			*r_va;
};

struct espcp_softc {
	device_t		sc_dev;
	int			sc_phandle;
	bus_space_tag_t		sc_bst;
	bus_dma_tag_t		sc_dmat;

	struct espcp_region	sc_fw;		/* image, through XIP */
	struct espcp_region	sc_sram;
	struct espcp_region	sc_lpram;
	bus_space_handle_t	sc_dbh;		/* FROM_CPU registers */

	bus_dma_segment_t	sc_extseg;	/* PSRAM for the firmware */
	size_t			sc_extsize;
	void			*sc_extva;
	struct espcp_shm	*sc_shm;

	void			*sc_ih;
	void			*sc_si;
	callout_t		sc_tick;
	u_int			sc_ticks;

	kmutex_t		sc_txlock;	/* host -> firmware ring */
	kmutex_t		sc_lock;	/* everything else */
	kcondvar_t		sc_cv;
	TAILQ_HEAD(, espcp_ctlreq) sc_ctlq;
	uint32_t		sc_seq;
	struct espcp_client	sc_client[ESPCP_NCHAN];
	struct espcp_client	sc_events;

	bool			sc_running;
	bool			sc_dead;
	uint32_t		sc_beat;
	u_int			sc_stuck;

	kmutex_t		sc_loglock;
	char			sc_logline[160];
	size_t			sc_loglen;

	struct evcnt		sc_ev_intr;
	struct evcnt		sc_ev_rxmsg;
	struct evcnt		sc_ev_txmsg;
	struct evcnt		sc_ev_txfull;
};

static int	espcp_match(device_t, cfdata_t, void *);
static void	espcp_attach(device_t, device_t, void *);
static void	espcp_start(device_t);
static int	espcp_intr(void *);
static void	espcp_softintr(void *);
static void	espcp_tick(void *);
static void	espcp_log_drain(struct espcp_softc *);
static int	espcp_print(void *, const char *);
static void	espcp_shutdown(void *);

CFATTACH_DECL_NEW(esp_cp, sizeof(struct espcp_softc),
	espcp_match, espcp_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-coproc" },
	DEVICE_COMPAT_EOL
};

static int
espcp_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static int
espcp_map(struct espcp_softc *sc, const char *name, struct espcp_region *r)
{
	int error;

	error = fdtbus_get_reg_byname(sc->sc_phandle, name, &r->r_addr,
	    &r->r_size);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "no \"%s\" region\n", name);
		return error;
	}
	error = bus_space_map(sc->sc_bst, r->r_addr, r->r_size,
	    BUS_SPACE_MAP_LINEAR, &r->r_bsh);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "can't map %s: %d\n", name, error);
		return error;
	}
	r->r_va = bus_space_vaddr(sc->sc_bst, r->r_bsh);
	return 0;
}

static void
espcp_attach(device_t parent, device_t self, void *aux)
{
	struct espcp_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_phandle = faa->faa_phandle;
	sc->sc_bst = faa->faa_bst;
	sc->sc_dmat = faa->faa_dmat;

	aprint_naive("\n");
	aprint_normal(": radio co-processor (hart 1)\n");

	if (espcp_map(sc, "firmware", &sc->sc_fw) != 0 ||
	    espcp_map(sc, "sram", &sc->sc_sram) != 0 ||
	    espcp_map(sc, "lpram", &sc->sc_lpram) != 0)
		return;
	if (fdtbus_get_reg_byname(sc->sc_phandle, "doorbell", &addr,
	    &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_dbh) != 0) {
		aprint_error_dev(self, "can't map doorbells\n");
		return;
	}

	mutex_init(&sc->sc_txlock, MUTEX_DEFAULT, IPL_NET);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_SOFTNET);
	mutex_init(&sc->sc_loglock, MUTEX_DEFAULT, IPL_SOFTNET);
	cv_init(&sc->sc_cv, "espcpctl");
	TAILQ_INIT(&sc->sc_ctlq);
	callout_init(&sc->sc_tick, CALLOUT_MPSAFE);
	callout_setfunc(&sc->sc_tick, espcp_tick, sc);

	evcnt_attach_dynamic(&sc->sc_ev_intr, EVCNT_TYPE_INTR, NULL,
	    device_xname(self), "doorbell");
	evcnt_attach_dynamic(&sc->sc_ev_rxmsg, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "messages in");
	evcnt_attach_dynamic(&sc->sc_ev_txmsg, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "messages out");
	evcnt_attach_dynamic(&sc->sc_ev_txfull, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "ring full");

	/* Loading and starting the firmware takes a while. */
	config_interrupts(self, espcp_start);
}

/* ------------------------------------------------------------------------
 * Loading
 */

static uint32_t
espcp_crc32(uint32_t crc, const uint8_t *p, size_t len)
{
	crc = ~crc;
	while (len-- > 0) {
		crc ^= *p++;
		for (int k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
	}
	return ~crc;
}

/* Where a load address falls: the mapped region and the offset in it. */
static void *
espcp_ramva(struct espcp_softc *sc, uint32_t addr, uint32_t len)
{
	struct espcp_region * const rr[] = { &sc->sc_sram, &sc->sc_lpram };

	for (u_int i = 0; i < __arraycount(rr); i++) {
		struct espcp_region * const r = rr[i];

		if (addr >= r->r_addr && len <= r->r_size &&
		    addr - r->r_addr <= r->r_size - len)
			return (char *)r->r_va + (addr - r->r_addr);
	}
	return NULL;
}

static int
espcp_load(struct espcp_softc *sc)
{
	const struct espcp_fw_header *eh = sc->sc_fw.r_va;
	struct espcp_bootparams *bp;
	uint32_t crc = 0;
	int nsegs, error;

	if (eh->eh_magic != ESPCP_FW_MAGIC) {
		aprint_normal_dev(sc->sc_dev, "no firmware in flash at %#"
		    PRIxBUSADDR "\n", sc->sc_fw.r_addr);
		return ENOENT;
	}
	if (eh->eh_version != ESPCP_FW_VERSION ||
	    eh->eh_hdrsize < sizeof(*eh) ||
	    eh->eh_imagesize > sc->sc_fw.r_size ||
	    eh->eh_xipbase != sc->sc_fw.r_addr ||
	    eh->eh_nsegs > ESPCP_FW_MAXSEGS ||
	    eh->eh_shmsize < sizeof(struct espcp_shm) ||
	    eh->eh_extmem < eh->eh_shmsize ||
	    eh->eh_extmem > 8 * 1024 * 1024) {
		aprint_error_dev(sc->sc_dev, "bad firmware header\n");
		return EINVAL;
	}
	for (u_int i = 0; i < eh->eh_nsegs; i++) {
		const struct espcp_fw_seg *es = &eh->eh_segs[i];

		if (es->es_filesz > es->es_memsz ||
		    es->es_offset > eh->eh_imagesize ||
		    es->es_filesz > eh->eh_imagesize - es->es_offset ||
		    espcp_ramva(sc, es->es_addr, es->es_memsz) == NULL) {
			aprint_error_dev(sc->sc_dev,
			    "bad firmware segment %u (%#x, %u bytes)\n",
			    i, es->es_addr, es->es_memsz);
			return EINVAL;
		}
		crc = espcp_crc32(crc, (const uint8_t *)eh + es->es_offset,
		    es->es_filesz);
	}
	if (crc != eh->eh_crc) {
		aprint_error_dev(sc->sc_dev, "firmware checksum mismatch\n");
		return EINVAL;
	}
	bp = espcp_ramva(sc, eh->eh_params, sizeof(*bp));
	if (bp == NULL) {
		aprint_error_dev(sc->sc_dev, "bad boot parameter address\n");
		return EINVAL;
	}

	/* PSRAM for the shared memory and the firmware's heap. */
	sc->sc_extsize = round_page(eh->eh_extmem);
	error = bus_dmamem_alloc(sc->sc_dmat, sc->sc_extsize, PAGE_SIZE, 0,
	    &sc->sc_extseg, 1, &nsegs, BUS_DMA_WAITOK);
	if (error == 0) {
		error = bus_dmamem_map(sc->sc_dmat, &sc->sc_extseg, 1,
		    sc->sc_extsize, &sc->sc_extva, BUS_DMA_WAITOK);
		if (error != 0)
			bus_dmamem_free(sc->sc_dmat, &sc->sc_extseg, 1);
	}
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "can't allocate %zu KiB for the firmware: %d\n",
		    sc->sc_extsize / 1024, error);
		return error;
	}
	memset(sc->sc_extva, 0, sc->sc_extsize);
	sc->sc_shm = sc->sc_extva;

	/* Stop hart 1 if it is running (a warm boot), then load and go. */
	(void)esp32s31_sbi_vendor_call(S31_SBI_HART1_RESET, 0, 0);
	for (u_int i = 0; i < eh->eh_nsegs; i++) {
		const struct espcp_fw_seg *es = &eh->eh_segs[i];
		char *dst = espcp_ramva(sc, es->es_addr, es->es_memsz);

		memcpy(dst, (const char *)eh + es->es_offset, es->es_filesz);
		memset(dst + es->es_filesz, 0, es->es_memsz - es->es_filesz);
	}
	memset(bp, 0, sizeof(*bp));
	bp->bp_extmem = sc->sc_extseg.ds_addr;
	bp->bp_extmemsize = sc->sc_extsize;
	bp->bp_shm = sc->sc_extseg.ds_addr;
	bp->bp_shmsize = eh->eh_shmsize;
	bp->bp_flags = (boothowto & AB_DEBUG) ? ESPCP_BP_LOG_UART : 0;
	fence();
	bp->bp_magic = ESPCP_BP_MAGIC;
	fence();

	char ident[sizeof(eh->eh_ident) + 1];
	strlcpy(ident, eh->eh_ident, sizeof(ident));
	aprint_normal_dev(sc->sc_dev, "%s, %u KiB, %zu KiB PSRAM at %#"
	    PRIxPADDR "\n", ident, eh->eh_imagesize / 1024,
	    sc->sc_extsize / 1024, (paddr_t)sc->sc_extseg.ds_addr);

	const long rv = esp32s31_sbi_vendor_call(S31_SBI_HART1_BOOT,
	    eh->eh_entry, 0);
	if (rv != 0) {
		aprint_error_dev(sc->sc_dev, "OpenSBI can't start hart 1 "
		    "(%ld); is its vendor extension too old?\n", rv);
		return ENXIO;
	}
	return 0;
}

static int
espcp_print(void *aux, const char *pnp)
{
	const struct espcp_attach_args * const ea = aux;

	if (pnp != NULL)
		aprint_normal("%s at %s", ea->ea_name, pnp);
	return UNCONF;
}

static void
espcp_start(device_t self)
{
	struct espcp_softc * const sc = device_private(self);
	struct espcp_attach_args ea;
	char intrstr[128];
	int i;

	if (espcp_load(sc) != 0)
		return;

	/* The firmware sets up the rings, then says when it is ready. */
	for (i = 0; i < 1000 && sc->sc_shm->es_magic != ESPCP_SHM_MAGIC; i++)
		kpause("espcpup", false, MAX(1, mstohz(10)), NULL);
	if (sc->sc_shm->es_magic != ESPCP_SHM_MAGIC) {
		aprint_error_dev(self, "firmware didn't start\n");
		return;
	}
	for (i = 0; i < 3000; i++) {
		fence();
		espcp_log_drain(sc);
		if (sc->sc_shm->es_state != ESPCP_STATE_BOOT)
			break;
		kpause("espcpinit", false, MAX(1, mstohz(10)), NULL);
	}
	espcp_log_drain(sc);
	if (sc->sc_shm->es_state != ESPCP_STATE_RUNNING) {
		aprint_error_dev(self, "firmware %s: %.*s\n",
		    sc->sc_shm->es_state == ESPCP_STATE_PANIC ? "failed" :
		    "timed out", (int)sizeof(sc->sc_shm->es_panic),
		    sc->sc_shm->es_panic);
		sc->sc_dead = true;
		return;
	}
	if (sc->sc_shm->es_version != ESPCP_PROTO_VERSION) {
		aprint_error_dev(self, "firmware speaks protocol %u, not %u\n",
		    sc->sc_shm->es_version, ESPCP_PROTO_VERSION);
		sc->sc_dead = true;
		return;
	}

	sc->sc_si = softint_establish(SOFTINT_NET | SOFTINT_MPSAFE,
	    espcp_softintr, sc);
	if (!fdtbus_intr_str(sc->sc_phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error_dev(self, "can't decode interrupt\n");
		return;
	}
	sc->sc_ih = fdtbus_intr_establish_xname(sc->sc_phandle, 0, IPL_NET,
	    FDT_INTR_MPSAFE, espcp_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error_dev(self, "can't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	char ident[sizeof(sc->sc_shm->es_ident) + 1];
	strlcpy(ident, sc->sc_shm->es_ident, sizeof(ident));
	aprint_normal_dev(self, "%s running, interrupting on %s\n", ident,
	    intrstr);

	/* Don't leave hart 1 writing to PSRAM across a reboot. */
	shutdownhook_establish(espcp_shutdown, sc);

	sc->sc_running = true;
	sc->sc_beat = sc->sc_shm->es_heartbeat;
	callout_schedule(&sc->sc_tick, MAX(1, hz / 10));
	softint_schedule(sc->sc_si);

	if (sc->sc_shm->es_caps & ESPCP_CAP_WIFI) {
		ea.ea_name = "espwifi";
		ea.ea_addr = sc->sc_shm->es_wifi_mac;
		config_found(self, &ea, espcp_print, CFARGS_NONE);
	}
	if (sc->sc_shm->es_caps & ESPCP_CAP_BT) {
		ea.ea_name = "espbt";
		ea.ea_addr = sc->sc_shm->es_bt_addr;
		config_found(self, &ea, espcp_print, CFARGS_NONE);
	}
}

static void
espcp_shutdown(void *arg)
{
	struct espcp_softc * const sc = arg;

	sc->sc_running = false;
	(void)esp32s31_sbi_vendor_call(S31_SBI_HART1_RESET, 0, 0);
}

/* ------------------------------------------------------------------------
 * The firmware's log: complete lines go to the system log.
 */

static void
espcp_log_line(struct espcp_softc *sc)
{
	sc->sc_logline[sc->sc_loglen] = '\0';
	if (sc->sc_loglen > 0) {
		if (sc->sc_running)
			log(LOG_INFO, "%s: %s\n", device_xname(sc->sc_dev),
			    sc->sc_logline);
		else
			aprint_verbose_dev(sc->sc_dev, "%s\n", sc->sc_logline);
	}
	sc->sc_loglen = 0;
}

static void
espcp_log_drain(struct espcp_softc *sc)
{
	struct espcp_ring * const r = &sc->sc_shm->es_log;
	const char *base;
	uint32_t tail, head;

	if (r->er_size == 0)
		return;
	mutex_enter(&sc->sc_loglock);
	base = (const char *)sc->sc_shm + r->er_offset;
	tail = r->er_tail;
	head = r->er_head;
	fence();
	for (; tail != head; tail++) {
		const char c = base[tail & (r->er_size - 1)];

		if (c == '\n') {
			espcp_log_line(sc);
		} else if (sc->sc_loglen < sizeof(sc->sc_logline) - 1) {
			if (c >= ' ' || c == '\t')
				sc->sc_logline[sc->sc_loglen++] = c;
		} else {
			espcp_log_line(sc);
		}
	}
	fence();
	r->er_tail = tail;
	mutex_exit(&sc->sc_loglock);
}

/* ------------------------------------------------------------------------
 * Interrupts, the receive ring, and liveness
 */

static void
espcp_doorbell(struct espcp_softc *sc)
{
	fence();
	bus_space_write_4(sc->sc_bst, sc->sc_dbh, 4 * ESPCP_DOORBELL_FW, 1);
}

static int
espcp_intr(void *arg)
{
	struct espcp_softc * const sc = arg;

	bus_space_write_4(sc->sc_bst, sc->sc_dbh, 4 * ESPCP_DOORBELL_HOST, 0);
	sc->sc_ev_intr.ev_count++;
	softint_schedule(sc->sc_si);
	return 1;
}

static void
espcp_ctl_input(struct espcp_softc *sc, uint8_t type, const uint8_t *data,
    size_t len)
{
	struct espcp_ctl c;
	struct espcp_ctlreq *cr;

	if (len < sizeof(c))
		return;
	memcpy(&c, data, sizeof(c));
	data += sizeof(c);
	len -= sizeof(c);

	if (c.ec_seq == 0) {
		if (sc->sc_events.cl_rx != NULL)
			(*sc->sc_events.cl_rx)(sc->sc_events.cl_arg, type, data,
			    len);
		return;
	}
	mutex_enter(&sc->sc_lock);
	TAILQ_FOREACH(cr, &sc->sc_ctlq, cr_link) {
		if (cr->cr_seq == c.ec_seq)
			break;
	}
	if (cr != NULL) {
		cr->cr_len = MIN(len, cr->cr_buflen);
		if (cr->cr_len > 0)
			memcpy(cr->cr_buf, data, cr->cr_len);
		cr->cr_status = c.ec_status;
		cr->cr_done = true;
		cv_broadcast(&sc->sc_cv);
	} else if (c.ec_status != 0) {
		log(LOG_WARNING, "%s: request %u failed: error %d\n",
		    device_xname(sc->sc_dev), type, c.ec_status);
	}
	mutex_exit(&sc->sc_lock);
}

static void
espcp_dead(struct espcp_softc *sc, const char *why)
{
	static const struct espcp_wifi_status off = {
		.ws_state = ESPCP_WIFI_OFF,
	};
	struct espcp_ctlreq *cr;

	if (sc->sc_dead)
		return;
	sc->sc_dead = true;
	espcp_log_drain(sc);
	printf("%s: firmware %s: %.*s\n", device_xname(sc->sc_dev), why,
	    (int)sizeof(sc->sc_shm->es_panic), sc->sc_shm->es_panic);

	mutex_enter(&sc->sc_lock);
	TAILQ_FOREACH(cr, &sc->sc_ctlq, cr_link) {
		cr->cr_status = ENXIO;
		cr->cr_done = true;
	}
	cv_broadcast(&sc->sc_cv);
	mutex_exit(&sc->sc_lock);
	if (sc->sc_events.cl_rx != NULL)
		(*sc->sc_events.cl_rx)(sc->sc_events.cl_arg,
		    ESPCP_EVT_WIFI_LINK, &off, sizeof(off));
}

static void
espcp_softintr(void *arg)
{
	struct espcp_softc * const sc = arg;
	struct espcp_ring * const r = &sc->sc_shm->es_f2h;
	const char * const base = (const char *)sc->sc_shm + r->er_offset;
	const uint32_t size = r->er_size;
	uint32_t tail;

	espcp_log_drain(sc);
	if (sc->sc_dead)
		return;
	if (sc->sc_shm->es_state == ESPCP_STATE_PANIC) {
		espcp_dead(sc, "stopped");
		return;
	}

	tail = r->er_tail;
	while (tail != r->er_head) {
		fence();
		const uint32_t off = tail & (size - 1);
		const struct espcp_msg *m = (const void *)(base + off);

		if (m->em_chan == ESPCP_CHAN_PAD) {
			tail += size - off;
			continue;
		}
		const uint32_t step = ESPCP_MSG_ALIGN(sizeof(*m) + m->em_len);
		if (m->em_len > ESPCP_MSG_MAX || step > size - off) {
			printf("%s: bad message from firmware\n",
			    device_xname(sc->sc_dev));
			tail = r->er_head;
			break;
		}
		const void *payload = m + 1;
		sc->sc_ev_rxmsg.ev_count++;
		if (m->em_chan == ESPCP_CHAN_CTL) {
			espcp_ctl_input(sc, m->em_type, payload, m->em_len);
		} else if (m->em_chan < ESPCP_NCHAN &&
		    sc->sc_client[m->em_chan].cl_rx != NULL) {
			struct espcp_client * const cl =
			    &sc->sc_client[m->em_chan];
			(*cl->cl_rx)(cl->cl_arg, m->em_type, payload,
			    m->em_len);
		}
		tail += step;
		fence();
		r->er_tail = tail;
	}
	fence();
	r->er_tail = tail;
	if (r->er_wait) {
		r->er_wait = 0;
		espcp_doorbell(sc);
	}

	/* Room again in the other direction? */
	for (u_int i = 0; i < ESPCP_NCHAN; i++) {
		struct espcp_client * const cl = &sc->sc_client[i];

		if (cl->cl_wait) {
			cl->cl_wait = false;
			if (cl->cl_space != NULL)
				(*cl->cl_space)(cl->cl_arg);
		}
	}
}

static void
espcp_tick(void *arg)
{
	struct espcp_softc * const sc = arg;

	espcp_log_drain(sc);
	if (sc->sc_dead)
		return;
	if (sc->sc_shm->es_state == ESPCP_STATE_PANIC) {
		softint_schedule(sc->sc_si);
	} else if (++sc->sc_ticks % 10 == 0) {
		const uint32_t beat = sc->sc_shm->es_heartbeat;

		if (beat != sc->sc_beat) {
			sc->sc_beat = beat;
			sc->sc_stuck = 0;
		} else if (++sc->sc_stuck == 15) {
			espcp_dead(sc, "stopped responding");
			return;
		}
	}
	callout_schedule(&sc->sc_tick, MAX(1, hz / 10));
}

/* ------------------------------------------------------------------------
 * Client interface
 */

void
espcp_register(device_t dev, uint8_t chan, espcp_rx_fn rx,
    espcp_space_fn space, void *arg)
{
	struct espcp_softc * const sc = device_private(dev);

	KASSERT(chan < ESPCP_NCHAN);
	mutex_enter(&sc->sc_lock);
	sc->sc_client[chan].cl_rx = rx;
	sc->sc_client[chan].cl_space = space;
	sc->sc_client[chan].cl_arg = arg;
	mutex_exit(&sc->sc_lock);
}

void
espcp_register_events(device_t dev, espcp_rx_fn rx, void *arg)
{
	struct espcp_softc * const sc = device_private(dev);

	mutex_enter(&sc->sc_lock);
	sc->sc_events.cl_rx = rx;
	sc->sc_events.cl_arg = arg;
	mutex_exit(&sc->sc_lock);
}

bool
espcp_running(device_t dev)
{
	struct espcp_softc * const sc = device_private(dev);

	return sc->sc_running && !sc->sc_dead;
}

int
espcp_send(device_t dev, uint8_t chan, uint8_t type, const void *hdr,
    size_t hlen, struct mbuf *m0)
{
	struct espcp_softc * const sc = device_private(dev);
	struct espcp_ring * const r = &sc->sc_shm->es_h2f;
	const size_t mlen = m0 != NULL ? m_length(m0) : 0;
	const size_t len = hlen + mlen;
	const uint32_t need = ESPCP_MSG_ALIGN(sizeof(struct espcp_msg) + len);
	char * const base = (char *)sc->sc_shm + r->er_offset;
	const uint32_t size = r->er_size;
	uint32_t head, off, contig, total;
	struct espcp_msg *m;

	if (!espcp_running(dev))
		return ENETDOWN;
	if (len > ESPCP_MSG_MAX)
		return EMSGSIZE;

	mutex_spin_enter(&sc->sc_txlock);
	head = r->er_head;
	off = head & (size - 1);
	contig = size - off;
	total = contig < need ? contig + need : need;
	if (size - (head - r->er_tail) < total) {
		if (chan < ESPCP_NCHAN)
			sc->sc_client[chan].cl_wait = true;
		r->er_wait = 1;
		mutex_spin_exit(&sc->sc_txlock);
		sc->sc_ev_txfull.ev_count++;
		espcp_doorbell(sc);
		return ENOBUFS;
	}
	if (contig < need) {
		m = (struct espcp_msg *)(base + off);
		m->em_len = 0;
		m->em_chan = ESPCP_CHAN_PAD;
		m->em_type = 0;
		head += contig;
		off = 0;
	}
	m = (struct espcp_msg *)(base + off);
	m->em_len = len;
	m->em_chan = chan;
	m->em_type = type;
	if (hlen > 0)
		memcpy(m + 1, hdr, hlen);
	if (mlen > 0)
		m_copydata(m0, 0, mlen, (char *)(m + 1) + hlen);
	fence();
	r->er_head = head + need;
	mutex_spin_exit(&sc->sc_txlock);

	sc->sc_ev_txmsg.ev_count++;
	espcp_doorbell(sc);
	return 0;
}

static int
espcp_ctl_send(struct espcp_softc *sc, uint8_t type, uint32_t seq,
    const void *req, size_t reqlen, int timo_ticks)
{
	const struct espcp_ctl c = { .ec_seq = seq, .ec_status = 0 };
	uint8_t *buf;
	int error;

	if (reqlen > ESPCP_MSG_MAX - sizeof(c))
		return EMSGSIZE;
	buf = kmem_alloc(sizeof(c) + reqlen, KM_SLEEP);
	memcpy(buf, &c, sizeof(c));
	if (reqlen > 0)
		memcpy(buf + sizeof(c), req, reqlen);
	for (int t = 0;; t++) {
		error = espcp_send(sc->sc_dev, ESPCP_CHAN_CTL, type, buf,
		    sizeof(c) + reqlen, NULL);
		if (error != ENOBUFS || t >= timo_ticks)
			break;
		kpause("espcptx", false, 1, NULL);
	}
	kmem_free(buf, sizeof(c) + reqlen);
	return error;
}

int
espcp_ctl(device_t dev, uint8_t type, const void *req, size_t reqlen,
    void *rbuf, size_t *rlen, int timo_ms)
{
	struct espcp_softc * const sc = device_private(dev);
	struct espcp_ctlreq cr = {
		.cr_buf = rbuf,
		.cr_buflen = rlen != NULL ? *rlen : 0,
	};
	const int timo = MAX(1, mstohz(timo_ms));
	const int deadline = getticks() + timo;
	int error;

	if (!espcp_running(dev))
		return ENETDOWN;

	mutex_enter(&sc->sc_lock);
	if (++sc->sc_seq == 0)
		++sc->sc_seq;
	cr.cr_seq = sc->sc_seq;
	TAILQ_INSERT_TAIL(&sc->sc_ctlq, &cr, cr_link);
	mutex_exit(&sc->sc_lock);

	error = espcp_ctl_send(sc, type, cr.cr_seq, req, reqlen, timo);

	mutex_enter(&sc->sc_lock);
	while (error == 0 && !cr.cr_done) {
		const int left = deadline - getticks();

		if (left <= 0) {
			error = ETIMEDOUT;
			break;
		}
		error = cv_timedwait_sig(&sc->sc_cv, &sc->sc_lock, left);
		if (error == EWOULDBLOCK)
			error = 0;	/* loop to check the deadline */
	}
	TAILQ_REMOVE(&sc->sc_ctlq, &cr, cr_link);
	mutex_exit(&sc->sc_lock);

	if (error == 0) {
		error = cr.cr_status;
		if (rlen != NULL)
			*rlen = cr.cr_len;
	}
	return error;
}

/* Doesn't sleep: callers may hold locks. */
int
espcp_ctl_async(device_t dev, uint8_t type, const void *req, size_t reqlen)
{
	struct espcp_softc * const sc = device_private(dev);
	struct espcp_ctl c = { .ec_status = 0 };
	uint8_t buf[sizeof(c) + 128];

	if (!espcp_running(dev))
		return ENETDOWN;
	if (reqlen > sizeof(buf) - sizeof(c))
		return EMSGSIZE;
	mutex_enter(&sc->sc_lock);
	if (++sc->sc_seq == 0)
		++sc->sc_seq;
	c.ec_seq = sc->sc_seq;
	mutex_exit(&sc->sc_lock);
	memcpy(buf, &c, sizeof(c));
	if (reqlen > 0)
		memcpy(buf + sizeof(c), req, reqlen);
	return espcp_send(dev, ESPCP_CHAN_CTL, type, buf, sizeof(c) + reqlen,
	    NULL);
}

