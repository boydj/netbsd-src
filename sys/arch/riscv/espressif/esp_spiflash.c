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
 * ESP32-S31 SPI NOR flash: a partition of the boot flash, as a disk.
 *
 * The flash sits behind the MSPI controller, which has two ports to it.
 * SPI0 is the cache's: the XIP window that the root file system
 * (espxiproot) and the radio firmware on hart 1 (espcp) are read through.
 * SPI1 runs commands issued by software; this driver uses it for
 * everything, reads included.  The controller arbitrates between the two.
 *
 * Erasing a sector or programming a page keeps the flash chip busy for up
 * to hundreds of milliseconds, during which it can't be read.  Rather than
 * stopping everything that might touch the XIP window (on both harts),
 * the controller's auto-suspend is used: a cache miss during an erase or
 * program has the controller suspend it (75h), do the read, and resume it
 * (7Ah) once SPI0 goes quiet; the controller also polls the status itself
 * (auto wait-idle), so an erase or program command completes only when
 * the chip is done.  This is ESP-IDF's CONFIG_SPI_FLASH_AUTO_SUSPEND
 * configuration.  The flash chip has to support suspend and resume, as
 * the GigaDevice, Winbond, XMC etc. parts on Espressif modules do.
 *
 * Nothing keeps the XIP window coherent with what's written here, so the
 * partition mustn't be read through it.
 *
 * Writes are read-modify-write per 4 KB sector, done by a kernel thread:
 * the erase is skipped when bits only go from 1 to 0, only the 64-byte
 * pieces that change are programmed, and the sector is read back and
 * checked afterwards.
 *
 * Bindings:
 *	compatible = "espressif,esp32s31-spi-flash";
 *	reg = <SPI1 (FLASH_SPI1) base>, <SPI0 (FLASH_SPI0) base>;
 *	reg-names = "spi1", "spi0";
 *	espressif,partition = <offset size>;	(in flash, 4 KB aligned)
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/buf.h>
#include <sys/bufq.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/disk.h>
#include <sys/disklabel.h>
#include <sys/fcntl.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/kthread.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/systm.h>

#include <dev/dkvar.h>
#include <dev/fdt/fdtvar.h>

#include "ioconf.h"

/* SPI1 (FLASH_SPI1, SPI1_MEM_C_*) */
#define	SPI_CMD				0x00
#define	 CMD_MST_ST			__BITS(3,0)
#define	 CMD_USR			__BIT(18)
#define	 CMD_FLASH_SE			__BIT(24)
#define	 CMD_FLASH_PP			__BIT(25)
#define	 CMD_FLASH_WREN			__BIT(30)
#define	SPI_ADDR			0x04
#define	SPI_CTRL			0x08
#define	 CTRL_FASTRD_MODE		__BIT(13)
#define	 CTRL_WP_REG			__BIT(21)
#define	SPI_CTRL1			0x0c
#define	 CTRL1_CS_HOLD_DLY_RES		__BITS(11,2)	/* tSUS, 128 clocks */
#define	 CTRL1_CS_HOLD_DLY_PER		__BITS(21,12)	/* tRS, 128 clocks */
#define	SPI_USER			0x18
#define	 USER_USR_MISO			__BIT(28)
#define	 USER_USR_DUMMY			__BIT(29)
#define	 USER_USR_ADDR			__BIT(30)
#define	 USER_USR_COMMAND		__BIT(31)
#define	SPI_USER1			0x1c
#define	 USER1_DUMMY_CYCLELEN		__BITS(5,0)
#define	 USER1_ADDR_BITLEN		__BITS(31,26)
#define	SPI_USER2			0x20
#define	 USER2_COMMAND_VALUE		__BITS(15,0)
#define	 USER2_COMMAND_BITLEN		__BITS(31,28)
#define	SPI_MOSI_DLEN			0x24
#define	SPI_MISO_DLEN			0x28
#define	SPI_CACHE_FCTRL			0x3c
#define	 CACHE_FCTRL_ADDR_4BYTE		__BIT(1)
#define	SPI_W(n)			(0x58 + 4 * (n))
#define	SPI_NW				16		/* 64-byte buffer */
#define	SPI_FLASH_WAITI_CTRL		0x98
#define	 WAITI_EN			__BIT(0)
#define	 WAITI_DUMMY			__BIT(1)
#define	 WAITI_ADDR_EN			__BIT(2)
#define	 WAITI_CMD_2B			__BIT(9)
#define	 WAITI_CMD			__BITS(31,16)
#define	SPI_FLASH_SUS_CTRL		0x9c
#define	 SUS_CTRL_PER_WAIT_EN		__BIT(2)
#define	 SUS_CTRL_PES_WAIT_EN		__BIT(3)
#define	 SUS_CTRL_PES_PER_EN		__BIT(4)	/* auto resume */
#define	 SUS_CTRL_PES_EN		__BIT(5)	/* auto suspend */
#define	 SUS_CTRL_PESR_END_MSK		__BITS(21,6)
#define	 SUS_CTRL_RD_SUS_2B		__BIT(22)
#define	 SUS_CTRL_PER_END_EN		__BIT(23)
#define	 SUS_CTRL_PES_END_EN		__BIT(24)
#define	 SUS_CTRL_SUS_TIMEOUT_CNT	__BITS(31,25)
#define	SPI_FLASH_SUS_CMD		0xa0
#define	 SUS_CMD_PES_COMMAND		__BITS(15,0)
#define	 SUS_CMD_WAIT_PESR_COMMAND	__BITS(31,16)
#define	SPI_SUS_STATUS			0xa4
#define	 SUS_STATUS_FLASH_SUS		__BIT(0)
#define	 SUS_STATUS_WAIT_PESR_CMD_2B	__BIT(1)
#define	 SUS_STATUS_PER_DLY_128		__BIT(5)
#define	 SUS_STATUS_PES_DLY_128		__BIT(6)
#define	 SUS_STATUS_SPI0_LOCK_EN	__BIT(7)
#define	 SUS_STATUS_PESR_CMD_2B		__BIT(15)
#define	 SUS_STATUS_PER_COMMAND		__BITS(31,16)

/* SPI0 (FLASH_SPI0, SPI_MEM_C_*) */
#define	SPI0_CTRL2			0x10
#define	 CTRL2_CS_HOLD_DELAY		__BITS(30,25)
#define	SPI0_FSM			0x54
#define	 FSM_LOCK_DELAY_TIME		__BITS(18,7)

/* Flash commands */
#define	FLASH_READ_FAST			0x0b
#define	FLASH_RDSR			0x05
#define	 FLASH_SR_WIP			0x01
#define	FLASH_RDSR2			0x35
#define	 FLASH_SR2_SUS			0x80
#define	FLASH_RDID			0x9f
#define	FLASH_SUSPEND			0x75
#define	FLASH_RESUME			0x7a

#define	ESPNOR_SECTOR			4096
#define	ESPNOR_CHUNK			(SPI_NW * 4)	/* per SPI1 transfer */

/*
 * Suspend latency (tSUS) and minimum resume-to-suspend time (tRS): 50us,
 * ESP-IDF's default, in units of 128 SPI clocks at up to 120 MHz.  tSHSL2
 * (CS high between suspend and the next command) 30ns in MSPI core clocks
 * at up to 480 MHz.  SPI0 keeps the bus after a transfer for a while.
 */
#define	ESPNOR_TSUS			47
#define	ESPNOR_TRS			47
#define	ESPNOR_TSHSL2			15
#define	ESPNOR_SPI0_LOCK		0x1f

/* Timeouts, us */
#define	ESPNOR_CMD_TIMO		10000
#define	ESPNOR_PP_TIMO		100000
#define	ESPNOR_SE_TIMO		4000000

struct espnor_softc {
	struct dk_softc		sc_dksc;	/* must be first */
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;		/* SPI1 */
	bus_space_handle_t	sc_spi0_bsh;	/* SPI0 */

	uint32_t		sc_offset;	/* partition, in flash */
	uint32_t		sc_size;

	uint8_t			*sc_old;	/* a sector as it is */
	uint8_t			*sc_new;	/* ... and as it is to be */

	kmutex_t		sc_lock;
	kcondvar_t		sc_cv;
	struct buf		*sc_bp;		/* being done by the thread */
	lwp_t			*sc_thread;
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))
#define	RD0(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_spi0_bsh, (r))
#define	WR0(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_spi0_bsh, (r), (v))

static int	espnor_match(device_t, cfdata_t, void *);
static void	espnor_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_spiflash, sizeof(struct espnor_softc),
	espnor_match, espnor_attach, NULL, NULL);

static dev_type_open(espnor_open);
static dev_type_close(espnor_close);
static dev_type_read(espnor_read);
static dev_type_write(espnor_write);
static dev_type_ioctl(espnor_ioctl);
static dev_type_strategy(espnor_strategy);
static dev_type_size(espnor_size);

const struct bdevsw espnor_bdevsw = {
	.d_open = espnor_open,
	.d_close = espnor_close,
	.d_strategy = espnor_strategy,
	.d_ioctl = espnor_ioctl,
	.d_dump = nodump,
	.d_psize = espnor_size,
	.d_discard = nodiscard,
	.d_flag = D_DISK | D_MPSAFE
};

const struct cdevsw espnor_cdevsw = {
	.d_open = espnor_open,
	.d_close = espnor_close,
	.d_read = espnor_read,
	.d_write = espnor_write,
	.d_ioctl = espnor_ioctl,
	.d_stop = nostop,
	.d_tty = notty,
	.d_poll = nopoll,
	.d_mmap = nommap,
	.d_kqfilter = nokqfilter,
	.d_discard = nodiscard,
	.d_flag = D_DISK | D_MPSAFE
};

static int	espnor_diskstart(device_t, struct buf *);
static void	espnor_label(device_t, struct disklabel *);

static const struct dkdriver espnor_dkdriver = {
	.d_open = espnor_open,
	.d_close = espnor_close,
	.d_strategy = espnor_strategy,
	.d_minphys = minphys,
	.d_diskstart = espnor_diskstart,
	.d_label = espnor_label,
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-spi-flash" },
	DEVICE_COMPAT_EOL
};

/*
 * SPI1 commands
 */

/*
 * Wait for the command in progress to finish.  Erase and program take a
 * while; after spin_us, sleep a tick at a time.
 */
static int
espnor_wait(struct espnor_softc *sc, u_int timo_us, u_int spin_us)
{
	u_int t = 0;

	while (RD(sc, SPI_CMD) != 0) {
		if (t >= timo_us)
			return ETIMEDOUT;
		if (t >= spin_us) {
			kpause("espnor", false, 1, NULL);
			t += tick;
		} else {
			DELAY(2);
			t += 2;
			if (t > 50)
				preempt_point();
		}
	}
	return 0;
}

/* A command with no data out and up to 64 bytes in, single-line. */
static int
espnor_usr(struct espnor_softc *sc, uint8_t cmd, u_int addrbits,
    uint32_t addr, u_int dummy, size_t inlen)
{
	uint32_t user = USER_USR_COMMAND, user1 = 0;

	KASSERT(inlen <= ESPNOR_CHUNK);

	if (addrbits) {
		user |= USER_USR_ADDR;
		user1 |= __SHIFTIN(addrbits - 1, USER1_ADDR_BITLEN);
	}
	if (dummy) {
		user |= USER_USR_DUMMY;
		user1 |= __SHIFTIN(dummy - 1, USER1_DUMMY_CYCLELEN);
	}
	if (inlen)
		user |= USER_USR_MISO;

	WR(sc, SPI_CTRL, (RD(sc, SPI_CTRL) & CTRL_WP_REG) | CTRL_FASTRD_MODE);
	WR(sc, SPI_CACHE_FCTRL,
	    RD(sc, SPI_CACHE_FCTRL) & ~CACHE_FCTRL_ADDR_4BYTE);
	WR(sc, SPI_USER, user);
	WR(sc, SPI_USER1, user1);
	WR(sc, SPI_USER2, __SHIFTIN(7, USER2_COMMAND_BITLEN) |
	    __SHIFTIN(cmd, USER2_COMMAND_VALUE));
	WR(sc, SPI_MOSI_DLEN, 0);
	WR(sc, SPI_MISO_DLEN, inlen ? inlen * 8 - 1 : 0);
	WR(sc, SPI_ADDR, addr);
	WR(sc, SPI_CMD, CMD_USR);

	return espnor_wait(sc, ESPNOR_CMD_TIMO, ESPNOR_CMD_TIMO);
}

static void
espnor_getbuf(struct espnor_softc *sc, uint8_t *buf, size_t len)
{
	for (u_int i = 0; len > 0; i++) {
		const uint32_t w = htole32(RD(sc, SPI_W(i)));
		const size_t n = MIN(len, sizeof(w));

		memcpy(buf, &w, n);
		buf += n;
		len -= n;
	}
}

static int
espnor_rdsr(struct espnor_softc *sc, uint8_t cmd, uint8_t *sr)
{
	int error;

	if ((error = espnor_usr(sc, cmd, 0, 0, 0, 1)) != 0)
		return error;
	*sr = RD(sc, SPI_W(0)) & 0xff;
	return 0;
}

/* Read from the flash, at any offset and length. */
static int
espnor_readflash(struct espnor_softc *sc, uint32_t addr, uint8_t *buf,
    size_t len)
{
	int error;

	while (len > 0) {
		const size_t n = MIN(len, ESPNOR_CHUNK);

		error = espnor_usr(sc, FLASH_READ_FAST, 24, addr, 8, n);
		if (error)
			return error;
		espnor_getbuf(sc, buf, n);
		addr += n;
		buf += n;
		len -= n;
	}
	return 0;
}

/*
 * Set up auto-suspend, auto-resume and auto wait-idle, as ESP-IDF's
 * spi_flash_hal_setup_read_suspend(), _setup_auto_suspend_mode() and
 * _setup_auto_resume_mode() with spi_flash_chip_generic's commands.
 * Done before each erase and program, in case anything else on the chip
 * changed them.
 */
static void
espnor_setup_suspend(struct espnor_softc *sc)
{
	uint32_t v;

	WR(sc, SPI_FLASH_SUS_CMD,
	    __SHIFTIN(FLASH_SUSPEND, SUS_CMD_PES_COMMAND) |
	    __SHIFTIN(FLASH_RDSR2, SUS_CMD_WAIT_PESR_COMMAND));

	v = RD(sc, SPI_SUS_STATUS);
	v &= ~(SUS_STATUS_PER_COMMAND | SUS_STATUS_WAIT_PESR_CMD_2B |
	    SUS_STATUS_PESR_CMD_2B);
	v |= __SHIFTIN(FLASH_RESUME, SUS_STATUS_PER_COMMAND) |
	    SUS_STATUS_PER_DLY_128 | SUS_STATUS_PES_DLY_128 |
	    SUS_STATUS_SPI0_LOCK_EN;
	WR(sc, SPI_SUS_STATUS, v);

	v = RD(sc, SPI_CTRL1);
	v &= ~(CTRL1_CS_HOLD_DLY_RES | CTRL1_CS_HOLD_DLY_PER);
	v |= __SHIFTIN(ESPNOR_TSUS, CTRL1_CS_HOLD_DLY_RES) |
	    __SHIFTIN(ESPNOR_TRS, CTRL1_CS_HOLD_DLY_PER);
	WR(sc, SPI_CTRL1, v);

	v = RD(sc, SPI_FLASH_WAITI_CTRL);
	v &= ~(WAITI_CMD | WAITI_CMD_2B | WAITI_DUMMY | WAITI_ADDR_EN);
	v |= WAITI_EN | __SHIFTIN(FLASH_RDSR, WAITI_CMD);
	WR(sc, SPI_FLASH_WAITI_CTRL, v);

	WR(sc, SPI_FLASH_SUS_CTRL,
	    SUS_CTRL_PER_WAIT_EN | SUS_CTRL_PES_PER_EN | SUS_CTRL_PES_EN |
	    __SHIFTIN(FLASH_SR2_SUS, SUS_CTRL_PESR_END_MSK) |
	    SUS_CTRL_PER_END_EN | SUS_CTRL_PES_END_EN |
	    __SHIFTIN(5, SUS_CTRL_SUS_TIMEOUT_CNT));

	v = RD0(sc, SPI0_CTRL2);
	if (__SHIFTOUT(v, CTRL2_CS_HOLD_DELAY) < ESPNOR_TSHSL2) {
		v &= ~CTRL2_CS_HOLD_DELAY;
		v |= __SHIFTIN(ESPNOR_TSHSL2, CTRL2_CS_HOLD_DELAY);
		WR0(sc, SPI0_CTRL2, v);
	}
	v = RD0(sc, SPI0_FSM);
	v &= ~FSM_LOCK_DELAY_TIME;
	v |= __SHIFTIN(ESPNOR_SPI0_LOCK, FSM_LOCK_DELAY_TIME);
	WR0(sc, SPI0_FSM, v);
}

/* Write enable, then set up a built-in command with a 24-bit address. */
static int
espnor_pe_start(struct espnor_softc *sc, uint32_t addr)
{
	int error;

	espnor_setup_suspend(sc);

	/* Single-line everything; WP not asserted. */
	WR(sc, SPI_CTRL, RD(sc, SPI_CTRL) & CTRL_WP_REG);
	WR(sc, SPI_CACHE_FCTRL,
	    RD(sc, SPI_CACHE_FCTRL) & ~CACHE_FCTRL_ADDR_4BYTE);
	WR(sc, SPI_USER, 0);
	WR(sc, SPI_CMD, CMD_FLASH_WREN);
	if ((error = espnor_wait(sc, ESPNOR_CMD_TIMO,
	    ESPNOR_CMD_TIMO)) != 0)
		return error;

	WR(sc, SPI_USER1, __SHIFTIN(24 - 1, USER1_ADDR_BITLEN));
	WR(sc, SPI_ADDR, addr);
	return 0;
}

/*
 * After an erase or program: the controller's wait-idle has seen the chip
 * finish.  Make sure, in case wait-idle isn't doing its job.
 */
static int
espnor_pe_done(struct espnor_softc *sc, u_int timo_us, u_int spin_us)
{
	uint8_t sr;
	u_int t;
	int error;

	if ((error = espnor_wait(sc, timo_us, spin_us)) != 0)
		return error;
	for (t = 0;; ) {
		if ((error = espnor_rdsr(sc, FLASH_RDSR, &sr)) != 0)
			return error;
		if ((sr & FLASH_SR_WIP) == 0 &&
		    (RD(sc, SPI_SUS_STATUS) & SUS_STATUS_FLASH_SUS) == 0)
			return 0;
		if (t >= timo_us)
			return ETIMEDOUT;
		if (t >= spin_us) {
			kpause("espnor", false, 1, NULL);
			t += tick;
		} else {
			DELAY(10);
			t += 10;
		}
	}
}

static int
espnor_erase_sector(struct espnor_softc *sc, uint32_t addr)
{
	int error;

	KASSERT((addr % ESPNOR_SECTOR) == 0);

	if ((error = espnor_pe_start(sc, addr)) != 0)
		return error;
	WR(sc, SPI_CMD, CMD_FLASH_SE);
	return espnor_pe_done(sc, ESPNOR_SE_TIMO, 200);
}

/* Program up to 64 bytes, not crossing a 256-byte page. */
static int
espnor_program(struct espnor_softc *sc, uint32_t addr,
    const uint8_t *buf, size_t len)
{
	int error;

	KASSERT(len > 0 && len <= ESPNOR_CHUNK);
	KASSERT((addr & 0xff) + len <= 256);

	if ((error = espnor_pe_start(sc, addr)) != 0)
		return error;
	WR(sc, SPI_ADDR, addr | (len << 24));
	for (u_int i = 0; i < howmany(len, 4); i++) {
		uint32_t w = 0;

		memcpy(&w, buf + 4 * i, MIN(len - 4 * i, 4));
		WR(sc, SPI_W(i), le32toh(w));
	}
	WR(sc, SPI_CMD, CMD_FLASH_PP);
	return espnor_pe_done(sc, ESPNOR_PP_TIMO, 2000);
}

/*
 * Make a sector (at sector-aligned partition offset off) go from old to
 * new.  Clobbers old.
 */
static int
espnor_update_sector(struct espnor_softc *sc, uint32_t off,
    uint8_t *old, const uint8_t *new)
{
	const uint32_t addr = sc->sc_offset + off;
	bool erase = false;
	int error;

	if (memcmp(old, new, ESPNOR_SECTOR) == 0)
		return 0;

	for (u_int i = 0; i < ESPNOR_SECTOR; i++) {
		if (new[i] & ~old[i]) {
			erase = true;
			break;
		}
	}
	if (erase) {
		if ((error = espnor_erase_sector(sc, addr)) != 0) {
			device_printf(sc->sc_dev,
			    "erase at 0x%" PRIx32 ": error %d\n", addr, error);
			return EIO;
		}
		memset(old, 0xff, ESPNOR_SECTOR);
	}
	for (u_int i = 0; i < ESPNOR_SECTOR; i += ESPNOR_CHUNK) {
		if (memcmp(old + i, new + i, ESPNOR_CHUNK) == 0)
			continue;
		if ((error = espnor_program(sc, addr + i, new + i,
		    ESPNOR_CHUNK)) != 0) {
			device_printf(sc->sc_dev,
			    "program at 0x%" PRIx32 ": error %d\n", addr + i,
			    error);
			return EIO;
		}
	}

	if ((error = espnor_readflash(sc, addr, old, ESPNOR_SECTOR)) != 0)
		return error;
	if (memcmp(old, new, ESPNOR_SECTOR) != 0) {
		device_printf(sc->sc_dev,
		    "sector at 0x%" PRIx32 " doesn't read back as written\n",
		    addr);
		return EIO;
	}
	return 0;
}

static int
espnor_writepart(struct espnor_softc *sc, uint32_t off,
    const uint8_t *buf, size_t len)
{
	int error;

	while (len > 0) {
		const uint32_t sec = rounddown(off, ESPNOR_SECTOR);
		const uint32_t soff = off - sec;
		const size_t n = MIN(len, ESPNOR_SECTOR - soff);

		error = espnor_readflash(sc, sc->sc_offset + sec, sc->sc_old,
		    ESPNOR_SECTOR);
		if (error)
			return error;
		memcpy(sc->sc_new, sc->sc_old, ESPNOR_SECTOR);
		memcpy(sc->sc_new + soff, buf, n);
		error = espnor_update_sector(sc, sec, sc->sc_old,
		    sc->sc_new);
		if (error)
			return error;
		off += n;
		buf += n;
		len -= n;
	}
	return 0;
}

/*
 * The disk
 */

static int
espnor_io(struct espnor_softc *sc, struct buf *bp)
{
	const uint64_t off = (uint64_t)bp->b_rawblkno * DEV_BSIZE;
	const size_t len = bp->b_bcount;

	if (off > sc->sc_size || len > sc->sc_size - off)
		return EINVAL;
	if (bp->b_flags & B_READ)
		return espnor_readflash(sc, sc->sc_offset + off, bp->b_data,
		    len);
	return espnor_writepart(sc, off, bp->b_data, len);
}

static void
espnor_thread(void *arg)
{
	struct espnor_softc * const sc = arg;
	struct buf *bp;
	int error;

	for (;;) {
		mutex_enter(&sc->sc_lock);
		while ((bp = sc->sc_bp) == NULL)
			cv_wait(&sc->sc_cv, &sc->sc_lock);
		mutex_exit(&sc->sc_lock);

		error = espnor_io(sc, bp);
		bp->b_error = error;
		bp->b_resid = error ? bp->b_bcount : 0;

		mutex_enter(&sc->sc_lock);
		sc->sc_bp = NULL;
		mutex_exit(&sc->sc_lock);

		dk_done(&sc->sc_dksc, bp);
		dk_start(&sc->sc_dksc, NULL);
	}
}

static int
espnor_diskstart(device_t self, struct buf *bp)
{
	struct espnor_softc * const sc = device_private(self);

	mutex_enter(&sc->sc_lock);
	if (sc->sc_bp != NULL) {
		mutex_exit(&sc->sc_lock);
		return EAGAIN;
	}
	sc->sc_bp = bp;
	cv_signal(&sc->sc_cv);
	mutex_exit(&sc->sc_lock);
	return 0;
}

/* Without a label on the disk: all of it is an FFS partition, a. */
static void
espnor_label(device_t self, struct disklabel *lp)
{
	lp->d_partitions[0].p_offset = 0;
	lp->d_partitions[0].p_size = lp->d_secperunit;
	lp->d_partitions[0].p_fstype = FS_BSDFFS;
	if (lp->d_npartitions < 1)
		lp->d_npartitions = 1;
}

static struct espnor_softc *
espnor_lookup(dev_t dev)
{
	return device_lookup_private(&espnor_cd, DISKUNIT(dev));
}

static int
espnor_open(dev_t dev, int flags, int fmt, struct lwp *l)
{
	struct espnor_softc *sc = espnor_lookup(dev);

	if (sc == NULL)
		return ENXIO;
	return dk_open(&sc->sc_dksc, dev, flags, fmt, l);
}

static int
espnor_close(dev_t dev, int flags, int fmt, struct lwp *l)
{
	struct espnor_softc *sc = espnor_lookup(dev);

	return dk_close(&sc->sc_dksc, dev, flags, fmt, l);
}

static int
espnor_read(dev_t dev, struct uio *uio, int ioflag)
{
	return physio(espnor_strategy, NULL, dev, B_READ, minphys, uio);
}

static int
espnor_write(dev_t dev, struct uio *uio, int ioflag)
{
	return physio(espnor_strategy, NULL, dev, B_WRITE, minphys, uio);
}

static int
espnor_ioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct espnor_softc *sc = espnor_lookup(dev);

	if (sc == NULL)
		return ENXIO;

	switch (cmd) {
	case DIOCCACHESYNC:
		/* Writes go straight to the flash. */
		return (flag & FWRITE) ? 0 : EBADF;
	default:
		return dk_ioctl(&sc->sc_dksc, dev, cmd, data, flag, l);
	}
}

static void
espnor_strategy(struct buf *bp)
{
	struct espnor_softc *sc = espnor_lookup(bp->b_dev);

	if (sc == NULL) {
		bp->b_error = ENXIO;
		bp->b_resid = bp->b_bcount;
		biodone(bp);
		return;
	}
	dk_strategy(&sc->sc_dksc, bp);
}

static int
espnor_size(dev_t dev)
{
	struct espnor_softc *sc = espnor_lookup(dev);

	if (sc == NULL)
		return -1;
	return dk_size(&sc->sc_dksc, dev);
}

/*
 * Autoconfiguration
 */

static int
espnor_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
espnor_attach(device_t parent, device_t self, void *aux)
{
	struct espnor_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct dk_softc * const dksc = &sc->sc_dksc;
	struct disk_geom *dg;
	bus_addr_t addr, addr0;
	bus_size_t size, size0;
	uint32_t part[2], id, chipsize;
	char pbuf[9];
	int error;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;

	if (fdtbus_get_reg_byname(phandle, "spi1", &addr, &size) != 0 ||
	    fdtbus_get_reg_byname(phandle, "spi0", &addr0, &size0) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (of_getprop_uint32_array(phandle, "espressif,partition", part,
	    2) != 0 || part[1] == 0 ||
	    (part[0] % ESPNOR_SECTOR) != 0 ||
	    (part[1] % ESPNOR_SECTOR) != 0) {
		aprint_error(": no valid espressif,partition\n");
		return;
	}
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0 ||
	    bus_space_map(sc->sc_bst, addr0, size0, 0,
	    &sc->sc_spi0_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	sc->sc_offset = part[0];
	sc->sc_size = part[1];

	/* Nothing should be going on, but in case of a warm boot ... */
	if (espnor_wait(sc, ESPNOR_SE_TIMO, ESPNOR_SE_TIMO) != 0) {
		aprint_error(": SPI1 busy\n");
		return;
	}
	if (espnor_usr(sc, FLASH_RDID, 0, 0, 0, 3) != 0) {
		aprint_error(": no answer from the flash\n");
		return;
	}
	id = RD(sc, SPI_W(0)) & 0xffffff;
	if (id == 0 || id == 0xffffff) {
		aprint_error(": no flash chip (ID %06x)\n", id);
		return;
	}
	chipsize = 0;
	if ((id >> 16) >= 0x10 && (id >> 16) <= 0x1f)
		chipsize = 1U << (id >> 16);

	format_bytes(pbuf, sizeof(pbuf), sc->sc_size);
	aprint_naive("\n");
	aprint_normal(": SPI flash (ID %02x %02x %02x), %s at 0x%" PRIx32 "\n",
	    id & 0xff, (id >> 8) & 0xff, id >> 16, pbuf, sc->sc_offset);
	if (chipsize != 0 && sc->sc_offset + sc->sc_size > chipsize) {
		aprint_error_dev(self, "partition is past the end of the "
		    "flash (%u KB)\n", chipsize / 1024);
		return;
	}

	sc->sc_old = kmem_alloc(ESPNOR_SECTOR, KM_SLEEP);
	sc->sc_new = kmem_alloc(ESPNOR_SECTOR, KM_SLEEP);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);
	cv_init(&sc->sc_cv, "espnor");

	espnor_setup_suspend(sc);

	error = kthread_create(PRI_BIO, KTHREAD_MPSAFE, NULL, espnor_thread,
	    sc, &sc->sc_thread, "%s", device_xname(self));
	if (error) {
		aprint_error_dev(self, "couldn't create thread: %d\n", error);
		return;
	}

	dk_init(dksc, self, DKTYPE_FLASH);
	disk_init(&dksc->sc_dkdev, dksc->sc_xname, &espnor_dkdriver);
	dg = &dksc->sc_dkdev.dk_geom;
	memset(dg, 0, sizeof(*dg));
	dg->dg_secsize = DEV_BSIZE;
	dg->dg_secperunit = sc->sc_size / DEV_BSIZE;
	dg->dg_nsectors = ESPNOR_SECTOR / DEV_BSIZE;
	dg->dg_ntracks = 1;
	dg->dg_ncylinders = dg->dg_secperunit / dg->dg_nsectors;
	dg->dg_physsecsize = ESPNOR_SECTOR;
	dk_attach(dksc);
	disk_attach(&dksc->sc_dkdev);
	bufq_alloc(&dksc->sc_bufq, "fcfs", 0);
	disk_set_info(self, &dksc->sc_dkdev, "SPI flash");
}
