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
 * ESP32-S31 I2S, as the CPU side of a simple-audio-card (audio(4) through
 * ausoc): standard (Philips) I2S, 16-bit stereo, 8 to 48 kHz, playback and
 * capture at once.  The port is the clock master: TX makes BCK and WS (and
 * MCLK, 256 fs) off the 40 MHz XTAL through the fractional divider, and RX
 * runs off them internally, so one codec wired to BCK, WS, DOUT and DIN
 * does both directions; capture alone keeps the clocks running.
 *
 * Data moves through a pair of the AHB GDMA's channels, each a ring of
 * descriptors, one an audio block, with an interrupt at the end of each.
 * The buffers are in (cached) PSRAM: bus_dma(9) syncs keep the cache out
 * of the way, the whole playback ring written back at every block (one
 * cache operation; the audio layer fills blocks well ahead), each capture
 * block invalidated as it completes.
 *
 *	i2s0: i2s@20387000 {
 *		compatible = "espressif,esp32s31-i2s";
 *		reg = <0x20387000 0x1000>,	(I2S)
 *		      <0x205870e8 0x10>,	(its HP_SYS_CLKRST clocks)
 *		      <0x20343000 0x1000>;	(AHB GDMA)
 *		interrupt-parent = <&intmtx>;
 *		interrupts = <41 4>, <36 4>;	(DMA out, in)
 *		clocks = <&clkrst 43>, <&clkrst 45>;
 *		resets = <&clkrst 35>;
 *		espressif,dma-channel = <0>;
 *		#sound-dai-cells = <0>;
 *	};
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/systm.h>

#include <sys/audioio.h>
#include <dev/audio/audio_if.h>
#include <dev/audio/audio_dai.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>

/* I2S */
#define	I2S_RX_CONF		0x20
#define	I2S_TX_CONF		0x24
#define	 CONF_RESET		__BIT(0)
#define	 CONF_FIFO_RESET	__BIT(1)
#define	 CONF_START		__BIT(2)
#define	 CONF_SLAVE_MOD		__BIT(3)
#define	 CONF_UPDATE		__BIT(8)
#define	 TX_CONF_SIG_LOOPBACK	__BIT(30)
/* standard I2S, 16 bits a slot, TDM engine, BCK = MCLK / 8 */
#define	I2S_TX_CONF_STD		0x00e8f200	/* TX_STOP_EN clear */
#define	I2S_RX_CONF_STD		0x00e8b400
#define	I2S_RX_CONF1		0x28
#define	I2S_TX_CONF1		0x2c
#define	 CONF1_STD16		0x787bc00f
#define	I2S_TX_PCM2PDM_CONF	0x44
#define	 PCM2PDM_CONV_EN	__BIT(25)
#define	I2S_RX_PDM2PCM_CONF	0x4c
#define	 PDM2PCM_EN		__BIT(19)
#define	I2S_RX_TDM_CTRL		0x50
#define	I2S_TX_TDM_CTRL		0x54
#define	 TDM_STEREO		0x00010003	/* two channels, both on */
#define	I2S_RXEOF_NUM		0x64
#define	I2S_DESTINATION		0xf4

/* HP_SYS_CLKRST, from the port's RX_CTRL0 */
#define	CLK_RX_CTRL0		0x0
#define	CLK_RX_DIV		0x4
#define	CLK_TX_CTRL0		0x8
#define	CLK_TX_DIV		0xc
#define	 CTRL0_CLK_EN		__BIT(0)
#define	 CTRL0_SRC_SEL		__BITS(2, 1)	/* 0: XTAL */
#define	 CTRL0_DIV_N		__BITS(10, 3)
#define	 CTRL0_MST_CLK_SEL	__BIT(11)	/* RX: MCLK from TX */
#define	 DIV_X			__BITS(8, 0)
#define	 DIV_Y			__BITS(17, 9)
#define	 DIV_Z			__BITS(26, 18)
#define	 DIV_YN1		__BIT(27)

/* AHB GDMA, channel pair p */
#define	DMA_IN_INT_RAW(p)	(0x00 + 0x10 * (p))
#define	DMA_IN_INT_ENA(p)	(0x08 + 0x10 * (p))
#define	DMA_IN_INT_CLR(p)	(0x0c + 0x10 * (p))
#define	 IN_INT_SUC_EOF		__BIT(1)
#define	 IN_INT_ERR_EOF		__BIT(2)
#define	DMA_OUT_INT_RAW(p)	(0x50 + 0x10 * (p))
#define	DMA_OUT_INT_ENA(p)	(0x58 + 0x10 * (p))
#define	DMA_OUT_INT_CLR(p)	(0x5c + 0x10 * (p))
#define	 OUT_INT_EOF		__BIT(1)
#define	DMA_IN_CONF0(p)		(0x100 + 0x100 * (p))
#define	 IN_RST			__BIT(0)
#define	 INDSCR_BURST_EN	__BIT(2)
#define	 IN_DATA_BURST_SEL	__BITS(7, 6)
#define	DMA_IN_CONF1(p)		(0x104 + 0x100 * (p))
#define	DMA_IN_LINK(p)		(0x110 + 0x100 * (p))
#define	 IN_LINK_AUTO_RET	__BIT(0)
#define	 IN_LINK_STOP		__BIT(1)
#define	 IN_LINK_START		__BIT(2)
#define	DMA_IN_LINK_ADDR(p)	(0x114 + 0x100 * (p))
#define	DMA_IN_SUC_EOF_DES(p)	(0x11c + 0x100 * (p))
#define	DMA_IN_PERI_SEL(p)	(0x138 + 0x100 * (p))
#define	DMA_OUT_CONF0(p)	(0x180 + 0x100 * (p))
#define	 OUT_RST		__BIT(0)
#define	 OUT_EOF_MODE		__BIT(3)
#define	 OUTDSCR_BURST_EN	__BIT(4)
#define	 OUT_DATA_BURST_SEL	__BITS(9, 8)
#define	DMA_OUT_CONF1(p)	(0x184 + 0x100 * (p))
#define	DMA_OUT_LINK(p)		(0x190 + 0x100 * (p))
#define	 OUT_LINK_STOP		__BIT(0)
#define	 OUT_LINK_START		__BIT(1)
#define	DMA_OUT_LINK_ADDR(p)	(0x194 + 0x100 * (p))
#define	DMA_OUT_EOF_DES(p)	(0x19c + 0x100 * (p))
#define	DMA_OUT_PERI_SEL(p)	(0x1b8 + 0x100 * (p))
#define	DMA_INTR_MEM_START	0x600
#define	DMA_INTR_MEM_END	0x604
#define	 BURST_32		2

/* GDMA descriptor */
struct esp_i2s_desc {
	uint32_t	d_ctrl;
#define	DESC_SIZE		__BITS(11, 0)
#define	DESC_LENGTH		__BITS(23, 12)
#define	DESC_SUC_EOF		__BIT(30)
#define	DESC_OWNER_DMA		__BIT(31)
	uint32_t	d_buf;
	uint32_t	d_next;
};

#define	ESP_I2S_SCLK		40000000	/* XTAL */
#define	ESP_I2S_MCLK_FS		256
#define	ESP_I2S_BCK_DIV		8		/* MCLK / (2 x 16 bits) */
#define	ESP_I2S_ALIGN		128		/* cache line */
#define	ESP_I2S_MAXBLK		3968		/* one descriptor, aligned */
#define	ESP_I2S_MAXBUF		65536
#define	ESP_I2S_NDESC		(ESP_I2S_MAXBUF / ESP_I2S_ALIGN)

struct esp_i2s_dma {
	LIST_ENTRY(esp_i2s_dma)	dma_list;
	bus_dmamap_t		dma_map;
	bus_dma_segment_t	dma_seg;
	void			*dma_kva;
	size_t			dma_size;
};

struct esp_i2s_chan {
	bool			ch_out;
	struct esp_i2s_dma	*ch_desc;	/* descriptor ring */
	struct esp_i2s_dma	*ch_buf;	/* the audio layer's ring */
	u_int			ch_nblk;
	u_int			ch_blksize;
	u_int			ch_next;	/* next block to complete */
	void			(*ch_intr)(void *);
	void			*ch_intrarg;
	bool			ch_running;
};

struct esp_i2s_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;		/* I2S */
	bus_space_handle_t	sc_cbsh;	/* its clocks */
	bus_space_handle_t	sc_dbsh;	/* AHB GDMA */
	bus_dma_tag_t		sc_dmat;
	u_int			sc_pair;	/* GDMA channel pair */
	u_int			sc_unit;
	kmutex_t		sc_lock;
	kmutex_t		sc_intr_lock;
	LIST_HEAD(, esp_i2s_dma) sc_dmalist;
	struct esp_i2s_chan	sc_play, sc_rec;
	u_int			sc_rate;
	struct audio_format	sc_format;
	struct audio_dai_device	sc_dai;
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))
#define	CRD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_cbsh, (r))
#define	CWR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_cbsh, (r), (v))
#define	DRD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_dbsh, (r))
#define	DWR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_dbsh, (r), (v))

static int	esp_i2s_match(device_t, cfdata_t, void *);
static void	esp_i2s_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_i2s, sizeof(struct esp_i2s_softc),
	esp_i2s_match, esp_i2s_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-i2s" },
	DEVICE_COMPAT_EOL
};

static const u_int esp_i2s_rates[] = {
	8000, 11025, 16000, 22050, 32000, 44100, 48000
};

static void
esp_i2s_rmw(struct esp_i2s_softc *sc, bus_size_t r, uint32_t clr,
    uint32_t set)
{
	WR(sc, r, (RD(sc, r) & ~clr) | set);
}

/*
 * Clocks
 */

/* MCLK = SCLK / (n + b/a), as ESP-IDF's hal_utils works it out */
static int
esp_i2s_mclk_div(u_int sclk, u_int mclk, u_int *np, u_int *bp, u_int *ap)
{
	u_int n = sclk / mclk, err = sclk % mclk, besta = 0, bestb = 0;
	uint64_t besterr = UINT64_MAX;

	if (sclk * 100ULL < mclk * 199ULL)
		return EINVAL;			/* needs SCLK >= 1.99 MCLK */
	if (err != 0 && err < mclk - (mclk / 511) * 2) {
		for (u_int a = 2; a < 512; a++) {
			const u_int b = ((uint64_t)a * err + mclk / 2) / mclk;
			const int64_t d = (int64_t)mclk * b - (int64_t)err * a;
			const uint64_t e = d < 0 ? -d : d;

			if (b == 0 || b >= a)
				continue;
			if (e < besterr) {
				besterr = e;
				besta = a;
				bestb = b;
				if (e == 0)
					break;
			}
		}
	} else if (err != 0) {
		n++;
	}
	if (n < 2 || n > 255)
		return EINVAL;
	*np = n;
	*bp = bestb;
	*ap = besta;
	return 0;
}

static void
esp_i2s_set_divider(struct esp_i2s_softc *sc, bus_size_t ctrl,
    bus_size_t div, u_int n, u_int num, u_int den)
{
	u_int x = 0, y = 0, z = 0, yn1 = 0;

	if (num != 0 && den != 0) {
		yn1 = num * 2 > den;
		z = yn1 ? den - num : num;
		x = den / z - 1;
		y = den % z;
	}
	/* in this order: works around the divider's double division */
	CWR(sc, ctrl, (CRD(sc, ctrl) & ~CTRL0_DIV_N) |
	    __SHIFTIN(2, CTRL0_DIV_N));
	CWR(sc, div, __SHIFTIN(1, DIV_Y));
	CWR(sc, div, __SHIFTIN(x, DIV_X) | __SHIFTIN(y, DIV_Y) |
	    __SHIFTIN(z, DIV_Z) | (yn1 ? DIV_YN1 : 0));
	CWR(sc, ctrl, (CRD(sc, ctrl) & ~CTRL0_DIV_N) |
	    __SHIFTIN(n, CTRL0_DIV_N));
}

static int
esp_i2s_set_rate(struct esp_i2s_softc *sc, u_int rate)
{
	u_int n, b, a;
	int error;

	error = esp_i2s_mclk_div(ESP_I2S_SCLK, rate * ESP_I2S_MCLK_FS,
	    &n, &b, &a);
	if (error)
		return error;
	/* TX and RX generators alike; MCLK out from TX's. */
	CWR(sc, CLK_TX_CTRL0, (CRD(sc, CLK_TX_CTRL0) & ~CTRL0_SRC_SEL) |
	    CTRL0_CLK_EN);
	CWR(sc, CLK_RX_CTRL0, (CRD(sc, CLK_RX_CTRL0) & ~CTRL0_SRC_SEL) |
	    CTRL0_CLK_EN | CTRL0_MST_CLK_SEL);
	esp_i2s_set_divider(sc, CLK_TX_CTRL0, CLK_TX_DIV, n, b, a);
	esp_i2s_set_divider(sc, CLK_RX_CTRL0, CLK_RX_DIV, n, b, a);
	sc->sc_rate = rate;
	return 0;
}

/* Registers to the I2S clock domain. */
static void
esp_i2s_update(struct esp_i2s_softc *sc, bus_size_t conf)
{
	esp_i2s_rmw(sc, conf, 0, CONF_UPDATE);
	for (u_int i = 0; i < 1000 && (RD(sc, conf) & CONF_UPDATE); i++)
		DELAY(1);
}

/*
 * DMA memory
 */

static struct esp_i2s_dma *
esp_i2s_dma_alloc(struct esp_i2s_softc *sc, size_t size)
{
	struct esp_i2s_dma *dma;
	int nsegs;

	dma = kmem_zalloc(sizeof(*dma), KM_SLEEP);
	dma->dma_size = roundup(size, ESP_I2S_ALIGN);
	if (bus_dmamem_alloc(sc->sc_dmat, dma->dma_size, ESP_I2S_ALIGN, 0,
	    &dma->dma_seg, 1, &nsegs, BUS_DMA_WAITOK) != 0)
		goto fail0;
	if (bus_dmamem_map(sc->sc_dmat, &dma->dma_seg, nsegs, dma->dma_size,
	    &dma->dma_kva, BUS_DMA_WAITOK) != 0)
		goto fail1;
	if (bus_dmamap_create(sc->sc_dmat, dma->dma_size, 1, dma->dma_size,
	    0, BUS_DMA_WAITOK, &dma->dma_map) != 0)
		goto fail2;
	if (bus_dmamap_load(sc->sc_dmat, dma->dma_map, dma->dma_kva,
	    dma->dma_size, NULL, BUS_DMA_WAITOK) != 0)
		goto fail3;
	memset(dma->dma_kva, 0, dma->dma_size);
	return dma;

fail3:	bus_dmamap_destroy(sc->sc_dmat, dma->dma_map);
fail2:	bus_dmamem_unmap(sc->sc_dmat, dma->dma_kva, dma->dma_size);
fail1:	bus_dmamem_free(sc->sc_dmat, &dma->dma_seg, 1);
fail0:	kmem_free(dma, sizeof(*dma));
	return NULL;
}

static void
esp_i2s_dma_free(struct esp_i2s_softc *sc, struct esp_i2s_dma *dma)
{
	bus_dmamap_unload(sc->sc_dmat, dma->dma_map);
	bus_dmamap_destroy(sc->sc_dmat, dma->dma_map);
	bus_dmamem_unmap(sc->sc_dmat, dma->dma_kva, dma->dma_size);
	bus_dmamem_free(sc->sc_dmat, &dma->dma_seg, 1);
	kmem_free(dma, sizeof(*dma));
}

static uint32_t
esp_i2s_dma_pa(struct esp_i2s_dma *dma, size_t off)
{
	return dma->dma_map->dm_segs[0].ds_addr + off;
}

/* Descriptors: one a block, a ring; the hardware ignores ownership. */
static void
esp_i2s_ring(struct esp_i2s_softc *sc, struct esp_i2s_chan *ch,
    void *start, void *end, int blksize)
{
	struct esp_i2s_desc * const d = ch->ch_desc->dma_kva;
	struct esp_i2s_dma *buf;

	LIST_FOREACH(buf, &sc->sc_dmalist, dma_list)
		if (buf->dma_kva == start)
			break;
	KASSERT(buf != NULL);
	ch->ch_buf = buf;
	ch->ch_blksize = blksize;
	ch->ch_nblk = ((uint8_t *)end - (uint8_t *)start) / blksize;
	KASSERT(ch->ch_nblk > 0 && ch->ch_nblk <= ESP_I2S_NDESC);
	ch->ch_next = 0;

	for (u_int i = 0; i < ch->ch_nblk; i++) {
		d[i].d_ctrl = __SHIFTIN(blksize, DESC_SIZE) |
		    __SHIFTIN(ch->ch_out ? blksize : 0, DESC_LENGTH) |
		    DESC_SUC_EOF | DESC_OWNER_DMA;
		d[i].d_buf = esp_i2s_dma_pa(buf, i * blksize);
		d[i].d_next = esp_i2s_dma_pa(ch->ch_desc,
		    ((i + 1) % ch->ch_nblk) * sizeof(*d));
	}
	bus_dmamap_sync(sc->sc_dmat, ch->ch_desc->dma_map, 0,
	    ch->ch_desc->dma_size, BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
}

/* Which block the descriptor at this address is. */
static u_int
esp_i2s_desc_index(struct esp_i2s_chan *ch, uint32_t pa)
{
	const uint32_t base = esp_i2s_dma_pa(ch->ch_desc, 0);

	return (pa - base) / sizeof(struct esp_i2s_desc);
}

/*
 * Clocks for both directions come from TX: run it while either is.
 */
static void
esp_i2s_tx_clock(struct esp_i2s_softc *sc, bool on)
{
	if (on && (RD(sc, I2S_TX_CONF) & CONF_START) == 0) {
		esp_i2s_update(sc, I2S_TX_CONF);
		esp_i2s_rmw(sc, I2S_TX_CONF, 0, CONF_START);
	} else if (!on) {
		esp_i2s_rmw(sc, I2S_TX_CONF, CONF_START, 0);
	}
}

/*
 * audio(4)
 */

static int
esp_i2s_query_format(void *priv, audio_format_query_t *afp)
{
	struct esp_i2s_softc * const sc = priv;

	return audio_query_format(&sc->sc_format, 1, afp);
}

static int
esp_i2s_set_format(void *priv, int setmode, const audio_params_t *play,
    const audio_params_t *rec, audio_filter_reg_t *pfil,
    audio_filter_reg_t *rfil)
{
	struct esp_i2s_softc * const sc = priv;
	const audio_params_t *p = (setmode & AUMODE_PLAY) ? play : rec;

	if (p == NULL)
		return 0;
	if (p->channels != 2 || p->precision != 16)
		return EINVAL;
	return esp_i2s_set_rate(sc, p->sample_rate);
}

static int
esp_i2s_round_blocksize(void *priv, int bs, int mode,
    const audio_params_t *params)
{
	bs = rounddown(bs, ESP_I2S_ALIGN);
	return MAX(MIN(bs, ESP_I2S_MAXBLK), ESP_I2S_ALIGN);
}

static size_t
esp_i2s_round_buffersize(void *priv, int dir, size_t size)
{
	return MIN(size, ESP_I2S_MAXBUF);
}

static int
esp_i2s_get_props(void *priv)
{
	return AUDIO_PROP_PLAYBACK | AUDIO_PROP_CAPTURE |
	    AUDIO_PROP_FULLDUPLEX;
}

static void *
esp_i2s_allocm(void *priv, int dir, size_t size)
{
	struct esp_i2s_softc * const sc = priv;
	struct esp_i2s_dma *dma;

	if ((dma = esp_i2s_dma_alloc(sc, size)) == NULL)
		return NULL;
	/* the list: under the interrupt lock, which trigger has */
	mutex_enter(&sc->sc_intr_lock);
	LIST_INSERT_HEAD(&sc->sc_dmalist, dma, dma_list);
	mutex_exit(&sc->sc_intr_lock);
	return dma->dma_kva;
}

static void
esp_i2s_freem(void *priv, void *addr, size_t size)
{
	struct esp_i2s_softc * const sc = priv;
	struct esp_i2s_dma *dma;

	mutex_enter(&sc->sc_intr_lock);
	LIST_FOREACH(dma, &sc->sc_dmalist, dma_list)
		if (dma->dma_kva == addr)
			break;
	if (dma != NULL)
		LIST_REMOVE(dma, dma_list);
	mutex_exit(&sc->sc_intr_lock);
	if (dma != NULL)
		esp_i2s_dma_free(sc, dma);
}

static int
esp_i2s_trigger_output(void *priv, void *start, void *end, int blksize,
    void (*intr)(void *), void *intrarg, const audio_params_t *params)
{
	struct esp_i2s_softc * const sc = priv;
	struct esp_i2s_chan * const ch = &sc->sc_play;
	const u_int p = sc->sc_pair;

	esp_i2s_ring(sc, ch, start, end, blksize);
	bus_dmamap_sync(sc->sc_dmat, ch->ch_buf->dma_map, 0,
	    ch->ch_buf->dma_size, BUS_DMASYNC_PREWRITE);
	ch->ch_intr = intr;
	ch->ch_intrarg = intrarg;

	esp_i2s_rmw(sc, I2S_TX_CONF, 0, CONF_FIFO_RESET);
	esp_i2s_rmw(sc, I2S_TX_CONF, CONF_FIFO_RESET, 0);
	DWR(sc, DMA_OUT_CONF0(p), DRD(sc, DMA_OUT_CONF0(p)) | OUT_RST);
	DWR(sc, DMA_OUT_CONF0(p), DRD(sc, DMA_OUT_CONF0(p)) & ~OUT_RST);
	DWR(sc, DMA_OUT_INT_CLR(p), 0xffffffff);
	DWR(sc, DMA_OUT_INT_ENA(p), OUT_INT_EOF);
	DWR(sc, DMA_OUT_LINK_ADDR(p), esp_i2s_dma_pa(ch->ch_desc, 0));
	DWR(sc, DMA_OUT_LINK(p), OUT_LINK_START);
	ch->ch_running = true;
	esp_i2s_tx_clock(sc, true);
	return 0;
}

static int
esp_i2s_trigger_input(void *priv, void *start, void *end, int blksize,
    void (*intr)(void *), void *intrarg, const audio_params_t *params)
{
	struct esp_i2s_softc * const sc = priv;
	struct esp_i2s_chan * const ch = &sc->sc_rec;
	const u_int p = sc->sc_pair;

	esp_i2s_ring(sc, ch, start, end, blksize);
	bus_dmamap_sync(sc->sc_dmat, ch->ch_buf->dma_map, 0,
	    ch->ch_buf->dma_size, BUS_DMASYNC_PREREAD);
	ch->ch_intr = intr;
	ch->ch_intrarg = intrarg;

	WR(sc, I2S_RXEOF_NUM, blksize);
	esp_i2s_rmw(sc, I2S_RX_CONF, 0, CONF_RESET | CONF_FIFO_RESET);
	esp_i2s_rmw(sc, I2S_RX_CONF, CONF_RESET | CONF_FIFO_RESET, 0);
	DWR(sc, DMA_IN_CONF0(p), DRD(sc, DMA_IN_CONF0(p)) | IN_RST);
	DWR(sc, DMA_IN_CONF0(p), DRD(sc, DMA_IN_CONF0(p)) & ~IN_RST);
	DWR(sc, DMA_IN_INT_CLR(p), 0xffffffff);
	DWR(sc, DMA_IN_INT_ENA(p), IN_INT_SUC_EOF | IN_INT_ERR_EOF);
	DWR(sc, DMA_IN_LINK_ADDR(p), esp_i2s_dma_pa(ch->ch_desc, 0));
	DWR(sc, DMA_IN_LINK(p), IN_LINK_AUTO_RET | IN_LINK_START);
	esp_i2s_update(sc, I2S_RX_CONF);
	esp_i2s_rmw(sc, I2S_RX_CONF, 0, CONF_START);
	ch->ch_running = true;
	esp_i2s_tx_clock(sc, true);
	return 0;
}

static int
esp_i2s_halt_output(void *priv)
{
	struct esp_i2s_softc * const sc = priv;
	struct esp_i2s_chan * const ch = &sc->sc_play;
	const u_int p = sc->sc_pair;

	ch->ch_running = false;
	if (!sc->sc_rec.ch_running)
		esp_i2s_tx_clock(sc, false);
	DWR(sc, DMA_OUT_LINK(p), OUT_LINK_STOP);
	DWR(sc, DMA_OUT_INT_ENA(p), 0);
	DWR(sc, DMA_OUT_INT_CLR(p), 0xffffffff);
	ch->ch_intr = NULL;
	return 0;
}

static int
esp_i2s_halt_input(void *priv)
{
	struct esp_i2s_softc * const sc = priv;
	struct esp_i2s_chan * const ch = &sc->sc_rec;
	const u_int p = sc->sc_pair;

	ch->ch_running = false;
	esp_i2s_rmw(sc, I2S_RX_CONF, CONF_START, 0);
	if (!sc->sc_play.ch_running)
		esp_i2s_tx_clock(sc, false);
	DWR(sc, DMA_IN_LINK(p), IN_LINK_AUTO_RET | IN_LINK_STOP);
	DWR(sc, DMA_IN_INT_ENA(p), 0);
	DWR(sc, DMA_IN_INT_CLR(p), 0xffffffff);
	ch->ch_intr = NULL;
	return 0;
}

static void
esp_i2s_get_locks(void *priv, kmutex_t **intr, kmutex_t **thread)
{
	struct esp_i2s_softc * const sc = priv;

	*intr = &sc->sc_intr_lock;
	*thread = &sc->sc_lock;
}

static const struct audio_hw_if esp_i2s_hw_if = {
	.query_format = esp_i2s_query_format,
	.set_format = esp_i2s_set_format,
	.round_blocksize = esp_i2s_round_blocksize,
	.round_buffersize = esp_i2s_round_buffersize,
	.get_props = esp_i2s_get_props,
	.allocm = esp_i2s_allocm,
	.freem = esp_i2s_freem,
	.trigger_output = esp_i2s_trigger_output,
	.trigger_input = esp_i2s_trigger_input,
	.halt_output = esp_i2s_halt_output,
	.halt_input = esp_i2s_halt_input,
	.get_locks = esp_i2s_get_locks,
};

/* Blocks done up to and including the one at this descriptor. */
static void
esp_i2s_complete(struct esp_i2s_softc *sc, struct esp_i2s_chan *ch,
    uint32_t despa)
{
	const u_int last = esp_i2s_desc_index(ch, despa);

	if (!ch->ch_running || last >= ch->ch_nblk)
		return;
	for (u_int n = 0; n < ch->ch_nblk; n++) {
		const u_int blk = ch->ch_next;

		if (!ch->ch_out)
			bus_dmamap_sync(sc->sc_dmat, ch->ch_buf->dma_map,
			    blk * ch->ch_blksize, ch->ch_blksize,
			    BUS_DMASYNC_POSTREAD);
		ch->ch_next = (blk + 1) % ch->ch_nblk;
		if (ch->ch_intr != NULL)
			ch->ch_intr(ch->ch_intrarg);
		if (blk == last)
			break;
	}
	if (ch->ch_out) {
		/* What the audio layer has written since, out to memory. */
		bus_dmamap_sync(sc->sc_dmat, ch->ch_buf->dma_map, 0,
		    ch->ch_buf->dma_size, BUS_DMASYNC_PREWRITE);
	}
}

static int
esp_i2s_intr_out(void *priv)
{
	struct esp_i2s_softc * const sc = priv;
	const u_int p = sc->sc_pair;
	uint32_t st;

	mutex_enter(&sc->sc_intr_lock);
	st = DRD(sc, DMA_OUT_INT_RAW(p)) & DRD(sc, DMA_OUT_INT_ENA(p));
	DWR(sc, DMA_OUT_INT_CLR(p), st);
	if (st & OUT_INT_EOF)
		esp_i2s_complete(sc, &sc->sc_play,
		    DRD(sc, DMA_OUT_EOF_DES(p)));
	mutex_exit(&sc->sc_intr_lock);
	return st != 0;
}

static int
esp_i2s_intr_in(void *priv)
{
	struct esp_i2s_softc * const sc = priv;
	const u_int p = sc->sc_pair;
	uint32_t st;

	mutex_enter(&sc->sc_intr_lock);
	st = DRD(sc, DMA_IN_INT_RAW(p)) & DRD(sc, DMA_IN_INT_ENA(p));
	DWR(sc, DMA_IN_INT_CLR(p), st);
	if (st & (IN_INT_SUC_EOF | IN_INT_ERR_EOF))
		esp_i2s_complete(sc, &sc->sc_rec,
		    DRD(sc, DMA_IN_SUC_EOF_DES(p)));
	mutex_exit(&sc->sc_intr_lock);
	return st != 0;
}

/*
 * DAI
 */

static int
esp_i2s_dai_set_format(audio_dai_tag_t dai, u_int format)
{
	const u_int fmt = __SHIFTOUT(format, AUDIO_DAI_FORMAT_MASK);
	const u_int pol = __SHIFTOUT(format, AUDIO_DAI_POLARITY_MASK);
	const u_int clk = __SHIFTOUT(format, AUDIO_DAI_CLOCK_MASK);

	/* Standard I2S, this end the clock master. */
	if (fmt != AUDIO_DAI_FORMAT_I2S || pol != AUDIO_DAI_POLARITY_NB_NF ||
	    clk != AUDIO_DAI_CLOCK_CBS_CFS)
		return EINVAL;
	return 0;
}

static audio_dai_tag_t
esp_i2s_dai_get_tag(device_t dev, const void *data, size_t len)
{
	struct esp_i2s_softc * const sc = device_private(dev);

	if (len != 4)
		return NULL;
	return &sc->sc_dai;
}

static struct fdtbus_dai_controller_func esp_i2s_dai_funcs = {
	.get_tag = esp_i2s_dai_get_tag
};

static void
esp_i2s_init(struct esp_i2s_softc *sc)
{
	const u_int p = sc->sc_pair;
	const u_int peri = sc->sc_unit == 0 ? 1 : 5;	/* I2S0, I2S1 */

	/* The DMA may reach internal SRAM and PSRAM. */
	DWR(sc, DMA_INTR_MEM_START, 0x2f000000);
	DWR(sc, DMA_INTR_MEM_END, 0x53ffffff);
	DWR(sc, DMA_OUT_INT_ENA(p), 0);
	DWR(sc, DMA_OUT_INT_CLR(p), 0xffffffff);
	DWR(sc, DMA_IN_INT_ENA(p), 0);
	DWR(sc, DMA_IN_INT_CLR(p), 0xffffffff);
	DWR(sc, DMA_OUT_CONF0(p), OUT_EOF_MODE | OUTDSCR_BURST_EN |
	    __SHIFTIN(BURST_32, OUT_DATA_BURST_SEL));
	DWR(sc, DMA_OUT_CONF1(p), 0);		/* no owner check */
	DWR(sc, DMA_IN_CONF0(p), INDSCR_BURST_EN |
	    __SHIFTIN(BURST_32, IN_DATA_BURST_SEL));
	DWR(sc, DMA_IN_CONF1(p), 0);
	DWR(sc, DMA_OUT_PERI_SEL(p), peri);
	DWR(sc, DMA_IN_PERI_SEL(p), peri);

	/* Standard I2S, 16-bit stereo; RX on TX's BCK and WS. */
	WR(sc, I2S_TX_CONF, I2S_TX_CONF_STD | CONF_RESET | CONF_FIFO_RESET);
	WR(sc, I2S_TX_CONF, I2S_TX_CONF_STD | TX_CONF_SIG_LOOPBACK);
	WR(sc, I2S_RX_CONF, I2S_RX_CONF_STD | CONF_RESET | CONF_FIFO_RESET);
	WR(sc, I2S_RX_CONF, I2S_RX_CONF_STD | CONF_SLAVE_MOD);
	WR(sc, I2S_TX_CONF1, CONF1_STD16);
	WR(sc, I2S_RX_CONF1, CONF1_STD16);
	WR(sc, I2S_TX_TDM_CTRL, TDM_STEREO);
	WR(sc, I2S_RX_TDM_CTRL, TDM_STEREO);
	esp_i2s_rmw(sc, I2S_TX_PCM2PDM_CONF, PCM2PDM_CONV_EN, 0);
	esp_i2s_rmw(sc, I2S_RX_PDM2PCM_CONF, PDM2PCM_EN, 0);
	WR(sc, I2S_DESTINATION, 0);
	(void)esp_i2s_set_rate(sc, 48000);
}

static int
esp_i2s_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_i2s_attach(device_t parent, device_t self, void *aux)
{
	struct esp_i2s_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_space_handle_t *bshs[] = { &sc->sc_bsh, &sc->sc_cbsh,
	    &sc->sc_dbsh };
	char intrstr[2][128];
	struct fdtbus_reset *rst;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	sc->sc_dmat = faa->faa_dmat;
	LIST_INIT(&sc->sc_dmalist);
	for (u_int i = 0; i < __arraycount(bshs); i++) {
		if (fdtbus_get_reg(phandle, i, &addr, &size) != 0 ||
		    bus_space_map(sc->sc_bst, addr, size, 0, bshs[i]) != 0) {
			aprint_error(": couldn't map registers\n");
			return;
		}
		if (i == 0)
			sc->sc_unit = addr == 0x20388000 ? 1 : 0;
	}
	if (of_getprop_uint32(phandle, "espressif,dma-channel",
	    &sc->sc_pair) != 0)
		sc->sc_pair = sc->sc_unit;
	for (u_int i = 0; (clk = fdtbus_clock_get_index(phandle, i)) != NULL;
	    i++) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock\n");
			return;
		}
	}
	if ((rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		fdtbus_reset_assert(rst);
		fdtbus_reset_deassert(rst);
	}
	for (u_int i = 0; i < 2; i++) {
		if (!fdtbus_intr_str(phandle, i, intrstr[i],
		    sizeof(intrstr[i]))) {
			aprint_error(": couldn't decode interrupt\n");
			return;
		}
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);
	mutex_init(&sc->sc_intr_lock, MUTEX_DEFAULT, IPL_AUDIO);
	sc->sc_play.ch_out = true;
	sc->sc_play.ch_desc = esp_i2s_dma_alloc(sc,
	    ESP_I2S_NDESC * sizeof(struct esp_i2s_desc));
	sc->sc_rec.ch_desc = esp_i2s_dma_alloc(sc,
	    ESP_I2S_NDESC * sizeof(struct esp_i2s_desc));
	if (sc->sc_play.ch_desc == NULL || sc->sc_rec.ch_desc == NULL) {
		aprint_error(": couldn't allocate descriptors\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": I2S%u, DMA channels %u\n", sc->sc_unit, sc->sc_pair);

	esp_i2s_init(sc);

	if (fdtbus_intr_establish_xname(phandle, 0, IPL_AUDIO,
	    FDT_INTR_MPSAFE, esp_i2s_intr_out, sc, device_xname(self)) == NULL ||
	    fdtbus_intr_establish_xname(phandle, 1, IPL_AUDIO,
	    FDT_INTR_MPSAFE, esp_i2s_intr_in, sc, device_xname(self)) == NULL) {
		aprint_error_dev(self, "couldn't establish interrupts\n");
		return;
	}
	aprint_normal_dev(self, "interrupting on %s (out), %s (in)\n",
	    intrstr[0], intrstr[1]);

	sc->sc_format.mode = AUMODE_PLAY | AUMODE_RECORD;
	sc->sc_format.encoding = AUDIO_ENCODING_SLINEAR_LE;
	sc->sc_format.validbits = 16;
	sc->sc_format.precision = 16;
	sc->sc_format.channels = 2;
	sc->sc_format.channel_mask = AUFMT_STEREO;
	sc->sc_format.frequency_type = __arraycount(esp_i2s_rates);
	for (u_int i = 0; i < __arraycount(esp_i2s_rates); i++)
		sc->sc_format.frequency[i] = esp_i2s_rates[i];

	sc->sc_dai.dai_set_format = esp_i2s_dai_set_format;
	sc->sc_dai.dai_hw_if = &esp_i2s_hw_if;
	sc->sc_dai.dai_dev = self;
	sc->sc_dai.dai_priv = sc;
	fdtbus_register_dai_controller(self, phandle, &esp_i2s_dai_funcs);
}
