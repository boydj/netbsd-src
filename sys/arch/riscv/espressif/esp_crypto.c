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
 * ESP32-S31 crypto accelerators.
 *
 * - AES (128 and 256-bit keys; the engine has no 192) becomes the
 *   kernel's AES implementation (aes(9): cgd, IPsec through swcrypto,
 *   ...).  Without DMA the engine only does single blocks, so CBC, XTS,
 *   CBC-MAC and CCM chain them here.
 * - SHA-1/256/384/512 serve opencrypto(9) sessions for SHA-1 and
 *   HMAC-SHA-1/256/384/512 (IPsec, /dev/crypto).
 * - The RSA (big number) engine does modular exponentiation up to 4096
 *   bits for /dev/crypto (CRK_MOD_EXP), in constant time.
 *
 * The ECC engine is left to the radio co-processor's Bluetooth
 * controller, which uses it for Secure Simple Pairing; the co-processor
 * firmware is built without mbedTLS hardware acceleration, so the other
 * engines are ours alone.  Each engine has a lock; AES and SHA work in
 * short critical sections (at most 16 blocks), RSA sleeps.
 *
 *	crypto@20508000 {
 *		compatible = "espressif,esp32s31-crypto";
 *		reg = <0x20508000 0x3000>;	(AES, SHA, RSA)
 *		clocks = <&clkrst 25>, <&clkrst 26>, <&clkrst 27>,
 *			 <&clkrst 28>;
 *		clock-names = "crypto", "aes", "sha", "rsa";
 *		resets = <&clkrst 17>, <&clkrst 30>, <&clkrst 18>,
 *			 <&clkrst 19>, <&clkrst 20>, <&clkrst 31>,
 *			 <&clkrst 32>, <&clkrst 33>;
 *		reset-names = "crypto", "sec", "aes", "sha", "rsa", "ds",
 *			      "hmac", "ecdsa";
 *	};
 */


#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/evcnt.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/systm.h>
#include <sys/uio.h>

#include <crypto/aes/aes.h>
#include <crypto/aes/aes_bear.h>
#include <crypto/aes/aes_impl.h>

#include <opencrypto/cryptodev.h>
#include <opencrypto/xform.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdtvar.h>

/* AES, offsets from the block base (0x20508000) */
#define	AES_KEY(i)		(0x000 + 4 * (i))
#define	AES_TEXT_IN(i)		(0x020 + 4 * (i))
#define	AES_TEXT_OUT(i)		(0x030 + 4 * (i))
#define	AES_MODE		0x040
#define	 AES_MODE_DEC		4
#define	 AES_MODE_256		2
#define	AES_TRIGGER		0x048
#define	AES_STATE		0x04c
#define	AES_DMA_ENABLE		0x090

/* SHA (0x20509000) */
#define	SHA_BASE		0x1000
#define	SHA_MODE		(SHA_BASE + 0x00)
#define	 SHA_MODE_SHA1		0
#define	 SHA_MODE_SHA256	2
#define	 SHA_MODE_SHA384	3
#define	 SHA_MODE_SHA512	4
#define	SHA_START		(SHA_BASE + 0x10)
#define	SHA_CONTINUE		(SHA_BASE + 0x14)
#define	SHA_BUSY		(SHA_BASE + 0x18)
#define	SHA_H_MEM(i)		(SHA_BASE + 0x40 + 4 * (i))
#define	SHA_M_MEM(i)		(SHA_BASE + 0x80 + 4 * (i))

/* RSA (0x2050a000) */
#define	RSA_BASE		0x2000
#define	RSA_M_MEM		(RSA_BASE + 0x000)
#define	RSA_Z_MEM		(RSA_BASE + 0x200)
#define	RSA_Y_MEM		(RSA_BASE + 0x400)
#define	RSA_X_MEM		(RSA_BASE + 0x600)
#define	RSA_M_PRIME		(RSA_BASE + 0x800)
#define	RSA_MODE		(RSA_BASE + 0x804)
#define	RSA_QUERY_CLEAN		(RSA_BASE + 0x808)
#define	RSA_SET_START_MODEXP	(RSA_BASE + 0x80c)
#define	RSA_QUERY_IDLE		(RSA_BASE + 0x818)
#define	RSA_INT_CLR		(RSA_BASE + 0x81c)
#define	RSA_CONSTANT_TIME	(RSA_BASE + 0x820)
#define	RSA_SEARCH_ENABLE	(RSA_BASE + 0x824)
#define	RSA_INT_ENA		(RSA_BASE + 0x82c)
#define	RSA_MAX_WORDS		128		/* 4096 bits */

#define	ESP_AES_BATCH		16	/* blocks per critical section */
#define	ESP_HASH_MAXBLOCK	128
#define	ESP_HASH_MAXSTATE	16	/* words */
#define	ESP_HASH_MAXDIGEST	64

struct esp_crypto_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	kmutex_t		sc_aes_lock;
	kmutex_t		sc_sha_lock;
	kmutex_t		sc_rsa_lock;
	bool			sc_aes_ok, sc_sha_ok, sc_rsa_ok;
	int32_t			sc_cid;
	kmutex_t		sc_ses_lock;
	struct esp_crypto_session **sc_sessions;
	u_int			sc_nsessions;
	struct evcnt		sc_ev_hash;	/* opencrypto requests */
	struct evcnt		sc_ev_modexp;
};

#define	RD(sc, r)	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR(sc, r, v)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static struct esp_crypto_softc *esp_crypto_sc;

static int	esp_crypto_match(device_t, cfdata_t, void *);
static void	esp_crypto_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_crypto, sizeof(struct esp_crypto_softc),
	esp_crypto_match, esp_crypto_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-crypto" },
	DEVICE_COMPAT_EOL
};

/*
 * AES
 */

/*
 * Run nblocks blocks through the engine, one at a time, each through
 * fn's pre/post processing (chaining).  The key (raw, little-endian
 * words, in the first words of the key schedule) is loaded once per
 * batch.
 */
static void
esp_aes_load(struct esp_crypto_softc *sc, const uint32_t *rk,
    uint32_t nrounds, bool dec)
{
	const u_int nk = nrounds == AES_256_NROUNDS ? 8 : 4;

	WR(sc, AES_MODE, (dec ? AES_MODE_DEC : 0) |
	    (nrounds == AES_256_NROUNDS ? AES_MODE_256 : 0));
	for (u_int i = 0; i < nk; i++)
		WR(sc, AES_KEY(i), rk[i]);
}

static void
esp_aes_crypt1(struct esp_crypto_softc *sc, const uint8_t in[static 16],
    uint8_t out[static 16])
{
	for (u_int i = 0; i < 4; i++)
		WR(sc, AES_TEXT_IN(i), le32dec(in + 4 * i));
	WR(sc, AES_TRIGGER, 1);
	while (RD(sc, AES_STATE) != 0)
		continue;
	for (u_int i = 0; i < 4; i++)
		le32enc(out + 4 * i, RD(sc, AES_TEXT_OUT(i)));
}

static bool
esp_aes_hw(uint32_t nrounds)
{
	return nrounds == AES_128_NROUNDS || nrounds == AES_256_NROUNDS;
}

static void
esp_aes_setkey(union aes *aes, const uint8_t *key, uint32_t nrounds)
{
	const u_int nk = nrounds == AES_256_NROUNDS ? 8 : 4;

	memset(aes, 0, sizeof(*aes));
	for (u_int i = 0; i < nk; i++)
		aes->aes_rk[i] = le32dec(key + 4 * i);
}

static void
esp_aes_setenckey(struct aesenc *enc, const uint8_t *key, uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_setenckey(enc, key, nrounds);
		return;
	}
	esp_aes_setkey(&enc->aese_aes, key, nrounds);
}

static void
esp_aes_setdeckey(struct aesdec *dec, const uint8_t *key, uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_setdeckey(dec, key, nrounds);
		return;
	}
	esp_aes_setkey(&dec->aesd_aes, key, nrounds);
}

/* Encrypt or decrypt one block. */
static void
esp_aes_block(const union aes *aes, uint32_t nrounds, bool dec,
    const uint8_t in[static 16], uint8_t out[static 16])
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;

	mutex_spin_enter(&sc->sc_aes_lock);
	esp_aes_load(sc, aes->aes_rk, nrounds, dec);
	esp_aes_crypt1(sc, in, out);
	mutex_spin_exit(&sc->sc_aes_lock);
}

static void
esp_aes_enc(const struct aesenc *enc, const uint8_t in[static 16],
    uint8_t out[static 16], uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_enc(enc, in, out, nrounds);
		return;
	}
	esp_aes_block(&enc->aese_aes, nrounds, false, in, out);
}

static void
esp_aes_dec(const struct aesdec *dec, const uint8_t in[static 16],
    uint8_t out[static 16], uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_dec(dec, in, out, nrounds);
		return;
	}
	esp_aes_block(&dec->aesd_aes, nrounds, true, in, out);
}

static void
esp_xor16(uint8_t *x, const uint8_t *a, const uint8_t *b)
{
	for (u_int i = 0; i < 16; i++)
		x[i] = a[i] ^ b[i];
}

static void
esp_aes_cbc_enc(const struct aesenc *enc, const uint8_t in[static 16],
    uint8_t out[static 16], size_t nbytes, uint8_t iv[static 16],
    uint32_t nrounds)
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;
	uint8_t x[16];

	KASSERT(nbytes % 16 == 0);
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_cbc_enc(enc, in, out, nbytes, iv, nrounds);
		return;
	}
	while (nbytes > 0) {
		mutex_spin_enter(&sc->sc_aes_lock);
		esp_aes_load(sc, enc->aese_aes.aes_rk, nrounds, false);
		for (u_int n = 0; n < ESP_AES_BATCH && nbytes > 0;
		    n++, nbytes -= 16, in += 16, out += 16) {
			esp_xor16(x, in, iv);
			esp_aes_crypt1(sc, x, out);
			memcpy(iv, out, 16);
		}
		mutex_spin_exit(&sc->sc_aes_lock);
	}
	explicit_memset(x, 0, sizeof(x));
}

static void
esp_aes_cbc_dec(const struct aesdec *dec, const uint8_t in[static 16],
    uint8_t out[static 16], size_t nbytes, uint8_t iv[static 16],
    uint32_t nrounds)
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;
	uint8_t c[16], x[16];

	KASSERT(nbytes % 16 == 0);
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_cbc_dec(dec, in, out, nbytes, iv, nrounds);
		return;
	}
	while (nbytes > 0) {
		mutex_spin_enter(&sc->sc_aes_lock);
		esp_aes_load(sc, dec->aesd_aes.aes_rk, nrounds, true);
		for (u_int n = 0; n < ESP_AES_BATCH && nbytes > 0;
		    n++, nbytes -= 16, in += 16, out += 16) {
			memcpy(c, in, 16);	/* in may be out */
			esp_aes_crypt1(sc, c, x);
			esp_xor16(out, x, iv);
			memcpy(iv, c, 16);
		}
		mutex_spin_exit(&sc->sc_aes_lock);
	}
	explicit_memset(c, 0, sizeof(c));
	explicit_memset(x, 0, sizeof(x));
}

/* Multiply the XTS tweak by x in GF(2^128), little-endian. */
static void
esp_xts_update(uint8_t t[static 16])
{
	uint32_t t0 = le32dec(t), t1 = le32dec(t + 4), t2 = le32dec(t + 8),
	    t3 = le32dec(t + 12);
	const uint32_t s0 = t0 >> 31, s1 = t1 >> 31, s2 = t2 >> 31,
	    s3 = t3 >> 31;

	t0 = (t0 << 1) ^ (-s3 & 0x87);
	t1 = (t1 << 1) ^ s0;
	t2 = (t2 << 1) ^ s1;
	t3 = (t3 << 1) ^ s2;
	le32enc(t, t0);
	le32enc(t + 4, t1);
	le32enc(t + 8, t2);
	le32enc(t + 12, t3);
}

static void
esp_aes_xts(const union aes *aes, const uint8_t *in, uint8_t *out,
    size_t nbytes, uint8_t tweak[static 16], uint32_t nrounds, bool dec)
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;
	uint8_t x[16];

	KASSERT(nbytes % 16 == 0);
	while (nbytes > 0) {
		mutex_spin_enter(&sc->sc_aes_lock);
		esp_aes_load(sc, aes->aes_rk, nrounds, dec);
		for (u_int n = 0; n < ESP_AES_BATCH && nbytes > 0;
		    n++, nbytes -= 16, in += 16, out += 16) {
			esp_xor16(x, in, tweak);
			esp_aes_crypt1(sc, x, x);
			esp_xor16(out, x, tweak);
			esp_xts_update(tweak);
		}
		mutex_spin_exit(&sc->sc_aes_lock);
	}
	explicit_memset(x, 0, sizeof(x));
}

static void
esp_aes_xts_enc(const struct aesenc *enc, const uint8_t in[static 16],
    uint8_t out[static 16], size_t nbytes, uint8_t tweak[static 16],
    uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_xts_enc(enc, in, out, nbytes, tweak, nrounds);
		return;
	}
	esp_aes_xts(&enc->aese_aes, in, out, nbytes, tweak, nrounds, false);
}

static void
esp_aes_xts_dec(const struct aesdec *dec, const uint8_t in[static 16],
    uint8_t out[static 16], size_t nbytes, uint8_t tweak[static 16],
    uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_xts_dec(dec, in, out, nbytes, tweak, nrounds);
		return;
	}
	esp_aes_xts(&dec->aesd_aes, in, out, nbytes, tweak, nrounds, true);
}

static void
esp_aes_cbcmac_update1(const struct aesenc *enc, const uint8_t in[static 16],
    size_t nbytes, uint8_t auth[static 16], uint32_t nrounds)
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;

	KASSERT(nbytes % 16 == 0);
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_cbcmac_update1(enc, in, nbytes, auth,
		    nrounds);
		return;
	}
	while (nbytes > 0) {
		mutex_spin_enter(&sc->sc_aes_lock);
		esp_aes_load(sc, enc->aese_aes.aes_rk, nrounds, false);
		for (u_int n = 0; n < ESP_AES_BATCH && nbytes > 0;
		    n++, nbytes -= 16, in += 16) {
			esp_xor16(auth, auth, in);
			esp_aes_crypt1(sc, auth, auth);
		}
		mutex_spin_exit(&sc->sc_aes_lock);
	}
}

/* CCM: authctr is the authenticator, then the counter block. */
static void
esp_aes_ccm(const struct aesenc *enc, const uint8_t *in, uint8_t *out,
    size_t nbytes, uint8_t authctr[static 32], uint32_t nrounds, bool dec)
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;
	uint8_t * const auth = authctr, * const ctr = authctr + 16;
	uint8_t pad[16], p[16];

	KASSERT(nbytes % 16 == 0);
	while (nbytes > 0) {
		mutex_spin_enter(&sc->sc_aes_lock);
		esp_aes_load(sc, enc->aese_aes.aes_rk, nrounds, false);
		for (u_int n = 0; n < ESP_AES_BATCH && nbytes > 0;
		    n++, nbytes -= 16, in += 16, out += 16) {
			be32enc(ctr + 12, be32dec(ctr + 12) + 1);
			esp_aes_crypt1(sc, ctr, pad);
			if (dec) {
				esp_xor16(p, in, pad);
				memcpy(out, p, 16);
			} else {
				memcpy(p, in, 16);
				esp_xor16(out, in, pad);
			}
			esp_xor16(auth, auth, p);
			esp_aes_crypt1(sc, auth, auth);
		}
		mutex_spin_exit(&sc->sc_aes_lock);
	}
	explicit_memset(pad, 0, sizeof(pad));
	explicit_memset(p, 0, sizeof(p));
}

static void
esp_aes_ccm_enc1(const struct aesenc *enc, const uint8_t in[static 16],
    uint8_t out[static 16], size_t nbytes, uint8_t authctr[static 32],
    uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_ccm_enc1(enc, in, out, nbytes, authctr,
		    nrounds);
		return;
	}
	esp_aes_ccm(enc, in, out, nbytes, authctr, nrounds, false);
}

static void
esp_aes_ccm_dec1(const struct aesenc *enc, const uint8_t in[static 16],
    uint8_t out[static 16], size_t nbytes, uint8_t authctr[static 32],
    uint32_t nrounds)
{
	if (!esp_aes_hw(nrounds)) {
		aes_bear_impl.ai_ccm_dec1(enc, in, out, nbytes, authctr,
		    nrounds);
		return;
	}
	esp_aes_ccm(enc, in, out, nbytes, authctr, nrounds, true);
}

static int
esp_aes_probe(void)
{
	return esp_crypto_sc != NULL && esp_crypto_sc->sc_aes_ok ? 0 : -1;
}

static const struct aes_impl esp_aes_impl = {
	.ai_name = "ESP32-S31 AES",
	.ai_probe = esp_aes_probe,
	.ai_setenckey = esp_aes_setenckey,
	.ai_setdeckey = esp_aes_setdeckey,
	.ai_enc = esp_aes_enc,
	.ai_dec = esp_aes_dec,
	.ai_cbc_enc = esp_aes_cbc_enc,
	.ai_cbc_dec = esp_aes_cbc_dec,
	.ai_xts_enc = esp_aes_xts_enc,
	.ai_xts_dec = esp_aes_xts_dec,
	.ai_cbcmac_update1 = esp_aes_cbcmac_update1,
	.ai_ccm_enc1 = esp_aes_ccm_enc1,
	.ai_ccm_dec1 = esp_aes_ccm_dec1,
};

/*
 * SHA: the engine hashes blocks, from its own initial value (START) or
 * from a state written back to it (CONTINUE); padding is done here.
 */

struct esp_hash_alg {
	u_int	ha_mode;
	u_int	ha_bsize;	/* block, bytes */
	u_int	ha_nstate;	/* state, words */
	u_int	ha_dsize;	/* digest, bytes */
};

static const struct esp_hash_alg esp_sha1 = { SHA_MODE_SHA1, 64, 5, 20 };
static const struct esp_hash_alg esp_sha256 = { SHA_MODE_SHA256, 64, 8, 32 };
static const struct esp_hash_alg esp_sha384 = { SHA_MODE_SHA384, 128, 16, 48 };
static const struct esp_hash_alg esp_sha512 = { SHA_MODE_SHA512, 128, 16, 64 };

struct esp_hash_ctx {
	const struct esp_hash_alg *hc_alg;
	bool		hc_started;
	uint32_t	hc_state[ESP_HASH_MAXSTATE];
	uint8_t		hc_buf[ESP_HASH_MAXBLOCK];
	u_int		hc_buflen;
	uint64_t	hc_total;
};

static void
esp_sha_wait(struct esp_crypto_softc *sc)
{
	while (RD(sc, SHA_BUSY) & 1)
		continue;
}

/* Hash whole blocks (data 4-byte aligned or not). */
static void
esp_sha_blocks(struct esp_hash_ctx *hc, const uint8_t *data, size_t nblocks)
{
	struct esp_crypto_softc * const sc = esp_crypto_sc;
	const struct esp_hash_alg * const ha = hc->hc_alg;

	while (nblocks > 0) {
		mutex_spin_enter(&sc->sc_sha_lock);
		esp_sha_wait(sc);
		WR(sc, SHA_MODE, ha->ha_mode);
		if (hc->hc_started) {
			for (u_int i = 0; i < ha->ha_nstate; i++)
				WR(sc, SHA_H_MEM(i), hc->hc_state[i]);
		}
		for (u_int n = 0; n < ESP_AES_BATCH && nblocks > 0;
		    n++, nblocks--, data += ha->ha_bsize) {
			esp_sha_wait(sc);
			for (u_int i = 0; i < ha->ha_bsize / 4; i++)
				WR(sc, SHA_M_MEM(i), le32dec(data + 4 * i));
			WR(sc, hc->hc_started ? SHA_CONTINUE : SHA_START, 1);
			hc->hc_started = true;
		}
		esp_sha_wait(sc);
		for (u_int i = 0; i < ha->ha_nstate; i++)
			hc->hc_state[i] = RD(sc, SHA_H_MEM(i));
		mutex_spin_exit(&sc->sc_sha_lock);
	}
}

static void
esp_hash_init(struct esp_hash_ctx *hc, const struct esp_hash_alg *ha)
{
	memset(hc, 0, sizeof(*hc));
	hc->hc_alg = ha;
}

static int
esp_hash_update(void *arg, void *vbuf, unsigned int len)
{
	struct esp_hash_ctx * const hc = arg;
	const u_int bs = hc->hc_alg->ha_bsize;
	const uint8_t *buf = vbuf;

	hc->hc_total += len;
	if (hc->hc_buflen > 0) {
		const u_int n = MIN(len, bs - hc->hc_buflen);

		memcpy(hc->hc_buf + hc->hc_buflen, buf, n);
		hc->hc_buflen += n;
		buf += n;
		len -= n;
		if (hc->hc_buflen < bs)
			return 0;
		esp_sha_blocks(hc, hc->hc_buf, 1);
		hc->hc_buflen = 0;
	}
	if (len >= bs) {
		esp_sha_blocks(hc, buf, len / bs);
		buf += len - len % bs;
		len %= bs;
	}
	memcpy(hc->hc_buf, buf, len);
	hc->hc_buflen = len;
	return 0;
}

static void
esp_hash_final(struct esp_hash_ctx *hc, uint8_t *digest)
{
	const u_int bs = hc->hc_alg->ha_bsize;
	const u_int lenbytes = bs == 128 ? 16 : 8;
	const uint64_t bits = hc->hc_total * 8;
	uint8_t d[ESP_HASH_MAXSTATE * 4];

	hc->hc_buf[hc->hc_buflen++] = 0x80;
	if (hc->hc_buflen > bs - lenbytes) {
		memset(hc->hc_buf + hc->hc_buflen, 0, bs - hc->hc_buflen);
		esp_sha_blocks(hc, hc->hc_buf, 1);
		hc->hc_buflen = 0;
	}
	memset(hc->hc_buf + hc->hc_buflen, 0, bs - hc->hc_buflen);
	be64enc(hc->hc_buf + bs - 8, bits);
	esp_sha_blocks(hc, hc->hc_buf, 1);

	for (u_int i = 0; i < hc->hc_alg->ha_nstate; i++)
		le32enc(d + 4 * i, hc->hc_state[i]);
	memcpy(digest, d, hc->hc_alg->ha_dsize);
	explicit_memset(d, 0, sizeof(d));
	explicit_memset(hc, 0, sizeof(*hc));
}

/* HMAC contexts after hashing the padded key: inner and outer. */
struct esp_hmac {
	struct esp_hash_ctx	hm_ictx;
	struct esp_hash_ctx	hm_octx;
};

static void
esp_hmac_init(struct esp_hmac *hm, const struct esp_hash_alg *ha,
    const uint8_t *key, size_t keylen)
{
	uint8_t k[ESP_HASH_MAXBLOCK], pad[ESP_HASH_MAXBLOCK];
	struct esp_hash_ctx hc;

	memset(k, 0, sizeof(k));
	if (keylen > ha->ha_bsize) {
		esp_hash_init(&hc, ha);
		esp_hash_update(&hc, __UNCONST(key), keylen);
		esp_hash_final(&hc, k);
	} else {
		memcpy(k, key, keylen);
	}
	for (u_int i = 0; i < ha->ha_bsize; i++)
		pad[i] = k[i] ^ 0x36;
	esp_hash_init(&hm->hm_ictx, ha);
	esp_hash_update(&hm->hm_ictx, pad, ha->ha_bsize);
	for (u_int i = 0; i < ha->ha_bsize; i++)
		pad[i] = k[i] ^ 0x5c;
	esp_hash_init(&hm->hm_octx, ha);
	esp_hash_update(&hm->hm_octx, pad, ha->ha_bsize);
	explicit_memset(k, 0, sizeof(k));
	explicit_memset(pad, 0, sizeof(pad));
}

/*
 * RSA: Z = X^Y mod M by the big number engine (Montgomery, with
 * R = 2^(32n)), little-endian 32-bit words.  Rinv = R^2 mod M and
 * M' = -M^-1 mod 2^32 are computed here.
 */

static int
bn_cmp(const uint32_t *a, const uint32_t *b, u_int n)
{
	while (n-- > 0) {
		if (a[n] != b[n])
			return a[n] < b[n] ? -1 : 1;
	}
	return 0;
}

static void
bn_sub(uint32_t *a, const uint32_t *b, u_int n)
{
	uint64_t borrow = 0;

	for (u_int i = 0; i < n; i++) {
		const uint64_t d = (uint64_t)a[i] - b[i] - borrow;

		a[i] = (uint32_t)d;
		borrow = (d >> 63) & 1;
	}
}

/* a = 2a + bit; returns the bit shifted out. */
static uint32_t
bn_shl1(uint32_t *a, u_int n, uint32_t bit)
{
	for (u_int i = 0; i < n; i++) {
		const uint32_t c = a[i] >> 31;

		a[i] = (a[i] << 1) | bit;
		bit = c;
	}
	return bit;
}

/* r = x mod m, x of nx words, r and m of n words (m nonzero, top set). */
static void
bn_mod(uint32_t *r, const uint32_t *x, u_int nx, const uint32_t *m, u_int n)
{
	memset(r, 0, n * 4);
	for (int i = nx * 32 - 1; i >= 0; i--) {
		const uint32_t bit = (x[i / 32] >> (i % 32)) & 1;

		if (bn_shl1(r, n, bit) || bn_cmp(r, m, n) >= 0)
			bn_sub(r, m, n);
	}
}

static u_int
bn_words(const uint32_t *a, u_int n)
{
	while (n > 0 && a[n - 1] == 0)
		n--;
	return n;
}

static int
esp_rsa_modexp(struct esp_crypto_softc *sc, uint32_t *z, const uint32_t *x,
    u_int nx, const uint32_t *y, u_int ny, const uint32_t *m, u_int nm)
{
	uint32_t *xr, *rinv, *one, inv;
	u_int n, guard;
	int error = 0;

	nm = bn_words(m, nm);
	ny = bn_words(y, ny);
	if (nm == 0 || (m[0] & 1) == 0)
		return EINVAL;
	n = MAX(nm, ny);
	if (n > RSA_MAX_WORDS)
		return E2BIG;

	xr = kmem_zalloc(n * 4, KM_SLEEP);
	rinv = kmem_zalloc(n * 4, KM_SLEEP);
	one = kmem_zalloc(n * 4, KM_SLEEP);

	/* X mod M, and R^2 mod M = (2^(64n)) mod M */
	{
		uint32_t *mm = kmem_zalloc(n * 4, KM_SLEEP);

		memcpy(mm, m, nm * 4);
		bn_mod(xr, x, nx, mm, n);
		one[0] = 1;
		memcpy(rinv, one, n * 4);
		for (u_int i = 0; i < 64 * n; i++) {
			if (bn_shl1(rinv, n, 0) || bn_cmp(rinv, mm, n) >= 0)
				bn_sub(rinv, mm, n);
		}
		kmem_free(mm, n * 4);
	}
	/* M' = -M^-1 mod 2^32, by Newton's iteration */
	inv = m[0];
	for (u_int i = 0; i < 5; i++)
		inv *= 2 - m[0] * inv;

	mutex_enter(&sc->sc_rsa_lock);
	WR(sc, RSA_MODE, n - 1);
	for (u_int i = 0; i < n; i++) {
		WR(sc, RSA_X_MEM + 4 * i, xr[i]);
		WR(sc, RSA_Y_MEM + 4 * i, i < ny ? y[i] : 0);
		WR(sc, RSA_M_MEM + 4 * i, i < nm ? m[i] : 0);
		WR(sc, RSA_Z_MEM + 4 * i, rinv[i]);
	}
	WR(sc, RSA_M_PRIME, -inv);
	WR(sc, RSA_CONSTANT_TIME, 1);
	WR(sc, RSA_SEARCH_ENABLE, 0);
	WR(sc, RSA_INT_CLR, 1);
	WR(sc, RSA_SET_START_MODEXP, 1);
	for (guard = 0; (RD(sc, RSA_QUERY_IDLE) & 1) == 0; guard++) {
		if (guard > 2000000) {		/* ~ 20 s */
			error = ETIMEDOUT;
			break;
		}
		DELAY(10);
		if ((guard & 0xff) == 0)
			preempt_point();
	}
	WR(sc, RSA_INT_CLR, 1);
	if (error == 0) {
		for (u_int i = 0; i < nm; i++)
			z[i] = RD(sc, RSA_Z_MEM + 4 * i);
	}
	mutex_exit(&sc->sc_rsa_lock);

	explicit_memset(xr, 0, n * 4);
	kmem_free(xr, n * 4);
	kmem_free(rinv, n * 4);
	kmem_free(one, n * 4);
	return error;
}

/*
 * opencrypto
 */

struct esp_crypto_session {
	int			ses_alg;
	const struct esp_hash_alg *ses_hash;
	const struct auth_hash	*ses_axf;
	struct esp_hmac		ses_hmac;	/* HMAC: precomputed */
};

static int
esp_crypto_newsession(void *arg, uint32_t *sidp, struct cryptoini *cri)
{
	struct esp_crypto_softc * const sc = arg;
	struct esp_crypto_session *ses, **sessions;
	const struct esp_hash_alg *ha;
	const struct auth_hash *axf;
	u_int i;

	if (cri == NULL || cri->cri_next != NULL)
		return EINVAL;		/* one algorithm a session */

	switch (cri->cri_alg) {
	case CRYPTO_SHA1:
		ha = &esp_sha1; axf = &auth_hash_sha1; break;
	case CRYPTO_SHA1_HMAC:
		ha = &esp_sha1; axf = &auth_hash_hmac_sha1; break;
	case CRYPTO_SHA2_256_HMAC:
		ha = &esp_sha256; axf = &auth_hash_hmac_sha2_256; break;
	case CRYPTO_SHA2_384_HMAC:
		ha = &esp_sha384; axf = &auth_hash_hmac_sha2_384; break;
	case CRYPTO_SHA2_512_HMAC:
		ha = &esp_sha512; axf = &auth_hash_hmac_sha2_512; break;
	default:
		return EINVAL;
	}

	ses = kmem_zalloc(sizeof(*ses), KM_SLEEP);
	ses->ses_alg = cri->cri_alg;
	ses->ses_hash = ha;
	ses->ses_axf = axf;
	if (cri->cri_alg != CRYPTO_SHA1) {
		if (cri->cri_key == NULL) {
			kmem_free(ses, sizeof(*ses));
			return EINVAL;
		}
		esp_hmac_init(&ses->ses_hmac, ha, cri->cri_key,
		    cri->cri_klen / 8);
	}

	mutex_enter(&sc->sc_ses_lock);
	for (i = 0; i < sc->sc_nsessions; i++)
		if (sc->sc_sessions[i] == NULL)
			break;
	if (i == sc->sc_nsessions) {
		const u_int nn = MAX(8, sc->sc_nsessions * 2);

		mutex_exit(&sc->sc_ses_lock);
		sessions = kmem_zalloc(nn * sizeof(*sessions), KM_SLEEP);
		mutex_enter(&sc->sc_ses_lock);
		if (sc->sc_nsessions < nn) {
			if (sc->sc_sessions != NULL) {
				memcpy(sessions, sc->sc_sessions,
				    sc->sc_nsessions * sizeof(*sessions));
				kmem_free(sc->sc_sessions,
				    sc->sc_nsessions * sizeof(*sessions));
			}
			sc->sc_sessions = sessions;
			sc->sc_nsessions = nn;
		} else {
			kmem_free(sessions, nn * sizeof(*sessions));
		}
		for (i = 0; i < sc->sc_nsessions; i++)
			if (sc->sc_sessions[i] == NULL)
				break;
	}
	sc->sc_sessions[i] = ses;
	mutex_exit(&sc->sc_ses_lock);
	*sidp = i;
	return 0;
}

static void
esp_crypto_freesession(void *arg, uint64_t tid)
{
	struct esp_crypto_softc * const sc = arg;
	const u_int sid = CRYPTO_SESID2LID(tid);
	struct esp_crypto_session *ses = NULL;

	mutex_enter(&sc->sc_ses_lock);
	if (sid < sc->sc_nsessions) {
		ses = sc->sc_sessions[sid];
		sc->sc_sessions[sid] = NULL;
	}
	mutex_exit(&sc->sc_ses_lock);
	if (ses != NULL) {
		explicit_memset(ses, 0, sizeof(*ses));
		kmem_free(ses, sizeof(*ses));
	}
}

static int
esp_crypto_auth(struct cryptop *crp, struct cryptodesc *crd,
    const struct esp_crypto_session *ses)
{
	const struct esp_hash_alg * const ha = ses->ses_hash;
	const u_int alen = ses->ses_axf->authsize;
	struct esp_hash_ctx hc;
	uint8_t md[ESP_HASH_MAXDIGEST];
	int error = 0;

	if (ses->ses_alg == CRYPTO_SHA1)
		esp_hash_init(&hc, ha);
	else
		hc = ses->ses_hmac.hm_ictx;

	if (crp->crp_flags & CRYPTO_F_IMBUF)
		error = m_apply(crp->crp_buf, crd->crd_skip, crd->crd_len,
		    esp_hash_update, &hc);
	else if (crp->crp_flags & CRYPTO_F_IOV)
		error = cuio_apply(crp->crp_buf, crd->crd_skip, crd->crd_len,
		    esp_hash_update, &hc);
	else
		esp_hash_update(&hc, (uint8_t *)crp->crp_buf + crd->crd_skip,
		    crd->crd_len);
	if (error)
		return error;
	esp_hash_final(&hc, md);

	if (ses->ses_alg != CRYPTO_SHA1) {
		hc = ses->ses_hmac.hm_octx;
		esp_hash_update(&hc, md, ha->ha_dsize);
		esp_hash_final(&hc, md);
	}

	if (crp->crp_flags & CRYPTO_F_IMBUF)
		m_copyback(crp->crp_buf, crd->crd_inject, alen, md);
	else if ((crp->crp_flags & CRYPTO_F_IOV) && crp->crp_mac != NULL)
		memcpy(crp->crp_mac, md, alen);
	else if (crp->crp_flags & CRYPTO_F_IOV)
		cuio_copyback(crp->crp_buf, crd->crd_inject, alen, md);
	else
		memcpy((uint8_t *)crp->crp_buf + crd->crd_inject, md, alen);
	explicit_memset(md, 0, sizeof(md));
	return 0;
}

static int
esp_crypto_process(void *arg, struct cryptop *crp, int hint)
{
	struct esp_crypto_softc * const sc = arg;
	const u_int sid = CRYPTO_SESID2LID(crp->crp_sid);
	struct esp_crypto_session *ses = NULL;
	struct cryptodesc *crd;
	int error = 0;

	mutex_enter(&sc->sc_ses_lock);
	if (sid < sc->sc_nsessions)
		ses = sc->sc_sessions[sid];
	mutex_exit(&sc->sc_ses_lock);
	if (ses == NULL) {
		error = EINVAL;
		goto out;
	}
	sc->sc_ev_hash.ev_count++;
	for (crd = crp->crp_desc; crd != NULL && error == 0;
	    crd = crd->crd_next) {
		if (crd->crd_alg != ses->ses_alg) {
			error = EINVAL;
			break;
		}
		error = esp_crypto_auth(crp, crd, ses);
	}
out:
	crp->crp_etype = error;
	crypto_done(crp);
	return 0;
}

/* CRK_MOD_EXP: params base, exponent, modulus; result. */
static int
esp_crypto_kprocess(void *arg, struct cryptkop *krp, int hint)
{
	struct esp_crypto_softc * const sc = arg;
	struct crparam * const p = krp->krp_param;
	uint32_t *x = NULL, *y = NULL, *m = NULL, *z = NULL;
	u_int nx = 0, ny = 0, nm = 0, outbytes;
	int error = 0;

	if (krp->krp_op != CRK_MOD_EXP || krp->krp_iparams != 3 ||
	    krp->krp_oparams != 1) {
		error = EINVAL;
		goto out;
	}
	nx = howmany(p[0].crp_nbits, 32);
	ny = howmany(p[1].crp_nbits, 32);
	nm = howmany(p[2].crp_nbits, 32);
	outbytes = howmany(p[3].crp_nbits, 8);
	if (nm == 0 || nm > RSA_MAX_WORDS || ny > RSA_MAX_WORDS ||
	    nx > 2 * RSA_MAX_WORDS || outbytes < howmany(p[2].crp_nbits, 8)) {
		error = E2BIG;
		goto out;
	}
	x = kmem_zalloc(MAX(nx, 1) * 4, KM_SLEEP);
	y = kmem_zalloc(MAX(ny, 1) * 4, KM_SLEEP);
	m = kmem_zalloc(nm * 4, KM_SLEEP);
	z = kmem_zalloc(nm * 4, KM_SLEEP);
	/* Parameters are little-endian byte strings. */
	memcpy(x, p[0].crp_p, howmany(p[0].crp_nbits, 8));
	memcpy(y, p[1].crp_p, howmany(p[1].crp_nbits, 8));
	memcpy(m, p[2].crp_p, howmany(p[2].crp_nbits, 8));
	for (u_int i = 0; i < nx; i++)
		x[i] = le32toh(x[i]);
	for (u_int i = 0; i < ny; i++)
		y[i] = le32toh(y[i]);
	for (u_int i = 0; i < nm; i++)
		m[i] = le32toh(m[i]);

	sc->sc_ev_modexp.ev_count++;
	error = esp_rsa_modexp(sc, z, x, MAX(nx, 1), y, ny, m, nm);
	if (error == 0) {
		uint8_t *out = p[3].crp_p;

		memset(out, 0, outbytes);
		for (u_int i = 0; i < nm * 4 && i < outbytes; i++)
			out[i] = z[i / 4] >> (8 * (i % 4));
	}
out:
	if (x != NULL) {
		explicit_memset(x, 0, MAX(nx, 1) * 4);
		kmem_free(x, MAX(nx, 1) * 4);
		explicit_memset(y, 0, MAX(ny, 1) * 4);
		kmem_free(y, MAX(ny, 1) * 4);
		kmem_free(m, nm * 4);
		explicit_memset(z, 0, nm * 4);
		kmem_free(z, nm * 4);
	}
	krp->krp_status = error;
	crypto_kdone(krp);
	return 0;
}

/*
 * Bring-up and self-tests
 */

static bool
esp_crypto_test_aes(struct esp_crypto_softc *sc)
{
	static const uint8_t pt[16] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
	static const uint8_t ct128[16] = {	/* FIPS-197 C.1 */
		0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
		0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a };
	static const uint8_t ct256[16] = {	/* FIPS-197 C.3 */
		0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf,
		0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89 };
	uint8_t key[32], out[16];
	union aes aes;

	for (u_int i = 0; i < 32; i++)
		key[i] = i;
	WR(sc, AES_DMA_ENABLE, 0);

	esp_aes_setkey(&aes, key, AES_128_NROUNDS);
	esp_aes_block(&aes, AES_128_NROUNDS, false, pt, out);
	if (memcmp(out, ct128, 16) != 0)
		return false;
	esp_aes_block(&aes, AES_128_NROUNDS, true, ct128, out);
	if (memcmp(out, pt, 16) != 0)
		return false;
	esp_aes_setkey(&aes, key, AES_256_NROUNDS);
	esp_aes_block(&aes, AES_256_NROUNDS, false, pt, out);
	if (memcmp(out, ct256, 16) != 0)
		return false;
	esp_aes_block(&aes, AES_256_NROUNDS, true, ct256, out);
	return memcmp(out, pt, 16) == 0;
}

static bool
esp_crypto_test_sha(struct esp_crypto_softc *sc)
{
	static const uint8_t sha1_abc[20] = {
		0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a, 0xba, 0x3e,
		0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c, 0x9c, 0xd0, 0xd8, 0x9d };
	static const uint8_t sha256_abc[32] = {
		0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
		0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
		0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
		0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad };
	static const uint8_t sha512_abc[64] = {
		0xdd, 0xaf, 0x35, 0xa1, 0x93, 0x61, 0x7a, 0xba,
		0xcc, 0x41, 0x73, 0x49, 0xae, 0x20, 0x41, 0x31,
		0x12, 0xe6, 0xfa, 0x4e, 0x89, 0xa9, 0x7e, 0xa2,
		0x0a, 0x9e, 0xee, 0xe6, 0x4b, 0x55, 0xd3, 0x9a,
		0x21, 0x92, 0x99, 0x2a, 0x27, 0x4f, 0xc1, 0xa8,
		0x36, 0xba, 0x3c, 0x23, 0xa3, 0xfe, 0xeb, 0xbd,
		0x45, 0x4d, 0x44, 0x23, 0x64, 0x3c, 0xe8, 0x0e,
		0x2a, 0x9a, 0xc9, 0x4f, 0xa5, 0x4c, 0xa4, 0x9f };
	/* SHA-256 of 1000 'a's: several blocks, and a padding block. */
	static const uint8_t sha256_a1000[32] = {
		0x41, 0xed, 0xec, 0xe4, 0x2d, 0x63, 0xe8, 0xd9,
		0xbf, 0x51, 0x5a, 0x9b, 0xa6, 0x93, 0x2e, 0x1c,
		0x20, 0xcb, 0xc9, 0xf5, 0xa5, 0xd1, 0x34, 0x64,
		0x5a, 0xdb, 0x5d, 0xb1, 0xb9, 0x73, 0x7e, 0xa3 };
	struct esp_hash_ctx hc;
	uint8_t md[64], a[100];

	esp_hash_init(&hc, &esp_sha1);
	esp_hash_update(&hc, __UNCONST("abc"), 3);
	esp_hash_final(&hc, md);
	if (memcmp(md, sha1_abc, 20) != 0)
		return false;
	esp_hash_init(&hc, &esp_sha256);
	esp_hash_update(&hc, __UNCONST("abc"), 3);
	esp_hash_final(&hc, md);
	if (memcmp(md, sha256_abc, 32) != 0)
		return false;
	esp_hash_init(&hc, &esp_sha512);
	esp_hash_update(&hc, __UNCONST("abc"), 3);
	esp_hash_final(&hc, md);
	if (memcmp(md, sha512_abc, 64) != 0)
		return false;
	memset(a, 'a', sizeof(a));
	esp_hash_init(&hc, &esp_sha256);
	for (u_int i = 0; i < 10; i++)
		esp_hash_update(&hc, a, sizeof(a));
	esp_hash_final(&hc, md);
	return memcmp(md, sha256_a1000, 32) == 0;
}

static bool
esp_crypto_test_rsa(struct esp_crypto_softc *sc)
{
	/* 4^13 mod 497 = 445 */
	static const uint32_t m1[1] = { 497 }, x1[1] = { 4 }, y1[1] = { 13 };
	/* Fermat: 2^(p-1) mod p = 1, p = 2^64 - 59 */
	static const uint32_t m2[2] = { 0xffffffc5, 0xffffffff };
	static const uint32_t y2[2] = { 0xffffffc4, 0xffffffff };
	static const uint32_t x2[1] = { 2 };
	uint32_t z[2];

	if (esp_rsa_modexp(sc, z, x1, 1, y1, 1, m1, 1) != 0 || z[0] != 445)
		return false;
	if (esp_rsa_modexp(sc, z, x2, 1, y2, 2, m2, 2) != 0 ||
	    z[0] != 1 || z[1] != 0)
		return false;
	return true;
}

static void
esp_crypto_reset(int phandle, const char *name, bool pulse)
{
	struct fdtbus_reset *rst = fdtbus_reset_get(phandle, name);

	if (rst == NULL)
		return;
	if (pulse)
		fdtbus_reset_assert(rst);
	fdtbus_reset_deassert(rst);
}

static int
esp_crypto_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_crypto_attach(device_t parent, device_t self, void *aux)
{
	struct esp_crypto_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	static const char * const clocks[] = { "crypto", "aes", "sha", "rsa" };
	bus_addr_t addr;
	bus_size_t size;
	u_int guard;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	for (u_int i = 0; i < __arraycount(clocks); i++) {
		struct clk * const clk = fdtbus_clock_get(phandle, clocks[i]);

		if (clk == NULL || clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock %s\n",
			    clocks[i]);
			return;
		}
	}
	/*
	 * Out of reset; the engines also need DS, HMAC and ECDSA out of
	 * reset, but resetting those would reset them all (and the ECC
	 * engine, the co-processor's), so they're only released.
	 */
	esp_crypto_reset(phandle, "crypto", false);
	esp_crypto_reset(phandle, "sec", false);
	esp_crypto_reset(phandle, "ds", false);
	esp_crypto_reset(phandle, "hmac", false);
	esp_crypto_reset(phandle, "ecdsa", false);
	esp_crypto_reset(phandle, "aes", true);
	esp_crypto_reset(phandle, "sha", true);
	esp_crypto_reset(phandle, "rsa", true);

	mutex_init(&sc->sc_aes_lock, MUTEX_DEFAULT, IPL_VM);
	mutex_init(&sc->sc_sha_lock, MUTEX_DEFAULT, IPL_VM);
	mutex_init(&sc->sc_rsa_lock, MUTEX_DEFAULT, IPL_NONE);
	mutex_init(&sc->sc_ses_lock, MUTEX_DEFAULT, IPL_NONE);
	esp_crypto_sc = sc;

	/* The big number engine clears its memories first. */
	for (guard = 10000; guard > 0 && (RD(sc, RSA_QUERY_CLEAN) & 1) == 0;
	    guard--)
		DELAY(1);
	WR(sc, RSA_INT_ENA, 0);

	/*
	 * Self-tests: an engine that gives wrong answers (or none: an
	 * emulator without it) isn't used.  The AES test is the engine's
	 * part; aes(9) tests the rest when it picks us.
	 */
	sc->sc_aes_ok = esp_crypto_test_aes(sc);
	sc->sc_sha_ok = esp_crypto_test_sha(sc);
	sc->sc_rsa_ok = guard > 0 && esp_crypto_test_rsa(sc);

	aprint_naive("\n");
	aprint_normal(": AES %s, SHA %s, RSA %s\n",
	    sc->sc_aes_ok ? "ok" : "FAILED", sc->sc_sha_ok ? "ok" : "FAILED",
	    sc->sc_rsa_ok ? "ok" : "FAILED");

	if (sc->sc_aes_ok)
		aes_md_init(&esp_aes_impl);

	if (!sc->sc_sha_ok && !sc->sc_rsa_ok)
		return;
	evcnt_attach_dynamic(&sc->sc_ev_hash, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "SHA requests");
	evcnt_attach_dynamic(&sc->sc_ev_modexp, EVCNT_TYPE_MISC, NULL,
	    device_xname(self), "modexp requests");
	sc->sc_cid = crypto_get_driverid(0);
	if (sc->sc_cid < 0) {
		aprint_error_dev(self, "couldn't get a crypto driver id\n");
		return;
	}
	if (sc->sc_sha_ok) {
		static const int algs[] = { CRYPTO_SHA1, CRYPTO_SHA1_HMAC,
		    CRYPTO_SHA2_256_HMAC, CRYPTO_SHA2_384_HMAC,
		    CRYPTO_SHA2_512_HMAC };

		for (u_int i = 0; i < __arraycount(algs); i++)
			crypto_register(sc->sc_cid, algs[i], 0, 0,
			    esp_crypto_newsession, esp_crypto_freesession,
			    esp_crypto_process, sc);
	}
	if (sc->sc_rsa_ok)
		crypto_kregister(sc->sc_cid, CRK_MOD_EXP, 0,
		    esp_crypto_kprocess, sc);
}
