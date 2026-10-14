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
 * ESP32-S31 on-chip sensors, read through the radio co-processor (whose
 * ESP-IDF owns the analog I2C bus the sensors hang off, and does their
 * calibration): the temperature sensor, and the SAR ADCs' 16 channels
 * (ADC1 0-7 on GPIO42-49, ADC2 0-7 on GPIO50-57), as envsys(4) sensors.
 *
 * Reading an ADC channel turns its pad into an analog input, so channels
 * are only read once enabled: sysctl -w hw.espsensor0.adc_channels=MASK
 * (bit 8*u+c for ADC u+1 channel c; e.g. 0x3 for GPIO42 and GPIO43).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <dev/sysmon/sysmonvar.h>

#include <riscv/espressif/esp_cpvar.h>

#define	ESPSENSOR_NSENSORS	(1 + ESPCP_ADC_CHANNELS)
#define	ESPSENSOR_CACHE_TICKS	(hz / 2)

struct espsensor_softc {
	device_t		sc_dev;
	device_t		sc_cp;
	kmutex_t		sc_lock;
	struct sysmon_envsys	*sc_sme;
	envsys_data_t		sc_sensor[ESPSENSOR_NSENSORS];
	uint32_t		sc_mask;	/* ADC channels to read */
	struct espcp_sensors	sc_last;
	int			sc_last_ticks;
	bool			sc_last_ok;
	struct sysctllog	*sc_sysctllog;
};

static int	espsensor_match(device_t, cfdata_t, void *);
static void	espsensor_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(espsensor, sizeof(struct espsensor_softc),
	espsensor_match, espsensor_attach, NULL, NULL);

static int
espsensor_match(device_t parent, cfdata_t cf, void *aux)
{
	const struct espcp_attach_args * const ea = aux;

	return strcmp(ea->ea_name, "espsensor") == 0;
}

/* Ask the firmware, unless we just did. */
static bool
espsensor_fetch(struct espsensor_softc *sc)
{
	struct espcp_sensors_req req;
	struct espcp_sensors r;
	size_t rlen = sizeof(r);
	int error;

	KASSERT(mutex_owned(&sc->sc_lock));

	if (sc->sc_last_ok &&
	    getticks() - sc->sc_last_ticks < ESPSENSOR_CACHE_TICKS)
		return true;

	req.sr_adc_mask = sc->sc_mask;
	error = espcp_ctl(sc->sc_cp, ESPCP_CTL_SENSORS, &req, sizeof(req),
	    &r, &rlen, 2000);
	if (error != 0 || rlen < sizeof(r)) {
		sc->sc_last_ok = false;
		return false;
	}
	sc->sc_last = r;
	sc->sc_last_ok = true;
	sc->sc_last_ticks = getticks();
	return true;
}

static void
espsensor_refresh(struct sysmon_envsys *sme, envsys_data_t *edata)
{
	struct espsensor_softc * const sc = sme->sme_cookie;
	const int i = edata->sensor;

	mutex_enter(&sc->sc_lock);
	edata->state = ENVSYS_SINVALID;
	if (!espsensor_fetch(sc))
		goto out;
	if (i == 0) {
		if (sc->sc_last.ss_flags & ESPCP_SENS_TEMP) {
			/* micro-kelvin */
			edata->value_cur = sc->sc_last.ss_temp * 1000 +
			    273150000;
			edata->state = ENVSYS_SVALID;
		}
	} else {
		const u_int ch = i - 1;

		if ((sc->sc_mask & sc->sc_last.ss_adc_valid & (1U << ch)) == 0)
			goto out;
		if (sc->sc_last.ss_adc_cal & (1U << ch))
			edata->value_cur = sc->sc_last.ss_adc_mv[ch] * 1000;
		else
			edata->value_cur = sc->sc_last.ss_adc_raw[ch];
		edata->state = ENVSYS_SVALID;
	}
out:
	mutex_exit(&sc->sc_lock);
}

static int
espsensor_sysctl_mask(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct espsensor_softc * const sc = node.sysctl_data;
	int mask, error;

	mask = sc->sc_mask;
	node.sysctl_data = &mask;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;
	if (mask < 0 || mask > 0xffff)
		return EINVAL;
	mutex_enter(&sc->sc_lock);
	sc->sc_mask = mask;
	sc->sc_last_ok = false;
	mutex_exit(&sc->sc_lock);
	return 0;
}

static void
espsensor_attach(device_t parent, device_t self, void *aux)
{
	struct espsensor_softc * const sc = device_private(self);
	const struct sysctlnode *node;

	sc->sc_dev = self;
	sc->sc_cp = parent;
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	aprint_naive("\n");
	aprint_normal(": temperature sensor, ADC1/ADC2\n");

	sc->sc_sme = sysmon_envsys_create();
	sc->sc_sme->sme_name = device_xname(self);
	sc->sc_sme->sme_cookie = sc;
	sc->sc_sme->sme_refresh = espsensor_refresh;

	sc->sc_sensor[0].units = ENVSYS_STEMP;
	sc->sc_sensor[0].state = ENVSYS_SINVALID;
	strlcpy(sc->sc_sensor[0].desc, "temperature",
	    sizeof(sc->sc_sensor[0].desc));
	for (u_int ch = 0; ch < ESPCP_ADC_CHANNELS; ch++) {
		envsys_data_t * const e = &sc->sc_sensor[1 + ch];

		e->units = ENVSYS_SVOLTS_DC;
		e->state = ENVSYS_SINVALID;
		snprintf(e->desc, sizeof(e->desc), "ADC%u_%u (GPIO%u)",
		    ch / 8 + 1, ch % 8, 42 + ch);
	}
	for (u_int i = 0; i < ESPSENSOR_NSENSORS; i++) {
		if (sysmon_envsys_sensor_attach(sc->sc_sme,
		    &sc->sc_sensor[i]) != 0) {
			aprint_error_dev(self, "couldn't attach sensor %u\n",
			    i);
			sysmon_envsys_destroy(sc->sc_sme);
			sc->sc_sme = NULL;
			return;
		}
	}
	if (sysmon_envsys_register(sc->sc_sme) != 0) {
		aprint_error_dev(self, "couldn't register with envsys\n");
		sysmon_envsys_destroy(sc->sc_sme);
		sc->sc_sme = NULL;
		return;
	}

	if (sysctl_createv(&sc->sc_sysctllog, 0, NULL, &node, 0,
	    CTLTYPE_NODE, device_xname(self), SYSCTL_DESCR("sensors"),
	    NULL, 0, NULL, 0, CTL_HW, CTL_CREATE, CTL_EOL) == 0)
		sysctl_createv(&sc->sc_sysctllog, 0, &node, NULL,
		    CTLFLAG_READWRITE, CTLTYPE_INT, "adc_channels",
		    SYSCTL_DESCR("ADC channels to read: bit 8*u+c is "
		    "ADC u+1 channel c (GPIO42-57)"),
		    espsensor_sysctl_mask, 0, (void *)sc, 0,
		    CTL_CREATE, CTL_EOL);
}
