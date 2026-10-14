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
 * espcp(4): the ESP32-S31 radio co-processor, and its interface to the
 * espwifi(4) and espbt(4) drivers that attach to it.
 */

#ifndef _RISCV_ESPRESSIF_ESP_CPVAR_H_
#define _RISCV_ESPRESSIF_ESP_CPVAR_H_

#include <sys/types.h>
#include <sys/device.h>
#include <sys/mbuf.h>

#include <riscv/espressif/esp_cpproto.h>

struct espcp_attach_args {
	const char	*ea_name;	/* "espwifi" or "espbt" */
	const uint8_t	*ea_addr;	/* MAC or BD_ADDR */
};

/*
 * A client's receive handler, called from a soft interrupt with a message
 * on its channel (the payload is only valid during the call).  The space
 * handler is called when the ring towards the firmware has room again
 * after a send failed with ENOBUFS.  Events are control messages with
 * sequence number 0.
 */
typedef void	(*espcp_rx_fn)(void *, uint8_t, const void *, size_t);
typedef void	(*espcp_space_fn)(void *);

void	espcp_register(device_t, uint8_t, espcp_rx_fn, espcp_space_fn, void *);
void	espcp_register_events(device_t, espcp_rx_fn, void *);

/* Send a message: a header, then an mbuf chain (either may be empty). */
int	espcp_send(device_t, uint8_t, uint8_t, const void *, size_t,
	    struct mbuf *);

/*
 * A control request: sleeps until the reply (ms timeout).  The reply's
 * payload is copied to rbuf (at most *rlen bytes; *rlen is set to its
 * length).  Returns an errno, the firmware's status included.
 */
int	espcp_ctl(device_t, uint8_t, const void *, size_t, void *, size_t *,
	    int);
/* A control request whose reply nobody waits for. */
int	espcp_ctl_async(device_t, uint8_t, const void *, size_t);

/* Is the firmware alive? */
bool	espcp_running(device_t);

#endif /* _RISCV_ESPRESSIF_ESP_CPVAR_H_ */
