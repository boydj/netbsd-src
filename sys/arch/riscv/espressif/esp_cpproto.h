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
 * ESP32-S31 radio co-processor: the interface between the host (NetBSD on
 * hart 0) and the firmware that runs Espressif's Wi-Fi and Bluetooth
 * stacks on hart 1.
 *
 * This file is shared with the firmware (esp32s31-netbsd: espcp/); keep
 * the two copies identical.
 *
 * The firmware image sits in flash and runs in place through the XIP
 * window, except for the segments that must be in RAM, which the host
 * copies before it starts hart 1 in M-mode (OpenSBI vendor call).  The
 * host also gives the firmware a block of PSRAM: it holds the shared
 * memory (struct espcp_shm) and the rest is heap.  PSRAM is cached, but
 * the L1 data cache is shared by both harts, so the shared memory needs
 * no cache maintenance, only ordering.
 *
 * Messages go through two rings in the shared memory, one each way; a
 * doorbell (a FROM_CPU interrupt) tells the other side to look at them.
 */

#ifndef _RISCV_ESPRESSIF_ESP_CPPROTO_H_
#define _RISCV_ESPRESSIF_ESP_CPPROTO_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdint.h>
#endif

/*
 * Firmware image header, at the start of the image.
 */
#define	ESPCP_FW_MAGIC		0x43313353u	/* "S31C" */
#define	ESPCP_FW_VERSION	1
#define	ESPCP_FW_MAXSEGS	12

struct espcp_fw_seg {
	uint32_t	es_addr;	/* load address */
	uint32_t	es_offset;	/* contents, from the start of the image */
	uint32_t	es_filesz;	/* bytes to copy */
	uint32_t	es_memsz;	/* >= es_filesz; the rest is cleared */
};

struct espcp_fw_header {
	uint32_t	eh_magic;	/* ESPCP_FW_MAGIC */
	uint16_t	eh_version;	/* ESPCP_FW_VERSION */
	uint16_t	eh_hdrsize;	/* sizeof(struct espcp_fw_header) */
	uint32_t	eh_imagesize;	/* bytes, header included */
	uint32_t	eh_xipbase;	/* address the image must be mapped at */
	uint32_t	eh_entry;	/* M-mode entry point, in a RAM segment */
	uint32_t	eh_params;	/* struct espcp_bootparams, in a RAM seg. */
	uint32_t	eh_extmem;	/* bytes of PSRAM wanted (shm + heap) */
	uint32_t	eh_shmsize;	/* bytes of it for struct espcp_shm */
	uint32_t	eh_nsegs;
	uint32_t	eh_crc;		/* CRC-32 of the RAM segments' contents */
	uint32_t	eh_reserved[2];
	char		eh_ident[64];	/* NUL-terminated */
	struct espcp_fw_seg eh_segs[ESPCP_FW_MAXSEGS];
};

/*
 * Filled in by the host after it has copied the RAM segments.
 */
#define	ESPCP_BP_MAGIC		0x50423343u	/* "C3BP" */

struct espcp_bootparams {
	uint32_t	bp_magic;	/* ESPCP_BP_MAGIC */
	uint32_t	bp_extmem;	/* physical address of the PSRAM block */
	uint32_t	bp_extmemsize;
	uint32_t	bp_shm;		/* struct espcp_shm, at the block's start */
	uint32_t	bp_shmsize;
	uint32_t	bp_flags;
#define	ESPCP_BP_LOG_UART	0x00000001	/* also log to UART0 */
	uint32_t	bp_reserved[2];
};

/*
 * Doorbells: HP_SYSTEM CPU_INT_FROM_CPU_n.  Write 1 to ring, 0 to clear.
 * FROM_CPU 0 is the firmware's own cross-core interrupt.
 */
#define	ESPCP_DOORBELL_FW	1	/* host -> firmware */
#define	ESPCP_DOORBELL_HOST	2	/* firmware -> host */
#define	ESPCP_FROM_CPU_SOURCE(n)	(65 + (n))	/* intr. matrix source */
#define	ESPCP_FROM_CPU_REG(n)		(0x20586010u + 4 * (n))

/*
 * A ring of messages.  Head and tail are free-running byte counts; the
 * producer only writes er_head and the consumer only writes er_tail.
 * Messages are 4-byte aligned and never wrap: a producer that does not
 * have room before the end of the ring puts an ESPCP_CHAN_PAD header
 * there, which means the rest of the ring is unused, and starts again at
 * the beginning.  A producer that runs out of space sets er_wait; the
 * consumer clears it and rings the doorbell when it has made room.
 */
struct espcp_ring {
	volatile uint32_t er_head;
	volatile uint32_t er_tail;
	uint32_t	er_offset;	/* from the start of the shared memory */
	uint32_t	er_size;	/* power of two */
	volatile uint32_t er_wait;
	uint32_t	er_reserved[3];
};

struct espcp_msg {
	uint16_t	em_len;		/* payload bytes */
	uint8_t		em_chan;	/* ESPCP_CHAN_* */
	uint8_t		em_type;	/* channel-specific */
	/* payload, padded to 4 bytes */
};

#define	ESPCP_MSG_ALIGN(n)	(((n) + 3u) & ~3u)
#define	ESPCP_MSG_MAX		2048	/* largest payload */

#define	ESPCP_CHAN_CTL		1	/* commands, replies and events */
#define	ESPCP_CHAN_WIFI		2	/* Ethernet frames, station interface */
#define	ESPCP_CHAN_HCI		3	/* HCI packet with its H4 type byte */
#define	ESPCP_CHAN_PAD		0xff

/*
 * The shared memory.  The host clears it before it starts the firmware;
 * the firmware lays out the rings and sets es_magic last.
 */
#define	ESPCP_SHM_MAGIC		0x4d485343u	/* "CSHM" */
#define	ESPCP_PROTO_VERSION	1

struct espcp_shm {
	volatile uint32_t es_magic;	/* ESPCP_SHM_MAGIC once running */
	uint32_t	es_version;	/* ESPCP_PROTO_VERSION */
	volatile uint32_t es_state;	/* ESPCP_STATE_* */
	volatile uint32_t es_heartbeat;	/* firmware's uptime in seconds */
	uint32_t	es_caps;	/* ESPCP_CAP_* */
	uint8_t		es_wifi_mac[6];
	uint8_t		es_bt_addr[6];
	char		es_ident[64];	/* firmware and ESP-IDF versions */
	volatile uint32_t es_heap_free;	/* bytes, internal and PSRAM */
	volatile uint32_t es_heap_min;	/* low-water mark */
	uint32_t	es_reserved[14];
	struct espcp_ring es_h2f;	/* host to firmware */
	struct espcp_ring es_f2h;	/* firmware to host */
	struct espcp_ring es_log;	/* firmware log, plain text */
	char		es_panic[256];	/* why the firmware stopped */
};

#define	ESPCP_STATE_BOOT	0
#define	ESPCP_STATE_RUNNING	1
#define	ESPCP_STATE_PANIC	2

#define	ESPCP_CAP_WIFI		0x00000001
#define	ESPCP_CAP_BT		0x00000002	/* BR/EDR and LE */
#define	ESPCP_CAP_SENSORS	0x00000004	/* temperature, ADCs */
#define	ESPCP_CAP_POWER		0x00000008	/* CPU clock, deep sleep */

/*
 * Control channel.  Every message starts with struct espcp_ctl; replies
 * echo the request's sequence number, events have sequence number 0.
 */
struct espcp_ctl {
	uint32_t	ec_seq;
	int32_t		ec_status;	/* replies: 0 or an ESPCP_E* */
};

/* Status codes: NetBSD's errno values. */
#define	ESPCP_ENOENT		2
#define	ESPCP_EIO		5
#define	ESPCP_ENOMEM		12
#define	ESPCP_EBUSY		16
#define	ESPCP_EINVAL		22
#define	ESPCP_EOPNOTSUPP	45
#define	ESPCP_ENETDOWN		50
#define	ESPCP_ETIMEDOUT		60

#define	ESPCP_CTL_PING		1
#define	ESPCP_CTL_WIFI_START	2	/* station mode, radio on */
#define	ESPCP_CTL_WIFI_STOP	3	/* radio off */
#define	ESPCP_CTL_WIFI_SCAN	4	/* reply: espcp_wifi_scan */
#define	ESPCP_CTL_WIFI_CONNECT	5	/* espcp_wifi_config */
#define	ESPCP_CTL_WIFI_DISCONNECT 6
#define	ESPCP_CTL_WIFI_STATUS	7	/* reply: espcp_wifi_status */
#define	ESPCP_CTL_BT_ENABLE	8
#define	ESPCP_CTL_BT_DISABLE	9
#define	ESPCP_CTL_SENSORS	10	/* espcp_sensors_req; reply: espcp_sensors */
#define	ESPCP_CTL_POWER		11	/* espcp_power_req; reply: espcp_power */

#define	ESPCP_EVT_WIFI_LINK	64	/* espcp_wifi_status */

/* Authentication modes: ESP-IDF's wifi_auth_mode_t. */
#define	ESPCP_AUTH_OPEN		0
#define	ESPCP_AUTH_WEP		1
#define	ESPCP_AUTH_WPA_PSK	2
#define	ESPCP_AUTH_WPA2_PSK	3
#define	ESPCP_AUTH_WPA_WPA2_PSK	4
#define	ESPCP_AUTH_ENTERPRISE	5
#define	ESPCP_AUTH_WPA3_PSK	6
#define	ESPCP_AUTH_WPA2_WPA3_PSK 7
#define	ESPCP_AUTH_WAPI_PSK	8
#define	ESPCP_AUTH_OWE		9
#define	ESPCP_AUTH_WPA3_ENT_192	10

struct espcp_wifi_config {
	char		wc_ssid[33];	/* NUL-terminated */
	char		wc_password[65];
	uint8_t		wc_bssid[6];
	uint8_t		wc_flags;
#define	ESPCP_WC_BSSID		0x01	/* only this BSSID */
	uint8_t		wc_minauth;	/* weakest authentication accepted */
	uint8_t		wc_reserved[2];
};

struct espcp_wifi_ap {
	uint8_t		ap_bssid[6];
	uint8_t		ap_ssid_len;
	uint8_t		ap_channel;
	int8_t		ap_rssi;
	uint8_t		ap_auth;	/* ESPCP_AUTH_* */
	uint8_t		ap_reserved[2];
	char		ap_ssid[32];
};

#define	ESPCP_SCAN_MAX		32

struct espcp_wifi_scan {
	uint32_t	ws_count;
	struct espcp_wifi_ap ws_ap[];
};

struct espcp_wifi_status {
	uint8_t		ws_state;	/* ESPCP_WIFI_* */
	uint8_t		ws_channel;
	int8_t		ws_rssi;
	uint8_t		ws_auth;
	uint8_t		ws_bssid[6];
	uint8_t		ws_reason;	/* last disconnection, IDF's codes */
	uint8_t		ws_ssid_len;
	char		ws_ssid[32];
};

#define	ESPCP_WIFI_OFF		0	/* radio off */
#define	ESPCP_WIFI_IDLE		1	/* on, not associated */
#define	ESPCP_WIFI_CONNECTING	2
#define	ESPCP_WIFI_CONNECTED	3

/*
 * Sensors: the on-chip temperature sensor and the two SAR ADCs, read with
 * ESP-IDF's drivers (which own the analog I2C bus and the calibration).
 * ADC channels: bit 8*u + c of the mask is unit u+1 channel c; ADC1
 * channels 0-7 are GPIO42-49, ADC2 channels 0-7 GPIO50-57.  A channel's
 * pad is switched to analog the first time it's read.
 */
#define	ESPCP_ADC_CHANNELS	16

struct espcp_sensors_req {
	uint32_t	sr_adc_mask;	/* channels to read */
};

struct espcp_sensors {
	int32_t		ss_temp;	/* milli-degrees Celsius */
	uint32_t	ss_flags;
#define	ESPCP_SENS_TEMP		0x01	/* ss_temp is valid */
	uint32_t	ss_adc_valid;	/* channels read */
	uint32_t	ss_adc_cal;	/* ... whose mV are calibrated */
	int32_t		ss_adc_raw[ESPCP_ADC_CHANNELS];
	int32_t		ss_adc_mv[ESPCP_ADC_CHANNELS];
};

/*
 * Power.  The CPU clock is the both harts', so the firmware changes it
 * (as ESP-IDF's own frequency scaling would: CPLL dividers only); deep
 * sleep powers the whole chip down, until the timer or a low LP GPIO
 * (0-7) wakes it, which is a reset.  POWER_OFF has no reply.
 */
struct espcp_power_req {
	uint32_t	pr_op;
#define	ESPCP_POWER_CPUFREQ	1	/* pr_arg: MHz, or 0 to ask */
#define	ESPCP_POWER_OFF		2	/* pr_arg: seconds to wake after, or 0;
					   pr_arg2: LP GPIOs to wake on */
	uint32_t	pr_arg;
	uint32_t	pr_arg2;
};

struct espcp_power {
	uint32_t	ps_mhz;		/* the CPU clock now */
	uint32_t	ps_avail[4];	/* those it may have, high to low */
};

/*
 * NetBSD's espwifi(4): driver-specific ioctls (SIOCGDRVSPEC/SIOCSDRVSPEC,
 * ifd_cmd), used by espwifictl(8).
 */
#define	ESPWIFI_SCAN		1	/* get: struct espcp_wifi_scan */
#define	ESPWIFI_JOIN		2	/* set: struct espcp_wifi_config */
#define	ESPWIFI_LEAVE		3	/* set */
#define	ESPWIFI_STATUS		4	/* get: struct espcp_wifi_status */

#endif /* _RISCV_ESPRESSIF_ESP_CPPROTO_H_ */
