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
 * sntpdate: set the clock from the first SNTP (RFC 4330) server that
 * answers.  The ESP32-S31 has no battery-backed RTC, so this runs from a
 * dhcpcd hook once the network is up.  Small on purpose: one request per
 * try, sanity checks on the reply, then a step (or a slew, for offsets
 * under 128ms) of the system clock.
 *
 * usage: sntpdate [-q] [-t timeout] server ...
 */

#include <sys/socket.h>
#include <sys/time.h>

#include <err.h>
#include <errno.h>
#include <netdb.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define	NTP_PORT	"123"
#define	NTP_UNIX_DELTA	2208988800ULL	/* 1900-01-01 to 1970-01-01, in s */
#define	TRIES		3
#define	SLEW_MAX	0.128		/* step the clock beyond this */

struct ntp_packet {
	uint8_t		li_vn_mode;
	uint8_t		stratum;
	uint8_t		poll;
	int8_t		precision;
	uint32_t	root_delay;
	uint32_t	root_dispersion;
	uint32_t	ref_id;
	uint32_t	ref_ts[2];
	uint32_t	orig_ts[2];
	uint32_t	recv_ts[2];
	uint32_t	xmit_ts[2];
};

static int timeout_ms = 2000;

static double
now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

/* NTP timestamp (network order) to Unix time, for NTP eras 0 and 1. */
static double
ntp2unix(const uint32_t ts[2])
{
	uint64_t sec = ntohl(ts[0]);

	if (sec < 0x80000000ULL)	/* after 2036-02-07: era 1 */
		sec += 0x100000000ULL;
	return (double)(sec - NTP_UNIX_DELTA) + ntohl(ts[1]) / 4294967296.0;
}

/* Ask one address; return 0 and the clock offset if it answered sanely. */
static int
query(const struct addrinfo *ai, double *offsetp)
{
	struct ntp_packet req, rep;
	struct pollfd pfd;
	uint32_t cookie[2];
	double t1, t4;
	ssize_t n;
	int s, try;

	if ((s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol)) == -1)
		return -1;
	if (connect(s, ai->ai_addr, ai->ai_addrlen) == -1) {
		close(s);
		return -1;
	}
	for (try = 0; try < TRIES; try++) {
		memset(&req, 0, sizeof(req));
		req.li_vn_mode = (0 << 6) | (4 << 3) | 3;	/* v4, client */
		/*
		 * The transmit timestamp comes back as the origin timestamp:
		 * use random bits there, so a forged reply has to guess them.
		 */
		arc4random_buf(cookie, sizeof(cookie));
		memcpy(req.xmit_ts, cookie, sizeof(cookie));
		t1 = now();
		if (send(s, &req, sizeof(req), 0) != sizeof(req))
			break;
		pfd.fd = s;
		pfd.events = POLLIN;
		while (poll(&pfd, 1, timeout_ms) == 1) {
			n = recv(s, &rep, sizeof(rep), 0);
			t4 = now();
			if (n < (ssize_t)sizeof(rep))
				continue;
			if (memcmp(rep.orig_ts, cookie, sizeof(cookie)) != 0)
				continue;		/* not our request */
			if ((rep.li_vn_mode & 7) != 4 ||	/* not a server */
			    (rep.li_vn_mode >> 6) == 3 ||	/* unsynchronised */
			    rep.stratum == 0 || rep.stratum > 15 ||
			    rep.xmit_ts[0] == 0)
				break;			/* kiss-o'-death etc. */
			/*
			 * The server's clock minus ours, from the midpoint of
			 * the exchange: ((T2 - T1) + (T3 - T4)) / 2.  We put
			 * no time in the request, so take T1 locally.
			 */
			*offsetp = ((ntp2unix(rep.recv_ts) - t1) +
			    (ntp2unix(rep.xmit_ts) - t4)) / 2;
			close(s);
			return 0;
		}
	}
	close(s);
	return -1;
}

static int
setclock(double offset)
{
	struct timespec ts;
	struct timeval tv;
	double t;

	if (offset > -SLEW_MAX && offset < SLEW_MAX) {
		tv.tv_sec = 0;
		tv.tv_usec = (suseconds_t)(offset * 1e6);
		return adjtime(&tv, NULL);
	}
	t = now() + offset;
	ts.tv_sec = (time_t)t;
	ts.tv_nsec = (long)((t - (double)ts.tv_sec) * 1e9);
	return clock_settime(CLOCK_REALTIME, &ts);
}

static void
usage(void)
{
	fprintf(stderr, "usage: %s [-q] [-t timeout] server ...\n",
	    getprogname());
	exit(2);
}

int
main(int argc, char **argv)
{
	struct addrinfo hints, *res, *ai;
	bool query_only = false;
	double offset;
	char *ep;
	int ch, error;

	while ((ch = getopt(argc, argv, "qt:")) != -1) {
		switch (ch) {
		case 'q':
			query_only = true;
			break;
		case 't':
			timeout_ms = (int)(strtod(optarg, &ep) * 1000);
			if (*ep != '\0' || timeout_ms <= 0)
				usage();
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc == 0)
		usage();

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	for (; argc > 0; argc--, argv++) {
		error = getaddrinfo(argv[0], NTP_PORT, &hints, &res);
		if (error) {
			warnx("%s: %s", argv[0], gai_strerror(error));
			continue;
		}
		for (ai = res; ai != NULL; ai = ai->ai_next) {
			if (query(ai, &offset) != 0)
				continue;
			freeaddrinfo(res);
			if (!query_only && setclock(offset) == -1)
				err(1, "can't set the clock");
			printf("%s: %s clock from %s, offset %+.3f s\n",
			    getprogname(), query_only ? "would set" :
			    (offset > -SLEW_MAX && offset < SLEW_MAX) ?
			    "slewed" : "set", argv[0], offset);
			return 0;
		}
		freeaddrinfo(res);
		warnx("%s: no answer", argv[0]);
	}
	return 1;
}
