/*	$NetBSD$	*/

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
 * cansend, candump: the can-utils basics for NetBSD's CAN sockets.
 *
 *	cansend esptwai0 123#deadbeef	standard id 0x123, 4 bytes
 *	cansend esptwai0 12345678#01	extended id (8 hex digits)
 *	cansend esptwai0 123#R		remote frame
 *	candump [-n count] [-t secs] esptwai0
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <net/if.h>
#include <netcan/can.h>

#include <ctype.h>
#include <err.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int
can_open(const char *ifname)
{
	struct sockaddr_can sa;
	int s;

	if ((s = socket(AF_CAN, SOCK_RAW, CAN_RAW)) == -1)
		err(EXIT_FAILURE, "socket");
	memset(&sa, 0, sizeof(sa));
	sa.can_len = sizeof(sa);
	sa.can_family = AF_CAN;
	if ((sa.can_ifindex = if_nametoindex(ifname)) == 0)
		errx(EXIT_FAILURE, "%s: no such interface", ifname);
	if (bind(s, (struct sockaddr *)&sa, sizeof(sa)) == -1)
		err(EXIT_FAILURE, "bind %s", ifname);
	return s;
}

static int
hexval(int c)
{
	c = tolower((unsigned char)c);
	return isdigit(c) ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

static void
parse_frame(const char *spec, struct can_frame *cf)
{
	const char *hash = strchr(spec, '#'), *p;
	size_t idlen;
	char *end;
	unsigned long id;

	memset(cf, 0, sizeof(*cf));
	if (hash == NULL)
		errx(EXIT_FAILURE, "%s: want id#data", spec);
	idlen = hash - spec;
	id = strtoul(spec, &end, 16);
	if (end != hash || idlen == 0 || idlen > 8)
		errx(EXIT_FAILURE, "%s: bad id", spec);
	if (idlen > 3 || id > CAN_SFF_MASK) {
		if (id > CAN_EFF_MASK)
			errx(EXIT_FAILURE, "%s: id too big", spec);
		cf->can_id = id | CAN_EFF_FLAG;
	} else {
		cf->can_id = id;
	}
	p = hash + 1;
	if (*p == 'R' || *p == 'r') {
		cf->can_id |= CAN_RTR_FLAG;
		if (isdigit((unsigned char)p[1]))
			cf->can_dlc = p[1] - '0';
		if (cf->can_dlc > CAN_MAX_DLC)
			errx(EXIT_FAILURE, "%s: bad length", spec);
		return;
	}
	while (*p != '\0') {
		int hi, lo;

		if (*p == '.') {
			p++;
			continue;
		}
		if ((hi = hexval(p[0])) < 0 || (lo = hexval(p[1])) < 0)
			errx(EXIT_FAILURE, "%s: bad data", spec);
		if (cf->can_dlc == CAN_MAX_DLEN)
			errx(EXIT_FAILURE, "%s: more than 8 bytes", spec);
		cf->data[cf->can_dlc++] = hi << 4 | lo;
		p += 2;
	}
}

static int
cansend(int argc, char **argv)
{
	struct can_frame cf;
	int s;

	if (argc != 3) {
		fprintf(stderr, "usage: cansend interface id#data\n");
		return EXIT_FAILURE;
	}
	parse_frame(argv[2], &cf);
	s = can_open(argv[1]);
	if (write(s, &cf, sizeof(cf)) != sizeof(cf))
		err(EXIT_FAILURE, "write");
	close(s);
	return EXIT_SUCCESS;
}

static int
candump(int argc, char **argv)
{
	const char *ifname;
	long count = -1, secs = -1;
	struct pollfd pfd;
	int ch, s;

	while ((ch = getopt(argc, argv, "n:t:")) != -1) {
		switch (ch) {
		case 'n':
			count = strtol(optarg, NULL, 10);
			break;
		case 't':
			secs = strtol(optarg, NULL, 10);
			break;
		default:
			goto usage;
		}
	}
	argc -= optind;
	argv += optind;
	if (argc != 1)
		goto usage;
	ifname = argv[0];
	s = can_open(ifname);
	setvbuf(stdout, NULL, _IOLBF, 0);

	pfd.fd = s;
	pfd.events = POLLIN;
	while (count != 0) {
		struct can_frame cf;
		ssize_t n;

		if (poll(&pfd, 1, secs < 0 ? INFTIM : (int)secs * 1000) <= 0)
			break;
		if ((n = read(s, &cf, sizeof(cf))) != sizeof(cf)) {
			if (n == -1)
				err(EXIT_FAILURE, "read");
			continue;
		}
		if (cf.can_id & CAN_EFF_FLAG)
			printf("  %s  %08X", ifname, cf.can_id & CAN_EFF_MASK);
		else
			printf("  %s  %03X", ifname, cf.can_id & CAN_SFF_MASK);
		printf("   [%u] ", cf.can_dlc);
		if (cf.can_id & CAN_RTR_FLAG)
			printf(" remote request");
		else
			for (u_int i = 0; i < cf.can_dlc && i < CAN_MAX_DLEN; i++)
				printf(" %02X", cf.data[i]);
		printf("\n");
		if (count > 0)
			count--;
	}
	close(s);
	return EXIT_SUCCESS;
usage:
	fprintf(stderr, "usage: candump [-n count] [-t secs] interface\n");
	return EXIT_FAILURE;
}

int
main(int argc, char **argv)
{
	const char *name = strrchr(argv[0], '/');

	name = name != NULL ? name + 1 : argv[0];
	if (strcmp(name, "candump") == 0)
		return candump(argc, argv);
	return cansend(argc, argv);
}
