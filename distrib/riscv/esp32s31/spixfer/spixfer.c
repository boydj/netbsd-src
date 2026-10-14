/*	$NetBSD$	*/

/*
 * spixfer: one SPI transfer through spi(4).
 *
 *	spixfer [-f /dev/spiN] [-s cs] [-m mode] [-c hz] [byte ...] [-r count]
 *
 * Sends the bytes, then reads count bytes (clocking out 0xff), with the
 * chip select held throughout.  Bytes read are printed in hex.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/ioctl.h>

#include <dev/spi/spi_io.h>

#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void
usage(void)
{
	fprintf(stderr, "usage: %s [-f device] [-s cs] [-m mode] [-c hz] "
	    "[byte ...] [-r count]\n", getprogname());
	exit(1);
}

static unsigned long
number(const char *s, unsigned long max)
{
	char *ep;
	unsigned long v = strtoul(s, &ep, 0);

	if (*s == '\0' || *ep != '\0' || v > max)
		errx(1, "bad number `%s'", s);
	return v;
}

int
main(int argc, char **argv)
{
	const char *dev = "/dev/spi0";
	uint8_t wbuf[4096], rbuf[4096];
	size_t wlen = 0, rlen = 0;
	spi_ioctl_configure_t sic = { .sic_addr = 0, .sic_mode = 0,
	    .sic_speed = 1000000 };
	spi_ioctl_transfer_t sit;
	int c, fd;

	while ((c = getopt(argc, argv, "c:f:m:r:s:")) != -1) {
		switch (c) {
		case 'c':
			sic.sic_speed = number(optarg, 80000000);
			break;
		case 'f':
			dev = optarg;
			break;
		case 'm':
			sic.sic_mode = number(optarg, 3);
			break;
		case 'r':
			rlen = number(optarg, sizeof(rbuf));
			break;
		case 's':
			sic.sic_addr = number(optarg, 15);
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	for (int i = 0; i < argc; i++) {
		if (wlen == sizeof(wbuf))
			errx(1, "too many bytes");
		wbuf[wlen++] = number(argv[i], 0xff);
	}
	if (wlen == 0 && rlen == 0)
		usage();

	if ((fd = open(dev, O_RDWR)) == -1)
		err(1, "%s", dev);
	if (ioctl(fd, SPI_IOCTL_CONFIGURE, &sic) == -1)
		err(1, "SPI_IOCTL_CONFIGURE");
	memset(&sit, 0, sizeof(sit));
	sit.sit_addr = sic.sic_addr;
	sit.sit_send = wlen ? wbuf : NULL;
	sit.sit_sendlen = wlen;
	sit.sit_recv = rlen ? rbuf : NULL;
	sit.sit_recvlen = rlen;
	if (ioctl(fd, SPI_IOCTL_TRANSFER, &sit) == -1)
		err(1, "SPI_IOCTL_TRANSFER");
	for (size_t i = 0; i < rlen; i++)
		printf("%02x%c", rbuf[i],
		    (i + 1 == rlen || (i + 1) % 16 == 0) ? '\n' : ' ');
	close(fd);
	return 0;
}
