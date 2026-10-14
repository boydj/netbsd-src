/*	$NetBSD$	*/

/*
 * i2cxfer: one I2C transfer through iic(4).
 *
 *	i2cxfer [-f /dev/iicN] addr [w byte ...] [r count]
 *
 * "w" bytes are written first; with "r" as well, the read follows with a
 * repeated start (the usual "write register number, read registers").
 * Bytes read are printed in hex.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/ioctl.h>

#include <dev/i2c/i2c_io.h>

#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void
usage(void)
{
	fprintf(stderr, "usage: %s [-f device] addr [w byte ...] [r count]\n",
	    getprogname());
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
	const char *dev = "/dev/iic0";
	uint8_t wbuf[256], rbuf[256];
	size_t wlen = 0, rlen = 0;
	i2c_ioctl_exec_t iie;
	int c, fd;

	while ((c = getopt(argc, argv, "f:")) != -1) {
		switch (c) {
		case 'f':
			dev = optarg;
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc < 1)
		usage();

	memset(&iie, 0, sizeof(iie));
	iie.iie_addr = number(argv[0], 0x7f);
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "w") == 0) {
			while (i + 1 < argc && strcmp(argv[i + 1], "r") != 0) {
				if (wlen == sizeof(wbuf))
					errx(1, "too many bytes");
				wbuf[wlen++] = number(argv[++i], 0xff);
			}
		} else if (strcmp(argv[i], "r") == 0 && i + 1 < argc) {
			rlen = number(argv[++i], sizeof(rbuf));
		} else {
			usage();
		}
	}

	if ((fd = open(dev, O_RDWR)) == -1)
		err(1, "%s", dev);
	if (rlen > 0) {
		iie.iie_op = I2C_OP_READ_WITH_STOP;
		iie.iie_cmd = wlen ? wbuf : NULL;
		iie.iie_cmdlen = wlen;
		iie.iie_buf = rbuf;
		iie.iie_buflen = rlen;
	} else {
		iie.iie_op = I2C_OP_WRITE_WITH_STOP;
		iie.iie_buf = wbuf;
		iie.iie_buflen = wlen;
	}
	if (ioctl(fd, I2C_IOCTL_EXEC, &iie) == -1)
		err(1, "I2C_IOCTL_EXEC");
	for (size_t i = 0; i < rlen; i++)
		printf("%02x%c", rbuf[i],
		    (i + 1 == rlen || (i + 1) % 16 == 0) ? '\n' : ' ');
	close(fd);
	return 0;
}
