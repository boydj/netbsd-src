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
 * espwifictl: Wi-Fi networks for espwifi(4), the ESP32-S31's Wi-Fi.
 *
 *	espwifictl [-i ifname] scan
 *	espwifictl [-i ifname] join [-b bssid] [-a minauth] ssid [passphrase]
 *	espwifictl [-i ifname] leave
 *	espwifictl [-i ifname] status
 *	espwifictl [-i ifname] wait [seconds]
 *
 * The co-processor's firmware does the association and WPA itself; the
 * interface's link goes up when it is associated, and dhcpcd does the rest.
 * Without a passphrase on the command line, join reads one from the
 * terminal ("-" for an open network).
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_ether.h>

#include <err.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <riscv/espressif/esp_cpproto.h>

static const char *ifname = "espwifi0";
static int sock = -1;

static const char *const authname[] = {
	[ESPCP_AUTH_OPEN] = "open",
	[ESPCP_AUTH_WEP] = "WEP",
	[ESPCP_AUTH_WPA_PSK] = "WPA",
	[ESPCP_AUTH_WPA2_PSK] = "WPA2",
	[ESPCP_AUTH_WPA_WPA2_PSK] = "WPA/WPA2",
	[ESPCP_AUTH_ENTERPRISE] = "WPA2-EAP",
	[ESPCP_AUTH_WPA3_PSK] = "WPA3",
	[ESPCP_AUTH_WPA2_WPA3_PSK] = "WPA2/WPA3",
	[ESPCP_AUTH_WAPI_PSK] = "WAPI",
	[ESPCP_AUTH_OWE] = "OWE",
	[ESPCP_AUTH_WPA3_ENT_192] = "WPA3-EAP-192",
};

static const char *const statename[] = {
	[ESPCP_WIFI_OFF] = "off",
	[ESPCP_WIFI_IDLE] = "not associated",
	[ESPCP_WIFI_CONNECTING] = "associating",
	[ESPCP_WIFI_CONNECTED] = "associated",
};

static const char *
auth_str(unsigned a)
{
	static char buf[16];

	if (a < __arraycount(authname) && authname[a] != NULL)
		return authname[a];
	snprintf(buf, sizeof(buf), "auth %u", a);
	return buf;
}

static int
auth_parse(const char *s)
{
	for (unsigned i = 0; i < __arraycount(authname); i++)
		if (authname[i] != NULL && strcasecmp(s, authname[i]) == 0)
			return i;
	errx(EXIT_FAILURE, "unknown authentication mode `%s'", s);
}

static void __dead
usage(void)
{
	fprintf(stderr,
	    "usage: %s [-i ifname] scan\n"
	    "       %s [-i ifname] join [-b bssid] [-a minauth] ssid "
	    "[passphrase | -]\n"
	    "       %s [-i ifname] leave | status\n"
	    "       %s [-i ifname] wait [seconds]\n",
	    getprogname(), getprogname(), getprogname(), getprogname());
	exit(EXIT_FAILURE);
}

static int
drvspec(u_long cmd, unsigned long what, void *data, size_t *len)
{
	struct ifdrv ifd;

	memset(&ifd, 0, sizeof(ifd));
	strlcpy(ifd.ifd_name, ifname, sizeof(ifd.ifd_name));
	ifd.ifd_cmd = what;
	ifd.ifd_len = *len;
	ifd.ifd_data = data;
	if (ioctl(sock, cmd, &ifd) == -1)
		return -1;
	*len = ifd.ifd_len;
	return 0;
}

static void
ssid_print(const char *ssid, size_t len, int width)
{
	int n = 0;

	for (size_t i = 0; i < len && ssid[i] != '\0'; i++, n++)
		putchar(ssid[i] >= ' ' && ssid[i] < 0x7f ? ssid[i] : '?');
	for (; n < width; n++)
		putchar(' ');
}

static int
scan(void)
{
	size_t len = sizeof(struct espcp_wifi_scan) +
	    ESPCP_SCAN_MAX * sizeof(struct espcp_wifi_ap);
	struct espcp_wifi_scan *ws = calloc(1, len);

	if (ws == NULL)
		err(EXIT_FAILURE, "calloc");
	if (drvspec(SIOCGDRVSPEC, ESPWIFI_SCAN, ws, &len) == -1)
		err(EXIT_FAILURE, "%s: scan", ifname);
	printf("%-32s  %-17s  %3s  %4s  %s\n", "SSID", "BSSID", "CH", "RSSI",
	    "SECURITY");
	for (uint32_t i = 0; i < ws->ws_count && i < ESPCP_SCAN_MAX; i++) {
		const struct espcp_wifi_ap *ap = &ws->ws_ap[i];

		ssid_print(ap->ap_ssid, ap->ap_ssid_len, 32);
		printf("  %s  %3u  %4d  %s\n",
		    ether_ntoa((const struct ether_addr *)ap->ap_bssid),
		    ap->ap_channel, ap->ap_rssi, auth_str(ap->ap_auth));
	}
	free(ws);
	return EXIT_SUCCESS;
}

static int
get_status(struct espcp_wifi_status *ws)
{
	size_t len = sizeof(*ws);

	memset(ws, 0, sizeof(*ws));
	return drvspec(SIOCGDRVSPEC, ESPWIFI_STATUS, ws, &len);
}

static int
status(void)
{
	struct espcp_wifi_status ws;

	if (get_status(&ws) == -1)
		err(EXIT_FAILURE, "%s: status", ifname);
	printf("%s: %s", ifname, ws.ws_state < __arraycount(statename) ?
	    statename[ws.ws_state] : "?");
	if (ws.ws_ssid_len > 0) {
		printf(ws.ws_state == ESPCP_WIFI_CONNECTED ? " with \"" :
		    ", network \"");
		ssid_print(ws.ws_ssid, ws.ws_ssid_len, 0);
		printf("\"");
	}
	if (ws.ws_state == ESPCP_WIFI_CONNECTED) {
		printf(" (%s), channel %u, %d dBm, %s",
		    ether_ntoa((const struct ether_addr *)ws.ws_bssid),
		    ws.ws_channel, ws.ws_rssi, auth_str(ws.ws_auth));
	} else if (ws.ws_reason != 0) {
		printf(", last disconnection reason %u", ws.ws_reason);
	}
	printf("\n");
	return ws.ws_state == ESPCP_WIFI_CONNECTED ? EXIT_SUCCESS : 2;
}

static int
waitfor(int secs)
{
	struct espcp_wifi_status ws;

	for (int i = 0; i <= secs * 4; i++) {
		if (get_status(&ws) == -1)
			err(EXIT_FAILURE, "%s: status", ifname);
		if (ws.ws_state == ESPCP_WIFI_CONNECTED)
			return status();
		usleep(250000);
	}
	status();
	return 2;
}

static int
join(int argc, char **argv)
{
	struct espcp_wifi_config wc;
	char pass[sizeof(wc.wc_password) + 1];
	const char *pw;
	size_t len = sizeof(wc);
	int ch;

	memset(&wc, 0, sizeof(wc));
	wc.wc_minauth = 0xff;
	optreset = 1;
	optind = 1;
	while ((ch = getopt(argc, argv, "a:b:")) != -1) {
		switch (ch) {
		case 'a':
			wc.wc_minauth = auth_parse(optarg);
			break;
		case 'b': {
			const struct ether_addr *ea = ether_aton(optarg);

			if (ea == NULL)
				errx(EXIT_FAILURE, "bad BSSID `%s'", optarg);
			memcpy(wc.wc_bssid, ea, sizeof(wc.wc_bssid));
			wc.wc_flags |= ESPCP_WC_BSSID;
			break;
		}
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc < 1 || argc > 2)
		usage();
	if (strlen(argv[0]) >= sizeof(wc.wc_ssid))
		errx(EXIT_FAILURE, "SSID too long");
	strlcpy(wc.wc_ssid, argv[0], sizeof(wc.wc_ssid));

	if (argc == 2) {
		pw = argv[1];
	} else {
		pw = getpass_r("Passphrase (- for none): ", pass,
		    sizeof(pass));
		if (pw == NULL)
			err(EXIT_FAILURE, "can't read the passphrase");
	}
	if (strcmp(pw, "-") == 0)
		pw = "";
	if (*pw != '\0' && (strlen(pw) < 8 || strlen(pw) > 64))
		errx(EXIT_FAILURE, "a WPA passphrase has 8 to 63 characters "
		    "(or 64 hex digits)");
	strlcpy(wc.wc_password, pw, sizeof(wc.wc_password));
	explicit_memset(pass, 0, sizeof(pass));
	if (wc.wc_minauth == 0xff)
		wc.wc_minauth = *pw != '\0' ? ESPCP_AUTH_WPA2_PSK :
		    ESPCP_AUTH_OPEN;

	if (drvspec(SIOCSDRVSPEC, ESPWIFI_JOIN, &wc, &len) == -1)
		err(EXIT_FAILURE, "%s: join", ifname);
	explicit_memset(&wc, 0, sizeof(wc));
	return EXIT_SUCCESS;
}

int
main(int argc, char **argv)
{
	size_t len = 0;
	int ch;

	while ((ch = getopt(argc, argv, "+i:")) != -1) {
		switch (ch) {
		case 'i':
			ifname = optarg;
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc < 1)
		usage();

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock == -1)
		err(EXIT_FAILURE, "socket");

	if (strcmp(argv[0], "scan") == 0 && argc == 1)
		return scan();
	if (strcmp(argv[0], "status") == 0 && argc == 1)
		return status();
	if (strcmp(argv[0], "join") == 0)
		return join(argc, argv);
	if (strcmp(argv[0], "wait") == 0 && argc <= 2)
		return waitfor(argc == 2 ? atoi(argv[1]) : 30);
	if (strcmp(argv[0], "leave") == 0 && argc == 1) {
		if (drvspec(SIOCSDRVSPEC, ESPWIFI_LEAVE, NULL, &len) == -1)
			err(EXIT_FAILURE, "%s: leave", ifname);
		return EXIT_SUCCESS;
	}
	usage();
}
