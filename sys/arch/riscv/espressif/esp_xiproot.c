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
 * ESP32-S31 root file system in SPI flash.
 *
 * The boot firmware maps the SPI flash into the XIP window at 0x40000000
 * (flash offset 0x100000 and up) through the cache.  A region of it holding
 * a file system image is handed to md(4) as the root memory disk, the same
 * way Espressif's Linux uses an "mtd-rom" root: no copy into PSRAM, and the
 * root is read-only (the PMA makes the window read/execute only, so it must
 * not be mounted read/write).
 *
 *	rootfs: flash@40700000 {
 *		compatible = "espressif,esp32s31-xip-rootfs";
 *		reg = <0x40700000 0x00900000>;
 *	};
 */

#include "opt_md.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>
#include <dev/md.h>

static int	esp_xiproot_match(device_t, cfdata_t, void *);
static void	esp_xiproot_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(esp_xiproot, 0,
	esp_xiproot_match, esp_xiproot_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "espressif,esp32s31-xip-rootfs" },
	DEVICE_COMPAT_EOL
};

static int
esp_xiproot_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
esp_xiproot_attach(device_t parent, device_t self, void *aux)
{
	struct fdt_attach_args * const faa = aux;
	bus_space_handle_t bsh;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(faa->faa_phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": %" PRIuBUSSIZE " KB file system in flash\n",
	    size / 1024);

#if defined(MEMORY_DISK_DYNAMIC) && defined(MEMORY_DISK_IS_ROOT)
	if (bus_space_map(faa->faa_bst, addr, size,
	    BUS_SPACE_MAP_CACHEABLE | BUS_SPACE_MAP_PREFETCHABLE, &bsh) != 0) {
		aprint_error_dev(self, "couldn't map the XIP window\n");
		return;
	}
	md_root_setconf((char *)bus_space_vaddr(faa->faa_bst, bsh), size);
	aprint_normal_dev(self, "root file system (read-only) on md0\n");
#else
	(void)bsh;
#endif
}
