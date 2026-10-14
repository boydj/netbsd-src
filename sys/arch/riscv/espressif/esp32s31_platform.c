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
 * Espressif ESP32-S31 platform support.
 *
 * The chip has two RV32IMAFC(+Zbb) harts with an Sv32 MMU, 512KiB of
 * internal SRAM (used by OpenSBI) and external PSRAM / NOR flash behind a
 * cache, mapped at 0x50000000 / 0x40000000.  The boot chain is ROM ->
 * U-Boot SPL -> OpenSBI -> U-Boot -> kernel, see sys/arch/riscv/espressif/
 * README.esp32s31.
 *
 * Things that differ from a "standard" RISC-V platform:
 *  - interrupts go through a CLIC in CLIC mode (options RISCV_CLIC);
 *  - the time CSR is emulated by OpenSBI and stops in wfi: the SYSTIMER
 *    peripheral is used for timekeeping;
 *  - the caches are outside of the hart, non-coherent, and not reached by
 *    fence.i: instruction cache maintenance is done by OpenSBI through a
 *    vendor SBI extension.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_platform.h>

#include <uvm/uvm_extern.h>
#include <uvm/pmap/pmap_devmap.h>

#include <machine/machdep.h>
#include <machine/sbi.h>

#include <riscv/fdt/riscv_fdtvar.h>

#include <riscv/espressif/esp32s31_reg.h>
#include <riscv/espressif/esp32s31_var.h>

extern struct bus_space riscv_generic_bs_tag;

static long esp32s31_sbi_vendor_eid = -1;

long
esp32s31_sbi_vendor_call(int fid, register_t a0, register_t a1)
{
	if (esp32s31_sbi_vendor_eid == -1)
		return SBI_ERR_NOT_SUPPORTED;

	struct sbiret ret = sbi_call(esp32s31_sbi_vendor_eid, fid,
	    a0, a1, 0, 0, 0, 0);

	return ret.error;
}

/*
 * Make code written through the D-cache visible to instruction fetch.
 * Only the PSRAM and flash windows are cached; the SBI call rejects
 * anything else.
 */
static void
esp32s31_icache_sync_range(paddr_t pa, psize_t len)
{
	if (pa >= S31_PSRAM_BASE && pa - S31_PSRAM_BASE < S31_PSRAM_SIZE) {
		(void)esp32s31_sbi_vendor_call(S31_SBI_ICACHE_SYNC_RANGE,
		    pa, len);
	}
}

static void
esp32s31_icache_sync_all(void)
{
	(void)esp32s31_sbi_vendor_call(S31_SBI_ICACHE_SYNC, 0, 0);
}

static const struct pmap_devmap *
esp32s31_platform_devmap(void)
{
	static const struct pmap_devmap devmap[] = {
		/* CLIC (M and S banks) */
		DEVMAP_ENTRY(VM_KERNEL_IO_BASE + 0x0000000, 0x10800000,
		    0x400000),
		/* HP peripherals: UART0, SYSTIMER, ... */
		DEVMAP_ENTRY(VM_KERNEL_IO_BASE + 0x0400000, 0x20000000,
		    0x400000),
		/* HP peripherals: INTMTX, TEE/APM, LP AON, ... */
		DEVMAP_ENTRY(VM_KERNEL_IO_BASE + 0x0800000, 0x20400000,
		    0x400000),
		/* HP peripherals: TRNG, ... */
		DEVMAP_ENTRY(VM_KERNEL_IO_BASE + 0x0c00000, 0x20800000,
		    0x400000),
		DEVMAP_ENTRY_END
	};

	return devmap;
}

static void
esp32s31_platform_bootstrap(void)
{
	riscv_fdt_cpu_bootstrap();

	/* Find the vendor SBI extension for cache maintenance. */
	struct sbiret ret = sbi_get_mvendorid();
	if (ret.error == SBI_SUCCESS) {
		const long eid = S31_SBI_EXT_VENDOR_START +
		    (ret.value & S31_SBI_EXT_VENDOR_MASK);

		ret = sbi_probe_extension(eid);
		if (ret.error == SBI_SUCCESS && ret.value != 0)
			esp32s31_sbi_vendor_eid = eid;
	}
	if (esp32s31_sbi_vendor_eid != -1) {
		riscv_icache_sync_range_hook = esp32s31_icache_sync_range;
		riscv_icache_sync_all_hook = esp32s31_icache_sync_all;
	}

	/* delay(9) works from here on. */
	const int phandle = of_find_bycompat(OF_finddevice("/"),
	    "espressif,esp32s31-systimer");
	bus_addr_t addr;
	if (phandle > 0 && fdtbus_get_reg(phandle, 0, &addr, NULL) == 0)
		esp_systimer_early_init(&riscv_generic_bs_tag, addr);
}

static u_int
esp32s31_platform_uart_freq(void)
{
	return 40000000;
}

static const struct fdt_platform esp32s31_platform = {
	.fp_devmap = esp32s31_platform_devmap,
	.fp_bootstrap = esp32s31_platform_bootstrap,
	.fp_uart_freq = esp32s31_platform_uart_freq,
	.fp_mpstart = riscv_fdt_cpu_mpstart,
};

FDT_PLATFORM(esp32s31, "espressif,esp32s31", &esp32s31_platform);
