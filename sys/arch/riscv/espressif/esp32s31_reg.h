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
 * Espressif ESP32-S31 register definitions.
 *
 * The register layouts below are taken from Espressif's Linux, U-Boot and
 * OpenSBI support for the chip (their "integration" esp32s31 branches).
 */

#ifndef _RISCV_ESPRESSIF_ESP32S31_REG_H_
#define _RISCV_ESPRESSIF_ESP32S31_REG_H_

/*
 * Memory map.
 */
#define	S31_HPRAM_BASE		0x2f000000	/* 512KiB HP SRAM (OpenSBI) */
#define	S31_FLASH_BASE		0x40000000	/* flash XIP cache window */
#define	S31_FLASH_SIZE		0x04000000
#define	S31_PSRAM_BASE		0x50000000	/* PSRAM cache window */
#define	S31_PSRAM_SIZE		0x04000000

/*
 * SBI vendor extension implemented by Espressif's OpenSBI.  The extension
 * ID is SBI_EXT_VENDOR_START + (mvendorid & 0xffffff).
 */
#define	S31_SBI_EXT_VENDOR_START	0x09000000
#define	S31_SBI_EXT_VENDOR_MASK		0x00ffffff
#define	S31_SBI_CACHE_WBACK		0	/* a0 = addr, a1 = size */
#define	S31_SBI_CACHE_INVAL		1
#define	S31_SBI_CACHE_WBACK_INVAL	2
#define	S31_SBI_ICACHE_SYNC		3	/* whole caches */
#define	S31_SBI_ICACHE_SYNC_RANGE	4	/* a0 = addr, a1 = size */
#define	S31_SBI_DCACHE_WBACK_ALL	5

/*
 * CLIC, supervisor-mode bank.  One byte each per interrupt id.
 * OpenSBI delegates ids 16-47 (the interrupt matrix outputs) to S-mode.
 */
#define	CLIC_INTIP(id)		(0x1000 + (id) * 4 + 0)	/* pending */
#define	CLIC_INTIE(id)		(0x1000 + (id) * 4 + 1)	/* enable */
#define	CLIC_INTATTR(id)	(0x1000 + (id) * 4 + 2)	/* attributes */
#define	 CLIC_INTATTR_MODE	__BITS(7, 6)
#define	 CLIC_INTATTR_TRIG	__BITS(2, 1)
#define	 CLIC_INTATTR_TRIG_LEVEL 0
#define	 CLIC_INTATTR_SHV	__BIT(0)
#define	CLIC_INTCTL(id)		(0x1000 + (id) * 4 + 3)	/* level/prio */
#define	 CLIC_INTCTL_MAX	0xe0
#define	CLIC_SMODE_FIRST	16
#define	CLIC_SMODE_LAST		47
#define	CLIC_NIDS		64

/*
 * Interrupt matrix: one 32-bit register per peripheral source, holding the
 * CLIC id that source is routed to (0 = not routed).
 */
#define	INTMTX_SRC(src)		((src) * 4)
#define	 INTMTX_SRC_SLOT	__BITS(5, 0)
#define	INTMTX_NSRC		128

/*
 * SYSTIMER: 52-bit counter clocked at 16MHz (40MHz XTAL / 2.5), running
 * through wfi.  We use UNIT0 and alarm TARGET0/COMP0.
 */
#define	SYSTIMER_CONF		0x00
#define	 SYSTIMER_CONF_UNIT0_WORK_EN	__BIT(30)
#define	 SYSTIMER_CONF_TARGET0_WORK_EN	__BIT(24)
#define	SYSTIMER_UNIT0_OP	0x04
#define	 SYSTIMER_UNIT0_UPDATE	__BIT(30)
#define	 SYSTIMER_UNIT0_VALID	__BIT(29)
#define	SYSTIMER_TARGET0_HI	0x1c
#define	SYSTIMER_TARGET0_LO	0x20
#define	SYSTIMER_TARGET0_CONF	0x34
#define	 SYSTIMER_TARGET0_UNIT_SEL	__BIT(31)
#define	 SYSTIMER_TARGET0_PERIOD_MODE	__BIT(30)
#define	SYSTIMER_UNIT0_VALUE_HI	0x40
#define	SYSTIMER_UNIT0_VALUE_LO	0x44
#define	SYSTIMER_COMP0_LOAD	0x50
#define	SYSTIMER_INT_ENA	0x64
#define	SYSTIMER_INT_RAW	0x68
#define	SYSTIMER_INT_CLR	0x6c
#define	SYSTIMER_INT_ST		0x70
#define	 SYSTIMER_INT_TARGET0	__BIT(0)
#define	SYSTIMER_HI_MASK	0x000fffff
#define	SYSTIMER_FREQ		16000000

/*
 * UART (ESP32-S3 compatible register layout).
 */
#define	UART_FIFO		0x00
#define	UART_INT_RAW		0x04
#define	UART_INT_ST		0x08
#define	UART_INT_ENA		0x0c
#define	UART_INT_CLR		0x10
#define	 UART_INT_RXFIFO_FULL	__BIT(0)
#define	 UART_INT_TXFIFO_EMPTY	__BIT(1)
#define	 UART_INT_PARITY_ERR	__BIT(2)
#define	 UART_INT_FRM_ERR	__BIT(3)
#define	 UART_INT_RXFIFO_OVF	__BIT(4)
#define	 UART_INT_BRK_DET	__BIT(7)
#define	 UART_INT_RXFIFO_TOUT	__BIT(8)
#define	UART_CLKDIV		0x14
#define	UART_STATUS		0x1c
#define	 UART_STATUS_RXFIFO_CNT	__BITS(9, 0)
#define	 UART_STATUS_TXFIFO_CNT	__BITS(25, 16)
#define	UART_CONF0		0x20
#define	UART_CONF1		0x24
#define	 UART_CONF1_RXFIFO_FULL_THRHD	__BITS(9, 0)
#define	 UART_CONF1_TXFIFO_EMPTY_THRHD	__BITS(19, 10)
#define	UART_REG_UPDATE		0x98
#define	 UART_REG_UPDATE_BUSY	__BIT(0)
#define	UART_FIFO_SIZE		128
#define	UART_TX_FIFO_MAX	127

/*
 * USB-Serial/JTAG controller: a fixed-function CDC-ACM device (plus a
 * JTAG bridge) on the chip's full-speed USB pins.  Endpoint 1 carries
 * the serial data through two 64-byte FIFOs.
 */
#define	USJ_EP1			0x00	/* RX/TX FIFO byte */
#define	USJ_EP1_CONF		0x04
#define	 USJ_EP1_CONF_WR_DONE	__BIT(0)	/* send what's in the FIFO */
#define	 USJ_EP1_CONF_IN_FREE	__BIT(1)	/* TX FIFO has room */
#define	 USJ_EP1_CONF_OUT_AVAIL	__BIT(2)	/* RX FIFO has data */
#define	USJ_INT_RAW		0x08
#define	USJ_INT_ST		0x0c
#define	USJ_INT_ENA		0x10
#define	USJ_INT_CLR		0x14
#define	 USJ_INT_OUT_RECV_PKT	__BIT(2)	/* host sent a packet */
#define	 USJ_INT_IN_EMPTY	__BIT(3)	/* host took our packet */
#define	USJ_IN_EP1_ST		0x2c
#define	 USJ_IN_EP1_WR_ADDR	__BITS(8, 2)
#define	USJ_OUT_EP1_ST		0x3c
#define	 USJ_OUT_EP1_REC_DATA_CNT __BITS(22, 16)
#define	USJ_FIFO_SIZE		64

#endif /* _RISCV_ESPRESSIF_ESP32S31_REG_H_ */
