/*
 * KeyASIC Ka2000 IO address definitions
 *
 * Copied from include/asm/arm/arch-omap/io.h
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

/*  -> Virtual IO = 0xfe000000 */
#define IO_PHYS		0xa0000000
#define IO_OFFSET	0x5e000000
#define IO_SIZE		0x00100000

#define IO_VIRT		(IO_PHYS + IO_OFFSET)
#define io_v2p(va)	((va) - IO_OFFSET)
#define io_p2v(pa)	((pa) + IO_OFFSET)

/*
 * Functions to access the KeyASIC Ka2000 IO region
 *
 * NOTE: - Use ka2000_read/write[bwl] for physical register addresses
 *	 - Use __raw_read/write[bwl]() for virtual register addresses
 *	 - Use IO_ADDRESS(phys_addr) to convert registers to virtual addresses
 *	 - DO NOT use hardcoded virtual addresses to allow changing the
 *	   IO address space again if needed
 */
#define ka2000_readb(a)	__raw_readb(IO_ADDRESS(a))
#define ka2000_readw(a)	__raw_readw(IO_ADDRESS(a))
#define ka2000_readl(a)	__raw_readl(IO_ADDRESS(a))

#define ka2000_writeb(v, a)	__raw_writeb(v, IO_ADDRESS(a))
#define ka2000_writew(v, a)	__raw_writew(v, IO_ADDRESS(a))
#define ka2000_writel(v, a)	__raw_writel(v, IO_ADDRESS(a))

#endif /* __ASM_ARCH_IO_H */
