/*
 * SPI driver for KeyASIC Ka2000 boards.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>

#include <asm/io.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/io.h>

#include "ka2000.h"

#define SSP_REG(x) KA2000_ADDR(KA_SSI_BASE + x)
#define PRE	SSP_REG(0x00)
#define CON	SSP_REG(0x04)
#define   CPHA		(1 << 0)
#define   CPOL		(1 << 1)
#define   SMOD_PO	(0 << 6)
#define   SMOD_INT	(1 << 6)
#define   SMOD_DMA	(2 << 6)
#define   ENSCK		(1 << 8)
#define   CS0		(1 << 9)
#define   MSTR		(1 << 10)
#define   SPIMW		(1 << 11)
#define STA	SSP_REG(0x08)
#define   READY		(1 << 0)
#define   TMOD_BYTE	(0 << 4)
#define   TMOD_HWORD	(1 << 4)
#define   TMOD_WORD	(2 << 4)
#define TDAT	SSP_REG(0x0c)
#define RDAT	SSP_REG(0x10)

#define MULTE 					(0x1 << 14)
#define MIMSK 					(0x1 << 13)
#define TAGD  					(0x1 << 12)
#define DRD   					(0x1 << 5)
#define DTD   					(0x1 << 4)
#define CSLV  					(0x1 << 3)
#define KEEP  					(0x1 << 2)
#define TMOD_MARK  				(0x2 << 4)

struct ka2000_ssp_spi {
	struct spi_bitbang bitbang;

	void __iomem *membase;
	unsigned char val;
	u32 con;
	struct ka2000_ssp_spi_info *info;
	struct platform_device *dev;
};

static void ka2000_ssp_spi_wait_ready(struct ka2000_ssp_spi *sp)
{
	int timeout = 40000;
	while((*STA & READY) == 0)
	{
		if (timeout-- <= 0) {
			printk_once(KERN_INFO "%s timeout, STA=%08x\n", __func__, *STA);
			break;
		}
		barrier();
	}
}

static u32 ka2000_ssp_spi_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	struct ka2000_ssp_spi *sp = spi_master_get_devdata(spi->master);
	u32 ret;
	ka2000_ssp_spi_wait_ready(sp);
	*TDAT = word;
	ka2000_ssp_spi_wait_ready(sp);
	ret = *RDAT;
	printk_once(KERN_INFO "%s %02x %d: %02x\n", __func__, word, bits, ret);
	return ret;
}

static u32 ka2000_ssp_spi_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	printk_once(KERN_INFO "%s\n", __func__);
	return 0;
}

static u32 ka2000_ssp_spi_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	printk_once(KERN_INFO "%s\n", __func__);
	return 0;
}

static u32 ka2000_ssp_spi_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	struct ka2000_ssp_spi *sp = spi_master_get_devdata(spi->master);
	u32 ret;
	ka2000_ssp_spi_wait_ready(sp);
	*TDAT = word;
	ka2000_ssp_spi_wait_ready(sp);
	ret = *RDAT;
	printk_once(KERN_INFO "%s %02x %d: %02x\n", __func__, word, bits, ret);
	return ret;
}

static void ka2000_ssp_spi_chipselect(struct spi_device *dev, int value)
{
	struct ka2000_ssp_spi *sp = spi_master_get_devdata(dev->master);

	if (value)
		sp->con |= CS0;
	else
		sp->con &= ~CS0;
	*CON = sp->con;
}

static int ka2000_ssp_spi_probe(struct platform_device *dev)
{
	struct resource	*r;
	struct spi_master *master;
	struct ka2000_ssp_spi *sp;
	int ret;

	master = spi_alloc_master(&dev->dev, sizeof(struct ka2000_ssp_spi));
	if (master == NULL) {
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err0;
	}

	sp = spi_master_get_devdata(master);

	platform_set_drvdata(dev, sp);
	sp->info = dev->dev.platform_data;

	/* setup spi bitbang adaptor */
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.master->bus_num = 0;
	sp->bitbang.master->num_chipselect = 1;
	sp->bitbang.chipselect = ka2000_ssp_spi_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = ka2000_ssp_spi_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = ka2000_ssp_spi_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = ka2000_ssp_spi_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = ka2000_ssp_spi_txrx_mode3;

	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		ret = -ENOENT;
		goto err1;
	}
	sp->membase = ioremap(r->start, resource_size(r));
	if (!sp->membase) {
		ret = -ENXIO;
		goto err1;
	}
	sp->val = 0;
	*PRE = (192000000 / 30000000) - 1;
	*CON = sp->con = SPIMW + MSTR + ENSCK + SMOD_PO + CS0 + CPOL + CPHA;
	*STA = TMOD_BYTE;
	printk(KERN_INFO "SPI: CON=%08x PRE=%08x STA=%08x\n", *CON, *PRE, *STA);

	ret = spi_bitbang_start(&sp->bitbang);
	if (ret < 0)
		goto err2;

	printk(KERN_INFO "Adding platform devices...\n");
	if (sp->info) {
		struct spi_board_info *flash = sp->info->flash_device;
		printk(KERN_INFO "Adding flash chip %s\n", flash->modalias);
		spi_new_device(master, flash);
	}
	return 0;

err2:
	iounmap(sp->membase);
err1:
	spi_master_put(sp->bitbang.master);
err0:
	return ret;
}

static int ka2000_ssp_spi_remove(struct platform_device *dev)
{
	struct ka2000_ssp_spi *sp = platform_get_drvdata(dev);

	iounmap(sp->membase);
	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);
	return 0;
}

static struct platform_driver ka2000_ssp_spi_drv = {
	.probe		= ka2000_ssp_spi_probe,
	.remove		= ka2000_ssp_spi_remove,
	.driver		= {
		.name	= "ka2000_ssp_spi",
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(ka2000_ssp_spi_drv);
