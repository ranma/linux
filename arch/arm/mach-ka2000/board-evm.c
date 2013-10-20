/*
 * KeyASIC KA2000 EVM board support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/serial_8250.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <mach/io.h>

#include "ka2000.h"
#include "ka2000-sdio.h"

static struct mtd_partition ka2000_spi_flash_partitions[] = {
	{
		.name   = "u-boot",
		.offset = 0x000000,
		.size   = 0x080000,
	}, {
		.name   = "rootfs", /* jffs2 */
		.offset = 0x080000,
		.size   = 0x180000,
	}, {
		.name   = "kernel",
		.offset = 0x200000,
		.size   = 0x300000,
	}, {
		.name	= "Ramdisk",
		.offset = 0x500000,
		.size	= 0x300000,
	}, {
		.name	= "JFFS2_Kernel_RFS",
		.offset = 0x080000,
		.size	= 0x780000,
	}, {
		.name	= "Kernel_RFS",
		.offset = 0x200000,
		.size	= 0x600000,
	},
};

static struct flash_platform_data ka2000_spi_flashdata = {
	.name="mx25l6405d",
	.type="mx25l6405d",
	.parts=ka2000_spi_flash_partitions,
	.nr_parts=ARRAY_SIZE(ka2000_spi_flash_partitions),
};

static struct spi_board_info spi_nor_flash_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &ka2000_spi_flashdata,
		.max_speed_hz	= KA2000_OSC_CLOCK,
		.bus_num	= 0,
		.chip_select	= 0,
	},
};

static struct resource ka2000_uart_resources = {
	.start	= KA2000_UART_BASE_PHYS,
	.end	= KA2000_UART_BASE_PHYS + 0x0fff,
	.flags	= IORESOURCE_MEM,
};

static struct plat_serial8250_port ka2000_uart_data[] = {
	{
		.mapbase	= KA2000_UART_BASE_PHYS,
		.membase	= (char *)io_p2v(KA2000_UART_BASE_PHYS),
		.irq		= IRQ_KA2000_UART,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= KA2000_UART_CLOCK,
	},
	{ },
};

static struct platform_device ka2000_uart = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= ka2000_uart_data,
	},
	.num_resources	= 1,
	.resource	= &ka2000_uart_resources,
};

static struct ka2000_ssp_spi_info ka2000_ssp_spi_data = {
	.flash_device = spi_nor_flash_info,
};

static struct resource ka2000_ssp_spi_resources[] = {
	{
		.start	= KA2000_SSP_BASE_PHYS,
		.end	= KA2000_SSP_BASE_PHYS + 0x0fff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ka2000_ssp_spi  = {
	.name		= "ka2000_ssp_spi",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ka2000_ssp_spi_resources),
	.resource	= ka2000_ssp_spi_resources,
	.dev			= {
		.platform_data	= &ka2000_ssp_spi_data,
	},
};

static struct resource ka2000_sdio_resources[] = {
	{
		.start	= KA2000_SDIO_BASE_PHYS,
		.end	= KA2000_SDIO_BASE_PHYS + 0x0fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start = IRQ_sdio_buf_tran_finish_int, // 32,
		.end   = IRQ_sdio_buf_tran_finish_int, // 32,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sdio_data_bound_int, // 33,
		.end   = IRQ_sdio_data_bound_int, // 33,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sdio_tran_done_int, // 34,
		.end   = IRQ_sdio_tran_done_int, // 34,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sdio_cmd_done_int, // 35,
		.end   = IRQ_sdio_cmd_done_int, // 35,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sdio_card_error_int, // 36,
		.end   = IRQ_sdio_card_error_int, // 36,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sdio_dma_int, // 37,
		.end   = IRQ_sdio_dma_int, // 37,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sdio_card_int, // 38,
		.end   = IRQ_sdio_card_int, // 38,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ka2000_sd_resources[] = {
	{
		.start	= KA2000_SD_BASE_PHYS,
		.end	= KA2000_SD_BASE_PHYS + 0x0fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start = IRQ_sd_buf_tran_finish_int, // 32,
		.end   = IRQ_sd_buf_tran_finish_int, // 32,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sd_data_bound_int, // 33,
		.end   = IRQ_sd_data_bound_int, // 33,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sd_tran_done_int, // 34,
		.end   = IRQ_sd_tran_done_int, // 34,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sd_cmd_done_int, // 35,
		.end   = IRQ_sd_cmd_done_int, // 35,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sd_card_error_int, // 36,
		.end   = IRQ_sd_card_error_int, // 36,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sd_dma_int, // 37,
		.end   = IRQ_sd_dma_int, // 37,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_sd_card_int, // 38,
		.end   = IRQ_sd_card_int, // 38,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ka_sdio_platdata ka_sdio_platdata = {
	.max_width	= 4,
	.clocks 	= NULL,
	.irq_num	= 7,
};

static u64 ka_sdio_dmamask = 0xffffffff;

static struct platform_device ka2000_sd_mci  = {
	.name		= "ka2000_mmc",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(ka2000_sd_resources),
	.resource	= ka2000_sd_resources,
	.dev		= {
		.dma_mask		= &ka_sdio_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ka_sdio_platdata,
	},
};

static struct platform_device ka2000_sdio_mci  = {
	.name		= "ka2000_mmc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(ka2000_sdio_resources),
	.resource	= ka2000_sdio_resources,
	.dev		= {
		.dma_mask		= &ka_sdio_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ka_sdio_platdata,
	},
};


static struct platform_device *ka2000_evm_devices[] __initdata = {
	&ka2000_uart,
	&ka2000_ssp_spi,
	&ka2000_sd_mci,
	&ka2000_sdio_mci,
};

static void __init ka2000_update_devices(void)
{
	struct clk *hclk = clk_get(NULL, "HCLK");
	u32 hclk_rate;
	if (IS_ERR(hclk)) {
		printk(KERN_WARNING "Could not get HCLK, failed to update device clocks.\n");
		return;
	}
	hclk_rate = clk_get_rate(hclk);
	clk_put(hclk);
	printk(KERN_INFO "Updating board devices with hclk=%dKHz\n", hclk_rate / 1000);
	ka2000_uart_data[0].uartclk = hclk_rate;
}

static void __init ka2000_evm_init(void)
{
	printk("ka2000_evm_init\n");
	ka2000_sys_init();
	ka2000_init_clocks();
	ka2000_update_devices();

	platform_add_devices(ka2000_evm_devices, ARRAY_SIZE(ka2000_evm_devices));
	//spi_register_board_info(spi_nor_flash_info, ARRAY_SIZE(spi_nor_flash_info));
	printk("ka2000_evm_init done\n");
}

MACHINE_START(KA2000_EVM, "KeyASIC Ka2000 EVM")
	.map_io		= ka2000_map_io,
	.init_early	= ka2000_init_early,
	.init_irq	= ka2000_init_irq,
	.init_time	= ka2000_init_time,
	.init_machine	= ka2000_evm_init,
	.nr_irqs	= ARCH_NR_IRQS,
	.restart	= ka2000_restart,
MACHINE_END
