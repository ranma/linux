/*
 * Interrupt handler for KeyASIC Ka2000 boards.
 *
 * Copyright (C) 2013 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <asm/mach/irq.h>
#include "ka2000.h"

#define INTC_REG(x) KA2000_ADDR(INTC_BASE + x)
#define INTC_REG64(x) KA2000_ADDR64(INTC_BASE + x)
#define SRC	INTC_REG64(0x00)
#define MODE	INTC_REG64(0x08)
#define MASK	INTC_REG64(0x10)
#define PEND	INTC_REG64(0x18)
#define PRIO	INTC_REG64(0x20)
#define PRO	INTC_REG64(0x28)
#define CLEAR	INTC_REG(0x30)
#define OFFSET	INTC_REG(0x34)

static void ka2000_irq_ack(struct irq_data *d)
{
	*PEND = (u64)1 << d->irq;
	*CLEAR = 0;
}

static void ka2000_irq_mask(struct irq_data *d)
{
	*MASK |= (u64)1 << d->irq;
}

static void ka2000_irq_unmask(struct irq_data *d)
{
	*MASK &= ~((u64)1 << d->irq);
}

static struct irq_chip ka2000_irq_chip = {
	.name		= "KA2000",
	.irq_ack	= ka2000_irq_ack,
	.irq_mask	= ka2000_irq_mask,
	.irq_unmask	= ka2000_irq_unmask,
};


void __init ka2000_init_irq(void)
{
	int i = 0;

	early_printk("ka2000_init_irq\n");

	/* Disable all interrupts */
	*MASK = ~0;

	/* Ack all still pending interrupts */
	*PEND = ~0;
	*CLEAR = 0;

	/* Route all sources to IRQ instead of FIQ */
	*PRIO = 0;

        /* Default to all level triggered */
	for(i = 0; i < ARCH_NR_IRQS; i++) {
		irq_set_chip_and_handler(i, &ka2000_irq_chip,
					 handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	/* Test: Enable all interrupts */
	*MASK = 0;
}
