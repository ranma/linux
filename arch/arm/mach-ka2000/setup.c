/*
 * Under GPLv2
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/pm.h>
#include <linux/of_address.h>
#include <linux/pinctrl/machine.h>
#include <linux/time.h>

#include <asm/setup.h>
#include <asm/system_misc.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/io.h>

#include "ka2000.h"

#define WDT_REG(x) KA2000_ADDR(WDT_BASE + x)
#define WDTCON		WDT_REG(0x0)
#define WDTDAT		WDT_REG(0x4)
#define WDTCNT		WDT_REG(0x8)

static struct map_desc ka2000_io_desc[] __initdata = {
	{	/* Peripheral IO space */
		.virtual	= (unsigned long)io_p2v(KA_REGIF_BASE),
		.pfn		= __phys_to_pfn(KA_REGIF_BASE),
		.length		= KA_REGIF_SIZE,
		.type		= MT_DEVICE
	},
};

void __init ka2000_map_io(void)
{
	early_print("ka2000_map_io\n");
	iotable_init(ka2000_io_desc, ARRAY_SIZE(ka2000_io_desc));
}

static void ka2000_pm_idle_noop(void)
{
	// FIXME: Does not wake up from arm wait for interrupt state.
	// Looks like the PWM block is not clocked during sleep?
	// Waking up by MMC interrupts works fine.
}

void __init ka2000_init_early(void)
{
	early_print("ka2000_init_early\n");
	arm_pm_idle = ka2000_pm_idle_noop;
	ka2000_early_init_clocks();
}

void __init ka2000_of_init(void)
{
}

void __init ka2000_dt_init_early(void)
{
}

void __init ka2000_initialize(unsigned long main_clock)
{
	printk(KERN_INFO "%s\n", __func__);
}

void ka2000_restart(enum reboot_mode mode, const char *cmd)
{
	printk(KERN_WARNING "%s\n", __func__);
	*WDTDAT = *WDTCNT = 0x1000;
	barrier();
	*WDTCON = 0xffffffff;
	barrier();
	for (;;);
}

void __init ka2000_sys_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
}
