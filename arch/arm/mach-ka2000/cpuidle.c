#include <linux/cpuidle.h>
#include <linux/module.h>
#include <asm/system_misc.h>
#include <asm/proc-fns.h>

#include "ka2000.h"

static int ka2000_cpuidle_enter(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	//printk(KERN_INFO "exit idle\n");
	return index;
}

static struct cpuidle_driver ka2000_cpuidle_driver = {
	.name		= "ka2000_cpuidle",
	.owner		= THIS_MODULE,
	.states[0] = {
		.enter		= ka2000_cpuidle_enter,
		.exit_latency	= 1,
		.target_residency = 1,
		.power_usage	= UINT_MAX,
		.flags		= CPUIDLE_FLAG_TIME_VALID,
		.name		= "KA2000 WFI",
		.desc		= "KA2000 ARM WFI",
	},
	.state_count = 1,
};

static int __init ka2000_cpuidle_init(void)
{
	printk(KERN_INFO "%s", __func__);
	return cpuidle_register(&ka2000_cpuidle_driver, NULL);
}
//device_initcall(ka2000_cpuidle_init);
