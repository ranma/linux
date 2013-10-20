
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>

#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/io.h>

#include "ka2000.h"

#define WDT_REG(x) KA2000_ADDR(WDT_BASE + x)
#define WDTCON	WDT_REG(0x0)
#define WDTDAT	WDT_REG(0x4)
#define WDTCNT	WDT_REG(0x8)

#define PWM_REG(x) KA2000_ADDR(PWM_BASE + x)
#define TCFG0                PWM_REG(0x00)
#define   T1T2_PRESC_MASK    0x00ff0000
#define   T1T2_PRESC_SHIFT   16
#define TCFG1                PWM_REG(0x04)
#define   T2_ACK             0x02000000
#define   T1_ACK             0x01000000
#define   T0_ACK             0x00100000

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

#define UART_REG(x) KA2000_ADDR(KA_UART_BASE + x)
#define LCR	UART_REG(0x0c)
#define LCR	UART_REG(0x0c)
#define LATCHL	UART_REG(0x00)
#define LATCHH	UART_REG(0x04)

#define CLOCK_REG(x) KA2000_ADDR(KA_SCU_BASE + x)
#define CLK_SRC_CTL	CLOCK_REG(0x00)
#define   HCLK_DIV_MASK		0x0000000e
#define   HCLK_DIV_SHIFT	1
#define   HCLK_DIV_WIDTH	3
#define   ARM_SRC_MASK		0x00000001
#define   ARM_SRC_WIDTH		1
#define   ARM_SRC_SHIFT		0
#define SRC_OSC		0
#define SRC_PLL		1
#define PLL_FREQ_SEL1	CLOCK_REG(0x04)
#define   SPI_DIV_MASK		0x001f0000
#define   SPI_DIV_SHIFT		16
#define   ARM_DIV_MASK		0x0000001f
#define   ARM_DIV_SHIFT		0
#define PLL_FREQ_SEL2	CLOCK_REG(0x08)
#define SYSTEM_CTL1	CLOCK_REG(0x0c)
#define   PLL_MULT_MASK		0x00003f00
#define   PLL_MULT_SHIFT	8
#define   PLL_DIV_MASK		0x0000c000
#define   PLL_DIV_SHIFT		14
#define   PLL_POWERDOWN		0x00000002
#define   PLL_RESET		0x00000001

#define GET_MASKED(x, mask, shift) (((x) & mask) >> shift)
#define GET_MASKED_OFS(x, mask, shift, ofs) ((((x) & mask) >> shift) + ofs)

enum ka2000_clks {
	CLK_OSC, CLK_PLL, CLK_ARM, CLK_HCLK, CLK_MAX
};

static struct clk *clocks[CLK_MAX];

static DEFINE_SPINLOCK(clk_lock);

static u32 ka2000_osc_clock = KA2000_OSC_CLOCK;

u32 ka2000_get_clk(const char* name, u32 base, u32 osc, u32 *srcreg, int src_shift,
		u32 *divreg, int div_shift, u32 div_mask)
{
	int div = GET_MASKED_OFS(*divreg, div_mask, div_shift, 1);
	int src_sel = SRC_PLL;
	if (srcreg)
		src_sel = (*srcreg >> src_shift) & 1;

	if (src_sel == SRC_OSC)
		base = osc;

	printk(KERN_INFO "%s clk=%dk base=%dk div=%d\n", name, base / div / 1000, base / 1000, div);

	return base / div;
}

u32 ka2000_get_pll_clk(u32 osc_clk)
{
	int prediv = 1 << GET_MASKED(*SYSTEM_CTL1, PLL_DIV_MASK, PLL_DIV_SHIFT);
	int mult = GET_MASKED_OFS(*SYSTEM_CTL1, PLL_MULT_MASK, PLL_MULT_SHIFT, 1);
	int pll =  osc_clk / prediv * mult;
	//printk(KERN_INFO "osc=%d pll prediv=%d mult=%d\n pll=%d", osc_clk, prediv, mult, pll);
	if (*SYSTEM_CTL1 & (PLL_POWERDOWN || PLL_RESET))
		return 0;
	return pll;
}

static void set_uart_clk(u32 hclk, u32 baudrate)
{
	u32 divisor = (hclk / 16) / baudrate;
	if (divisor > 0xffff) {
		printk(KERN_INFO "Can't set baudrate %d, divisor %d is too high!\n",
			baudrate, divisor);
		return;
	}

	*LCR |= 0x80;
	barrier();
	*LATCHH = divisor >> 8;
	*LATCHL = divisor & 0xff;
	barrier();
	*LCR &= ~0x80;
	barrier();
}

static u32 hclk_osc = 24000000;
static u32 hclk_pll = 96000000;

void ka2000_pm_idle(void)
{
	/* FIXME: This is borken, see comment in setup.c */
	struct ka2000_clk_state clk_state;
	int switch_to_osc = 1;
	u32 nwait = 0;

	timer_debug = 1;
	printk(KERN_INFO "%s %016llx %016llx %016llx\n", __func__, *SRC, *PEND, *MASK);
//	set_uart_clk(hclk_pll, 38400);
	/* save registers */
	clk_state.ctl = *CLK_SRC_CTL;
	clk_state.sys1 = *SYSTEM_CTL1;
	barrier();

	if (switch_to_osc) {
		/* switch to OSC */
		*CLK_SRC_CTL = 0x10c22610;
		barrier();
		*SYSTEM_CTL1 = 1;
		barrier();
	}

#if 1
	*WDTCON = 4;
	barrier();
#endif

	asm(" mov	r0, #0\n\t"
	"mrc	p15, 0, r1, c1, c0, 0		@ Read control register\n\t"
	"mcr	p15, 0, r0, c7, c10, 4		@ Drain write buffer\n\t"
	"bic	r2, r1, #1 << 12\n\t"
	"mrs	r3, cpsr			@ Disable FIQs while Icache\n\t"
	"orr	ip, r3, #0x40			@ is disabled\n\t"
	"msr	cpsr_c, ip\n\t"
	"mcr	p15, 0, r2, c1, c0, 0		@ Disable I cache\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"mcr	p15, 0, r0, c7, c0, 4		@ Wait for interrupt\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"mcr	p15, 0, r1, c1, c0, 0		@ Restore ICache enable\n\t"
	"msr	cpsr_c, r3			@ Restore FIQ state\n\t");

	if (switch_to_osc) {
		*SYSTEM_CTL1 = clk_state.sys1 | PLL_POWERDOWN | PLL_RESET;
		barrier(); //udelay(1);
		*SYSTEM_CTL1 = clk_state.sys1 | PLL_RESET;
		barrier(); //udelay(200);
		*SYSTEM_CTL1 = clk_state.sys1;
		barrier();

		while ((*CLK_SRC_CTL & 0x10000000) ==  0) {
			nwait++;
			barrier();
		}

		*CLK_SRC_CTL = clk_state.ctl;
		barrier();
	}

	printk(KERN_INFO "%s %016llx %016llx %016llx (nwait=%d)\n", __func__, *SRC, *PEND, *MASK, nwait);
	*WDTCON = 0;
	barrier();
}

static void switch_pll(int prescaler, int multiplier, int hclk_divisor, int pll_en, int verbose)
{
	u32 prediv = 1 << prescaler;
	u32 arm_clk = ka2000_osc_clock * multiplier / prediv;
	u32 hclk = arm_clk / hclk_divisor;
	u32 pllctl = (prescaler << PLL_DIV_SHIFT) |
		((multiplier - 1) << PLL_MULT_SHIFT);
	u32 srcctl, nwait;

	BUG_ON(prescaler < 0 || prescaler > 3);
	BUG_ON(multiplier < 1 || multiplier > 32);
	BUG_ON(hclk_divisor < 1 || hclk_divisor > 8);

	srcctl = *CLK_SRC_CTL & ~(HCLK_DIV_MASK|ARM_SRC_MASK);
	srcctl |= SRC_PLL << ARM_SRC_SHIFT;
	srcctl |= (hclk_divisor - 1) << HCLK_DIV_SHIFT;

	if (verbose)
		printk(KERN_INFO "Switching to %dKHz ARM clock, %dKHz HCLK, SRC_CTL=%08x PLL_CTL=%04x.\n",
			arm_clk / 1000, hclk / 1000, srcctl, pllctl);

	barrier();
	*CLK_SRC_CTL &= ~(HCLK_DIV_MASK|ARM_SRC_MASK);
	barrier();
	if (verbose) {
		set_uart_clk(ka2000_osc_clock, 38400);
		printk(KERN_INFO "Now on OSC clock. CLK_SRC_CTL=%08x\n", *CLK_SRC_CTL);
	}

	*SYSTEM_CTL1 = pllctl | PLL_RESET | PLL_POWERDOWN;
	udelay(1);
	*SYSTEM_CTL1 = pllctl | PLL_RESET;
	if (!pll_en)
		return;

	udelay(20);
	*SYSTEM_CTL1 = pllctl;
	barrier();

	nwait = 0;
	while ((*CLK_SRC_CTL & 0x10000000) ==  0) {
		nwait++;
		barrier();
	}
	if (verbose)
		printk(KERN_INFO "Waited %d loops for PLL startup\n", nwait);
	*CLK_SRC_CTL = srcctl;
}

static struct clk *ka2000_register_osc(void)
{
	return clk_register_fixed_rate(NULL, "OSC", NULL, CLK_IS_ROOT, ka2000_osc_clock);
}

static unsigned long ka2000_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	return ka2000_get_pll_clk(parent_rate);
}

static struct clk_ops pll_hw_ops = {
	.recalc_rate = ka2000_pll_recalc_rate,
};
static const char *pll_parent_names[] = {"OSC"};
static const struct clk_init_data pll_hw_init_data = {
	.name = "PLL",
	.ops = &pll_hw_ops,
	.parent_names = pll_parent_names,
	.num_parents = ARRAY_SIZE(pll_parent_names),
	.flags = CLK_GET_RATE_NOCACHE,
};
static struct clk_hw pll_hw = { .init = &pll_hw_init_data, };

static const char *arm_parent_names[] = {"OSC", "PLL"};

static void register_clocktree(void)
{
	clocks[CLK_OSC] = ka2000_register_osc();
	clocks[CLK_PLL] = clk_register(NULL, &pll_hw);
	clocks[CLK_ARM] = clk_register_mux(NULL, "ARM", arm_parent_names, ARRAY_SIZE(arm_parent_names),
		CLK_GET_RATE_NOCACHE, CLK_SRC_CTL, ARM_SRC_SHIFT, ARM_SRC_WIDTH,
		0 /* flags */, &clk_lock);
	clocks[CLK_HCLK] = clk_register_divider(NULL, "HCLK", "ARM", CLK_GET_RATE_NOCACHE,
		CLK_SRC_CTL, HCLK_DIV_SHIFT, HCLK_DIV_WIDTH,
		0 /* flags */, &clk_lock);
}

void __init ka2000_early_init_clocks(void)
{
	printk(KERN_INFO "CLK_SRC_CTL=%08x\n", *CLK_SRC_CTL);
	printk(KERN_INFO "PLL_FREQ_SEL1=%08x\n", *PLL_FREQ_SEL1);
	printk(KERN_INFO "PLL_FREQ_SEL2=%08x\n", *PLL_FREQ_SEL2);
	printk(KERN_INFO "SYSTEM_CTL1=%08x\n", *SYSTEM_CTL1);

	*CLK_SRC_CTL &= 0xfff000ff;
	*CLK_SRC_CTL |= 0x00066600;
	printk(KERN_INFO "Switched SD/SDIO/SDSWITCH to OSC, CLK_SRC_CTL=%08x.\n", *CLK_SRC_CTL);
	printk(KERN_INFO "Switching HCLK to OSC rate (power-save).\n");
	/*
	 * Having HCLK at OSC rate makes switching CPU frequency easier since
	 * the timer uart and other HCLK-based rates can stay mostly independent
	 * of ARM core clock rate.
	 */
	switch_pll(2, 32, 8, 1, 1);
}

void __init ka2000_init_clocks(void)
{
	register_clocktree();
}

static struct cpufreq_frequency_table ka2000_freqs_table[] ={
	{ .frequency = KA2000_OSC_CLOCK * 8 / 1000, },
	{ .frequency = KA2000_OSC_CLOCK / 1000, },
	{ .frequency = CPUFREQ_TABLE_END, },
};

static int ka2000_verify_policy(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, ka2000_freqs_table);
}

static unsigned int ka2000_cpufreq_get(unsigned int cpu)
{
	u32 osc_freq_khz = ka2000_osc_clock / 1000;
	u32 pll_clk_khz = ka2000_get_pll_clk(ka2000_osc_clock) / 1000;
	if (*CLK_SRC_CTL & ARM_SRC_MASK)
		return pll_clk_khz;
	return osc_freq_khz;
}

static int ka2000_set_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	struct cpufreq_freqs freqs;
	unsigned long flags;
	int i;

	freqs.old = policy->cur;
	freqs.new = target_freq;

	for (i=0; ka2000_freqs_table[i].frequency != target_freq; i++) {
		if (ka2000_freqs_table[i].frequency == CPUFREQ_TABLE_END)
			return -EINVAL;
	}

	/*
	 * Tell everyone what we're about to do...
	 * you should add a notify client with any platform specific
	 * Vcc changing capability
	 */
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	local_irq_save(flags);

	if (target_freq == ka2000_osc_clock / 1000) {
		switch_pll(2, 32, 8, 0, 0);
	} else {
		switch_pll(2, 32, 8, 1, 0);
	}

	local_irq_restore(flags);

	/*
	 * Tell everyone what we've just done...
	 * you should add a notify client with any platform specific
	 * SDRAM refresh timer adjustments
	 */
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static int ka2000_cpufreq_init(struct cpufreq_policy *policy)
{
	/* set default policy and cpuinfo */
	policy->cpuinfo.transition_latency = 1000; /* FIXME: 1 ms, assumed */
	policy->cur = ka2000_cpufreq_get(0);   /* current freq */
	policy->min = ka2000_osc_clock / 1000;
	policy->max = policy->cur;

	cpufreq_frequency_table_cpuinfo(policy, ka2000_freqs_table);

	printk(KERN_INFO "KA2000 CPU frequency change support initialized (min=%d max=%d)\n",
		policy->min, policy->max);

	return 0;
}

static struct cpufreq_driver ka2000_cpufreq_driver = {
	.verify = ka2000_verify_policy,
	.target = ka2000_set_target,
	.init = ka2000_cpufreq_init,
	.get = ka2000_cpufreq_get,
	.name = "KA2000",
};

static int __init ka2000_cpufreq_register(void)
{
	return cpufreq_register_driver(&ka2000_cpufreq_driver);
}

static void __exit ka2000_cpufreq_unregister(void)
{
	cpufreq_unregister_driver(&ka2000_cpufreq_driver);
}

module_init(ka2000_cpufreq_register);
module_init(ka2000_cpufreq_unregister);
