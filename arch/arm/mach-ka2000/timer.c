/*
 * Timer is similar to S3C2440, with slight differences:
 * Three timers, T0, T1, T2
 *   T0 for buzzer pwm, T1&T2 available.
 * Init T2:
 *   TCNTB2 = 0xa0002048 = 0xffff      // reload value
 *   TCON1 = 0xa000200c = 0x00000005  // T2 enable, with autoreload (interrupt 23)
 * Init T1:
 *   TCNTB1 = 0xa0002040 = 0xffff
 *   TCON0 = 0xa0002008 = 0x00500000  // T1 enable, with autoreload (interrupt 22)
 * Ack T2 interrupt:
 *   TCFG1 = 0xa0002004 = 0x02000000
 *   TCFG1 = 0xa0002004 = 0x00000000
 */

#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/sched_clock.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include "ka2000.h"

#define WDT_REG(x) KA2000_ADDR(WDT_BASE + x)
#define WDTCON		WDT_REG(0x0)
#define WDTDAT		WDT_REG(0x4)
#define WDTCNT		WDT_REG(0x8)

#define PWM_REG(x) KA2000_ADDR(PWM_BASE + x)
#define TCFG0                PWM_REG(0x00)
#define   T1T2_PRESC_MASK    0x00ff0000
#define   T1T2_PRESC_SHIFT   16
#define TCFG1                PWM_REG(0x04)
#define   T2_ACK             0x02000000
#define   T1_ACK             0x01000000
#define   T0_ACK             0x00100000
#define   T1_DIV2            0x00000000
#define   T1_DIV4            0x00000100
#define   T1_DIV8            0x00000200
#define   T1_DIV16           0x00000300
#define   T2_DIV2            0x00000000
#define   T2_DIV4            0x00000400
#define   T2_DIV8            0x00000800
#define   T2_DIV16           0x00000c00
#define TCON0                PWM_REG(0x08)
#define   T0_ENABLE          0x00000001
#define   T0_AUTORELOAD      0x00000008
#define   T1_ENABLE          0x00100000
#define   T1_LOAD            0x00200000
#define   T1_AUTORELOAD      0x00400000
#define TCON1                PWM_REG(0x0c)
#define   T2_ENABLE          0x00000001
#define   T2_LOAD            0x00000002
#define   T2_AUTORELOAD      0x00000004
#define TCNTB0               PWM_REG(0x10)
#define TCMPB0               PWM_REG(0x14)
#define TCNTO0               ((volatile u32*)PWM_REG(0x18))
#define TCNTB1               PWM_REG(0x40)
#define TCNTO1               ((volatile u32*)PWM_REG(0x44))
#define TCNTB2               PWM_REG(0x48)
#define TCNTO2               ((volatile u32*)PWM_REG(0x4c))

#define INTC_REG(x) KA2000_ADDR(INTC_BASE + x)
#define INTC_REG64(x) KA2000_ADDR64(INTC_BASE + x)
#define PEND	INTC_REG64(0x18)

static u32 notrace ka2000_read_sched_clock(void);

static enum clock_event_mode ka2000_timer1_mode = CLOCK_EVT_MODE_SHUTDOWN;

static void ka2000_timer_ack(u32 mask)
{
	u32 tcfg1;

	/* clear pending interrupt */
	tcfg1 = *TCFG1 & ~mask;
	*TCFG1 = tcfg1 | mask;
	barrier();
	*TCFG1 = tcfg1;
}

int timer_debug = 0;

static irqreturn_t ka2000_timer1_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	if (timer_debug) {
		printk(KERN_INFO "%s\n", __func__);
	}

	if (ka2000_timer1_mode == CLOCK_EVT_MODE_ONESHOT) {
		*TCON0 |= T1_LOAD;
		*TCFG1 |= T1_ACK;
		printk(KERN_INFO "timer1 irq handled\n");
	} else {
		ka2000_timer_ack(T1_ACK);
	}
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int ka2000_set_next_event_t1(unsigned long delta,
    struct clock_event_device *dev)
{
	u32 tcon0 = *TCON0;
	WARN_ON(delta == 0);

	printk(KERN_INFO "set_next_event(%ld)\n", delta);

	*TCNTB1 = delta;
	*TCON0 = tcon0 | T1_LOAD;
	barrier();
	*TCFG1 &= ~T1_ACK;  /* enable timer1 irq */
	barrier();
	*TCON0 = tcon0 | T1_ENABLE;
	return 0;
}

#define ka2000_prescaler 240
#define ka2000_clockevent_t1_div (KA2000_TIMER_CLOCK / ka2000_prescaler / 2 / HZ)
#define ka2000_clockevent_t1_freq (KA2000_TIMER_CLOCK / ka2000_prescaler / 2 / ka2000_clockevent_t1_div)
#define ka2000_clocksource_freq (KA2000_TIMER_CLOCK / ka2000_prescaler / 16)
#define ka2000_wdt_freq KA2000_TIMER_CLOCK


static void ka2000_set_mode_t1(enum clock_event_mode mode,
			    struct clock_event_device *evt)
{
	printk(KERN_INFO "set_mode_t1(%d)\n", mode);
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		*TCNTB1 = ka2000_clockevent_t1_div - 1;
		*TCON0 |= (T1_ENABLE | T1_AUTORELOAD);
		*TCFG1 &= ~T1_ACK;  /* enable timer1 irq */
		ka2000_timer1_mode = mode;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set by 'set next_event' */
		*TCON0 &= ~(T1_ENABLE | T1_AUTORELOAD);
		ka2000_timer1_mode = mode;
		break;
	case CLOCK_EVT_MODE_RESUME:
		*TCON0 |= T1_ENABLE;
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
	default:
		*TCON0 &= ~T1_ENABLE;
		break;
	}
}

static struct clock_event_device clockevent_ka2000_timer1 = {
	.name		= "ka2000 timer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.rating		= 200,
	.set_mode	= ka2000_set_mode_t1,
	.set_next_event	= ka2000_set_next_event_t1,
	.irq		= IRQ_KA2000_TIMER1,
};

static struct irqaction ka2000_timer1_irq = {
	.name		= "timer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= ka2000_timer1_interrupt,
	.dev_id		= &clockevent_ka2000_timer1,
};

static u32 notrace ka2000_read_sched_clock(void)
{
	return ~*WDTCNT;
}

static cycle_t ka2000_clocksource_tcnto2_read(struct clocksource *c)
{
	return ~*TCNTO2;
}

static cycle_t ka2000_clocksource_wdtcnt_read(struct clocksource *c)
{
	return ~*WDTCNT;
}

void __init ka2000_clocksource_init(void)
{
	*WDTDAT = 0xffffffff;

	clocksource_mmio_init(NULL, "TCNTO2", ka2000_clocksource_freq, 200, 16,
			ka2000_clocksource_tcnto2_read);
	clocksource_mmio_init(NULL, "WDTCNT", ka2000_wdt_freq, 300, 32,
			ka2000_clocksource_wdtcnt_read);
	setup_sched_clock(ka2000_read_sched_clock, 32, ka2000_wdt_freq);
}

void __init ka2000_clockevent_init(void)
{
	setup_irq(clockevent_ka2000_timer1.irq, &ka2000_timer1_irq);
	clockevents_config_and_register(&clockevent_ka2000_timer1,
					ka2000_clockevent_t1_freq, 0xf, 0xffff);
}

void __init ka2000_init_time(void)
{
	early_printk("ka2000_init_time HZ=%d div=%d freq=%d csfreq=%d\n", HZ,
		ka2000_clockevent_t1_div,
		ka2000_clockevent_t1_freq,
		ka2000_clocksource_freq);

	*TCFG0 = (ka2000_prescaler - 1) << T1T2_PRESC_SHIFT; /* T1/T2 prescaler */
	*TCFG1 = T1_DIV2 | T2_DIV16;
	*TCNTB1 = ka2000_clockevent_t1_div - 1;
	*TCNTB2 = 0xffff;  /* reload value, count from 0xffff down to 0x0000 */
	barrier();
	*TCON0 |= T1_LOAD;
	*TCON1 |= T2_LOAD;
	barrier();
	*TCON0 &= ~T1_LOAD;
	*TCON1 &= ~T2_LOAD;
	*TCON1 |= (T2_ENABLE | T2_AUTORELOAD); /* enable T2 */
	*TCFG1 |= T0_ACK|T1_ACK|T2_ACK;  /* disable timer irqs */

	ka2000_clockevent_init();
	ka2000_clocksource_init();
}

