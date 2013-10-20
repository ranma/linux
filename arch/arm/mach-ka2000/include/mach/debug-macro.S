#include <mach/hardware.h>
#include <mach/io.h>

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =KA_UART_BASE			@ physical base address
	ldr	\rv, =io_p2v(KA_UART_BASE)		@ virtual base address
	.endm

#define UART_SHIFT 2
#include <asm/hardware/debug-8250.S>
