#ifndef __ASM_KA2000_DEFINE_H
#define __ASM_KA2000_DEFINE_H


#define KA_REGIF_BASE            0xA0000000			//old:REGIF_BASE
#define KA_REGIF_SIZE            0x00100000			// rounded to 1MB page size.

#define SYSTEM_BASE              KA_REGIF_BASE
#define SYSTEM_CTRL_REG          SYSTEM_BASE
#define CLK_DIV_REG              SYSTEM_BASE + 0x04

#define SDRAM_BASE               0x00000000 + 0x00000000

#define KA_SCU_BASE              KA_REGIF_BASE                   /* System Control Unit */
#define KA_SSI_BASE              KA_REGIF_BASE + 0x1000
#define KA_PWM_BASE              KA_REGIF_BASE + 0x2000
#define KA_WDT_BASE              KA_REGIF_BASE + 0x3000
#define KA_UART_BASE             KA_REGIF_BASE + 0x4000
#define KA_GPIO0_BASE            KA_REGIF_BASE + 0x5000
#define KA_INTC_BASE             KA_REGIF_BASE + 0x6000
#define KA_DMA_BASE              KA_REGIF_BASE + 0x7000
#define KA_SDRAM_CTRL_BASE       KA_REGIF_BASE + 0x8000
#define KA_SDIO_BASE             KA_REGIF_BASE + 0x9000
#define KA_SD_SWITCH_BASE        KA_REGIF_BASE + 0xa000
#define KA_SD_CTRL_BASE          KA_REGIF_BASE + 0xb000
#define KA_GPIO1_BASE            KA_REGIF_BASE + 0xc000

#define KA2000_SSP_BASE_PHYS     KA_REGIF_BASE + 0x1000
#define KA2000_UART_BASE_PHYS    KA_REGIF_BASE + 0x4000
#define KA2000_SDIO_BASE_PHYS    KA_REGIF_BASE + 0x9000
#define KA2000_SD_BASE_PHYS      KA_REGIF_BASE + 0xb000

#ifndef __ASSEMBLY__
#define KA2000_ADDR(x)		((u32 __iomem __force *)io_p2v(x))
#define KA2000_ADDR64(x)	((u64 __iomem __force *)io_p2v(x))
#else
#define KA2000_ADDR(x)		(io_p2v(x))
#endif
#define KA2000_VA_TIMER		KA2000_ADDRESS(KA_PWM_BASE)

#define KA2000_UART0_BASE        KA_UART_BASE

/* scu */
#define SCU_CLK_SRC_CTL		 KA_SCU_BASE + 0x00
#define SCU_PLL_FREQ_SEL1	 KA_SCU_BASE + 0x04
#define SCU_PLL_FREQ_SEL2	 KA_SCU_BASE + 0x08
#define SCU_SYSTEM_CTL1		 KA_SCU_BASE + 0x0c
#define SCU_SYSTEM_CTL2		 KA_SCU_BASE + 0x10
#define SCU_SYSTEM_CTL3		 KA_SCU_BASE + 0x14

/* ssi */
#define SSI_BASE                 KA_SSI_BASE
#define SSI_PRE                  SSI_BASE + 0x00
#define SSI_CON                  SSI_BASE + 0x04
#define SSI_STA                  SSI_BASE + 0x08
#define SSI_TDAT                 SSI_BASE + 0x0c
#define SSI_RDAT                 SSI_BASE + 0x10

/* sdio */
#define SDIO_BASE                KA_SDIO_BASE
#define SDIO_CARD_BLOCK_SET_REG  SDIO_BASE  + 0x00
#define SDIO_CTRL_REG            SDIO_BASE  + 0x04
#define SDIO_CMD_ARGUMENT_REG    SDIO_BASE  + 0x08
#define SDIO_SPECIAL_COMMAND_REG SDIO_BASE  + 0x0c
#define SDIO_STATUS_REG          SDIO_BASE  + 0x10
#define SDIO_ERROR_ENABLE_REG    SDIO_BASE  + 0x14
#define SDIO_RESPONSE1_REG       SDIO_BASE  + 0x18
#define SDIO_RESPONSE2_REG       SDIO_BASE  + 0x1c
#define SDIO_RESPONSE3_REG       SDIO_BASE  + 0x20
#define SDIO_RESPONSE4_REG       SDIO_BASE  + 0x24
#define SDIO_BUF_TRAN_RESP_REG   SDIO_BASE  + 0x28
#define SDIO_BUF_TRAN_CTRL_REG   SDIO_BASE  + 0x2c
#define SDIO_DMA_SACH0_REG       SDIO_BASE  + 0x30
#define SDIO_DMA_TCCH0_REG       SDIO_BASE  + 0x34
#define SDIO_DMA_CTRCH0_REG      SDIO_BASE  + 0x38
#define SDIO_reserved            SDIO_BASE  + 0x3c
#define SDIO_DMA_DACH1_REG       SDIO_BASE  + 0x40
#define SDIO_DMA_TCCH1_REG       SDIO_BASE  + 0x44
#define SDIO_DMA_CTRCH1_REG      SDIO_BASE  + 0x48
#define SDIO_DMA_INTS_REG        SDIO_BASE  + 0x4c
#define SDIO_DMA_FIFO_STATUS_REG SDIO_BASE  + 0x50

/* pwm */
#define PWM_BASE                 KA_PWM_BASE
#define PWM_TCFG0                PWM_BASE + 0x00
#define PWM_TCFG1                PWM_BASE + 0x04
#define PWM_TCON0                PWM_BASE + 0x08
#define PWM_TCON1                PWM_BASE + 0x0c
#define PWM_TCNTB0               PWM_BASE + 0x10
#define PWM_TCMPB0               PWM_BASE + 0x14
#define PWM_TCNTO0               PWM_BASE + 0x18
#define PWM_TCNTB1               PWM_BASE + 0x40
#define PWM_TCNTO1               PWM_BASE + 0x44
#define PWM_TCNTB2               PWM_BASE + 0x48
#define PWM_TCNTO2               PWM_BASE + 0x4c

/* sd switch */
#define SDSW_BASE           	 KA_SD_SWITCH_BASE
#define SDSW_M1_CTRL0	 	 SDSW_BASE + 0x00
#define SDSW_M2_CTRL0	 	 SDSW_BASE + 0x04
#define SDSW_M1_STATUS	 	 SDSW_BASE + 0x08
#define SDSW_M2_STATUS		 SDSW_BASE + 0x0c
#define SDSW_READ_SWDAT	 	 SDSW_BASE + 0x18
#define SDSW_SW_CTRL0	 	 SDSW_BASE + 0x1c
#define SDSW_TEST_REG	 	 SDSW_BASE + 0x20
#define SDSW_M1_PREV_CMD_REG	 SDSW_BASE + 0x30
#define SDSW_M1_PREV_ARGU_REG	 SDSW_BASE + 0x34
#define SDSW_M1_PREV_RSP_REG0	 SDSW_BASE + 0x38
#define SDSW_M1_PREV_RSP_REG1	 SDSW_BASE + 0x3c
#define SDSW_M1_CURR_CMD_REG	 SDSW_BASE + 0x40
#define SDSW_M1_CURR_ARGU_REG	 SDSW_BASE + 0x44
#define SDSW_M1_CURR_RSP_REG0	 SDSW_BASE + 0x48
#define SDSW_M1_CURR_RSP_REG1	 SDSW_BASE + 0x4c
#define SDSW_M1_CURR_RSP_REG2	 SDSW_BASE + 0x50
#define SDSW_M1_CURR_RSP_REG3	 SDSW_BASE + 0x54
#define SDSW_M1_CURR_RSP_REG4	 SDSW_BASE + 0x58
#define SDSW_M1_CMD_FLG_REG0	 SDSW_BASE + 0x5c
#define SDSW_M1_CMD_CRC_FLG_REG0 SDSW_BASE + 0x60
#define SDSW_M1_CMD_FLG_REG1     SDSW_BASE + 0x64
#define SDSW_M1_CMD_CRC_FLG_REG1 SDSW_BASE + 0x68
#define SDSW_M1_CMD_FLG_REG2     SDSW_BASE + 0x6c
#define SDSW_M1_CMD_CRC_FLG_REG2 SDSW_BASE + 0x70
#define SDSW_M1_CMD_FLG_REG3     SDSW_BASE + 0x74
#define SDSW_M1_CMD_CRC_FLG_REG3 SDSW_BASE + 0x78
#define SDSW_BOMB_START_ADDR_REG SDSW_BASE + 0x80
#define SDSW_BOMB_END_ADDR_REG   SDSW_BASE + 0x84
#define SDSW_BOMB_Flag_ADDR_REG  SDSW_BASE + 0x88
#define SDSW_BOMB_START_ADDR_REG2 SDSW_BASE + 0x180
#define SDSW_BOMB_END_ADDR_REG2   SDSW_BASE + 0x184
#define SDSW_BOMB_Flag_ADDR_REG2  SDSW_BASE + 0x188
#define SDSW_M1_CID_REG0         SDSW_BASE + 0x8c
#define SDSW_M1_CID_REG1         SDSW_BASE + 0x90
#define SDSW_M1_CID_REG2         SDSW_BASE + 0x94
#define SDSW_M1_CID_REG3         SDSW_BASE + 0x98
#define SDSW_M1_RCA_REG          SDSW_BASE + 0x9c
#define SDSW_M1_DSR_REG          SDSW_BASE + 0xa0
#define SDSW_M1_CIC_REG          SDSW_BASE + 0xa4
#define SDSW_M1_CSD_REG0         SDSW_BASE + 0xa8
#define SDSW_M1_CSD_REG1         SDSW_BASE + 0xac
#define SDSW_M1_CSD_REG2         SDSW_BASE + 0xb0
#define SDSW_M1_CSD_REG3         SDSW_BASE + 0xb4
#define SDSW_M1_CSR_REG          SDSW_BASE + 0xb8
#define SDSW_M1_BLR_REG          SDSW_BASE + 0xbc
#define SDSW_M1_EWBS_REG         SDSW_BASE + 0xc0
#define SDSW_M1_EWBE_REG         SDSW_BASE + 0xc4
#define SDSW_M1_SBW_REG          SDSW_BASE + 0xc8
#define SDSW_M1_SWBEC_REG        SDSW_BASE + 0xcc
#define SDSW_M1_OCR_REG          SDSW_BASE + 0xd0
#define SDSW_M1_SCCD_REG         SDSW_BASE + 0xd4
#define SDSW_M1_SSR_REG0         SDSW_BASE + 0xd8
#define SDSW_M1_SSR_REG1         SDSW_BASE + 0xdc
#define SDSW_M1_SSR_REG2         SDSW_BASE + 0xe0
#define SDSW_M1_SSR_REG3         SDSW_BASE + 0xe4
#define SDSW_M1_SCR_REG0         SDSW_BASE + 0xe8
#define SDSW_M1_SCR_REG1         SDSW_BASE + 0xec
#define SDSW_M1_SNWB_REG         SDSW_BASE + 0xf0
#define SDSW_M1_BLOCK_LEN_REG    SDSW_BASE + 0xf4
#define SDSW_M1_WDATA_TOUT_REG   SDSW_BASE + 0xf8
#define SDSW_M1_RDATA_TOUT_REG   SDSW_BASE + 0xfc
#define SDSW_DIRECT_START_TRANS_REG     SDSW_BASE + 0x100
#define SDSW_DIRECT_CMD_INDEX_REG       SDSW_BASE + 0x104
#define SDSW_DIRECT_CMD_ARGU_REG        SDSW_BASE + 0x108
#define SDSW_DIRECT_CTRL_REG            SDSW_BASE + 0x10c
#define SDSW_DIRECT_BLOCK_LENGTH_REG    SDSW_BASE + 0x110
#define SDSW_DIRECT_WRITE_SW_CYCLE_REG  SDSW_BASE + 0x114
#define SDSW_M1_CMD_FLAG_INTEN_REG0     SDSW_BASE + 0x120
#define SDSW_M1_CMD_FLAG_INTEN_REG1     SDSW_BASE + 0x124
#define SDSW_M1_CMD_FLAG_INTEN_REG2     SDSW_BASE + 0x128
#define SDSW_M1_CMD_FLAG_INTEN_REG3     SDSW_BASE + 0x12c

/* wdt */
#define WDT_BASE                 KA_WDT_BASE
#define WDT_CON                  WDT_BASE + 0x00
#define WDT_DAT                  WDT_BASE + 0x04
#define WDT_CNT                  WDT_BASE + 0x08
#define WDT_CON_CONTER_EN        0x20

/* gpio0 */
#define GPIO0_BASE               KA_GPIO0_BASE		//old:GPIO_BASE
#define GPIO_OEN                 GPIO0_BASE + 0x00
#define GPIO_INPUT               GPIO0_BASE + 0x04
#define GPIO_OUTPUT              GPIO0_BASE + 0x08
#define GPIO_INT                 GPIO0_BASE + 0x0c
#define GPIO_INT_CLR0            GPIO0_BASE + 0x10
#define GPIO_INT_CLR1            GPIO0_BASE + 0x14
#define GPIO_INT_CLR2            GPIO0_BASE + 0x18
#define GPIO_INT_CLR3            GPIO0_BASE + 0x1c
#define GPIO_INT_CLR4            GPIO0_BASE + 0x20
#define GPIO_INT_CLR5            GPIO0_BASE + 0x24
#define GPIO_INT_CLR6            GPIO0_BASE + 0x28
#define GPIO_INT_CLR7            GPIO0_BASE + 0x2c

/* gpio1 */
#define GPIO1_BASE               KA_GPIO1_BASE
#define GPO_OEN                  GPIO1_BASE + 0x00
#define GPI_INPUT                GPIO1_BASE + 0x04
#define GPO_OUTPUT               GPIO1_BASE + 0x08
#define GPI_INT                  GPIO1_BASE + 0x0c
#define GPI_INT_CLR0             GPIO1_BASE + 0x10
#define GPI_INT_CLR1             GPIO1_BASE + 0x14
#define GPI_INT_CLR2             GPIO1_BASE + 0x18
#define GPI_INT_CLR3             GPIO1_BASE + 0x1c
#define GPI_INT_CLR4             GPIO1_BASE + 0x20
#define GPI_INT_CLR5             GPIO1_BASE + 0x24
#define GPI_INT_CLR6             GPIO1_BASE + 0x28
#define GPI_INT_CLR7             GPIO1_BASE + 0x2c

// sdram base
#define SDRAM_CTRL_BASE          KA_SDRAM_CTRL_BASE

/* uart */
#define KA_UART_RECV             KA_UART_BASE + 0x00
#define KA_UART_THR              KA_UART_BASE + 0x00
#define KA_UART_INTR             KA_UART_BASE + 0x04
#define KA_UART_FCR              KA_UART_BASE + 0x08
#define KA_UART_IIR              KA_UART_BASE + 0x08
#define KA_UART_LCR              KA_UART_BASE + 0x0c
#define KA_UART_MCR              KA_UART_BASE + 0x10
#define KA_UART_LINE             KA_UART_BASE + 0x14
#define KA_UART_MSR              KA_UART_BASE + 0x18
#define KA_UART_RFST             KA_UART_BASE + 0x1c

/*sd card */
#define SDR_BASE                  KA_SD_CTRL_BASE
#define SDR_Card_BLOCK_SET_REG    SDR_BASE + 0x00
#define SDR_CTRL_REG              SDR_BASE + 0x04
#define SDR_CMD_ARGUMENT_REG      SDR_BASE + 0x08
#define SDR_SPECIAL_CTRL_REG      SDR_BASE + 0x0C	//old:SDR_ADDRESS_REG
#define SDR_STATUS_REG            SDR_BASE + 0x10
#define SDR_Error_Enable_REG      SDR_BASE + 0x14
#define SDR_RESPONSE1_REG         SDR_BASE + 0x18
#define SDR_RESPONSE2_REG         SDR_BASE + 0x1C
#define SDR_RESPONSE3_REG         SDR_BASE + 0x20
#define SDR_RESPONSE4_REG         SDR_BASE + 0x24
#define SDR_DMA_TRAN_RESP_REG     SDR_BASE + 0x28
#define SDR_BUF_TRAN_CTRL_REG     SDR_BASE + 0x2C

#define SDR_DMA_SACH0_REG         SDR_BASE + 0x30
#define SDR_DMA_TCCH0_REG         SDR_BASE + 0x34
#define SDR_DMA_CTRCH0_REG        SDR_BASE + 0x38
#define SDR_DMA_DACH1_REG         SDR_BASE + 0x40
#define SDR_DMA_TCCH1_REG         SDR_BASE + 0x44
#define SDR_DMA_CTRCH1_REG        SDR_BASE + 0x48
#define SDR_DMA_INTS_REG          SDR_BASE + 0x4C
#define SDR_DMA_FIFO_STATUS_REG   SDR_BASE + 0x50

/* interrupt controller */
#define INTC_BASE                 KA_INTC_BASE
#define INTC_INTSRC1_ADDR         INTC_BASE + 0x00
#define INTC_INTSRC2_ADDR         INTC_BASE + 0x04
#define INTC_INTMOD1_ADDR         INTC_BASE + 0x08
#define INTC_INTMOD2_ADDR         INTC_BASE + 0x0c
#define INTC_INTMSK1_ADDR         INTC_BASE + 0x10
#define INTC_INTMSK2_ADDR         INTC_BASE + 0x14
#define INTC_INTPND1_ADDR         INTC_BASE + 0x18
#define INTC_INTPND2_ADDR         INTC_BASE + 0x1c
#define INTC_INTPRT1_ADDR         INTC_BASE + 0x20
#define INTC_INTPRT2_ADDR         INTC_BASE + 0x24
#define INTC_INTPRO1_ADDR         INTC_BASE + 0x28
#define INTC_INTPRO2_ADDR         INTC_BASE + 0x2c
#define INTC_INTCLR_ADDR          INTC_BASE + 0x30
#define INTC_INTOFS_ADDR          INTC_BASE + 0x34

#endif /* __ASM_KA2000_DEFINE_H */

#ifndef KA2000_H
#define KA2000_H

#define _KA2000_DEBUG_ 0
#define CONFIG_KA2000_CHIP_VERSION 0xD

#define OSC_MHZ   24
#if CONFIG_KA2000_CHIP_VERSION == 0xA
#define HCLK_RATE   4
#else
#define HCLK_RATE   2
#endif
#define KA2000_OSC_CLOCK 24000000
#define KA2000_UART_CLOCK KA2000_OSC_CLOCK
#define KA2000_TIMER_CLOCK KA2000_OSC_CLOCK
#define KA2000_PLL_CLOCK (KA2000_OSC_CLOCK * 8)
#define KA2000_MHZ (KA2000_PLL_CLOCK / 1000000)
#define UART_BAUD_RATE 38400


//#define KA2000_PLL_CLOCK 12000000

//------------------------------------------------------------------------------
#define ka2000_set_gpio(value); { *(u32 *)(0xf5005008) = value; }
#define KA_UARTC(ch);	*(volatile u32 __force *) (0x55000000 + 0xa0004000) = (ch);

/* 12M or 24M mode */
#define TIMER_PRESCALE_V (33)
#define TIMER_INT_COUNT (100)
#define TIMER_T1_MUX 0
#define TIMER_T2_MUX 0
#define TIMER_CLOCK_SOURCE_DIV (TIMER_PRESCALE_V)
#define TIMER_CLOCK_EVENT_DIV  (TIMER_PRESCALE_V*50)

#define TIMER_PERIOD   (0x1d4c)
#define TIMER_PERIOD_2 0xffff

#define ARCH_NR_IRQS      64
#define IRQ_KA2000_UART    1
#define IRQ_KA2000_TIMER1 22
#define IRQ_KA2000_TIMER2 23
#define IRQ_sd_buf_tran_finish_int       24
#define IRQ_sd_data_bound_int            25
#define IRQ_sd_tran_done_int             26
#define IRQ_sd_cmd_done_int              27
#define IRQ_sd_card_error_int            28
#define IRQ_sd_dma_int                   29
#define IRQ_sd_card_int                  30 // speculative
#define IRQ_sdio_buf_tran_finish_int       32
#define IRQ_sdio_data_bound_int            33
#define IRQ_sdio_tran_done_int             34
#define IRQ_sdio_cmd_done_int              35
#define IRQ_sdio_card_error_int            36
#define IRQ_sdio_dma_int                   37
#define IRQ_sdio_card_int                  38

#define KA2000_TIMER0_BASE 	(IO_PHYS + 0x2000)  //in pwm timer
#define NUM_TIMERS          	2
#define KA2000_WDOG_BASE   	(IO_PHYS + 0x3000)

#endif
