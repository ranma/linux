#ifndef KA2000_SETUP_H
#define KA2000_SETUP_H

#include <linux/reboot.h>

struct ka2000_ssp_spi_info {
	struct spi_board_info *flash_device;
};

struct ka2000_clk_state {
	u32 ctl;
	u32 sys1;
};

extern void ka2000_map_io(void);
extern void ka2000_sys_init(void);
extern void ka2000_init_irq(void);
extern void ka2000_init_time(void);
extern void ka2000_init_early(void);
extern void ka2000_restart(enum reboot_mode mode, const char *cmd);

extern void ka2000_dt_init_early(void);
extern void ka2000_of_init(void);

extern void ka2000_early_init_clocks(void);
extern void ka2000_init_clocks(void);
extern void ka2000_pm_idle(void);

extern int timer_debug;

#endif
