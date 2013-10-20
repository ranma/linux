#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/major.h>

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>

#include <asm/types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/io.h>

#include "ka2000.h"

#define DRIVER_NAME "ka2000_mmc"

#define BUFFER_SIZE   16384
#define KA2000_NUM_IRQS 7
#define MAX_BLOCK_COUNT 1

#define DEBUG_ALL_REG_WRITES 0
#define DEBUG_READ_REG_WRITES 0
#define DEBUG_WRITE_REG_WRITES 0
#define DEBUG_ALL_STATE_TRANSITIONS 0
#define DEBUG_READ_STATE_TRANSITIONS 0
#define DEBUG_WRITE_STATE_TRANSITIONS 0

#define INTC_REG(x) KA2000_ADDR(INTC_BASE + x)
#define INTC_REG64(x) KA2000_ADDR64(INTC_BASE + x)
#define INTC_SRC	INTC_REG64(0x00)
#define INTC_MASK	INTC_REG64(0x10)
#define INTC_PEND	INTC_REG64(0x18)
#define INTC_CLEAR	INTC_REG(0x30)

#define SDSW_REG(x) KA2000_ADDR(SDSW_BASE + x)
#define M1_CTRL0	SDSW_REG(0x00)
#define M2_CTRL0	SDSW_REG(0x04)

#define CARD_BLOCK_SET_REG		0x00
#define   BLK_SIZE_SHIFT		16
#define   BLK_CNT_SHIFT			0
#define CTRL_REG			0x04
#define   CARD_INT_EN			(1 << 25)
#define   INT_EN			(1 << 24)
#define   OPCODE_SHIFT			16
#define   ENABLE_DATA			(1 << 14)
#define   CMD_RSP_CRC7			(1 << 13)
#define   CMD_RSP_OPCODE		(1 << 12)
#define   CMD_TYPE_RSPBUSY		(3 << 10)
#define   CMD_TYPE_R4R5R6		(2 << 10)
#define   CMD_TYPE_RSP136		(1 << 10)
#define   CMD_TYPE_NORESP		(0 << 10)
#define   XFER_MODE_MULTIPLE		(1 << 9) // speculative, ka2000-sdhc.ko sets this for cmd25
#define   XFER_MODE_WRITE		(0 << 8)
#define   XFER_MODE_READ		(1 << 8)
#define   HARD_RESET			(1 << 7)
#define   SOFT_RESET			(1 << 6)
#define   FORCE_CLOCK			(1 << 4)
#define   BUS_WIDTH_4			(1 << 0)
#define CMD_ARGUMENT_REG		0x08
#define SPECIAL_CTRL_REG		0x0C
#define   RESUME_RW			(1 << 1)
#define STATUS_REG			0x10
#define   STATUS_ERROR_MASK		0xffff0000
#define   STATUS_DATA_ENDBIT_ERR	(1 << 22)
#define   STATUS_DATA_CRC_FAIL		(1 << 21)
#define   STATUS_CMD_INDEX_ERR		(1 << 19)
#define   STATUS_CMD_ENDBIT_ERR		(1 << 18)
#define   STATUS_CMD_CRC_FAIL		(1 << 17)
#define   STATUS_CMD_TIMEOUT		(1 << 16)
#define ERROR_ENABLE_REG		0x14
#define   ENABLE_DATA_END_BIT_ERR	(1 << 6)
#define   ENABLE_DATA_CRC_ERR		(1 << 5)
#define   ENABLE_UNKOWN			(1 << 4)  // bit can be set
#define   ENABLE_CMD_INDEX_ERR		(1 << 3)
#define   ENABLE_CMD_END_BIT_ERR	(1 << 2)
#define   ENABLE_CMD_CRC_ERR		(1 << 1)
#define   ENABLE_CMD_TIMEOUT_ERR	(1 << 0)
#define   ENABLE_ALL_ERR		0x6f
#define RESPONSE1_REG			0x18
#define RESPONSE2_REG			0x1C
#define RESPONSE3_REG			0x20
#define RESPONSE4_REG			0x24
#define BUF_TRAN_RESP_REG		0x28
#define   FAKE_DMA_WRITE_STATUS		(1 << 16) // used internally
#define   FAKE_DMA_READ_STATUS		(1 << 15) // used internally
#define   CARD_INT_STATUS		(1 << 5)
#define   CARD_ERR_STATUS		(1 << 4)
#define   CARD_DATA_BOUND_STATUS	(1 << 3)
#define   CARD_CMD_DONE_STATUS		(1 << 2)
#define   XFER_DONE_STATUS		(1 << 1)
#define   DATA_OK_STATUS		(1 << 0)
#define BUF_TRAN_CTRL_REG		0x2C
#define   TRAN_READ			(0<<1)
#define   TRAN_WRITE			(1<<1)
#define   TRAN_START			(1<<2)
#define DMA_SACH0_REG			0x30
#define DMA_TCCH0_REG			0x34
#define DMA_CTRCH0_REG			0x38
#define   CTRCH_NOBURST			(0 << 4)
#define   CTRCH_BURST4			(1 << 4)
#define   CTRCH_BURST8			(2 << 4)
#define   CTRCH_BURST16			(3 << 4)
#define   CTRCH_ENABLE			(1 << 3)
#define   CTRCH_CNT_RELOAD		(1 << 2)
#define   CTRCH_ADDR_INC		(1 << 1)
#define   CTRCH_INT_EN			(1 << 0)
#define DMA_DACH1_REG			0x40
#define DMA_TCCH1_REG			0x44
#define DMA_CTRCH1_REG			0x48
#define DMA_INTS_REG			0x4C
#define   CH0_INT			(1 << 0)
#define   CH1_INT			(1 << 1)
#define DMA_FIFO_STATUS_REG		0x50
#define   DMA_FIFO_EMPTY		0x5

enum irq_idx {
	/* irqs in platform_data order */
	idx_buf_tran_finish_irq = 0,
	idx_data_bound_irq,
	idx_tran_done_irq,
	idx_cmd_done_irq,
	idx_card_error_irq,
	idx_dma_irq,
	idx_card_irq,
};

enum irq_source {
	source_none,
	source_buf_tran_finish_irq,
	source_data_bound_irq,
	source_tran_done_irq,
	source_cmd_done_irq,
	source_card_error_irq,
	source_dma_ch0_write_irq,
	source_dma_ch1_read_irq,
	source_card_irq,
};

enum cmd_state {
	state_idle,
	state_sent_cmd,
	state_wait_tran_done,
	state_wait_tran_finish,
	state_wait_data_ok,
	state_wait_read_dma_done,
	state_wait_write_dma_done,
	state_wait_err_irq,
	state_recv_data,
};

struct ka2000_mmc_host;

struct ka2000_mmc_host {
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_host		*mmc;
	struct device		*dev;
	unsigned char		id; /* KA2000 has 2 MMC blocks */
	void __iomem		*virt_base;
	unsigned int		phys_base;
	int			irqs[KA2000_NUM_IRQS];
	unsigned char		bus_width;
	unsigned int		bus_clock;

	unsigned int		sg_len;
	unsigned		dma_done:1;
	unsigned		dma_in_use:1;

	void __iomem		*reg_base;
	u32			reg_phys;

	void (*handle_irq)(struct ka2000_mmc_host *host, enum irq_source irq_src);
	enum cmd_state		cmd_state;
	int			last_mmc_flags;
	spinlock_t		request_lock;

	u32			mmc_irq_state;
};

struct ka2000_irq {
	const char *name;
	irqreturn_t (*handler)(int irq, void *dev_id);
	int is_masked;
};

/* State machine irq handlers. */
static void ka2000_handle_irq_idle(struct ka2000_mmc_host *host, enum irq_source irq_src);
static void ka2000_handle_irq_simplecmd(struct ka2000_mmc_host *host, enum irq_source irq_src);
static void ka2000_handle_irq_readcmd(struct ka2000_mmc_host *host, enum irq_source irq_src);
static void ka2000_handle_irq_writecmd(struct ka2000_mmc_host *host, enum irq_source irq_src);

static inline void ka2000_mmc_writel(const struct ka2000_mmc_host *host, u32 reg, u32 value)
{
	if (DEBUG_ALL_REG_WRITES ||
	    (DEBUG_READ_REG_WRITES && host->handle_irq == ka2000_handle_irq_readcmd) ||
	    (DEBUG_WRITE_REG_WRITES && host->handle_irq == ka2000_handle_irq_writecmd) ||
	     host->reg_phys == SDIO_BASE) {
		u32 addr = host->reg_phys + reg;
		printk(KERN_INFO "%s: mw %08x %08x\n", __func__, addr, value);
	}
	writel(value, host->reg_base + reg);
}

static inline u32 ka2000_mmc_readl(const struct ka2000_mmc_host *host, u32 reg)
{
	return readl(host->reg_base + reg);
}

static void ka2000_dump_regs(struct ka2000_mmc_host *host)
{
	int i;
	for (i=0; i<0x60; i+= 0x10) {
		printk(KERN_WARNING "mmc%d %02x: %08x %08x %08x %08x\n",
			host->id, i,
			ka2000_mmc_readl(host, i),
			ka2000_mmc_readl(host, i+4),
			ka2000_mmc_readl(host, i+8),
			ka2000_mmc_readl(host, i+12));
	}
}

static const char *ka2000_irq_src_str(enum irq_source irq_src)
{
	switch (irq_src) {
	case source_dma_ch0_write_irq: return "dma_ch0_write";
	case source_dma_ch1_read_irq: return "dma_ch1_read";
	case source_buf_tran_finish_irq: return "buf_tran_finish";
	case source_data_bound_irq: return "data_bound";
	case source_tran_done_irq: return "tran_done";
	case source_cmd_done_irq: return "cmd_done";
	case source_card_error_irq: return "card_error";
	case source_card_irq: return "card";
	default: break;
	}
	return "unknown";
}

static const char* ka2000_cmd_state_str(enum cmd_state cmd_state)
{
	switch (cmd_state) {
	case state_idle: return "idle";
	case state_sent_cmd: return "sent_cmd";
	case state_wait_tran_done: return "wait_tran_done";
	case state_wait_tran_finish: return "wait_tran_finish";
	case state_wait_data_ok: return "wait_data_ok";
	case state_wait_read_dma_done: return "wait_read_dma_done";
	case state_wait_write_dma_done: return "wait_write_dma_done";
	case state_wait_err_irq: return "wait_err_irq";
	case state_recv_data: return "recv_data";
	default: break;
	}
	return "unknown";
}

static void ka2000_print_last_mmc_flags(struct ka2000_mmc_host *host)
{
	int flags = host->last_mmc_flags;
	printk(KERN_INFO "mmc_flags=%08x\n", flags);
	if (flags & MMC_RSP_PRESENT)
		printk(KERN_INFO "  MMC_RSP_PRESENT\n");
	if (flags & MMC_RSP_136)
		printk(KERN_INFO "  MMC_RSP_136\n");
	if (flags & MMC_RSP_BUSY)
		printk(KERN_INFO "  MMC_RSP_BUSY\n");
	if (flags & MMC_RSP_CRC)
		printk(KERN_INFO "  MMC_RSP_CRC\n");
	if (flags & MMC_RSP_OPCODE)
		printk(KERN_INFO "  MMC_RSP_OPCODE\n");
}

static void ka2000_debug_state(struct ka2000_mmc_host *host, const char *prefix, enum irq_source irq_src)
{
	u32 tran_resp = ka2000_mmc_readl(host, BUF_TRAN_RESP_REG);
	u32 tran_ctrl = ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG);
	u32 fifo_status = ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG);
	u32 block_set = ka2000_mmc_readl(host, CARD_BLOCK_SET_REG);
	u32 status = ka2000_mmc_readl(host, STATUS_REG);
	printk(KERN_INFO "%s: irq_source=%s irq_state=%08x cmd_state=%s\ntran_resp=%08x tran_ctrl=%08x fifo=%08x blocks=%08x status=%08x\n",
		prefix, ka2000_irq_src_str(irq_src), host->mmc_irq_state,
		ka2000_cmd_state_str(host->cmd_state),
		tran_resp, tran_ctrl, fifo_status, block_set, status);
	ka2000_print_last_mmc_flags(host);
	ka2000_dump_regs(host);
}

static void ka2000_update_state(struct ka2000_mmc_host *host, enum cmd_state new_state)
{
	if (DEBUG_ALL_STATE_TRANSITIONS ||
	    (DEBUG_READ_STATE_TRANSITIONS && host->handle_irq == ka2000_handle_irq_readcmd) ||
	    (DEBUG_WRITE_STATE_TRANSITIONS && host->handle_irq == ka2000_handle_irq_writecmd) ||
	     host->reg_phys == SDIO_BASE) {
		printk(KERN_INFO "transition %s->%s\n",
			ka2000_cmd_state_str(host->cmd_state),
			ka2000_cmd_state_str(new_state));
	}
	host->cmd_state = new_state;
}

static void ka2000_mmc_start_command(struct ka2000_mmc_host *host, struct mmc_command *cmd)
{
	u32 ctl = CARD_INT_EN | INT_EN;

	/*
	printk(KERN_INFO "%s %d opcode=%04x arg=%08x flags=%08x\n", __func__, host->id,
		cmd->opcode, cmd->arg, cmd->flags);
	*/
	if (host->bus_width == MMC_BUS_WIDTH_4)
		ctl |= BUS_WIDTH_4;

	if (host->bus_width == MMC_BUS_WIDTH_1 &&
	    cmd->data && (cmd->data->flags & MMC_DATA_WRITE)) {
		printk(KERN_WARNING "%s: bus width 1 not supported for writes (broken crc generator)\n",
		       __func__);
	}

	ctl |= (cmd->opcode << OPCODE_SHIFT);

	if (!(cmd->flags & MMC_RSP_PRESENT))
		ctl |= CMD_TYPE_NORESP;
	else if (cmd->flags & MMC_RSP_136)
		ctl |= CMD_TYPE_RSP136;  /* R2 */
	else if (cmd->flags & MMC_RSP_BUSY)
		ctl |= CMD_TYPE_RSPBUSY;
	else
		ctl |= CMD_TYPE_R4R5R6;

	if (cmd->flags & MMC_RSP_CRC)
		ctl |= CMD_RSP_CRC7;
	if (cmd->flags & MMC_RSP_OPCODE)
		ctl |= CMD_RSP_OPCODE;

	if (cmd->data != NULL) {
		int is_read = cmd->data->flags & MMC_DATA_READ;
		if (is_read) {
			ctl |= XFER_MODE_READ;
		}

		ctl |= ENABLE_DATA;
	}

	ka2000_mmc_writel(host, CMD_ARGUMENT_REG, cmd->arg);
	barrier();
	ka2000_mmc_writel(host, CTRL_REG, ctl);
	barrier();
}

static void ka2000_mmc_prepare_data(struct ka2000_mmc_host *host, struct mmc_request *req)
{
	struct mmc_data *data;
	int is_read;
	u32 block_reg;

	if (!req || !req->cmd || !req->cmd->data)
		return;

	data = req->cmd->data;

	BUG_ON(data->sg_len > 1);

	is_read = (data->flags & MMC_DATA_READ) != 0;
	if (is_read)
		host->handle_irq = ka2000_handle_irq_readcmd;
	else
		host->handle_irq = ka2000_handle_irq_writecmd;

	/* FIXME: Something is wrong with the reg description:
	 * Using it as advertised in the sources results in the
	 * data transfer being off by a factor of 16.
	 * Testing writeable registery bits in u-boot:
	 * KA2000#mw a000b000 ffffffff
	 * KA2000#md a000b000
	 * a000b000: 0fff03ff
	 *
	 * BLK_SIZE_SHIFT adjusted to 16 instead of 20.
	 * '1' doesn't seem to mean 512 either.
	 */
	if (data->blksz == 512 ) {
		block_reg = 0x100;
	} else {
		block_reg = data->blksz << BLK_SIZE_SHIFT;
	}
	if (host->reg_phys == SDIO_BASE) {
		if (data->blksz == 512) {
			block_reg = (1 << 16);
		} else {
			block_reg = data->blksz << 20;
		}
	}
	block_reg |= data->blocks;
	ka2000_mmc_writel(host, CARD_BLOCK_SET_REG, block_reg);

	dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
	           is_read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	if (is_read) {
		if (ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG) != DMA_FIFO_EMPTY) {
			printk(KERN_WARNING "dma fifo not empty: fifo=%08x tran=%08x blockset=%08x\n",
				ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG),
				ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG),
				ka2000_mmc_readl(host, CARD_BLOCK_SET_REG));
		}
		if (ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG) & 0xffff0000) {
			printk(KERN_WARNING "tran ctrl blocks not 0: fifo=%08x tran=%08x blockset=%08x\n",
				ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG),
				ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG),
				ka2000_mmc_readl(host, CARD_BLOCK_SET_REG));
		}
		ka2000_mmc_writel(host, DMA_DACH1_REG, sg_dma_address(data->sg)); /* Start Addr, DMA_WADDR */
		ka2000_mmc_writel(host, DMA_TCCH1_REG, data->blocks * data->blksz); /* Transfer Count */
	} else {
		ka2000_mmc_writel(host, DMA_SACH0_REG, sg_dma_address(data->sg)); /* Start Addr, DMA_WADDR */
		ka2000_mmc_writel(host, DMA_TCCH0_REG, data->blocks * data->blksz); /* Transfer Count */
		barrier();

		/* Don't need dma interrupt as cmd finishes after dma does. */
		ka2000_mmc_writel(host, DMA_CTRCH0_REG, CTRCH_BURST16|/*CTRCH_INT_EN|*/CTRCH_ADDR_INC|CTRCH_CNT_RELOAD|CTRCH_ENABLE);
		barrier();
		ka2000_mmc_writel(host, DMA_CTRCH0_REG, CTRCH_BURST16|/*CTRCH_INT_EN|*/CTRCH_ADDR_INC);
		barrier();
		ka2000_mmc_writel(host, BUF_TRAN_CTRL_REG, ((data->blocks << 16) | TRAN_WRITE | TRAN_START));
		barrier();
	}
}

static void ka2000_mmc_unprepare_data(struct ka2000_mmc_host *host, struct mmc_request *req)
{
	struct mmc_data *data;
	int is_read;

	if (!req || !req->cmd || !req->cmd->data)
		return;

	data = req->cmd->data;
	is_read = (data->flags & MMC_DATA_READ) != 0;
	if (data->sg) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     is_read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	}
	if (is_read) {
		ka2000_mmc_writel(host, DMA_CTRCH1_REG, 0);
	} else {
		ka2000_mmc_writel(host, DMA_CTRCH0_REG, 0);
	}
}

static void ka2000_sdsw_start_request_hook(struct ka2000_mmc_host *host)
{
	/* Switch to master 2 (SoC) */
	*M1_CTRL0 = 0;
	barrier();
	*M2_CTRL0 = 1;
	barrier();
}

static void ka2000_sdsw_end_request_hook(struct ka2000_mmc_host *host)
{
	/* Switch back to master 1 (cardreader/host) */
	*M1_CTRL0 = 1;
	barrier();
	*M2_CTRL0 = 0;
	barrier();
}

static void ka2000_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct ka2000_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;

	BUG_ON(host->mrq != NULL);
	BUG_ON(!req);

	if (host->reg_phys == SDR_BASE) {
		ka2000_sdsw_start_request_hook(host);
	}

	host->mrq = req;
	host->cmd = req->cmd;
	host->data = req->data;
	host->last_mmc_flags = req->cmd->flags;

	host->handle_irq = ka2000_handle_irq_simplecmd;

	spin_lock_irqsave(&host->request_lock, flags);
	host->mmc_irq_state = 0;

	ka2000_mmc_prepare_data(host, req);
	ka2000_update_state(host, state_sent_cmd);
	ka2000_mmc_start_command(host, req->cmd);

	spin_unlock_irqrestore(&host->request_lock, flags);
}

static void ka2000_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ka2000_mmc_host *host = mmc_priv(mmc);

	host->bus_width = ios->bus_width;
	host->bus_clock = ios->clock;
}

static int ka2000_mmc_get_ro(struct mmc_host *mmc)
{
	return 0;  /* No read-only switch available */
}

static const struct mmc_host_ops ka2000_mmc_ops = {
	.request	= ka2000_mmc_request,
	.set_ios	= ka2000_mmc_set_ios,
	.get_ro		= ka2000_mmc_get_ro,
};

static void drain_read_dma(struct ka2000_mmc_host *host)
{
	char *test = host->virt_base;
	int i, n = 0;
	printk(KERN_INFO "%s: Trying to drain read dma channel\n", __func__);
	ka2000_dump_regs(host);
	while (ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG) != DMA_FIFO_EMPTY) {
		printk(KERN_WARNING "dma fifo not empty: fifo=%08x tran=%08x blockset=%08x\n",
			ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG),
			ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG),
			ka2000_mmc_readl(host, CARD_BLOCK_SET_REG));
		if (ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG) == DMA_FIFO_EMPTY) {
			printk(KERN_WARNING "now it's empty after all, delay was needed\n");
			break;
		}

		memset(host->virt_base, 0x5a, 16);
		dma_sync_single_for_device(host->dev, host->phys_base, 16, DMA_TO_DEVICE);
		dma_sync_single_for_device(host->dev, host->phys_base, 16, DMA_FROM_DEVICE);

		printk(KERN_INFO "Triggering transfer\n");
		ka2000_mmc_writel(host, DMA_DACH1_REG, host->phys_base);
		ka2000_mmc_writel(host, DMA_TCCH1_REG, 16); /* Transfer Count */
		barrier();
		ka2000_mmc_writel(host, DMA_CTRCH1_REG, CTRCH_BURST16|CTRCH_INT_EN|CTRCH_ADDR_INC|CTRCH_CNT_RELOAD|CTRCH_ENABLE);
		barrier();
		ka2000_mmc_writel(host, DMA_CTRCH1_REG, CTRCH_BURST16|CTRCH_INT_EN|CTRCH_ADDR_INC);
		barrier();

		n++;

		printk(KERN_INFO "Waiting...\n");
		while (ka2000_mmc_readl(host, DMA_INTS_REG) == 0)
			barrier();

		dma_sync_single_for_cpu(host->dev, host->phys_base, 16, DMA_TO_DEVICE);
		dma_sync_single_for_cpu(host->dev, host->phys_base, 16, DMA_FROM_DEVICE);
		for (i=0; i<16; i+=4) {
			printk(KERN_INFO "%02x: %02x %02x %02x %02x\n", i,
				 test[i], test[i+1], test[i+2], test[i+3]);
		}

		if (test[0] == 0x5a && test[1] == 0x5a && test[2] == 0x5a && test[3] == 0x5a)
			break;
	}
	ka2000_dump_regs(host);

	printk(KERN_INFO "Drained approx %d bytes!\n", 16 * n);
}

static void ka2000_finish_cmd(struct ka2000_mmc_host *host)
{
	u32 int_status = ka2000_mmc_readl(host, BUF_TRAN_RESP_REG);
	u32 status = ka2000_mmc_readl(host, STATUS_REG);
	struct mmc_command *cmd;

	BUG_ON(!host->cmd);
	cmd = host->cmd;

	if (status & (STATUS_CMD_CRC_FAIL | STATUS_CMD_TIMEOUT |
	              STATUS_CMD_INDEX_ERR | STATUS_CMD_ENDBIT_ERR)) {
		printk(KERN_INFO "%s: card_err_status %08x %08x\n", __func__,
			int_status, status);
		if (status & (STATUS_CMD_CRC_FAIL |
		              STATUS_CMD_INDEX_ERR |
		              STATUS_CMD_ENDBIT_ERR))
			cmd->error = -EILSEQ;
		if (status & STATUS_CMD_TIMEOUT)
			cmd->error = -ETIMEDOUT;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[0] = ka2000_mmc_readl(host, RESPONSE4_REG);
			cmd->resp[1] = ka2000_mmc_readl(host, RESPONSE3_REG);
			cmd->resp[2] = ka2000_mmc_readl(host, RESPONSE2_REG);
			cmd->resp[3] = ka2000_mmc_readl(host, RESPONSE1_REG);
			/*
			printk(KERN_INFO "resp: %08x %08x %08x %08x\n",
				cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);
			*/
		} else {
			cmd->resp[0] = ka2000_mmc_readl(host, RESPONSE1_REG);
			// printk(KERN_INFO "resp: %08x\n", cmd->resp[0]);
		}
	}
}

static void ka2000_finish_request(struct ka2000_mmc_host *host)
{
	BUG_ON(!host->mmc);
	BUG_ON(!host->mrq);
	mmc_request_done(host->mmc, host->mrq);
	host->mrq = NULL;
	host->data = NULL;
	host->cmd = NULL;
	host->mmc_irq_state = 0;
	host->handle_irq = ka2000_handle_irq_idle;
	ka2000_update_state(host, state_idle);

	if (host->reg_phys == SDR_BASE) {
		ka2000_sdsw_end_request_hook(host);
	}
}

static void ka2000_handle_irq_idle(struct ka2000_mmc_host *host, enum irq_source irq_src)
{
	u32 tran_resp = ka2000_mmc_readl(host, BUF_TRAN_RESP_REG);
	printk(KERN_WARNING "state=idle, but got interrupt? tran_resp=%08x\n", tran_resp);
	ka2000_debug_state(host, "idle", irq_src);
}

static void ka2000_handle_irq_simplecmd(struct ka2000_mmc_host *host, enum irq_source irq_src)
{
	switch (host->mmc_irq_state) {
	case (CARD_CMD_DONE_STATUS):
		ka2000_finish_cmd(host);
		ka2000_finish_request(host);
		break;
	default:
		ka2000_debug_state(host, "simplecmd unexpected irq state", irq_src);
		break;
	}
}

static void ka2000_handle_irq_readcmd(struct ka2000_mmc_host *host, enum irq_source irq_src)
{
	u32 status = ka2000_mmc_readl(host, STATUS_REG);
	struct mmc_data *data = host->data;
	BUG_ON(!data);

	/* Implements state machine for mmc/sd commands with read data phase. */
	switch (host->mmc_irq_state) {
	case CARD_CMD_DONE_STATUS:
		/* need XFER_DONE_STATUS as well */
		break;
	case (CARD_CMD_DONE_STATUS | XFER_DONE_STATUS):
		ka2000_mmc_writel(host, BUF_TRAN_CTRL_REG, ((data->blocks << 16) | TRAN_READ | TRAN_START));
		barrier();
		ka2000_mmc_writel(host, DMA_CTRCH1_REG, CTRCH_BURST16|CTRCH_INT_EN|CTRCH_ADDR_INC|CTRCH_CNT_RELOAD|CTRCH_ENABLE);
		barrier();
		ka2000_mmc_writel(host, DMA_CTRCH1_REG, CTRCH_BURST16|CTRCH_INT_EN|CTRCH_ADDR_INC);
		barrier();
		ka2000_update_state(host, state_wait_read_dma_done);
		break;
	case (CARD_CMD_DONE_STATUS | XFER_DONE_STATUS |
	      DATA_OK_STATUS):
		/* still need dma irq */
		break;
	case (CARD_CMD_DONE_STATUS | XFER_DONE_STATUS |
	      FAKE_DMA_READ_STATUS):
		/* still need data_ok irq */
		break;
	case (CARD_CMD_DONE_STATUS | XFER_DONE_STATUS |
	      DATA_OK_STATUS | FAKE_DMA_READ_STATUS):
		if (status & (STATUS_DATA_CRC_FAIL | STATUS_DATA_ENDBIT_ERR)) {
			printk(KERN_INFO "%s: error status %08x\n", __func__, status);
			data->error = -EILSEQ;
		} else {
			data->bytes_xfered += data->sg->length;
		}

		ka2000_mmc_unprepare_data(host, host->mrq);
		barrier();

		if (ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG) != DMA_FIFO_EMPTY) {
			printk(KERN_WARNING "dma fifo not empty: fifo=%08x tran=%08x blockset=%08x\n",
				ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG),
				ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG),
				ka2000_mmc_readl(host, CARD_BLOCK_SET_REG));
			drain_read_dma(host);
		}
		if (ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG) & 0xffff0000) {
			printk(KERN_WARNING "tran ctrl blocks not 0: fifo=%08x tran=%08x blockset=%08x\n",
				ka2000_mmc_readl(host, DMA_FIFO_STATUS_REG),
				ka2000_mmc_readl(host, BUF_TRAN_CTRL_REG),
				ka2000_mmc_readl(host, CARD_BLOCK_SET_REG));
		}

		ka2000_mmc_writel(host, DMA_CTRCH1_REG, 0);
		ka2000_finish_cmd(host);
		ka2000_finish_request(host);
		break;
	default:
		ka2000_debug_state(host, "readcmd unknown cmd_state", irq_src);
		break;
	}
}

static void ka2000_handle_irq_writecmd(struct ka2000_mmc_host *host, enum irq_source irq_src)
{
	u32 status = ka2000_mmc_readl(host, STATUS_REG);
	struct mmc_data *data = host->data;
	BUG_ON(!data);

	/* Implements state machine for mmc/sd commands with write data phase. */
	switch (host->mmc_irq_state) {
	case (DATA_OK_STATUS | CARD_CMD_DONE_STATUS):
		/* still need xfer_done */
		break;
	case (CARD_CMD_DONE_STATUS | DATA_OK_STATUS | XFER_DONE_STATUS):
		ka2000_finish_cmd(host);
		if (status & (STATUS_DATA_CRC_FAIL | STATUS_DATA_ENDBIT_ERR)) {
			printk(KERN_INFO "%s: error status %08x\n", __func__, status);
			//ka2000_print_last_mmc_flags(host);
			data->error = -EILSEQ;
		} else {
			data->bytes_xfered += data->sg->length;
		}
		ka2000_mmc_writel(host, DMA_CTRCH0_REG, 0);
		ka2000_mmc_unprepare_data(host, host->mrq);
		ka2000_finish_request(host);

		ka2000_mmc_writel(host, BUF_TRAN_RESP_REG, CARD_CMD_DONE_STATUS);
		break;
	default:
		ka2000_debug_state(host, "writecmd unknown cmd_state", irq_src);
		break;
	}
}

static irqreturn_t ka2000_mmc_dma_irq(int irq, void *dev_id)
{
	struct ka2000_mmc_host *host = dev_id;
	u32 dma_ints;
	while ((dma_ints = ka2000_mmc_readl(host, DMA_INTS_REG)) == 0) {
		barrier();
		printk(KERN_WARNING "%s: dma irq source is 0?\n", __func__);
	}
	if (host->reg_phys == SDIO_BASE) {
		printk(KERN_INFO "%s: %08x\n", __func__, dma_ints);
	}
	if (dma_ints & CH0_INT) {
		host->mmc_irq_state |= FAKE_DMA_WRITE_STATUS;
		host->handle_irq(host, source_dma_ch0_write_irq);
	} else {
		host->mmc_irq_state |= FAKE_DMA_READ_STATUS;
		host->handle_irq(host, source_dma_ch1_read_irq);
	}
	ka2000_mmc_writel(host, DMA_INTS_REG, dma_ints);
	return IRQ_HANDLED;
}

static void ka2000_ack_pending(struct ka2000_mmc_host *host, u32 mask)
{
	u64 ack_mask = 0;

	if (mask & DATA_OK_STATUS)
		ack_mask |= host->irqs[idx_buf_tran_finish_irq];
	if (mask & CARD_DATA_BOUND_STATUS)
		ack_mask |= host->irqs[idx_data_bound_irq];
	if (mask & XFER_DONE_STATUS)
		ack_mask |= host->irqs[idx_tran_done_irq];
	if (mask & CARD_CMD_DONE_STATUS)
		ack_mask |= host->irqs[idx_cmd_done_irq];
	if (mask & CARD_ERR_STATUS)
		ack_mask |= host->irqs[idx_card_error_irq];
	if (mask & CARD_INT_STATUS)
		ack_mask |= host->irqs[idx_card_irq];

	*INTC_PEND = ack_mask;
}

static irqreturn_t ka2000_mmc_generic_irq(int irq, void *dev_id, u32 mask, enum irq_source irq_src)
{
	struct ka2000_mmc_host *host = dev_id;
	u32 need_ack = ka2000_mmc_readl(host, BUF_TRAN_RESP_REG) & ~mask;
	host->mmc_irq_state |= (mask | need_ack) & ~CARD_ERR_STATUS;
	if (host->reg_phys == SDIO_BASE) {
		printk(KERN_INFO "%s: %08x %08x\n", __func__, mask, need_ack);
	}
	host->handle_irq(host, irq_src);
	/* Note: after handle_irq, mmc_irq_state can be 0 (from finish_request) */
	/* ack on the mmc controller */
	ka2000_mmc_writel(host, BUF_TRAN_RESP_REG, mask | need_ack);
	/* ack on the interrupt controller */
	if (need_ack)
		ka2000_ack_pending(host, need_ack);
	return IRQ_HANDLED;
}

static irqreturn_t ka2000_mmc_buf_tran_finish_irq(int irq, void *dev_id)
{
	return ka2000_mmc_generic_irq(irq, dev_id, DATA_OK_STATUS, source_buf_tran_finish_irq);
}

static irqreturn_t ka2000_mmc_data_bound_irq(int irq, void *dev_id)
{
	return ka2000_mmc_generic_irq(irq, dev_id, CARD_DATA_BOUND_STATUS, source_data_bound_irq);
}

static irqreturn_t ka2000_mmc_tran_done_irq(int irq, void *dev_id)
{
	return ka2000_mmc_generic_irq(irq, dev_id, XFER_DONE_STATUS, source_tran_done_irq);
}

static irqreturn_t ka2000_mmc_cmd_done_irq(int irq, void *dev_id)
{
	return ka2000_mmc_generic_irq(irq, dev_id, CARD_CMD_DONE_STATUS, source_cmd_done_irq);
}

static irqreturn_t ka2000_mmc_card_error_irq(int irq, void *dev_id)
{
	/* Note: Is masked since we always look at status bits anyway, */
	BUG_ON(true);
	return ka2000_mmc_generic_irq(irq, dev_id, CARD_ERR_STATUS, source_card_error_irq);
}

static irqreturn_t ka2000_mmc_card_irq(int irq, void *dev_id)
{
	return ka2000_mmc_generic_irq(irq, dev_id, CARD_INT_STATUS, source_card_irq);
}

static const struct ka2000_irq irq_handlers[] = {
	{ .name = "buf_tran_finish", .handler = ka2000_mmc_buf_tran_finish_irq, .is_masked = 0 },
	{ .name = "data_bound", .handler = ka2000_mmc_data_bound_irq, .is_masked = 0 },
	{ .name = "tran_done", .handler = ka2000_mmc_tran_done_irq, .is_masked = 0 },
	{ .name = "cmd_done", .handler = ka2000_mmc_cmd_done_irq, .is_masked = 0 },
	{ .name = "card_error", .handler = ka2000_mmc_card_error_irq, .is_masked = 1 },
	{ .name = "dma", .handler = ka2000_mmc_dma_irq, .is_masked = 0 },
	{ .name = "card", .handler = ka2000_mmc_card_irq, .is_masked = 0 },
};

extern void sdio_enable_gpio0_power_on_sdio_wifi(void);

static int ka2000_mmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct ka2000_mmc_host *host = NULL;
	struct resource *res;
	int ret = 0;
	int irqs[KA2000_NUM_IRQS];
	int i;
	dma_addr_t buf_addr;

	sdio_enable_gpio0_power_on_sdio_wifi();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	for (i=0; i<KA2000_NUM_IRQS; i++) {
		irqs[i] = platform_get_irq(pdev, i);
		if (res == NULL || irqs[i] < 0)
			return -ENXIO;
	}

	mmc = mmc_alloc_host(sizeof(struct ka2000_mmc_host), &pdev->dev);
	if (mmc == NULL) {
		ret = -ENOMEM;
		goto err_alloc_host_failed;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->handle_irq = ka2000_handle_irq_idle;

	pr_info("mmc: Mapping %lX to %lX\n", (long)res->start, (long)res->end);
	host->reg_phys = res->start;
	host->reg_base = ioremap(res->start, resource_size(res));
	if (host->reg_base == NULL) {
		ret = -ENOMEM;
		goto ioremap_failed;
	}
	host->virt_base = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE,
					     &buf_addr, GFP_KERNEL);

	if (host->virt_base == 0) {
		ret = -ENOMEM;
		goto dma_alloc_failed;
	}
	host->phys_base = buf_addr;

	host->id = pdev->id;
	memcpy(host->irqs, irqs, sizeof(host->irqs));
	pr_info("mmc: pdev id %d (%s), base irq %d, index %d\n",
		host->id, host->id ? "sdio" : "sd", host->irqs[0], mmc->index);

	mmc->ops = &ka2000_mmc_ops;
	mmc->f_min = 400000;
	mmc->f_max = 24000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA;

	mmc->max_segs = 1;
	mmc->max_blk_size = 2048;
	mmc->max_blk_count = MAX_BLOCK_COUNT;
	mmc->max_req_size = BUFFER_SIZE;
	mmc->max_seg_size = mmc->max_req_size;

	for (i=0; i<KA2000_NUM_IRQS; i++) {
		int name_size = strlen(irq_handlers[i].name) + sizeof(DRIVER_NAME) + 3;
		char *name = kzalloc(name_size, GFP_KERNEL);
		if (!name) {
			dev_err(&pdev->dev, "Failed kzalloc\n");
			goto err_request_irq_failed;
		}
		snprintf(name, name_size, "%s%d_%s", DRIVER_NAME, host->id, irq_handlers[i].name);
		/* FIXME: Leaking string memory */
		ret = request_irq(host->irqs[i], irq_handlers[i].handler, 0, name, host);
		if (ret) {
			dev_err(&pdev->dev, "Failed IRQ Adding ka2000 MMC\n");
			goto err_request_irq_failed;
		}
		if (irq_handlers[i].is_masked) {
			printk(KERN_INFO "Masking %s irq\n", irq_handlers[i].name);
			disable_irq(host->irqs[i]);
		}
	}

	host->dev = &pdev->dev;
	platform_set_drvdata(pdev, host);

	ka2000_mmc_writel(host, CTRL_REG, HARD_RESET);
	udelay(200);
	ka2000_mmc_writel(host, ERROR_ENABLE_REG, ENABLE_ALL_ERR);
	ka2000_update_state(host, state_idle);
	spin_lock_init(&host->request_lock);

	mmc_add_host(mmc);
	return 0;

err_request_irq_failed:
	dma_free_coherent(&pdev->dev, BUFFER_SIZE, host->virt_base,
			  host->phys_base);
dma_alloc_failed:
	iounmap(host->reg_base);
ioremap_failed:
	mmc_free_host(host->mmc);
err_alloc_host_failed:
	return ret;
}

static int ka2000_mmc_remove(struct platform_device *pdev)
{
	struct ka2000_mmc_host *host = platform_get_drvdata(pdev);
	int i;

	BUG_ON(host == NULL);

	mmc_remove_host(host->mmc);
	for (i=0; i<KA2000_NUM_IRQS; i++) {
		free_irq(host->irqs[i], host);
	}
	dma_free_coherent(&pdev->dev, BUFFER_SIZE, host->virt_base, host->phys_base);
	iounmap(host->reg_base);
	mmc_free_host(host->mmc);
	return 0;
}

static struct platform_driver ka2000_mmc_driver = {
	.probe	= ka2000_mmc_probe,
	.remove	= ka2000_mmc_remove,
	.driver	= {
		.name	= DRIVER_NAME,
	},
};

module_platform_driver(ka2000_mmc_driver);
