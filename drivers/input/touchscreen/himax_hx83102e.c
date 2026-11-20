// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Himax HX83102E SPI touchscreen controller.
 *
 * The controller has no flash: the host loads its firmware into SRAM over
 * SPI after every reset, checks it with the controller's hardware CRC engine,
 * and then reads 10-point touch frames from the event stack.
 *
 * Written from the behaviour of a binary kernel build, not from vendor source.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define HX_MAX_POINTS		10
#define HX_MAX_PARTS		8
#define HX_FRAME_LEN		0x50
#define HX_FRAME_CSUM_LEN	0x38
#define HX_XFER_MAX		128
#define HX_FW_NAME_LEN		128
#define HX_MAX_BUS_ERRORS	20
#define HX_DEFAULT_FW		"himax/hx83102e_ls_hsd_firmware.bin"

/* SPI command bytes */
#define HX_SPI_WRITE		0xf2
#define HX_SPI_READ		0xf3

/* Bus registers (the first byte after the command) */
#define HX_BUS_ADDR		0x00
#define HX_BUS_RDATA		0x08
#define HX_BUS_ACCESS_DIR	0x0c
#define HX_BUS_INCR4		0x0d
#define HX_BUS_CONTI		0x13
#define HX_BUS_EVENT_STACK	0x30
#define HX_CONTI_VAL		0x31
#define HX_INCR4_VAL		0x10

/* Safe-mode handshake bytes, written as `f2 31 27` and `f2 32 95` */
#define HX_SAFE_MODE_A		0x31
#define HX_SAFE_MODE_A_VAL	0x27
#define HX_SAFE_MODE_B		0x32
#define HX_SAFE_MODE_B_VAL	0x95

/* Controller registers, reached through the bus register window */
#define HX_REG_SYSTEM_RESET	0x90000018
#define HX_REG_RELOAD_ACTIVE	0x90000048
#define HX_REG_CTRL_FW		0x9000005c
#define HX_REG_CENTRAL_STATE	0x900000a8
#define HX_REG_ICID		0x900000d0
#define HX_REG_TCON_ON_RST	0x80020020
#define HX_REG_CRC_STATUS	0x80050000
#define HX_REG_CRC_RESULT	0x80050018
#define HX_REG_CRC_ADDR		0x80050020
#define HX_REG_CRC_CMD		0x80050028
#define HX_REG_FW_VERSION	0x10007004
#define HX_REG_SORTING_MODE	0x10007f04
#define HX_REG_FLASH_RELOAD	0x10007f00
#define HX_REG_RAW_OUT_SEL	0x100072ec
#define HX_REG_FRAME_ADDR	0x10007294
#define HX_REG_RELOAD_2ND	0x100072c0
#define HX_REG_MAXPT_XYRVS	0x100070f8

#define HX_SYSTEM_RESET_VAL	0x55
#define HX_CTRL_FW_SENSE_OFF	0xa5
#define HX_RELOAD_ACTIVE_VAL	0xec
#define HX_RELOAD_ACTIVE_DONE	0x1ec
#define HX_RELOAD_2ND_DONE	0x72c0
#define HX_CRC_CMD_BASE		0x00990000
#define HX_STATE_ISR		0x05
#define HX_STATE_SAFE_MODE	0x0c

#define HX_ICID_MASK		0xffff0000
#define HX_ICID_FAMILY		0x83100000
#define HX_ICID_MODEL_MASK	0xffffff00
#define HX_ICID_MODEL		0x83102e00

/* Firmware image layout */
#define HX_FW_SCAN_LIMIT	0x400
#define HX_FW_DESC_MAGIC	0x00000a00
#define HX_FW_DESC_TAG_OFF	0x0e
#define HX_FW_DESC_TAG		0x87
#define HX_FW_TABLE_COUNT_OFF	0x0c

#define HX_CRC_POLY		0x82f63b78

static unsigned int debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "Debug log mask (bit 0: bus and firmware steps, bit 1: touch frames)");

static char *firmware_name;
module_param(firmware_name, charp, 0644);
MODULE_PARM_DESC(firmware_name, "Unused: the device tree firmware-name, or the built-in default, always wins");

static unsigned int init_delay_ms = 1000;
module_param(init_delay_ms, uint, 0644);
MODULE_PARM_DESC(init_delay_ms, "Delay between probe and the first controller initialisation, in ms");

static unsigned int init_retries = 30;
module_param(init_retries, uint, 0644);
MODULE_PARM_DESC(init_retries, "How many times a failed initialisation is retried, 2 s apart");

static unsigned int poll_ms;
module_param(poll_ms, uint, 0644);
MODULE_PARM_DESC(poll_ms, "Poll the controller every this many ms instead of using the interrupt (0: interrupt)");

static bool skip_firmware;
module_param(skip_firmware, bool, 0644);
MODULE_PARM_DESC(skip_firmware, "Stop after identifying the controller; load no firmware");

static unsigned int spi_speed_hz;
module_param(spi_speed_hz, uint, 0644);
MODULE_PARM_DESC(spi_speed_hz, "SPI clock override in Hz (0: use the device tree value)");

static unsigned int sram_chunk = 4096;
module_param(sram_chunk, uint, 0644);
MODULE_PARM_DESC(sram_chunk, "Bytes per SPI transfer when loading firmware (multiple of 4, 4..131072)");

struct hx_part {
	u32 sram;
	u32 size;
	u32 off;
};

struct hx83102e {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset;
	struct regulator *vccd;
	struct regulator *vsp;
	struct regulator *vsn;

	struct mutex lock;		/* serialises bus access and the buffers below */
	struct mutex state_lock;	/* serialises init, stop, suspend and reconnect */
	u8 *tx;				/* write buffer, tx_size bytes */
	u8 *xfer;			/* read buffers: command, then reply */
	u8 *frame;			/* HX_FRAME_LEN bytes */
	u32 chunk;			/* firmware bytes per transfer, fixed at probe */

	const struct firmware *fw;
	char fw_name[HX_FW_NAME_LEN];
	struct hx_part parts[HX_MAX_PARTS];	/* [0] is the main code */
	u32 cfg_sram;
	u32 cfg_size;
	u32 cfg_crc;
	u8 *cfg_buf;

	struct touchscreen_properties prop;
	u32 max_points;
	int irq;
	bool irq_enabled;
	struct delayed_work init_work;
	struct delayed_work poll_work;
	u32 init_attempts;
	u32 bus_errors;
	bool ready;
	bool suspended;
	bool powered;
};

/* Bit-wise CRC32C-style checksum (reflected 0x82f63b78) over whole 32-bit words. */
static u32 hx_crc(const u8 *buf, u32 len)
{
	u32 crc = 0xffffffff;
	u32 i;
	int bit;

	for (i = 0; i + 4 <= len; i += 4) {
		crc ^= get_unaligned_le32(buf + i);
		for (bit = 0; bit < 32; bit++)
			crc = (crc & 1) ? (crc >> 1) ^ HX_CRC_POLY : crc >> 1;
	}
	return crc;
}

static void hx_reset(struct hx83102e *hx)
{
	if (!hx->reset)
		return;
	gpiod_set_value_cansleep(hx->reset, 1);
	usleep_range(3000, 3100);
	gpiod_set_value_cansleep(hx->reset, 0);
	usleep_range(2000, 2100);
}

static void hx_reset_long(struct hx83102e *hx)
{
	if (!hx->reset)
		return;
	gpiod_set_value_cansleep(hx->reset, 1);
	msleep(10);
	gpiod_set_value_cansleep(hx->reset, 0);
	msleep(20);
}

/* The caller holds hx->lock for everything below that touches hx->tx or hx->xfer. */

static int hx_write_cmd(struct hx83102e *hx, const u8 *cmd, size_t len)
{
	if (len > HX_XFER_MAX)
		return -EINVAL;
	memcpy(hx->tx, cmd, len);
	return spi_write(hx->spi, hx->tx, len);
}

static int hx_bus_read(struct hx83102e *hx, u8 reg, void *buf, size_t len)
{
	u8 *txb = hx->xfer;
	u8 *rxb = hx->xfer + HX_XFER_MAX;
	struct spi_transfer xfer = {
		.tx_buf = txb,
		.rx_buf = rxb,
		.len = len + 3,
	};
	int ret;

	if (len + 3 > HX_XFER_MAX)
		return -EINVAL;

	memset(txb, 0, len + 3);
	txb[0] = HX_SPI_READ;
	txb[1] = reg;
	ret = spi_sync_transfer(hx->spi, &xfer, 1);
	if (!ret)
		memcpy(buf, rxb + 3, len);
	return ret;
}

static int hx_burst_enable(struct hx83102e *hx, bool en)
{
	u8 conti[] = { HX_SPI_WRITE, HX_BUS_CONTI, HX_CONTI_VAL };
	u8 incr[] = { HX_SPI_WRITE, HX_BUS_INCR4, HX_INCR4_VAL | en };

	int ret = hx_write_cmd(hx, conti, sizeof(conti));

	if (ret)
		return ret;
	return hx_write_cmd(hx, incr, sizeof(incr));
}

/* Make sure the controller is in burst mode with auto-increment. */
static int hx_interface_on(struct hx83102e *hx)
{
	u8 conti = 0, incr = 0;
	u8 dummy[4];
	int ret, i;

	mutex_lock(&hx->lock);
	ret = hx_bus_read(hx, HX_BUS_RDATA, dummy, sizeof(dummy));
	mutex_unlock(&hx->lock);
	if (ret)
		return ret;

	for (i = 0; i < 10; i++) {
		mutex_lock(&hx->lock);
		ret = hx_burst_enable(hx, false);
		if (!ret)
			ret = hx_bus_read(hx, HX_BUS_CONTI, &conti, 1);
		if (!ret)
			ret = hx_bus_read(hx, HX_BUS_INCR4, &incr, 1);
		mutex_unlock(&hx->lock);
		if (ret)
			return ret;

		if (conti == HX_CONTI_VAL && incr == HX_INCR4_VAL) {
			if (i && debug)
				dev_info(&hx->spi->dev, "burst mode ok after %d tries\n", i);
			return 0;
		}
		usleep_range(1000, 1100);
	}

	dev_warn(&hx->spi->dev, "burst mode did not stick (0x13=0x%02x 0x0d=0x%02x)\n",
		 conti, incr);
	return 0;
}

static int hx_reg_read(struct hx83102e *hx, u32 addr, u32 *val)
{
	u8 setaddr[6] = { HX_SPI_WRITE, HX_BUS_ADDR };
	u8 dir[] = { HX_SPI_WRITE, HX_BUS_ACCESS_DIR, 0x00 };
	__le32 data;
	int ret;

	if (debug)
		dev_info(&hx->spi->dev, "r %08x len %zu\n", addr, sizeof(*val));

	put_unaligned_le32(addr, &setaddr[2]);

	mutex_lock(&hx->lock);
	ret = hx_burst_enable(hx, false);
	if (!ret)
		ret = hx_write_cmd(hx, setaddr, sizeof(setaddr));
	if (!ret)
		ret = hx_write_cmd(hx, dir, sizeof(dir));
	if (!ret)
		ret = hx_bus_read(hx, HX_BUS_RDATA, &data, sizeof(data));
	mutex_unlock(&hx->lock);
	if (!ret)
		*val = le32_to_cpu(data);
	return ret;
}

static int hx_reg_write(struct hx83102e *hx, u32 addr, u32 val)
{
	u8 cmd[10] = { HX_SPI_WRITE, HX_BUS_ADDR };
	int ret;

	if (debug)
		dev_info(&hx->spi->dev, "w %08x len %zu\n", addr, sizeof(val));

	put_unaligned_le32(addr, &cmd[2]);
	put_unaligned_le32(val, &cmd[6]);

	mutex_lock(&hx->lock);
	ret = hx_burst_enable(hx, false);
	if (!ret)
		ret = hx_write_cmd(hx, cmd, sizeof(cmd));
	mutex_unlock(&hx->lock);
	return ret;
}

/* Ask the firmware to stop scanning, then force the controller into safe mode. */
static int hx_sense_off(struct hx83102e *hx)
{
	static const u8 safe_a[] = { HX_SPI_WRITE, HX_SAFE_MODE_A, HX_SAFE_MODE_A_VAL };
	static const u8 safe_b[] = { HX_SPI_WRITE, HX_SAFE_MODE_B, HX_SAFE_MODE_B_VAL };
	struct device *dev = &hx->spi->dev;
	u32 val = 0;
	u32 state;
	int ret, i;

	ret = hx_reg_write(hx, HX_REG_CTRL_FW, HX_CTRL_FW_SENSE_OFF);
	if (ret)
		return ret;
	usleep_range(100, 200);

	ret = hx_reg_read(hx, HX_REG_CENTRAL_STATE, &val);
	if (ret)
		return ret;
	state = val & 0xff;

	if (state == HX_STATE_ISR) {
		usleep_range(10000, 11000);
		val = HX_STATE_ISR;
		ret = hx_reg_read(hx, HX_REG_CTRL_FW, &val);
		if (ret)
			return ret;
		state = val & 0xff;
		if (debug)
			dev_info(dev, "fw isr 0x%02x (pass %d)\n", state, 0);
	}

	for (i = 0; i < 5; i++) {
		mutex_lock(&hx->lock);
		ret = hx_write_cmd(hx, safe_a, sizeof(safe_a));
		mutex_unlock(&hx->lock);
		if (ret)
			return ret;

		mutex_lock(&hx->lock);
		ret = hx_write_cmd(hx, safe_b, sizeof(safe_b));
		mutex_unlock(&hx->lock);
		if (ret)
			return ret;

		val = state;
		ret = hx_reg_read(hx, HX_REG_CENTRAL_STATE, &val);
		if (ret)
			return ret;
		state = val & 0xff;
		if (debug)
			dev_info(dev, "central state 0x%02x (try %d)\n", state, i);

		if (state == HX_STATE_SAFE_MODE)
			return hx_reg_write(hx, HX_REG_TCON_ON_RST, 0);

		usleep_range(5000, 5100);
		hx_reset_long(hx);
	}

	dev_err(dev, "failed to enter safe mode\n");
	return -ETIMEDOUT;
}

/*
 * Write @len bytes to SRAM at @addr and have the controller's hardware CRC
 * engine check them against @expected, retrying the whole transfer up to
 * three times.
 */
static int hx_sram_write_crc(struct hx83102e *hx, u32 addr, const u8 *buf,
			     u32 len, u32 expected)
{
	struct device *dev = &hx->spi->dev;
	u32 val, off, n, crc;
	u8 cmd[10] = { HX_SPI_WRITE, HX_BUS_ADDR };
	int ret, tries, polls;

	for (tries = 0; tries < 3; ) {
		for (off = 0; off < len; off += hx->chunk) {
			n = min(len - off, hx->chunk);

			mutex_lock(&hx->lock);
			ret = hx_burst_enable(hx, true);
			if (ret) {
				mutex_unlock(&hx->lock);
				return ret;
			}
			hx->tx[0] = HX_SPI_WRITE;
			hx->tx[1] = HX_BUS_ADDR;
			put_unaligned_le32(addr + off, &hx->tx[2]);
			memcpy(&hx->tx[6], buf + off, n);
			ret = spi_write(hx->spi, hx->tx, n + 6);
			mutex_unlock(&hx->lock);
			if (ret)
				return ret;
			udelay(100);
		}

		ret = hx_reg_write(hx, HX_REG_CRC_ADDR, addr);
		if (ret)
			return ret;

		mutex_lock(&hx->lock);
		ret = hx_burst_enable(hx, false);
		if (ret) {
			mutex_unlock(&hx->lock);
			return ret;
		}
		put_unaligned_le32(HX_REG_CRC_CMD, &cmd[2]);
		put_unaligned_le32(HX_CRC_CMD_BASE | ((len >> 2) & 0xffff), &cmd[6]);
		ret = hx_write_cmd(hx, cmd, sizeof(cmd));
		mutex_unlock(&hx->lock);
		if (ret)
			return ret;

		for (polls = 0; ; ) {
			ret = hx_reg_read(hx, HX_REG_CRC_STATUS, &val);
			if (ret)
				return ret;
			if (!(val & 1))
				break;
			usleep_range(1000, 1100);
			if (++polls == 100) {
				dev_err(dev, "hardware CRC engine stuck\n");
				return -ETIMEDOUT;
			}
		}

		ret = hx_reg_read(hx, HX_REG_CRC_RESULT, &crc);
		if (ret)
			return ret;
		if (debug)
			dev_info(dev, "hw crc 0x%08x+%u = 0x%08x\n", addr, len, crc);
		if (crc == expected)
			return 0;

		tries++;
		dev_warn(dev, "SRAM CRC at 0x%08x: got 0x%08x want 0x%08x (try %d)\n",
			 addr, crc, expected, tries);
	}
	return -EIO;
}

static void hx_report_frame(struct hx83102e *hx, const u8 *f)
{
	struct input_dev *input = hx->input;
	u32 count, i;
	u8 sum = 0;

	if (!memchr_inv(f, 0, HX_FRAME_CSUM_LEN))
		return;

	for (i = 0; i < HX_FRAME_CSUM_LEN; i++)
		sum += f[i];
	if (sum) {
		if (debug & 2)
			dev_info(&hx->spi->dev, "frame checksum mismatch\n");
		return;
	}

	/* Only the touch frame (selector 0) and its alias 15 carry points. */
	if (!((0x8001UL >> (f[0x34] >> 4)) & 1)) {
		if (debug & 2)
			dev_info(&hx->spi->dev, "skipping raw-data frame (sel 0x%x)\n",
				 f[0x34] >> 4);
		return;
	}

	count = min_t(u32, f[0x34] & 0xf, hx->max_points);
	for (i = 0; i < hx->max_points; i++) {
		u32 x = get_unaligned_be16(f + 4 * i);
		u32 y = get_unaligned_be16(f + 4 * i + 2);
		bool valid = i < count && x <= hx->prop.max_x && y <= hx->prop.max_y;

		input_event(input, EV_ABS, ABS_MT_SLOT, i);
		if (input_mt_report_slot_state(input, MT_TOOL_FINGER, valid)) {
			touchscreen_report_pos(input, &hx->prop, x, y, true);
			input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, f[0x28 + i]);
		}
	}

	input_mt_sync_frame(input);
	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
}

/* Read one event frame and report it; after too many failures ask for a re-init. */
static void hx_read_frame(struct hx83102e *hx, bool clear_errors)
{
	int ret;

	mutex_lock(&hx->lock);
	ret = hx_bus_read(hx, HX_BUS_EVENT_STACK, hx->frame, HX_FRAME_LEN);
	mutex_unlock(&hx->lock);

	if (!ret) {
		if (clear_errors)
			hx->bus_errors = 0;
		hx_report_frame(hx, hx->frame);
		return;
	}

	if (++hx->bus_errors > HX_MAX_BUS_ERRORS) {
		dev_err(&hx->spi->dev, "too many bus errors, reinitialising\n");
		hx->bus_errors = 0;
		hx->ready = false;
		schedule_delayed_work(&hx->init_work, 0);
	}
}

static irqreturn_t hx_irq_thread(int irq, void *data)
{
	struct hx83102e *hx = data;

	if (hx->ready)
		hx_read_frame(hx, true);
	return IRQ_HANDLED;
}

static void hx_poll_work(struct work_struct *work)
{
	struct hx83102e *hx = container_of(work, struct hx83102e, poll_work.work);

	if (!hx->ready || hx->suspended)
		return;

	hx_read_frame(hx, false);
	schedule_delayed_work(&hx->poll_work, msecs_to_jiffies(poll_ms));
}

/* Locate the config table in the firmware image and fill hx->parts[]. */
static int hx_parse_firmware(struct hx83102e *hx)
{
	struct device *dev = &hx->spi->dev;
	const struct firmware *fw = hx->fw;
	u32 off, limit, table = 0, count, i, min_sram, max_sram, max_idx;
	bool found = false;

	if (fw->size < 0x10)
		goto no_table;

	limit = min_t(size_t, fw->size, HX_FW_SCAN_LIMIT);
	for (off = 0; ; off += 0x10) {
		const u8 *blk = fw->data + off;

		if (!memchr_inv(blk, 0, 0x10))
			break;
		if (blk[HX_FW_DESC_TAG_OFF] == HX_FW_DESC_TAG &&
		    get_unaligned_le32(blk) == HX_FW_DESC_MAGIC) {
			table = get_unaligned_le32(blk + 4);
			found = true;
			break;
		}
		if (limit < off + 0x20)
			break;
	}
	if (!found) {
no_table:
		dev_err(dev, "no TP_CONFIG_TABLE descriptor in firmware\n");
		return -EINVAL;
	}

	if (debug)
		dev_info(dev, "config table at 0x%x\n", table);
	if ((u64)table + 0x10 > fw->size) {
		dev_err(dev, "config table outside firmware\n");
		return -EINVAL;
	}

	count = fw->data[table + HX_FW_TABLE_COUNT_OFF];
	if (count < 2 || count > HX_MAX_PARTS) {
		dev_err(dev, "bad partition count %u\n", count);
		return -EINVAL;
	}
	/* Not checked by the original; keeps the entry reads below inside the image. */
	if ((u64)table + 0x10 * count > fw->size) {
		dev_err(dev, "config table outside firmware\n");
		return -EINVAL;
	}

	min_sram = 0xffffffff;
	max_sram = 0;
	max_idx = 0;
	for (i = 0; i < count; i++) {
		const u8 *e = fw->data + table + 0x10 * i;
		struct hx_part *p = &hx->parts[i];

		p->sram = get_unaligned_le32(e);
		p->size = get_unaligned_le32(e + 4);
		p->off = get_unaligned_le32(e + 8);
		if (fw->size < (u64)p->size + p->off) {
			dev_err(dev, "partition %u runs past the end of the firmware\n", i);
			return -EINVAL;
		}
		if (debug)
			dev_info(dev, "part %u: sram 0x%08x size %u file 0x%x\n",
				 i, p->sram, p->size, p->off);
		if (!i)
			continue;
		min_sram = min(min_sram, p->sram);
		if (p->sram > max_sram) {
			max_sram = p->sram;
			max_idx = i;
		}
	}
	if (min_sram == 0xffffffff) {
		dev_err(dev, "firmware has no config partitions\n");
		return -EINVAL;
	}

	hx->cfg_sram = min_sram & ~3U;
	hx->cfg_size = ALIGN(max_sram + hx->parts[max_idx].size - hx->cfg_sram, 16);
	hx->cfg_buf = devm_kzalloc(dev, hx->cfg_size, GFP_KERNEL);
	if (!hx->cfg_buf)
		return -ENOMEM;

	for (i = 1; i < count; i++)
		memmove(hx->cfg_buf + (hx->parts[i].sram - hx->cfg_sram),
			fw->data + hx->parts[i].off, hx->parts[i].size);
	hx->cfg_crc = hx_crc(hx->cfg_buf, hx->cfg_size);

	/* The main code image must checksum to zero over its own CRC word. */
	if (hx_crc(fw->data + hx->parts[0].off, hx->parts[0].size)) {
		dev_err(dev, "main code CRC 0x%08x, expected 0: wrong or corrupt image\n",
			hx_crc(fw->data + hx->parts[0].off, hx->parts[0].size));
		devm_kfree(dev, hx->cfg_buf);
		hx->cfg_buf = NULL;
		return -EINVAL;
	}

	dev_info(dev, "firmware %s: main 0x%08x+%u, config 0x%08x+%u crc 0x%08x\n",
		 hx->fw_name, hx->parts[0].sram, hx->parts[0].size,
		 hx->cfg_sram, hx->cfg_size, hx->cfg_crc);
	return 0;
}

static int hx_power_on(struct hx83102e *hx)
{
	struct device *dev = &hx->spi->dev;
	int ret;

	if (hx->vccd) {
		ret = regulator_enable(hx->vccd);
		if (ret) {
			dev_err(dev, "vccd enable failed: %d\n", ret);
			return ret;
		}
		usleep_range(1000, 2000);
	}
	if (hx->vsp) {
		ret = regulator_enable(hx->vsp);
		if (ret) {
			dev_err(dev, "vsp enable failed: %d\n", ret);
			return ret;
		}
		usleep_range(1000, 2000);
	}
	if (hx->vsn) {
		ret = regulator_enable(hx->vsn);
		if (ret) {
			dev_err(dev, "vsn enable failed: %d\n", ret);
			return ret;
		}
		usleep_range(1000, 2000);
	}
	hx->powered = true;
	return 0;
}

/* Identify the chip; returns 0, a bus error or -ENODEV. */
static int hx_identify(struct hx83102e *hx)
{
	struct device *dev = &hx->spi->dev;
	u32 id = 0;
	int ret, i;

	for (i = 0; i < 5; i++) {
		ret = hx_reg_read(hx, HX_REG_ICID, &id);
		if (ret)
			return ret;
		dev_info(dev, "IC ID %02x %02x %02x %02x\n",
			 id >> 24, (id >> 16) & 0xff, (id >> 8) & 0xff, id & 0xff);
		if ((id & HX_ICID_MASK) == HX_ICID_FAMILY) {
			if ((id & HX_ICID_MODEL_MASK) != HX_ICID_MODEL)
				dev_warn(dev, "IC ID 0x%08x is not a HX83102E, continuing\n", id);
			return 0;
		}
		usleep_range(1000, 1100);
	}
	dev_err(dev, "unexpected IC ID 0x%08x\n", id);
	return -ENODEV;
}

/* Load the firmware and wait for it to start; the caller holds state_lock. */
static int hx_load_and_start(struct hx83102e *hx)
{
	struct device *dev = &hx->spi->dev;
	u32 val = 0;
	int ret, i;

	ret = hx_reg_write(hx, HX_REG_SYSTEM_RESET, HX_SYSTEM_RESET_VAL);
	if (!ret)
		ret = hx_sense_off(hx);
	if (!ret)
		ret = hx_sram_write_crc(hx, hx->parts[0].sram,
					hx->fw->data + hx->parts[0].off,
					hx->parts[0].size, 0);
	if (!ret)
		ret = hx_sram_write_crc(hx, hx->cfg_sram, hx->cfg_buf,
					hx->cfg_size, hx->cfg_crc);
	if (ret) {
		dev_err(dev, "firmware download failed: %d\n", ret);
		return ret;
	}

	ret = hx_reg_write(hx, HX_REG_FLASH_RELOAD, 0);
	if (ret)
		return ret;
	ret = hx_reg_write(hx, HX_REG_RAW_OUT_SEL, 0);
	if (ret)
		return ret;

	for (i = 0; ; ) {
		ret = hx_reg_write(hx, HX_REG_SORTING_MODE, 0);
		if (ret)
			return ret;
		usleep_range(1000, 1100);
		ret = hx_reg_read(hx, HX_REG_SORTING_MODE, &val);
		if (ret)
			return ret;
		if (!val)
			break;
		if (++i == 5) {
			dev_err(dev, "sorting mode write failed\n");
			return -EIO;
		}
	}

	ret = hx_reg_write(hx, HX_REG_FRAME_ADDR, 1);
	if (ret)
		return ret;
	ret = hx_reg_write(hx, HX_REG_RELOAD_2ND, 0);
	if (ret)
		return ret;
	ret = hx_interface_on(hx);
	if (ret)
		return ret;
	ret = hx_reg_write(hx, HX_REG_CTRL_FW, 0);
	if (ret)
		return ret;
	usleep_range(100, 200);
	hx_reset(hx);

	val = 0;
	for (i = 0; ; ) {
		ret = hx_reg_write(hx, HX_REG_RELOAD_ACTIVE, HX_RELOAD_ACTIVE_VAL);
		if (ret)
			return ret;
		usleep_range(1000, 1100);
		ret = hx_reg_read(hx, HX_REG_RELOAD_ACTIVE, &val);
		if (ret)
			return ret;
		if (debug)
			dev_info(dev, "reload -> 0x%02x%02x\n", (val >> 8) & 0xff, val & 0xff);
		if (val == HX_RELOAD_ACTIVE_DONE)
			break;
		if (++i == 5) {
			dev_err(dev, "reload-to-active failed\n");
			return -ETIMEDOUT;
		}
	}

	val = 0;
	for (i = 0; ; ) {
		ret = hx_reg_read(hx, HX_REG_RELOAD_2ND, &val);
		if (ret)
			return ret;
		if (val == HX_RELOAD_2ND_DONE)
			break;
		usleep_range(5000, 5100);
		if (++i == 30) {
			dev_err(dev, "firmware reload did not complete\n");
			return -ETIMEDOUT;
		}
	}
	if (debug)
		dev_info(dev, "firmware reload done (%d ms)\n", i * 5);

	if (!hx_reg_read(hx, HX_REG_FW_VERSION, &val))
		dev_info(dev, "firmware version 0x%08x\n", val);
	if (!hx_reg_read(hx, HX_REG_MAXPT_XYRVS, &val) && debug)
		dev_info(dev, "0x%08x = 0x%08x\n", HX_REG_MAXPT_XYRVS, val);

	/* Flush the frames queued while the firmware was starting. */
	for (i = 0; i < 3; i++) {
		int err;

		mutex_lock(&hx->lock);
		err = hx_bus_read(hx, HX_BUS_EVENT_STACK, hx->frame, HX_FRAME_LEN);
		mutex_unlock(&hx->lock);
		if (err)
			break;
	}
	return 0;
}

static void hx_init_work(struct work_struct *work)
{
	struct hx83102e *hx = container_of(work, struct hx83102e, init_work.work);
	struct device *dev = &hx->spi->dev;
	u32 val = 0;
	int ret;

	mutex_lock(&hx->state_lock);
	if (hx->ready)
		goto out_unlock;

	if (!hx->powered) {
		ret = hx_power_on(hx);
		if (ret)
			goto fail;
	}

	ret = hx_interface_on(hx);
	if (ret)
		goto err;
	hx_reset(hx);

	mutex_lock(&hx->lock);
	ret = hx_bus_read(hx, HX_BUS_CONTI, &val, 1);
	mutex_unlock(&hx->lock);
	if (ret) {
		dev_err(dev, "bus read failed: %d\n", ret);
		goto err;
	}
	if (debug)
		dev_info(dev, "0x13 reads 0x%02x\n", val & 0xff);

	if (hx_sense_off(hx)) {
		dev_warn(dev, "safe mode handshake failed, retrying after a long reset\n");
		hx_reset_long(hx);
		if (hx_sense_off(hx))
			dev_warn(dev, "safe mode handshake failed again, reading the ID anyway\n");
	}

	ret = hx_identify(hx);
	if (ret)
		goto err;

	if (skip_firmware) {
		dev_info(dev, "skip_firmware=1: stopping after identification\n");
		goto out_unlock;
	}

	if (!hx->fw) {
		ret = request_firmware(&hx->fw, hx->fw_name, dev);
		if (ret) {
			dev_info(dev, "firmware %s not available: %d\n", hx->fw_name, ret);
			goto err;
		}
		ret = hx_parse_firmware(hx);
		if (ret) {
			release_firmware(hx->fw);
			hx->fw = NULL;
			goto err;
		}
	}

	ret = hx_load_and_start(hx);
	if (ret)
		goto err;

	hx->bus_errors = 0;
	hx->ready = true;
	if (!poll_ms) {
		if (hx->irq > 0 && !hx->irq_enabled) {
			enable_irq(hx->irq);
			hx->irq_enabled = true;
		}
	} else {
		schedule_delayed_work(&hx->poll_work, 0);
	}
	dev_info(dev, "touch ready: %u points, %ux%u, %s\n", hx->max_points,
		 hx->prop.max_x + 1, hx->prop.max_y + 1, poll_ms ? "polling" : "interrupt");

out_unlock:
	mutex_unlock(&hx->state_lock);
	return;

err:
	if (hx->irq > 0 && hx->irq_enabled) {
		disable_irq(hx->irq);
		hx->irq_enabled = false;
	}
fail:
	mutex_unlock(&hx->state_lock);
	if (hx->init_attempts++ < init_retries) {
		dev_info(dev, "init failed (%d), retry %u/%u in 2 s\n", ret,
			 hx->init_attempts, init_retries);
		schedule_delayed_work(&hx->init_work, 2 * HZ);
	} else {
		dev_err(dev, "giving up after %u attempts\n", hx->init_attempts);
	}
}

/* The caller holds state_lock. */
static void hx_stop(struct hx83102e *hx)
{
	if (!hx->ready)
		return;

	hx->ready = false;
	if (!poll_ms && hx->irq > 0 && hx->irq_enabled) {
		disable_irq(hx->irq);
		hx->irq_enabled = false;
	}
	cancel_delayed_work_sync(&hx->poll_work);
	if (hx_sense_off(hx))
		dev_warn(&hx->spi->dev, "sense-off on stop failed\n");
}

static int hx_suspend(struct device *dev)
{
	struct hx83102e *hx = dev_get_drvdata(dev);

	mutex_lock(&hx->state_lock);
	hx->suspended = true;
	hx_stop(hx);
	mutex_unlock(&hx->state_lock);
	return 0;
}

static int hx_resume(struct device *dev)
{
	struct hx83102e *hx = dev_get_drvdata(dev);

	mutex_lock(&hx->state_lock);
	hx->suspended = false;
	mutex_unlock(&hx->state_lock);
	schedule_delayed_work(&hx->init_work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(hx_pm_ops, hx_suspend, hx_resume);

static ssize_t icid_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct hx83102e *hx = dev_get_drvdata(dev);
	u32 id = 0;
	int ret;

	mutex_lock(&hx->state_lock);
	ret = hx_reg_read(hx, HX_REG_ICID, &id);
	mutex_unlock(&hx->state_lock);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%02x %02x %02x %02x\n",
			  id >> 24, (id >> 16) & 0xff, (id >> 8) & 0xff, id & 0xff);
}
static DEVICE_ATTR_RO(icid);

static ssize_t reconnect_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct hx83102e *hx = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&hx->init_work);
	mutex_lock(&hx->state_lock);
	hx_stop(hx);
	hx->init_attempts = 0;
	mutex_unlock(&hx->state_lock);
	schedule_delayed_work(&hx->init_work, 0);
	return count;
}
static DEVICE_ATTR_WO(reconnect);

static struct attribute *hx_attrs[] = {
	&dev_attr_reconnect.attr,
	&dev_attr_icid.attr,
	NULL
};
ATTRIBUTE_GROUPS(hx);

static void hx_remove(struct spi_device *spi)
{
	struct hx83102e *hx = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&hx->init_work);
	cancel_delayed_work_sync(&hx->poll_work);
	mutex_lock(&hx->state_lock);
	hx_stop(hx);
	mutex_unlock(&hx->state_lock);

	if (hx->fw) {
		release_firmware(hx->fw);
		hx->fw = NULL;
	}
	if (hx->powered) {
		hx->powered = false;
		if (hx->vsn)
			regulator_disable(hx->vsn);
		if (hx->vsp)
			regulator_disable(hx->vsp);
		if (hx->vccd)
			regulator_disable(hx->vccd);
	}
}

static struct regulator *hx_get_supply(struct device *dev, const char *id,
				       int *err)
{
	struct regulator *reg = devm_regulator_get_optional(dev, id);

	if (IS_ERR(reg)) {
		if (PTR_ERR(reg) == -ENODEV) {
			reg = NULL;
		} else {
			*err = dev_err_probe(dev, PTR_ERR(reg), "cannot get %s\n", id);
			reg = NULL;
		}
	}
	return reg;
}

static int hx_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct hx83102e *hx;
	const char *name = firmware_name;
	u32 tx_size, abs_max;
	int ret = 0;

	hx = devm_kzalloc(dev, sizeof(*hx), GFP_KERNEL);
	if (!hx)
		return -ENOMEM;

	hx->spi = spi;
	hx->max_points = HX_MAX_POINTS;
	mutex_init(&hx->lock);
	mutex_init(&hx->state_lock);
	INIT_DELAYED_WORK(&hx->init_work, hx_init_work);
	INIT_DELAYED_WORK(&hx->poll_work, hx_poll_work);
	spi_set_drvdata(spi, hx);

	if (!(spi->mode & SPI_MODE_X_MASK)) {
		dev_warn(dev, "device tree has no spi-cpha/spi-cpol, forcing mode 3\n");
		spi->mode = SPI_MODE_3;
	}
	spi->bits_per_word = 8;
	if (spi_speed_hz)
		spi->max_speed_hz = spi_speed_hz;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(dev, "spi_setup failed: %d\n", ret);
		return ret;
	}

	if (sram_chunk >= 4 && sram_chunk <= 0x20000) {
		sram_chunk &= ~3U;
		tx_size = sram_chunk + 6;
	} else {
		sram_chunk = 0x1000;
		tx_size = 0x1006;
	}
	hx->chunk = sram_chunk;

	hx->tx = devm_kzalloc(dev, max_t(u32, tx_size, HX_XFER_MAX), GFP_KERNEL);
	hx->xfer = devm_kzalloc(dev, 2 * HX_XFER_MAX, GFP_KERNEL);
	hx->frame = devm_kzalloc(dev, HX_FRAME_LEN, GFP_KERNEL);
	if (!hx->tx || !hx->xfer || !hx->frame)
		return -ENOMEM;

	hx->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(hx->reset))
		return dev_err_probe(dev, PTR_ERR(hx->reset), "cannot get reset gpio\n");

	hx->vccd = hx_get_supply(dev, "vccd", &ret);
	if (ret)
		return ret;
	hx->vsp = hx_get_supply(dev, "vsp", &ret);
	if (ret)
		return ret;
	hx->vsn = hx_get_supply(dev, "vsn", &ret);
	if (ret)
		return ret;

	/*
	 * The device tree name wins; a missing or empty property always gives
	 * the built-in default, so the firmware_name parameter has no effect.
	 * This is how the binary behaves; see the commit message.
	 */
	if (device_property_read_string(dev, "firmware-name", &name) || !name)
		name = HX_DEFAULT_FW;
	strscpy(hx->fw_name, name, sizeof(hx->fw_name));

	hx->input = devm_input_allocate_device(dev);
	if (!hx->input)
		return -ENOMEM;
	hx->input->name = "himax-hx83102e";
	hx->input->id.bustype = BUS_SPI;
	hx->input->dev.parent = dev;
	input_set_abs_params(hx->input, ABS_MT_POSITION_X, 0, 0x4af, 0, 0);
	input_set_abs_params(hx->input, ABS_MT_POSITION_Y, 0, 0x77f, 0, 0);
	input_set_abs_params(hx->input, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
	input_set_capability(hx->input, EV_KEY, BTN_TOUCH);
	touchscreen_parse_properties(hx->input, true, &hx->prop);
	abs_max = max(hx->prop.max_x, hx->prop.max_y);
	input_set_abs_params(hx->input, ABS_X, 0, abs_max, 0, 0);
	input_set_abs_params(hx->input, ABS_Y, 0, abs_max, 0, 0);

	ret = input_mt_init_slots(hx->input, hx->max_points,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(dev, "input_mt_init_slots failed: %d\n", ret);
		return ret;
	}
	ret = input_register_device(hx->input);
	if (ret) {
		dev_err(dev, "input_register_device failed: %d\n", ret);
		return ret;
	}

	hx->irq = spi->irq;
	if (!poll_ms) {
		if (hx->irq < 1) {
			dev_warn(dev, "no interrupt, polling every 10 ms\n");
			poll_ms = 10;
		} else {
			/*
			 * As in the binary: the line stays enabled after the
			 * request while irq_enabled is false, so the first
			 * init enable_irq() is unbalanced (it warns once).
			 */
			ret = devm_request_threaded_irq(dev, hx->irq, NULL, hx_irq_thread,
							IRQF_ONESHOT, "himax_hx83102e", hx);
			if (ret) {
				dev_err(dev, "cannot request irq %d: %d, polling instead\n",
					hx->irq, ret);
				poll_ms = 10;
				hx->irq = 0;
			}
		}
	} else {
		dev_info(dev, "polling every %u ms\n", poll_ms);
	}

	dev_info(dev, "Himax HX83102E: SPI bus %d cs %u mode 0x%x %u Hz irq %d fw %s\n",
		 spi->controller->bus_num, spi_get_chipselect(spi, 0), spi->mode,
		 spi->max_speed_hz, hx->irq, hx->fw_name);

	schedule_delayed_work(&hx->init_work, msecs_to_jiffies(init_delay_ms));
	return 0;
}

static const struct spi_device_id hx_spi_ids[] = {
	{ "hx83102e" },
	{ "hx83102" },
	{ }
};
MODULE_DEVICE_TABLE(spi, hx_spi_ids);

static const struct of_device_id hx_of_match[] = {
	{ .compatible = "himax,hx83102e" },
	{ }
};
MODULE_DEVICE_TABLE(of, hx_of_match);

static struct spi_driver hx_spi_driver = {
	.driver = {
		.name = "himax_hx83102e",
		.of_match_table = hx_of_match,
		.dev_groups = hx_groups,
		.pm = pm_sleep_ptr(&hx_pm_ops),
	},
	.id_table = hx_spi_ids,
	.probe = hx_probe,
	.remove = hx_remove,
};
module_spi_driver(hx_spi_driver);

MODULE_DESCRIPTION("Himax HX83102E SPI touchscreen driver");
MODULE_LICENSE("GPL");
