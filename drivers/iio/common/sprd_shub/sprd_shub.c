// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc sensor hub IIO driver
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/iio/buffer.h>
#include <linux/iio/common/sprd_shub.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define SHUB_MSG_MAGIC_CHAR		'~'
#define SHUB_MSG_MAGIC_COUNT		4

enum {
	SHUB_ACCELEROMETER_SENSOR = 1,
	SHUB_GEOMAGNETIC_SENSOR = 2,
	SHUB_ORIENTATION_SENSOR = 3,
	SHUB_GYROSCOPE_SENSOR = 4,
	SHUB_LIGHT_SENSOR = 5,
	SHUB_PROXIMITY_SENSOR = 8,
	SHUB_SAMSUNG_CALIB_SENSOR = 0x1a,
};

enum shub_dialect {
	SHUB_DIALECT_GENERIC = 0,
	SHUB_DIALECT_SAMSUNG = 1,
};

struct shub_dialect_data {
	enum shub_dialect dialect;
};

struct shub_msg_hdr {
	u8 magic[SHUB_MSG_MAGIC_COUNT];
	u8 sensor;
	u8 type;
	__be16 length;
	__be16 checksum;
} __packed;

struct shub_batch_cmd {
	__le32 sample_interval_us;
	__le32 batch_interval_us;
	__le32 reserved;
} __packed;

struct shub_data_evt {
	u8 handle;
	u8 reserved;
	u8 data[24];
	__le64 timestamp;
} __packed;

struct shub_sensor_info {
	char model[20];
	char manufacturer[20];
	__le32 version;
	__le32 handle;
	__le32 maxrange;
	__le32 resolution;
	__le32 power;
	__le32 mindelay_us;
	__le32 fifo_reserved_event_count;
	__le32 fifo_max_event_count;
	__le32 maxdelay_us;
} __packed;

struct shub_timesync_cmd {
	__le64 boottime;
	__le64 systimer_cnt;
	__le64 sysfrt_cnt;
	__le32 systimer_mult;
	__le32 systimer_shift;
	__le32 sysfrt_mult;
	__le32 sysfrt_shift;
} __packed;

enum {
	SHUB_CMD_ENABLE = 0x00,
	SHUB_CMD_DISABLE = 0x01,
	SHUB_CMD_SET_BATCH = 0x02,
	SHUB_EVT_DATA = 0x04,
	SHUB_EVT_CM4_LOG = 0x05,
	SHUB_CMD_DOWNLOAD_OPCODE = 0x06,
	SHUB_CMD_DOWNLOAD_CALIB = 0x07,
	SHUB_EVT_RESPONSE = 0x08,
	SHUB_CMD_GET_SENSOR_INFO = 0x09,
	SHUB_CMD_SET_CALIB_DATA = 0xc2,
	SHUB_CMD_START_CALIBRATION = 0xc3,
	SHUB_CMD_SET_TIMESYNC = 0xc4,
	SHUB_CMD_SET_HOST_STATUS = 0xc5,
	SHUB_CMD_GET_CALIB_DATA = 0xc6,
	SHUB_CMD_GET_FWVERSION = 0xcd,
};

#define SHUB_SAMSUNG_DATA_FLAG		0x81
#define SHUB_SAMSUNG_FLUSH_FLAG		0x82
#define SHUB_HOST_STATUS_NORMAL		0x02

enum shub_msg_rx_state {
	SHUB_MSG_RX_SYNC,
	SHUB_MSG_RX_HEADER,
	SHUB_MSG_RX_DATA,
};

struct sprd_shub;

struct sprd_shub_ep {
	struct sprd_shub *shub;
	struct rpmsg_endpoint *ept;
	u8 buf_id;
	enum shub_msg_rx_state rx_state;
	size_t rx_pos;
	struct shub_msg_hdr rx_hdr;
	u8 rx_data[256];
};

struct sprd_shub {
	struct device *dev;
	struct device *parent;
	enum shub_dialect dialect;
	struct rpmsg_endpoint *ept;
	struct sprd_shub_ep *eps[2];
	struct list_head list;
	int refcount;

	struct delayed_work init_work;
	unsigned int wait_tries;
	bool init_delayed;

	struct mutex tx_lock;
	struct mutex cmd_lock;
	struct completion cmd_done;
	u16 wait_subtype;
	bool waiting;
	u8 rsp_status;
	u32 fw_version;

	spinlock_t lock;
	struct idr sensors;
};

struct sprd_shub_sensor {
	struct sprd_shub *shub;
	struct platform_device *pdev;
	struct work_struct reg_work;
	u8 id;
};

static LIST_HEAD(shub_cores);
static DEFINE_MUTEX(shub_cores_lock);

static char *shub_fw_prefix = "sprd/sensorhub";
module_param_named(fw_prefix, shub_fw_prefix, charp, 0444);

static const char *const shub_accel_fw[] = {
	"accelerometer_icm42607",
	"accelerometer_bmi320",
	"accelerometer_bmi220",
	NULL,
};

static const char *const shub_gyro_fw[] = {
	"gyroscope_icm42607",
	"gyroscope_bmi320",
	"gyroscope_bmi220",
	NULL,
};

static const char *const shub_mag_fw[] = {
	"magnetic_mmc5603",
	"magnetic_qmc6308",
	NULL,
};

static const char *const shub_light_fw[] = {
	"light_ltr578als",
	"light_ls8500",
	"light_stk3x1x",
	"dual_light",
	NULL,
};

static u16 shub_msg_checksum(const void *data, size_t len)
{
	const __be16 *buf = data;
	u32 sum = 0;

	while (len >= 2) {
		sum += be16_to_cpu(*buf++);
		len -= 2;
	}

	if (len == 1)
		sum += be16_to_cpu(*buf) & 0xff00;

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);

	return ~sum;
}

static inline u8 shub_sensor_handle(struct sprd_shub *shub, u8 sensor_id)
{
	return shub->dialect == SHUB_DIALECT_SAMSUNG ? sensor_id : 100 + sensor_id;
}

static const char *shub_sensor_type_names[] = {
	[SHUB_ACCELEROMETER_SENSOR] = "sprd-shub-accel",
	[SHUB_GEOMAGNETIC_SENSOR] = "sprd-shub-magn",
	[SHUB_ORIENTATION_SENSOR] = "sprd-shub-compass",
	[SHUB_GYROSCOPE_SENSOR] = "sprd-shub-gyro",
	[SHUB_LIGHT_SENSOR] = "sprd-shub-light",
	[SHUB_PROXIMITY_SENSOR] = "sprd-shub-proximity",
};

static void shub_sensor_register(struct work_struct *work)
{
	struct sprd_shub_sensor *sensor =
		container_of(work, struct sprd_shub_sensor, reg_work);
	struct sprd_shub *shub = sensor->shub;
	struct platform_device *pdev;
	const char *name = NULL;

	if (sensor->id < ARRAY_SIZE(shub_sensor_type_names))
		name = shub_sensor_type_names[sensor->id];

	if (!name) {
		dev_warn(shub->dev, "unknown sensor ID %d\n", sensor->id);
		return;
	}

	pdev = platform_device_register_data(shub->dev, name, sensor->id,
					     &sensor, sizeof(sensor));
	if (IS_ERR(pdev))
		dev_err(shub->dev, "failed to register %s: %pe\n", name, pdev);

	spin_lock_irq(&shub->lock);
	sensor->pdev = pdev;
	spin_unlock_irq(&shub->lock);
}

static void shub_add_sensor(struct sprd_shub *shub, u8 sensor_id)
{
	struct sprd_shub_sensor *sensor;
	unsigned long flags;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_ATOMIC);
	if (!sensor)
		return;

	sensor->shub = shub;
	sensor->id = sensor_id;
	INIT_WORK(&sensor->reg_work, shub_sensor_register);

	spin_lock_irqsave(&shub->lock, flags);
	ret = idr_alloc(&shub->sensors, sensor, sensor_id, sensor_id + 1, GFP_ATOMIC);
	if (ret == sensor_id) {
		schedule_work(&sensor->reg_work);
	} else {
		dev_err(shub->dev, "sensor %d already allocated\n", sensor_id);
		kfree(sensor);
	}
	spin_unlock_irqrestore(&shub->lock, flags);
}

static int shub_send_msg(struct sprd_shub *shub, u8 sensor, u8 type,
			 const void *data, size_t data_size)
{
	static u8 dummy_data = 0xff;
	struct shub_msg_hdr *hdr;
	size_t total, off = 0;
	ssize_t mtu;
	u8 *frame;
	int ret = 0;

	if (!shub->ept)
		return -ENOTCONN;

	if (data_size > U16_MAX)
		return -EINVAL;

	if (data_size == 0) {
		data = &dummy_data;
		data_size = 1;
	}

	dev_dbg(shub->dev, "send command: 0x%02x 0x%02x, data size 0x%04zx\n",
		sensor, type, data_size);

	total = sizeof(*hdr) + data_size + sizeof(__be16);
	frame = kmalloc(total, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	hdr = (struct shub_msg_hdr *)frame;
	memset(hdr->magic, SHUB_MSG_MAGIC_CHAR, SHUB_MSG_MAGIC_COUNT);
	hdr->sensor = sensor;
	hdr->type = type;
	hdr->length = cpu_to_be16(data_size);
	hdr->checksum = cpu_to_be16(shub_msg_checksum(hdr, sizeof(*hdr) - 2));

	memcpy(frame + sizeof(*hdr), data, data_size);
	put_unaligned_be16(shub_msg_checksum(data, data_size),
			 frame + sizeof(*hdr) + data_size);

	mtu = rpmsg_get_mtu(shub->ept);
	if (mtu <= 0)
		mtu = 384;

	mutex_lock(&shub->tx_lock);
	while (off < total) {
		size_t chunk = min_t(size_t, total - off, mtu);

		ret = rpmsg_send(shub->ept, frame + off, chunk);
		if (ret < 0) {
			dev_err(shub->dev, "failed to send command 0x%02x: %d\n",
				type, ret);
			break;
		}
		off += chunk;
	}
	mutex_unlock(&shub->tx_lock);

	kfree(frame);
	return ret < 0 ? ret : 0;
}

static void shub_handle_sensor_info(struct sprd_shub *shub,
				    struct sprd_shub_ep *ep)
{
	struct shub_sensor_info *info = (void *)ep->rx_data;
	u8 sensor_id = le32_to_cpu(info->handle);

	shub_add_sensor(shub, sensor_id);
}

static void shub_handle_data_generic(struct sprd_shub *shub,
				     struct sprd_shub_ep *ep)
{
	struct shub_data_evt *evt = (void *)ep->rx_data;
	struct sprd_shub_sensor_priv *priv;
	struct sprd_shub_sensor *sensor;
	struct iio_dev *iio_dev;
	unsigned long flags;

	spin_lock_irqsave(&shub->lock, flags);

	sensor = idr_find(&shub->sensors, evt->handle - 100);
	if (!sensor || !sensor->pdev)
		goto out;

	iio_dev = platform_get_drvdata(sensor->pdev);
	if (!iio_dev)
		goto out;

	priv = iio_priv(iio_dev);
	priv->process(iio_dev, evt->data, le64_to_cpu(evt->timestamp));

out:
	spin_unlock_irqrestore(&shub->lock, flags);
}

static void shub_handle_data_samsung(struct sprd_shub *shub,
				     struct sprd_shub_ep *ep, u16 len)
{
	struct sprd_shub_sensor_priv *priv;
	struct sprd_shub_sensor *sensor;
	struct iio_dev *iio_dev;
	unsigned long flags;
	u8 flag;
	u16 handle;
	__le32 vals[3];

	if (len < 28) {
		dev_warn_ratelimited(shub->dev,
				     "short data event (%u bytes), expected at least 28\n",
				     len);
		return;
	}

	flag = ep->rx_data[0];
	if (flag == SHUB_SAMSUNG_FLUSH_FLAG) {
		dev_dbg(shub->dev, "flush event on handle %u\n",
			get_unaligned_le16(&ep->rx_data[2]));
		return;
	}

	if (flag != SHUB_SAMSUNG_DATA_FLAG) {
		dev_dbg(shub->dev, "unknown data event flag 0x%02x\n", flag);
		return;
	}

	handle = get_unaligned_le16(&ep->rx_data[2]);
	memcpy(&vals[0], &ep->rx_data[8], sizeof(vals[0]));
	memcpy(&vals[1], &ep->rx_data[16], sizeof(vals[1]));
	memcpy(&vals[2], &ep->rx_data[24], sizeof(vals[2]));

	spin_lock_irqsave(&shub->lock, flags);

	sensor = idr_find(&shub->sensors, handle);
	if (!sensor || !sensor->pdev)
		goto out;

	iio_dev = platform_get_drvdata(sensor->pdev);
	if (!iio_dev)
		goto out;

	priv = iio_priv(iio_dev);
	priv->process(iio_dev, vals, ktime_get_ns());

out:
	spin_unlock_irqrestore(&shub->lock, flags);
}

static void shub_handle_msg(struct sprd_shub_ep *ep)
{
	struct sprd_shub *shub = ep->shub;
	u16 len = be16_to_cpu(ep->rx_hdr.length);
	u8 type = ep->rx_hdr.type;

	dev_dbg(shub->dev, "event: 0x%02x 0x%02x, data size 0x%04x\n",
		ep->rx_hdr.sensor, type, len);

	if (shub->waiting) {
		if (type == SHUB_EVT_RESPONSE && len >= 2) {
			u8 subtype = ep->rx_data[0];
			u8 status = ep->rx_data[1];

			dev_dbg(shub->dev, "response: subtype 0x%02x status %u\n",
				subtype, status);
			if (shub->wait_subtype == subtype) {
				shub->rsp_status = status;
				complete(&shub->cmd_done);
				return;
			}
		} else if (type == shub->wait_subtype) {
			if (type == SHUB_CMD_GET_FWVERSION && len > 4 &&
			    ep->rx_data[0] == 0x40) {
				shub->fw_version = get_unaligned_le32(&ep->rx_data[1]);
			}
			shub->rsp_status = 0;
			complete(&shub->cmd_done);
			return;
		}
	}

	switch (type) {
	case SHUB_EVT_DATA:
		if (shub->dialect == SHUB_DIALECT_SAMSUNG)
			shub_handle_data_samsung(shub, ep, len);
		else
			shub_handle_data_generic(shub, ep);
		break;

	case SHUB_EVT_CM4_LOG:
		dev_info(shub->dev, "[CM4] %.*s\n", min_t(int, len, sizeof(ep->rx_data)),
			 ep->rx_data);
		break;

	case SHUB_CMD_GET_SENSOR_INFO:
		if (shub->dialect == SHUB_DIALECT_GENERIC)
			shub_handle_sensor_info(shub, ep);
		break;

	case SHUB_CMD_GET_FWVERSION:
		if (shub->dialect == SHUB_DIALECT_GENERIC)
			schedule_delayed_work(&shub->init_work, 2 * HZ);
		break;
	}
}

static size_t shub_rx_buf(u8 **buf, int *cnt, void *to, size_t *pos,
			  size_t expected, size_t avail)
{
	size_t len = min_t(size_t, *cnt, expected - *pos);

	if (*pos + len <= avail)
		memcpy(to + *pos, *buf, len);
	else if (*pos < avail)
		memcpy(to + *pos, *buf, avail - *pos);

	*pos += len;
	*buf += len;
	*cnt -= len;

	return expected - *pos;
}

static int shub_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
			       int cnt, void *userdata, u32 addr)
{
	struct sprd_shub_ep *ep = userdata;
	u8 *buf = data;

	while (cnt > 0) {
		switch (ep->rx_state) {
		case SHUB_MSG_RX_SYNC:
			while (cnt && ep->rx_pos < SHUB_MSG_MAGIC_COUNT) {
				if (*buf == SHUB_MSG_MAGIC_CHAR)
					ep->rx_pos++;
				else
					ep->rx_pos = 0;
				buf++;
				cnt--;
			}

			if (cnt == 0)
				break;

			ep->rx_state = SHUB_MSG_RX_HEADER;
			fallthrough;

		case SHUB_MSG_RX_HEADER:
			if (shub_rx_buf(&buf, &cnt, &ep->rx_hdr, &ep->rx_pos,
					sizeof(struct shub_msg_hdr),
					sizeof(struct shub_msg_hdr)))
				return 0;

			ep->rx_pos = 0;
			ep->rx_state = SHUB_MSG_RX_DATA;
			break;

		case SHUB_MSG_RX_DATA:
			if (shub_rx_buf(&buf, &cnt, ep->rx_data, &ep->rx_pos,
					be16_to_cpu(ep->rx_hdr.length),
					sizeof(ep->rx_data)))
				return 0;

			shub_handle_msg(ep);

			ep->rx_pos = 0;
			ep->rx_state = SHUB_MSG_RX_SYNC;
			break;
		}
	}

	return 0;
}

static const int sprd_shub_frequencies[][2] = {
	{   3, 125000 },
	{   6, 250000 },
	{  12, 500000 },
	{  25, 0 },
	{  50, 0 },
	{ 100, 0 },
	{ 200, 0 },
	{ 500, 0 },
};

static const int sprd_shub_intervals[] = {
	320000,
	160000,
	80000,
	40000,
	20000,
	10000,
	5000,
	2000,
};

static int shub_sensor_postenable(struct iio_dev *iio_dev)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;
	struct shub_batch_cmd batch_cmd = {
		.sample_interval_us = sprd_shub_intervals[priv->freq_idx],
		.batch_interval_us = 0,
	};
	u8 handle = shub_sensor_handle(shub, sensor->id);
	int ret;

	spin_lock_irq(&shub->lock);
	priv->scan_mask = *iio_dev->active_scan_mask;
	spin_unlock_irq(&shub->lock);

	ret = shub_send_msg(shub, handle, SHUB_CMD_SET_BATCH,
			    &batch_cmd, sizeof(batch_cmd));
	if (ret)
		return ret;

	return shub_send_msg(shub, handle, SHUB_CMD_ENABLE, NULL, 0);
}

static int shub_sensor_postdisable(struct iio_dev *iio_dev)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;

	return shub_send_msg(shub, shub_sensor_handle(shub, sensor->id),
			     SHUB_CMD_DISABLE, NULL, 0);
}

const struct iio_buffer_setup_ops sprd_shub_buffer_ops = {
	.postenable = &shub_sensor_postenable,
	.postdisable = &shub_sensor_postdisable
};
EXPORT_SYMBOL_GPL(sprd_shub_buffer_ops);

static const struct iio_mount_matrix shub_samsung_mount_matrix = {
	.rotation = {
		"1", "0", "0",
		"0", "1", "0",
		"0", "0", "1",
	},
};

static const struct iio_mount_matrix shub_default_mount_matrix = {
	.rotation = {
		"-1", "0", "0",
		"0", "-1", "0",
		"0", "0", "1",
	},
};

const struct iio_mount_matrix *
sprd_shub_get_mount_matrix(const struct iio_dev *iio_dev,
			   const struct iio_chan_spec *chan)
{
	struct sprd_shub_sensor_priv *priv = iio_priv((struct iio_dev *)iio_dev);

	if (priv && priv->sensor && priv->sensor->shub &&
	    priv->sensor->shub->dialect == SHUB_DIALECT_SAMSUNG)
		return &shub_samsung_mount_matrix;

	return &shub_default_mount_matrix;
}
EXPORT_SYMBOL_GPL(sprd_shub_get_mount_matrix);

int sprd_shub_calib_read(struct iio_dev *iio_dev, int idx, int *val)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);

	if (idx >= ARRAY_SIZE(priv->calib_data))
		return -EINVAL;

	*val = priv->calib_data[idx];

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_shub_calib_read);

int sprd_shub_calib_write(struct iio_dev *iio_dev, int idx, int val)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;

	if (idx >= ARRAY_SIZE(priv->calib_data))
		return -EINVAL;

	priv->calib_data[idx] = val;

	return shub_send_msg(shub, shub_sensor_handle(shub, sensor->id),
			     SHUB_CMD_SET_CALIB_DATA,
			     priv->calib_data, sizeof(priv->calib_data));
}
EXPORT_SYMBOL_GPL(sprd_shub_calib_write);

int sprd_shub_read_freq(struct iio_dev *iio_dev, int *val, int *val2)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);

	*val = sprd_shub_frequencies[priv->freq_idx][0];
	*val2 = sprd_shub_frequencies[priv->freq_idx][1];

	return IIO_VAL_INT_PLUS_MICRO;
}
EXPORT_SYMBOL_GPL(sprd_shub_read_freq);

int sprd_shub_write_freq(struct iio_dev *iio_dev, int val, int val2)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;
	struct shub_batch_cmd batch_cmd = {
		.batch_interval_us = 0,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(sprd_shub_frequencies); i++) {
		if (sprd_shub_frequencies[i][0] == val &&
		    sprd_shub_frequencies[i][1] == val2) {
			priv->freq_idx = i;
			batch_cmd.sample_interval_us = sprd_shub_intervals[i];
			break;
		}
	}

	if (i == ARRAY_SIZE(sprd_shub_frequencies))
		return -EINVAL;

	return shub_send_msg(shub, shub_sensor_handle(shub, sensor->id),
			     SHUB_CMD_SET_BATCH, &batch_cmd, sizeof(batch_cmd));
}
EXPORT_SYMBOL_GPL(sprd_shub_write_freq);

int sprd_shub_read_avail(struct iio_dev *iio_dev,
			 struct iio_chan_spec const *chan,
			 const int **vals, int *type,
			 int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*length = ARRAY_SIZE(sprd_shub_frequencies) * 2;
		*vals = &sprd_shub_frequencies[0][0];
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_LIST;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(sprd_shub_read_avail);

static s32 shub_float_to_int(u32 float32)
{
	int fraction, shift,
	    scale = SPRD_SHUB_FIXED_SCALE,
	    mantissa = float32 & GENMASK(22, 0),
	    sign = (float32 & BIT(31)) ? -1 : 1,
	    exp = (float32 & ~BIT(31)) >> 23;

	/* special case 0 */
	if (!exp && !mantissa)
		return 0;

	exp -= 127;
	if (exp < 0) {
		exp = -exp;
		/* return values ranging from 1 to scale-1 */
		return sign * ((((BIT(23) + mantissa) * scale) >> 23) >> exp);
	}

	/* return values starting at 1*scale */
	shift = 23 - exp;
	float32 = BIT(exp) + (mantissa >> shift);
	fraction = mantissa & GENMASK(shift - 1, 0);

	return sign * (float32 * scale + ((fraction * scale) >> shift));
}

void sprd_shub_process_xyz(struct iio_dev *iio_dev, void *data, u64 timestamp)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	s32 buf[6] __aligned(8);
	s32 *p = buf;
	__le32 *flt = data;
	unsigned int i;

	memset(buf, 0, sizeof(buf));

	for_each_set_bit(i, &priv->scan_mask, iio_get_masklength(iio_dev)) {
		if (i < 3)
			*p++ = shub_float_to_int(get_unaligned_le32(&flt[i]));
	}

	iio_push_to_buffers_with_timestamp(iio_dev, buf, timestamp);
}
EXPORT_SYMBOL_GPL(sprd_shub_process_xyz);

void sprd_shub_process_single(struct iio_dev *iio_dev, void *data, u64 timestamp)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	s32 buf[4] __aligned(8);

	memset(buf, 0, sizeof(buf));

	if (test_bit(0, &priv->scan_mask))
		buf[0] = shub_float_to_int(get_unaligned_le32(data));

	iio_push_to_buffers_with_timestamp(iio_dev, buf, timestamp);
}
EXPORT_SYMBOL_GPL(sprd_shub_process_single);

static int shub_samsung_load_fw(struct sprd_shub *shub, const char *name,
				const struct firmware **fw_out)
{
	char path[96];

	snprintf(path, sizeof(path), "%s/shub_fw_%s", shub_fw_prefix, name);
	return request_firmware_direct(fw_out, path, shub->dev);
}

static bool shub_samsung_identify(struct sprd_shub *shub, u8 handle,
				  const char *const *candidates,
				  const struct firmware **cali_out,
				  const char **name_out)
{
	int i;

	*cali_out = NULL;
	*name_out = NULL;

	for (i = 0; candidates[i]; i++) {
		const struct firmware *fw = NULL;
		const char *cand = candidates[i];
		char cali_path[64];
		int retry, ret;
		u8 status = 0xff;

		ret = shub_samsung_load_fw(shub, cand, &fw);
		if (ret) {
			dev_warn(shub->dev, "no opcode table for %s: %d\n", cand, ret);
			continue;
		}

		dev_info(shub->dev, "try compatible sensor: %s\n", cand);

		for (retry = 0; retry < 10; retry++) {
			mutex_lock(&shub->cmd_lock);
			reinit_completion(&shub->cmd_done);
			shub->wait_subtype = SHUB_CMD_DOWNLOAD_OPCODE;
			shub->waiting = true;
			shub->rsp_status = 0xff;

			ret = shub_send_msg(shub, handle, SHUB_CMD_DOWNLOAD_OPCODE,
					    fw->data, fw->size);
			if (!ret) {
				if (!wait_for_completion_timeout(&shub->cmd_done,
								 msecs_to_jiffies(300)))
					ret = -ETIMEDOUT;
				else
					status = shub->rsp_status;
			}
			shub->waiting = false;
			mutex_unlock(&shub->cmd_lock);

			if (ret == -ETIMEDOUT) {
				msleep(200);
				continue;
			}
			break;
		}

		if (ret) {
			dev_err(shub->dev, "opcode upload failed: %d\n", ret);
			release_firmware(fw);
			continue;
		}

		if (status) {
			dev_info(shub->dev, "hub rejected the opcode table (status %u)\n",
				 status);
			release_firmware(fw);
			continue;
		}

		snprintf(cali_path, sizeof(cali_path), "%s/shub_fw_%s_cali",
			 shub_fw_prefix, cand);
		ret = request_firmware_direct(cali_out, cali_path, shub->dev);
		if (ret) {
			dev_dbg(shub->dev, "no calibration file %s: %d\n", cali_path, ret);
			*cali_out = NULL;
		}

		*name_out = cand;
		release_firmware(fw);
		dev_info(shub->dev, "init sensor success: %s (handle %u)\n",
			 cand, handle);
		return true;
	}

	dev_err(shub->dev, "no compatible opcode table worked for handle %u\n", handle);
	return false;
}

static void shub_samsung_send_calib(struct sprd_shub *shub,
				    const struct firmware *mag_cali,
				    const struct firmware *light_cali,
				    const struct firmware *accel_cali,
				    const struct firmware *gyro_cali)
{
	size_t mag_sz = mag_cali ? mag_cali->size : 0;
	size_t light_sz = light_cali ? light_cali->size : 0;
	size_t accel_sz = accel_cali ? accel_cali->size : 0;
	size_t gyro_sz = gyro_cali ? gyro_cali->size : 0;
	size_t total = 8 + mag_sz + light_sz + accel_sz + gyro_sz + 4;
	u8 *blob, *p;
	int ret;

	blob = kmalloc(total, GFP_KERNEL);
	if (!blob)
		return;

	blob[0] = (u8)mag_sz;
	blob[1] = 0;
	blob[2] = (u8)light_sz;
	blob[3] = (u8)accel_sz;
	blob[4] = (u8)gyro_sz;
	blob[5] = 0;
	put_unaligned_le16(4, &blob[6]);

	p = blob + 8;
	if (mag_cali) {
		memcpy(p, mag_cali->data, mag_sz);
		p += mag_sz;
	}
	if (light_cali) {
		memcpy(p, light_cali->data, light_sz);
		p += light_sz;
	}
	if (accel_cali) {
		memcpy(p, accel_cali->data, accel_sz);
		p += accel_sz;
	}
	if (gyro_cali) {
		memcpy(p, gyro_cali->data, gyro_sz);
		p += gyro_sz;
	}
	put_unaligned_le32(0, p);

	ret = shub_send_msg(shub, SHUB_SAMSUNG_CALIB_SENSOR,
			    SHUB_CMD_DOWNLOAD_CALIB, blob, total);
	dev_info(shub->dev, "calibration blob (%zu bytes) sent: %d\n", total, ret);
	kfree(blob);
}

static void shub_samsung_setup(struct sprd_shub *shub)
{
	const struct firmware *accel_cali = NULL, *gyro_cali = NULL;
	const struct firmware *mag_cali = NULL, *light_cali = NULL;
	const char *accel_name = NULL, *gyro_name = NULL;
	const char *mag_name = NULL, *light_name = NULL;
	u8 host_status = SHUB_HOST_STATUS_NORMAL;
	int retry, ret;

	if (!shub->eps[0] || !shub->eps[1]) {
		if (shub->wait_tries++ < 20) {
			dev_dbg(shub->dev, "waiting for both sbuf endpoints (try %u)\n",
				shub->wait_tries);
			schedule_delayed_work(&shub->init_work, msecs_to_jiffies(100));
			return;
		}
		if (!shub->eps[0]) {
			dev_err(shub->dev, "no main endpoint, cannot talk to the hub\n");
			return;
		}
		dev_warn(shub->dev,
			 "only the wakeup endpoint bound, continuing with it\n");
	}

	if (!shub->init_delayed) {
		shub->init_delayed = true;
		msleep(200);
	}

	for (retry = 0; retry < 15; retry++) {
		mutex_lock(&shub->cmd_lock);
		reinit_completion(&shub->cmd_done);
		shub->wait_subtype = SHUB_CMD_GET_FWVERSION;
		shub->waiting = true;

		ret = shub_send_msg(shub, 0xff, SHUB_CMD_GET_FWVERSION, NULL, 0);
		if (!ret && !wait_for_completion_timeout(&shub->cmd_done, HZ))
			ret = -ETIMEDOUT;
		shub->waiting = false;
		mutex_unlock(&shub->cmd_lock);

		if (!ret)
			break;
		dev_warn(shub->dev, "Get Version Fail retry %d times (%d)\n",
			 retry + 1, ret);
	}

	if (retry == 15) {
		dev_err(shub->dev,
			"Get Version Fail, cm4 or hub is not up: no frame was answered on shub_main\n");
		return;
	}

	dev_info(shub->dev, "CM4 firmware version: %u\n", shub->fw_version);

	if (!shub_samsung_identify(shub, SHUB_ACCELEROMETER_SENSOR,
				   shub_accel_fw, &accel_cali, &accel_name))
		goto out_release;

	msleep(200);
	shub_samsung_identify(shub, SHUB_GYROSCOPE_SENSOR,
			      shub_gyro_fw, &gyro_cali, &gyro_name);

	msleep(200);
	shub_samsung_identify(shub, SHUB_GEOMAGNETIC_SENSOR,
			      shub_mag_fw, &mag_cali, &mag_name);

	msleep(200);
	shub_samsung_identify(shub, SHUB_LIGHT_SENSOR,
			      shub_light_fw, &light_cali, &light_name);

	shub_samsung_send_calib(shub, mag_cali, light_cali, accel_cali, gyro_cali);

	ret = shub_send_msg(shub, 0xff, SHUB_CMD_SET_HOST_STATUS,
			    &host_status, sizeof(host_status));
	dev_info(shub->dev, "host status NORMAL sent: %d\n", ret);

	if (accel_name)
		shub_add_sensor(shub, SHUB_ACCELEROMETER_SENSOR);
	if (gyro_name)
		shub_add_sensor(shub, SHUB_GYROSCOPE_SENSOR);
	if (mag_name)
		shub_add_sensor(shub, SHUB_GEOMAGNETIC_SENSOR);
	if (light_name)
		shub_add_sensor(shub, SHUB_LIGHT_SENSOR);

	dev_info(shub->dev, "sensors: accel=%s gyro=%s mag=%s light=%s\n",
		 accel_name ?: "none", gyro_name ?: "none",
		 mag_name ?: "none", light_name ?: "none");

out_release:
	release_firmware(accel_cali);
	release_firmware(gyro_cali);
	release_firmware(mag_cali);
	release_firmware(light_cali);
}

static void shub_initialize(struct work_struct *work)
{
	struct sprd_shub *shub = container_of(to_delayed_work(work),
					      struct sprd_shub, init_work);

	if (shub->dialect == SHUB_DIALECT_SAMSUNG) {
		shub_samsung_setup(shub);
		return;
	}

	/*
	 * This command makes the sensor hub send an info event for
	 * each sensor, which we handle by registering the sensor.
	 */
	shub_send_msg(shub, 0xff, SHUB_CMD_GET_SENSOR_INFO, NULL, 0);
}

static struct sprd_shub *shub_core_alloc(struct rpmsg_device *rpdev,
					 enum shub_dialect dialect)
{
	struct sprd_shub *shub;

	shub = kzalloc(sizeof(*shub), GFP_KERNEL);
	if (!shub)
		return NULL;

	shub->dev = &rpdev->dev;
	shub->parent = rpdev->dev.parent;
	shub->dialect = dialect;
	INIT_LIST_HEAD(&shub->list);
	mutex_init(&shub->tx_lock);
	mutex_init(&shub->cmd_lock);
	init_completion(&shub->cmd_done);
	spin_lock_init(&shub->lock);
	idr_init(&shub->sensors);
	INIT_DELAYED_WORK(&shub->init_work, shub_initialize);

	return shub;
}

static void shub_core_free(struct sprd_shub *shub)
{
	struct sprd_shub_sensor *sensor;
	int sensor_id;

	cancel_delayed_work_sync(&shub->init_work);

	idr_for_each_entry(&shub->sensors, sensor, sensor_id) {
		cancel_work_sync(&sensor->reg_work);
		if (sensor->pdev)
			platform_device_unregister(sensor->pdev);
		kfree(sensor);
	}
	idr_destroy(&shub->sensors);
	kfree(shub);
}

static int shub_rpmsg_probe(struct rpmsg_device *rpdev)
{
	const struct shub_dialect_data *match;
	enum shub_dialect dialect;
	struct rpmsg_channel_info chinfo = {
		.src = RPMSG_ADDR_ANY,
		.dst = RPMSG_ADDR_ANY,
	};
	struct shub_timesync_cmd timesync = {
		.systimer_mult = cpu_to_le32(4096000000),
		.systimer_shift = cpu_to_le32(12),
		.sysfrt_mult = cpu_to_le32(4000000000),
		.sysfrt_shift = cpu_to_le32(17),
	};
	struct sprd_shub *shub = NULL, *iter;
	struct sprd_shub_ep *ep;
	bool is_new = false;
	u8 buf_id;
	int ret;

	match = of_device_get_match_data(&rpdev->dev);
	dialect = match ? match->dialect : SHUB_DIALECT_GENERIC;

	mutex_lock(&shub_cores_lock);
	if (dialect == SHUB_DIALECT_SAMSUNG) {
		list_for_each_entry(iter, &shub_cores, list) {
			if (iter->parent == rpdev->dev.parent) {
				shub = iter;
				break;
			}
		}
	}
	if (!shub) {
		shub = shub_core_alloc(rpdev, dialect);
		if (!shub) {
			mutex_unlock(&shub_cores_lock);
			return -ENOMEM;
		}
		if (dialect == SHUB_DIALECT_SAMSUNG)
			list_add(&shub->list, &shub_cores);
		is_new = true;
	}
	shub->refcount++;
	mutex_unlock(&shub_cores_lock);

	ep = devm_kzalloc(&rpdev->dev, sizeof(*ep), GFP_KERNEL);
	if (!ep) {
		ret = -ENOMEM;
		goto err_put_core;
	}
	ep->shub = shub;

	if (dialect == SHUB_DIALECT_SAMSUNG) {
		if (rpdev->dst > 1) {
			dev_err(&rpdev->dev, "unexpected buffer id %u\n", rpdev->dst);
			ret = -EINVAL;
			goto err_put_core;
		}
		buf_id = rpdev->dst;
		if (shub->eps[buf_id]) {
			dev_err(&rpdev->dev, "buffer %u already has an endpoint\n", buf_id);
			ret = -EBUSY;
			goto err_put_core;
		}
		ep->buf_id = buf_id;
		strscpy(chinfo.name, rpdev->id.name, sizeof(chinfo.name));
		ep->ept = rpmsg_create_ept(rpdev, shub_rpmsg_callback, ep, chinfo);
		if (!ep->ept) {
			ret = -ENOMEM;
			goto err_put_core;
		}
		shub->eps[buf_id] = ep;
		if (buf_id == 0)
			shub->ept = ep->ept;
		dev_set_drvdata(&rpdev->dev, ep);
		dev_info(&rpdev->dev, "bound sbuf buffer %u as %s\n", buf_id, rpdev->id.name);

		schedule_delayed_work(&shub->init_work, 0);
		return 0;
	}

	ep->buf_id = 0;
	strscpy(chinfo.name, "shub_main", sizeof(chinfo.name));
	ep->ept = rpmsg_create_ept(rpdev, shub_rpmsg_callback, ep, chinfo);
	if (!ep->ept) {
		ret = -ENOMEM;
		goto err_put_core;
	}
	shub->eps[0] = ep;
	shub->ept = ep->ept;
	dev_set_drvdata(&rpdev->dev, ep);

	ret = shub_send_msg(shub, 0xff, SHUB_CMD_SET_TIMESYNC,
			    &timesync, sizeof(timesync));
	if (ret)
		goto err_destroy_ept;

	ret = shub_send_msg(shub, 0xff, SHUB_CMD_GET_FWVERSION, NULL, 0);
	if (ret)
		goto err_destroy_ept;

	/*
	 * Register virtual sensors.
	 * FIXME: These sensors may depend on multiple physical sensors.
	 * If these are not present on some devices, the corresponding
	 * virtual sensors should not be registered.
	 */
	shub_add_sensor(shub, SHUB_ORIENTATION_SENSOR);

	return 0;

err_destroy_ept:
	rpmsg_destroy_ept(ep->ept);
	shub->ept = NULL;
	shub->eps[0] = NULL;
err_put_core:
	mutex_lock(&shub_cores_lock);
	shub->refcount--;
	if (shub->refcount == 0) {
		if (!list_empty(&shub->list))
			list_del(&shub->list);
		mutex_unlock(&shub_cores_lock);
		shub_core_free(shub);
	} else {
		mutex_unlock(&shub_cores_lock);
	}
	(void)is_new;
	return ret;
}

static void shub_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct sprd_shub_ep *ep = dev_get_drvdata(&rpdev->dev);
	struct sprd_shub *shub;

	if (!ep)
		return;

	shub = ep->shub;
	if (ep->ept)
		rpmsg_destroy_ept(ep->ept);

	mutex_lock(&shub_cores_lock);
	if (ep->buf_id < ARRAY_SIZE(shub->eps) && shub->eps[ep->buf_id] == ep)
		shub->eps[ep->buf_id] = NULL;
	if (ep->buf_id == 0)
		shub->ept = NULL;
	shub->refcount--;
	if (shub->refcount == 0) {
		if (!list_empty(&shub->list))
			list_del(&shub->list);
		mutex_unlock(&shub_cores_lock);
		shub_core_free(shub);
	} else {
		mutex_unlock(&shub_cores_lock);
	}
}

static const struct shub_dialect_data shub_dialect_generic = {
	.dialect = SHUB_DIALECT_GENERIC,
};

static const struct shub_dialect_data shub_dialect_samsung = {
	.dialect = SHUB_DIALECT_SAMSUNG,
};

static const struct of_device_id shub_of_match[] = {
	{ .compatible = "sprd,shub", .data = &shub_dialect_generic },
	{ .compatible = "samsung,sprd-shub", .data = &shub_dialect_samsung },
	{ }
};
MODULE_DEVICE_TABLE(of, shub_of_match);

static struct rpmsg_driver shub_rpmsg_driver = {
	.probe = shub_rpmsg_probe,
	.remove = shub_rpmsg_remove,
	.drv = {
		.name = "sprd-shub",
		.of_match_table = shub_of_match,
	},
};

module_rpmsg_driver(shub_rpmsg_driver);

MODULE_DESCRIPTION("Unisoc sensor hub IIO driver");
MODULE_LICENSE("GPL");
