/* ds3231.c - Driver for Maxim DS3231 temperature sensor */

/*
 * Copyright (c) 2024 Gergo Vari
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <math.h>

#include "ds3231.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(DS3231, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "DS3231 driver enabled without any devices"
#endif

struct drv_data {
	struct k_sem lock;
	const struct device *dev;
	int32_t temp_int;
	int32_t temp_frac;
};

struct drv_conf {
	struct i2c_dt_spec i2c_bus;
};

static int i2c_get_registers(const struct device *dev, uint8_t start_reg, uint8_t *buf, const size_t buf_size)
{
	struct drv_data *data = dev->data;
	const struct drv_conf *config = dev->config;
	
	/* FIXME: bad start_reg/buf_size values break i2c for that run */

	(void)k_sem_take(&data->lock, K_FOREVER);
	int err = i2c_burst_read_dt(&config->i2c_bus, start_reg, buf, buf_size);
	k_sem_give(&data->lock);

	return err;
}

/* Fetch and Get (will be deprecated) */

int ds3231_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct drv_data *data = dev->data;
	int err;
	
	const size_t buf_size = 2;
	uint8_t buf[buf_size];

	err = i2c_get_registers(dev, DS3231_REG_TEMP_MSB, buf, buf_size);
	if (err != 0) {
		return err;
	}
	
	int16_t itemp = ( buf[0] << 8 | (buf[1] & 0xC0) );
	double temp = ( (double)itemp / 256.0 );
	
	data->temp_int = (int)temp;
	data->temp_frac = (int)((temp - data->temp_int) * 100) / pow(10, -6);

	return 0;
}

static int ds3231_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct drv_data *data = dev->data;

	switch (chan) {
		case SENSOR_CHAN_AMBIENT_TEMP:
			val->val1 = data->temp_int;
			val->val2 = data->temp_frac;
			break;
		default:
			return -ENOTSUP;
	}

	return 0;
}

/* Read and Decode */
/*
void ds3231_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider to increase "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, ds3231_submit_sync);
}



static int ds3231_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					  uint16_t *frame_count)
{
	const struct ds3231_encoded_data *edata = (const struct ds3231_encoded_data *)buffer;
	int32_t ret = -ENOTSUP;

	if (chan_spec.chan_idx != 0) {
		return ret;
	}

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		*frame_count = edata->has_temp ? 1 : 0;
		break;
	case SENSOR_CHAN_PRESS:
		*frame_count = edata->has_press ? 1 : 0;
		break;
	case SENSOR_CHAN_HUMIDITY:
		*frame_count = edata->has_humidity ? 1 : 0;
		break;
	default:
		return ret;
	}

	if (*frame_count > 0) {
		ret = 0;
	}

	return ret;
}

static int ds3231_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_HUMIDITY:
	case SENSOR_CHAN_PRESS:
		*base_size = sizeof(struct sensor_q31_sample_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int ds3231_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				 uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct ds3231_encoded_data *edata = (const struct ds3231_encoded_data *)buffer;

	if (*fit != 0) {
		return 0;
	}

	struct sensor_q31_data *out = data_out;

	out->header.base_timestamp_ns = edata->header.timestamp;
	out->header.reading_count = 1;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		if (edata->has_temp) {
			int32_t readq = edata->reading.comp_temp * pow(2, 31 - BME280_TEMP_SHIFT);
			int32_t convq = BME280_TEMP_CONV * pow(2, 31 - BME280_TEMP_SHIFT);

			out->readings[0].temperature =
				(int32_t)((((int64_t)readq) << (31 - BME280_TEMP_SHIFT)) /
					  ((int64_t)convq));
			out->shift = BME280_TEMP_SHIFT;
		} else {
			return -ENODATA;
		}
		break;
	case SENSOR_CHAN_PRESS:
		if (edata->has_press) {
			int32_t readq = edata->reading.comp_press;
			int32_t convq = BME280_PRESS_CONV_KPA * pow(2, 31 - BME280_PRESS_SHIFT);

			out->readings[0].pressure =
				(int32_t)((((int64_t)readq) << (31 - BME280_PRESS_SHIFT)) /
					  ((int64_t)convq));
			out->shift = BME280_PRESS_SHIFT;
		} else {
			return -ENODATA;
		}
		break;
	case SENSOR_CHAN_HUMIDITY:
		if (edata->has_humidity) {
			out->readings[0].humidity = edata->reading.comp_humidity;
			out->shift = BME280_HUM_SHIFT;
		} else {
			return -ENODATA;
		}
		break;
	default:
		return -EINVAL;
	}

	*fit = 1;

	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = ds3231_decoder_get_frame_count,
	.get_size_info = ds3231_decoder_get_size_info,
	.decode = ds3231_decoder_decode,
};

int ds3231_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}*/

static int init_i2c(const struct drv_conf *config, struct drv_data *data) {
	k_sem_init(&data->lock, 1, 1);
	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready.");
		return -ENODEV;
	}
	return 0;
}

static int init(const struct device *dev)
{
	int err = 0;
		
	const struct drv_conf *config = dev->config;
	struct drv_data *data = dev->data;

	err = init_i2c(config, data);
	if (err != 0) {
		LOG_ERR("Failed to init I2C.");
		return err;
	}

	return 0;
}


static DEVICE_API(sensor, driver_api) = {
	.sample_fetch = ds3231_sample_fetch,
	.channel_get = ds3231_channel_get,
	/*
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = ds3231_submit,
	.get_decoder = ds3231_get_decoder,
#endif
*/
};

#define DS3231_DEFINE(inst)						\
	static struct drv_data drv_data_##inst;                                              \
	static const struct drv_conf drv_conf_##inst = {                             \
		.i2c_bus = I2C_DT_SPEC_INST_GET(inst)             \
	};											\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			&init,				\
			NULL,			\
			&drv_data_##inst,				\
			&drv_conf_##inst,				\
			POST_KERNEL,					\
			CONFIG_SENSOR_INIT_PRIORITY,			\
			&driver_api);

DT_INST_FOREACH_STATUS_OKAY(DS3231_DEFINE)
