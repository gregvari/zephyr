/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Gergo Vari <work@varigergo.hu>
 */

/* TODO: implement modifiable settings */
/* TODO: implement configurable settings */
/* TODO: implement get_temp */
/* TODO: implement 24h/ampm modes */
/* TODO: handle century bit */
/* TODO: decide if we need to deal with aging offset */
/* TODO: decide if we need to deal with CONV */
/* TODO: implement device power management */

#include <zephyr/drivers/rtc/rtc_ds3231.h>

#include <zephyr/drivers/rtc.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT maxim_ds3231

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ds3231, CONFIG_RTC_LOG_LEVEL);

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#define ALARM_COUNT 2
struct alarm {
	rtc_alarm_callback cb;
	void *user_data;
};

struct ds3231_drv_data {
	struct alarm alarms[ALARM_COUNT];
	struct k_sem lock;
	struct gpio_callback isw_cb_data;
	struct k_work work;
	const struct device *dev;
};

struct ds3231_drv_conf {
	struct i2c_dt_spec i2c_bus;
	struct gpio_dt_spec freq_32k_gpios;
	struct gpio_dt_spec isw_gpios;
};

static int i2c_set_registers(const struct device *dev, uint8_t start_reg, const uint8_t *buf, const size_t buf_size)
{
	struct ds3231_drv_data *data = dev->data;
	const struct ds3231_drv_conf *config = dev->config;

	(void)k_sem_take(&data->lock, K_FOREVER);
	int err = i2c_burst_write_dt(&config->i2c_bus, start_reg, buf, buf_size);
	k_sem_give(&data->lock);

	return err;
}
static int i2c_get_registers(const struct device *dev, uint8_t start_reg, uint8_t *buf, const size_t buf_size)
{
	struct ds3231_drv_data *data = dev->data;
	const struct ds3231_drv_conf *config = dev->config;

	(void)k_sem_take(&data->lock, K_FOREVER);
	int err = i2c_burst_read_dt(&config->i2c_bus, start_reg, buf, buf_size);
	k_sem_give(&data->lock);

	return err;
}
static int i2c_modify_register(const struct device *dev, uint8_t reg, uint8_t *buf, const uint8_t bitmask)
{
	int err;
	if (bitmask != 255) {
		uint8_t og_buf = 0;
		err = i2c_get_registers(dev, reg, &og_buf, 1);
		if (err != 0) {
			return err;
		}
		og_buf &= ~bitmask;
		*buf &= bitmask;
		og_buf |= *buf;
		*buf = og_buf;
	}
	if (err != 0) {
		return err;
	}
	err = i2c_set_registers(dev, reg, buf, 1);
	return err;
}

enum freq {FREQ_1000, FREQ_1024, FREQ_4096, FREQ_8192};
struct ds3231_ctrl {
	bool en_osc;

	bool conv;

	enum freq sqw_freq;

	bool intctrl;
	bool en_alarm_1;
	bool en_alarm_2;
};
static int ds3231_ctrl_to_buf(const struct ds3231_ctrl *ctrl, uint8_t *buf)
{
	if (ctrl->en_alarm_1) {
		*buf |= DS3231_BITS_CTRL_ALARM_1_EN;
	}

	if (ctrl->en_alarm_2) {
		*buf |= DS3231_BITS_CTRL_ALARM_2_EN;
	}

	switch (ctrl->sqw_freq) {
		case FREQ_1000:
			break;
		case FREQ_1024:
			*buf |= DS3231_BITS_CTRL_RS1;
			break;
		case FREQ_4096:
			*buf |= DS3231_BITS_CTRL_RS2;
			break;
		case FREQ_8192:
			*buf |= DS3231_BITS_CTRL_RS1;
			*buf |= DS3231_BITS_CTRL_RS2;
			break;
	}
	if (ctrl->intctrl) {
		*buf |= DS3231_BITS_CTRL_INTCTRL;
	} else { /* enable sqw */
		*buf |= DS3231_BITS_CTRL_BBSQW;
	}

	if (ctrl->conv) {
		*buf |= DS3231_BITS_CTRL_CONV;
	}

	if (!ctrl->en_osc) { /* active low */
		*buf |= DS3231_BITS_CTRL_EOSC;
	}
	return 0;
}
static int ds3231_modify_ctrl(const struct device *dev, const struct ds3231_ctrl *ctrl, const uint8_t bitmask)
{
	uint8_t reg = DS3231_REG_CTRL;
	uint8_t buf = 0;

	int err = ds3231_ctrl_to_buf(ctrl, &buf);
	if (err != 0) {
		return err;
	}

	return i2c_modify_register(dev, reg, &buf, bitmask);
}
static int ds3231_set_ctrl(const struct device *dev, const struct ds3231_ctrl *ctrl)
{
	return ds3231_modify_ctrl(dev, ctrl, 255);
}

struct ds3231_ctrl_sts {
	bool osf;
	bool en_32khz;
	bool bsy;
	bool a1f;
	bool a2f;
};
static int ds3231_ctrl_sts_to_buf(const struct ds3231_ctrl_sts *ctrl, uint8_t *buf)
{
	if (ctrl->a1f) {
		*buf |= DS3231_BITS_CTRL_STS_ALARM_1_FLAG;
	}
	if (ctrl->a2f) {
		*buf |= DS3231_BITS_CTRL_STS_ALARM_2_FLAG;
	}
	if (ctrl->osf) {
		*buf |= DS3231_BITS_CTRL_STS_OSF;
	}
	if (ctrl->en_32khz) {
		*buf |= DS3231_BITS_CTRL_STS_32_EN;
	}
	if (ctrl->bsy) {
		*buf |= DS3231_BITS_CTRL_STS_BSY;
	}
	return 0;
}
static int modify_ctrl_sts(const struct device *dev, const struct ds3231_ctrl_sts *ctrl, const uint8_t bitmask)
{
	const uint8_t reg = DS3231_REG_CTRL_STS;
	uint8_t buf = 0;

	int err = ds3231_ctrl_sts_to_buf(ctrl, &buf);
	if (err != 0) {
		return err;
	}

	return i2c_modify_register(dev, reg, &buf, bitmask);
}
static int get_ctrl_sts(const struct device *dev, uint8_t *buf)
{
	return i2c_get_registers(dev, DS3231_REG_CTRL_STS, buf, 1);
}
static int ds3231_set_ctrl_sts(const struct device *dev, const struct ds3231_ctrl_sts *ctrl)
{
	return modify_ctrl_sts(dev, ctrl, 255);
}

struct ds3231_settings {
	bool osc;
	bool intctrl_or_sqw;
	enum freq freq_sqw;
	bool freq_32khz;
	bool alarm_1;
	bool alarm_2;
};
static int ds3231_set_settings(const struct device *dev, const struct ds3231_settings *conf)
{
	const struct ds3231_ctrl ctrl = {
		conf->osc,
		false,
		conf->freq_sqw,
		conf->intctrl_or_sqw,
		conf->alarm_1,
		conf->alarm_2
	};

	const struct ds3231_ctrl_sts ctrl_sts = {
		false,
		conf->freq_32khz,
		false,
		false,
		false
	};

	int err = ds3231_set_ctrl(dev, &ctrl);
	if (err != 0) {
		LOG_ERR("Couldn't set control register.");
		return -EIO;
	}
	err = ds3231_set_ctrl_sts(dev, &ctrl_sts);
	if (err != 0) {
		LOG_ERR("Couldn't set status register.");
		return -EIO;
	}
	return 0;
}

static int rtc_time_to_alarm_buf(const struct rtc_time *tm, int id, const uint16_t mask, uint8_t *buf)
{
	if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) && (mask & RTC_ALARM_TIME_MASK_MONTHDAY)) {
		LOG_ERR("rtc_time_to_alarm_buf: Mask is invalid (%d)!\n", mask);
		return -EINVAL;
	}
	if (id < 0 || id >= 2) {
		LOG_ERR("rtc_time_to_alarm_buf: Alarm ID is out of range (%d)!\n", id);
		return -EINVAL;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		buf[1] = bin2bcd(tm->tm_min) & DS3231_BITS_TIME_MINUTES;
	} else {
		buf[1] |= DS3231_BITS_ALARM_RATE;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		buf[2] = bin2bcd(tm->tm_hour) & DS3231_BITS_TIME_HOURS;
	} else {
		buf[2] |= DS3231_BITS_ALARM_RATE;
	}

	if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		buf[3] = bin2bcd(tm->tm_wday) & DS3231_BITS_TIME_DAY_OF_WEEK;
		buf[3] |= DS3231_BITS_ALARM_DATE_W_OR_M;
	} else if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		buf[3] = bin2bcd(tm->tm_mday) & DS3231_BITS_TIME_DATE;
	} else {
		buf[3] |= DS3231_BITS_ALARM_RATE;
	}

	switch (id) {
		case 0:
			if (mask & RTC_ALARM_TIME_MASK_SECOND) {
				buf[0] = bin2bcd(tm->tm_sec) & DS3231_BITS_TIME_SECONDS;
			} else {
				buf[0] |= DS3231_BITS_ALARM_RATE;
			}
			break;
		case 1:
			if (mask & RTC_ALARM_TIME_MASK_SECOND) {
				return -EINVAL;
			}

			for (int i = 0; i < 3; i++) {
				buf[i] = buf[i + 1];
			}

			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int modify_alarm_time(const struct device *dev, int id, const struct rtc_time *tm, const uint8_t mask)
{
	uint8_t start_reg;
	size_t buf_size;

	switch (id) {
		case 0:
			start_reg = DS3231_REG_ALARM_1_SECONDS;
			buf_size = 4;
			break;
		case 1:
			start_reg = DS3231_REG_ALARM_2_MINUTES;
			buf_size = 3;
			break;
		default:
			return -EINVAL;
	}

	uint8_t buf[buf_size];
	int err = rtc_time_to_alarm_buf(tm, id, mask, buf);
	if (err != 0) {
		return err;
	}

	return i2c_set_registers(dev, start_reg, buf, buf_size);
}

static int rtc_time_to_buf(const struct rtc_time *tm, uint8_t *buf)
{
	buf[0] = bin2bcd(tm->tm_sec)  & DS3231_BITS_TIME_SECONDS;
	buf[1] = bin2bcd(tm->tm_min)  & DS3231_BITS_TIME_MINUTES;
	buf[2] = bin2bcd(tm->tm_hour) & DS3231_BITS_TIME_HOURS;
	buf[3] = bin2bcd(tm->tm_wday) & DS3231_BITS_TIME_DAY_OF_WEEK;
	buf[4] = bin2bcd(tm->tm_mday) & DS3231_BITS_TIME_DATE;
	buf[5] = bin2bcd(tm->tm_mon)  & DS3231_BITS_TIME_MONTH;

	/* here modulo 100 returns the last two digits of the year,
	   as the DS3231 chip can only store year data for 0-99,
	   hitting that ceiling can be detected with the century bit. */
	/* TODO: figure out a way to store the WHOLE year, not just the last 2 digits */
	buf[6] = bin2bcd((tm->tm_year % 100)) & DS3231_BITS_TIME_YEAR;
	return 0;
}
static int set_time(const struct device *dev, const struct rtc_time *tm)
{
	int buf_size = 7;
	uint8_t buf[buf_size];
	int err = rtc_time_to_buf(tm, buf);
	if (err != 0) {
		return err;
	}

	return i2c_set_registers(dev, DS3231_REG_TIME_SECONDS, buf, buf_size);
}

static int reset_rtc_time(struct rtc_time *tm)
{
	tm->tm_sec = 0;
	tm->tm_min = 0;
	tm->tm_hour= 0;
	tm->tm_wday = 0;
	tm->tm_mday = 0;
	tm->tm_mon = 0;
	tm->tm_year = 0;
	tm->tm_nsec = 0;
	tm->tm_isdst = -1;
	tm->tm_yday = -1;
	return 0;
}
static int buf_to_rtc_time(const uint8_t *buf, struct rtc_time *timeptr)
{
	int err = reset_rtc_time(timeptr);
	if (err != 0) {
		return -EINVAL;
	}

	timeptr->tm_sec = bcd2bin(buf[0] & DS3231_BITS_TIME_SECONDS);
	timeptr->tm_min = bcd2bin(buf[1] & DS3231_BITS_TIME_MINUTES);
	timeptr->tm_hour = bcd2bin(buf[2] & DS3231_BITS_TIME_HOURS);
	timeptr->tm_wday = bcd2bin(buf[3] & DS3231_BITS_TIME_DAY_OF_WEEK);
	timeptr->tm_mday = bcd2bin(buf[4] & DS3231_BITS_TIME_DATE);
	timeptr->tm_mon = bcd2bin(buf[5] & DS3231_BITS_TIME_MONTH);
	timeptr->tm_year = bcd2bin(buf[6] & DS3231_BITS_TIME_YEAR);

	/* FIXME: we will always just set us to 20xx for year */
	timeptr->tm_year = timeptr->tm_year + 100;

	return 0;
}
static int get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const size_t buf_size = 7;
	uint8_t buf[buf_size];
	int err = i2c_get_registers(dev, DS3231_REG_TIME_SECONDS, buf, buf_size);
	if (err != 0) {
		return err;
	}

	return buf_to_rtc_time(buf, timeptr);
}

static int alarm_get_supported_fields(const struct device *dev, uint16_t id, uint16_t *mask)
{
	*mask = RTC_ALARM_TIME_MASK_MONTHDAY
		| RTC_ALARM_TIME_MASK_WEEKDAY
		| RTC_ALARM_TIME_MASK_HOUR
		| RTC_ALARM_TIME_MASK_MINUTE;

	switch (id) {
		case 0:
			*mask |= RTC_ALARM_TIME_MASK_SECOND;
			break;
		case 1:
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int modify_alarm_state(const struct device *dev, uint16_t id, bool state)
{
	struct ds3231_ctrl ctrl;
	uint8_t ctrl_mask = 0;

	switch (id) {
		case 0:
			ctrl.en_alarm_1 = state;
			ctrl_mask |= DS3231_BITS_CTRL_ALARM_1_EN;
			break;
		case 1:
			ctrl.en_alarm_2 = state;
			ctrl_mask |= DS3231_BITS_CTRL_ALARM_2_EN;
			break;
		default:
			return -EINVAL;
	}

	/* TODO: move to modifiable settings when implemented */
	return ds3231_modify_ctrl(dev, &ctrl, ctrl_mask);
}
static int alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask, const struct rtc_time *timeptr)
{
	if (mask == 0) {
		return modify_alarm_state(dev, id, false);
	}

	int err = modify_alarm_state(dev, id, true);
	if (err != 0) {
		return err;
	}

	return modify_alarm_time(dev, id, timeptr, mask);
}

static int alarm_buf_to_rtc_time(uint8_t *buf, int id, struct rtc_time *tm, uint16_t *mask)
{
	int err = reset_rtc_time(tm);
	if (err != 0) {
		return -EINVAL;
	}

	if (id < 0 || id > 1) {
		return -EINVAL;
	} else if (id == 1) {
		/* shift to the right to match original func */
		for (int i = 3; i > 0; i--) {
			buf[i] = buf[i - 1];
		}
		buf[0] = 0;
	}

	*mask = 0;
	if (!(buf[1] & DS3231_BITS_ALARM_RATE)) {
		tm->tm_min = bcd2bin(buf[1] & DS3231_BITS_TIME_MINUTES);
		*mask |= RTC_ALARM_TIME_MASK_MINUTE;
	}
	if (!(buf[2] & DS3231_BITS_ALARM_RATE)) {
		tm->tm_hour = bcd2bin(buf[2] & DS3231_BITS_TIME_HOURS);
		*mask |= RTC_ALARM_TIME_MASK_HOUR;
	}
	if (!(buf[3] & DS3231_BITS_ALARM_RATE)) {
		if (buf[3] & DS3231_BITS_ALARM_DATE_W_OR_M) {
			tm->tm_wday = bcd2bin(buf[3] & DS3231_BITS_TIME_DAY_OF_WEEK);
			*mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
		} else {
			tm->tm_mday = bcd2bin(buf[3] & DS3231_BITS_TIME_DATE);
			*mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
		}
	}
	if (!(buf[0] & DS3231_BITS_ALARM_RATE)) {
		tm->tm_sec = bcd2bin(buf[0] & DS3231_BITS_TIME_SECONDS);
		*mask |= RTC_ALARM_TIME_MASK_SECOND;
	}

	if ((*mask & RTC_ALARM_TIME_MASK_WEEKDAY) && (*mask & RTC_ALARM_TIME_MASK_MONTHDAY)) {
		return -EINVAL;
	}

	return 0;
}
static int alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask, struct rtc_time *timeptr)
{
	uint8_t start_reg;
	size_t buf_size;
	/* TODO: remove code duplication */
	switch (id) {
		case 0:
			start_reg = DS3231_REG_ALARM_1_SECONDS;
			buf_size = 4;
			break;
		case 1:
			start_reg = DS3231_REG_ALARM_2_MINUTES;
			buf_size = 3;
			break;
		default:
			return -EINVAL;
	}

	uint8_t buf[4];
	int err = i2c_get_registers(dev, start_reg, buf, buf_size);
	if (err != 0) {
		return err;
	}

	return alarm_buf_to_rtc_time(buf, id, timeptr, mask);
}

static int alarm_is_pending(const struct device *dev, uint16_t id)
{
	uint8_t buf;
	int err = get_ctrl_sts(dev, &buf);
	if (err != 0) {
		return err;
	}

	uint8_t mask = 0;
	switch (id) {
		case 0:
			mask |= DS3231_BITS_CTRL_STS_ALARM_1_FLAG;
			break;
		case 1:
			mask |= DS3231_BITS_CTRL_STS_ALARM_2_FLAG;
			break;
		default:
			return -EINVAL;
	}

	bool state = buf & mask;
	if (state) {
		const struct ds3231_ctrl_sts ctrl = {
			.a1f = false,
			.a2f = false
		};
		err = modify_ctrl_sts(dev, &ctrl, mask);
		if (err != 0) {
			return err;
		}
	}
	return state;
}

static int get_alarm_states(const struct device *dev, bool *states)
{
	int err = 0;
	for (int i = 0; i < ALARM_COUNT; i++) {
		states[i] = alarm_is_pending(dev, i);
		if (!(states[i] == 0 || states[i] == 1)) {
			states[i] = -EINVAL;
			err = -EINVAL;
		}
	}
	return err;
}

static int alarm_set_callback(const struct device *dev, uint16_t id, rtc_alarm_callback cb, void *user_data)
{
	if (id < 0 || id >= ALARM_COUNT) {
		return -EINVAL;
	}

	struct ds3231_drv_data *data = dev->data;
	data->alarms[id] = (struct alarm){
		cb,
		user_data
	};

	return 0;
}
static void k_alarm_cbs_h(struct k_work *work)
{
	struct ds3231_drv_data *data = CONTAINER_OF(work, struct ds3231_drv_data, work);
	const struct device *dev = data->dev;
	
	/* FIXME: states stay false all the time! */
	bool states[2];
	get_alarm_states(dev, states);

	for (int i = 0; i < ALARM_COUNT; i++) {
		if (states[i]) {
			if (data->alarms[i].cb) {
				data->alarms[i].cb(dev, i, data->alarms[i].user_data);
			}
		}
	}
}

static void isw_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ds3231_drv_data *data = CONTAINER_OF(cb, struct ds3231_drv_data, isw_cb_data);

	k_work_submit(&data->work);
}

static const struct rtc_driver_api ds3231_driver_api = {
	.set_time = set_time,
	.get_time = get_time,

#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = alarm_get_supported_fields,
	.alarm_set_time = alarm_set_time,
	.alarm_get_time = alarm_get_time,
	.alarm_is_pending = alarm_is_pending,
	.alarm_set_callback = alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
	/*.update_set_callback = update_set_callback,*/
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION
	/*.set_calibration = set_calibration,
	.get_calibration = get_calibration,*/
#endif /* CONFIG_RTC_CALIBRATION */
};

static int ds3231_init(const struct device *dev)
{
	struct ds3231_drv_data *data = dev->data;
	k_sem_init(&data->lock, 1, 1);
	data->dev = dev;
	data->alarms[0] = (struct alarm){NULL, NULL};
	data->alarms[1] = (struct alarm){NULL, NULL};

	const struct ds3231_drv_conf *config = dev->config;
	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready.");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&config->isw_gpios)) {
		LOG_ERR("ISW GPIO pin is not ready.");
		return -ENODEV;
	}

	struct ds3231_settings conf = {
		.osc = true,
		.intctrl_or_sqw = false,
		.freq_sqw = FREQ_1000,
		.freq_32khz = false,
		/* TODO: don't turn off alarms, only modify settings and leave these alone */
		.alarm_1 = false,
		.alarm_2 = false
	};
	int err = ds3231_set_settings(dev, &conf);
	if (err != 0) {
		LOG_ERR("Failed to init settings.");
		return err;
	}
	
	k_work_init(&data->work, k_alarm_cbs_h);

	err = gpio_pin_configure_dt(&(config->isw_gpios), GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("Couldn't configure ISW GPIO pin.");
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&(config->isw_gpios), GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		LOG_ERR("Couldn't configure ISW interrupt.");
		return err;
	}

	gpio_init_callback(&data->isw_cb_data, isw_isr, BIT((config->isw_gpios).pin));
	err = gpio_add_callback((config->isw_gpios).port, &data->isw_cb_data);
	if (err != 0) {
		LOG_ERR("Couldn't add ISW interrupt callback.");
		return err;
	}
	return 0;
}

/* TODO: decide which pins are needed, ifdef them... */
#define DS3231_DEFINE(inst)                                                                        \
        static struct ds3231_drv_data ds3231_drv_data_##inst;                                              \
        static const struct ds3231_drv_conf ds3231_drv_conf_##inst = {                             \
                .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                             \
                .isw_gpios = GPIO_DT_SPEC_INST_GET(inst, isw_gpios),                               \
                .freq_32k_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, freq_32khz_gpios, {})             \
        };                                                                                         \
        DEVICE_DT_INST_DEFINE(inst, &ds3231_init, NULL, &ds3231_drv_data_##inst,                   \
                              &ds3231_drv_conf_##inst, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,      \
                              &ds3231_driver_api);


DT_INST_FOREACH_STATUS_OKAY(DS3231_DEFINE)
