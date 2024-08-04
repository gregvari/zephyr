/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Copyright (c) 2023 Arunmani Alagarsamy <arunmani27100@gmail.com>
 * Copyright (c) 2024 Gergo Vari <work@varigergo.hu>
 */

/* TODO: implement control status */
/* TODO: implement abstracted settings */
/* TODO: implement configurable settings */
/* TODO: implement get_temp */
/* TODO: implement 24h/ampm modes */
/* TODO: handle century bit */
/* TODO: decide if we need to deal with aging offset */
/* TODO: decide if we need to deal with CONV */
/* TODO: implement device power management */

#include <zephyr/drivers/rtc.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT maxim_ds3231

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ds3231, CONFIG_RTC_LOG_LEVEL);

#include <zephyr/drivers/i2c.h>
/*           *
 * REGISTERS *
 *           */

/* Time registers */
#define DS3231_REG_TIME_SECONDS     0x00
#define DS3231_REG_TIME_MINUTES     0x01
#define DS3231_REG_TIME_HOURS       0x02
#define DS3231_REG_TIME_DAY_OF_WEEK 0x03
#define DS3231_REG_TIME_DATE        0x04
#define DS3231_REG_TIME_MONTH       0x05
#define DS3231_REG_TIME_YEAR        0x06

/* Alarm 1 registers */
#define DS3231_REG_ALARM_1_SECONDS 0x07
#define DS3231_REG_ALARM_1_MINUTES 0x08
#define DS3231_REG_ALARM_1_HOURS   0x09
#define DS3231_REG_ALARM_1_DATE    0x0A 

/* Alarm 2 registers */
/* Alarm 2 has no seconds to set, it only has minute accuracy. */
#define DS3231_REG_ALARM_2_MINUTES 0x0B
#define DS3231_REG_ALARM_2_HOURS   0x0C
#define DS3231_REG_ALARM_2_DATE    0x0D

/* Control registers */
#define DS3231_REG_CTRL         0x0E
#define DS3231_REG_CTRL_STATUS  0x0F

/* Aging offset register */
#define DS3231_REG_AGING_OFFSET 0x10

/* Temperature registers */
#define DS3231_REG_TEMP_MSB 0x11
#define DS3231_REG_TEMP_LSB 0x12



/*           *
 * BITMASKS  *
 *           */

/* Time bitmasks */
#define DS3231_BITS_TIME_SECONDS     GENMASK(6, 0) 
#define DS3231_BITS_TIME_MINUTES     GENMASK(6, 0)
#define DS3231_BITS_TIME_HOURS       GENMASK(5, 0) 
#define DS3231_BITS_TIME_AMPM        BIT(5)
#define DS3231_BITS_TIME_12HR        BIT(6)
#define DS3231_BITS_TIME_DAY_OF_WEEK GENMASK(2, 0)
#define DS3231_BITS_TIME_DATE        GENMASK(5, 0) 
#define DS3231_BITS_TIME_MONTH       GENMASK(4, 0) 
#define DS3231_BITS_TIME_CENTURY     BIT(7)
#define DS3231_BITS_TIME_YEAR        GENMASK(7, 0)

/* Alarm bitmasks */
/* All alarm bitmasks match with time other than date and the alarm rate bit. */
#define DS3231_BITS_ALARM_RATE        BIT(7)
#define DS3231_BITS_ALARM_DATE_W_OR_M BIT(6)

#define DS3231_BITS_SIGN BIT(7)
/* Control bitmasks */
#define DS3231_BITS_CTRL_EOSC BIT(7) /* enable oscillator, active low */
#define DS3231_BITS_CTRL_BBSQW  BIT(6) /* enable battery-backed square-wave */

/*  Setting the CONV bit to 1 forces the temperature sensor to 
 *  convert the temperature into digital code and 
 *  execute the TCXO algorithm to update 
 *  the capacitance array to the oscillator. This can only 
 *  happen when a conversion is not already in progress. 
 *  The user should check the status bit BSY before 
 *  forcing the controller to start a new TCXO execution. 
 *  A user-initiated temperature conversion 
 *  does not affect the internal 64-second update cycle. */
#define DS3231_BITS_CTRL_CONV BIT(6) 

/* Rate selectors */
/* TODO: the datasheet is probably wrong and it's not 1Hz but 1kHz, should check though */
/* RS2 | RS1 | SQW FREQ
 *  0  |  0  | 1Hz
 *  0  |  1  | 1.024kHz
 *  1  |  0  | 4.096kHz
 *  1  |  1  | 8.192kHz */
#define DS3231_BITS_CTRL_RS2 BIT(4) 
#define DS3231_BITS_CTRL_RS1 BIT(3) 

#define DS3231_BITS_CTRL_INTCTRL    BIT(2) 
#define DS3231_BITS_CTRL_ALARM_2_EN BIT(1) 
#define DS3231_BITS_CTRL_ALARM_1_EN BIT(0) 

/* Control status bitmasks */
/* For some reason you can access OSF in both control and control status registers. */
#define DS3231_BITS_CTRL_STS_OSF          BIT(7) /* oscillator stop flag */
#define DS3231_BITS_CTRL_STS_32_EN        BIT(3) /* 32kHz square-wave */
#define DS3231_BITS_CTRL_STS_BSY          BIT(2) /* set when TXCO is busy, see CONV flag */
#define DS3231_BITS_CTRL_STS_ALARM_2_FLAG BIT(1)
#define DS3231_BITS_CTRL_STS_ALARM_1_FLAG BIT(0)

/* Aging offset bitmask */
#define DS3231_BITS_DATA BIT(6, 0)

/* Temperature bitmasks */
#define DS3231_BITS_TEMP_MSB GENMASK(6, 0) /* integer portion */
#define DS3231_BITS_TEMP_LSB GENMASK(7, 6) /* fractional portion */

struct ds3231_drv_conf {
	struct i2c_dt_spec i2c_bus;
};

struct ds3231_data {
	struct k_spinlock lock;
};

static int i2c_set_registers(const struct device *dev, uint8_t start_reg, const uint8_t *buf, const size_t buf_size) {
	int err;
	struct ds3231_data *data = dev->data;
	const struct ds3231_drv_conf *config = dev->config;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	err = i2c_burst_write_dt(&config->i2c_bus, start_reg, buf, buf_size);

	k_spin_unlock(&data->lock, key);
	return err;
}

static int i2c_get_registers(const struct device *dev, uint8_t start_reg, uint8_t *buf, const size_t buf_size) {
	int err;
	struct ds3231_data *data = dev->data;
	const struct ds3231_drv_conf *config = dev->config;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	err = i2c_burst_read_dt(&config->i2c_bus, start_reg, buf, buf_size);

	k_spin_unlock(&data->lock, key);
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
static int ds3231_ctrl_to_buf(const struct ds3231_ctrl *ctrl, uint8_t *buf) {
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
static int ds3231_set_ctrl(const struct device *dev, const struct ds3231_ctrl *ctrl) {
	uint8_t buf;
	int err = ds3231_ctrl_to_buf(ctrl, &buf);
	if (err != 0) {
		return err;
	}
	err = i2c_set_registers(dev, DS3231_REG_CTRL, &buf, 1);
	printf("set_ctrl: %d\n", buf);
	return err;
}

static int ds3231_set_time(const struct device *dev, const struct rtc_time *tm)
{
	LOG_DBG("set time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
		"min = %d, sec = %d",
		tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_wday, tm->tm_hour, tm->tm_min,
		tm->tm_sec);

	const size_t buf_size = 7;
	uint8_t buf[buf_size];
	buf[0] = bin2bcd(tm->tm_sec)  & DS3231_BITS_TIME_SECONDS;
	buf[1] = bin2bcd(tm->tm_min)  & DS3231_BITS_TIME_MINUTES;
	buf[2] = bin2bcd(tm->tm_hour) & DS3231_BITS_TIME_HOURS;
	buf[3] = bin2bcd(tm->tm_wday) & DS3231_BITS_TIME_DAY_OF_WEEK;
	buf[4] = bin2bcd(tm->tm_mday) & DS3231_BITS_TIME_DATE;
	buf[5] = bin2bcd(tm->tm_mon)  & DS3231_BITS_TIME_MONTH;
	
	/* here modulo 100 returns the last two digits of the year,
	 * as the DS3231 chip can only store year data for 0-99,
	 * hitting that ceiling can be detected with the century bit. */
	/* TODO: figure out a way to store the WHOLE year, not just the last 2 digits */
	buf[6] = bin2bcd((tm->tm_year % 100)) & DS3231_BITS_TIME_YEAR; 

	return i2c_set_registers(dev, DS3231_REG_TIME_SECONDS, buf, buf_size);
}

static int ds3231_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const size_t buf_size = 7;
	uint8_t buf[buf_size];
	int err = i2c_get_registers(dev, DS3231_REG_TIME_SECONDS, buf, buf_size);
	if (err != 0) {
		return err;
	}

	timeptr->tm_sec = bcd2bin(buf[0] & DS3231_BITS_TIME_SECONDS);
	timeptr->tm_min = bcd2bin(buf[1] & DS3231_BITS_TIME_MINUTES);
	timeptr->tm_hour = bcd2bin(buf[2] & DS3231_BITS_TIME_HOURS);
	timeptr->tm_wday = bcd2bin(buf[3] & DS3231_BITS_TIME_DAY_OF_WEEK);
	timeptr->tm_mday = bcd2bin(buf[4] & DS3231_BITS_TIME_DATE);
	timeptr->tm_mon = bcd2bin(buf[5] & DS3231_BITS_TIME_MONTH);
	timeptr->tm_year = bcd2bin(buf[6] & DS3231_BITS_TIME_YEAR);
	timeptr->tm_year = timeptr->tm_year + 100; /* FIXME: we will always just set us to 20xx for year */
	
	timeptr->tm_nsec = 0;
	timeptr->tm_isdst = -1;
	timeptr->tm_yday = -1;

	LOG_DBG("get time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
		"min = %d, sec = %d",
		timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	return 0;
}

static const struct rtc_driver_api ds3231_driver_api = {
	.set_time = ds3231_set_time,
	.get_time = ds3231_get_time,

#ifdef CONFIG_RTC_ALARM
	/*.alarm_get_supported_fields = ds3231_alarm_get_supported_fields,
	.alarm_set_time = ds3231_alarm_set_time,
	.alarm_get_time = ds3231_alarm_get_time,
	.alarm_is_pending = ds3231_alarm_is_pending,
	.alarm_set_callback = ds3231_alarm_set_callback,*/
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
	/*.update_set_callback = ds3231_update_set_callback,*/
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION
	/*.set_calibration = ds3231_set_calibration,
	.get_calibration = ds3231_get_calibration,*/
#endif /* CONFIG_RTC_CALIBRATION */
};

static int ds3231_init(const struct device *dev)
{
	int err;
	const struct ds3231_drv_conf *config = dev->config;
	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready.");
		return -ENODEV;
	}
	
	const struct ds3231_ctrl ctrl = {
		true, /* enable oscillator */
		false, /* disable conv */
		FREQ_1000,
		true, /* enable alarm interrupts, disable square wave */
		false, /* disable alarm 1 */
		false /* disable alarm 1 */
	};
	err = ds3231_set_ctrl(dev, &ctrl);

	return err;
}

#define DS3231_DEFINE(inst)                                                                        \
	static struct ds3231_data ds3231_data_##inst;                                              \
	static const struct ds3231_drv_conf ds3231_drv_conf_##inst = {                                 \
		.i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &ds3231_init, NULL, &ds3231_data_##inst,                       \
			      &ds3231_drv_conf_##inst, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,        \
			      &ds3231_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DS3231_DEFINE)
