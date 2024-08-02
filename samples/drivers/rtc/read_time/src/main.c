/*
 * Copyright (c) 2024, Muhammad Waleed Badar <walid.badar@gmail.com>
 * Modified by Gergo Vari
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <stdio.h>

/* Get the device structure for the RTC */
static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_rtc));

static int32_t get_date_time()
{
	int32_t ret = 0;
	struct rtc_time tm;

	ret = rtc_get_time(dev, &tm);
	if (ret < 0) {
		printk("Cannot read date time: %d\n", ret);
		return ret;
	}

	printk("Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);

	return ret;
}

int main(void) {
	int32_t ret = 0;

	/* Check if the RTC is available */
	if (dev == NULL) {
		printk("Error: no device found\n");
		return 0;
	}

	/* Check if the RTC is ready */
	if (!device_is_ready(dev)) {
		printk("Device is not ready\n");
		return 0;
	}

	/* Continuously read the current time from the RTC */
	while (1) {
		ret = get_date_time(dev);
		if (ret < 0) {
			printk("Failed to read from rtc\n");
			return 0;
		}
		k_sleep(K_MSEC(1000));  // Sleep for 1 second
	}
	
	return 0;
}
