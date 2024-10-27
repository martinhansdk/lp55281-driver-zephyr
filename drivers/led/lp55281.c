/*
 * Copyright (c) 2023 Phytec Messtechnik GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_lp55281

/**
 * @file
 * @brief LP55281 LED controller
 *
 * The LP55281 is a 12-channel LED driver that communicates over I2C or SPI (only SPI is supported by this driver).
 */

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/led.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lp55281, CONFIG_LED_LOG_LEVEL);

#define LP55281_NUM_LEDS			12

/** SPI operation word constant, SPI mode 0, CPOL = 0, CPHA = 0 */
#define LP55281_SPI_OPERATION (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8))

// Enables biy positions
#define NSTBY 7
#define EN_BOOST 6
#define EN_AUTOLOAD 5
#define EN_RGB4 3
#define EN_RGB3 2
#define EN_RGB2 1
#define EN_RGB1 0

// IPLS values
#define CURRENT_25PCT  (0)
#define CURRENT_50PCT  (1 << 6)
#define CURRENT_75PCT  (2 << 6)
#define CURRENT_100PCT  (3 << 6)


/* Register addresses */
#define LP55281_ADDR_LED0_PWM  0x00
#define LP55281_ADDR_BOOST     0x0F
#define LP55281_ADDR_PWM_FREQ  0x10
#define LP55281_ADDR_ENABLE    0x11
#define LP55281_ADDR_RESET     0x60

struct lp55281_config {
	struct spi_dt_spec bus;
};

static int lp55281_write_byte(const struct spi_dt_spec *dev, uint8_t address, uint8_t value) {
	uint8_t tx_buffer[2] = {address << 1 | 0x1, value};
	struct spi_buf tx_spi_buf		= {.buf = (void *)&tx_buffer, .len = sizeof(tx_buffer)};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};

	return spi_write_dt(dev, &tx_spi_buf_set);
}

static int lp55281_led_set_brightness(const struct device *dev, uint32_t led,
				     uint8_t brightness)
{
	const struct lp55281_config *config = dev->config;
	uint8_t val;
	int ret;

	if (led >= LP55281_NUM_LEDS || brightness > 100) {
		return -EINVAL;
	}

	/* Map 0-100 % to 0-63 pwm register value */
	val = CURRENT_25PCT | (brightness * 63 / 100);

	ret = lp55281_write_byte(&config->bus, LP55281_ADDR_LED0_PWM + led, val);
	if (ret < 0) {
		LOG_ERR("LED reg update failed");
		return ret;
	}

	return 0;
}

static inline int lp55281_led_on(const struct device *dev, uint32_t led)
{
	/* Set LED brightness to 100 % */
	return lp55281_led_set_brightness(dev, led, 100);
}

static inline int lp55281_led_off(const struct device *dev, uint32_t led)
{
	/* Set LED brightness to 0 % */
	return lp55281_led_set_brightness(dev, led, 0);
}

static int lp55281_enable(const struct device *dev)
{
	const struct lp55281_config *config = dev->config;
	int ret;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	k_sleep(K_MSEC(100));

	ret = lp55281_write_byte(&config->bus, LP55281_ADDR_RESET, 0);
	if (ret < 0) {
		LOG_ERR("Reset LP55281 failed");
		return ret;
	}

	k_sleep(K_MSEC(100));

	ret = lp55281_write_byte(&config->bus, LP55281_ADDR_ENABLE, 
		(1 << NSTBY) | (1 << EN_BOOST) | (1 << EN_AUTOLOAD) |
		(1 << EN_RGB4) | (1 << EN_RGB3) | (1<< EN_RGB2) | (1 << EN_RGB1));
	if (ret < 0) {
		LOG_ERR("Enable LP55281 failed");
		return ret;
	}

	k_sleep(K_MSEC(100));

	return 0;
}

static int lp55281_init(const struct device *dev)
{
	/* If the device is behind a power domain, it will start in
	 * PM_DEVICE_STATE_OFF.
	 */
	if (pm_device_on_power_domain(dev)) {
		pm_device_init_off(dev);
		LOG_INF("Init %s as PM_DEVICE_STATE_OFF", dev->name);
		return 0;
	}

	return lp55281_enable(dev);
}

#ifdef CONFIG_PM_DEVICE
static int lp55281_pm_action(const struct device *dev,
			    enum pm_device_action action)
{
	const struct lp55281_config *config = dev->config;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_RESUME:
		ret = lp55281_enable(dev);
		if (ret < 0) {
			LOG_ERR("Enable LP55281 failed");
			return ret;
		}
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_SUSPEND:
		ret = i2c_reg_update_byte_dt(&config->bus, LP55281_CONFIG,
					   LP55281_CHIP_EN, 0);
		if (ret < 0) {
			LOG_ERR("Disable LP55281 failed");
			return ret;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct led_driver_api lp55281_led_api = {
	.set_brightness = lp55281_led_set_brightness,
	.on = lp55281_led_on,
	.off = lp55281_led_off,
};

#define LP55281_DEFINE(id)						\
	static const struct lp55281_config lp55281_config_##id = {	\
		.bus = SPI_DT_SPEC_INST_GET(id, LP55281_SPI_OPERATION, 0),			\
	};								\
									\
	PM_DEVICE_DT_INST_DEFINE(id, lp55281_pm_action);			\
									\
	DEVICE_DT_INST_DEFINE(id, &lp55281_init,				\
			      PM_DEVICE_DT_INST_GET(id),		\
			      NULL,					\
			      &lp55281_config_##id, POST_KERNEL,		\
			      CONFIG_LED_INIT_PRIORITY,			\
			      &lp55281_led_api);

DT_INST_FOREACH_STATUS_OKAY(LP55281_DEFINE)
