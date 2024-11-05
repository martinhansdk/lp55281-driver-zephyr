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

#define LP55281_MAX_LEDS			4
#define LP55281_COLORS_PER_LED      3

/** SPI operation word constant, SPI mode 0, CPOL = 0, CPHA = 0 */
#define LP55281_SPI_OPERATION (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8))

// Enables bit positions
#define NSTBY 7
#define EN_BOOST 6
#define EN_AUTOLOAD 5


/* Register addresses */
#define LP55281_ADDR_LED0_PWM  0x00
#define LP55281_ADDR_BOOST     0x0F
#define LP55281_ADDR_PWM_FREQ  0x10
#define LP55281_ADDR_ENABLE    0x11
#define LP55281_ADDR_RESET     0x60

struct lp55281_config {
    struct spi_dt_spec bus;
    uint8_t num_leds;
    int boost_frequency;
    int boost_voltage;
    int pwm_frequency;
    bool autoload;
    const struct led_info *leds_info;
    const uint8_t ipls[LP55281_MAX_LEDS][3];
};

static int lp55281_led_set_color(const struct device *dev, uint32_t led, uint8_t num_colors, const uint8_t *color);

static int lp55281_write_reg(const struct spi_dt_spec *dev, uint8_t address, uint8_t value) {
    uint8_t tx_buffer[2] = {address << 1 | 0x1, value};
    struct spi_buf tx_spi_buf		= {.buf = (void *)&tx_buffer, .len = sizeof(tx_buffer)};
    struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};

    return spi_write_dt(dev, &tx_spi_buf_set);
}

static const struct led_info *lp55281_led_to_info(const struct lp55281_config *config, uint32_t led)
{
    if (led < config->num_leds) {
        return &config->leds_info[led];
    }

    return NULL;
}

static int lp55281_get_info(const struct device *dev, uint32_t led, const struct led_info **info)
{
    const struct lp55281_config *config = dev->config;
    const struct led_info *led_info = lp55281_led_to_info(config, led);

    if (!led_info) {
        return -EINVAL;
    }

    *info = led_info;

    return 0;
}

static int lp55281_led_set_brightness(const struct device *dev, uint32_t led,
                     uint8_t brightness)
{
    uint8_t color[3] = {brightness, brightness, brightness};
    return lp55281_led_set_color(dev, led, 3, color);
}

static int lp55281_led_set_color(const struct device *dev, uint32_t led, uint8_t num_colors, const uint8_t *color) {
    const struct lp55281_config *config = dev->config;
	const struct led_info *led_info = lp55281_led_to_info(config, led);

	if (!led_info) {
		return -ENODEV;
	}

	if (num_colors != led_info->num_colors) {
		LOG_ERR("%s: invalid number of colors: got=%d, expected=%d",
			dev->name,
			num_colors,
			led_info->num_colors);
		return -EINVAL;
	}

    int retval = 0;
    uint8_t register_address = LP55281_ADDR_LED0_PWM + LP55281_COLORS_PER_LED*led_info->index;
    for(int i = 0 ; i < LP55281_COLORS_PER_LED ; i++) {
        uint8_t pwm = (((uint16_t)color[i]) * 63) / 100;
        uint8_t register_value = (config->ipls[led][i] << 6) | pwm;

        retval = lp55281_write_reg(&config->bus, register_address, register_value);
        if(retval != 0) {
            break;
        }
        register_address++;
    }

	return retval;
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

    ret = lp55281_write_reg(&config->bus, LP55281_ADDR_RESET, 0);
    if (ret < 0) {
        LOG_ERR("Reset LP55281 failed");
        return ret;
    }

    k_sleep(K_MSEC(100));

    // check IPLS settings
    for(int led = 0; led < config->num_leds ; led++) {
        for(int i = 0 ; i < 4 ; i++) {
            if(config->ipls[led][i] > 3) {
                LOG_ERR("LED current setting out of range %d is not valid. Valid range: 0-3", config->ipls[led][i]);
                return -ENOTSUP;
            }
        }
    }

    uint8_t frq_sel = 0;
    switch(config->boost_frequency) {
        case 0:
        case 1000000:
            frq_sel = 1;
            break;
        case 1670000:
            frq_sel = 2;
            break;
        case 2000000:
            frq_sel = 4;
            break;
        default:
            LOG_ERR("Unsupported boost frequency: %d", config->boost_frequency);
            return ENOTSUP;
    }

    uint8_t boost_output = 0;
    switch(config->boost_voltage) {
        case 4000:
            boost_output = 0x00;
            break;
        case 4250:
            boost_output = 0x01;
            break;
        case 4400:
            boost_output = 0x03;
            break;
        case 4550:
            boost_output = 0x07;
            break;
        case 4700:
            boost_output = 0x0F;
            break;
        case 4850:
            boost_output = 0x1F;
            break;
        case 5000:
            boost_output = 0x3F;
            break;
        case 5150:
            boost_output = 0x7F;
            break;
        case 5300:
            boost_output = 0xFF;
            break;
        default:
            LOG_ERR("Unsupported boost voltage: %d", config->boost_voltage);
            return ENOTSUP;
    }

    uint8_t fpwm = 0;
    switch(config->pwm_frequency) {
        case 10000:
            fpwm = 0;
            break;
        case 20000:
            fpwm = 1;
            break;
        case 40000:
            fpwm = 2;
            break;
        default:
            LOG_ERR("Unsupported PWM frequency: %d", config->pwm_frequency);
            return -ENOTSUP;
    }

    ret = lp55281_write_reg(&config->bus, LP55281_ADDR_BOOST, boost_output);
    if (ret < 0) {
        LOG_ERR("Setting boost voltage failed");
        return ret;
    }

    ret = lp55281_write_reg(&config->bus, LP55281_ADDR_PWM_FREQ, (fpwm << 4) | frq_sel);
    if (ret < 0) {
        LOG_ERR("Setting boost voltage failed");
        return ret;
    }

    uint8_t led_en = 0;
    for(int led = 0; led < config->num_leds ; led++) {
        const struct led_info *info = lp55281_led_to_info(config, led);
        if(info->index >= LP55281_MAX_LEDS) {
            LOG_ERR("LED index %d does not exist. Valid range: 0-3", info->index);
            return -ENOTSUP;
        } else {
            led_en |= (1 << info->index);
        }
    }

    ret = lp55281_write_reg(&config->bus, LP55281_ADDR_ENABLE, 
        (1 << NSTBY) |
        ((config->boost_frequency != 0) << EN_BOOST) |
        (config->autoload << EN_AUTOLOAD) |
        led_en);

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
    .set_color = lp55281_led_set_color,
    .on = lp55281_led_on,
    .off = lp55281_led_off,
    .get_info = lp55281_get_info,
};

#define COLOR_MAPPING(led_node_id)                      \
    const uint8_t color_mapping_##led_node_id[] =       \
        DT_PROP(led_node_id, color_mapping);

#define LED_CURRENT(led_node_id) DT_PROP(led_node_id, led_current),

#define LED_INFO(led_node_id)                                   \
    {                                                           \
        .label		= DT_PROP(led_node_id, label),              \
        .index		= DT_PROP(led_node_id, index),              \
        .num_colors	= DT_PROP_LEN(led_node_id, color_mapping),  \
        .color_mapping	= color_mapping_##led_node_id,          \
    },

#define LP55281_DEFINE(id)                                          \
	DT_INST_FOREACH_CHILD(id, COLOR_MAPPING)                        \
                                                                    \
	static const struct led_info lp55281_leds_##id[] = {            \
		DT_INST_FOREACH_CHILD(id, LED_INFO)                         \
	};                                                              \
                                                                    \
    static const struct lp55281_config lp55281_config_##id = {	    \
        .bus = SPI_DT_SPEC_INST_GET(id, LP55281_SPI_OPERATION, 0),  \
        .num_leds = ARRAY_SIZE(lp55281_leds_##id),                  \
        .leds_info = lp55281_leds_##id,                             \
        .boost_frequency = DT_INST_PROP(id, boost_frequency),       \
        .boost_voltage = DT_INST_PROP(id, boost_voltage),           \
        .pwm_frequency = DT_INST_PROP(id, pwm_frequency),           \
        .autoload = DT_INST_PROP(id, autoload),                     \
        .ipls = { DT_INST_FOREACH_CHILD(id, LED_CURRENT) },  \
    };                                                              \
                                                                    \
    PM_DEVICE_DT_INST_DEFINE(id, lp55281_pm_action);                \
                                                                    \
    DEVICE_DT_INST_DEFINE(id, &lp55281_init,                        \
                  PM_DEVICE_DT_INST_GET(id),                        \
                  NULL,                                             \
                  &lp55281_config_##id, POST_KERNEL,                \
                  CONFIG_LED_INIT_PRIORITY,                         \
                  &lp55281_led_api);

DT_INST_FOREACH_STATUS_OKAY(LP55281_DEFINE)
