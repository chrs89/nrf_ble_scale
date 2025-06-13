/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_NAU7802_NAU7802_H_
#define ZEPHYR_DRIVERS_SENSOR_NAU7802_NAU7802_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

/*! The possible LDO voltages */
typedef enum _ldovoltages
{
    NAU7802_4V5,
    NAU7802_4V2,
    NAU7802_3V9,
    NAU7802_3V6,
    NAU7802_3V3,
    NAU7802_3V0,
    NAU7802_2V7,
    NAU7802_2V4,
    NAU7802_EXTERNAL,
} NAU7802_LDOVoltage;

/*! The possible gains */
typedef enum _gains
{
    NAU7802_GAIN_1,
    NAU7802_GAIN_2,
    NAU7802_GAIN_4,
    NAU7802_GAIN_8,
    NAU7802_GAIN_16,
    NAU7802_GAIN_32,
    NAU7802_GAIN_64,
    NAU7802_GAIN_128,
} NAU7802_Gain;
/* Array to map index to the enum values*/
static const NAU7802_Gain GainMap[] = {
    NAU7802_GAIN_1,   // Index 0
    NAU7802_GAIN_2,   // Index 1
    NAU7802_GAIN_4,   // Index 2
    NAU7802_GAIN_8,   // Index 3
    NAU7802_GAIN_16,  // Index 4
    NAU7802_GAIN_32,  // Index 5
    NAU7802_GAIN_64,  // Index 6
    NAU7802_GAIN_128, // Index 7
};

/*! The possible sample rates */
typedef enum _sample_rates
{
    NAU7802_RATE_10SPS = 0,
    NAU7802_RATE_20SPS = 1,
    NAU7802_RATE_40SPS = 2,
    NAU7802_RATE_80SPS = 3,
    NAU7802_RATE_320SPS = 7,
} NAU7802_SampleRate;

static const NAU7802_SampleRate sampleRateMap[] = {
    NAU7802_RATE_10SPS,  // Index 0
    NAU7802_RATE_20SPS,  // Index 1
    NAU7802_RATE_40SPS,  // Index 2
    NAU7802_RATE_80SPS,  // Index 3
    NAU7802_RATE_320SPS, // Index 4
};

/*! The possible calibration modes */
typedef enum _calib_mode
{
    NAU7802_CALMOD_INTERNAL = 0,
    NAU7802_CALMOD_OFFSET = 2,
    NAU7802_CALMOD_GAIN = 3,
} NAU7802_Calibration;

typedef enum _thread_state
{
    THREAD_RUNNING = 1,
    THREAD_SUSPENDED = 0,
} thread_state;

/*calibration data */
struct calibDataManuf
{
    int32_t zero_offset;
    float32_t calibration_factor;
    uint32_t magic;
};

/* Define a channel for force reading*/
enum sensor_channel_nuvoton_nau7802
{
    /* Force reading output in Newtons */
    SENSOR_CHAN_FORCE = SENSOR_CHAN_PRIV_START,

    /* Raw ADC reading */
    SENSOR_CHAN_RAW = SENSOR_CHAN_PRIV_START + 1,
};

/* Define a channel for force reading*/
enum sensor_attr_nuvoton_nau7802
{
    /* Force reading in Newton*/
    SENSOR_ATTR_Manufacturing_CALIBRATION_FACTOR = SENSOR_ATTR_PRIV_START,
    SENSOR_ATTR_Manufacturing_ZERO_OFFSET = SENSOR_ATTR_PRIV_START + 1,
    SENSOR_ATTR_TARA = SENSOR_ATTR_PRIV_START + 2,
};

/* Define data (RAM) and configuration (ROM) structures: */
struct nau7802_data
{
    /* per-device values to store in RAM */
    int32_t zero_offset;
    float32_t calibration_factor;
    int32_t sample;
#ifdef CONFIG_NAU7802_TRIGGER
    struct gpio_callback gpio_cb;
    sensor_trigger_handler_t handler_drdy;
    const struct sensor_trigger *trig_drdy;

#if defined(CONFIG_NAU7802_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_NAU7802_THREAD_STACK_SIZE);
    struct k_thread thread;
    struct k_sem gpio_sem;
    thread_state threadState;
#elif defined(CONFIG_NAU7802_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif /* CONFIG_NAU7802_TRIGGER_MODE */
#endif /* CONFIG_NAU7802_TRIGGER */

#if defined(CONFIG_NAU7802_TRIGGER_GLOBAL_THREAD) || \
    defined(CONFIG_NAU7802_TRIGGER_DIRECT)
    const struct device *dev;
#endif
};
struct nau7802_config
{
    /* other configuration to store in ROM */
    const struct i2c_dt_spec bus;
    uint16_t conversions_per_second_idx;
    uint8_t gain_idx;

#ifdef CONFIG_NAU7802_TRIGGER
    const struct gpio_dt_spec drdy_gpios;
#endif /* CONFIG_NAU7802_TRIGGER */
};

/* Define Prototypes for Driver API*/
typedef int (*nau7802_ownThreadSuspend_t)(const struct device *dev);
typedef int (*nau7802_ownThreadResume_t)(const struct device *dev);
typedef int (*nau7802_trigger_set_t)(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler);
typedef int (*nau7802_attr_set_t)(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val);
typedef int (*nau7802_sample_fetch_t)(const struct device *dev, enum sensor_channel chan);
typedef int (*nau7802_channel_get_t)(const struct device *dev, enum sensor_channel chan, struct sensor_value *val);

/* Define API structure*/
__subsystem struct nau7802_driver_api
{
#if CONFIG_NAU7802_TRIGGER
    nau7802_trigger_set_t trigger_set;
#if defined CONFIG_NAU7802_TRIGGER_OWN_THREAD
    nau7802_ownThreadSuspend_t ownThread_suspend;
    nau7802_ownThreadResume_t ownThread_resume;
#endif /*CONFIG_NAU7802_TRIGGER*/
#endif /*CONFIG_NAU7802_TRIGGER_OWN_THREAD*/
    nau7802_attr_set_t attr_set;
    nau7802_sample_fetch_t sample_fetch;
    nau7802_channel_get_t channel_get;
};

/* Define syscall Functions*/

__syscall int nau7802_ownThreadSuspend(const struct device *dev);

static inline int z_impl_nau7802_ownThreadSuspend(const struct device *dev)
{
    const struct nau7802_driver_api *api = (const struct nau7802_driver_api *)dev->api;
    if (api->ownThread_suspend == NULL)
    {
        return -ENOSYS;
    }
    return api->ownThread_suspend(dev);
}

__syscall int nau7802_ownThreadResume(const struct device *dev);

static inline int z_impl_nau7802_ownThreadResume(const struct device *dev)
{
    const struct nau7802_driver_api *api = (const struct nau7802_driver_api *)dev->api;
    if (api->ownThread_resume == NULL)
    {
        return -ENOSYS;
    }
    return api->ownThread_resume(dev);
}

__syscall int nau7802_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler);

static inline int z_impl_nau7802_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
    const struct nau7802_driver_api *api = (const struct nau7802_driver_api *)dev->api;
    if (api->trigger_set == NULL)
    {
        return -ENOSYS;
    }
    return api->trigger_set(dev, trig, handler);
}

__syscall int nau7802_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val);

static inline int z_impl_nau7802_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
    const struct nau7802_driver_api *api = (const struct nau7802_driver_api *)dev->api;
    if (api->attr_set == NULL)
    {
        return -ENOSYS;
    }
    return api->attr_set(dev, chan, attr, val);
}

__syscall int nau7802_sample_fetch(const struct device *dev);

static inline int z_impl_nau7802_sample_fetch(const struct device *dev)
{
    const struct nau7802_driver_api *api = (const struct nau7802_driver_api *)dev->api;
    if (api->sample_fetch == NULL)
    {
        return -ENOSYS;
    }
    return api->sample_fetch(dev, SENSOR_CHAN_ALL);
}

__syscall int nau7802_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val);

static inline int z_impl_nau7802_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    const struct nau7802_driver_api *api = (const struct nau7802_driver_api *)dev->api;
    if (api->channel_get == NULL)
    {
        return -ENOSYS;
    }
    return api->channel_get(dev, chan, val);
}

/*Prototypes*/
#ifdef CONFIG_NAU7802_TRIGGER
int trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                sensor_trigger_handler_t handler);

int nau7802_init_interrupt(const struct device *dev);
#endif

#ifdef CONFIG_NAU7802_TRIGGER_OWN_THREAD
int ownThread_resume(const struct device *dev);
int ownThread_suspend(const struct device *dev);
#endif

#include <zephyr/syscalls/nau7802.h>

#endif