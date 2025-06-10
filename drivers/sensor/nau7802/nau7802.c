/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include "nau7802.h"
#include "nau7802_regmap.h"

#define DT_DRV_COMPAT nuvoton_nau7802

/* Register the module to logging submodule*/
LOG_MODULE_REGISTER(NAU7802, LOG_LEVEL_DBG);
// LOG_MODULE_REGISTER(NAU7802, CONFIG_I2C_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(nuvoton_nau7802) == 0
#warning "Custom NAU7802 driver enabled without any devices"
#endif

/**************************************************************************/
/*!
    @brief Perform a soft reset
    @return 0 if success
*/
/**************************************************************************/
static int nau7802_reset(struct nau7802_config *config)
{
    int ret;
    /* Set the RR bit to 1 in R0x00, to guarantee a reset of all register values.*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_RR,
                                 (1 << NAU7802_SHIFT_PU_CTRL_RR));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip reset failed", ret);
        return ret;
    }
    /* Set the RR bit to 0 and PUD bit 1, in R0x00, to enter normal operation*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_RR,
                                 (0 << NAU7802_SHIFT_PU_CTRL_RR));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip reset RR bit failed", ret);
        return ret;
    }
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUD,
                                 (1 << NAU7802_SHIFT_PU_CTRL_PUD));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set PUD bit failed", ret);
        return ret;
    }
    /*!
    After about 200 microseconds, the PWRUP bit will be Logic=1 indicating the
    device is ready for the remaining programming setup.
    */
    k_sleep(K_USEC(300));
    uint8_t pu_ctrl_val;
    ret = i2c_reg_read_byte_dt(&config->bus, NAU7802_PU_CTRL, &pu_ctrl_val);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Read PUR bit failed", ret);
        return ret;
    }

    if ((pu_ctrl_val & NAU7802_MASK_PU_CTRL_PUR) == 0)
    {
        LOG_ERR("Chip is not powered up, PUR bit is 0.");
        return EIO;
    }

    return 0;
}

/**************************************************************************/
/*!
    @brief  Whether to have the sensor enabled and working or in power down mode
    @param  flag True to be in powered mode, False for power down mode
    @return 0 if success
*/
/**************************************************************************/
static int nau7802_enable(struct nau7802_config *config, bool flag)
{
    int ret;
    /* Turn off the IC*/
    if (!flag)
    {
        /* Reset the PUA bit*/
        ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL,
                                     NAU7802_MASK_PU_CTRL_PUA,
                                     (0 << NAU7802_SHIFT_PU_CTRL_PUA));
        if (ret != 0)
        {
            LOG_ERR("ret:%d, Chip reset PUA bit failed", ret);
            return ret;
        }

        /* Reset the PUD bit*/
        ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL,
                                     NAU7802_MASK_PU_CTRL_PUD,
                                     (0 << NAU7802_SHIFT_PU_CTRL_PUD));
        if (ret != 0)
        {
            LOG_ERR("ret:%d, Chip reset PUD bit failed", ret);
            return ret;
        }
        /* success*/
        return 0;
    }

    /* Turn on the IC*/
    /* Set the PUA bit*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUA,
                                 (1 << NAU7802_SHIFT_PU_CTRL_PUA));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set PUA bit failed", ret);
        return ret;
    }

    /* Set the PUD bit*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUD,
                                 (1 << NAU7802_SHIFT_PU_CTRL_PUD));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set PUD bit failed", ret);
        return ret;
    }

    /*!
        RDY: Analog part wakeup stable plus Data Ready after exiting power-down
        mode 600ms
    */
    k_sleep(K_MSEC(600));

    uint8_t pu_ctrl_val;
    ret = i2c_reg_read_byte_dt(&config->bus, NAU7802_PU_CTRL, &pu_ctrl_val);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Read PUR bit failed", ret);
        return ret;
    }

    if ((pu_ctrl_val & NAU7802_MASK_PU_CTRL_PUR) == 0)
    {
        LOG_ERR("Chip is not powered up, PUR bit is 0.");
        return EIO;
    }

    /* success*/
    return 0;
}

/**************************************************************************/
/*!
    @brief  The desired LDO voltage setter
    @param voltage The LDO setting: NAU7802_4V5, NAU7802_4V2, NAU7802_3V9,
    NAU7802_3V6, NAU7802_3V3, NAU7802_3V0, NAU7802_2V7, NAU7802_2V4, or
    NAU7802_EXTERNAL if we are not using the internal LDO
    @return 0 if success
*/
/**************************************************************************/
static int nau7802_setLDO(struct nau7802_config *config, NAU7802_LDOVoltage voltage)
{
    int ret;

    if (voltage == NAU7802_EXTERNAL)
    {
        /* Reset the AVDD bit in PU_CTRL register*/
        ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL,
                                     NAU7802_MASK_PU_CTRL_AVDDS,
                                     (0 << NAU7802_SHIFT_PU_CTRL_AVDDS));
        if (ret != 0)
        {
            LOG_ERR("ret:%d, Chip reset AVDDS bit failed", ret);
            return ret;
        }

        /* success*/
        return 0;
    }

    /* Set the AVDD bit in PU_CTRL register*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_AVDDS,
                                 (1 << NAU7802_SHIFT_PU_CTRL_AVDDS));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set AVDDS bit failed", ret);
        return ret;
    }

    /* Write the LDO voltage to CTRL1 register*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_CTRL1, NAU7802_MASK_CTRL1_VLDO,
                                 (voltage << NAU7802_SHIFT_CTRL1_VLDO));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set VLDO bits failed", ret);
        return ret;
    }
    /* success*/
    return 0;
}

/**************************************************************************/
/*!
    @brief  The desired ADC gain setter
    @param  gain_idx Index of the desired gain in dts bindings:
    NAU7802_GAIN_1=0,
    NAU7802_GAIN_2=1,
    NAU7802_GAIN_4=2,
    NAU7802_GAIN_8=3,
    NAU7802_GAIN_16=4,
    NAU7802_GAIN_32=5,
    NAU7802_GAIN_64=6,
    NAU7802_GAIN_128=7
    @returns 0 if seccess
*/
/**************************************************************************/
static int nau7802_setGain(struct nau7802_config *config)
{
    int ret;
    NAU7802_Gain gain = GainMap[config->gain_idx];
    /* Write the PGA gain to CTRL1 register*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_CTRL1, NAU7802_MASK_CTRL1_GAINS,
                                 (gain << NAU7802_SHIFT_CTRL1_GAINS));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set GAINS bits failed", ret);
        return ret;
    }
    /* success*/
    return 0;
}

/**************************************************************************/
/*!
    @brief  The desired conversion rate setter
    @param conversions_per_second_idx The index of desired rate in dts bindings:
    NAU7802_RATE_10SPS=0,
    NAU7802_RATE_20SPS=1,
    NAU7802_RATE_40SPS=2,
    NAU7802_RATE_80SPS=3,
    NAU7802_RATE_320SPS=4
    @returns 0 if seccess
*/
/**************************************************************************/
static int nau7802_setRate(const struct device *dev, const struct sensor_value *sps)
{
    int ret;
    struct nau7802_config *config = dev->config;
    struct nau7802_data *data = dev->data;
    uint16_t conversions_per_second_idx = (uint16_t)sps->val1;

    // /*check if Thread is running: set_rate prohibited if thread running*/
    // if (data->threadState == THREAD_RUNNING)
    // {
    //     LOG_ERR("Sensor THREAD_RUNNING: Change of SPS prohibited");
    //     return ENOTSUP;
    // }

    /*Check if Value is within allowed SPS rate*/
    if (!(conversions_per_second_idx == NAU7802_RATE_10SPS ||
          conversions_per_second_idx == NAU7802_RATE_20SPS ||
          conversions_per_second_idx == NAU7802_RATE_40SPS ||
          conversions_per_second_idx == NAU7802_RATE_80SPS ||
          conversions_per_second_idx == NAU7802_RATE_320SPS))
    {
        return -EINVAL;
    }

    NAU7802_SampleRate rate = conversions_per_second_idx;
    /* Write the sample rate to CTRL2 register*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_CTRL2, NAU7802_MASK_CTRL2_CRS,
                                 (rate << NAU7802_SHIFT_CTRL2_CRS));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Chip set CRS bits failed", ret);
        return ret;
    }
    /*Set Value to config*/
    config->conversions_per_second_idx = conversions_per_second_idx;
    /* success*/
    return 0;
}

/**************************************************************************/
/*!
    @brief  The ADC offset error setter
    @param offset Should be a float value to add on the calibrated sensor
    reading. It's expressed as int32.
    Use memcpy to extract the data from offset->val1
    @returns 0 if seccess
*/
/**************************************************************************/
// static int nau7802_setOffset(const struct device *nau7802, const struct sensor_value *offset)
static int nau7802_setOffset(const struct device *nau7802, const struct sensor_value *offset)
{
    if (nau7802 == NULL || nau7802->data == NULL)
    {
        LOG_ERR("Device or device data is NULL");
        return -ENOTSUP;
    }

    if (offset == NULL)
    {
        LOG_ERR("Offset value couldn't be NULL");
        return -EINVAL;
    }

    struct nau7802_data *data = nau7802->data;

    if (offset == NULL)
    {
        LOG_ERR("Offset value couldn't be NULL");
        return -ENOTSUP;
    }

    /* Reconstruct the input value to float*/
    memcpy(&data->zero_offset, &offset->val1, sizeof(int32_t));

    /* success*/
    return 0;
}

/**************************************************************************/
/*!
    @brief  The calibration factor setter
    @param calibrationFactor Should be a float value but express as int32.
    Use memcpy to extract the data from calibrationFactor->val1
    @returns 0 if seccess
*/
/**************************************************************************/
// static int nau7802_setCalibration(const struct device *nau7802, const struct sensor_value
// *calibrationFactor)
static int nau7802_setCalibrationFactor(const struct device *nau7802,
                                        const struct sensor_value *calibrationFactor)
{
    struct nau7802_data *data = nau7802->data;

    if (calibrationFactor == NULL)
    {
        LOG_ERR("Offset value couldn't be NULL");
        return -ENOTSUP;
    }

    /* Reconstruct the input value to float*/
    float32_t calibFactor = (float32_t)sensor_value_to_float(calibrationFactor);
    memcpy(&data->calibration_factor, &calibFactor, sizeof(float32_t));

    /* success*/
    return 0;
}

/**************************************************************************/
/*!
    @brief  Conduct internal sensor calibration
    @param calibrationMode
    @returns 0 if seccess
*/
/**************************************************************************/
static int nau7802_IntCalibration(struct nau7802_config *config,
                                  NAU7802_Calibration calibrationMode)
{
    int ret;

    if (calibrationMode == NULL)
    {
        LOG_ERR("Calibration mode couldn't be NULL");
        return -ENOTSUP;
    }

    /* Write the calib mode to CTRL2 register*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_CTRL2, NAU7802_MASK_CTRL2_CALMOD,
                                 (calibrationMode << NAU7802_SHIFT_CTRL2_CALMOD));
    if (ret != 0)
    {
        LOG_ERR("ret%d, Write calib mode failed.", ret);
        return ret;
    }

    /* Set the CALS bit in CTRL2 register to start the calibration*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_CTRL2, NAU7802_MASK_CTRL2_CALS,
                                 (1 << NAU7802_SHIFT_CTRL2_CALS));
    if (ret != 0)
    {
        LOG_ERR("ret%d, Failed to start the calibration.", ret);
        return ret;
    }

    /* Poll the CALS bit until it became 0*/
    uint8_t ctrl2_val;
    ret = i2c_reg_read_byte_dt(&config->bus, NAU7802_CTRL2, &ctrl2_val);
    while ((ctrl2_val & NAU7802_MASK_CTRL2_CALS != 0) && (ret == 0))
    {
        k_sleep(K_MSEC(10));
        ret = i2c_reg_read_byte_dt(&config->bus, NAU7802_CTRL2, &ctrl2_val);
    }
    if (ret != 0)
    {
        LOG_ERR("ret%d, Failed to poll the CALS bit.", ret);
        return ret;
    }
    LOG_DBG("Internal Calibration done");

    /* Check the CAL_ERR bit in CTRL2 to see if the calibration is successful*/
    if (ctrl2_val & NAU7802_MASK_CTRL2_CAL_ERR != 0)
    {
        LOG_ERR("Calibration failed.");
        return -EIO;
    }

    /* success*/
    LOG_DBG("Internal calibration success");
    return 0;
}

/* Sensor API function implementation*/
static int attr_set(const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr, const struct sensor_value *val)
{
    int ret;

    switch (attr)
    {
    case SENSOR_ATTR_OFFSET:
        return nau7802_setOffset(dev, val);

    case SENSOR_ATTR_Manufacturing_CALIBRATION_FACTOR:
        return nau7802_setCalibrationFactor(dev, val);

    case SENSOR_ATTR_SAMPLING_FREQUENCY:
        return nau7802_setRate(dev, val);

    default:
        LOG_WRN("attr_set() does not support this attribute.");
        return -ENOTSUP;
    }

    return ret;
}

static int sample_fetch(const struct device *dev, enum sensor_channel chan)
{

    struct nau7802_data *data = dev->data;
    struct nau7802_config *config = dev->config;
    uint8_t out[3];

    if (chan == SENSOR_CHAN_ALL)
    {
        if (i2c_burst_read_dt(&config->bus, NAU7802_ADCO_B2, out, 3) < 0)
        {
            LOG_DBG("Failed to read sample");
            return -EIO;
        }
    }
    else
    {
        LOG_ERR("Invalid channel");
        return -ENOTSUP;
    }

    /* Reconstruct the signed 24-bit output data*/
    data->sample = (int32_t)((uint32_t)(out[2]) | ((uint32_t)(out[1]) << 8) |
                             ((uint32_t)(out[0]) << 16));

    /* Check the sign bit of the 24-bit value and extend it */
    if (data->sample & 0x00800000)
    {                               // If the 24th bit is set
        data->sample |= 0xFF000000; // Set the upper 8 bits to 1s
    }
    else
    {
        data->sample &= 0x00FFFFFF; // Ensure the upper 8 bits are 0s
    }

    return 0;
}

static int channel_get(const struct device *dev, enum sensor_channel chan,
                       struct sensor_value *val)
{
    struct nau7802_data *data = dev->data;
    float uval;

    if ((enum sensor_channel_nuvoton_nau7802)chan != SENSOR_CHAN_FORCE && (enum sensor_channel_nuvoton_nau7802)chan != SENSOR_CHAN_RAW)
    {
        return -ENOTSUP;
    }

    switch (chan)
    {
    case SENSOR_CHAN_FORCE:
        /* convert the ADC value to force value */
        uval = ((float32_t)(data->sample) - data->zero_offset) * (1 / data->calibration_factor);
        sensor_value_from_float(val, uval);
        break;
    case SENSOR_CHAN_RAW:
        uval = (float32_t)(data->sample) * 1.0 + 0; // Operands indicate that on purpose Cal_factor and offset is not affecting value
        sensor_value_from_float(val, uval);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

/* Define API structure --> Deprecated Implement Custom Sensor API See below*/
/*
static const struct sensor_driver_api nau7802_api = {
#if CONFIG_NAU7802_TRIGGER
    .trigger_set = nau7802_trigger_set,
#endif
    .attr_set = nau7802_attr_set,
    .sample_fetch = nau7802_sample_fetch,
    .channel_get = nau7802_channel_get,
};
*/

/* Define Custom Sensor API structure*/
static const struct nau7802_driver_api nau7802_api = {
    .attr_set = &attr_set,
    .channel_get = &channel_get,
    .sample_fetch = &sample_fetch,
#if CONFIG_NAU7802_TRIGGER
    .trigger_set = &trigger_set,

#if defined CONFIG_NAU7802_TRIGGER_OWN_THREAD
    .ownThread_suspend = &ownThread_suspend,
    .ownThread_resume = &ownThread_resume,
#endif /*CONFIG_NAU7802_TRIGGER*/
#endif /*CONFIG_NAU7802_TRIGGER_OWN_THREAD*/
};

/* Init function*/
static int nau7802_init(const struct device *dev)
{
    struct nau7802_config *config = dev->config;
    struct nau7802_data *data = dev->data;
    int ret;

    /* Check if the i2c bus is ready*/
    if (!device_is_ready(config->bus.bus))
    {
        LOG_ERR("ret:%d, I2C dev %s not ready", ret, config->bus.bus->name);
        return -ENODEV;
    }

    /* Reset the IC*/
    ret = nau7802_reset(config);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Reset process failed", ret);
        return ret;
    }
    LOG_DBG("ret:%d, finish reset", ret);

    /* Power on the IC*/
    ret = nau7802_enable(config, true);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Enable process failed", ret);
        return ret;
    }
    LOG_DBG("ret:%d, Enable success", ret);

    /* Check and assert the IC revision ID*/
    // Do I need to do this?

    /* Set the LDO*/
    ret = nau7802_setLDO(config, NAU7802_3V0);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, SetLDO process failed", ret);
        return ret;
    }
    LOG_DBG("ret:%d, Set LDO done", ret);

    /* Configure the PGA gain*/
    ret = nau7802_setGain(config);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, SetGain process failed", ret);
        return ret;
    }
    LOG_DBG("ret:%d, Set gain done", ret);

    /* Configure the output data rate*/
    struct sensor_value sps;
    sps.val1 = sampleRateMap[config->conversions_per_second_idx];
    ret = nau7802_setRate(dev, &sps);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, SetRate process failed", ret);
        return ret;
    }
    LOG_DBG("ret:%d, Set rate done", ret);

    /* Disable ADC chopper clock*/
    /* Set the bit 4 and bit 5 of ADC register*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_ADC, NAU7802_MASK_ADC_REG_CHPS,
                                 (0b11 << NAU7802_SHIFT_ADC_REG_CHPS));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Disabling chopper clock failed", ret);
        return ret;
    }
    /* Use low ESR caps*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_PGA, NAU7802_MASK_PGA_LDOMODE,
                                 (0 << NAU7802_SHIFT_PGA_LDOMODE));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Setting low ESR failed", ret);
        return ret;
    }

    /* PGA stabilizer cap on output*/
    ret = i2c_reg_update_byte_dt(&config->bus, NAU7802_POWER, NAU7802_MASK_POWER_PGA_CAP_EN,
                                 (1 << NAU7802_SHIFT_POWER_PGA_CAP_EN));
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Enabling PGA cap failed", ret);
        return ret;
    }

    /* initialize the offset value and calibration factor */
    /* This setting output the raw setting*/
    data->zero_offset = 0;
    data->calibration_factor = 1.0;

    /* Conduct internal calibration*/
    ret = nau7802_IntCalibration(config, NAU7802_CALMOD_OFFSET);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Internal Calibration failed", ret);
        return ret;
    }

    /* Initialize NVS & Load Calibration Factos*/
    ret = nau7802_nvs_init();
    if (ret != 0)
    {
        LOG_ERR("ret:%d, NVS init process fail", ret);
        return ret;
    }

    /*Set Calibration Data from NVS --> nau7802 device*/
    ret = load_calibration_data_nvs(data);
    if (ret != 0)
    {
        LOG_ERR("Failed to load calibration data.");
    }
    else
    {
        LOG_INF("Loaded Calibration Data: Zero_Offset = %d, Calibration_Factor = %f", data->zero_offset, data->calibration_factor);
    }
    LOG_INF("Loaded Calibration Data: Zero_Offset = %d, Calibration_Factor = %f", data->zero_offset, data->calibration_factor);

    // /* Load Calib. Data*/
    // struct nau7802_data test_data;
    // ret = load_calibration_data_nvs(&test_data);
    // if (ret != 0)
    // {
    //     LOG_ERR("Failed to load calibration data.");
    // }
    // else
    // {
    //     LOG_INF("Loaded Calibration Data: Zero_Offset = %d, Calibration_Factor = %f", test_data.zero_offset, test_data.calibration_factor);
    // }
    // LOG_INF("Loaded Calibration Data: Zero_Offset = %d, Calibration_Factor = %f", test_data.zero_offset, test_data.calibration_factor);

#ifdef CONFIG_NAU7802_TRIGGER
    ret = nau7802_init_interrupt(dev);
    if (ret != 0)
    {
        LOG_ERR("ret:%d, Interrupt init process fail", ret);
        return ret;
    }

#endif
    /*!
        RDY: Provide Time for stabalizing of Analog Part before host powers down
    */
    k_sleep(K_MSEC(600));

    /* success*/
    LOG_DBG("Chip init done.");

    // data->threadState = THREAD_SUSPENDED;

    return 0;
}

static int custom_nau7802_pm_action(const struct device *dev,
                                    enum pm_device_action action)
{
    struct nau7802_config *config = dev->config;
    int ret = 0;

    switch (action)
    {
    case PM_DEVICE_ACTION_RESUME:
        LOG_INF("Resuming NAU7802 sensor");
        /* Re-initialize the chip */
        ret = nau7802_enable(config, true);
        if (ret != 0)
        {
            LOG_DBG("PM_DEVICE_ACTION_Resuming Failed %d", ret);
        }
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_INF("Suspending NAU7802 sensor");
        /* Put the chip into sleep mode */
        ret = nau7802_enable(config, false);

        if (ret != 0)
        {
            LOG_DBG("PM_DEVICE_ACTION_SUSPEND: Failed %d", ret);
        }
        break;
    default:
        return -ENOTSUP;
    }

    return ret;
}

/* Macro function to selectively include the drdy gpio pin */
#if defined(CONFIG_NAU7802_TRIGGER)
#define NAU7802_INT_CFG(inst) .drdy_gpios = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),
#else
#define NAU7802_INT_CFG(inst)
#endif

/* Use the Instance-based APIs */
#define CREATE_NAU7802_INST(inst)                                                     \
    static struct nau7802_data nau7802_data_##inst;                                   \
    static struct nau7802_config nau7802_config_##inst = {                            \
        NAU7802_INT_CFG(inst)                                                         \
            .bus = I2C_DT_SPEC_INST_GET(inst),                                        \
        .conversions_per_second_idx = DT_INST_ENUM_IDX(inst, conversions_per_second), \
        .gain_idx = DT_INST_ENUM_IDX(inst, gain)};                                    \
    /* Power Management device initialization */                                      \
    PM_DEVICE_DT_INST_DEFINE(inst, custom_nau7802_pm_action);                         \
    /* Sensor device instantiation */                                                 \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                                \
                                 nau7802_init,                                        \
                                 PM_DEVICE_DT_INST_GET(inst),                         \
                                 &nau7802_data_##inst,                                \
                                 &nau7802_config_##inst,                              \
                                 POST_KERNEL,                                         \
                                 CONFIG_SENSOR_INIT_PRIORITY,                         \
                                 &nau7802_api);

DT_INST_FOREACH_STATUS_OKAY(CREATE_NAU7802_INST)
