#include "calibration.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "sensor/nau7802/nau7802.h" // Adjust path if different
                                                  // If your NVS store function is elsewhere

LOG_MODULE_REGISTER(CALIBRATION, LOG_LEVEL_DBG);

extern const struct device *const nau7802; // May not be needed if passed as argument

/**
 * Fetch 100 samples, discard first 50, average remaining for offset.
 */
int get_offset_data(const struct device *dev)
{
    int err;
    struct sensor_value force_val;
    int32_t sum = 0;
    struct nau7802_data *data = dev->data;

    for (int i = 0; i < 100; i++)
    {
        err = nau7802_sample_fetch(dev);
        if (err < 0)
        {
            LOG_ERR("Fetch sample failed (%d)", err);
            return -ENODATA;
        }

        err = nau7802_channel_get(dev, SENSOR_CHAN_RAW, &force_val);
        if (err < 0)
        {
            LOG_ERR("Get channel failed (%d)", err);
            return -ENODATA;
        }

        if (i >= 50)
        {
            sum += force_val.val1;
        }

        k_sleep(K_MSEC(10));
    }

    data->zero_offset = sum / 50;
    LOG_INF("Offset calculated: %d", data->zero_offset);

    return 0;
}

/**
 * Calculate calibration factor using known weight.
 */
int get_calFactor_data(const struct device *dev, float32_t cal_weight)
{
    int err;
    struct sensor_value force_val;
    float32_t sum = 0.0;
    struct nau7802_data *data = dev->data;

    for (int i = 0; i < 100; i++)
    {
        err = nau7802_sample_fetch(dev);
        if (err < 0)
        {
            LOG_ERR("Fetch sample failed (%d)", err);
            return -ENODATA;
        }

        err = nau7802_channel_get(dev, SENSOR_CHAN_RAW, &force_val);
        if (err < 0)
        {
            LOG_ERR("Get channel failed (%d)", err);
            return -ENODATA;
        }

        if (i >= 50)
        {
            sum += force_val.val1;
        }

        k_sleep(K_MSEC(10));
    }

    data->calibration_factor = (sum / 50 - data->zero_offset) / cal_weight;
    LOG_INF("Calibration factor: %f (for weight: %f)", data->calibration_factor, cal_weight);

    return 0;
}

/**
 * Save calibration data to NVS.
 */
int set_calData_nvs(const struct device *dev)
{
    struct nau7802_data *data = dev->data;

    int err = store_calibration_data_nvs(data->zero_offset, data->calibration_factor);
    if (err != 0)
    {
        LOG_ERR("Failed to store calibration to NVS (%d)", err);
        return err;
    }

    LOG_INF("Calibration data saved to NVS.");
    return 0;
}
