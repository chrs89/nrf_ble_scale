#include "appcall_nau7802.h"
#include <zephyr/logging/log.h>
#ifdef CONFIG_APP_ENABLE_NVSRW
#include "../nvs/nvs.h"
#endif
#include "calibration.h"

LOG_MODULE_REGISTER(NAU7802_Application, LOG_LEVEL_DBG);

extern const struct device *const nau7802; // May not be needed if passed as argument

#ifdef CONFIG_APP_ENABLE_NVSRW
int load_calib_fromNVS(const struct device *dev)
{
    if (!device_is_ready(nau7802))
    {
        LOG_ERR("Sensor not ready");
        return ENODEV;
    }

    /* Initialize NVS & Load Calibration Factos*/
    int ret = nau7802_nvs_init();
    if (ret != 0)
    {
        LOG_ERR("ret:%d, NVS init process fail", ret);
        return ret;
    }

    struct calibDataManuf cal_dataRead;
    /*Load Calibration Data from NVS*/
    ret = load_calibration_data_nvs(&cal_dataRead);
    if (ret != 0)
    {
        LOG_ERR("Failed to load calibration data, %d", ret);
        return ret;
    }

    LOG_INF("Loaded Calibration Data: Zero_Offset = %d, Calibration_Factor = %f", cal_dataRead.zero_offset, cal_dataRead.calibration_factor);

    /*USE NAU7802 SENSOR API to SET CALIBRATION FACTOR*/

    /*Set_Offset*/

    const struct sensor_value offset_val = {
        .val1 = cal_dataRead.zero_offset,
        .val2 = 0,
    };
    ret = nau7802_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OFFSET, &offset_val);
    if (ret != 0)
    {
        LOG_ERR("Failed to set Offset to NAU7802 Device, %d", ret);
    }

    struct sensor_value calib_val;
    sensor_value_from_float(&calib_val, cal_dataRead.calibration_factor);

    const struct sensor_value *ptr = &calib_val;

    ret = nau7802_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_Manufacturing_CALIBRATION_FACTOR, ptr);
    if (ret != 0)
    {
        LOG_ERR("Failed to set Offset to NAU7802 Device, %d", ret);
    }

    return 0;
}
#endif

// === Individual Command Handlers for Requests of <lbs_cb.aindx_cb> ===

static void handle_resume_thread(void)
{
    LOG_INF("Resume thread");
    nau7802_ownThreadResume(nau7802);
}

static void handle_suspend_thread(void)
{
    LOG_INF("Suspend thread");
    nau7802_ownThreadSuspend(nau7802);
}

static void handle_set_10sps(void)
{
    LOG_INF("Set 10 SPS");
    struct sensor_value rate = {.val1 = NAU7802_RATE_10SPS, .val2 = 0};
    nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &rate);
}

static void handle_set_20sps(void)
{
    LOG_INF("Set 20 SPS");
    struct sensor_value rate = {.val1 = NAU7802_RATE_20SPS, .val2 = 0};
    nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &rate);
}

static void handle_set_40sps(void)
{
    LOG_INF("Set 40 SPS");
    struct sensor_value rate = {.val1 = NAU7802_RATE_40SPS, .val2 = 0};
    nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &rate);
}

static void handle_set_80sps(void)
{
    LOG_INF("Set 80 SPS");
    struct sensor_value rate = {.val1 = NAU7802_RATE_80SPS, .val2 = 0};
    nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &rate);
}

static void handle_set_320sps(void)
{
    LOG_INF("Set 320 SPS");
    struct sensor_value rate = {.val1 = NAU7802_RATE_320SPS, .val2 = 0};
    nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &rate);
}

static void handle_tare(void)
{
    LOG_INF("Tare sensor");
    get_offset_data(nau7802);
}

// Optional stubs
static void handle_get_offset(void)
{
    LOG_INF("Set Offset (stub)");
    get_offset_data(nau7802);
}

static void handle_get_calibFactor_5kg(void)
{
    LOG_INF("Set Calibration Factor (stub)");
    float32_t selected_weight = (float32_t)calibrationWeightMap[CALIB_WEIGHT_5000G_IDX];
    get_calFactor_data(nau7802, selected_weight);
}

// === Handler Table ===

static const nau7802_cmd_handler_t handler_table[NAU7802_CMD_COUNT] = {
    [NAU7802_RESUME_THREAD] = handle_resume_thread,              // Index 0x00
    [NAU7802_SUSPEND_THREAD] = handle_suspend_thread,            // Index 0x01
    [NAU7802_SET_10SPS] = handle_set_10sps,                      // Index 0x02
    [NAU7802_SET_20SPS] = handle_set_20sps,                      // Index 0x03
    [NAU7802_SET_40SPS] = handle_set_40sps,                      // Index 0x04
    [NAU7802_SET_80SPS] = handle_set_80sps,                      // Index 0x05
    [NAU7802_SET_320SPS] = handle_set_320sps,                    // Index 0x06
    [NAU7802_TARA] = handle_tare,                                // Index 0x07
    [NAU7802_GET_OFFSET] = handle_get_offset,                    // Index 0x08
    [NAU7802_GET_CALIBRATIONF_5KG] = handle_get_calibFactor_5kg, // Index 0x09
};

int nau7802_execute_command(enum app_nau7802_command cmd)
{
    if (!device_is_ready(nau7802))
    {
        LOG_ERR("Sensor not ready");
        return ENODEV;
    }

    if (cmd >= NAU7802_CMD_COUNT || handler_table[cmd] == NULL)
    {
        LOG_WRN("Invalid or unimplemented command: %d", cmd);
        return ENOTSUP;
    }

    handler_table[cmd](); // Dispatch to actual handler

    return 0;
}