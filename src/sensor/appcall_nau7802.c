#include "../nvs/nvs.h"
#include "appcall_nau7802.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(NAU7802_Application, LOG_LEVEL_DBG);

extern const struct device *const nau7802; // May not be needed if passed as argument

int load_calib_fromNVS(const struct device *dev)
{

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
        LOG_ERR("Failed to load calibration data, %d",ret);
        return ret;
    }

    LOG_INF("Loaded Calibration Data: Zero_Offset = %d, Calibration_Factor = %f", cal_dataRead.zero_offset, cal_dataRead.calibration_factor);

    /*USE NAU7802 SENSOR API to SET CALIBRATION FACTOR*/

    /*Set_Offset*/

    const struct sensor_value offset_val = {
        .val1 = cal_dataRead.zero_offset,
        .val2 = 0,
    };
    ret = nau7802_attr_set(dev,SENSOR_CHAN_ALL,SENSOR_ATTR_OFFSET,&offset_val);
    if (ret != 0)
    {
        LOG_ERR("Failed to set Offset to NAU7802 Device, %d",ret);
    }
    
    struct sensor_value calib_val;
    sensor_value_from_float(&calib_val,cal_dataRead.calibration_factor);

    const struct sensor_value *ptr = &calib_val;
    


    ret = nau7802_attr_set(dev,SENSOR_CHAN_ALL,SENSOR_ATTR_Manufacturing_CALIBRATION_FACTOR,ptr);
    if (ret != 0)
    {
        LOG_ERR("Failed to set Offset to NAU7802 Device, %d",ret);
    }



    return 0;
}