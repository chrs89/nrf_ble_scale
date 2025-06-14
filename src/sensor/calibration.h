#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

typedef enum _calib_weight
{
    CALIB_WEIGHT_1000G_IDX = 0,
    CALIB_WEIGHT_2000G_IDX,
    CALIB_WEIGHT_5000G_IDX,
    CALIB_WEIGHT_COUNT
} calibration_weight;

static const float32_t calibrationWeightMap[CALIB_WEIGHT_COUNT] = {
    [CALIB_WEIGHT_1000G_IDX] = 1.0f,
    [CALIB_WEIGHT_2000G_IDX] = 2.0f,
    [CALIB_WEIGHT_5000G_IDX] = 5.0f,
};

int get_offset_data(const struct device *dev);
int get_calFactor_data(const struct device *dev, float32_t cal_weight);
int set_calData_nvs(const struct device *dev);

#endif // CALIBRATION_H
