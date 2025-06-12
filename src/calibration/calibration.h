#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

int get_offset_data(const struct device *dev);
int get_calFactor_data(const struct device *dev, float32_t cal_weight);
int set_calData_nvs(const struct device *dev);

#endif // CALIBRATION_H
