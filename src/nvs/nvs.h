/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NVS_H_
#define NVS_H_

#ifdef CONFIG_APP_ENABLE_NVSRW

#include "sensor/nau7802/nau7802.h"

/*NVS-Specific Function Prototypes*/
int nau7802_nvs_init(void);
int load_gain_nvs(float32_t *gain);
int load_offset_nvs(int32_t *offset);
int store_offset_nvs(int32_t offset);
int store_gain_nvs(float gain);
int store_calibration_data_nvs(int32_t offset, float32_t gain);
int load_calibration_data_nvs(struct calibDataManuf *cal_data);

#endif /* CONFIG_APP_ENABLE_NVSRW */
#endif