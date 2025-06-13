/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPCALL_NAU7802_H_
#define APPCALL_NAU7802_H_

#include "sensor/nau7802/nau7802.h"

/*NVS-Specific Function Prototypes*/
int load_calib_fromNVS(const struct device *dev);

#endif