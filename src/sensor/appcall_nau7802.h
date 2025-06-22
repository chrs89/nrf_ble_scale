/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPCALL_NAU7802_H_
#define APPCALL_NAU7802_H_

#include "sensor/nau7802/nau7802.h"
#include "../app_nau7802_cmd.h"

typedef void (*nau7802_cmd_handler_t)(void);

/*NVS-Specific Function Prototypes*/
int load_calib_fromNVS(const struct device *dev);

/*Request Handler Write States from Central*/
int nau7802_execute_command(enum app_nau7802_command cmd);

void reset_reason_print(void);

#endif