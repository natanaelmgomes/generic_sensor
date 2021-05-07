/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <stdint.h>

#ifndef GENERIC_SENSOR_ADC__H
#define GENERIC_SENSOR_ADC__H

void generic_sensor_adc_sample(int16_t adc_voltage[]);
int generic_sensor_adc_init(void);

#endif


