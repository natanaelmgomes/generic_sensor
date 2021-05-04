/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include "generic_sensor_adc.h"

#include <stdio.h>
#include <string.h>
#include <drivers/uart.h>
#include <drivers/adc.h>
#include <zephyr.h>

const struct device *adc_dev;

#include <hal/nrf_saadc.h>
#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 14
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 3)
#define ADC_CHANNEL_1_ID 1
#define ADC_CHANNEL_2_ID 2
#define ADC_CHANNEL_3_ID 3
#define BUFFER_SIZE 3

static const struct adc_channel_cfg m_channel_1_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_1_ID,
    .differential = 1,
    .input_positive = NRF_SAADC_INPUT_AIN0,
    .input_negative = NRF_SAADC_INPUT_AIN1,
};

static const struct adc_channel_cfg m_channel_2_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_2_ID,
    .differential = 1,
    .input_positive = NRF_SAADC_INPUT_AIN2,
    .input_negative = NRF_SAADC_INPUT_AIN3,
};
static const struct adc_channel_cfg m_channel_3_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_3_ID,
    .differential = 1,
    .input_positive = NRF_SAADC_INPUT_AIN4,
    .input_negative = NRF_SAADC_INPUT_AIN5,
};

int * generic_sensor_adc_sample(void)
{
    static int err;
    static int16_t m_sample_buffer[BUFFER_SIZE];
    static int adc_voltage[BUFFER_SIZE];

    if (!adc_dev) {
        printk("Missing device\n");
        return adc_voltage;
    }
    
    // init the adc_voltage with zeros
    for (int i = 0; i < BUFFER_SIZE; i++) {
        adc_voltage[i] = 0;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_1_ID) | BIT(ADC_CHANNEL_2_ID) | BIT(ADC_CHANNEL_3_ID),
        .buffer = m_sample_buffer,
        .buffer_size = sizeof(m_sample_buffer),
        .resolution = ADC_RESOLUTION,
    };
        
    err = adc_read(adc_dev, &sequence);
    if (err) {
        printk("Error in adc sampling: %d\n", err);
    }
    
    // Convert the values
    for (int i = 0; i < BUFFER_SIZE; i++) {
        /*
         2^11 = 2048
         2^12 = 4096
         2^13 = 8192
         2^14 = 16384
         */
        adc_voltage[i] = (int)((((float)m_sample_buffer[i] / 8192.0f) * 600.0f * 6.0f) * 1.0f);
        // Print the values
        printk("ADC raw value: %d \n", m_sample_buffer[i]);
        printk("Estimated voltage: %d mV\n", adc_voltage[i]);
    }

    printk("\n");
    return adc_voltage;
}

int generic_sensor_adc_init(void)
{
    int err;

    printk("nrf52 saadc sampling 3 channels differential AIN0~AIN1 AIN2~AIN3 AIN4~AIN5\n");

    adc_dev = device_get_binding("ADC_0");
    if (!adc_dev) {
        printk("device_get_binding ADC_0 failed\n");
        return -1;
    }
    // Config ADC
    err = adc_channel_setup(adc_dev, &m_channel_1_cfg);
    if (err) {
        printk("Error in adc setup 1 : %d\n", err);
        return -1;
    }

    err = adc_channel_setup(adc_dev, &m_channel_2_cfg);
    if (err) {
        printk("Error in adc setup 2: %d\n", err);
        return -1;
    }

    err = adc_channel_setup(adc_dev, &m_channel_3_cfg);
    if (err) {
        printk("Error in adc setup 3: %d\n", err);
        return -1;
    }

    /* Trigger offset calibration
    * As this generates a _DONE and _RESULT event
    * the first result will be incorrect.
    */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    static int * values;
    values = generic_sensor_adc_sample();
    // while (1) {
    //     static int * values;
    //     values = generic_sensor_adc_sample();

    //     /* Print the received values */
    //     for (int i = 0; i < BUFFER_SIZE; i++) {
    //         printf("received value: %i mV\n", values[i]);
    //     }
    //     printk("*** *** *** *** *** *** *** *** *** ***\n\n");
    //     k_sleep(K_MSEC(50));
    // }
    return 0;
}
