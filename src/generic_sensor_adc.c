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

static const float adc_max_scale = (600.0f * 6.0f) / (16383.0f);

// int16_t adc_voltage[BUFFER_SIZE];

static const struct adc_channel_cfg m_channel_1_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_1_ID,
    // .differential = 1,
    .input_positive = NRF_SAADC_INPUT_AIN0,
    // .input_negative = NRF_SAADC_INPUT_AIN1,
};

static const struct adc_channel_cfg m_channel_2_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_2_ID,
    // .differential = 1,
    .input_positive = NRF_SAADC_INPUT_AIN1,
    // .input_negative = NRF_SAADC_INPUT_AIN3,
};
static const struct adc_channel_cfg m_channel_3_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_3_ID,
    // .differential = 1,
    .input_positive = NRF_SAADC_INPUT_AIN2,
    // .input_negative = NRF_SAADC_INPUT_AIN5,
};

void generic_sensor_adc_sample(int16_t adc_voltage[])
{
    static int err;
    static int16_t m_sample_buffer[BUFFER_SIZE];

    if (!adc_dev) {
        printk("Missing device\n");
        return;
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
        adc_voltage[i] = (int)((float)m_sample_buffer[i] * adc_max_scale);
        // Print the values adc_max_scale
        // printf("scale: %2.20f \n", adc_max_scale); //0,21973997436366965757187328328145
        // printf("dif: %f \n", adc_max_scale - 0.21973997436366965757187328328145f);
        // printk("ADC raw value: %d \n", m_sample_buffer[i]);
        // printk("Estimated voltage: %d mV\n", adc_voltage[i]);
    }
}

void generic_sensor_adc_multi_sample(int16_t adc_voltage[])
{
    static int err;
    static int16_t m_sample_buffer[BUFFER_SIZE];
    static int32_t cum[BUFFER_SIZE];
    static const int16_t oversample_N = 20;

    if (!adc_dev) {
        printk("Missing device\n");
        return;
    }
    
    // init the adc_voltage with zeros
    for (int i = 0; i < BUFFER_SIZE; i++) {
        adc_voltage[i] = 0;
        cum[i] = 0;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_1_ID) | BIT(ADC_CHANNEL_2_ID) | BIT(ADC_CHANNEL_3_ID),
        .buffer = m_sample_buffer,
        .buffer_size = sizeof(m_sample_buffer),
        .resolution = ADC_RESOLUTION,
    };
    
    for (int i = 0; i < oversample_N; i++) {
        // printk("iteration: %d\n", i);
    
        err = adc_read(adc_dev, &sequence);
        if (err) {
            printk("Error in adc sampling: %d\n", err);
        }

        for (int j = 0; j < BUFFER_SIZE; j++) {
            cum[j] = cum[j] + m_sample_buffer[j];
            // printk("cumulated value: %d \n", cum[j]);
            // printk("buffer: %d mV\n", m_sample_buffer[j]);
        }
    }
        
    // Convert the values
    for (int i = 0; i < BUFFER_SIZE; i++) {
        /*
        2^11 = 2048
        2^12 = 4096
        2^13 = 8192
        2^14 = 16384
        */
        adc_voltage[i] = (int)(( (float)cum[i] / (float)oversample_N) * adc_max_scale);
        // Print the values
        // printk("cumulated value: %d \n", cum[i]);
        printk("Estimated voltage: %d mV\n", adc_voltage[i]);
    }

}

int generic_sensor_adc_init(void)
{
    int err;

    printk("nRF52 SAADC sampling 3 channels AIN0 AIN1 AIN2\n");

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
    int16_t values[3];
    printk("Calibration triggered, first value will be incorrect.\n");
    generic_sensor_adc_sample(values);
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

