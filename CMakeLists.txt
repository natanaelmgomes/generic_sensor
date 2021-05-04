# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(generic_sensor)

target_sources(app PRIVATE
    src/main.c
    src/generic_sensor_adc.c
    src/generic_sensor_adc.h
    src/generic_led.c
    src/generic_led.h
)

FILE(GLOB app_sources src/*.c)

# zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
