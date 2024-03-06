/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

/* Ints are used here to be able to include the file in assembly as well */
#define EXAMPLE_ADC_CHANNEL     6 //GPIO7 on ESP32-S2

#define EXAMPLE_ADC_UNIT        0 // ADC_UNIT_1
#define EXAMPLE_ADC_ATTEN       0 // ADC_ATTEN_DB_11
#define EXAMPLE_ADC_WIDTH       0 // ADC_BITWIDTH_DEFAULT

/* Set low and high thresholds, approx. 3.00V - 3.30V*/
#define EXAMPLE_ADC_LOW_TRESHOLD   500
#define EXAMPLE_ADC_HIGH_TRESHOLD   600
