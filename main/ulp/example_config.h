/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once
//#define RAINMAKER

/* Ints are used here to be able to include the file in assembly as well */
#define EXAMPLE_ADC_CHANNEL     6 //GPIO7 on ESP32-S2

#define EXAMPLE_ADC_UNIT        0 // ADC_UNIT_1
#define EXAMPLE_ADC_ATTEN       0 // ADC_ATTEN_DB_11
#define EXAMPLE_ADC_WIDTH       0 // ADC_BITWIDTH_DEFAULT

#ifndef RAINMAKER
/* Set low and high thresholds, approx. 2.80V - 3.00V*/
#define EXAMPLE_ADC_LOW_TRESHOLD   260 //stable -> hibernate
#define EXAMPLE_ADC_HIGH_TRESHOLD   260 //wakeup
#define EXAMPLE_ADC_HIGHER_TRESHOLD   300//negative->hibernate
#define EXAMPLE_ADC_HIGH   500 //wakeup from stable high, not implemented yet lol
#endif


#ifdef RAINMAKER
/* Set low and high thresholds, approx. 2.80V - 3.00V*/
#define EXAMPLE_ADC_LOW_TRESHOLD   20 //stable -> hibernate
#define EXAMPLE_ADC_HIGH_TRESHOLD   21 //wakeup
#define EXAMPLE_ADC_HIGHER_TRESHOLD   50//negative->hibernate
#define EXAMPLE_ADC_HIGH   500 //wakeup from stable high, not implemented yet lol
#endif
