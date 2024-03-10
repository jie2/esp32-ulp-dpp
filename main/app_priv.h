/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define REPORTING_PERIOD   0.1 /* Seconds */

extern esp_rmaker_device_t *dpp_device;

void app_driver_init(void);
void app_sensor_update();
float app_get_measurement();
