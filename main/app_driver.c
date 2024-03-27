/*  Temperature Sensor demo implementation using RGB LED and timer

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 
#include "ulp.h"
#include "ulp_main.h"
#include "esp_adc/adc_oneshot.h"
#include "ulp/example_config.h"
#include "ulp_adc.h"

#include "driver/temperature_sensor.h"

#include <app_reset.h>
#include <ws2812_led.h>
#include "app_priv.h"

#include <string.h>

#define STORAGE_SIZE 5

static TimerHandle_t sensor_timer; 


float tsens_value;
static float g_adc;
static float g_temp;
static float g_charge;
char g_past_adc[100]; //change 100 to max size of string
void floatArrayToString(uint32_t arr[], int size, char *str);


void app_sensor_update()
{
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(dpp_device, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(g_temp)
    );

    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(dpp_device, "ADC"),
            esp_rmaker_float(g_adc)
    );

    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(dpp_device, "State of Charge"),
            esp_rmaker_float(g_charge)
    );

    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(dpp_device, "Past ADC Readings"),
            esp_rmaker_array(g_past_adc)
    );

}

float app_get_measurement()
{

    //TEMPERATURE SENSOR
    /*
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_LOGI(TAG, "Read temperature");
    */
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));


    floatArrayToString(ulp_adc_reading, STORAGE_SIZE, g_past_adc); // Convert array to string to upload as JSON array

    //printf("%s\n", g_past_adc);
    


    g_temp = tsens_value;
    g_charge = ulp_charge_state; 
    g_adc = ulp_adc_reading[STORAGE_SIZE-1];

    
    return g_temp;
}

esp_err_t app_sensor_init()
{
    esp_err_t err = ws2812_led_init();
    if (err != ESP_OK) {
        return err;
    }

    sensor_timer = xTimerCreate("app_sensor_update_tm", (REPORTING_PERIOD * 1000) / portTICK_PERIOD_MS,
                                pdTRUE, NULL, app_sensor_update);
    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);

        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_driver_init()
{
    app_sensor_init();
}

//converts c string to string, required parameter for JSON 
void floatArrayToString(uint32_t arr[], int size, char *str) {
    int i;
    strcat(str, "[");
    for (i = 0; i < size; i++) {
        char temp[100]; //MAX size set to 100
        snprintf(temp, sizeof(temp), "%"PRIu32"", arr[i]); // Format float to string with 2 decimal places
        strcat(str, temp);
        if (i < size - 1) {
            strcat(str, ", ");
        }
    }
    strcat(str, "]");
}
