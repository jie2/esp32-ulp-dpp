/* Temperature Sensor Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_periph.h"
#include "soc/sens_reg.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "ulp.h"
#include "ulp_main.h"
#include "esp_adc/adc_oneshot.h"
#include "ulp/example_config.h"
#include "ulp_adc.h"

#include "driver/temperature_sensor.h"

#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_pm.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define STORAGE_SIZE 5

//rainmaker repos, definition for test purposes
#define RAINMAKER

#ifdef RAINMAKER

#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_standard_devices.h"

#include "app_wifi.h"
#include "app_insights.h"

#include "app_priv.h"

#endif

#ifdef RAINMAKER
static const char *TAG = "app_main";

esp_rmaker_device_t *dpp_device;
#endif
//ULP repos
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */
static void init_ulp_program(void);

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program(void);

static void update_readings(esp_err_t);

static void reclaim_readings(esp_err_t);

void app_main()
{
    #ifdef RAINMAKER
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();
    #endif

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup\n");
        reclaim_readings(err); //reclaim prev readings from NVS
        printf("ULP did %"PRIu32" measurements since last reset\n", ulp_sample_counter & UINT16_MAX);
        
        init_ulp_program();
    } else {
        printf("Deep sleep wakeup\n");
        printf("ULP did %"PRIu32" measurements since last reset\n", ulp_sample_counter & UINT16_MAX);
        printf("Thresholds:  low=%"PRIu32"  high=%"PRIu32"\n", ulp_low_thr, ulp_high_thr);
        ulp_last_result &= UINT16_MAX;
        ulp_previous_result &= UINT16_MAX;

        printf("Value=%"PRIu32" was the previous result.\n", ulp_previous_result);
        printf("Value=%"PRIu32" was the last result, it is %s threshold\n", ulp_last_result,
                ulp_last_result < ulp_low_thr ? "below" : "above");
                
        printf("Value=%"PRIu32" is the SOC\n", ulp_charge_state);

        printf("The recorded adc readings from the ULP are: \n");
        for(int i=0;i<STORAGE_SIZE;i++)
        {
            ulp_adc_reading[i] &= UINT16_MAX;
            printf("Value=%"PRIu32" at [%d]\n", ulp_adc_reading[i],i);
        }

        
        printf("Value=%"PRIu32" was the temperature value sensed.\n", ulp_temperature_result & UINT16_MAX);
        update_readings(err); //update new readings to NVS
    } 
    #ifdef RAINMAKER
    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_wifi_init();
    
    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_wifi_init() but before app_wifi_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "DPP Sensor");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    
    /* Create a device and add the relevant parameters to it */
    dpp_device = esp_rmaker_temp_sensor_device_create("DPP Sensor", NULL , app_get_measurement());
    esp_rmaker_device_add_param(dpp_device, esp_rmaker_param_create("ADC", NULL, esp_rmaker_float(true), PROP_FLAG_READ | PROP_FLAG_SIMPLE_TIME_SERIES));
    esp_rmaker_device_add_param(dpp_device, esp_rmaker_param_create("Past ADC Readings", NULL, esp_rmaker_array("[]"),PROP_FLAG_READ));
    esp_rmaker_device_add_param(dpp_device, esp_rmaker_param_create("State of Charge", NULL, esp_rmaker_float(true), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES));

    esp_rmaker_node_add_device(node, dpp_device);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    //enable other stuff
    esp_rmaker_timezone_service_enable();


    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */

    err = app_wifi_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    vTaskDelay(300);
    #endif

    printf("Entering deep sleep\n\n");

    start_ulp_program();
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    
    
    esp_deep_sleep_start();
    
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* ULP ADC configurations. */
    ulp_adc_cfg_t cfg = {
        .adc_n    = EXAMPLE_ADC_UNIT,
        .channel  = EXAMPLE_ADC_CHANNEL,
        .width    = EXAMPLE_ADC_WIDTH,
        .atten    = EXAMPLE_ADC_ATTEN,
        .ulp_mode = ADC_ULP_MODE_FSM,
    };

    ESP_ERROR_CHECK(ulp_adc_init(&cfg));


    /* GPIO used for test. */
    gpio_num_t gpio_num = GPIO_NUM_2;
    //int rtcio_num = rtc_io_number_get(gpio_num);
    //assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");


    /* Initialize ulp variables*/
    ulp_low_thr = EXAMPLE_ADC_LOW_TRESHOLD; 
    ulp_high_thr = EXAMPLE_ADC_HIGH_TRESHOLD;
    ulp_higher_thr = EXAMPLE_ADC_HIGHER_TRESHOLD;
    ulp_higher_thr = EXAMPLE_ADC_HIGH;
    //ulp_io_number = rtc_io_desc[gpio_num].rtc_num; /* map from GPIO# to RTC_IO# */

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_OUTPUT_ONLY);


    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);


    /* Set ULP wake up period to 1s */
    ulp_set_wakeup_period(0, 1000000);

#if CONFIG_IDF_TARGET_ESP32
    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors on modules which have these (e.g. ESP32-WROVER)
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
#endif // CONFIG_IDF_TARGET_ESP32

    esp_deep_sleep_disable_rom_logging(); // suppress boot messages   
}

static void start_ulp_program(void)
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

//update readings to NVS
static void update_readings(esp_err_t err)
{
    const char* namespace = "ulp";
    const char* adc_key = "aydeesee";

    ESP_ERROR_CHECK(err); //init nvs
    nvs_handle handle;
    ESP_ERROR_CHECK( nvs_open(namespace, NVS_READWRITE, &handle));

    size_t size = sizeof(uint16_t);
    err = nvs_get_blob(handle,adc_key, NULL, &size);  
    uint16_t* aydeesee = malloc(size);


    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);


    for(int i=0;i<STORAGE_SIZE;i++)
    {
        ulp_adc_reading[i] &= UINT16_MAX;
        aydeesee[i] = ulp_adc_reading[i];
    }
    
    /* Save the new value to NVS */
    ESP_ERROR_CHECK(nvs_set_blob(handle, adc_key, aydeesee, size));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    //Print what was saved
    for(int i=0;i<STORAGE_SIZE;i++)
    {
        printf("Wrote readings to NVS: %u\n", aydeesee[i]);
    }
    
}

//read previous reading from NVS
static void reclaim_readings(esp_err_t err)
{
    const char* namespace = "ulp";
    const char* adc_key = "aydeesee";

    ESP_ERROR_CHECK( err ); //init nvs
    nvs_handle handle;
    ESP_ERROR_CHECK( nvs_open(namespace, NVS_READWRITE, &handle));
    
    size_t size = sizeof(uint16_t);
    err = nvs_get_blob(handle,adc_key, NULL, &size); 
    uint16_t* aydeesee = malloc(size);
 
    err = nvs_get_blob(handle, adc_key, aydeesee, &size);

    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);

    for(int i=0;i<STORAGE_SIZE;i++)
    {
        printf("Reading from NVS: %u\n", aydeesee[i]);
    }

    for(int i=0;i<STORAGE_SIZE;i++)
    {
        ulp_adc_reading[i] = aydeesee[i];
    }
    
    nvs_close(handle);
}

