/* ULP Example

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
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_pm.h"

#include "lwip/err.h"
#include "lwip/sys.h"

//CLOUD REPO

#define EXAMPLE_ESP_WIFI_SSID      "Li Xin P30"
#define EXAMPLE_ESP_WIFI_PASS      "peepeepoopoo"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER ""

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

//Array Size - storage for deep sleep cycling
#define STORAGE_SIZE 5
static const char *TAG = "lx"; 

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

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

void app_main(void)
{  
    //TEMPERATURE SENSOR
    /*
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_LOGI(TAG, "Read temperature");
    */
    float tsens_value;
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    
    
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));

    /*
    //WIFI MODULE

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    */

   /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        reclaim_readings(err); //reclaim prev readings from NVS
        printf("Not ULP wakeup\n");
        init_ulp_program();
    } else {
        printf("Deep sleep wakeup\n");
        printf("ULP did %"PRIu32" measurements since last reset\n", (ulp_sample_counter) & UINT16_MAX);
        printf("Thresholds:  low=%"PRIu32"  high=%"PRIu32"\n", ulp_low_thr, ulp_high_thr);
        ulp_last_result &= UINT16_MAX;
        ulp_previous_result &= UINT16_MAX;

        printf("Value=%"PRIu32" was the previous result\n", ulp_previous_result);
        printf("Value=%"PRIu32" was %s threshold\n", ulp_last_result,
                ulp_last_result < ulp_low_thr ? "below" : "above");
        printf("SOC is %"PRIu32".\n", ulp_charge_state);

        printf("The recorded adc readings from the ULP are: \n");
        for(int i=0;i<STORAGE_SIZE;i++)
        {
            ulp_adc_reading[i] &= UINT16_MAX;
            printf("Value=%"PRIu32" at [%d]\n", ulp_adc_reading[i],i);
        }

        printf("Temperature value %.02f ℃ \n", tsens_value);
        printf("Value=%"PRIu32" was the temperature value sensed.\n", ulp_temperature_result & UINT16_MAX); 
        update_readings(err); //update new readings to NVS
    }
    printf("Entering deep sleep\n\n");

    start_ulp_program();
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    

#if !CONFIG_IDF_TARGET_ESP32
    /* RTC peripheral power domain needs to be kept on to keep SAR ADC related configs during sleep */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
#endif

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

    /* GPIO used for test. */
    gpio_num_t gpio_num = GPIO_NUM_2;
    //int rtcio_num = rtc_io_number_get(gpio_num);
    //assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

    ESP_ERROR_CHECK(ulp_adc_init(&cfg));

    ulp_low_thr = EXAMPLE_ADC_LOW_TRESHOLD;
    ulp_high_thr = EXAMPLE_ADC_HIGH_TRESHOLD;
    ulp_higher_thr = EXAMPLE_ADC_HIGHER_TRESHOLD;
    ulp_high = EXAMPLE_ADC_HIGH;

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
