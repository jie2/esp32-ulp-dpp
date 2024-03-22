| Supported Targets | ESP32 | ESP32-S2 |
| ----------------- | ----- | -------- |

# ULP-DPP V3.1

Repositories required: ESP-32 RainMaker
https://rainmaker.espressif.com/docs/get-started.html

V3.1:
Added features on NVS storage. 

Based on the ULP ADC example, this board goes into deep sleep and wakes up at increasing voltage levels (SOC measured at >50), around >3.1V. Two voltage measurements are taken with 4.08s time interval in-between, and a value for SOC is calculated by subtracting the two values. This determines if the supply is increasing or decreasing, and has the system adjust states between deep-sleep and active mode accordingly. ESP-RainMaker is used to upload data (Temperature readings, ADC readings, and SOC) onto the cloud and retrievable through the mobile app. 

ULP Wakeup timer: 1s 
This timer determines when the board awakes from hibernate mode and performs tasks in deep-sleep, which can be extended for longer.

Time between measurements: 4.08s
This is set by the number of WAITs in deep-sleep.

Time for upload: 14s (3 provisions)
This is highly deterministic on duration of Wi-Fi provisioning and signal strength. 



Issues:
1. A high threshold isn't set, so there's a possibility of the board waking up from random voltage spikes.
2. ADC PIN7 is grounded for all measurements, results may vary with an open PIN7 due to adc reading.
3. No measurements can be conducted in hibernate mode, so any increase or decrease in that 1s duration won't be registered.
4. Data is not retained after brownout (supply drop).
5. Sleep instructions don't work on the S2 and S3, hence redundant in the code.

