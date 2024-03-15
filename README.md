| Supported Targets | ESP32 | ESP32-S2 |
| ----------------- | ----- | -------- |

# ULP DPP V2.1
Compared to V2, this version is edited for use in a board with removed components: Power LED, CP2102N Chip and UART Communications.
Instead of (120k(ohm) - 12k(ohm)), a (120k(ohm) - 120k(ohm)) voltage divider is used.

Based on the ULP ADC example, this board goes into deep sleep and wakes up at increasing voltage levels (SOC measured at >50), around >3.1V. Two voltage measurements are taken with 4.08s time interval in-between, and a value for SOC is calculated by subtracting the two values. This determines if the supply is increasing or decreasing, and has the system adjust states between deep-sleep and active mode accordingly.

ULP Wakeup timer: 1s 
This timer determines when the board awakes from hibernate mode and performs tasks in deep-sleep, which can be extended for longer.
Time between measurements: 4.08s
This is set by the number of WAITs in deep-sleep.

Issues:
1. ADC PIN7 is grounded for all measurements, results may vary with an open PIN7 due to adc reading.
2. No measurements can be conducted in hibernate mode, so any increase or decrease in that 1s duration won't be registered.
3. Data is not retained after brownout (supply drop).
4. Wi-Fi is initialized but not implemented.
5. Sleep instructions don't work on the S2 and S3, hence redundant in the code.

