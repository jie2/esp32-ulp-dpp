| Supported Targets | ESP32 | ESP32-S2 |
| ----------------- | ----- | -------- |

# ULP ADC Example

Based on the ULP ADC example, this board goes into deep sleep and wakes up at increasing voltage levels (SOC measured at >50), around >3.1V. Two voltage measurements are taken with 4.08s time interval in-between, and a value for SOC is calculated by subtracting the two values. This determines if the supply is increasing or decreasing, and has the system adjust states between deep-sleep and active mode accordingly.

ULP Wakeup timer: 1s 
This timer determines when the board awakes from hibernate mode and performs tasks in deep-sleep, which can be extended for longer.
Time between measurements: 4.08s
This is set by the number of WAITs in deep-sleep.

Issues:
1. A high threshold isn't set, so there's a possibility of the board waking up from a random voltage spike.
2. ADC PIN7 is grounded for all measurements, but 

