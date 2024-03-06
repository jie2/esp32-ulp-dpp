| Supported Targets | ESP32 | ESP32-S2 |
| ----------------- | ----- | -------- |

# ULP ADC Example

Based on the ULP ADC example, this code goes into deep sleep and wakes up at increasing voltage levels from around 3.3V to take a temperature measurement from the ESP32 temperature sensor in active mode.
When the voltage level goes under around 3.0V, the system hibernates until the next wakeup from ULP timer (every 1s).

Taking immediate values of voltage, the system actively reacts to changes in voltage supply.

Connections:
Connect PIN7 (ADC) to Voltage Divider/Power Supply.

System1-ADC - graph of measured ADC against Voltage Supply to ADC through Voltage Divider (120k(ohm) - 12k (ohm))
