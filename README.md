# DFRobot Gravity TDS Sensor for ESP32

FreeRTOS task to read the DFRobot Gravity TDS sensor on an ESP32.

Mark Benson 2018
Licence - MIT or whatever is used for the rest of this project!

Compiles with the Espressif ESP-IDF.

The sensor is powered directly from an ESP32 GPIO pin as it only requires 3-6mA and the GPIOs can sink 12mA. Defaults to GPIO_NUM_32.
The analog pin is connected to ADC1_CHANNEL_0 (GPIO_NUM_36).

What it does:
Enables the sensor and waits 10 seconds for the reading to settle.
Then takes a number of readings a few seconds apart and gives an average sample value.
The sample value is then converted to a PPM value. The conversion includes some compensation values for the SAR ADC 11dB range. There is also a temperature compensation value that is currently static, but this should be measured to improve accuracy. 
Once the value is computed it is returned and the sensor is disabled until the next reading is needed.

The given TDS PPM value should be compared with a TDS meter and calibrated against a known sample.

TDS PPM conversion based on https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
