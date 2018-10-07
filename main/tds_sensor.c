/* TDS Sensor Example

   Mark Benson 2018

   Licence: MIT (or whatever is used by the rest of this project)
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

#define TDS_ENABLE_GPIO     GPIO_NUM_32 //Note: Power from GPIO is 12mA, board requires 3~6mA, so enable pin powers board
#define TDS_ANALOG_GPIO     ADC1_CHANNEL_0 //ADC1 is availalbe on pins 15, 34, 35 & 36

#define TDS_STABILISATION_DELAY     10  //(int) How long to wait (in seconds) after enabling sensor before taking a reading
#define TDS_NUM_SAMPLES             10  //(int) Number of reading to take for an average
#define TDS_SAMPLE_PERIOD           20  //(int) Sample period (delay between samples == sample period / number of readings)
#define TDS_TEMPERATURE             18.0  //(float) Temperature of water (we should measure this with a sensor to get an accurate reading)
#define TDS_VREF                    1.18   //(float) Voltage reference for ADC. We should measure the actual value of each ESP32

static const char *TDS = "TDS INFO";
float sampleDelay = (TDS_SAMPLE_PERIOD / TDS_NUM_SAMPLES) * 1000;

void expose_vref(){
    // Expose the ADC VREF to a GPIO so we can measure it rather than assume it is 1.1v
    ESP_ERROR_CHECK(adc2_vref_to_gpio(GPIO_NUM_25));
    ESP_LOGI(TDS, "VREF routed to ADC1, pin 25\n");
}

void config_pins(){
    ESP_LOGI(TDS, "Configure pins required for TDS sensor.");
    // Pin to power the TDS sensor
    gpio_pad_select_gpio(TDS_ENABLE_GPIO);
    gpio_set_direction(TDS_ENABLE_GPIO, GPIO_MODE_OUTPUT);
    // Pin to read the TDS sensor analog output
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TDS_ANALOG_GPIO, ADC_ATTEN_DB_11);
}

void enable_tds_sensor(){
    ESP_LOGI(TDS, "Enabling TDS sensor & waiting 10 seconds.");
    gpio_set_level(TDS_ENABLE_GPIO, 1);
    // Wait 10 seconds
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}

void disable_tds_sensor(){
    ESP_LOGI(TDS, "Disabling TDS sensor.");
    gpio_set_level(TDS_ENABLE_GPIO, 0);
}

float read_tds_sensor(int numSamples, float sampleDelay){
    // Take n sensor readings every p millseconds where n is numSamples, and p is sampleDelay.
    // Return the average sample value.
    uint32_t runningSampleValue = 0;

    for(int i = 0; i < numSamples; i++) {
        // Read analogue value
        int analogSample = adc1_get_raw(TDS_ANALOG_GPIO);
        ESP_LOGI(TDS, "Read analog value %d then sleep for %f milli seconds.", analogSample, sampleDelay);
        runningSampleValue = runningSampleValue + analogSample;
        vTaskDelay(sampleDelay / portTICK_PERIOD_MS);
        
    }
    
    float tdsAverage = runningSampleValue / TDS_NUM_SAMPLES;
    ESP_LOGI(TDS, "Calculated average = %f", tdsAverage);
    return tdsAverage;
}

float convert_to_ppm(float analogReading){
    ESP_LOGI(TDS, "Converting an analog value to a TDS PPM value.");
    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
    float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
    float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
    float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
    float compensationCoefficient=1.0+0.02*(TDS_TEMPERATURE-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;  //temperature compensation
    float tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

    ESP_LOGI(TDS, "Volts per division = %f", vPerDiv);
    ESP_LOGI(TDS, "Average Voltage = %f", averageVoltage);
    ESP_LOGI(TDS, "Temperature (currently fixed, we should measure this) = %f", TDS_TEMPERATURE);
    ESP_LOGI(TDS, "Compensation Coefficient = %f", compensationCoefficient);
    ESP_LOGI(TDS, "Compensation Voltge = %f", compensationVolatge);
    ESP_LOGI(TDS, "tdsValue = %f ppm", tdsValue);
    return tdsValue;
}

void tds_task(void * pvParameters){
    ESP_LOGI(TDS, "TDS Measurement Control Task: Starting");
    while(1){
        ESP_LOGI(TDS, "TDS Measurement Control Task: Read Sensor");
        enable_tds_sensor();
        float sensorReading = read_tds_sensor(TDS_NUM_SAMPLES, sampleDelay);
        float tdsResult = convert_to_ppm(sensorReading);
        printf("TDS Reading = %f ppm\n", tdsResult);
        disable_tds_sensor();
        ESP_LOGI(TDS, "TDS Measurement Control Task: Sleeping 1 minute");
        vTaskDelay(((1000 / portTICK_PERIOD_MS)*60)*1); //delay in minutes between measurements
    }
}

void app_main()
{
    // Enable sensor then wait 10 seconds for reading to stabilise before reading
    // Read the sensor a few times and get an average
    // Convert reading to a Total Disolved Solids (TDS) value
    //expose_vref();
    config_pins();
    // Start TDS Task
    xTaskCreate(tds_task, "tds_task", 2048, NULL, 5, NULL);
}