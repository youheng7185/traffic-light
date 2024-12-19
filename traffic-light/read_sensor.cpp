#include "sensor.h"
#include <Arduino_FreeRTOS.h>
#include <Arduino.h>

void countCarTaskTOF(void *pvParameters) {
  while(1) {
    unsigned long startTime = millis(); // Record the start time of the loop
    uint16_t range = 0;
    uint8_t count = 1;

    for (int idx = VL53L0X_IDX_FIRST; idx <= VL53L0X_IDX_SIXTH; ++idx) {
      vl53l0x_read_range_single(idx, range);
      // Read and print range for the first sensor
      //convertValue(VL53L0X_IDX_FIRST, rangeVal);
      Serial.print("Sensor ");
      Serial.print(count);
      Serial.print(": ");
      Serial.println(range);
      count++;
      if(count == 7) {
        count = 1;
      }
    }

    unsigned long endTime = millis(); // Record the end time of the loop
    unsigned long executionTime = endTime - startTime; // Calculate execution time

    Serial.print("Loop Execution Time: ");
    Serial.print(executionTime);
    Serial.println(" ms");

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
