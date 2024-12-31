#include "HardwareSerial.h"
#include "FreeRTOSVariant.h"
#include "sensor.h"
#include "led.h"
#include <Arduino_FreeRTOS.h>
#include <Arduino.h>

const int car_threshold = 100; // to be changed accordingly
const int32_t min_green_time = 5000;

void countCarTaskTOF(void *pvParameters) {
  unsigned long startTime = millis(); // Record the start time of the loop
  unsigned long lastPrintTime = startTime;
  int32_t car_count[3] = {0, 10, 20};
  bool last_state[6] = {0};

  int roadToGo = 0;

  while(1) {
    unsigned long currentTime = millis();
    uint16_t range = 0;
    uint8_t sensorNum = 1; // sensor number (1-6)

    for (int idx = VL53L0X_IDX_FIRST; idx <= VL53L0X_IDX_SIXTH; ++idx) {
      vl53l0x_read_range_single(idx, range);

      if (range < car_threshold) {
        if (last_state[sensorNum - 1] == 0) {
          if (sensorNum % 2) // enter road sensor
            ++car_count[(sensorNum - 1) / 2];
          else // exit road sensor
            --car_count[(sensorNum - 1) / 2];
        car_count[(sensorNum - 1) / 2] = max(0, car_count[(sensorNum - 1) / 2]);
        last_state[sensorNum - 1] = 1;
        }
      }
      else
        last_state[sensorNum - 1] = 0;

	  if (currentTime - lastPrintTime >= 100) {
      Serial.print(roadToGo);
      Serial.print(",");
      Serial.print(car_count[0]);
      Serial.print(",");
      Serial.print(car_count[1]);
      Serial.print(",");
      Serial.print(car_count[2]);
      Serial.print(",");
      Serial.print(currentTime - startTime);
      Serial.print(",");      
      Serial.print(min(min_green_time + car_count[roadToGo] * 2000, 30000));
      Serial.println();
      lastPrintTime = currentTime;
	  }

      sensorNum++;
      if(sensorNum == 7) {
        sensorNum = 1;
      }
    }

    // 6s base + 2s per car, max 30s
    if (currentTime - startTime > min(min_green_time + car_count[roadToGo] * 2000, 30000)) {
      vTaskResume(trafficLightTaskHandle);
      roadToGo = (roadToGo + 1) % 3;
      startTime = millis();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

