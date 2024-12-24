#include "HardwareSerial.h"
#include "FreeRTOSVariant.h"
#include "sensor.h"
#include "led.h"
#include <Arduino_FreeRTOS.h>
#include <Arduino.h>

const int car_threshold = 100;
const int min_green_time = 5000;

void countCarTaskTOF(void *pvParameters) {
  unsigned long startTime = millis(); // Record the start time of the loop
  int car_count[3] = {0, 10, 20};
  bool last_state[6] = {0};

  int roadToGo = 0;
  while(1) {
    unsigned long currentTime = millis();
    uint16_t range = 0;
    uint8_t count = 1;

    for (int idx = VL53L0X_IDX_FIRST; idx <= VL53L0X_IDX_SIXTH; ++idx) {
      vl53l0x_read_range_single(idx, range);
      // Read and print range for the first sensor
      //convertValue(VL53L0X_IDX_FIRST, rangeVal);
      // Serial.print("Sensor ");
      // Serial.print(count);
      // Serial.print(": ");
      // Serial.println(range);

      if (range < car_threshold)
      {
        if (last_state[count - 1] == 0)
          car_count[(count - 1) / 2] += (count % 2) - !(count % 2);
        car_count[(count - 1) / 2] = max(0, car_count[(count - 1) / 2]);
        last_state[count - 1] = 1;
      }
      else
        last_state[count - 1] = 0;

      Serial.print("Road 1 :");
      Serial.println(car_count[0]);

      Serial.print("Road 2 :");
      Serial.println(car_count[1]);

      Serial.print("Road 3 :");
      Serial.println(car_count[2]);

      Serial.print("Currently at road: ");
      Serial.println(roadToGo + 1);
      Serial.println();

      count++;
      if(count == 7) {
        count = 1;
      }
    }

    // 6s base + 2s per car, max 30s
    if (currentTime - startTime > min(100000 + max(0,car_count[roadToGo]) * 20000, 500000) / portTICK_PERIOD_MS) {
      vTaskResume(trafficLightTaskHandle);
      startTime = millis();
      roadToGo = (roadToGo + 1) % 3;
      // Serial.print("Switching to road ");
      // Serial.println(roadToGo + 1);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

