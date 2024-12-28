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
  unsigned long lastPrintTime = startTime;
  int car_count[3] = {0, 10, 20};
  bool last_state[6] = {0};

  int roadToGo = 0;

  while(1) {
    unsigned long currentTime = millis();
    uint16_t range = 0;
    uint8_t count = 1; // sensor number (1-6)

    for (int idx = VL53L0X_IDX_FIRST; idx <= VL53L0X_IDX_SIXTH; ++idx) {
      vl53l0x_read_range_single(idx, range);

      if (range < car_threshold) {
        if (last_state[count - 1] == 0) {
          if (count % 2)
            ++car_count[(count - 1) / 2];
          else
            --car_count[(count - 1) / 2];
        car_count[(count - 1) / 2] = max(0, car_count[(count - 1) / 2]);
        last_state[count - 1] = 1;
        }
      }
      else
        last_state[count - 1] = 0;

      if (currentTime - lastPrintTime >= 1000)
      {
        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(car_count[0]);
        Serial.print(",");
        Serial.print(car_count[1]);
        Serial.print(",");
        Serial.print(car_count[2]);
        Serial.println();
        lastPrintTime = currentTime;
      }

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
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

