#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "led.h"

// lane A
#define AR 2
#define AY 3
#define AG 4

// lane B
#define BR 5
#define BY 6
#define BG 7

// lane C
#define CR 8
#define CY 9
#define CG 10

int allLedPins[9] = {AR, AY, AG, BR, BY, BG, CR, CY, CG};

void initLed();
void trafficLightTask(void *pvParameters);
void green(int road);
void turnOffAll();

void initLed() {
  for (int i = 0; i < 9; i++) {
    pinMode(allLedPins[i], OUTPUT);
    digitalWrite(allLedPins[i], LOW);
  }
}

void trafficLightTask(void *pvParameters) {
  int currentRoad = 0;
  while (1) {
    green(currentRoad);

    currentRoad = (currentRoad + 1) % 3;

    // Serial.println("Suspending task after cycle");
    vTaskSuspend(NULL);  // Suspend this task until explicitly resumed
  }
}

void blink_green(uint8_t pin)
{
  for (int i = 0; i < 3; ++i) {
    digitalWrite(pin, LOW);
    vTaskDelay(250/portTICK_PERIOD_MS);
    digitalWrite(pin, HIGH);
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
}

void green(int road) {
  switch(road) {
    case 0:
      // previous lane blink green
      blink_green(CG);
      // reset
      turnOffAll();
      // previous lane turns yellow
      digitalWrite(CY, HIGH);
      digitalWrite(BR, HIGH);
      digitalWrite(AR, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // wait 1s
      // previous lane turns red
      digitalWrite(CR, HIGH);
      digitalWrite(CY, LOW);
      // current lane turns green
      digitalWrite(AG, HIGH);
      digitalWrite(AR, LOW);
      // Serial.println("current road A");
      break;
    case 1:
      blink_green(AG);
      turnOffAll();
      digitalWrite(AY, HIGH);
      digitalWrite(CR, HIGH); // keep c stop
      digitalWrite(BR, HIGH); // keep b stop first
      vTaskDelay(1000 / portTICK_PERIOD_MS); // yellow transition 1s
      digitalWrite(AY, LOW);
      digitalWrite(AR, HIGH);
      digitalWrite(BG, HIGH); // let b go
      digitalWrite(BR, LOW);
      // Serial.println("current road B");
      break;
    case 2:
      blink_green(BG);
      turnOffAll();
      digitalWrite(BY, HIGH);
      digitalWrite(AR, HIGH); // keep a stop
      digitalWrite(CR, HIGH); // keep c stop first
      vTaskDelay(1000 / portTICK_PERIOD_MS); // yellow transition 1s
      digitalWrite(BY, LOW);
      digitalWrite(BR, HIGH);
      digitalWrite(CG, HIGH); // let c go
      digitalWrite(CR, LOW);
      // Serial.println("current road C");
      break;
    default:
      Serial.println("program should not reach here");
      break;
  }
}

void turnOffAll() {
  for (int i = 0; i < 9; i++) {
    digitalWrite(allLedPins[i], LOW);
  }
}

void red_all() {
  turnOffAll();
  digitalWrite(AR, HIGH);
  digitalWrite(BR, HIGH);
  digitalWrite(CR, HIGH);
}

void offCR() {
  digitalWrite(CR, LOW);
}