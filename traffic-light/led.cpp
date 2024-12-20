#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "led.h"

#define AR 2
#define AY 3
#define AG 4

#define BR 5
#define BY 6
#define BG 7

#define CR 8
#define CY 9
#define CG 10

int allLedPins[9] = {AR, AY, AG, BR, BY, BG, CR, CY, CG};

void initLed();
void trafficLightTask(void *pvParameters);
bool green(int road);
bool turnOffAll();

void initLed() {
  for (int i = 0; i < 9; i++) {
    pinMode(allLedPins[i], OUTPUT);
    digitalWrite(allLedPins[i], LOW);
  }
}

void trafficLightTask(void *pvParameters) {
  int currentRoadGo = 2;
  while (1) {
    if (currentRoadGo != 2) {
      currentRoadGo++;
    } else {
      currentRoadGo = 0;
    }

    green(currentRoadGo);

    Serial.println("Suspending task after cycle");
    vTaskSuspend(NULL);  // Suspend this task until explicitly resumed
  }
}

bool green(int road) {
  switch(road) {
    case 0:
      turnOffAll();
      digitalWrite(BR, HIGH); // keep b stop
      digitalWrite(AR, HIGH); // keep a stop first
      digitalWrite(CY, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // yellow transition 1s
      digitalWrite(CY, LOW);
      digitalWrite(CR, HIGH);
      digitalWrite(AG, HIGH); // now let a go
      digitalWrite(AY, LOW);
      digitalWrite(AR, LOW);
      Serial.println("current road A");
      break;
    case 1:
      turnOffAll();
      digitalWrite(CR, HIGH); // keep c stop
      digitalWrite(BR, HIGH); // keep a stop first
      digitalWrite(AY, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // yellow transition 1s
      digitalWrite(AY, LOW);
      digitalWrite(AR, HIGH);
      digitalWrite(BG, HIGH); // let b go
      digitalWrite(BY, LOW);
      digitalWrite(BR, LOW);
      Serial.println("current road B");
      break;
    case 2:
      turnOffAll();
      digitalWrite(CR, HIGH); // keep b stop
      digitalWrite(AR, HIGH); // keep a stop first
      digitalWrite(BY, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // yellow transition 1s
      digitalWrite(BY, LOW);
      digitalWrite(BR, HIGH);
      digitalWrite(CG, HIGH); // let c go
      digitalWrite(CY, LOW);
      digitalWrite(CR, LOW);
      Serial.println("current road C");
      break;
    default:
      Serial.println("program should not reach here");
      break;
  }
}

bool turnOffAll() {
  for (int i = 0; i < 9; i++) {
    digitalWrite(allLedPins[i], LOW);
  }
}

bool toggleLight() {
  if (trafficLightTaskHandle != NULL) {
    Serial.println("Resuming traffic light task...");
    vTaskResume(trafficLightTaskHandle);
    return true;  // Task resumed successfully
  }
  Serial.println("Failed to resume task.");
  return false;  // Task handle is invalid
}

void switchTraffic(void *pvParameters) {
  Serial.println("switch traffic task initilised");
  while(1) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    toggleLight();
    Serial.println("traffic light toggle");
  }
}