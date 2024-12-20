#ifndef _LED_H
#define _LED_H

#include <Arduino_FreeRTOS.h>

void initLed();
bool turnOffAll();
void trafficLightTask(void *pvParameters);
bool green(int);
bool toggleLight();
void switchTraffic(void *pvParameters);

extern TaskHandle_t trafficLightTaskHandle;

#endif