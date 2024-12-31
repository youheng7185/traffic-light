#ifndef _LED_H
#define _LED_H

#include <Arduino_FreeRTOS.h>

void initLed();
void trafficLightTask(void *pvParameters);
void red_all();
void offCR();

extern TaskHandle_t trafficLightTaskHandle;

#endif