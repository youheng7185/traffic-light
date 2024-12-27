#ifndef _LED_H
#define _LED_H

#include <Arduino_FreeRTOS.h>

void initLed();
void trafficLightTask(void *pvParameters);

extern TaskHandle_t trafficLightTaskHandle;

#endif