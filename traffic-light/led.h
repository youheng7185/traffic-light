#ifndef _LED_H
#define _LED_H

bool turnOffAll();
void trafficLightTask(void *pvParameters);
bool green(int);
bool toggleLight();

extern TaskHandle_t trafficLightTaskHandle;

#endif