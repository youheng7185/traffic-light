#ifndef _LED_H
#define _LED_H

void initLed();
bool turnOffAll();
void trafficLightTask(void *pvParameters);
bool green(int);
bool toggleLight();
void switchTraffic(void *pvParameters);

extern TaskHandle_t trafficLightTaskHandle;

#endif