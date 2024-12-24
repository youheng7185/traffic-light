#ifndef _TEST_SENSOR_H
#define _TEST_SENSOR_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>

void setupFakeSensor();
void readFakeData();
void countCar();
void countCarTask(void *pvParameters);
void processCarCountTask(void *pvParameters);

extern int carA, carB;
extern QueueHandle_t carQueue;

#endif