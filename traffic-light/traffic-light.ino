#include "sensor.h"
#include "test_sensor.h"
#include "led.h"
#include "read_sensor.h"
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Wire.h>

QueueHandle_t carQueue;
TaskHandle_t trafficLightTaskHandle = NULL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.setClock(100000);

  setupFakeSensor();
  initLed();
  if(!vl53l0x_init()){
    Serial.println("fail init multiple sensor");
    while(1);
  } else {
    Serial.println("success init multiple sensor");
  }

  // start rtos tasks
  // tasks to count car
  xTaskCreate(countCarTaskTOF, "CountCarTOF", 128, NULL, 1, NULL);
  // transition of traffic light at the background
  xTaskCreate(trafficLightTask, "TrafficLightTask", 128, NULL, 1, &trafficLightTaskHandle);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}


