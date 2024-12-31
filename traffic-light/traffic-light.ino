#include "sensor.h"
#include "led.h"
#include "read_sensor.h"
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Wire.h>

TaskHandle_t trafficLightTaskHandle = NULL;

void setup() {
  unsigned long startTime = millis();

  Serial.begin(115200);
  Wire.setClock(400000);
  initLed();
  red_all(); // red while init sensor
  // init sensor
  if(!vl53l0x_init()){
    Serial.println("fail init multiple sensor");
    while(1);
  } else {
    Serial.println("success init multiple sensor");
  }

  while (millis() - startTime < 1000); // "delay" 1s
  offCR(); // turn off red line on lane C

  // start rtos tasks
  // tasks to count car
  xTaskCreate(countCarTaskTOF, "CountCarTOF", 128, NULL, 1, NULL);
  // transition of traffic light at the background
  xTaskCreate(trafficLightTask, "TrafficLightTask", 128, NULL, 1, &trafficLightTaskHandle);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // imagine needing to use loop
}


