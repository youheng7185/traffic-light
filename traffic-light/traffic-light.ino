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
  //delay(1000);
  //Serial.print("done");
  //setupSensor();
  setupFakeSensor();
  initLed();
  if(!vl53l0x_init()){
    Serial.println("fail init multiple sensor");
    while(1);
  } else {
    Serial.println("success init multiple sensor");
  }

  /*
  // Create a queue with space for 2 items (for carA and carB)
  carQueue = xQueueCreate(2, sizeof(int));

  // Create the car counting task
  xTaskCreate(countCarTask, "CountCar", 128, NULL, 1, NULL);
  xTaskCreate(processCarCountTask, "ProcessCarCount", 128, NULL, 1, NULL);
  */

  xTaskCreate(countCarTaskTOF, "CountCarTOF", 128, NULL, 1, NULL);
  xTaskCreate(trafficLightTask, "TrafficLightTask", 128, NULL, 1, &trafficLightTaskHandle);
  //xTaskCreate(switchTraffic, "loop traffic light", 128, NULL, 1, NULL);


  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}


