#include "sensor.h"
#include <Arduino_FreeRTOS.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  Serial.print("done");
  setupSensor();
}

void loop() {
  // put your main code here, to run repeatedly:
  pinMode(LED_BUILTIN, OUTPUT);
}

void testTask(void *pvParameters) {
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(1000); 
  }
}
