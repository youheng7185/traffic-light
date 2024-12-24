#include <Arduino.h>
#include "test_sensor.h"
#include <Arduino_FreeRTOS.h>
#include <queue.h>
// these function are used to simulate the sensor data with infrared sensor

void setupFakeSensor() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

int a1, a2, b1, b2;

void readFakeData() {
  a1 = digitalRead(A0);
  a2 = digitalRead(A1);
  b1 = digitalRead(A2);
  b2 = digitalRead(A3);
  Serial.print("a1=");
  Serial.print(a1);
  Serial.print("a2=");
  Serial.print(a2);
  Serial.print("b1=");
  Serial.print(b1);
  Serial.print("b2=");
  Serial.println(b2);
}

int prevA, prevB;
int lastStateA1, currentStateA1;
int lastStateA2, currentStateA2;
int lastStateB1, currentStateB1;
int lastStateB2, currentStateB2;

int carA = 0, carB = 0;

void countCar() {
  currentStateA1 = digitalRead(A0);
  if(currentStateA1 == 0 && lastStateA1 == 1){
    carA++;
    Serial.println("car added");
  }

  currentStateA2 = digitalRead(A1);
  if(currentStateA2 == 0 && lastStateA2 == 1){
    carA--;
    Serial.println("car gone");
  }

  lastStateA1 = currentStateA1;
  lastStateA2 = currentStateA2;

  if(prevA != carA)
  {
    Serial.print("current car at A: ");
    Serial.println(carA);
  }

  currentStateB1 = digitalRead(A2);
  if(currentStateB1 == 0 && lastStateB1 == 1){
    carB++;
    Serial.println("car added");
  }

  currentStateB2 = digitalRead(A3);
  if(currentStateB2 == 0 && lastStateB2 == 1){
    carB--;
    Serial.println("car gone");
  }

  lastStateB1 = currentStateB1;
  lastStateB2 = currentStateB2;

  if(prevB != carB)
  {
    Serial.print("current car at B: ");
    Serial.println(carB);
  }
  
  prevA = carA;
  prevB = carB;
}

void countCarTask(void *pvParameters) {
  int currentStateA1, currentStateA2;
  int currentStateB1, currentStateB2;

  while (1) {
    // Read inputs for A
    currentStateA1 = digitalRead(A0);
    if (currentStateA1 == 0 && lastStateA1 == 1) {  // Rising edge (car added)
      carA++;
      Serial.println("Car A added");
      xQueueSend(carQueue, &carA, portMAX_DELAY);
    }

    currentStateA2 = digitalRead(A1);
    if (currentStateA2 == 0 && lastStateA2 == 1) {  // Rising edge (car gone)
      carA--;
      Serial.println("Car A gone");
      xQueueSend(carQueue, &carA, portMAX_DELAY);
    }

    // Update last states
    lastStateA1 = currentStateA1;
    lastStateA2 = currentStateA2;

    // Read inputs for B
    currentStateB1 = digitalRead(A2);
    if (currentStateB1 == 0 && lastStateB1 == 1) {  // Rising edge (car added)
      carB++;
      Serial.println("Car B added");
      xQueueSend(carQueue, &carB, portMAX_DELAY);
    }

    currentStateB2 = digitalRead(A3);
    if (currentStateB2 == 0 && lastStateB2 == 1) {  // Rising edge (car gone)
      carB--;
      Serial.println("Car B gone");
      xQueueSend(carQueue, &carB, portMAX_DELAY);
    }

    // Update last states
    lastStateB1 = currentStateB1;
    lastStateB2 = currentStateB2;

    // Delay to prevent excessive reads
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay as necessary
  }
}

void processCarCountTask(void *pvParameters) {
  int carCount;
  
  while (1) {
    // Check if there is any data in the queue
    if (xQueueReceive(carQueue, &carCount, portMAX_DELAY)) {
      // Print the current count of cars in A and B
      if (prevA != carA) {
        Serial.print("Current car count at A: ");
        Serial.println(carA);
        prevA = carA;
      }
      if (prevB != carB) {
        Serial.print("Current car count at B: ");
        Serial.println(carB);
        prevB = carB;
      }
    }

    // Delay to prevent the task from running too fast
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

