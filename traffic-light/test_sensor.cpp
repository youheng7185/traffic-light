#include <Arduino.h>

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

int carA = 0, carB = 0;
int prevA, prevB;
int lastStateA1, currentStateA1;
int lastStateA2, currentStateA2;
int lastStateB1, currentStateB1;
int lastStateB2, currentStateB2;

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