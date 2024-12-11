#include "Adafruit_VL53L0X.h"

#define XSHUT1 A0
#define XSHUT2 A1
#define XSHUT3 A2
#define XSHUT4 A3
#define XSHUT5 13
#define XSHUT6 12

Adafruit_VL53L0X* lox[6];

int xshut[6] = {XSHUT1, XSHUT2, XSHUT3, XSHUT4, XSHUT5, XSHUT6};
int firstI2cAddr = 0x30;

bool setupSensor() {
  for (int i = 0; i < 6; i++) {
    pinMode(xshut[i], OUTPUT); // set xshut pin as output
    digitalWrite(xshut[i], LOW); // turn off all sensor
  }

  delay(10);

  for (int i = 0; i < 6; i++) {
    digitalWrite(xshut[i], HIGH);
    delay(10);

    lox[i] = new Adafruit_VL53L0X();
    if (!lox[i]->begin(firstI2cAddr)) {
      Serial.println("Failed to initialise");
      //while(1);
    }
    Serial.print("initialise complete at 0x");
    Serial.println(firstI2cAddr, HEX);

    firstI2cAddr++;
  }
}

