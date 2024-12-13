#include "sensor.h"
#include "test_sensor.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  Serial.print("done");
  //setupSensor();
  setupFakeSensor();

}
void loop() {
  // put your main code here, to run repeatedly:
  //fetchData();
  //readFakeData();
  countCar();
  delay(100);
}

