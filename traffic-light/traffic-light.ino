#include "sensor.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  Serial.print("done");
  setupSensor();

}
void loop() {
  // put your main code here, to run repeatedly:
  fetchData();
}

