#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Number of sensors
#define SENSOR_COUNT 6

// Define the GPIO pins for XSHUT
int xshutPins[SENSOR_COUNT] = {A0, A1, A2, A3, 13, 12};

// Unique I2C addresses for each sensor
uint8_t sensorAddresses[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};

// Single VL53L0X object
Adafruit_VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial Monitor
  
  Serial.println("Initializing VL53L0X sensors...");

  // Set up XSHUT pins and turn off all sensors
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW); // Put the sensor in shutdown mode
  }
  delay(10);

  // Initialize each sensor with a unique I2C address
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Activate the current sensor
    digitalWrite(xshutPins[i], HIGH);
    delay(10);

    // Initialize the sensor
    if (!sensor.begin(0x29)) { // Default I2C address
      Serial.print("Failed to initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Set the new I2C address
    if (!sensor.setAddress(sensorAddresses[i])) {
      Serial.print("Failed to set I2C address for sensor ");
      Serial.println(i);
      while (1);
    }

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" initialized with address 0x");
    Serial.println(sensorAddresses[i], HEX);

    // Deactivate the sensor again
    digitalWrite(xshutPins[i], LOW);
  }

  // Reactivate all sensors
  for (int i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(10);
  }
}

void loop() {
  // Read and print the distance measurements from all sensors dynamically
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Activate the current sensor
    for (int j = 0; j < SENSOR_COUNT; j++) {
      digitalWrite(xshutPins[j], (j == i) ? HIGH : LOW);
    }
    delay(10);

    // Initialize the sensor
    if (!sensor.begin(sensorAddresses[i])) {
      Serial.print("Failed to initialize sensor ");
      Serial.println(i);
      continue;
    }

    // Read the distance
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) { // 4 means out of range
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" (0x");
      Serial.print(sensorAddresses[i], HEX);
      Serial.print("): ");
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(": Out of range");
    }
  }

  delay(500); // Adjust based on your application
}
