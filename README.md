
# Intelligent Traffic Light Control System

A brief description of what this project does and who it's for


## Usage/Examples

This program is written for a T-junction with junction A, B and C.


Switch the side to green light with a function call:
```
bool toggleLight();
```

Get the number of car in each junction:

Refer processCarCountTask function, and use

```
xQueueReceive(carQueue, &carCount, portMAX_DELAY);
```






## Deployment

To deploy this project, open in Arduino IDE and install library below:

* FreeRTOS by Richard Barry

* Adafruit_VL53L0X

