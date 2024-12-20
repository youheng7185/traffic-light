
# Intelligent Traffic Light Control System

A brief description of what this project does and who it's for


## Usage/Examples

This program is written for a T-junction with junction A, B and C.


Switch the side to green light with a function call:
```
vTaskResume(trafficLightTaskHandle);
```

## Todos:

vTaskResume shouldnt be called when the traffic light is transitioning

Create new task to write traffic light algorithm or just write on the countCarTaskTOF

value range is the distance measured by the sensor in millimeter




## Deployment

To deploy this project, open in Arduino IDE and install library below:

* FreeRTOS by Richard Barry

## Pinout

#### VL53L0X

##### Junction A
| Sensor Pin | Arduino Pin     |
| :-------- | :------- |
| `XSHUT1` | `A0` |
| `XSHUT2` | `A1` |

#### Junction B
| Sensor Pin | Arduino Pin     |
| :-------- | :------- |
| `XSHUT3` | `A2` |
| `XSHUT4` | `A3` |

#### Junction C
| Sensor Pin | Arduino Pin     |
| :-------- | :------- |
| `XSHUT5` | `13` |
| `XSHUT6` | `12` |

#### Junction A LED

| Sensor Pin | Arduino Pin     |
| :-------- | :------- |
| `RED` | `2` |
| `YELLOW` | `3` |
| `GREEN` | `4` |

#### Junction B LED

| Sensor Pin | Arduino Pin     |
| :-------- | :------- |
| `RED` | `5` |
| `YELLOW` | `6` |
| `GREEN` | `7` |

#### Junction C LED

| Sensor Pin | Arduino Pin     |
| :-------- | :------- |
| `RED` | `8` |
| `YELLOW` | `9` |
| `GREEN` | `10` |
