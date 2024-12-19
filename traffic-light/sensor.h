#ifndef SENSOR_H
#define SENSOR_H

#define VL53L0X_OUT_OF_RANGE (8190)

// single sensor
bool setupSensor();
// multiple sensor
bool vl53l0x_init();

// add sensor at init address function, configure gpio

typedef enum
{
    VL53L0X_IDX_FIRST,
    VL53L0X_IDX_SECOND,
    VL53L0X_IDX_THIRD,
    VL53L0X_IDX_FOURTH,
    VL53L0X_IDX_FIFTH,
    VL53L0X_IDX_SIXTH
} vl53l0x_idx_t;

bool vl53l0x_read_range_single(vl53l0x_idx_t idx, uint16_t &range);

#endif