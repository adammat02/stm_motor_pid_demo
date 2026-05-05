#ifndef TOF_VL53L0X_H
#define TOF_VL53L0X_H

#include "vl53l0x_api.h"
#include "i2c.h"

/** VL53L0X sensor handle. */
typedef struct
{
  VL53L0X_Dev_t dev;                             /**< VL53L0X device structure. */
  VL53L0X_RangingMeasurementData_t ranging_data; /**< Ranging measurement data. */
  I2C_HandleTypeDef *hi2c;                       /**< I2C peripheral handle. */
  uint8_t i2c_addr;                              /**< I2C address of the sensor. */
  uint16_t distance;                             /**< Last valid distance reading [mm]. */
  float cf_distance;                             /**< Complementary-filtered distance [mm]. */
} tof_t;

/**
 * @brief Initialize the VL53L0X sensor.
 * @param tof  Pointer to sensor handle.
 */
void tof_init(tof_t *tof);

/**
 * @brief Return the most recently measured distance in millimetres.
 * @param tof  Pointer to sensor handle.
 * @return Distance in mm, or TOF_MAX_DIST if the measurement is invalid.
 */
uint16_t tof_get_distance(tof_t *tof);

#endif // TOF_VL53L0X_H
