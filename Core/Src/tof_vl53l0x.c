#include "tof_vl53l0x.h"

#define TOF_MAX_DIST 2000

void tof_init(tof_t *tof)
{
  tof->dev.I2cHandle = tof->hi2c;
  tof->dev.I2cDevAddr = tof->i2c_addr;

  VL53L0X_WaitDeviceBooted(&tof->dev);
  VL53L0X_DataInit(&tof->dev);
  VL53L0X_StaticInit(&tof->dev);

  uint32_t ref_spad_count;
  uint8_t is_aperture_spads;
  uint8_t vhv_settings;
  uint8_t phase_cal;

  VL53L0X_PerformRefCalibration(&tof->dev, &vhv_settings, &phase_cal);
  VL53L0X_PerformRefSpadManagement(&tof->dev, &ref_spad_count, &is_aperture_spads);
  VL53L0X_SetDeviceMode(&tof->dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

  VL53L0X_SetLimitCheckEnable(&tof->dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  VL53L0X_SetLimitCheckEnable(&tof->dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  VL53L0X_SetLimitCheckValue(&tof->dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.02 * 65536));
  VL53L0X_SetLimitCheckValue(&tof->dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
  VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tof->dev, 10000);
  VL53L0X_SetVcselPulsePeriod(&tof->dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  VL53L0X_SetVcselPulsePeriod(&tof->dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  VL53L0X_StartMeasurement(&tof->dev);

  tof->distance = TOF_MAX_DIST;
}

uint16_t tof_get_distance(tof_t *tof)
{
  uint8_t ready = 0;
  VL53L0X_GetMeasurementDataReady(&tof->dev, &ready);
  if (!ready)
    return tof->distance;

  VL53L0X_GetRangingMeasurementData(&tof->dev, &tof->ranging_data);
  VL53L0X_ClearInterruptMask(&tof->dev, 0);

  if (tof->ranging_data.RangeStatus == 0 && tof->ranging_data.RangeMilliMeter <= TOF_MAX_DIST)
    tof->distance = tof->ranging_data.RangeMilliMeter;

  return tof->distance;
}
