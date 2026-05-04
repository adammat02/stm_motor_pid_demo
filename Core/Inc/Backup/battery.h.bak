/**
 * @file battery.h
 * @brief Battery voltage measurement via ADC triggered by a periodic timer.
 *
 * The ADC is triggered by a hardware timer (htim) at a fixed rate. Each
 * conversion fires an interrupt; the caller must forward it by calling
 * battery_measure_callback() from HAL_ADC_ConvCpltCallback.
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "tim.h"
#include "adc.h"

/** Battery measurement handle. */
typedef struct battery
{
  ADC_HandleTypeDef *hadc;  /**< ADC peripheral used for measurement. */
  TIM_HandleTypeDef *htim;  /**< Timer that triggers ADC conversions. */
  volatile float vbat;      /**< Last computed battery voltage [V]. */
  float vref;               /**< ADC reference voltage [V]. */
  float adc_max;            /**< Maximum ADC raw value (2^resolution - 1). */
  float div_ratio;          /**< Voltage divider ratio (V_bat / V_adc). */
} battery_t;

/**
 * @brief Calibrate ADC, start interrupt-driven conversions, and start the trigger timer.
 *        Must be called once before any measurement.
 * @param battery  Pointer to battery handle.
 */
void battery_measure_init(battery_t *battery);

/**
 * @brief Process a completed ADC conversion and update battery->vbat.
 *        Call from HAL_ADC_ConvCpltCallback.
 * @param battery  Pointer to battery handle whose hadc fired the interrupt.
 */
void battery_measure_callback(battery_t *battery);

/**
 * @brief Return the most recently measured battery voltage in volts.
 * @param battery  Pointer to battery handle.
 * @return Battery voltage [V], or 0.0 if no conversion has completed yet.
 */
float battery_get_voltage(battery_t *battery);

#endif