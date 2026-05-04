#include "battery.h"

static volatile uint16_t adc_raw = 0;

void battery_measure_init(battery_t *battery)
{
  HAL_ADCEx_Calibration_Start(battery->hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_IT(battery->hadc);
  HAL_TIM_Base_Start(battery->htim);
}

void battery_measure_callback(battery_t *battery)
{
  adc_raw = HAL_ADC_GetValue(battery->hadc);
  battery->vbat = (adc_raw * battery->vref / battery->adc_max) * battery->div_ratio;
}

float battery_get_voltage(battery_t *battery)
{
  return battery->vbat;
}