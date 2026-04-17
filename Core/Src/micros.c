#include "micros.h"

static TIM_HandleTypeDef *timer;

void micros_tim_init(TIM_HandleTypeDef *htim)
{
  HAL_TIM_Base_Start(htim);
  timer = htim;
}

uint32_t micros(void)
{
  return __HAL_TIM_GET_COUNTER(timer);
}