#include "motor_driver.h"

static void clamp_pwm(uint32_t *value, uint32_t max_value)
{
  *value = ((*value <= max_value) ? *value : max_value);
}

void motor_init(motor_t *motor)
{
  HAL_TIM_PWM_Start(motor->htim, motor->channel);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}

void motor_set_pwm(motor_t *motor, uint32_t value, RotationDirection_t dir)
{
  GPIO_PinState state = ((dir == ROTATION_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  clamp_pwm(&value, motor->max_pwm);

  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, state);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, value);
}

void motor_stop(motor_t *motor)
{
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}