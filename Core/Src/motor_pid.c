#include "motor_pid.h"
#include <stdio.h>


void motor_pid_init(motor_pid_t *motor_pid)
{
  motor_pid->pid.Kp = motor_pid->kp;
  motor_pid->pid.Ki = motor_pid->ki;
  motor_pid->pid.Kd = motor_pid->kd;
  motor_pid->ff_gain = (float32_t)motor_pid->motor->max_pwm / (float32_t)motor_pid->max_rpm;
  motor_pid->last_time = 0;
  motor_pid->prev_rot = .0f;
  arm_pid_init_f32(&motor_pid->pid, 1);
}

void motor_pid_update(motor_pid_t *motor_pid, float32_t set_speed, RotationDirection_t dir)
{
  uint32_t time = HAL_GetTick();
  uint32_t dt = time - motor_pid->last_time;
  motor_pid->last_time = time;
  if (dt == 0) return;

  float rot = encoder_get_rotations(motor_pid->encoder);
  float speed = (rot - motor_pid->prev_rot) / (float)dt * 60000.0;
  motor_pid->prev_rot = rot;

  float32_t error = set_speed - (float32_t)speed;
  float32_t pid_out = arm_pid_f32(&motor_pid->pid, error);

  float32_t pwm_ff = set_speed * motor_pid->ff_gain;
  float32_t pwm = pwm_ff + pid_out;

  printf(">speed:%f,set_speed:%f,pwm:%f\r\n", speed, set_speed, pwm);

  motor_set_pwm(motor_pid->motor, (uint32_t)pwm, dir);
}