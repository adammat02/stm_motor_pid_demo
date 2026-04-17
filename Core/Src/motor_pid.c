#include "motor_pid.h"
#include "main.h"
#include <stdio.h>

void motor_pid_init(motor_pid_t *motor_pid)
{
  motor_pid->pid.Kp = motor_pid->kp;
  motor_pid->pid.Ki = motor_pid->ki;
  motor_pid->pid.Kd = motor_pid->kd;

  if (motor_pid->ff_enable)
    motor_pid->ff_gain = (float32_t)motor_pid->motor->max_pwm / (float32_t)motor_pid->max_rpm;

  motor_pid->last_time = 0;
  motor_pid->prev_rot = 0.0f;
  motor_pid->rpm_filtered = 0.0f;
  arm_pid_init_f32(&motor_pid->pid, 1);
}


void motor_pid_update(motor_pid_t *motor_pid, float32_t set_rpm, RotationDirection_t dir)
{
  uint32_t time = micros();
  uint32_t dt = time - motor_pid->last_time;
  motor_pid->last_time = time;
  if (dt == 0)
    return;

  float rot = encoder_get_rotations(motor_pid->encoder);
  float rpm = (rot - motor_pid->prev_rot) / (float)dt * 60000000.0f;
  motor_pid->prev_rot = rot;

  motor_pid->rpm_filtered = motor_pid->alpha * rpm + (1.0f - motor_pid->alpha) * motor_pid->rpm_filtered;

  float32_t error = set_rpm - motor_pid->rpm_filtered;
  float32_t pid_out = arm_pid_f32(&motor_pid->pid, error);

  float32_t pwm_ff = set_rpm * motor_pid->ff_gain;
  float32_t pwm = pwm_ff + pid_out;

  if (pwm > motor_pid->motor->max_pwm)
  {
    pwm = motor_pid->motor->max_pwm;
    pid_out = pwm - pwm_ff;
    motor_pid->pid.state[2] = pid_out;
  }
  else if (pwm < 0.0f)
  {
    pwm = 0.0f;
    pid_out = -pwm_ff;
    motor_pid->pid.state[2] = pid_out;
  }

  motor_set_pwm(motor_pid->motor, (uint32_t)pwm, dir);
}