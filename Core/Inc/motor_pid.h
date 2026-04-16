#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include "arm_math.h"
#include "motor_driver.h"
#include "encoder.h"

typedef struct
{
  motor_t *motor;
  encoder_t *encoder;
  arm_pid_instance_f32 pid;
  int16_t max_rpm;
  float prev_rot;
  uint32_t last_time;
  float32_t ff_gain;
  float32_t kp;
  float32_t ki;
  float32_t kd;
} motor_pid_t;

void motor_pid_init(motor_pid_t *motor_pid);

void motor_pid_update(motor_pid_t *motor_pid, float32_t set_speed, RotationDirection_t dir);

#endif