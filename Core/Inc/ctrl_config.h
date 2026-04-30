#ifndef CTRL_CONFIG_H
#define CTRL_CONFIG_H

#include "motor_driver.h"
#include "encoder.h"
#include "motor_pid.h"

#define N_MOTORS 4
#define PER_REV 1940
#define MAX_RPM 160
#define KP 3.0f
#define KI 0.5f
#define KD 0.2f
#define ALPHA 0.5f
#define CMD_TIMEOUT_US 500000

static motor_t motors[N_MOTORS] = {
  { .htim = &htim8, .channel = TIM_CHANNEL_1, .dir_port = M1_DIR_GPIO_Port, .dir_pin = M1_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_2, .dir_port = M2_DIR_GPIO_Port, .dir_pin = M2_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_3, .dir_port = M3_DIR_GPIO_Port, .dir_pin = M3_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_4, .dir_port = M4_DIR_GPIO_Port, .dir_pin = M4_DIR_Pin, .max_pwm = TIM8_ARR }
};

static encoder_t encoders[N_MOTORS] = {
  { .htim = &htim1, .per_rev = PER_REV },
  { .htim = &htim2, .per_rev = PER_REV },
  { .htim = &htim3, .per_rev = PER_REV },
  { .htim = &htim4, .per_rev = PER_REV }
};

static motor_pid_t motor_pids[N_MOTORS] = {
  { .motor = &motors[0], .encoder = &encoders[0], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[1], .encoder = &encoders[1], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[2], .encoder = &encoders[2], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[3], .encoder = &encoders[3], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA }
};

static const int8_t motor_sign[N_MOTORS] = { 1, -1, 1, -1};

#endif