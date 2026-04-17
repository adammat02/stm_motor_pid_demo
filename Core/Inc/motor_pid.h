/**
 * @file motor_pid.h
 * @brief Closed-loop RPM controller for a DC motor.
 *
 * Combines a CMSIS-DSP PID with optional feed-forward and an EMA low-pass
 * filter on the speed measurement. Anti-windup is applied by clamping the
 * PID internal state when the PWM output saturates.
 *
 * Typical usage:
 * @code
 *   motor_pid_t ctrl = {
 *     .motor     = &my_motor,
 *     .encoder   = &my_enc,
 *     .max_rpm   = 3000,
 *     .ff_enable = 1,
 *     .kp = 0.5f, .ki = 0.1f, .kd = 0.01f,
 *     .alpha = 0.2f,   // EMA coefficient (0 = max filtering, 1 = none)
 *   };
 *   motor_pid_init(&ctrl);
 *
 *   // in control loop:
 *   motor_pid_update(&ctrl, 1500.0f, FORWARD);
 * @endcode
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include "arm_math.h"
#include "motor_driver.h"
#include "encoder.h"

typedef struct
{
  motor_t               *motor;
  encoder_t             *encoder;
  arm_pid_instance_f32   pid;
  int16_t                max_rpm;
  uint8_t                ff_enable;
  float                  prev_rot;
  uint32_t               last_time;
  float32_t              ff_gain;       /**< Computed in init: max_pwm / max_rpm */
  float32_t              kp;
  float32_t              ki;
  float32_t              kd;
  float32_t              alpha;         /**< EMA coefficient: 0.0 = max damping, 1.0 = no filter */
  float32_t              rpm_filtered;  /**< Last filtered RPM reading */
} motor_pid_t;

/**
 * @brief Initialise PID gains, feed-forward gain and internal state.
 *
 * Must be called once before the first motor_pid_update(). Reads kp/ki/kd
 * and ff_enable from the struct; all other state is zeroed.
 *
 * @param motor_pid  Pointer to a caller-allocated motor_pid_t.
 */
void motor_pid_init(motor_pid_t *motor_pid);

/**
 * @brief Run one PID iteration and update motor PWM.
 *
 * Measures elapsed time with micros(), computes RPM from encoder delta,
 * applies EMA filtering, feeds the error into the CMSIS-DSP PID, adds
 * feed-forward, clamps to [0, max_pwm] with anti-windup, then calls
 * motor_set_pwm().
 *
 * @param motor_pid  Controller instance.
 * @param set_rpm    Target speed in RPM.
 * @param dir        Rotation direction passed to motor_set_pwm().
 */
void motor_pid_update(motor_pid_t *motor_pid, float32_t set_rpm, RotationDirection_t dir);

/**
 * @brief Update PID gains at runtime without resetting the controller state.
 *
 * Recalculates the internal CMSIS-DSP coefficients (A0/A1/A2) from the new
 * gains. The integrator state is preserved, so the output remains bumpless
 * for small gain changes.
 *
 * @param motor_pid  Controller instance.
 * @param kp         Proportional gain.
 * @param ki         Integral gain.
 * @param kd         Derivative gain.
 */
void motor_pid_set_pid(motor_pid_t *motor_pid, float32_t kp, float32_t ki, float32_t kd);

#endif
