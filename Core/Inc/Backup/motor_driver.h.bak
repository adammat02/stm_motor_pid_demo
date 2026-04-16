/**
 * @file motor_driver.h
 * @brief DC motor driver interface using PWM and GPIO direction control.
 *
 * Provides initialization, PWM speed control, and stop functionality
 * for a single DC motor driven by a PWM-capable timer channel and a
 * direction GPIO pin.
 */

#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include "gpio.h"
#include "tim.h"

/**
 * @brief Motor handle containing hardware configuration.
 *
 * Fill this structure before calling motor_init(). The caller owns
 * the memory; the driver only stores the pointer.
 */
typedef struct
{
  TIM_HandleTypeDef *htim;   /**< Timer used to generate the PWM signal.      */
  uint32_t channel;          /**< Timer channel (e.g. TIM_CHANNEL_1).         */
  GPIO_TypeDef *dir_port;    /**< GPIO port for the direction control pin.    */
  uint16_t dir_pin;          /**< GPIO pin number for direction control.      */
  uint32_t max_pwm;          /**< Timer auto-reload value (100 % duty cycle). */
} motor_t;

/**
 * @brief Motor rotation direction.
 */
typedef enum
{
  ROTATION_CW,        /**< Clockwise rotation.         */
  ROTATION_CCW        /**< Counter-clockwise rotation. */
} RotationDirection_t;


/**
 * @brief Initialize the motor and start the PWM peripheral.
 *
 * Starts the timer PWM output at 0 % duty cycle.
 * Must be called once before motor_set_pwm() or motor_stop().
 *
 * @param motor  Pointer to a fully populated motor_t handle.
 */
void motor_init(motor_t *motor);

/**
 * @brief Set motor speed and direction.
 *
 * @param motor  Pointer to an initialized motor_t handle.
 * @param value  PWM compare value in the range [0, max_pwm].
 *               0 → stopped, max_pwm → full speed.
 * @param dir    Desired rotation direction.
 */
void motor_set_pwm(motor_t *motor, uint32_t value, RotationDirection_t dir);

/**
 * @brief Stop the motor immediately (0 % duty cycle).
 *
 * @param motor  Pointer to an initialized motor_t handle.
 */
void motor_stop(motor_t *motor);


#endif