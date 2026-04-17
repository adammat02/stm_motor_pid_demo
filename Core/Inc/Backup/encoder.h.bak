#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"

/**
 * @brief Quadrature encoder handle.
 *
 * @param htim      Pointer to the timer configured in encoder mode.
 * @param per_rev   Number of timer counts per full shaft revolution.
 * @param state     Last captured counter value (uint16_t wrap-around safe).
 * @param rotations Accumulated rotation count (positive = forward, negative = reverse).
 */
typedef struct
{
  TIM_HandleTypeDef *htim;
  int16_t per_rev;
  int16_t state;
  float rotations;
} encoder_t;

/**
 * @brief Initialize the encoder and start the timer in encoder mode.
 *        Resets the hardware counter and clears the internal state.
 *
 * @param encoder Pointer to encoder handle.
 */
void encoder_init(encoder_t *encoder);

/**
 * @brief Compute the accumulated shaft rotations since initialization.
 *        Must be called frequently enough so the counter does not advance
 *        more than 32767 counts between calls.
 *
 * @param encoder Pointer to encoder handle.
 * @return Rotation count as a float (fractional rotations included).
 */
float encoder_get_rotations(encoder_t *encoder);

#endif
