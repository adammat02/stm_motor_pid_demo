#include "encoder.h"

static const int16_t safe_factor = 10;

void encoder_init(encoder_t *encoder)
{
  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  encoder->state = 0;
  encoder->rotations = .0f;
}

static int16_t encoder_get_diff(encoder_t *encoder)
{
  int16_t prev_state = encoder->state;
  encoder->state = __HAL_TIM_GET_COUNTER(encoder->htim);
  int16_t diff = encoder->state - prev_state;

  int16_t max = encoder->per_rev * safe_factor;
  if (encoder->state >= max || encoder->state <= -max)
  {
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->state = 0;
  }

  return diff;
}

float encoder_get_rotations(encoder_t *encoder)
{
  int16_t diff = encoder_get_diff(encoder);

  encoder->rotations += (float)diff / (float)encoder->per_rev;
  return encoder->rotations;
}

void encoder_reset(encoder_t *encoder)
{
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  encoder->state = 0;
  encoder->rotations = 0;
}
