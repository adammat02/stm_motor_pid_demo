/**
 * @file micros.h
 * @brief Microsecond timer using a hardware TIM peripheral.
 *
 * Call micros_tim_init() once at startup with a timer configured for 1 MHz
 * (prescaler = F_CPU/1000000 - 1). After that, micros() returns the current
 * counter value which wraps at the timer's ARR period.
 */

#ifndef MICROS_H
#define MICROS_H

#include "tim.h"

/**
 * @brief Bind a hardware timer to the microsecond counter.
 * @param htim  Timer handle configured at 1 MHz tick rate.
 */
void micros_tim_init(TIM_HandleTypeDef *htim);

/**
 * @brief Return current microsecond timestamp.
 * @return Counter value in microseconds (wraps at ARR).
 */
uint32_t micros(void);

#endif
