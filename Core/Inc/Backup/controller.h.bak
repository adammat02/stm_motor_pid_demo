/**
 * @file controller.h
 * @brief High-level robot controller.
 *
 * Ties together the motor drivers, encoders, PID loops, and UART command
 * interface. The caller is expected to invoke controller_poll() on every
 * iteration of the main loop and controller_update() at a fixed control rate.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>

/**
 * @brief Initialize motors, encoders, and PID controllers.
 *        Must be called once before poll or update.
 * @param debug  When true, RX/TX traffic is printed to the debug serial (UART2).
 */
void controller_init(bool debug);

/**
 * @brief Check for a new UART command and execute it if available.
 *        Non-blocking — returns immediately when no data is ready.
 */
void controller_poll(void);

/**
 * @brief Run one PID update step for all motors.
 *        Call at a fixed, consistent rate (e.g. from a timer interrupt or RTOS task).
 */
void controller_update(void);

#endif
