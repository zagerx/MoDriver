/* SPDX-License-Identifier: GPL-2.0 */

/**
 * @file close_loop.h
 * @brief Motor closed-loop control header file
 */

#ifndef CLOSE_LOOP_H
#define CLOSE_LOOP_H

struct motor;

/**
 * @brief Execute motor velocity loop control
 * @param motor Pointer to motor structure
 */
void motor_velocity_loop(struct motor *motor);
void motor_currment_loop(struct motor *motor);

#endif /* CLOSE_LOOP_H */
