#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "types_utils.h"
#include "config_utils.h" // For robot configuration constants

/**
 * @brief Computes the wheel speeds required to achieve a desired robot velocity.
 *
 * This function calculates the individual wheel speeds necessary for the robot
 * to move with the specified linear and angular velocity.
 *
 * @param[in] v The desired robot velocity (linear and angular components).
 * @param[out] w Pointer to a WheelSpeeds structure where the computed wheel speeds will be stored.
 */
void compute_inverse_kinematics(Velocity v, WheelSpeeds *w);

/**
 * @brief Computes the robot velocity from given wheel speeds.
 *
 * This function calculates the resulting linear and angular velocity of the robot
 * based on the current wheel speeds.
 *
 * @param[in] w The current wheel speeds.
 * @param[out] v Pointer to a Velocity structure where the computed velocity will be stored.
 */
void compute_forward_kinematics(WheelSpeeds w, Velocity *v);

#endif // KINEMATICS_H
