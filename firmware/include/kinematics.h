/**
 * @file kinematics.h
 * @brief Forward and inverse kinematics for omnidirectional robot
 * 
 * This module provides kinematic transformations between robot velocity
 * (vx, vy, wz) and individual wheel angular velocities for a three-wheeled
 * omnidirectional drive system.
 * 
 * Kinematic model:
 * - Inverse kinematics: Converts desired robot velocity to wheel speeds
 * - Forward kinematics: Converts measured wheel speeds to robot velocity
 * 
 * The robot uses three omniwheels arranged at specific angles around the
 * chassis. Configuration parameters (wheel radius, body radius, wheel angles)
 * are defined in config_utils.h.
 * 
 * Thread-safety: Functions are thread-safe and reentrant (no shared state).
 * 
 * @note All identifiers follow snake_case naming convention
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "types_utils.h"
#include "config_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute wheel speeds from desired robot velocity (inverse kinematics)
 *
 * Transforms robot velocity in Cartesian coordinates (vx, vy, wz) into
 * individual wheel angular velocities (phi_dot) using the inverse kinematic
 * equations for a three-wheeled omnidirectional robot.
 *
 * Equation for wheel i:
 * φ̇ᵢ = (-sin(αᵢ)*vx + cos(αᵢ)*vy + L*wz) / r
 * 
 * where:
 * - αᵢ: wheel orientation angle
 * - L: robot body radius (distance from center to wheel)
 * - r: wheel radius
 *
 * @param[in] v Desired robot velocity (vx, vy, wz)
 * @param[out] w Pointer to structure for computed wheel speeds
 * 
 * Thread-safety: Thread-safe and reentrant
 */
void compute_inverse_kinematics(velocity_t v, wheel_speeds_t *w);

/**
 * @brief Compute robot velocity from measured wheel speeds (forward kinematics)
 *
 * Transforms measured wheel angular velocities into robot velocity using
 * the pseudo-inverse of the kinematic Jacobian matrix. This provides the
 * least-squares best estimate of robot velocity given wheel measurements.
 *
 * Uses the equation:
 * [vx, vy, wz]ᵀ = A⁺ * [φ̇₁, φ̇₂, φ̇₃]ᵀ
 * 
 * where A⁺ is the Moore-Penrose pseudo-inverse: A⁺ = (AᵀA)⁻¹Aᵀ
 *
 * @param[in] w Current wheel speeds
 * @param[out] v Pointer to structure for computed robot velocity
 * 
 * Thread-safety: Thread-safe after first call (uses lazy initialization)
 * 
 * @note First call performs one-time matrix computation. Subsequent calls
 *       use cached result.
 */
void compute_forward_kinematics(wheel_speeds_t w, velocity_t *v);

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_H
