/**
 * @file config_utils.h
 * @brief System-wide configuration constants for RoboCup Goalkeeper firmware
 * 
 * This header centralizes all configuration parameters for the robot including:
 * - Physical robot dimensions
 * - Motor control parameters
 * - PID controller tuning
 * - Task timing and priorities
 * - Sensor filtering parameters
 * 
 * All constants follow SCREAMING_SNAKE_CASE naming convention for macros/defines.
 * 
 * @note Update these values when calibrating or modifying hardware
 */

#ifndef CONFIG_UTILS_H
#define CONFIG_UTILS_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// ROBOT IDENTIFICATION
// =============================================================================

#define ROBOT_NAME "RoboCupGoalkeeper"     ///< Robot name identifier
#define ROBOT_VERSION "2.0"                ///< Firmware version string

// =============================================================================
// ROBOT PHYSICAL DIMENSIONS (meters and radians)
// =============================================================================

#define ROBOT_BODY_RADIUS 0.08f            ///< Robot chassis radius (m)
#define ROBOT_WHEEL_RADIUS 0.03f           ///< Wheel radius (m)
#define ROBOT_WHEEL_0_OFFSET (M_PI / 6.0f)     ///< Wheel 0 angular offset (rad) - 30°
#define ROBOT_WHEEL_1_OFFSET (5.0f * M_PI / 6.0f)  ///< Wheel 1 angular offset (rad) - 150°
#define ROBOT_WHEEL_2_OFFSET (3.0f * M_PI / 2.0f)  ///< Wheel 2 angular offset (rad) - 270°

// =============================================================================
// MOTOR CONFIGURATION
// =============================================================================

#define MOTOR_MAX_SPEED_PERCENT 12         ///< Maximum motor speed (% of PWM range)
#define MOTOR_MIN_SPEED_PERCENT 5          ///< Minimum motor speed (% of PWM range)
#define MOTOR_PWM_RESOLUTION_BITS 14       ///< PWM resolution in bits
#define MOTOR_MAX_POWER_PERCENT 90         ///< Maximum power limit (% of capability)

/**
 * @brief Motor direction multiplier for forward motion
 * @param i Motor index (0, 1, or 2)
 * @return Direction multiplier: 1 for forward, -1 for reverse
 */
#define MOTOR_DIRECTION_FORWARD(i) (-1)    ///< All motors use -1 for forward

// =============================================================================
// PID CONTROLLER TUNING PARAMETERS
// =============================================================================

// -----------------------------------------------------------------------------
// WHEEL SPEED PID (Inner Loop) - Controls individual wheel velocities
// -----------------------------------------------------------------------------

#define PID_MOTOR_KP 0.1f                  ///< Proportional gain
#define PID_MOTOR_KI 0.006f                ///< Integral gain
#define PID_MOTOR_KD 0.0f                  ///< Derivative gain
#define PID_MOTOR_BETA 0.0f                ///< Beta filter coefficient for derivative term
#define PID_MOTOR_MAX_OUTPUT 80.0f         ///< Maximum PID output (% motor speed)
#define PID_MOTOR_MIN_OUTPUT -80.0f        ///< Minimum PID output (% motor speed)

// -----------------------------------------------------------------------------
// ROBOT VELOCITY PID (Outer Loop) - Controls robot vx, vy, wz
// -----------------------------------------------------------------------------

#define PID_VELOCITY_KP 1.0f               ///< Proportional gain for velocity control
#define PID_VELOCITY_KI 0.01f               ///< Integral gain for velocity control
#define PID_VELOCITY_KD 0.00f              ///< Derivative gain for velocity control
#define PID_VELOCITY_BETA 0.0f             ///< Beta filter coefficient for derivative term
#define PID_VELOCITY_MAX_OUTPUT 1.0f       ///< Maximum velocity output (m/s or rad/s)
#define PID_VELOCITY_MIN_OUTPUT -1.0f      ///< Minimum velocity output (m/s or rad/s)

// =============================================================================
// SENSOR TASK CONFIGURATION
// =============================================================================

#define SENSOR_TASK_PERIOD_MS 2            ///< Sensor reading task period (ms)
#define SENSOR_TASK_SAMPLE_RATE_HZ (1000.0f / SENSOR_TASK_PERIOD_MS)  ///< Sample rate (Hz)
#define SENSOR_CUTOFF_FREQUENCY_OMEGA_HZ 1.0f  ///< Low-pass filter cutoff frequency (Hz)
#define SENSOR_KALMAN_Q 0.001f             ///< Kalman filter process noise covariance
#define SENSOR_KALMAN_R 10.0f              ///< Kalman filter measurement noise covariance

/**
 * @brief Sensor reading direction multiplier
 * @param i Sensor index (0, 1, or 2)
 * @return Direction multiplier: 1 for forward, -1 for reverse
 */
#define SENSOR_ANGULAR_DIRECTION_FORWARD(i) (-1)  ///< All sensors use -1 for forward

// =============================================================================
// CONTROL LOOP CONFIGURATION
// =============================================================================

#define CONTROL_TASK_PERIOD_MS 2           ///< Wheel PID control loop period (ms)
#define CONTROL_TASK_SAMPLE_RATE_HZ (1000.0f / CONTROL_TASK_PERIOD_MS)  ///< Sample rate (Hz)

#define VELOCITY_CONTROL_TASK_PERIOD_MS 10 ///< Velocity PID control loop period (ms)
#define VELOCITY_CONTROL_SAMPLE_RATE_HZ (1000.0f / VELOCITY_CONTROL_TASK_PERIOD_MS)  ///< Sample rate (Hz)

// =============================================================================
// KINEMATICS TASK CONFIGURATION
// =============================================================================

#define KINEMATICS_TASK_PERIOD_MS 10       ///< Inverse kinematics task period (ms)

// =============================================================================
// I2C BUS CONFIGURATION
// =============================================================================

#define BNO055_I2C_MASTER_NUM 0            ///< I2C master port for BNO055 IMU

#ifdef __cplusplus
}
#endif

#endif // CONFIG_UTILS_H