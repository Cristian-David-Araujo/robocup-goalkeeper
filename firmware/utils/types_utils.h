/**
 * @file types_utils.h
 * @brief Common type definitions for the RoboCup Goalkeeper firmware
 * 
 * This header defines data structures used throughout the firmware for sensor readings,
 * velocity commands, and wheel speed control. All types follow snake_case naming convention.
 * 
 * Thread-safety: The structures defined here are plain data types. Synchronization
 * must be handled by the calling code using appropriate FreeRTOS primitives.
 */

#ifndef TYPES_UTILS_H
#define TYPES_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Number of motor encoders in the system
#define NUM_ENCODERS 3

/**
 * @brief Per-encoder sensor reading
 * 
 * Contains angle and angular velocity data from a single encoder.
 * Thread-safety: Not thread-safe. Protect with mutex when sharing between tasks.
 */
typedef struct {
    float angle_deg;    ///< Angle in degrees [0, 360)
    float omega_rad;    ///< Angular velocity in radians per second
} encoder_reading_t;

/**
 * @brief Aggregate sensor data from all encoders
 * 
 * Contains readings from all encoder sensors in the system.
 * Thread-safety: Not thread-safe. Use xSensorDataMutex for inter-task access.
 */
typedef struct {
    encoder_reading_t encoders[NUM_ENCODERS];  ///< Array of encoder readings
    // Extensible for additional sensors (e.g., IMU data)
    // float imu_pitch_rad;
    // float imu_yaw_rad;
} raw_sensor_data_t;

/**
 * @brief Robot velocity in Cartesian coordinates
 * 
 * Represents the desired or actual robot velocity with linear components
 * in the x and y directions and angular velocity around z-axis.
 * Thread-safety: Not thread-safe. Protect with mutex when sharing between tasks.
 */
typedef struct {
    float vx;  ///< Linear velocity in x-direction (m/s)
    float vy;  ///< Linear velocity in y-direction (m/s)
    float wz;  ///< Angular velocity around z-axis (rad/s)
} velocity_t;

/**
 * @brief Wheel angular velocities for omnidirectional drive
 * 
 * Contains target or measured angular velocities for all three wheels.
 * Thread-safety: Not thread-safe. Protect with mutex when sharing between tasks.
 */
typedef struct {
    float phi_dot[3];  ///< Angular velocity (rad/s) for wheels 0, 1, 2
} wheel_speeds_t;

#ifdef __cplusplus
}
#endif

#endif // TYPES_UTILS_H