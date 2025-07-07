#ifndef TYPES_UTILS_H
#define TYPES_UTILS_H

#include <stdint.h>

#define NUM_ENCODERS 3  // Update if you have more encoders

/// @brief Per-encoder data
typedef struct {
    float angle_deg;    ///< Angle in degrees [0, 360)
    float omega_rad;    ///< Angular velocity in radians per second
} EncoderReading;

/// @brief Shared structure for all sensor readings
typedef struct {
    EncoderReading encoders[NUM_ENCODERS];  ///< Encoder readings
    // Add more fields below if needed (e.g., IMU data)
    // float imu_pitch;
    // float imu_yaw;
} RawSensorData;


/// @brief Velocity command structure
/// Contains linear and angular velocities for the robot
typedef struct {
    float vx;  // m/s
    float vy;  // m/s
    float wz;  // rad/s
} Velocity;

/// @brief Wheel speeds structure
/// Contains angular velocities for each wheel
/// Each wheel speed is in radians per second
typedef struct {
    float phi_dot[3];  // rad/s for wheel 1, 2, 3
} WheelSpeeds;

// // Declare globally accessible instance and mutex (defined in sensor_data.c)
// extern RawSensorData sensor_data;
// extern SemaphoreHandle_t xSensorDataMutex;

#endif //TYPES_UTILS_H