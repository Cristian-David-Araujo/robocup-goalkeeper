#ifndef CONFIG_UTILS_H
#define CONFIG_UTILS_H

#include <math.h>

#define ROBOT_NAME "RoboCupGoalkeeper" // Name of the robot
#define ROBOT_VERSION "1.0" // Version of the robot firmware
#define ROBOT_BODY_RADIUS 0.08f // Radius of the robot in meters
#define ROBOT_WHEEL_RADIUS 0.03f // Radius of the robot wheels in meters
#define ROBOT_WHEEL_0_OFFSET M_PI/6 // Offset angle for wheel 0 in radians
#define ROBOT_WHEEL_1_OFFSET 5*M_PI/6 // Offset angle for wheel 1 in radians
#define ROBOT_WHEEL_2_OFFSET 3*M_PI/2 // Offset angle for wheel 2 in radians


#define MOTOR_MAX_SPEED_PERCENT 12  // Maximum speed percentage for motors
#define MOTOR_MIN_SPEED_PERCENT 5   // Minimum speed percentage for motors
#define MOTOR_PWM_RESOLUTION_BITS 14 // PWM resolution for motors
#define MOTOR_MAX_POWER_PERCENT 90 // Maximum power percentage for motors
#define MOTOR_DIRECTION_FORWARD(i) (((i) == 0) ? -1 : (((i) == 1) ? -1 : (((i) == 2) ? -1 : -1))) // Motor direction forward, i = motor index


/* PID MOTOR configuration */
#define PID_MOTOR_KP 0.1f                // Proportional gain
#define PID_MOTOR_KI 0.006f                // Integral gain
#define PID_MOTOR_KD 0.0f               // Derivative gain
#define PID_MOTOR_BETA 0.0f              // Beta filter coefficient for derivative term
#define PID_MOTOR_MAX_OUTPUT 80.0f      // Maximum output of PID controller
#define PID_MOTOR_MIN_OUTPUT -80.0f     // Minimum output of PID controller


/** @brief Configuration for sensor reading task
 * 
 * This configuration defines the task period, sample rate, and cutoff frequency
 * for the angular velocity sensor low-pass filter.
 */
#define SENSOR_TASK_PERIOD_MS 2
#define SENSOR_TASK_SAMPLE_RATE_HZ (1000.0f/(SENSOR_TASK_PERIOD_MS)) // Sensor reading task sample rate in Hz
#define SENSOR_CUTOFF_FREQUENCY_OMEGA_HZ 1.0f // Cutoff frequency for angular velocity sensor low-pass filter
#define SENSOR_KALMAN_Q 0.001f // Process noise covariance for Kalman filter
#define SENSOR_KALMAN_R 10.0f // Measurement noise covariance for Kalman filter
// Angular velocity sensor direction (1 for forward, -1 for backward), i = sensor index
#define SENSOR_ANGULAR_DIRECTION_FORWARD(i) (((i) == 0) ? -1 : ((i) == 1) ? -1 : ((i) == 2) ? -1 : -1)

/**
 * @brief Configuration for control loop task
 * 
 */
#define CONTROL_TASK_PERIOD_MS 2 // Control loop task period in milliseconds
#define CONTROL_TASK_SAMPLE_RATE_HZ (1000.0f/(CONTROL_TASK_PERIOD_MS)) // Control loop sample rate in Hz

#define KINEMATICS_TASK_PERIOD_MS 10 // Inverse kinematics task period in milliseconds

#define BNO055_I2C_MASTER_NUM 0
#endif // CONFIG_UTILS_H