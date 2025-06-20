#ifndef CONFIG_UTILS_H
#define CONFIG_UTILS_H

#define MOTOR_MAX_SPEED_PERCENT 12  // Maximum speed percentage for motors
#define MOTOR_MIN_SPEED_PERCENT 5   // Minimum speed percentage for motors
#define MOTOR_PWM_RESOLUTION_BITS 14 // PWM resolution for motors
#define MOTOR_MAX_POWER_PERCENT 90 // Maximum power percentage for motors
#define MOTOR_DIRECTION_FORWARD(i) (((i) == 0) ? -1 : (((i) == 1) ? -1 : (((i) == 2) ? -1 : -1))) // Motor direction forward, i = motor index


/* PID MOTOR configuration */
#define PID_MOTOR_KP 0.1f                // Proportional gain
#define PID_MOTOR_KI 0.02f                // Integral gain
#define PID_MOTOR_KD 0.1f               // Derivative gain
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
// Angular velocity sensor direction (1 for forward, -1 for backward), i = sensor index
#define SENSOR_ANGULAR_DIRECTION_FORWARD(i) (((i) == 0) ? -1 : ((i) == 1) ? -1 : ((i) == 2) ? -1 : -1)

/**
 * @brief Configuration for control loop task
 * 
 */
#define CONTROL_TASK_PERIOD_MS 2 // Control loop task period in milliseconds
#define CONTROL_TASK_SAMPLE_RATE_HZ (1000.0f/(CONTROL_TASK_PERIOD_MS)) // Control loop sample rate in Hz


#endif // CONFIG_UTILS_H