#ifndef CONFIG_UTILS_H
#define CONFIG_UTILS_H

#define MOTOR_MAX_SPEED_PERCENT 12  // Maximum speed percentage for motors
#define MOTOR_MIN_SPEED_PERCENT 5   // Minimum speed percentage for motors
#define MOTOR_PWM_RESOLUTION_BITS 14 // PWM resolution for motors
#define MOTOR_MAX_POWER_PERCENT 90 // Maximum power percentage for motors

/* PID MOTOR configuration */
#define PID_MOTOR_KP 0.5f                // Proportional gain
#define PID_MOTOR_KI 0.0f                // Integral gain
#define PID_MOTOR_KD 0.0f               // Derivative gain
#define PID_MOTOR_BETA 0.0f              // Beta filter coefficient for derivative term
#define PID_MOTOR_MAX_OUTPUT 100.0f      // Maximum output of PID controller
#define PID_MOTOR_MIN_OUTPUT -100.0f     // Minimum output of PID controller



#endif // CONFIG_UTILS_H