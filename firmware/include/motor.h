/**
 * @file motor.h
 * @brief Brushless motor control interface using ESP32 LEDC PWM
 * 
 * This module provides functions to initialize and control brushless DC motors
 * using the ESP32's LEDC (LED Control) peripheral for PWM generation.
 * 
 * Features:
 * - Bidirectional speed control with signed percentage input
 * - Configurable speed limits and PWM resolution
 * - Motor calibration routines
 * 
 * Thread-safety: Functions are NOT thread-safe. External synchronization required
 * if motors are controlled from multiple tasks.
 * 
 * @note All identifiers follow snake_case naming convention
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"
#include "config_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Brushless motor configuration and state structure
 * 
 * Contains all parameters needed to control a single brushless motor via PWM.
 * This structure should be initialized with hardware-specific values before
 * calling motor_init().
 * 
 * Thread-safety: Not thread-safe. Do not modify while motor is active.
 */
typedef struct {
    int pwm_pin_speed;              ///< GPIO pin for speed PWM signal
    int pwm_pin_reverse;            ///< GPIO pin for reverse direction PWM signal
    uint8_t resolution_bits;        ///< PWM resolution in bits (8-15 typical)
    uint8_t max_speed_percent;      ///< Maximum allowed speed as percentage (0-100)
    uint8_t min_speed_percent;      ///< Minimum allowed speed as percentage (0-100)
    uint8_t max_power_percent;      ///< Maximum power limit as percentage (0-100)
    ledc_timer_t timer_num;         ///< LEDC timer number for this motor
    ledc_mode_t speed_mode;         ///< LEDC speed mode (LEDC_HIGH_SPEED_MODE or LEDC_LOW_SPEED_MODE)
    ledc_channel_t speed_channel;   ///< LEDC channel for speed control
    ledc_channel_t reverse_channel; ///< LEDC channel for reverse signal
    bool is_reversed;               ///< Current direction state (true = reverse)
} motor_brushless_t;

/**
 * @brief Initialize a brushless motor using LEDC PWM
 * 
 * Configures the LEDC timer and channels for motor control. Sets initial
 * speed to minimum and direction to forward.
 * 
 * @param[in,out] motor Pointer to motor configuration structure
 * @return true on successful initialization, false on failure
 * 
 * @note This function configures hardware and must be called before any
 *       other motor control functions.
 */
bool motor_init(motor_brushless_t *motor);

/**
 * @brief Set motor speed with signed percentage
 *
 * Controls motor speed and direction using a signed percentage value.
 * Negative values indicate reverse direction, positive values forward.
 * The function automatically handles:
 * - Input clamping to [-100.0, 100.0]
 * - Scaling to configured min/max speed limits
 * - Direction signal generation
 * 
 * @param[in,out] motor Pointer to motor instance
 * @param[in] signed_speed_percent Speed in range [-100.0, 100.0]
 *                                 Negative = reverse, Positive = forward
 * 
 * Thread-safety: Not thread-safe. Caller must ensure exclusive access.
 */
void motor_set_speed(motor_brushless_t *motor, float signed_speed_percent);

/**
 * @brief Stop the motor (set speed to minimum)
 * 
 * Sets motor speed to the configured minimum value and clears reverse signal.
 * Motor will be in a stopped/idle state after this call.
 * 
 * @param[in,out] motor Pointer to motor instance
 */
void motor_stop(motor_brushless_t *motor);

/**
 * @brief Calibrate a single motor
 * 
 * Performs motor calibration sequence by holding at minimum speed for
 * a defined period. Used during initialization to establish baseline.
 * 
 * @param[in,out] motor Pointer to motor instance
 * 
 * @warning This function blocks for several seconds
 */
void motor_calibration(motor_brushless_t *motor);

/**
 * @brief Calibrate three brushless motors simultaneously
 * 
 * Calibrates three motors in parallel by setting them to minimum speed
 * and waiting for stabilization. Typically used during system startup.
 * 
 * @param[in,out] motor_0 Pointer to first motor instance
 * @param[in,out] motor_1 Pointer to second motor instance
 * @param[in,out] motor_2 Pointer to third motor instance
 * 
 * @warning This function blocks for several seconds
 */
void motor_calibration3(motor_brushless_t *motor_0, 
                       motor_brushless_t *motor_1, 
                       motor_brushless_t *motor_2);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
