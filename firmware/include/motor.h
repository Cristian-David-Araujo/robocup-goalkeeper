#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Brushless motor configuration and state structure
 */
typedef struct {
    int pwm_pin_speed;              /**< GPIO pin for speed PWM signal */
    int pwm_pin_reverse;           /**< GPIO pin for reverse direction PWM signal */
    uint8_t resolution_bits;       /**< PWM resolution in bits (e.g., 8–15) */
    uint8_t max_speed_percent;     /**< Max speed allowed as a percentage (0–100) */
    ledc_timer_t timer_num;        /**< LEDC timer used for PWM */
    ledc_mode_t speed_mode;        /**< LEDC speed mode (high or low) */
    ledc_channel_t speed_channel;  /**< LEDC channel for speed */
    ledc_channel_t reverse_channel;/**< LEDC channel for reverse signal */
    bool is_reversed;              /**< Current direction state */
} motor_brushless_t;

/**
 * @brief Initialize the brushless motor using LEDC PWM
 * 
 * @param motor Pointer to the motor configuration structure
 * @return true on success, false on failure
 */
bool motor_init(motor_brushless_t *motor);

/**
 * @brief Set motor speed and direction in signed units based on resolution
 * 
 * @param motor Pointer to the motor instance
 * @param signed_speed Signed speed value (-max_duty to +max_duty)
 */
void motor_set_speed(motor_brushless_t *motor, int32_t signed_speed);

/**
 * @brief Stop the motor (set speed to 0)
 * 
 * @param motor Pointer to the motor instance
 */
void motor_stop(motor_brushless_t *motor);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
