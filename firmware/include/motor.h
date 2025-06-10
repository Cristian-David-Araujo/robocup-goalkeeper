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
    uint8_t min_speed_percent;     /**< Min speed allowed as a percentage (0–100) */
    uint8_t max_power_percent;    /**< Max power allowed as a percentage (0–100) */
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
 * @brief Set motor speed with signed percentage.
 *
 * @param motor Pointer to motor instance.
 * @param signed_speed_percent Speed in range [-100.0, 100.0]. Negative values mean reverse.
 */
void motor_set_speed(motor_brushless_t *motor, float signed_speed_percent);

/**
 * @brief Stop the motor (set speed to 0)
 * 
 * @param motor Pointer to the motor instance
 */
void motor_stop(motor_brushless_t *motor);


/**
 * @brief Calibrate the motor by setting the speed to 0 and reversing direction
 * 
 * This function is used to calibrate the motor by setting it to a known state.
 * It can be used during initialization or when the motor needs to be reset.
 * 
 * @param motor Pointer to the motor instance
 */
void motor_calibration(motor_brushless_t *motor);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
