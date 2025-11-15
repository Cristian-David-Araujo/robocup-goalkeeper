/**
 * @file motor.c
 * @brief Brushless motor control implementation
 * 
 * Implements PWM-based motor control using ESP32 LEDC peripheral.
 * Handles speed scaling, direction control, and calibration sequences.
 */

#include "motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MOTOR_DRIVER";

/// @brief Track which timers have been initialized to avoid reconfiguration
static bool g_timer_initialized[LEDC_TIMER_MAX][2] = { false };

bool motor_init(motor_brushless_t *motor) 
{
    // Validate input parameters
    if (!motor || motor->max_speed_percent > 100) {
        ESP_LOGE(TAG, "Invalid motor configuration");
        return false;
    }

    // Configure LEDC timer (shared by speed and reverse channels)
    ledc_timer_config_t timer_conf = {
        .duty_resolution = motor->resolution_bits,
        .freq_hz = 60,  // 60 Hz recommended for brushless motors
        .speed_mode = motor->speed_mode,
        .timer_num = motor->timer_num,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    // Configure timer only once per speed_mode and timer_num combination
    int mode_idx = (motor->speed_mode == LEDC_LOW_SPEED_MODE) ? 1 : 0;
    if (!g_timer_initialized[motor->timer_num][mode_idx]) {
        esp_err_t ret = ledc_timer_config(&timer_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
            return false;
        }
        g_timer_initialized[motor->timer_num][mode_idx] = true;
        ESP_LOGI(TAG, "LEDC timer %d (mode %d) configured", motor->timer_num, mode_idx);
    }

    // Configure speed channel with initial duty at minimum speed
    uint32_t max_duty = (1U << motor->resolution_bits) - 1U;
    uint32_t init_duty = (motor->min_speed_percent * max_duty) / 100U;
    
    ledc_channel_config_t speed_channel = {
        .channel    = motor->speed_channel,
        .duty       = init_duty,
        .gpio_num   = motor->pwm_pin_speed,
        .speed_mode = motor->speed_mode,
        .hpoint     = 0,
        .timer_sel  = motor->timer_num
    };
    
    esp_err_t ret = ledc_channel_config(&speed_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure speed channel: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure reverse channel with 75% duty (reverse OFF state)
    ledc_channel_config_t reverse_channel = {
        .channel    = motor->reverse_channel,
        .duty       = (75U * max_duty) / 100U,
        .gpio_num   = motor->pwm_pin_reverse,
        .speed_mode = motor->speed_mode,
        .hpoint     = 0,
        .timer_sel  = motor->timer_num
    };
    
    ret = ledc_channel_config(&reverse_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure reverse channel: %s", esp_err_to_name(ret));
        return false;
    }

    motor->is_reversed = false;
    ESP_LOGI(TAG, "Motor initialized on speed pin %d, reverse pin %d", 
             motor->pwm_pin_speed, motor->pwm_pin_reverse);
    
    return true;
}

void motor_set_speed(motor_brushless_t *motor, float signed_speed_percent)
{
    if (!motor) {
        ESP_LOGW(TAG, "motor_set_speed called with NULL pointer");
        return;
    }

    // Clamp input to valid range [-100.0, 100.0]
    signed_speed_percent = fmaxf(fminf(signed_speed_percent, 100.0f), -100.0f);

    // Determine direction and absolute speed
    bool reverse = (signed_speed_percent < 0.0f);
    float abs_speed_percent = fabsf(signed_speed_percent);

    // Scale to effective speed using configured min/max bounds
    // This maps [0, 100] user input to [min_speed, max_speed] hardware range
    float speed_range = motor->max_speed_percent - motor->min_speed_percent;
    float effective_speed_percent = motor->min_speed_percent + 
                                   (abs_speed_percent * speed_range / 100.0f);

    // Calculate duty cycle from effective speed
    uint32_t max_duty = (1U << motor->resolution_bits) - 1U;
    uint32_t duty = (uint32_t)lroundf((effective_speed_percent / 100.0f) * max_duty);

    // Apply speed duty cycle
    ledc_set_duty(motor->speed_mode, motor->speed_channel, duty);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);

    // Set reverse PWM signal: 25% for reverse ON, 75% for reverse OFF
    uint32_t rev_duty = reverse ? ((25U * max_duty) / 100U) : ((75U * max_duty) / 100U);
    ledc_set_duty(motor->speed_mode, motor->reverse_channel, rev_duty);
    ledc_update_duty(motor->speed_mode, motor->reverse_channel);

    // Update internal state
    motor->is_reversed = reverse;
}

void motor_stop(motor_brushless_t *motor) 
{
    if (!motor) {
        ESP_LOGW(TAG, "motor_stop called with NULL pointer");
        return;
    }

    uint32_t max_duty = (1U << motor->resolution_bits) - 1U;
    uint32_t min_duty = (motor->min_speed_percent * max_duty) / 100U;

    // Set speed to minimum
    ledc_set_duty(motor->speed_mode, motor->speed_channel, min_duty);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);

    // Turn off reverse signal (75% duty)
    ledc_set_duty(motor->speed_mode, motor->reverse_channel, (75U * max_duty) / 100U);
    ledc_update_duty(motor->speed_mode, motor->reverse_channel);

    motor->is_reversed = false;
    ESP_LOGI(TAG, "Motor stopped");
}

void motor_calibration(motor_brushless_t *motor)
{
    if (!motor) {
        ESP_LOGW(TAG, "motor_calibration called with NULL pointer");
        return;
    }
    
    ESP_LOGI(TAG, "Starting motor calibration - waiting 4 seconds...");
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    // Set speed to minimum without using motor_set_speed function
    uint32_t max_duty = (1U << motor->resolution_bits) - 1U;
    uint32_t min_duty = (motor->min_speed_percent * max_duty) / 100U;
    
    ledc_set_duty(motor->speed_mode, motor->speed_channel, min_duty);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);
    
    ESP_LOGI(TAG, "Motor at minimum speed - waiting 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "Motor calibration complete");
}

void motor_calibration3(motor_brushless_t *motor_0, 
                       motor_brushless_t *motor_1, 
                       motor_brushless_t *motor_2)
{
    if (!motor_0 || !motor_1 || !motor_2) {
        ESP_LOGE(TAG, "motor_calibration3 called with NULL pointer(s)");
        return;
    }

    ESP_LOGI(TAG, "Starting 3-motor calibration - waiting 5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Calculate minimum duty for all motors (assume same resolution)
    uint32_t max_duty = (1U << motor_0->resolution_bits) - 1U;
    uint32_t min_duty = (motor_0->min_speed_percent * max_duty) / 100U;
    
    // Set all motors to minimum speed simultaneously
    ledc_set_duty(motor_0->speed_mode, motor_0->speed_channel, min_duty);
    ledc_set_duty(motor_1->speed_mode, motor_1->speed_channel, min_duty); 
    ledc_set_duty(motor_2->speed_mode, motor_2->speed_channel, min_duty);
    
    // Update all duties
    ledc_update_duty(motor_0->speed_mode, motor_0->speed_channel);
    ledc_update_duty(motor_1->speed_mode, motor_1->speed_channel);
    ledc_update_duty(motor_2->speed_mode, motor_2->speed_channel);
    
    ESP_LOGI(TAG, "All motors at minimum speed - waiting 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "3-motor calibration complete");
}
