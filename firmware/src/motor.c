#include "motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "MOTOR_DRIVER"

bool motor_init(motor_brushless_t *motor) {
    if (!motor || motor->max_speed_percent > 100) {
        ESP_LOGE(TAG, "Invalid motor configuration");
        return false;
    }

    ledc_timer_config_t timer_conf = {
        .duty_resolution = motor->resolution_bits,
        .freq_hz = 60, // 60 recommended for motors
        .speed_mode = motor->speed_mode,
        .timer_num = motor->timer_num,
        .clk_cfg = LEDC_AUTO_CLK
    };
    // Configure timer only if not already configured for this timer_num and speed_mode
    static bool timer_initialized[LEDC_TIMER_MAX][2] = { false };
    int mode_idx = (motor->speed_mode == LEDC_LOW_SPEED_MODE) ? 1 : 0;
    if (!timer_initialized[motor->timer_num][mode_idx]) {
        if (ledc_timer_config(&timer_conf) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC timer");
            return false;
        }
        timer_initialized[motor->timer_num][mode_idx] = true;
    }

    ledc_channel_config_t speed_channel = {
        .channel    = motor->speed_channel,
        .duty       = (uint32_t)(motor->min_speed_percent * ((1 << motor->resolution_bits) - 1) /100), // Set initial duty top of MOTOR_MAX_SPEED_PERCENT
        .gpio_num   = motor->pwm_pin_speed,
        .speed_mode = motor->speed_mode,
        .hpoint     = 0,
        .timer_sel  = motor->timer_num
    };
    if (ledc_channel_config(&speed_channel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure speed channel");
        return false;
    }

    ledc_channel_config_t reverse_channel = {
        .channel    = motor->reverse_channel,
        .duty       = (uint32_t)(75 * ((1 << motor->resolution_bits) - 1) /100),
        .gpio_num   = motor->pwm_pin_reverse,
        .speed_mode = motor->speed_mode,
        .hpoint     = 0,
        .timer_sel  = motor->timer_num
    };
    if (ledc_channel_config(&reverse_channel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure reverse channel");
        return false;
    }

    motor->is_reversed = false;
    return true;
}

void motor_set_speed(motor_brushless_t *motor, float signed_speed_percent)
{
    if (!motor) return;

    // Clamp input to [-100.0, 100.0]
    signed_speed_percent = fmaxf(fminf(signed_speed_percent, 100.0f), -100.0f);

    // Determine direction and absolute speed
    bool reverse = (signed_speed_percent < 0.0f);
    float abs_speed_percent = fabsf(signed_speed_percent);

    // Scale to effective speed using configured min/max bounds
    float speed_range = motor->max_speed_percent - motor->min_speed_percent;
    float effective_speed_percent = motor->min_speed_percent + (abs_speed_percent * speed_range / 100.0f);

    // Calculate duty cycle from effective speed
    uint32_t max_duty = (1U << motor->resolution_bits) - 1U;
    uint32_t duty = (uint32_t)lroundf((effective_speed_percent / 100.0f) * max_duty);

    // Log values
    // ESP_LOGI(TAG, "Set motor speed: %.2f%% | Dir: %s | Effective: %.2f%% | Duty: %lu",
    //          signed_speed_percent, reverse ? "REVERSE" : "FORWARD", effective_speed_percent, (unsigned long)duty);

    // Apply speed duty
    ledc_set_duty(motor->speed_mode, motor->speed_channel, duty);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);

    // Set reverse PWM signal: 25% for reverse ON, 75% for OFF
    uint32_t rev_duty = reverse ? (max_duty * 25U) / 100U : (max_duty * 75U) / 100U;
    ledc_set_duty(motor->speed_mode, motor->reverse_channel, rev_duty);
    ledc_update_duty(motor->speed_mode, motor->reverse_channel);

    // Update state
    motor->is_reversed = reverse;
}

void motor_stop(motor_brushless_t *motor) {
    if (!motor) return;

    ledc_set_duty(motor->speed_mode, motor->speed_channel, (motor->min_speed_percent * ((1 << motor->resolution_bits) - 1)) / 100);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);

    ledc_set_duty(motor->speed_mode, motor->reverse_channel, 0);
    ledc_update_duty(motor->speed_mode, motor->reverse_channel);

    motor->is_reversed = false;
}

void motor_calibration(motor_brushless_t *motor)
{
    if (!motor) return;
    // Wait for 2 seconds
    vTaskDelay(pdMS_TO_TICKS(4000));
    // Set speed to min_speed_percent without using the motor_set_speed function
    uint32_t min_duty = (motor->min_speed_percent * ((1 << motor->resolution_bits) - 1)) / 100;
    ledc_set_duty(motor->speed_mode, motor->speed_channel, min_duty);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);
    // Wait for 6 seconds
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void motor_calibration3(motor_brushless_t *motor_0, motor_brushless_t *motor_1, motor_brushless_t *motor_2)
{
    if (!motor_0 || !motor_1 || !motor_2) return;

    // Wait for 2 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));
    // Set speed to min_speed_percent without using the motor_set_speed function
    uint32_t min_duty = (motor_0->min_speed_percent * ((1 << motor_0->resolution_bits) - 1)) / 100;
    // Set speed for motor 0
    ledc_set_duty(motor_0->speed_mode, motor_0->speed_channel, min_duty);
    // Set speed for motor 1
    ledc_set_duty(motor_1->speed_mode, motor_1->speed_channel, min_duty); 
    // Set speed for motor 2
    ledc_set_duty(motor_2->speed_mode, motor_2->speed_channel, min_duty);
    // Update duty for all motors
    ledc_update_duty(motor_1->speed_mode, motor_1->speed_channel);
    ledc_update_duty(motor_0->speed_mode, motor_0->speed_channel);
    ledc_update_duty(motor_2->speed_mode, motor_2->speed_channel);
    // Wait for 6 seconds
    vTaskDelay(pdMS_TO_TICKS(3000));
}
