#include "motor.h"
#include "esp_log.h"

#define TAG "MOTOR_DRIVER"

bool motor_init(motor_brushless_t *motor) {
    if (!motor || motor->max_speed_percent > 100) {
        ESP_LOGE(TAG, "Invalid motor configuration");
        return false;
    }

    ledc_timer_config_t timer_conf = {
        .duty_resolution = motor->resolution_bits,
        .freq_hz = 20000, // 20 kHz recommended for motors
        .speed_mode = motor->speed_mode,
        .timer_num = motor->timer_num,
        .clk_cfg = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&timer_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return false;
    }

    ledc_channel_config_t speed_channel = {
        .channel    = motor->speed_channel,
        .duty       = 0,
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
        .duty       = 0,
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

void motor_set_speed(motor_brushless_t *motor, int32_t signed_speed) {
    if (!motor) return;

    int32_t max_duty = ((1 << motor->resolution_bits) - 1) * motor->max_speed_percent / 100;

    if (signed_speed > max_duty) signed_speed = max_duty;
    if (signed_speed < -max_duty) signed_speed = -max_duty;

    bool reverse = (signed_speed < 0);
    uint32_t duty = (uint32_t)(reverse ? -signed_speed : signed_speed);

    // Set speed duty
    ledc_set_duty(motor->speed_mode, motor->speed_channel, duty);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);

    // Set reverse signal (25% for reverse ON, 75% for reverse OFF)
    uint32_t max_duty = (1 << motor->resolution_bits) - 1;
    uint32_t rev_duty = reverse ? (max_duty * 25) / 100 : (max_duty * 75) / 100;

    ledc_set_duty(motor->speed_mode, motor->reverse_channel, rev_duty);
    ledc_update_duty(motor->speed_mode, motor->reverse_channel);

    motor->is_reversed = reverse;
}

void motor_stop(motor_brushless_t *motor) {
    if (!motor) return;

    ledc_set_duty(motor->speed_mode, motor->speed_channel, 0);
    ledc_update_duty(motor->speed_mode, motor->speed_channel);

    ledc_set_duty(motor->speed_mode, motor->reverse_channel, 0);
    ledc_update_duty(motor->speed_mode, motor->reverse_channel);

    motor->is_reversed = false;
}
