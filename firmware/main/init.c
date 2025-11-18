/**
 * @file init.c
 * @brief Hardware initialization implementation
 * 
 * Implements initialization routines for sensors, motors, and PID controllers.
 */

#include "init.h"

static const char *TAG = "INIT";

int init_sensors(void)
{
    ESP_LOGI(TAG, "Initializing sensors...");
    
    // ==========================================================================
    // AS5600 MAGNETIC ENCODER CONFIGURATION
    // ==========================================================================
    
    // Configure output pins for analog reading mode
    g_as5600[0].out = GPIO_ENCODER_0_IN_ANALOG;
    g_as5600[0].conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR;  // Analog 10%-90% range
    
    g_as5600[1].out = GPIO_ENCODER_1_IN_ANALOG;
    g_as5600[1].conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR;
    
    g_as5600[2].out = GPIO_ENCODER_2_IN_ANALOG;
    g_as5600[2].conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR;

    // Create shared ADC unit for all encoders
    if (!adc_create_unit(&g_shared_adc_handle)) {
        ESP_LOGE(TAG, "Failed to create ADC unit");
        return INIT_ERROR_AS5600;
    }

    // Initialize each AS5600 encoder with shared ADC
    for (int i = 0; i < 3; i++) {
        AS5600_InitADC_2(&g_as5600[i], g_shared_adc_handle);
    }
    
    ESP_LOGI(TAG, "AS5600 encoders initialized successfully");

    // ==========================================================================
    // BNO055 IMU CONFIGURATION (Currently disabled)
    // ==========================================================================
    
    /*
    int status = BNO055_Init(&g_bno055, GPIO_IMU_I2C_SDA, GPIO_IMU_I2C_SCL, BNO055_I2C_MASTER_NUM);
    while (status != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize BNO055 sensor, retrying...");
        BNO055_Reset(&g_bno055);
        vTaskDelay(pdMS_TO_TICKS(1000));
        status = BNO055_Init(&g_bno055, GPIO_IMU_I2C_SDA, GPIO_IMU_I2C_SCL, BNO055_I2C_MASTER_NUM);
    }
    ESP_LOGI(TAG, "BNO055 IMU initialized successfully");
    */

    ESP_LOGI(TAG, "Sensor initialization complete");
    return INIT_SUCCESS;
}

int init_motors(void)
{
    ESP_LOGI(TAG, "Initializing motors...");
    
    // ==========================================================================
    // MOTOR 0 CONFIGURATION
    // ==========================================================================
    
    g_motor[0] = (motor_brushless_t){
        .pwm_pin_speed = GPIO_MOTOR_0_SIGNAL_OUT_PWM,
        .pwm_pin_reverse = GPIO_MOTOR_0_REVERSE_OUT_PWM,
        .resolution_bits = MOTOR_PWM_RESOLUTION_BITS,
        .max_speed_percent = MOTOR_MAX_SPEED_PERCENT,
        .min_speed_percent = MOTOR_MIN_SPEED_PERCENT,
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .speed_channel = LEDC_CHANNEL_0,
        .reverse_channel = LEDC_CHANNEL_1,
        .is_reversed = false
    };
    
    // ==========================================================================
    // MOTOR 1 CONFIGURATION
    // ==========================================================================
    
    g_motor[1] = (motor_brushless_t){
        .pwm_pin_speed = GPIO_MOTOR_1_SIGNAL_OUT_PWM,
        .pwm_pin_reverse = GPIO_MOTOR_1_REVERSE_OUT_PWM,
        .resolution_bits = MOTOR_PWM_RESOLUTION_BITS,
        .max_speed_percent = MOTOR_MAX_SPEED_PERCENT,
        .min_speed_percent = MOTOR_MIN_SPEED_PERCENT,
        .timer_num = LEDC_TIMER_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .speed_channel = LEDC_CHANNEL_2,
        .reverse_channel = LEDC_CHANNEL_3,
        .is_reversed = false
    };
    
    // ==========================================================================
    // MOTOR 2 CONFIGURATION
    // ==========================================================================
    
    g_motor[2] = (motor_brushless_t){
        .pwm_pin_speed = GPIO_MOTOR_2_SIGNAL_OUT_PWM,
        .pwm_pin_reverse = GPIO_MOTOR_2_REVERSE_OUT_PWM,
        .resolution_bits = MOTOR_PWM_RESOLUTION_BITS,
        .max_speed_percent = MOTOR_MAX_SPEED_PERCENT,
        .min_speed_percent = MOTOR_MIN_SPEED_PERCENT,
        .timer_num = LEDC_TIMER_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .speed_channel = LEDC_CHANNEL_4,
        .reverse_channel = LEDC_CHANNEL_5,
        .is_reversed = false
    };

    // Initialize all motors
    for (int i = 0; i < 3; i++) {
        if (!motor_init(&g_motor[i])) {
            ESP_LOGE(TAG, "Failed to initialize motor %d", i);
            return INIT_ERROR_MOTOR;
        }
    }

    ESP_LOGI(TAG, "All motors initialized successfully");
    return INIT_SUCCESS;
}

int init_pid(void)
{
    ESP_LOGI(TAG, "Initializing PID controllers...");
    
    // ==========================================================================
    // WHEEL SPEED PID CONTROLLERS (Inner Loop)
    // ==========================================================================
    
    pid_config_t pid_config = {
        .init_param = g_pid_param
    };

    // Create PID control blocks for each motor (wheel speeds)
    for (int i = 0; i < 3; i++) {
        int ret = pid_new_control_block(&pid_config, &g_pid[i]);
        if (ret != PID_OK || g_pid[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create wheel PID controller %d", i);
            return INIT_ERROR_PID;
        }
    }
    
    ESP_LOGI(TAG, "Wheel PID controllers initialized successfully");
    
    // ==========================================================================
    // ROBOT VELOCITY PID CONTROLLERS (Outer Loop)
    // ==========================================================================
    
    pid_config_t velocity_pid_config = {
        .init_param = g_velocity_pid_param
    };

    // Create PID control blocks for robot velocity (vx, vy, wz)
    for (int i = 0; i < 3; i++) {
        int ret = pid_new_control_block(&velocity_pid_config, &g_velocity_pid[i]);
        if (ret != PID_OK || g_velocity_pid[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create velocity PID controller %d", i);
            return INIT_ERROR_PID;
        }
    }
    
    ESP_LOGI(TAG, "Velocity PID controllers initialized successfully");
    return INIT_SUCCESS;
}
