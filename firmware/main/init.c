#include "init.h"

int init_sensors(void)
{


    
    /*Set the parameters for the AS5600 sensors*/
    as5600_0.out = GPIO_ENCODER_0_IN_ANALOG; // Set the OUT pin for AS5600 0
    as5600_0.conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR; // Set the output stage for AS5600 0
    // as5600_1.out = GPIO_ENCODER_1_IN_ANALOG; // Set the OUT pin for AS5600
    // as5600_1.conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR; // Set the output stage for AS5600 1
    // as5600_2.out = GPIO_ENCODER_2_IN_ANALOG; // Set the OUT pin for AS5600 2
    // as5600_2.conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR; // Set the output stage for AS5600 2

    int status = INIT_SUCCESS;

    // Initialize AS5600 sensors
    
    AS5600_InitADC(&as5600_0);
    
    // status = AS5600_Init_ADC(&as5600_1);
    // if (status != INIT_SUCCESS) {
    //     return INIT_ERROR_AS5600; // Return error if AS5600 1 initialization fails
    // }
    // status = AS5600_Init_ADC(&as5600_2);
    // if (status != INIT_SUCCESS) {
    //     return INIT_ERROR_AS5600; // Return error if AS5600 2 initialization fails
    // }


    // Initialize BNO055 sensor
    // status = BNO055_Init(&bno055, GPIO_IMU_UART_TX, GPIO_IMU_UART_RX);
    // if (status != INIT_SUCCESS) {
    //     return INIT_ERROR_BNO055; // Return error if BNO055 initialization fails
    // }

    
    return status; // Return success if all sensors are initialized successfully

}

int init_motors(void)
{
    motor_0 = (motor_brushless_t){
        .pwm_pin_speed = GPIO_MOTOR_0_SIGNAL_OUT_PWM,
        .pwm_pin_reverse = GPIO_MOTOR_0_REVERSE_OUT_PWM,
        .resolution_bits = MOTOR_PWM_RESOLUTION_BITS, // Example resolution
        .max_speed_percent = MOTOR_MAX_SPEED_PERCENT, // Max speed percentage
        .min_speed_percent = MOTOR_MIN_SPEED_PERCENT, // Min speed percentage
        .timer_num = LEDC_TIMER_0, // Use timer 0
        .speed_mode = LEDC_LOW_SPEED_MODE, // Low speed mode
        .speed_channel = LEDC_CHANNEL_0, // Channel 0 for speed
        .reverse_channel = LEDC_CHANNEL_1, // Channel 1 for reverse
        .is_reversed = false // Initial direction state
    };
    if (!motor_init(&motor_0)) {
        return INIT_ERROR_AS5600; // Return error if motor 0 initialization fails
    }
    

    // motor_1 = (motor_brushless_t){
    //     .pwm_pin_speed = GPIO_MOTOR_1_SIGNAL_OUT_PWM,
    //     .pwm_pin_reverse = GPIO_MOTOR_1_REVERSE_OUT_PWM,
    //     .resolution_bits = MOTOR_PWM_RESOLUTION_BITS, // Example resolution
    //     .max_speed_percent = MOTOR_MAX_SPEED_PERCENT, // Max speed percentage
    //     .timer_num = LEDC_TIMER_1, // Use timer 1
    //     .speed_mode = LEDC_HIGH_SPEED_MODE, // High speed mode
    //     .speed_channel = LEDC_CHANNEL_2, // Channel 2 for speed
    //     .reverse_channel = LEDC_CHANNEL_3, // Channel 3 for reverse
    //     .is_reversed = false // Initial direction state
    // };
    // if (!motor_init(&motor_1)) {
    //     return INIT_ERROR_AS5600; // Return error if motor 1 initialization fails
    // }

    // motor_2 = (motor_brushless_t){
    //     .pwm_pin_speed = GPIO_MOTOR_2_SIGNAL_OUT_PWM,
    //     .pwm_pin_reverse = GPIO_MOTOR_2_REVERSE_OUT_PWM,
    //     .resolution_bits = MOTOR_PWM_RESOLUTION_BITS, // Example resolution
    //     .max_speed_percent = MOTOR_MAX_SPEED_PERCENT, // Max speed percentage
    //     .timer_num = LEDC_TIMER_2, // Use timer 2
    //     .speed_mode = LEDC_HIGH_SPEED_MODE, // High speed mode
    //     .speed_channel = LEDC_CHANNEL_4, // Channel 4 for speed
    //     .reverse_channel = LEDC_CHANNEL_5, // Channel 5 for reverse
    //     .is_reversed = false // Initial direction state
    // };
    // if (!motor_init(&motor_2)) {
    //     return INIT_ERROR_AS5600; // Return error if motor 2 initialization fails
    // }

    return INIT_SUCCESS; // Return success if all motors are initialized successfully
}

int init_pid(void)
{
    pid_parameter_t pid_param = {
        .kp = PID_MOTOR_KP,
        .ki = PID_MOTOR_KI,
        .kd = PID_MOTOR_KD,
        .max_output = PID_MOTOR_MAX_OUTPUT, // Set maximum output for PID controller
        .min_output = PID_MOTOR_MIN_OUTPUT, // Set minimum output for PID controller
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .beta = PID_MOTOR_BETA // Set beta filter coefficient for derivative term
    };

    pid_config_t pid_config = {
        .init_param = pid_param // Initialize PID parameters
    };

    pid_new_control_block(&pid, &pid_config); // Create a new PID control block with the specified parameters
    
    if (pid == NULL) {
        return INIT_ERROR_PID; // Return error if PID initialization fails
    }

    return INIT_SUCCESS; // Return success if PID controller is initialized successfully
}
