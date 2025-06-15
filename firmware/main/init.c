#include "init.h"


int init_sensors(void)
{

    ///< ---------------------- AS5600 -------------------
    AS5600_Init(&as5600_0, GPIO_ENCODER_0_I2C_MASTER_NUM, GPIO_ENCODER_0_I2C_SCL, GPIO_ENCODER_0_I2C_SDA, GPIO_ENCODER_0_IN_ANALOG);

    // Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };
    AS5600_SetConf(&as5600_0, conf);
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&as5600_0, AS5600_REG_CONF_H, &conf_reg);
    printf("Configuration register readed: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);

    ///< ------------- For calibration process. -------------
    AS5600_SetStartPosition(&as5600_0, 0x0000); ///< Set start position to 0 degrees
    AS5600_SetStopPosition(&as5600_0, 0x0FFF); ///< Set stop position to 360 degrees

    ///< Get the current start and stop positions
    uint16_t start_position, stop_position;
    AS5600_GetStartPosition(&as5600_0, &start_position); ///< Get start position
    AS5600_GetStopPosition(&as5600_0, &stop_position); ///< Get stop position

    printf("Start position: 0x%04X, Stop position: 0x%04X\n", start_position, stop_position);


    
    /*Set the parameters for the AS5600 sensors*/
    // as5600_0.out = GPIO_ENCODER_0_IN_ANALOG; // Set the OUT pin for AS5600 0
    // as5600_0.conf.OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR; // Set the output stage for AS5600 0
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

    pid_config_t pid_config = {
        .init_param = pid_param // Initialize PID parameters
    };

    pid_new_control_block(&pid_config, &pid); // Create a new PID control block with the specified parameters
    
    if (pid == NULL) {
        return INIT_ERROR_PID; // Return error if PID initialization fails
    }

    return INIT_SUCCESS; // Return success if PID controller is initialized successfully
}
