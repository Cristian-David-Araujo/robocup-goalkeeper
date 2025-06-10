#ifndef INIT_H
#define INIT_H




/* Header file for sensor initialization */
#include "as5600.h"
#include "bno055.h"
#include "platform_esp32s3.h"
#include "motor.h"
#include "pid.h"


#include "gpio_utils.h"
#include "config_utils.h"


// Define macros for error handling
#define INIT_SUCCESS 0
#define INIT_ERROR_AS5600 1  ///< Error code for AS5600 initialization failure
#define INIT_ERROR_BNO055 2 ///< Error code for BNO055 initialization failure
#define INIT_ERROR_PID 3 ///< Error code for PID initialization failure



extern AS5600_t as5600_0; ///< AS5600 sensor structure
extern AS5600_t as5600_1; ///< AS5600 sensor structure
extern AS5600_t as5600_2; ///< AS5600 sensor structure

extern BNO055_t bno055; ///< BNO055 sensor structure

extern motor_brushless_t motor_0; ///< Motor 0 configuration structure
extern motor_brushless_t motor_1; ///< Motor 1 configuration structure
extern motor_brushless_t motor_2; ///< Motor 2 configuration structure

extern pid_block_handle_t pid; ///< PID controller handle


/**
 * @brief Initialize the sensors
 * This function initializes the AS5600 and BNO055 sensors, sets up the UART for communication,
 * and configures the GPIO pins for the sensors.
 * @param AS5600_t *as5600 Pointer to the AS5600 sensor structure
 * @param BNO055_t *bno055 Pointer to the BNO055 sensor structure
 * @return int
 *         - INIT_SUCCESS if initialization is successful
 *        - INIT_ERROR_AS5600 if AS5600 initialization fails
 *        - INIT_ERROR_BNO055 if BNO055 initialization fails
 */
int init_sensors(void);

/**
 * @brief Initialize the motors
 * This function initializes the brushless motors using LEDC PWM.
 * It sets up the GPIO pins, LEDC timers, and channels for motor control.
 * @return int
 *         - INIT_SUCCESS if initialization is successful
 *         - INIT_ERROR_MOTOR if motor initialization fails
 */
int init_motors(void);

/**
 * @brief Initialize the PID controller
 * This function initializes the PID controller with default parameters.
 * It sets up the PID block handle and configures the PID parameters.
 * @return int
 *         - INIT_SUCCESS if initialization is successful
 *         - INIT_ERROR_PID if PID initialization fails
 */
int init_pid(void);


#endif // INIT_H