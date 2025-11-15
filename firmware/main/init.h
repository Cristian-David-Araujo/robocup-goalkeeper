/**
 * @file init.h
 * @brief Hardware initialization interface for RoboCup Goalkeeper firmware
 * 
 * This module handles initialization of all hardware peripherals including:
 * - AS5600 magnetic encoders (3 units)
 * - BNO055 IMU sensor
 * - Brushless motor controllers (3 units)
 * - PID controllers (3 units)
 * 
 * All identifiers follow snake_case naming convention.
 * 
 * @note Call init functions in order: sensors -> motors -> pid
 */

#ifndef INIT_H
#define INIT_H

#include "esp_log.h"
#include "as5600.h"
#include "bno055.h"
#include "platform_esp32s3.h"
#include "motor.h"
#include "pid.h"
#include "gpio_utils.h"
#include "config_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// ERROR CODES
// =============================================================================

#define INIT_SUCCESS 0              ///< Initialization successful
#define INIT_ERROR_AS5600 1         ///< AS5600 encoder initialization failed
#define INIT_ERROR_BNO055 2         ///< BNO055 IMU initialization failed
#define INIT_ERROR_PID 3            ///< PID controller initialization failed
#define INIT_ERROR_MOTOR 4          ///< Motor initialization failed

// =============================================================================
// EXTERNAL HARDWARE INSTANCE DECLARATIONS
// =============================================================================

/// @brief Array of AS5600 encoder instances (one per motor)
extern AS5600_t g_as5600[3];

/// @brief BNO055 IMU sensor instance
extern BNO055_t g_bno055;

/// @brief Array of brushless motor instances
extern motor_brushless_t g_motor[3];

/// @brief Array of PID controller handles
extern pid_block_handle_t g_pid[3];

/// @brief PID controller parameters (shared configuration)
extern pid_parameter_t g_pid_param;

/// @brief Shared ADC handle for encoder analog reading
extern adc_oneshot_unit_handle_t g_shared_adc_handle;

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

/**
 * @brief Initialize all sensor peripherals
 *
 * Configures and initializes:
 * - AS5600 magnetic encoders (analog mode with shared ADC)
 * - BNO055 IMU (I2C interface) - currently disabled in implementation
 *
 * @return int Status code
 *         - INIT_SUCCESS on successful initialization
 *         - INIT_ERROR_AS5600 if encoder setup fails
 *         - INIT_ERROR_BNO055 if IMU setup fails
 * 
 * @note This function must be called before init_motors() and init_pid()
 */
int init_sensors(void);

/**
 * @brief Initialize all motor controllers
 *
 * Configures LEDC PWM channels and initializes three brushless motor
 * controllers with speed and direction control signals.
 *
 * @return int Status code
 *         - INIT_SUCCESS on successful initialization
 *         - INIT_ERROR_MOTOR if any motor fails to initialize
 * 
 * @note Requires GPIO pins to be properly configured
 */
int init_motors(void);

/**
 * @brief Initialize PID controllers
 *
 * Creates and configures PID control blocks for each motor using
 * parameters defined in g_pid_param.
 *
 * @return int Status code
 *         - INIT_SUCCESS on successful initialization
 *         - INIT_ERROR_PID if PID block creation fails
 * 
 * @note PID parameters can be tuned in config_utils.h
 */
int init_pid(void);

#ifdef __cplusplus
}
#endif

#endif // INIT_H