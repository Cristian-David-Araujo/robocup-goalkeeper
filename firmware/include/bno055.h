/**
 * @file bno055.h
 * @brief BNO055 9-axis IMU sensor driver interface
 * 
 * This module provides functions to initialize and control the BNO055 sensor
 * using I2C communication. Supports absolute orientation, gyroscope, accelerometer,
 * and magnetometer data readings.
 * 
 * Features:
 * - Multiple operation modes (NDOF, IMU, COMPASS, etc.)
 * - Calibration profile management
 * - Unit configuration (m/s², rad/s, rad, etc.)
 * - Power mode control
 * 
 * Thread-safety: Functions are NOT thread-safe. External synchronization required.
 * 
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @version 0.2
 * @date 2024-11-25
 * @copyright Copyright (c) 2024
 */

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "platform_esp32s3.h"
#include "bno055_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CONFIGURATION CONSTANTS
// =============================================================================

#define BNO055_I2C_FREQ_HZ  (400 * 1000)   ///< I2C clock frequency: 400 kHz
#define BNO055_SENSOR_ADDR  (0x29)         ///< Default I2C slave address for BNO055

// =============================================================================
// ENUMERATIONS
// =============================================================================

/**
 * @brief BNO055 operation modes
 * 
 * Defines available sensor fusion and non-fusion operation modes.
 * Switch to CONFIGMODE before changing configuration registers.
 */
typedef enum {
    BNO055_MODE_CONFIG      = 0x00,   ///< Configuration mode (all sensors off)
    BNO055_MODE_ACCONLY     = 0x01,   ///< Accelerometer only
    BNO055_MODE_MAGONLY     = 0x02,   ///< Magnetometer only
    BNO055_MODE_GYROONLY    = 0x03,   ///< Gyroscope only
    BNO055_MODE_ACCMAG      = 0x04,   ///< Accelerometer + Magnetometer
    BNO055_MODE_ACCGYRO     = 0x05,   ///< Accelerometer + Gyroscope
    BNO055_MODE_MAGGYRO     = 0x06,   ///< Magnetometer + Gyroscope
    BNO055_MODE_AMG         = 0x07,   ///< All three sensors (no fusion)
    BNO055_MODE_IMU         = 0x08,   ///< Fusion: Accel + Gyro (no magnetometer)
    BNO055_MODE_COMPASS     = 0x09,   ///< Fusion: Accel + Mag (no gyroscope)
    BNO055_MODE_M4G         = 0x0A,   ///< Fusion: All sensors
    BNO055_MODE_NDOF_FMC_OFF = 0x0B,  ///< NDOF without fast mag calibration
    BNO055_MODE_NDOF        = 0x0C,   ///< Nine Degrees of Freedom (full fusion)
    BNO055_MODE_INIT        = 0x0D    ///< Initialization state (internal use)
} bno055_operation_mode_t;

/**
 * @brief BNO055 power modes
 * 
 * Controls power consumption and sensor update rates.
 */
typedef enum {
    BNO055_POWER_NORMAL     = BNO055_POWER_MODE_NORMAL,     ///< Normal power mode
    BNO055_POWER_LOWPOWER   = BNO055_POWER_MODE_LOWPOWER,   ///< Low power mode
    BNO055_POWER_SUSPEND    = BNO055_POWER_MODE_SUSPEND     ///< Suspend mode
} bno055_power_mode_t;

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

/**
 * @brief BNO055 unit configuration structure
 * 
 * Stores the currently configured units for each sensor type.
 */
typedef struct {
    uint8_t accel_unit;   ///< Accelerometer unit (m/s² or mg)
    uint8_t gyro_unit;    ///< Gyroscope unit (dps or rps)
    uint8_t euler_unit;   ///< Euler angles unit (degrees or radians)
    uint8_t temp_unit;    ///< Temperature unit (Celsius or Fahrenheit)
    uint8_t ori_unit;     ///< Orientation convention (Windows or Android)
} bno055_unit_settings_t;

/**
 * @brief BNO055 calibration profile structure
 * 
 * Contains calibration offsets and radius values for all sensors.
 * Can be saved and restored for faster startup.
 */
typedef struct {
    uint8_t sys_calib_stat;   ///< System calibration status
    uint16_t accel_offset_x;  ///< Accelerometer X-axis offset
    uint16_t accel_offset_y;  ///< Accelerometer Y-axis offset
    uint16_t accel_offset_z;  ///< Accelerometer Z-axis offset
    uint16_t mag_offset_x;    ///< Magnetometer X-axis offset
    uint16_t mag_offset_y;    ///< Magnetometer Y-axis offset
    uint16_t mag_offset_z;    ///< Magnetometer Z-axis offset
    uint16_t gyro_offset_x;   ///< Gyroscope X-axis offset
    uint16_t gyro_offset_y;   ///< Gyroscope Y-axis offset
    uint16_t gyro_offset_z;   ///< Gyroscope Z-axis offset
    uint16_t accel_radius;    ///< Accelerometer calibration radius
    uint16_t mag_radius;      ///< Magnetometer calibration radius
} bno055_calib_profile_t;

/**
 * @brief BNO055 sensor instance structure
 * 
 * Contains hardware configuration, sensor state, and cached sensor readings.
 * Must be initialized with bno055_init() before use.
 * 
 * Thread-safety: Not thread-safe. Do not access from multiple tasks concurrently.
 */
typedef struct {
    // Hardware interfaces
    i2c_t i2c_handle;                       ///< I2C communication handle
    gpio_t rst_pin;                         ///< Optional reset pin (GPIO handle)
    
    // Configuration
    bno055_operation_mode_t operation_mode; ///< Current operation mode
    bno055_power_mode_t power_mode;         ///< Current power mode
    bno055_unit_settings_t unit_settings;   ///< Current unit configuration
    
    // Device information
    uint8_t chip_id;                        ///< Chip ID (should be 0xA0)
    uint8_t sw_rev_id[2];                   ///< Software revision [LSB, MSB]
    uint8_t page_id;                        ///< Current register page
    uint8_t accel_rev_id;                   ///< Accelerometer revision ID
    uint8_t mag_rev_id;                     ///< Magnetometer revision ID
    uint8_t gyro_rev_id;                    ///< Gyroscope revision ID
    uint8_t bl_rev_id;                      ///< Bootloader revision ID
    
    // Calibration and status
    uint8_t calib_stat;                     ///< Calibration status byte
    uint8_t test_stat;                      ///< Self-test result status
    
    // Cached sensor data (updated by read functions)
    float yaw;    ///< Euler yaw angle (heading)
    float pitch;  ///< Euler pitch angle
    float roll;   ///< Euler roll angle
    
    float ax;     ///< Acceleration X-axis
    float ay;     ///< Acceleration Y-axis
    float az;     ///< Acceleration Z-axis
    
    float gx;     ///< Angular velocity X-axis
    float gy;     ///< Angular velocity Y-axis
    float gz;     ///< Angular velocity Z-axis
    
    float mx;     ///< Magnetic field X-axis
    float my;     ///< Magnetic field Y-axis
    float mz;     ///< Magnetic field Z-axis
    
    // Internal buffer
    uint8_t buffer[128];                    ///< Internal communication buffer
} bno055_t;


// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

/**
 * @brief Initialize BNO055 sensor
 * 
 * Configures I2C communication, sets default operation mode (NDOF), unit settings,
 * and power mode. Retrieves device information for verification.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance structure
 * @param[in] sda GPIO pin for I2C SDA
 * @param[in] scl GPIO pin for I2C SCL
 * @param[in] i2c_num I2C port number
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 * 
 * @note Must be called before any other BNO055 functions
 */
int8_t bno055_init(bno055_t *bno055, uint8_t sda, uint8_t scl, uint8_t i2c_num);

/**
 * @brief Reset the BNO055 sensor
 * 
 * Deinitializes I2C communication. If a reset pin is configured,
 * toggles it to perform hardware reset.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 */
void bno055_reset(bno055_t *bno055);

/**
 * @brief Get calibration status of all sensors
 * 
 * Reads and updates the calibration status byte. Each sensor (system, gyro,
 * accel, mag) has a 2-bit calibration level (0-3, where 3 = fully calibrated).
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @return BNO055_SUCCESS if fully calibrated, BNO055_ERROR otherwise
 */
int8_t bno055_get_calibration_status(bno055_t *bno055);

/**
 * @brief Get device information
 * 
 * Reads chip ID, revision IDs, and software version from the sensor.
 * Updates the corresponding fields in the bno055_t structure.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_get_info(bno055_t *bno055);

/**
 * @brief Set sensor operation mode
 * 
 * Changes the current operation mode. Must switch to CONFIG mode before
 * modifying most configuration registers.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[in] mode Target operation mode
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 * 
 * @note Mode changes require a brief settling time
 */
int8_t bno055_set_operation_mode(bno055_t *bno055, bno055_operation_mode_t mode);

/**
 * @brief Get Euler angles (orientation)
 * 
 * Reads and converts Euler angle data from the sensor. Values are cached
 * in the bno055_t structure. If read fails, returns last valid values.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[out] yaw Pointer to store yaw angle (heading)
 * @param[out] pitch Pointer to store pitch angle
 * @param[out] roll Pointer to store roll angle
 * 
 * @note Units depend on euler_unit setting (degrees or radians)
 */
void bno055_get_euler_angles(bno055_t *bno055, float *yaw, float *pitch, float *roll);

/**
 * @brief Get linear acceleration data
 * 
 * Reads raw accelerometer data and converts to configured units.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[out] x Pointer to store X-axis acceleration
 * @param[out] y Pointer to store Y-axis acceleration
 * @param[out] z Pointer to store Z-axis acceleration
 * 
 * @note Units depend on accel_unit setting (m/s² or mg)
 */
void bno055_get_acceleration(bno055_t *bno055, float *x, float *y, float *z);

/**
 * @brief Get gyroscope angular velocity data
 * 
 * Reads raw gyroscope data and converts to configured units.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[out] gx Pointer to store X-axis angular velocity
 * @param[out] gy Pointer to store Y-axis angular velocity
 * @param[out] gz Pointer to store Z-axis angular velocity
 * 
 * @note Units depend on gyro_unit setting (dps or rps)
 */
void bno055_get_gyro(bno055_t *bno055, float *gx, float *gy, float *gz);

/**
 * @brief Get magnetometer data
 * 
 * Reads raw magnetometer data and converts to microTesla (µT).
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[out] mx Pointer to store X-axis magnetic field
 * @param[out] my Pointer to store Y-axis magnetic field
 * @param[out] mz Pointer to store Z-axis magnetic field
 */
void bno055_get_magnetometer(bno055_t *bno055, float *mx, float *my, float *mz);

/**
 * @brief Read all sensor data in a single transaction
 * 
 * Efficiently reads accelerometer, magnetometer, gyroscope, and Euler angle
 * data in one I2C burst read. Updates cached values in bno055_t structure.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_read_all(bno055_t *bno055);

/**
 * @brief Read all sensor data with linear acceleration
 * 
 * Similar to bno055_read_all but reads linear acceleration instead of
 * raw acceleration (gravity component removed by fusion algorithm).
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_read_all_linear(bno055_t *bno055);

/**
 * @brief Configure measurement units
 * 
 * Sets the output units for all sensor types. Must be called in CONFIG mode.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[in] accel_unit Acceleration unit (BNO055_ACCEL_UNIT_MSQ or _MG)
 * @param[in] gyro_unit Angular rate unit (BNO055_GYRO_UNIT_DPS or _RPS)
 * @param[in] euler_unit Euler angle unit (BNO055_EULER_UNIT_DEG or _RAD)
 * @param[in] temp_unit Temperature unit (BNO055_TEMP_UNIT_CELSIUS or _FAHRENHEIT)
 * @param[in] ori_unit Orientation convention (BNO055_ANDROID_ORIENTATION or _WINDOWS)
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_set_unit(bno055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, 
                       uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit);

/**
 * @brief Set power mode
 * 
 * Controls power consumption and sensor update rates.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[in] mode Target power mode
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_set_power_mode(bno055_t *bno055, bno055_power_mode_t mode);

/**
 * @brief Get complete calibration profile
 * 
 * Reads all calibration offsets and radius values from the sensor.
 * These values can be saved and restored to skip calibration on subsequent boots.
 * 
 * @param[in,out] bno055 Pointer to BNO055 instance
 * @param[out] calib_data Pointer to calibration profile structure to fill
 * @return BNO055_SUCCESS if calibrated and read successfully, BNO055_ERROR otherwise
 * 
 * @note Sensor must be fully calibrated before calling this function
 */
int8_t bno055_get_calibration_profile(bno055_t *bno055, bno055_calib_profile_t *calib_data);

#ifdef __cplusplus
}
#endif

#endif // BNO055_H