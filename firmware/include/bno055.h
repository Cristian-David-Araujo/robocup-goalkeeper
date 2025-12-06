/**
 * @file bno055.h
 * @brief BNO055 9-DOF IMU sensor driver interface
 * 
 * This module provides functions to interface with the Bosch BNO055 Intelligent
 * 9-axis Absolute Orientation Sensor via I2C communication. The BNO055 integrates
 * a triaxial accelerometer, gyroscope, and magnetometer with a sensor fusion
 * algorithm to provide:
 * - Absolute orientation (Euler angles)
 * - Angular velocity
 * - Linear acceleration
 * - Magnetic field strength
 * 
 * Features:
 * - Multiple operating modes (IMU, NDOF, Compass, etc.)
 * - Configurable units (m/s², rad/s, radians, etc.)
 * - Sensor calibration support
 * - Power mode management
 * 
 * Thread-safety: Functions are NOT thread-safe. External synchronization required
 * if sensor is accessed from multiple tasks.
 * 
 * @note All identifiers follow snake_case naming convention
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @date 2024-11-08
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

#define BNO055_I2C_FREQ_HZ   400000    ///< I2C clock frequency (400 kHz)
#define BNO055_SENSOR_ADDR   0x29      ///< I2C slave address

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

/**
 * @brief BNO055 operation modes
 * 
 * The BNO055 supports multiple operation modes that control which sensors are
 * active and whether sensor fusion is enabled.
 */
typedef enum {
    BNO055_MODE_CONFIG       = 0x00,   ///< Configuration mode (no sensors active)
    BNO055_MODE_ACCONLY      = 0x01,   ///< Accelerometer only
    BNO055_MODE_MAGONLY      = 0x02,   ///< Magnetometer only
    BNO055_MODE_GYROONLY     = 0x03,   ///< Gyroscope only
    BNO055_MODE_ACCMAG       = 0x04,   ///< Accelerometer + Magnetometer
    BNO055_MODE_ACCGYRO      = 0x05,   ///< Accelerometer + Gyroscope
    BNO055_MODE_MAGGYRO      = 0x06,   ///< Magnetometer + Gyroscope
    BNO055_MODE_AMG          = 0x07,   ///< All sensors without fusion
    BNO055_MODE_IMU          = 0x08,   ///< Fusion: Accelerometer + Gyroscope
    BNO055_MODE_COMPASS      = 0x09,   ///< Fusion: Accelerometer + Magnetometer
    BNO055_MODE_M4G          = 0x0A,   ///< Fusion: Accel + Gyro + Mag (no fast mag cal)
    BNO055_MODE_NDOF_FMC_OFF = 0x0B,   ///< Fusion: NDOF without fast mag calibration
    BNO055_MODE_NDOF         = 0x0C,   ///< Fusion: Full NDOF with all calibrations
    BNO055_MODE_INIT         = 0x0D    ///< Internal initialization state
} bno055_operation_mode_t;

/**
 * @brief BNO055 power modes
 * 
 * Controls power consumption by enabling/disabling sensors and processing.
 */
typedef enum {
    BNO055_POWER_NORMAL  = BNO055_POWER_MODE_NORMAL,   ///< Normal operation (all systems active)
    BNO055_POWER_LOW     = BNO055_POWER_MODE_LOWPOWER, ///< Low power mode (reduced sampling)
    BNO055_POWER_SUSPEND = BNO055_POWER_MODE_SUSPEND   ///< Suspend mode (minimal power)
} bno055_power_mode_t;

/**
 * @brief BNO055 unit settings structure
 * 
 * Stores the current unit configuration for all sensor outputs.
 */
typedef struct {
    uint8_t accel_unit;  ///< Accelerometer unit (0=m/s², 1=mg)
    uint8_t gyro_unit;   ///< Gyroscope unit (0=dps, 1=rps)
    uint8_t euler_unit;  ///< Euler angles unit (0=degrees, 1=radians)
    uint8_t temp_unit;   ///< Temperature unit (0=Celsius, 1=Fahrenheit)
    uint8_t ori_unit;    ///< Orientation convention (0=Windows, 1=Android)
} bno055_unit_settings_t;

/**
 * @brief BNO055 calibration profile structure
 * 
 * Contains calibration offsets and radius values for all sensors.
 * Can be saved and restored to skip calibration on subsequent boots.
 */
typedef struct {
    uint8_t  sys_calib_stat;   ///< System calibration status
    uint16_t accel_offset_x;   ///< Accelerometer X-axis offset
    uint16_t accel_offset_y;   ///< Accelerometer Y-axis offset
    uint16_t accel_offset_z;   ///< Accelerometer Z-axis offset
    uint16_t mag_offset_x;     ///< Magnetometer X-axis offset
    uint16_t mag_offset_y;     ///< Magnetometer Y-axis offset
    uint16_t mag_offset_z;     ///< Magnetometer Z-axis offset
    uint16_t gyro_offset_x;    ///< Gyroscope X-axis offset
    uint16_t gyro_offset_y;    ///< Gyroscope Y-axis offset
    uint16_t gyro_offset_z;    ///< Gyroscope Z-axis offset
    uint16_t accel_radius;     ///< Accelerometer calibration radius
    uint16_t mag_radius;       ///< Magnetometer calibration radius
} bno055_calib_profile_t;

/**
 * @brief BNO055 sensor instance structure
 * 
 * Contains all state, configuration, and data for a BNO055 sensor instance.
 * Initialize using bno055_init() before use.
 * 
 * Thread-safety: Not thread-safe. External synchronization required for
 * concurrent access from multiple tasks.
 */
typedef struct {
    // Hardware interface
    i2c_t i2c_handle;                       ///< I2C communication handle
    gpio_t rst_pin;                         ///< Reset pin control (optional)
    
    // Configuration
    bno055_operation_mode_t operation_mode; ///< Current operation mode
    bno055_power_mode_t power_mode;         ///< Current power mode
    bno055_unit_settings_t unit_settings;   ///< Unit configuration
    
    // Sensor identification
    uint8_t chip_id;                        ///< Chip ID (should be 0xA0)
    uint8_t sw_rev_id[2];                   ///< Software revision ID
    uint8_t page_id;                        ///< Current register page
    uint8_t accel_rev_id;                   ///< Accelerometer revision ID
    uint8_t mag_rev_id;                     ///< Magnetometer revision ID
    uint8_t gyro_rev_id;                    ///< Gyroscope revision ID
    uint8_t bl_rev_id;                      ///< Bootloader revision ID
    
    // Calibration status
    uint8_t calib_stat;                     ///< Calibration status register value
    uint8_t test_stat;                      ///< Self-test status register value
    
    // Sensor data (Euler angles)
    float yaw;                              ///< Heading/yaw angle
    float pitch;                            ///< Pitch angle
    float roll;                             ///< Roll angle
    
    // Sensor data (Linear acceleration)
    float ax;                               ///< Accelerometer X-axis
    float ay;                               ///< Accelerometer Y-axis
    float az;                               ///< Accelerometer Z-axis
    
    // Sensor data (Angular velocity)
    float gx;                               ///< Gyroscope X-axis
    float gy;                               ///< Gyroscope Y-axis
    float gz;                               ///< Gyroscope Z-axis
    
    // Sensor data (Magnetic field)
    float mx;                               ///< Magnetometer X-axis
    float my;                               ///< Magnetometer Y-axis
    float mz;                               ///< Magnetometer Z-axis
    
    // Internal buffer
    uint8_t buffer[128];                    ///< Communication buffer
} bno055_t;


// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

/**
 * @brief Initialize BNO055 IMU sensor
 * 
 * Configures I2C communication, sets default operation mode (NDOF), power mode
 * (NORMAL), and unit settings (m/s², rad/s, radians).
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[in] sda I2C SDA GPIO pin number
 * @param[in] scl I2C SCL GPIO pin number
 * @param[in] i2c_num I2C port number (0 or 1)
 * @return int8_t BNO055_SUCCESS (0) on success, BNO055_ERROR (-1) on failure
 * 
 * @note This function must be called before any other BNO055 functions
 */
int8_t bno055_init(bno055_t *bno055, uint8_t sda, uint8_t scl, uint8_t i2c_num);

/**
 * @brief Reset BNO055 sensor via I2C de-initialization
 * 
 * Deinitializes the I2C interface. Note: Hardware reset via RST pin is
 * currently not implemented.
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 */
void bno055_reset(bno055_t *bno055);

/**
 * @brief Get calibration status of all sensors
 * 
 * Reads the calibration status register and updates the internal calibration
 * status. Each sensor (system, gyro, accel, mag) has a 2-bit status (0-3).
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @return int8_t BNO055_SUCCESS if fully calibrated, BNO055_ERROR otherwise
 */
int8_t bno055_get_calibration_status(bno055_t *bno055);

/**
 * @brief Read sensor identification information
 * 
 * Retrieves chip ID, revision IDs for all sensors, software version,
 * bootloader version, and current page ID.
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_get_info(bno055_t *bno055);

/**
 * @brief Set the operation mode of the BNO055 sensor
 * 
 * ## Operating Modes:
 * 
 * | Mode Name       | Mode Value | Description                              |
 * |-----------------|------------|------------------------------------------|
 * | CONFIGMODE      | 0x00       | Configuration mode, disables all sensors |
 * | ACCONLY         | 0x01       | Accelerometer only                       |
 * | MAGONLY         | 0x02       | Magnetometer only                        |
 * | GYROONLY        | 0x03       | Gyroscope only                           |
 * | ACCMAG          | 0x04       | Accelerometer + Magnetometer             |
 * | ACCGYRO         | 0x05       | Accelerometer + Gyroscope                |
 * | MAGGYRO         | 0x06       | Magnetometer + Gyroscope                 |
 * | AMG             | 0x07       | Accelerometer + Magnetometer + Gyroscope |
 * | IMU             | 0x08       | Fusion: Accelerometer + Gyroscope        |
 * | COMPASS         | 0x09       | Fusion: Accelerometer + Magnetometer     |
 * | M4G             | 0x0A       | Fusion: Accelerometer + Gyroscope + Magnetometer |
 * | NDOF_FMC_OFF    | 0x0B       | Fusion: Full NDOF without fast magnetometer calibration |
 * | NDOF            | 0x0C       | Fusion: Full NDOF with calibration       |
 * 
 * @param mode Mode of operation 
 */
int8_t bno055_set_operation_mode(bno055_t *bno055, bno055_operation_mode_t mode);

/**
 * @brief Read Euler orientation angles
 * 
 * Retrieves absolute orientation as Euler angles (yaw/heading, pitch, roll)
 * from the sensor fusion algorithm. Units depend on configuration.
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[out] yaw Pointer to store yaw/heading angle
 * @param[out] pitch Pointer to store pitch angle
 * @param[out] roll Pointer to store roll angle
 */
void bno055_get_euler_angles(bno055_t *bno055, float *yaw, float *pitch, float *roll);

/**
 * @brief Read linear acceleration data
 * 
 * Retrieves linear acceleration values for all three axes. Units depend on
 * configuration (m/s² or mg).
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[out] x Pointer to store X-axis acceleration
 * @param[out] y Pointer to store Y-axis acceleration
 * @param[out] z Pointer to store Z-axis acceleration
 */
void bno055_get_acceleration(bno055_t *bno055, float *x, float *y, float *z);

/**
 * @brief Read gyroscope angular velocity data
 * 
 * Retrieves angular velocity values for all three axes. Units depend on
 * configuration (dps or rps).
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[out] gx Pointer to store X-axis angular velocity
 * @param[out] gy Pointer to store Y-axis angular velocity
 * @param[out] gz Pointer to store Z-axis angular velocity
 */
void bno055_get_gyro(bno055_t *bno055, float *gx, float *gy, float *gz);

/**
 * @brief Read magnetometer magnetic field data
 * 
 * Retrieves magnetic field strength values for all three axes in µT (microtesla).
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[out] mx Pointer to store X-axis magnetic field
 * @param[out] my Pointer to store Y-axis magnetic field
 * @param[out] mz Pointer to store Z-axis magnetic field
 */
void bno055_get_magnetometer(bno055_t *bno055, float *mx, float *my, float *mz);

// =============================================================================
// LOW-LEVEL I2C COMMUNICATION FUNCTIONS
// =============================================================================

/**
 * @brief Write data to BNO055 register via I2C
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[in] reg Register address
 * @param[in] data Pointer to data buffer to write
 * @param[in] len Number of bytes to write
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_write(bno055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Read data from BNO055 register via I2C
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[in] reg Register address
 * @param[out] data Pointer to buffer to store read data
 * @param[in] len Number of bytes to read
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_read(bno055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Read all sensor data in a single operation
 * 
 * Efficiently reads accelerometer, magnetometer, gyroscope, and Euler angle
 * data in one I2C transaction (24 bytes total).
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_read_all(bno055_t *bno055);

/**
 * @brief Read all sensor data using linear acceleration
 * 
 * Similar to bno055_read_all() but uses linear acceleration (gravity removed)
 * instead of raw acceleration data.
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_read_all_linear(bno055_t *bno055);

/**
 * @brief Configure output units for all sensors
 * 
 * Sets the units for accelerometer, gyroscope, Euler angles, temperature,
 * and orientation convention.
 * 
 * Unit options:
 * - accel_unit: 0=m/s², 1=mg
 * - gyro_unit: 0=dps, 1=rps
 * - euler_unit: 0=degrees, 1=radians
 * - temp_unit: 0=Celsius, 1=Fahrenheit
 * - ori_unit: 0=Windows, 1=Android
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[in] accel_unit Accelerometer unit selection
 * @param[in] gyro_unit Gyroscope unit selection
 * @param[in] euler_unit Euler angle unit selection
 * @param[in] temp_unit Temperature unit selection
 * @param[in] ori_unit Orientation convention selection
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_set_unit(bno055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, 
                       uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit);

/**
 * @brief Set power mode
 * 
 * Controls power consumption by enabling/disabling sensors and processing.
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[in] mode Power mode to set
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
int8_t bno055_set_power_mode(bno055_t *bno055, bno055_power_mode_t mode);

/**
 * @brief Read complete calibration profile
 * 
 * Retrieves all calibration offsets and radius values for accelerometer,
 * magnetometer, and gyroscope. Can be saved and restored to skip calibration.
 * 
 * @param[in,out] bno055 Pointer to BNO055 sensor instance
 * @param[out] calib_data Pointer to structure to store calibration data
 * @return int8_t BNO055_SUCCESS on success, BNO055_ERROR on failure
 * 
 * @note Sensor must be fully calibrated before calling this function
 */
int8_t bno055_get_calibration_profile(bno055_t *bno055, bno055_calib_profile_t *calib_data);

#ifdef __cplusplus
}
#endif

#endif // BNO055_H