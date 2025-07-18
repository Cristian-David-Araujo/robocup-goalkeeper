/**
 * @file bno055.h
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef BNO055_H
#define BNO055_H

// Standard C includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ESP32-S3 hardware abstraction layer (Your platform may have a different HAL)
#include "platform_esp32s3.h"

// Constants for BNO055 include
#include "bno055_defs.h"

#define I2C_MASTER_FREQ_HZ  400*1000    /*!< I2C master clock frequency */
#define BNO055_SENSOR_ADDR  0x29        /*!< slave address for BNO055 sensor */

/**
 * @brief Enumerated type for the operation mode of the BNO055 sensor
 * 
 */
typedef enum {
    CONFIGMODE = 0x00,
    ACCONLY = 0x01,
    MAGONLY = 0x02,
    GYROONLY = 0x03,
    ACCMAG = 0x04,
    ACCGYRO = 0x05,
    MAGGYRO = 0x06,
    AMG = 0x07,
    IMU = 0x08,
    COMPASS = 0x09,
    M4G = 0x0A,
    NDOF_FMC_OFF = 0x0B,
    NDOF = 0x0C,
    INIT = 0x0D
} BNO055_OperationMode;

/**
 * @brief Enum power mode of the BNO055 sensor
 * 
 */
typedef enum {
    NORMAL = BNO055_POWER_MODE_NORMAL,
    LOWPOWER = BNO055_POWER_MODE_LOWPOWER,
    SUSPEND = BNO055_POWER_MODE_SUSPEND
} BNO055_PowerMode;


/**
 * @brief Structure for save the actual unit of the BNO055 sensor
 * 
 */
typedef struct {
    uint8_t accel_unit; ///> Accelerometer unit
    uint8_t gyro_unit; ///> Gyroscope unit
    uint8_t euler_unit; ///> Euler unit
    uint8_t temp_unit; ///> Temperature unit
    uint8_t ori_unit; ///> Orientation unit
} BNO055_UnitSettings_t;

/**
 * @brief Srtucture for calibration profile of the BNO055 sensor
 * 
 * This structure is used to save the calibration profile of the BNO055 sensor.
 */
typedef struct {
    uint8_t sys_calib_stat; ///> System calibration status
    uint16_t accel_offset_x; ///> Accelerometer offset X
    uint16_t accel_offset_y; ///> Accelerometer offset Y
    uint16_t accel_offset_z; ///> Accelerometer offset Z
    uint16_t mag_offset_x; ///> Magnetometer offset X
    uint16_t mag_offset_y; ///> Magnetometer offset Y
    uint16_t mag_offset_z; ///> Magnetometer offset Z
    uint16_t gyro_offset_x; ///> Gyroscope offset X
    uint16_t gyro_offset_y; ///> Gyroscope offset Y
    uint16_t gyro_offset_z; ///> Gyroscope offset Z
    uint16_t accel_radius; ///> Accelerometer radius
    uint16_t mag_radius; ///> Magnetometer radius
} BNO055_CalibProfile_t;

/**
 * @brief Structure for the BNO055 sensor
 * 
 */
typedef struct {
    uart_t uart_config; ///> UART configuration structure
    i2c_t i2c_handle;   ///< I2C handle for the BNO055 sensor
    BNO055_OperationMode operation_mode; ///> Operation mode of the BNO055 sensor
    BNO055_PowerMode power_mode; ///> Power mode of the BNO055 sensor
    BNO055_UnitSettings_t unit_settings; ///> Unit settings of the BNO055 sensor

    ///> Buffer for data
    uint8_t buffer[128]; ///> Buffer for data

    ///> BNO055 sensor data
    uint8_t chip_id; ///> Chip ID
    uint8_t sw_rev_id[2]; ///> Software revision ID
    uint8_t page_id; ///> Page ID
    uint8_t accel_rev_id; ///> Accelerometer revision ID
    uint8_t mag_rev_id; ///> Magnetometer revision ID
    uint8_t gyro_rev_id; ///> Gyroscope revision ID
    uint8_t bl_rev_id; ///> Bootloader revision ID

    ///> Calibration data
    uint8_t calib_stat; ///> Calibration status
    uint8_t test_stat; ///> Self test status

    ///> data
    float yaw; ///> Yaw value
    float pitch; ///> Pitch value
    float roll; ///> Roll value

    float ax; ///> Accelerometer X value
    float ay; ///> Accelerometer Y value
    float az; ///> Accelerometer Z value

    float gx; ///> Gyroscope X value
    float gy; ///> Gyroscope Y value
    float gz; ///> Gyroscope Z value

    float mx; ///> Magnetometer X value
    float my; ///> Magnetometer Y value
    float mz; ///> Magnetometer Z value

    gpio_t rst_pin; ///> Reset pin
    
} BNO055_t;


/**
 * @brief Initialize BNO055 sensor
 * 
 * This function initializes the BNO055 sensor with the specified I2c (or uart) configuration.
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 */
int8_t BNO055_Init(BNO055_t *bno055, uint8_t sda, uint8_t scl, uint8_t i2c_num);

/**
 * @brief Reset the BNO055 sensor
 * 
 * @param bno055 
 */
void BNO055_Reset(BNO055_t *bno055);

/**
 * @brief Get the calibration status of the BNO055 sensor
 * 
 * This function retrieves the calibration status of the BNO055 sensor
 * and returns the system, gyroscope, accelerometer, and magnetometer calibration status.
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 */
int8_t BNO055_GetCalibrationStatus(BNO055_t *bno055);

/**
 * @brief Get information from the BNO055 sensor
 * 
 * This function retrieves the chip ID, software revision ID, page ID, accelerometer revision ID,
 * magnetometer revision ID, gyroscope revision ID, and bootloader revision ID from the BNO055 sensor.
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 */
int8_t BNO055_GetInfo(BNO055_t *bno055);

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
int8_t BNO055_SetOperationMode(BNO055_t *bno055, BNO055_OperationMode mode);

/**
 * @brief Get the operation mode of the BNO055 sensor
 * 
 * This function retrieves the current operation mode of the BNO055 sensor
 * and returns the yaw, pitch, and roll values.
 * 
 * @param yaw Pointer to a variable where the yaw value will be stored
 * @param pitch Pointer to a variable where the pitch value will be stored
 * @param roll Pointer to a variable where the roll value will be stored
 */
void BNO055_GetEulerAngles(BNO055_t *bno055, float *yaw , float *pitch , float *roll); ///> Getorientation data ( Euler angles )

/**
 * @brief Get the linear acceleration of the BNO055 sensor
 * 
 * This function retrieves the linear acceleration of the BNO055 sensor
 * and returns the x, y, and z values.
 * 
 * @param x Pointer to a variable where the x value will be stored
 * @param y Pointer to a variable where the y value will be stored
 * @param z Pointer to a variable where the z value will be stored
 */
void BNO055_GetAcceleration(BNO055_t *bno055, float *x, float *y, float *z);///> Get linear acceleration

/**
 * @brief Get the gyroscope data of the BNO055 sensor
 * 
 * @param bno055 Structure with the BNO055 sensor data
 * @param gx Pointer to a variable where the gx value will be stored
 * @param gy Pointer to a variable where the gy value will be stored
 * @param gz Pointer to a variable where the gz value will be stored
 */
void BNO055_GetGyro(BNO055_t *bno055, float *gx , float *gy , float *gz); ///> Get gyroscope data

/**
 * @brief Get the magnetic field of the BNO055 sensor
 * 
 * This function retrieves the magnetic field of the BNO055 sensor
 * and returns the x, y, and z values.
 * 
 * @param mx Pointer to a variable where the mx value will be stored
 * @param my Pointer to a variable where the my value will be stored
 * @param mz Pointer to a variable where the mz value will be stored
 */
void BNO055_GetMagnetometer(BNO055_t *bno055, float *mx , float *my , float *mz);///> Get magnetometer data

/**
 * @brief Fuctión to send data to the BNO055 sensor for UART communication
 * 
 * @param bno055
 * @param reg Address of the register to write in HEX
 * @param data Pointer to the data to write
 * @param len Length of the data to write in bytes
 */
 int8_t BN055_Write_Uart(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Fuctión to send data to the BNO055 sensor for I2C communication
 * 
 * @param bno055
 * @param reg Address of the register to write in HEX
 * @param data Pointer to the data to write
 * @param len Length of the data to write in bytes
 */
int8_t BN055_Write(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Function to read data from the BNO055 sensor for UART communication
 * 
 * @param bno055
 * @param reg Address of the register to read in HEX
 * @param data Pointer to the data to read
 * @param len Length of the data to read in bytes
 * @param timeout_ms Timeout in milliseconds
 * @return int8_t
 */
int8_t BNO055_Read_Uart(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len, uint8_t timeout_ms);

/**
 * @brief Function to read data from the BNO055 sensor for I2C communication
 * 
 * @param bno055
 * @param reg Address of the register to read in HEX
 * @param data Pointer to the data to read
 * @param len Length of the data to read in bytes
 * @return int8_t
 */
 int8_t BNO055_Read(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Check the ACK value of the BNO055 sensor
 * 
 * @param data Received data from the BNO055 sensor
 * @return int8_t 
 */
int8_t BNO055_CheckAck(uint8_t *data);

/**
 * @brief Read all data from the BNO055 sensor with I2C (or uart)
 * 
 * @param bno055
 * @return int8_t 
 */
int8_t BNO055_ReadAll(BNO055_t *bno055);

/**
 * @brief Read all data from the BNO055 sensor with I2C (or uart) with Lineal Acceleration
 * 
 * @param bno055
 * @return int8_t 
 */
int8_t BNO055_ReadAll_Lineal(BNO055_t *bno055);

/**
 * @brief Set the unit of the BNO055 sensor
 * 
 * ## Bit Values:
 * 
 * Bit 0 Acceleration unit (0: m/s^2, 1: mg)
 * Bit 1 Angular rate unit (0: dps, 1: rps)
 * Bit 2 Euler unit (0: degrees, 1: radians)
 * Bit 4 Temperature unit (0: Celsius, 1: Fahrenheit)
 * Bit 7 Orientation unit (0: Windows, 1: Android)
 * 
 * @param bno055 
 * @param unit  Bit value for the unit
 * @return uint8_t 
 */
int8_t BNO055_SetUnit(BNO055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit);

/**
 * @brief Set the power mode of the BNO055 sensor
 * 
 * @param bno055 
 * @param mode 
 * @return int8_t 
 */
int8_t BNO055_SetPowerMode(BNO055_t *bno055, BNO055_PowerMode mode);

/**
 * @brief Coverting the data from the BNO055 sensor to the correct units
 * 
 */
void BNO055_ConvertData(BNO055_t *bno055);

/**
 * @brief Get the calibration profile of the BNO055 sensor
 * 
 * This function retrieves the calibration profile of the BNO055 sensor
 * and returns the system, gyroscope, accelerometer, and magnetometer calibration offsets and radius.
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 * @param calib_data Pointer to the calibration profile structure
 */
int8_t BNO055_GetCalibrationProfile(BNO055_t *bno055, BNO055_CalibProfile_t *calib_data);

#endif ///> BNO055_H