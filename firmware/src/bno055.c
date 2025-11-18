/**
 * @file bno055.c
 * @brief BNO055 9-axis IMU sensor driver implementation
 * 
 * Provides I2C-based communication with the BNO055 sensor for reading
 * orientation, acceleration, gyroscope, and magnetometer data.
 * 
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @version 0.2
 * @date 2024-11-25
 * @copyright Copyright (c) 2024
 */

#include "bno055.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

// =============================================================================
// PRIVATE CONSTANTS
// =============================================================================

static const char *TAG = "BNO055";

// =============================================================================
// INTERNAL HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Write data to BNO055 register via I2C
 * 
 * @param bno055 Pointer to BNO055 instance
 * @param reg Register address
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
static int8_t bno055_write(bno055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        ESP_LOGE(TAG, "Null pointer in write operation");
        return BNO055_ERROR;
    }

    if (!i2c_write_reg(&bno055->i2c_handle, reg, data, len)) {
        ESP_LOGE(TAG, "I2C write failed at register 0x%02X", reg);
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

/**
 * @brief Read data from BNO055 register via I2C
 * 
 * @param bno055 Pointer to BNO055 instance
 * @param reg Register address
 * @param data Pointer to buffer for read data
 * @param len Number of bytes to read
 * @return BNO055_SUCCESS on success, BNO055_ERROR on failure
 */
static int8_t bno055_read(bno055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        ESP_LOGE(TAG, "Null pointer in read operation");
        return BNO055_ERROR;
    }

    if (!i2c_read_reg(&bno055->i2c_handle, reg, data, len)) {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

/**
 * @brief Convert raw accelerometer data to float
 * 
 * @param bno055 Pointer to BNO055 instance (for unit settings)
 * @param data Raw data buffer (6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
 * @param x Output X-axis value
 * @param y Output Y-axis value
 * @param z Output Z-axis value
 */
static void bno055_convert_accel_data(bno055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "Null pointer in data conversion");
        return;
    }

    // Convert 16-bit little-endian to signed integer
    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    // Apply scale factor based on unit
    if (bno055->unit_settings.accel_unit == BNO055_ACCEL_UNIT_MSQ) {
        // 1 m/s² = 100 LSB
        *x = (float)raw_x / 100.0f;
        *y = (float)raw_y / 100.0f;
        *z = (float)raw_z / 100.0f;
    } else {
        // 1 mg = 1 LSB
        *x = (float)raw_x;
        *y = (float)raw_y;
        *z = (float)raw_z;
    }
}

/**
 * @brief Convert raw gyroscope data to float
 * 
 * @param bno055 Pointer to BNO055 instance (for unit settings)
 * @param data Raw data buffer (6 bytes)
 * @param x Output X-axis angular velocity
 * @param y Output Y-axis angular velocity
 * @param z Output Z-axis angular velocity
 */
static void bno055_convert_gyro_data(bno055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "Null pointer in data conversion");
        return;
    }

    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    if (bno055->unit_settings.gyro_unit == BNO055_GYRO_UNIT_DPS) {
        // 1 dps = 16 LSB
        *x = (float)raw_x / 16.0f;
        *y = (float)raw_y / 16.0f;
        *z = (float)raw_z / 16.0f;
    } else {
        // 1 rps = 900 LSB
        *x = (float)raw_x / 900.0f;
        *y = (float)raw_y / 900.0f;
        *z = (float)raw_z / 900.0f;
    }
}

/**
 * @brief Convert raw Euler angle data to float
 * 
 * @param bno055 Pointer to BNO055 instance (for unit settings)
 * @param data Raw data buffer (6 bytes: yaw, pitch, roll)
 * @param yaw Output yaw angle
 * @param pitch Output pitch angle
 * @param roll Output roll angle
 */
static void bno055_convert_euler_data(bno055_t *bno055, uint8_t *data, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || data == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        ESP_LOGE(TAG, "Null pointer in data conversion");
        return;
    }

    int16_t raw_yaw = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_pitch = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_roll = (int16_t)(data[5] << 8 | data[4]);

    if (bno055->unit_settings.euler_unit == BNO055_EULER_UNIT_DEG) {
        // 1 degree = 16 LSB
        *yaw = (float)raw_yaw / 16.0f;
        *pitch = (float)raw_pitch / 16.0f;
        *roll = (float)raw_roll / 16.0f;
    } else {
        // 1 radian = 900 LSB
        *yaw = (float)raw_yaw / 900.0f;
        *pitch = (float)raw_pitch / 900.0f;
        *roll = (float)raw_roll / 900.0f;
    }
}

/**
 * @brief Convert raw magnetometer data to float
 * 
 * @param bno055 Pointer to BNO055 instance (for unit settings)
 * @param data Raw data buffer (6 bytes)
 * @param x Output X-axis magnetic field
 * @param y Output Y-axis magnetic field
 * @param z Output Z-axis magnetic field
 */
static void bno055_convert_mag_data(bno055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "Null pointer in data conversion");
        return;
    }

    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    // 1 µT = 16 LSB (unit is fixed for magnetometer)
    *x = (float)raw_x / 16.0f;
    *y = (float)raw_y / 16.0f;
    *z = (float)raw_z / 16.0f;
}

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

int8_t bno055_init(bno055_t *bno055, uint8_t sda, uint8_t scl, uint8_t i2c_num)
{
    int8_t success = BNO055_SUCCESS;

    ESP_LOGI(TAG, "Initializing BNO055 sensor...");

    // Initialize I2C communication
    if (!i2c_init(&bno055->i2c_handle, i2c_num, scl, sda, BNO055_I2C_FREQ_HZ, BNO055_SENSOR_ADDR)) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return BNO055_ERROR;
    }

    // Set operation mode to CONFIG for configuration
    bno055->operation_mode = BNO055_MODE_INIT;
    success += bno055_set_operation_mode(bno055, BNO055_MODE_CONFIG);
    
    // Write default page as zero
    uint8_t data = BNO055_PAGE_ZERO;
    success += bno055_write(bno055, BNO055_PAGE_ID_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
    
    // Set units: m/s², dps, radians, Celsius, Android orientation
    success += bno055_set_unit(bno055, BNO055_ACCEL_UNIT_MSQ, BNO055_GYRO_UNIT_DPS, 
                               BNO055_EULER_UNIT_RAD, BNO055_TEMP_UNIT_CELSIUS, 
                               BNO055_ANDROID_ORIENTATION);
    
    // Set power mode to normal
    bno055->power_mode = BNO055_POWER_LOWPOWER;
    success += bno055_set_power_mode(bno055, BNO055_POWER_NORMAL);
    
    // Set operation mode to NDOF (full sensor fusion)
    success += bno055_set_operation_mode(bno055, BNO055_MODE_NDOF);
    
    // Get device information
    success += bno055_get_info(bno055);

    // Log sensor information
    ESP_LOGI(TAG, "----------------- BNO055 Sensor Data -----------------");
    ESP_LOGI(TAG, "Operation mode: 0x%02X", bno055->operation_mode);
    ESP_LOGI(TAG, "Power mode: 0x%02X", bno055->power_mode);
    ESP_LOGI(TAG, "Chip ID: 0x%02X", bno055->chip_id);
    ESP_LOGI(TAG, "Accel Rev ID: 0x%02X", bno055->accel_rev_id);
    ESP_LOGI(TAG, "Mag Rev ID: 0x%02X", bno055->mag_rev_id);
    ESP_LOGI(TAG, "Gyro Rev ID: 0x%02X", bno055->gyro_rev_id);
    ESP_LOGI(TAG, "Bootloader Rev ID: 0x%02X", bno055->bl_rev_id);
    ESP_LOGI(TAG, "SW Rev ID: 0x%02X%02X", bno055->sw_rev_id[0], bno055->sw_rev_id[1]);
    ESP_LOGI(TAG, "Page ID: 0x%02X", bno055->page_id);
    ESP_LOGI(TAG, "------------------------------------------------------");

    if (success == BNO055_SUCCESS) {
        ESP_LOGI(TAG, "BNO055 sensor initialized successfully");
        return BNO055_SUCCESS;
    } else {
        ESP_LOGE(TAG, "Failed to initialize BNO055 sensor");
        return BNO055_ERROR;
    }
}

void bno055_reset(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null pointer in reset");
        return;
    }
    
    i2c_deinit(&bno055->i2c_handle);
    ESP_LOGI(TAG, "BNO055 I2C deinitialized");
    
    // Optional: Hardware reset via GPIO if configured
    // gpio_set_low(&bno055->rst_pin);
    // vTaskDelay(pdMS_TO_TICKS(10));
    // gpio_set_high(&bno055->rst_pin);
    // vTaskDelay(pdMS_TO_TICKS(10));
}

int8_t bno055_get_calibration_status(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_calibration_status");
        return BNO055_ERROR;
    }

    uint8_t calib_status = 0;
    int8_t success = bno055_read(bno055, BNO055_CALIB_STAT_ADDR, &calib_status, 1);

    if (success != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read calibration status");
        return BNO055_ERROR;
    }

    // Update cached status
    bno055->calib_stat = calib_status;

    // Extract individual calibration levels (each 2 bits)
    uint8_t sys_calib = (calib_status >> 6) & 0x03;
    uint8_t gyro_calib = (calib_status >> 4) & 0x03;
    uint8_t accel_calib = (calib_status >> 2) & 0x03;
    uint8_t mag_calib = calib_status & 0x03;

    ESP_LOGI(TAG, "Calibration: SYS=%d GYRO=%d ACCEL=%d MAG=%d", 
             sys_calib, gyro_calib, accel_calib, mag_calib);

    // Check if fully calibrated (all sensors at level 3)
    if (calib_status == BNO055_CALIB_STAT_OK) {
        ESP_LOGI(TAG, "System fully calibrated");
        return BNO055_SUCCESS;
    } else {
        return BNO055_ERROR;
    }
}

int8_t bno055_get_info(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_info");
        return BNO055_ERROR;
    }
    
    uint8_t data_read[8] = {0};
    int8_t success = bno055_read(bno055, BNO055_CHIP_ID_ADDR, data_read, 8);

    if (success != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read device information");
        return BNO055_ERROR;
    }

    // Parse device information
    bno055->chip_id = data_read[0];
    bno055->accel_rev_id = data_read[1];
    bno055->mag_rev_id = data_read[2];
    bno055->gyro_rev_id = data_read[3];
    bno055->sw_rev_id[0] = data_read[4];  // LSB
    bno055->sw_rev_id[1] = data_read[5];  // MSB
    bno055->bl_rev_id = data_read[6];
    bno055->page_id = data_read[7];

    return BNO055_SUCCESS;
}

void bno055_get_euler_angles(bno055_t *bno055, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_euler_angles");
        return;
    }

    uint8_t data[6] = {0};
    int8_t success = bno055_read(bno055, BNO055_EULER_H_LSB_ADDR, data, 6);

    // If read fails, return cached values
    if (success != BNO055_SUCCESS) {
        *yaw = bno055->yaw;
        *pitch = bno055->pitch;
        *roll = bno055->roll;
        return;
    }

    // Convert and update
    bno055_convert_euler_data(bno055, data, yaw, pitch, roll);
    bno055->yaw = *yaw;
    bno055->pitch = *pitch;
    bno055->roll = *roll;
}

// Note: bno055_write is already defined as static helper function above

// UART communication not currently used - removed for clarity

// UART read function removed - using I2C only

// Note: bno055_read is already defined as static helper function above

// CheckAck function removed - not needed for I2C communication

int8_t bno055_read_all(bno055_t *bno055)
{
    uint8_t data[24]; // 24 bytes: accel(6) + mag(6) + gyro(6) + euler(6)
    int8_t result = bno055_read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 24);

    if (result != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return BNO055_ERROR;
    }

    // Split data into sensor-specific buffers
    uint8_t *data_accel = &data[0];
    uint8_t *data_mag = &data[6];
    uint8_t *data_gyro = &data[12];
    uint8_t *data_euler = &data[18];

    // Convert and cache all sensor readings
    bno055_convert_accel_data(bno055, data_accel, &bno055->ax, &bno055->ay, &bno055->az);
    bno055_convert_mag_data(bno055, data_mag, &bno055->mx, &bno055->my, &bno055->mz);
    bno055_convert_gyro_data(bno055, data_gyro, &bno055->gx, &bno055->gy, &bno055->gz);
    bno055_convert_euler_data(bno055, data_euler, &bno055->yaw, &bno055->pitch, &bno055->roll);

    return BNO055_SUCCESS;
}

int8_t bno055_read_all_linear(bno055_t *bno055)
{
    uint8_t data[24];
    
    // Read linear acceleration (gravity removed by fusion algorithm)
    int8_t result1 = bno055_read(bno055, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, data, 6);
    // Read mag, gyro, and euler (18 bytes)
    int8_t result2 = bno055_read(bno055, BNO055_MAG_DATA_X_LSB_ADDR, data + 6, 18);

    if (result1 != BNO055_SUCCESS || result2 != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read linear acceleration data");
        return BNO055_ERROR;
    }

    // Split and convert
    uint8_t *data_accel = &data[0];
    uint8_t *data_mag = &data[6];
    uint8_t *data_gyro = &data[12];
    uint8_t *data_euler = &data[18];

    bno055_convert_accel_data(bno055, data_accel, &bno055->ax, &bno055->ay, &bno055->az);
    bno055_convert_mag_data(bno055, data_mag, &bno055->mx, &bno055->my, &bno055->mz);
    bno055_convert_gyro_data(bno055, data_gyro, &bno055->gx, &bno055->gy, &bno055->gz);
    bno055_convert_euler_data(bno055, data_euler, &bno055->yaw, &bno055->pitch, &bno055->roll);

    return BNO055_SUCCESS;
}

int8_t bno055_set_operation_mode(bno055_t *bno055, bno055_operation_mode_t mode)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null pointer in set_operation_mode");
        return BNO055_ERROR;
    }

    // Only write if mode actually changes
    if (mode != bno055->operation_mode) {
        uint8_t data = (uint8_t)mode;
        int8_t success = bno055_write(bno055, BNO055_OPR_MODE_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
        if (success == BNO055_SUCCESS) {
            bno055->operation_mode = mode;
        }
        return success;
    }

    return BNO055_SUCCESS;
}

void bno055_get_acceleration(bno055_t *bno055, float *x, float *y, float *z)
{
    if (bno055 == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_acceleration");
        return;
    }

    uint8_t data[6];
    if (bno055_read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 6) == BNO055_SUCCESS) {
        bno055_convert_accel_data(bno055, data, x, y, z);
    }
}

void bno055_get_gyro(bno055_t *bno055, float *gx, float *gy, float *gz)
{
    if (bno055 == NULL || gx == NULL || gy == NULL || gz == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_gyro");
        return;
    }

    uint8_t data[6];
    if (bno055_read(bno055, BNO055_GYRO_DATA_X_LSB_ADDR, data, 6) == BNO055_SUCCESS) {
        bno055_convert_gyro_data(bno055, data, gx, gy, gz);
    }
}

void bno055_get_magnetometer(bno055_t *bno055, float *mx, float *my, float *mz)
{
    if (bno055 == NULL || mx == NULL || my == NULL || mz == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_magnetometer");
        return;
    }

    uint8_t data[6];
    if (bno055_read(bno055, BNO055_MAG_DATA_X_LSB_ADDR, data, 6) == BNO055_SUCCESS) {
        bno055_convert_mag_data(bno055, data, mx, my, mz);
    }
}

int8_t bno055_set_unit(bno055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, 
                       uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null pointer in set_unit");
        return BNO055_ERROR;
    }

    // Build unit configuration byte
    uint8_t data = 0x00;
    data |= (accel_unit & 0x01) << 0;
    data |= (gyro_unit & 0x01) << 1;
    data |= (euler_unit & 0x01) << 2;
    data |= (temp_unit & 0x01) << 4;
    data |= (ori_unit & 0x01) << 7;

    int8_t success = bno055_write(bno055, BNO055_UNIT_SEL_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);

    if (success == BNO055_SUCCESS) {
        // Update cached settings
        bno055->unit_settings.accel_unit = accel_unit;
        bno055->unit_settings.gyro_unit = gyro_unit;
        bno055->unit_settings.euler_unit = euler_unit;
        bno055->unit_settings.temp_unit = temp_unit;
        bno055->unit_settings.ori_unit = ori_unit;
    }

    return success;
}

int8_t bno055_set_power_mode(bno055_t *bno055, bno055_power_mode_t mode)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "Null pointer in set_power_mode");
        return BNO055_ERROR;
    }

    // Only write if mode changes
    if (mode != bno055->power_mode) {
        uint8_t data = (uint8_t)mode;
        int8_t success = bno055_write(bno055, BNO055_PWR_MODE_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
        if (success == BNO055_SUCCESS) {
            bno055->power_mode = mode;
        }
        return success;
    }

    return BNO055_SUCCESS;
}

int8_t bno055_get_calibration_profile(bno055_t *bno055, bno055_calib_profile_t *calib_data)
{
    if (bno055 == NULL || calib_data == NULL) {
        ESP_LOGE(TAG, "Null pointer in get_calibration_profile");
        return BNO055_ERROR;
    }

    // Verify sensor is calibrated
    bno055_get_calibration_status(bno055);
    if (bno055->calib_stat != BNO055_CALIB_STAT_OK) {
        ESP_LOGW(TAG, "Sensor not fully calibrated, profile may be invalid");
        return BNO055_ERROR;
    }

    // Save current mode and switch to CONFIG
    bno055_operation_mode_t current_mode = bno055->operation_mode;
    int8_t success = bno055_set_operation_mode(bno055, BNO055_MODE_CONFIG);
    if (success != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enter CONFIG mode");
        return BNO055_ERROR;
    }

    // Read calibration offsets (22 bytes)
    uint8_t calib_offsets[22] = {0};
    success = bno055_read(bno055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, calib_offsets, 22);
    if (success != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read calibration offsets");
        bno055_set_operation_mode(bno055, current_mode);
        return BNO055_ERROR;
    }

    // Parse offsets (little-endian 16-bit values)
    calib_data->accel_offset_x = (int16_t)(calib_offsets[1] << 8 | calib_offsets[0]);
    calib_data->accel_offset_y = (int16_t)(calib_offsets[3] << 8 | calib_offsets[2]);
    calib_data->accel_offset_z = (int16_t)(calib_offsets[5] << 8 | calib_offsets[4]);

    calib_data->mag_offset_x = (int16_t)(calib_offsets[7] << 8 | calib_offsets[6]);
    calib_data->mag_offset_y = (int16_t)(calib_offsets[9] << 8 | calib_offsets[8]);
    calib_data->mag_offset_z = (int16_t)(calib_offsets[11] << 8 | calib_offsets[10]);

    calib_data->gyro_offset_x = (int16_t)(calib_offsets[13] << 8 | calib_offsets[12]);
    calib_data->gyro_offset_y = (int16_t)(calib_offsets[15] << 8 | calib_offsets[14]);
    calib_data->gyro_offset_z = (int16_t)(calib_offsets[17] << 8 | calib_offsets[16]);

    calib_data->accel_radius = (int16_t)(calib_offsets[19] << 8 | calib_offsets[18]);
    calib_data->mag_radius = (int16_t)(calib_offsets[21] << 8 | calib_offsets[20]);

    // Restore previous operation mode
    success = bno055_set_operation_mode(bno055, current_mode);
    if (success != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to restore operation mode");
    }

    // Log calibration profile
    ESP_LOGI(TAG, "------------------------------------------------------");
    ESP_LOGI(TAG, "Calibration Profile:");
    ESP_LOGI(TAG, "Accel Offsets: X=0x%04X Y=0x%04X Z=0x%04X", 
             calib_data->accel_offset_x, calib_data->accel_offset_y, calib_data->accel_offset_z);
    ESP_LOGI(TAG, "Mag Offsets: X=0x%04X Y=0x%04X Z=0x%04X", 
             calib_data->mag_offset_x, calib_data->mag_offset_y, calib_data->mag_offset_z);
    ESP_LOGI(TAG, "Gyro Offsets: X=0x%04X Y=0x%04X Z=0x%04X", 
             calib_data->gyro_offset_x, calib_data->gyro_offset_y, calib_data->gyro_offset_z);
    ESP_LOGI(TAG, "Accel Radius: 0x%04X, Mag Radius: 0x%04X", 
             calib_data->accel_radius, calib_data->mag_radius);
    ESP_LOGI(TAG, "------------------------------------------------------");

    return BNO055_SUCCESS;
}