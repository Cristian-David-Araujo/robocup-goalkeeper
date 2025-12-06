/**
 * @file bno055.c
 * @brief BNO055 9-DOF IMU sensor driver implementation
 * 
 * Implements I2C communication and sensor data processing for the Bosch BNO055
 * Intelligent 9-axis Absolute Orientation Sensor.
 * 
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @date 2024-11-08
 */

#include "bno055.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "BNO055_DRIVER";

// =============================================================================
// PRIVATE HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Convert raw accelerometer data to float
 * 
 * Converts 6 bytes of raw data to three float values based on configured units.
 * 
 * @param bno055 Pointer to sensor instance (for unit settings)
 * @param data Raw data buffer (6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
 * @param x Output X-axis value
 * @param y Output Y-axis value
 * @param z Output Z-axis value
 */
static void convert_accel_data(bno055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGW(TAG, "convert_accel_data: NULL pointer provided");
        return;
    }

    // Convert raw bytes to signed 16-bit values
    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    // Scale based on unit setting
    if (bno055->unit_settings.accel_unit == BNO055_ACCEL_UNIT_MSQ) {
        // m/s² unit: 1 m/s² = 100 LSB
        *x = (float)raw_x / 100.0f;
        *y = (float)raw_y / 100.0f;
        *z = (float)raw_z / 100.0f;
    } else {
        // mg unit: 1 mg = 1 LSB
        *x = (float)raw_x;
        *y = (float)raw_y;
        *z = (float)raw_z;
    }
}

/**
 * @brief Convert raw gyroscope data to float
 * 
 * @param bno055 Pointer to sensor instance
 * @param data Raw data buffer (6 bytes)
 * @param x Output X-axis angular velocity
 * @param y Output Y-axis angular velocity
 * @param z Output Z-axis angular velocity
 */
static void convert_gyro_data(bno055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGW(TAG, "convert_gyro_data: NULL pointer provided");
        return;
    }

    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    if (bno055->unit_settings.gyro_unit == BNO055_GYRO_UNIT_DPS) {
        // dps unit: 1 dps = 16 LSB
        *x = (float)raw_x / 16.0f;
        *y = (float)raw_y / 16.0f;
        *z = (float)raw_z / 16.0f;
    } else {
        // rps unit: 1 rps = 900 LSB
        *x = (float)raw_x / 900.0f;
        *y = (float)raw_y / 900.0f;
        *z = (float)raw_z / 900.0f;
    }
}

/**
 * @brief Convert raw Euler angle data to float
 * 
 * @param bno055 Pointer to sensor instance
 * @param data Raw data buffer (6 bytes: H_LSB, H_MSB, R_LSB, R_MSB, P_LSB, P_MSB)
 * @param yaw Output yaw/heading angle
 * @param pitch Output pitch angle
 * @param roll Output roll angle
 */
static void convert_euler_data(bno055_t *bno055, uint8_t *data, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || data == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        ESP_LOGW(TAG, "convert_euler_data: NULL pointer provided");
        return;
    }

    int16_t raw_yaw = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_pitch = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_roll = (int16_t)(data[5] << 8 | data[4]);

    if (bno055->unit_settings.euler_unit == BNO055_EULER_UNIT_DEG) {
        // degrees unit: 1 degree = 16 LSB
        *yaw = (float)raw_yaw / 16.0f;
        *pitch = (float)raw_pitch / 16.0f;
        *roll = (float)raw_roll / 16.0f;
    } else {
        // radians unit: 1 radian = 900 LSB
        *yaw = (float)raw_yaw / 900.0f;
        *pitch = (float)raw_pitch / 900.0f;
        *roll = (float)raw_roll / 900.0f;
    }
}

/**
 * @brief Convert raw magnetometer data to float
 * 
 * @param bno055 Pointer to sensor instance
 * @param data Raw data buffer (6 bytes)
 * @param x Output X-axis magnetic field
 * @param y Output Y-axis magnetic field
 * @param z Output Z-axis magnetic field
 */
static void convert_mag_data(bno055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGW(TAG, "convert_mag_data: NULL pointer provided");
        return;
    }

    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    // Magnetometer unit is always µT: 1 µT = 16 LSB
    *x = (float)raw_x / 16.0f;
    *y = (float)raw_y / 16.0f;
    *z = (float)raw_z / 16.0f;
}

// =============================================================================
// LOW-LEVEL I2C COMMUNICATION
// =============================================================================

int8_t bno055_write(bno055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        ESP_LOGW(TAG, "bno055_write: NULL pointer provided");
        return BNO055_ERROR;
    }

    if (!i2c_write_reg(&bno055->i2c_handle, reg, data, len)) {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

int8_t bno055_read(bno055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        ESP_LOGW(TAG, "bno055_read: NULL pointer provided");
        return BNO055_ERROR;
    }

    if (!i2c_read_reg(&bno055->i2c_handle, reg, data, len)) {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

int8_t bno055_init(bno055_t *bno055, uint8_t sda, uint8_t scl, uint8_t i2c_num)
{
    if (bno055 == NULL) {
        ESP_LOGE(TAG, "bno055_init: NULL pointer provided");
        return BNO055_ERROR;
    }

    ESP_LOGI(TAG, "Initializing BNO055 sensor...");

    // Initialize I2C communication
    if (!i2c_init(&bno055->i2c_handle, i2c_num, scl, sda, BNO055_I2C_FREQ_HZ, BNO055_SENSOR_ADDR)) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return BNO055_ERROR;
    }

    // Set to configuration mode first
    bno055->operation_mode = BNO055_MODE_INIT;
    if (bno055_set_operation_mode(bno055, BNO055_MODE_CONFIG) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enter CONFIG mode");
        return BNO055_ERROR;
    }

    // Set page to 0
    uint8_t page = BNO055_PAGE_ZERO;
    if (bno055_write(bno055, BNO055_PAGE_ID_ADDR, &page, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set page 0");
        return BNO055_ERROR;
    }

    // Configure units: m/s², rad/s, radians, Celsius, Android orientation
    if (bno055_set_unit(bno055, BNO055_ACCEL_UNIT_MSQ, BNO055_GYRO_UNIT_DPS, 
                        BNO055_EULER_UNIT_RAD, BNO055_TEMP_UNIT_CELSIUS, 
                        BNO055_ANDROID_ORIENTATION) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set units");
        return BNO055_ERROR;
    }

    // Set power mode to normal
    bno055->power_mode = BNO055_POWER_LOW;
    if (bno055_set_power_mode(bno055, BNO055_POWER_NORMAL) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set power mode");
        return BNO055_ERROR;
    }

    // Set operation mode to NDOF (9-DOF fusion)
    if (bno055_set_operation_mode(bno055, BNO055_MODE_NDOF) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set NDOF mode");
        return BNO055_ERROR;
    }

    // Read sensor information
    if (bno055_get_info(bno055) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read sensor info");
    }

    // Print initialization summary
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "BNO055 Sensor Initialized");
    ESP_LOGI(TAG, "  Chip ID: 0x%02X", bno055->chip_id);
    ESP_LOGI(TAG, "  SW Revision: %02X.%02X", bno055->sw_rev_id[1], bno055->sw_rev_id[0]);
    ESP_LOGI(TAG, "  Accel Rev: 0x%02X", bno055->accel_rev_id);
    ESP_LOGI(TAG, "  Mag Rev: 0x%02X", bno055->mag_rev_id);
    ESP_LOGI(TAG, "  Gyro Rev: 0x%02X", bno055->gyro_rev_id);
    ESP_LOGI(TAG, "  BL Rev: 0x%02X", bno055->bl_rev_id);
    ESP_LOGI(TAG, "  Operation Mode: NDOF (0x%02X)", bno055->operation_mode);
    ESP_LOGI(TAG, "  Power Mode: NORMAL (0x%02X)", bno055->power_mode);
    ESP_LOGI(TAG, "=================================================");

    return BNO055_SUCCESS;
}

void bno055_reset(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_reset: NULL pointer provided");
        return;
    }

    ESP_LOGI(TAG, "Resetting BNO055 sensor");
    i2c_deinit(&bno055->i2c_handle);
    
    // Hardware reset via RST pin could be implemented here if needed
    // gpio_set_low(&bno055->rst_pin);
    // vTaskDelay(pdMS_TO_TICKS(10));
    // gpio_set_high(&bno055->rst_pin);
}

int8_t bno055_get_calibration_status(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_get_calibration_status: NULL pointer provided");
        return BNO055_ERROR;
    }

    uint8_t calib_status = 0;
    if (bno055_read(bno055, BNO055_CALIB_STAT_ADDR, &calib_status, 1) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read calibration status");
        return BNO055_ERROR;
    }

    bno055->calib_stat = calib_status;

    // Extract individual calibration statuses (2 bits each)
    uint8_t sys = (calib_status >> 6) & 0x03;
    uint8_t gyro = (calib_status >> 4) & 0x03;
    uint8_t accel = (calib_status >> 2) & 0x03;
    uint8_t mag = calib_status & 0x03;

    ESP_LOGI(TAG, "Calibration Status - Sys:%d Gyro:%d Accel:%d Mag:%d", sys, gyro, accel, mag);

    // Return success only if system is fully calibrated
    return (calib_status == BNO055_CALIB_STAT_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}

int8_t bno055_get_info(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_get_info: NULL pointer provided");
        return BNO055_ERROR;
    }

    uint8_t data[8] = {0};
    if (bno055_read(bno055, BNO055_CHIP_ID_ADDR, data, 8) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read sensor info");
        return BNO055_ERROR;
    }

    bno055->chip_id = data[0];
    bno055->accel_rev_id = data[1];
    bno055->mag_rev_id = data[2];
    bno055->gyro_rev_id = data[3];
    bno055->sw_rev_id[0] = data[4];  // LSB
    bno055->sw_rev_id[1] = data[5];  // MSB
    bno055->bl_rev_id = data[6];
    bno055->page_id = data[7];

    return BNO055_SUCCESS;
}

int8_t bno055_set_operation_mode(bno055_t *bno055, bno055_operation_mode_t mode)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_set_operation_mode: NULL pointer provided");
        return BNO055_ERROR;
    }

    // Only write if mode changed
    if (mode != bno055->operation_mode) {
        uint8_t mode_byte = (uint8_t)mode;
        if (bno055_write(bno055, BNO055_OPR_MODE_ADDR, &mode_byte, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
            ESP_LOGE(TAG, "Failed to set operation mode");
            return BNO055_ERROR;
        }
        bno055->operation_mode = mode;
        ESP_LOGI(TAG, "Operation mode set to 0x%02X", mode);
    }

    return BNO055_SUCCESS;
}

void bno055_get_euler_angles(bno055_t *bno055, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        ESP_LOGW(TAG, "bno055_get_euler_angles: NULL pointer provided");
        return;
    }

    uint8_t data[6] = {0};
    if (bno055_read(bno055, BNO055_EULER_H_LSB_ADDR, data, 6) != BNO055_SUCCESS) {
        // On read failure, return last known values
        *yaw = bno055->yaw;
        *pitch = bno055->pitch;
        *roll = bno055->roll;
        return;
    }

    convert_euler_data(bno055, data, yaw, pitch, roll);

    // Save values for future fallback
    bno055->yaw = *yaw;
    bno055->pitch = *pitch;
    bno055->roll = *roll;
}

void bno055_get_acceleration(bno055_t *bno055, float *x, float *y, float *z)
{
    if (bno055 == NULL || x == NULL || y == NULL || z == NULL) {
        ESP_LOGW(TAG, "bno055_get_acceleration: NULL pointer provided");
        return;
    }

    uint8_t data[6] = {0};
    if (bno055_read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 6) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read acceleration data");
        return;
    }

    convert_accel_data(bno055, data, x, y, z);
}

void bno055_get_gyro(bno055_t *bno055, float *gx, float *gy, float *gz)
{
    if (bno055 == NULL || gx == NULL || gy == NULL || gz == NULL) {
        ESP_LOGW(TAG, "bno055_get_gyro: NULL pointer provided");
        return;
    }

    uint8_t data[6] = {0};
    if (bno055_read(bno055, BNO055_GYRO_DATA_X_LSB_ADDR, data, 6) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read gyro data");
        return;
    }

    convert_gyro_data(bno055, data, gx, gy, gz);
}

void bno055_get_magnetometer(bno055_t *bno055, float *mx, float *my, float *mz)
{
    if (bno055 == NULL || mx == NULL || my == NULL || mz == NULL) {
        ESP_LOGW(TAG, "bno055_get_magnetometer: NULL pointer provided");
        return;
    }

    uint8_t data[6] = {0};
    if (bno055_read(bno055, BNO055_MAG_DATA_X_LSB_ADDR, data, 6) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read magnetometer data");
        return;
    }

    convert_mag_data(bno055, data, mx, my, mz);
}

int8_t bno055_read_all(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_read_all: NULL pointer provided");
        return BNO055_ERROR;
    }

    // Read 24 bytes: accel(6) + mag(6) + gyro(6) + euler(6)
    uint8_t data[24] = {0};
    if (bno055_read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 24) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read all sensor data");
        return BNO055_ERROR;
    }

    // Parse data into separate arrays
    convert_accel_data(bno055, &data[0], &bno055->ax, &bno055->ay, &bno055->az);
    convert_mag_data(bno055, &data[6], &bno055->mx, &bno055->my, &bno055->mz);
    convert_gyro_data(bno055, &data[12], &bno055->gx, &bno055->gy, &bno055->gz);
    convert_euler_data(bno055, &data[18], &bno055->yaw, &bno055->pitch, &bno055->roll);

    return BNO055_SUCCESS;
}

int8_t bno055_read_all_linear(bno055_t *bno055)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_read_all_linear: NULL pointer provided");
        return BNO055_ERROR;
    }

    // Read linear acceleration (6 bytes)
    uint8_t accel_data[6] = {0};
    if (bno055_read(bno055, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, accel_data, 6) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read linear acceleration data");
        return BNO055_ERROR;
    }

    // Read mag + gyro + euler (18 bytes)
    uint8_t other_data[18] = {0};
    if (bno055_read(bno055, BNO055_MAG_DATA_X_LSB_ADDR, other_data, 18) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read sensor data");
        return BNO055_ERROR;
    }

    convert_accel_data(bno055, accel_data, &bno055->ax, &bno055->ay, &bno055->az);
    convert_mag_data(bno055, &other_data[0], &bno055->mx, &bno055->my, &bno055->mz);
    convert_gyro_data(bno055, &other_data[6], &bno055->gx, &bno055->gy, &bno055->gz);
    convert_euler_data(bno055, &other_data[12], &bno055->yaw, &bno055->pitch, &bno055->roll);

    return BNO055_SUCCESS;
}

int8_t bno055_set_unit(bno055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit,
                       uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_set_unit: NULL pointer provided");
        return BNO055_ERROR;
    }

    // Construct unit selection byte
    uint8_t unit_byte = 0x00;
    unit_byte |= (accel_unit & 0x01) << 0;
    unit_byte |= (gyro_unit & 0x01) << 1;
    unit_byte |= (euler_unit & 0x01) << 2;
    unit_byte |= (temp_unit & 0x01) << 4;
    unit_byte |= (ori_unit & 0x01) << 7;

    if (bno055_write(bno055, BNO055_UNIT_SEL_ADDR, &unit_byte, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set units");
        return BNO055_ERROR;
    }

    // Update internal unit settings
    bno055->unit_settings.accel_unit = accel_unit;
    bno055->unit_settings.gyro_unit = gyro_unit;
    bno055->unit_settings.euler_unit = euler_unit;
    bno055->unit_settings.temp_unit = temp_unit;
    bno055->unit_settings.ori_unit = ori_unit;

    ESP_LOGI(TAG, "Units configured: accel=%d gyro=%d euler=%d temp=%d ori=%d",
             accel_unit, gyro_unit, euler_unit, temp_unit, ori_unit);

    return BNO055_SUCCESS;
}

int8_t bno055_set_power_mode(bno055_t *bno055, bno055_power_mode_t mode)
{
    if (bno055 == NULL) {
        ESP_LOGW(TAG, "bno055_set_power_mode: NULL pointer provided");
        return BNO055_ERROR;
    }

    // Only write if mode changed
    if (mode != bno055->power_mode) {
        uint8_t mode_byte = (uint8_t)mode;
        if (bno055_write(bno055, BNO055_PWR_MODE_ADDR, &mode_byte, BNO055_GEN_READ_WRITE_LENGTH) != BNO055_SUCCESS) {
            ESP_LOGE(TAG, "Failed to set power mode");
            return BNO055_ERROR;
        }
        bno055->power_mode = mode;
        ESP_LOGI(TAG, "Power mode set to 0x%02X", mode);
    }

    return BNO055_SUCCESS;
}

int8_t bno055_get_calibration_profile(bno055_t *bno055, bno055_calib_profile_t *calib_data)
{
    if (bno055 == NULL || calib_data == NULL) {
        ESP_LOGW(TAG, "bno055_get_calibration_profile: NULL pointer provided");
        return BNO055_ERROR;
    }

    // Check calibration status first
    if (bno055_get_calibration_status(bno055) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Sensor not fully calibrated, cannot read calibration profile");
        return BNO055_ERROR;
    }

    // Save current mode
    bno055_operation_mode_t current_mode = bno055->operation_mode;

    // Switch to CONFIG mode to read calibration data
    if (bno055_set_operation_mode(bno055, BNO055_MODE_CONFIG) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enter CONFIG mode");
        return BNO055_ERROR;
    }

    // Read all 22 bytes of calibration data
    uint8_t calib_offsets[22] = {0};
    if (bno055_read(bno055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, calib_offsets, 22) != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read calibration offsets");
        bno055_set_operation_mode(bno055, current_mode);  // Restore mode
        return BNO055_ERROR;
    }

    // Parse calibration offsets (all values are 16-bit little-endian)
    calib_data->accel_offset_x = (uint16_t)(calib_offsets[1] << 8 | calib_offsets[0]);
    calib_data->accel_offset_y = (uint16_t)(calib_offsets[3] << 8 | calib_offsets[2]);
    calib_data->accel_offset_z = (uint16_t)(calib_offsets[5] << 8 | calib_offsets[4]);
    
    calib_data->mag_offset_x = (uint16_t)(calib_offsets[7] << 8 | calib_offsets[6]);
    calib_data->mag_offset_y = (uint16_t)(calib_offsets[9] << 8 | calib_offsets[8]);
    calib_data->mag_offset_z = (uint16_t)(calib_offsets[11] << 8 | calib_offsets[10]);
    
    calib_data->gyro_offset_x = (uint16_t)(calib_offsets[13] << 8 | calib_offsets[12]);
    calib_data->gyro_offset_y = (uint16_t)(calib_offsets[15] << 8 | calib_offsets[14]);
    calib_data->gyro_offset_z = (uint16_t)(calib_offsets[17] << 8 | calib_offsets[16]);
    
    calib_data->accel_radius = (uint16_t)(calib_offsets[19] << 8 | calib_offsets[18]);
    calib_data->mag_radius = (uint16_t)(calib_offsets[21] << 8 | calib_offsets[20]);

    // Restore previous operation mode
    if (bno055_set_operation_mode(bno055, current_mode) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to restore operation mode");
    }

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "Calibration Profile Retrieved");
    ESP_LOGI(TAG, "  Accel Offsets: X=0x%04X Y=0x%04X Z=0x%04X",
             calib_data->accel_offset_x, calib_data->accel_offset_y, calib_data->accel_offset_z);
    ESP_LOGI(TAG, "  Mag Offsets: X=0x%04X Y=0x%04X Z=0x%04X",
             calib_data->mag_offset_x, calib_data->mag_offset_y, calib_data->mag_offset_z);
    ESP_LOGI(TAG, "  Gyro Offsets: X=0x%04X Y=0x%04X Z=0x%04X",
             calib_data->gyro_offset_x, calib_data->gyro_offset_y, calib_data->gyro_offset_z);
    ESP_LOGI(TAG, "  Accel Radius: 0x%04X  Mag Radius: 0x%04X",
             calib_data->accel_radius, calib_data->mag_radius);
    ESP_LOGI(TAG, "=================================================");

    return BNO055_SUCCESS;
}
