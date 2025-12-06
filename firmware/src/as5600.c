/**
 * @file as5600.c
 * @brief AS5600 magnetic position sensor driver implementation
 * 
 * Implements I2C communication, ADC reading, and register management for
 * the AMS AS5600 12-bit magnetic rotary position sensor.
 */

#include "as5600.h"
#include "esp_log.h"
#include <string.h>

// =============================================================================
// CONSTANTS
// =============================================================================

static const char *TAG = "as5600";

// Helper macros for linear mapping and clamping
#define MAP(val, in_min, in_max, out_min, out_max) \
    (((val) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#define CLAMP(val, min_val, max_val) \
    ((val) < (min_val) ? (min_val) : ((val) > (max_val) ? (max_val) : (val)))

// =============================================================================
// PRIVATE HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Check if register is valid for read operations
 */
static bool is_valid_read_register(as5600_reg_t reg)
{
    return (reg == AS5600_REG_ZMCO || 
            reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L ||
            reg == AS5600_REG_MPOS_H || reg == AS5600_REG_MPOS_L ||
            reg == AS5600_REG_MANG_H || reg == AS5600_REG_MANG_L ||
            reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L ||
            reg == AS5600_REG_STATUS || 
            reg == AS5600_REG_RAW_ANGLE_H || reg == AS5600_REG_RAW_ANGLE_L ||
            reg == AS5600_REG_ANGLE_H || reg == AS5600_REG_ANGLE_L ||
            reg == AS5600_REG_AGC ||
            reg == AS5600_REG_MAGNITUDE_H || reg == AS5600_REG_MAGNITUDE_L);
}

/**
 * @brief Check if register is valid for write operations
 */
static bool is_valid_write_register(as5600_reg_t reg)
{
    return (reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L ||
            reg == AS5600_REG_MPOS_H || reg == AS5600_REG_MPOS_L ||
            reg == AS5600_REG_MANG_H || reg == AS5600_REG_MANG_L ||
            reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L ||
            reg == AS5600_REG_BURN);
}

// =============================================================================
// INITIALIZATION AND CONFIGURATION
// =============================================================================

bool as5600_init(as5600_t *sensor, i2c_port_t i2c_port,
                 uint8_t scl_pin, uint8_t sda_pin, uint8_t out_pin)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    sensor->out_pin = out_pin;

    // Initialize I2C communication
    if (!i2c_init(&sensor->i2c, i2c_port, scl_pin, sda_pin, 
                  AS5600_I2C_FREQ_HZ, AS5600_I2C_ADDR)) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return false;
    }

    ESP_LOGI(TAG, "AS5600 initialized on I2C port %d (SCL:%d, SDA:%d, OUT:%d)", 
             i2c_port, scl_pin, sda_pin, out_pin);
    return true;
}

void as5600_deinit(as5600_t *sensor)
{
    if (!sensor) return;
    
    i2c_deinit(&sensor->i2c);
    adc_deinit(&sensor->adc);
    gpio_deinit(&sensor->gpio);
    
    ESP_LOGI(TAG, "AS5600 deinitialized");
}

bool as5600_init_adc(as5600_t *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    if (!adc_init(&sensor->adc, sensor->out_pin)) {
        ESP_LOGE(TAG, "ADC initialization failed for GPIO %d", sensor->out_pin);
        return false;
    }

    ESP_LOGI(TAG, "ADC initialized for AS5600 OUT pin (GPIO %d)", sensor->out_pin);
    return true;
}

bool as5600_init_adc_shared(as5600_t *sensor, adc_oneshot_unit_handle_t shared_handle)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    if (!adc_config_channel(&sensor->adc, sensor->out_pin, shared_handle)) {
        ESP_LOGE(TAG, "ADC channel configuration failed for GPIO %d", sensor->out_pin);
        return false;
    }

    ESP_LOGI(TAG, "ADC channel configured for AS5600 on GPIO %d", sensor->out_pin);
    return true;
}

void as5600_deinit_adc(as5600_t *sensor)
{
    if (!sensor) return;
    adc_deinit(&sensor->adc);
}

bool as5600_init_gpio(as5600_t *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    // Initialize as output, drive strength 2, no pull-up/down
    if (!gpio_init_basic(&sensor->gpio, sensor->out_pin, 2, false, false)) {
        ESP_LOGE(TAG, "GPIO initialization failed for pin %d", sensor->out_pin);
        return false;
    }

    // Set low initially (programming/calibration mode)
    gpio_set_low(&sensor->gpio);
    
    ESP_LOGI(TAG, "GPIO initialized for AS5600 OUT pin (GPIO %d)", sensor->out_pin);
    return true;
}

void as5600_deinit_gpio(as5600_t *sensor)
{
    if (!sensor) return;
    gpio_deinit(&sensor->gpio);
}

void as5600_set_gpio(as5600_t *sensor, uint8_t level)
{
    if (!sensor) return;
    
    if (level) {
        gpio_set_high(&sensor->gpio);
    } else {
        gpio_set_low(&sensor->gpio);
    }
}

// =============================================================================
// ANGLE MEASUREMENT
// =============================================================================

float as5600_read_angle_adc(as5600_t *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return -1.0f;
    }

    // Check if ADC is calibrated and sensor configured for analog output
    if (!sensor->adc.is_calibrated || sensor->config.OUTS != AS5600_OUTPUT_STAGE_ANALOG_RR) {
        ESP_LOGW(TAG, "ADC not calibrated or sensor not in analog mode");
        return -1.0f;
    }

    // Read voltage from ADC
    uint16_t voltage_mv;
    adc_read_mvolt(&sensor->adc, &voltage_mv);

    // Clamp to 10%-90% range
    voltage_mv = CLAMP(voltage_mv, AS5600_VCC_MIN_MV, AS5600_VCC_MAX_MV);

    // Convert voltage to angle (0-360 degrees)
    float angle = MAP((float)voltage_mv, AS5600_VCC_MIN_MV, AS5600_VCC_MAX_MV, 
                     0.0f, AS5600_DEGREES_MAX);

    return angle;
}

bool as5600_get_raw_angle(as5600_t *sensor, uint16_t *raw_angle)
{
    if (!sensor || !raw_angle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    return as5600_read_register(sensor, AS5600_REG_RAW_ANGLE_H, raw_angle);
}

bool as5600_get_angle(as5600_t *sensor, uint16_t *angle)
{
    if (!sensor || !angle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    return as5600_read_register(sensor, AS5600_REG_ANGLE_H, angle);
}

// =============================================================================
// CONFIGURATION REGISTERS
// =============================================================================

bool as5600_set_start_position(as5600_t *sensor, uint16_t start_position)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    // Write both high and low bytes
    uint8_t write_buffer[] = {AS5600_REG_ZPOS_H, start_position >> 8, start_position & 0xFF};
    i2c_write(&sensor->i2c, write_buffer, 3);

    ESP_LOGD(TAG, "Start position set to %u", start_position);
    return true;
}

bool as5600_get_start_position(as5600_t *sensor, uint16_t *start_position)
{
    if (!sensor || !start_position) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    return as5600_read_register(sensor, AS5600_REG_ZPOS_H, start_position);
}

bool as5600_set_stop_position(as5600_t *sensor, uint16_t stop_position)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    uint8_t write_buffer[] = {AS5600_REG_MPOS_H, stop_position >> 8, stop_position & 0xFF};
    i2c_write(&sensor->i2c, write_buffer, 3);

    ESP_LOGD(TAG, "Stop position set to %u", stop_position);
    return true;
}

bool as5600_get_stop_position(as5600_t *sensor, uint16_t *stop_position)
{
    if (!sensor || !stop_position) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    return as5600_read_register(sensor, AS5600_REG_MPOS_H, stop_position);
}

bool as5600_set_max_angle(as5600_t *sensor, uint16_t max_angle)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    uint8_t write_buffer[] = {AS5600_REG_MANG_H, max_angle >> 8, max_angle & 0xFF};
    i2c_write(&sensor->i2c, write_buffer, 3);

    ESP_LOGD(TAG, "Max angle set to %u", max_angle);
    return true;
}

bool as5600_get_max_angle(as5600_t *sensor, uint16_t *max_angle)
{
    if (!sensor || !max_angle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    return as5600_read_register(sensor, AS5600_REG_MANG_H, max_angle);
}

bool as5600_set_config(as5600_t *sensor, as5600_config_t config)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    sensor->config = config;
    
    uint8_t write_buffer[] = {AS5600_REG_CONF_H, config.WORD >> 8, config.WORD & 0xFF};
    i2c_write(&sensor->i2c, write_buffer, 3);

    ESP_LOGD(TAG, "Configuration set to 0x%04X", config.WORD);
    return true;
}

bool as5600_get_config(as5600_t *sensor, as5600_config_t *config)
{
    if (!sensor || !config) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    uint16_t conf_word;
    if (!as5600_read_register(sensor, AS5600_REG_CONF_H, &conf_word)) {
        return false;
    }

    config->WORD = conf_word;
    sensor->config = *config;
    return true;
}

// =============================================================================
// STATUS AND DIAGNOSTICS
// =============================================================================

bool as5600_get_status(as5600_t *sensor, uint8_t *status)
{
    if (!sensor || !status) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    if (!i2c_read_reg(&sensor->i2c, AS5600_REG_STATUS, status, 1)) {
        ESP_LOGE(TAG, "Failed to read status register");
        return false;
    }

    return true;
}

bool as5600_get_agc(as5600_t *sensor, uint8_t *agc)
{
    if (!sensor || !agc) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    if (!i2c_read_reg(&sensor->i2c, AS5600_REG_AGC, agc, 1)) {
        ESP_LOGE(TAG, "Failed to read AGC register");
        return false;
    }

    return true;
}

bool as5600_get_magnitude(as5600_t *sensor, uint16_t *magnitude)
{
    if (!sensor || !magnitude) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    return as5600_read_register(sensor, AS5600_REG_MAGNITUDE_H, magnitude);
}

// =============================================================================
// PERMANENT PROGRAMMING (BURN COMMANDS)
// =============================================================================

bool as5600_burn_angle(as5600_t *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    uint8_t data = AS5600_BURN_MODE_BURN_ANGLE;
    if (!i2c_write_reg(&sensor->i2c, AS5600_REG_BURN, &data, 1)) {
        ESP_LOGE(TAG, "BURN_ANGLE command failed");
        return false;
    }

    ESP_LOGW(TAG, "BURN_ANGLE executed - ZPOS/MPOS permanently programmed");
    return true;
}

bool as5600_burn_setting(as5600_t *sensor)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    uint8_t data = AS5600_BURN_MODE_BURN_SETTING;
    if (!i2c_write_reg(&sensor->i2c, AS5600_REG_BURN, &data, 1)) {
        ESP_LOGE(TAG, "BURN_SETTING command failed");
        return false;
    }

    ESP_LOGW(TAG, "BURN_SETTING executed - MANG/CONF permanently programmed");
    return true;
}

// =============================================================================
// LOW-LEVEL REGISTER ACCESS
// =============================================================================

bool as5600_read_register(as5600_t *sensor, as5600_reg_t reg, uint16_t *data)
{
    if (!sensor || !data) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    if (!is_valid_read_register(reg)) {
        ESP_LOGE(TAG, "Invalid read register: 0x%02X", reg);
        return false;
    }

    // Single-byte registers
    if (reg == AS5600_REG_ZMCO || reg == AS5600_REG_STATUS || reg == AS5600_REG_AGC) {
        uint8_t byte_data;
        if (!i2c_read_reg(&sensor->i2c, reg, &byte_data, 1)) {
            ESP_LOGE(TAG, "I2C read failed for register 0x%02X", reg);
            return false;
        }
        *data = byte_data;
    }
    // Two-byte registers (need byte swap for big-endian)
    else {
        uint8_t buffer[2];
        if (!i2c_read_reg(&sensor->i2c, reg, buffer, 2)) {
            ESP_LOGE(TAG, "I2C read failed for register 0x%02X", reg);
            return false;
        }
        // AS5600 sends high byte first, combine properly
        *data = (buffer[0] << 8) | buffer[1];
    }

    return true;
}

bool as5600_write_register(as5600_t *sensor, as5600_reg_t reg, uint16_t data)
{
    if (!sensor) {
        ESP_LOGE(TAG, "Sensor pointer is NULL");
        return false;
    }

    if (!is_valid_write_register(reg)) {
        ESP_LOGE(TAG, "Invalid write register: 0x%02X", reg);
        return false;
    }

    // Single-byte register (BURN command)
    if (reg == AS5600_REG_BURN) {
        uint8_t byte_data = data & 0xFF;
        if (!i2c_write_reg(&sensor->i2c, reg, &byte_data, 1)) {
            ESP_LOGE(TAG, "I2C write failed for register 0x%02X", reg);
            return false;
        }
    }
    // Two-byte registers
    else {
        uint8_t write_buffer[] = {data >> 8, data & 0xFF};
        if (!i2c_write_reg(&sensor->i2c, reg, write_buffer, 2)) {
            ESP_LOGE(TAG, "I2C write failed for register 0x%02X", reg);
            return false;
        }
    }

    return true;
}

as5600_reg_t as5600_reg_name_to_addr(as5600_t *sensor, const char *reg_str)
{
    if (!sensor || !reg_str) {
        ESP_LOGE(TAG, "Invalid parameters");
        return -1;
    }

    as5600_reg_t reg = -1;

    if (strcmp(reg_str, "zmco") == 0) {
        reg = AS5600_REG_ZMCO;
    } else if (strcmp(reg_str, "zpos") == 0) {
        reg = AS5600_REG_ZPOS_H;
    } else if (strcmp(reg_str, "mpos") == 0) {
        reg = AS5600_REG_MPOS_H;
    } else if (strcmp(reg_str, "mang") == 0) {
        reg = AS5600_REG_MANG_H;
    } else if (strcmp(reg_str, "conf") == 0) {
        reg = AS5600_REG_CONF_H;
    } else if (strcmp(reg_str, "stat") == 0) {
        reg = AS5600_REG_STATUS;
    } else if (strcmp(reg_str, "rang") == 0) {
        reg = AS5600_REG_RAW_ANGLE_H;
    } else if (strcmp(reg_str, "angl") == 0) {
        reg = AS5600_REG_ANGLE_H;
    } else if (strcmp(reg_str, "agco") == 0) {
        reg = AS5600_REG_AGC;
    } else if (strcmp(reg_str, "magn") == 0) {
        reg = AS5600_REG_MAGNITUDE_H;
    } else if (strcmp(reg_str, "burn") == 0) {
        reg = AS5600_REG_BURN;
    } else {
        ESP_LOGW(TAG, "Unknown register name: %s", reg_str);
        return -1;
    }

    sensor->last_reg = reg;
    return reg;
}
