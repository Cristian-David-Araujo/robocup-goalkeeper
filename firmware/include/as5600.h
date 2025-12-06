/**
 * @file as5600.h
 * @brief AS5600 magnetic position sensor driver for ESP32-S3
 * 
 * Driver for the AMS AS5600 12-bit magnetic rotary position sensor with I2C interface.
 * Supports angle measurement via I2C digital interface and analog OUT pin via ADC.
 * 
 * Key Features:
 * - 12-bit angular position measurement (0.088° resolution)
 * - Contactless 360° absolute angle measurement
 * - Programmable zero position and maximum angle
 * - Analog output (10%-90% VCC) and digital output via I2C
 * - Automatic Gain Control (AGC) for optimal magnet distance
 * - Permanent angle programming capability (BURN commands)
 * 
 * Hardware Configuration:
 * - I2C Address: 0x36 (fixed)
 * - I2C Clock: 400 kHz (Fast mode)
 * - VCC: 3.3V (required for ESP32 ADC compatibility)
 * - OUT Pin: Analog 10%-90% range (matches ESP32 ADC linear range)
 * 
 * Thread-safety: Not thread-safe. External synchronization required if
 * sensor instances are accessed from multiple tasks.
 * 
 * @note All identifiers follow snake_case naming convention
 * @note ADC reading requires OUT pin configured for analog output (10%-90%)
 * 
 * @author  MaverickST (original), Refactored for consistency
 * @version 1.0.0
 * @date    December 2024
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>

#include "as5600_defs.h"
#include "platform_esp32s3.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CONSTANTS AND MACROS
// =============================================================================

#define AS5600_I2C_ADDR         0x36        ///< AS5600 I2C slave address (fixed)
#define AS5600_I2C_FREQ_HZ      400000      ///< I2C clock frequency (400 kHz Fast mode)

#define AS5600_VCC_MV           3300        ///< VCC voltage in millivolts
#define AS5600_VCC_MIN_MV       330         ///< OUT pin minimum voltage (10% of VCC)
#define AS5600_VCC_MAX_MV       2970        ///< OUT pin maximum voltage (90% of VCC)

#define AS5600_ANGLE_MAX        4095        ///< Maximum 12-bit angle value
#define AS5600_DEGREES_MAX      360.0f      ///< Maximum angle in degrees

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

/**
 * @brief AS5600 sensor instance structure
 * 
 * Contains all configuration and peripheral handles for an AS5600 sensor.
 * Opaque to users - access only through API functions.
 */
typedef struct {
    as5600_config_t config;     ///< Current sensor configuration
    as5600_reg_t last_reg;      ///< Last accessed register (for string conversion)
    uint8_t out_pin;            ///< GPIO pin number for OUT (analog) signal
    
    // Platform peripheral handles
    i2c_t i2c;                  ///< I2C bus handle
    adc_t adc;                  ///< ADC handle for OUT pin
    gpio_t gpio;                ///< GPIO handle for OUT pin control
} as5600_t;

// =============================================================================
// INITIALIZATION AND CONFIGURATION
// =============================================================================

/**
 * @brief Initialize AS5600 sensor with I2C communication
 *
 * Configures I2C bus for communication with AS5600 sensor. The OUT pin is
 * stored but not configured - call as5600_init_adc() or as5600_init_gpio()
 * separately based on desired OUT pin function.
 *
 * @param[in,out] sensor    Pointer to AS5600 instance structure
 * @param[in]     i2c_port  I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param[in]     scl_pin   GPIO pin for I2C SCL
 * @param[in]     sda_pin   GPIO pin for I2C SDA
 * @param[in]     out_pin   GPIO pin connected to AS5600 OUT signal
 * 
 * @return true on success, false on failure
 * 
 * @note After initialization, configure OUT pin function:
 *       - as5600_init_adc() for analog angle reading
 *       - as5600_init_gpio() for calibration/programming mode
 */
bool as5600_init(as5600_t *sensor, i2c_port_t i2c_port, 
                 uint8_t scl_pin, uint8_t sda_pin, uint8_t out_pin);

/**
 * @brief Deinitialize AS5600 sensor and free resources
 *
 * Releases I2C, ADC, and GPIO resources associated with the sensor.
 *
 * @param[in] sensor Pointer to AS5600 instance
 */
void as5600_deinit(as5600_t *sensor);

/**
 * @brief Initialize ADC for analog OUT pin reading
 *
 * Configures ADC channel for reading analog angle output from OUT pin.
 * Enables as5600_read_angle_adc() function.
 *
 * @param[in,out] sensor Pointer to AS5600 instance
 * @return true on success, false on failure
 * 
 * @note Requires AS5600 OUTS configuration set to analog mode (10%-90%)
 */
bool as5600_init_adc(as5600_t *sensor);

/**
 * @brief Initialize ADC using shared ADC unit handle
 *
 * Alternative ADC initialization when multiple channels share same ADC unit.
 * Use when ADC unit is already initialized elsewhere.
 *
 * @param[in,out] sensor        Pointer to AS5600 instance
 * @param[in]     shared_handle Existing ADC unit handle
 * @return true on success, false on failure
 */
bool as5600_init_adc_shared(as5600_t *sensor, adc_oneshot_unit_handle_t shared_handle);

/**
 * @brief Deinitialize ADC channel
 *
 * @param[in] sensor Pointer to AS5600 instance
 */
void as5600_deinit_adc(as5600_t *sensor);

/**
 * @brief Initialize GPIO for OUT pin control
 *
 * Configures OUT pin as GPIO output for sensor calibration or programming.
 * Initially sets pin LOW (programming mode).
 *
 * @param[in,out] sensor Pointer to AS5600 instance
 * @return true on success, false on failure
 * 
 * @note Used during BURN operations or sensor programming
 */
bool as5600_init_gpio(as5600_t *sensor);

/**
 * @brief Deinitialize GPIO
 *
 * @param[in] sensor Pointer to AS5600 instance
 */
void as5600_deinit_gpio(as5600_t *sensor);

/**
 * @brief Set GPIO OUT pin state
 *
 * Controls OUT pin when configured as GPIO. Used for calibration or
 * entering programming mode.
 *
 * @param[in] sensor Pointer to AS5600 instance
 * @param[in] level  Desired pin level (0=LOW, 1=HIGH)
 */
void as5600_set_gpio(as5600_t *sensor, uint8_t level);

// =============================================================================
// ANGLE MEASUREMENT
// =============================================================================

/**
 * @brief Read angle from AS5600 via ADC (analog OUT pin)
 *
 * Reads analog voltage from OUT pin and converts to angle in degrees.
 * Requires:
 * - ADC initialized via as5600_init_adc()
 * - AS5600 configured for analog output (OUTS = 10%-90%)
 *
 * The OUT pin voltage range (10%-90% of VCC) matches ESP32 ADC linear
 * operating range for accurate measurements.
 *
 * @param[in] sensor Pointer to AS5600 instance
 * @return Angle in degrees (0.0-360.0), or -1.0 on error
 * 
 * @note Returns -1.0 if ADC not calibrated or sensor not in analog mode
 */
float as5600_read_angle_adc(as5600_t *sensor);

/**
 * @brief Read raw angle register via I2C
 *
 * Reads unscaled 12-bit angle from RAW ANGLE register (0x0C).
 * This is the measured angle before zero position and maximum angle scaling.
 *
 * @param[in]  sensor    Pointer to AS5600 instance
 * @param[out] raw_angle Pointer to receive 12-bit raw angle value
 * @return true on success, false on I2C error
 */
bool as5600_get_raw_angle(as5600_t *sensor, uint16_t *raw_angle);

/**
 * @brief Read scaled angle register via I2C
 *
 * Reads 12-bit angle from ANGLE register (0x0E), which is scaled based
 * on ZPOS and MPOS configuration. This is the application-ready angle.
 *
 * @param[in]  sensor Pointer to AS5600 instance
 * @param[out] angle  Pointer to receive 12-bit scaled angle value
 * @return true on success, false on I2C error
 */
bool as5600_get_angle(as5600_t *sensor, uint16_t *angle);

// =============================================================================
// CONFIGURATION REGISTERS
// =============================================================================

/**
 * @brief Set zero position (ZPOS register)
 *
 * Programs start position for angle measurement range. Combined with MPOS,
 * defines angular operating range. Can be permanently programmed via BURN.
 *
 * @param[in] sensor         Pointer to AS5600 instance
 * @param[in] start_position 12-bit zero position value (0-4095)
 * @return true on success, false on I2C error
 */
bool as5600_set_start_position(as5600_t *sensor, uint16_t start_position);

/**
 * @brief Get zero position (ZPOS register)
 *
 * @param[in]  sensor         Pointer to AS5600 instance
 * @param[out] start_position Pointer to receive 12-bit zero position
 * @return true on success, false on I2C error
 */
bool as5600_get_start_position(as5600_t *sensor, uint16_t *start_position);

/**
 * @brief Set stop position (MPOS register)
 *
 * Programs end position for angle measurement range. Combined with ZPOS,
 * defines angular operating range. Can be permanently programmed via BURN.
 *
 * @param[in] sensor        Pointer to AS5600 instance
 * @param[in] stop_position 12-bit stop position value (0-4095)
 * @return true on success, false on I2C error
 */
bool as5600_set_stop_position(as5600_t *sensor, uint16_t stop_position);

/**
 * @brief Get stop position (MPOS register)
 *
 * @param[in]  sensor        Pointer to AS5600 instance
 * @param[out] stop_position Pointer to receive 12-bit stop position
 * @return true on success, false on I2C error
 */
bool as5600_get_stop_position(as5600_t *sensor, uint16_t *stop_position);

/**
 * @brief Set maximum angle (MANG register)
 *
 * Programs maximum angle for proportional angle output. Used with analog
 * output configuration. Can be permanently programmed via BURN.
 *
 * @param[in] sensor    Pointer to AS5600 instance
 * @param[in] max_angle 12-bit maximum angle value (0-4095)
 * @return true on success, false on I2C error
 */
bool as5600_set_max_angle(as5600_t *sensor, uint16_t max_angle);

/**
 * @brief Get maximum angle (MANG register)
 *
 * @param[in]  sensor    Pointer to AS5600 instance
 * @param[out] max_angle Pointer to receive 12-bit maximum angle
 * @return true on success, false on I2C error
 */
bool as5600_get_max_angle(as5600_t *sensor, uint16_t *max_angle);

/**
 * @brief Set sensor configuration (CONF register)
 *
 * Programs AS5600 operating mode including:
 * - Power mode, Hysteresis, Output stage, PWM frequency
 * - Slow filter, Fast filter threshold, Watchdog
 *
 * @param[in] sensor Pointer to AS5600 instance
 * @param[in] config Configuration structure
 * @return true on success, false on I2C error
 * 
 * @note Configuration can be permanently programmed via BURN_SETTING
 */
bool as5600_set_config(as5600_t *sensor, as5600_config_t config);

/**
 * @brief Get sensor configuration (CONF register)
 *
 * @param[in]  sensor Pointer to AS5600 instance
 * @param[out] config Pointer to receive configuration
 * @return true on success, false on I2C error
 */
bool as5600_get_config(as5600_t *sensor, as5600_config_t *config);

// =============================================================================
// STATUS AND DIAGNOSTICS
// =============================================================================

/**
 * @brief Read status register
 *
 * Status bits indicate:
 * - MD: Magnet detected
 * - ML: Magnet too weak
 * - MH: Magnet too strong
 *
 * @param[in]  sensor Pointer to AS5600 instance
 * @param[out] status Pointer to receive 8-bit status value
 * @return true on success, false on I2C error
 */
bool as5600_get_status(as5600_t *sensor, uint8_t *status);

/**
 * @brief Read Automatic Gain Control value
 *
 * AGC value indicates optimal magnet positioning. Target value is
 * approximately midrange for best performance.
 *
 * @param[in]  sensor Pointer to AS5600 instance
 * @param[out] agc    Pointer to receive 8-bit AGC value
 * @return true on success, false on I2C error
 */
bool as5600_get_agc(as5600_t *sensor, uint8_t *agc);

/**
 * @brief Read magnitude of internal CORDIC
 *
 * Magnitude indicates magnetic field strength. Useful for magnet
 * positioning during installation.
 *
 * @param[in]  sensor    Pointer to AS5600 instance
 * @param[out] magnitude Pointer to receive 12-bit magnitude value
 * @return true on success, false on I2C error
 */
bool as5600_get_magnitude(as5600_t *sensor, uint16_t *magnitude);

// =============================================================================
// PERMANENT PROGRAMMING (BURN COMMANDS)
// =============================================================================

/**
 * @brief Execute BURN_ANGLE command
 *
 * Permanently programs ZPOS and MPOS values to OTP memory. Can be executed
 * up to 3 times (check ZMCO register). Requires magnet present (MD=1).
 *
 * @param[in] sensor Pointer to AS5600 instance
 * @return true on success, false on error
 * 
 * @warning Permanent operation! Can only be done 3 times per device.
 * @note Verify ZPOS/MPOS values before burning
 */
bool as5600_burn_angle(as5600_t *sensor);

/**
 * @brief Execute BURN_SETTING command
 *
 * Permanently programs MANG and CONF values to OTP memory. Can only be
 * executed once. Only possible if ZPOS/MPOS never burned (ZMCO=0).
 *
 * @param[in] sensor Pointer to AS5600 instance
 * @return true on success, false on error
 * 
 * @warning Permanent operation! Can only be done once per device.
 * @note Verify MANG/CONF values before burning
 */
bool as5600_burn_setting(as5600_t *sensor);

// =============================================================================
// LOW-LEVEL REGISTER ACCESS
// =============================================================================

/**
 * @brief Read AS5600 register
 *
 * Low-level register read operation. Automatically handles 1-byte and
 * 2-byte registers with proper endianness conversion.
 *
 * @param[in]  sensor Pointer to AS5600 instance
 * @param[in]  reg    Register address to read
 * @param[out] data   Pointer to receive register value (8 or 16-bit)
 * @return true on success, false on invalid register or I2C error
 */
bool as5600_read_register(as5600_t *sensor, as5600_reg_t reg, uint16_t *data);

/**
 * @brief Write AS5600 register
 *
 * Low-level register write operation. Automatically handles 1-byte and
 * 2-byte registers with proper endianness conversion.
 *
 * @param[in] sensor Pointer to AS5600 instance
 * @param[in] reg    Register address to write
 * @param[in] data   Data value to write (8 or 16-bit)
 * @return true on success, false on invalid register or I2C error
 */
bool as5600_write_register(as5600_t *sensor, as5600_reg_t reg, uint16_t data);

/**
 * @brief Convert register name string to address
 *
 * Helper function for debugging/CLI. Converts register names like "zmco",
 * "zpos", "angle" to corresponding register addresses.
 *
 * @param[in]  sensor  Pointer to AS5600 instance
 * @param[in]  reg_str Register name string
 * @return Register address, or -1 if invalid name
 * 
 * @note Result is also stored in sensor->last_reg
 */
as5600_reg_t as5600_reg_name_to_addr(as5600_t *sensor, const char *reg_str);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H
