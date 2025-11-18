/**
 * @file as5600.h
 * @brief AS5600 12-bit magnetic rotary position sensor driver interface
 * 
 * This module provides functions to initialize and control the AS5600 contactless
 * magnetic angle sensor using I2C communication and optional analog output via ADC.
 * 
 * Features:
 * - 12-bit angular resolution (0.088° per LSB)
 * - I2C digital interface
 * - Analog output with reduced range (10%-90% VCC)
 * - Programmable zero position and maximum angle
 * - Non-volatile memory for configuration
 * 
 * ADC Analog Output Notes:
 * The ESP32 ADC has best linearity in the 5%-90% range. The AS5600 OUT pin is
 * configured for 10%-90% range to stay within this linear region when using 3.3V VCC.
 * 
 * Thread-safety: Functions are NOT thread-safe. External synchronization required.
 * 
 * @author MaverickST
 * @version 0.2
 * @date 2024-11-25
 * @copyright Unlicensed
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "as5600_defs.h"
#include "platform_esp32s3.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CONFIGURATION CONSTANTS
// =============================================================================

#define AS5600_VCC_3V3_MV           (3300)      ///< VCC voltage in millivolts
#define AS5600_VCC_3V3_MIN_RR_MV    (330)       ///< Min analog output (10% VCC)
#define AS5600_VCC_3V3_MAX_RR_MV    (2970)      ///< Max analog output (90% VCC)
#define AS5600_I2C_FREQ_HZ          (400 * 1000)  ///< I2C clock frequency: 400 kHz
#define AS5600_SENSOR_ADDR          (0x36)      ///< Default I2C slave address

// =============================================================================
// UTILITY MACROS
// =============================================================================

/// @brief Map value from input range to output range
#define AS5600_MAP(val, in_min, in_max, out_min, out_max) \
    ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

/// @brief Clamp value between min and max
#define AS5600_LIMIT(a, min, max) ((a) < (min) ? (min) : ((a) > (max) ? (max) : (a)))

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

/**
 * @brief AS5600 sensor instance structure
 * 
 * Contains hardware configuration and peripheral handles for a single AS5600 sensor.
 * Must be initialized with as5600_init() before use.
 * 
 * Thread-safety: Not thread-safe. Do not access from multiple tasks concurrently.
 */
typedef struct {
    as5600_config_t conf;   ///< Sensor configuration register value
    as5600_reg_t reg;       ///< Last accessed register (internal use)
    uint8_t out;            ///< GPIO pin for analog OUT signal
    
    // Peripheral handles
    i2c_t i2c_handle;       ///< I2C communication handle
    adc_t adc_handle;       ///< ADC handle for analog reading
    gpio_t gpio_handle;     ///< GPIO handle (for OUT pin control if needed)
} as5600_t;

// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

/**
 * @brief Initialize AS5600 sensor
 * 
 * Configures I2C communication for the sensor. Stores GPIO pin for analog output.
 * Does NOT initialize ADC or GPIO - use as5600_init_adc() or as5600_init_adc_shared()
 * separately if analog reading is needed.
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance structure
 * @param[in] i2c_num I2C port number
 * @param[in] scl GPIO pin for I2C SCL
 * @param[in] sda GPIO pin for I2C SDA
 * @param[in] out GPIO pin for analog OUT signal (for ADC reading)
 * 
 * @note Must be called before any other AS5600 functions
 */
void as5600_init(as5600_t *as5600, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, uint8_t out);

/**
 * @brief Deinitialize AS5600 sensor
 * 
 * Releases I2C, ADC, and GPIO resources.
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 */
void as5600_deinit(as5600_t *as5600);

/**
 * @brief Get angle in degrees from analog output via ADC
 * 
 * Reads the analog OUT pin voltage via ADC and maps it to 0-360 degrees.
 * Requires ADC to be calibrated and output configured for analog reduced range.
 * 
 * @param[in] as5600 Pointer to AS5600 instance
 * @return Angle in degrees [0-360], or -1 on error
 * 
 * @note Sensor must be configured with OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR
 * @note ADC must be initialized and calibrated before calling
 */
float as5600_adc_get_angle(as5600_t *as5600);

/**
 * @brief Burn zero and max position to non-volatile memory
 * 
 * Permanently writes ZPOS and MPOS registers. Can be executed up to 3 times only.
 * Check ZMCO register to see remaining burn count.
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * 
 * @warning This operation is PERMANENT and has limited uses
 * @note Magnet must be detected (MD=1) for this operation to succeed
 */
void as5600_burn_angle_command(as5600_t *as5600);

/**
 * @brief Burn max angle and configuration to non-volatile memory
 * 
 * Permanently writes MANG and CONF registers. Can only be executed once,
 * and only if ZPOS/MPOS have never been burned (ZMCO=0).
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * 
 * @warning This operation is PERMANENT and can only be done once ever
 */
void as5600_burn_setting_command(as5600_t *as5600);

/**
 * @brief Convert register name string to register address
 * 
 * Utility function to map human-readable register names to addresses.
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance (stores result in reg field)
 * @param[in] reg_str Register name string (e.g., "zmco", "zpos", "angl")
 * @return Register address, or -1 if string not recognized
 */
as5600_reg_t as5600_reg_str_to_addr(as5600_t *as5600, const char *reg_str);

// =============================================================================
// GPIO AND ADC FUNCTIONS
// =============================================================================

/**
 * @brief Initialize ADC for analog angle reading
 * 
 * Configures ADC for the OUT pin. Use for independent ADC configuration.
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 */
void as5600_init_adc(as5600_t *as5600);

/**
 * @brief Initialize ADC channel with shared ADC unit
 * 
 * Configures ADC channel on an already-initialized ADC unit handle.
 * Use when multiple ADC channels share the same ADC unit.
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] shared_handle Shared ADC unit handle
 */
void as5600_init_adc_shared(as5600_t *as5600, adc_oneshot_unit_handle_t shared_handle);

/**
 * @brief Deinitialize ADC
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 */
void as5600_deinit_adc(as5600_t *as5600);

/**
 * @brief Initialize GPIO for OUT pin control
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 */
void as5600_init_gpio(as5600_t *as5600);

/**
 * @brief Deinitialize GPIO
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 */
void as5600_deinit_gpio(as5600_t *as5600);

/**
 * @brief Set GPIO OUT pin level
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] value Level to set (0=low, 1=high)
 */
void as5600_set_gpio(as5600_t *as5600, uint8_t value);

// =============================================================================
// I2C REGISTER ACCESS FUNCTIONS
// =============================================================================

/**
 * @brief Read AS5600 register
 * 
 * Reads 1 byte (ZMCO, STATUS, AGC) or 2 bytes (all other registers).
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] reg Register address
 * @param[out] data Pointer to store read value
 */
void as5600_read_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t *data);

/**
 * @brief Write AS5600 register
 * 
 * Writes 1 byte (BURN) or 2 bytes (configuration registers).
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] reg Register address
 * @param[in] data Data value to write
 */
void as5600_write_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t data);

/**
 * @brief Check if register is valid for reading
 * 
 * @param[in] as5600 Pointer to AS5600 instance
 * @param[in] reg Register address
 * @return true if register is readable
 */
bool as5600_is_valid_read_reg(as5600_t *as5600, as5600_reg_t reg);

/**
 * @brief Check if register is valid for writing
 * 
 * @param[in] as5600 Pointer to AS5600 instance
 * @param[in] reg Register address
 * @return true if register is writable
 */
bool as5600_is_valid_write_reg(as5600_t *as5600, as5600_reg_t reg);

// =============================================================================
// CONFIGURATION REGISTER FUNCTIONS
// =============================================================================

/**
 * @brief Set zero position (ZPOS)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] start_position Zero position value (0-4095)
 */
void as5600_set_start_position(as5600_t *as5600, uint16_t start_position);

/**
 * @brief Get zero position (ZPOS)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] start_position Pointer to store zero position value
 */
void as5600_get_start_position(as5600_t *as5600, uint16_t *start_position);

/**
 * @brief Set maximum position (MPOS)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] stop_position Maximum position value (0-4095)
 */
void as5600_set_stop_position(as5600_t *as5600, uint16_t stop_position);

/**
 * @brief Get maximum position (MPOS)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] stop_position Pointer to store maximum position value
 */
void as5600_get_stop_position(as5600_t *as5600, uint16_t *stop_position);

/**
 * @brief Set maximum angle (MANG)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] max_angle Maximum angle value (0-4095)
 */
void as5600_set_max_angle(as5600_t *as5600, uint16_t max_angle);

/**
 * @brief Get maximum angle (MANG)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] max_angle Pointer to store maximum angle value
 */
void as5600_get_max_angle(as5600_t *as5600, uint16_t *max_angle);

/**
 * @brief Set configuration register (CONF)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[in] conf Configuration structure
 */
void as5600_set_conf(as5600_t *as5600, as5600_config_t conf);

/**
 * @brief Get configuration register (CONF)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] conf Pointer to store configuration structure
 */
void as5600_get_conf(as5600_t *as5600, as5600_config_t *conf);


// =============================================================================
// OUTPUT REGISTER FUNCTIONS
// =============================================================================

/**
 * @brief Get raw angle (no zero/max position applied)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] raw_angle Pointer to store raw angle (0-4095)
 */
void as5600_get_raw_angle(as5600_t *as5600, uint16_t *raw_angle);

/**
 * @brief Get scaled angle (with zero/max position applied)
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] angle Pointer to store scaled angle (0-4095)
 */
void as5600_get_angle(as5600_t *as5600, uint16_t *angle);

// =============================================================================
// STATUS REGISTER FUNCTIONS
// =============================================================================

/**
 * @brief Get status register
 * 
 * Status bits include magnet detection (MD), magnet too strong (ML), magnet too weak (MH).
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] status Pointer to store status byte
 */
void as5600_get_status(as5600_t *as5600, uint8_t *status);

/**
 * @brief Get automatic gain control value
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] agc Pointer to store AGC value (0-255)
 */
void as5600_get_agc(as5600_t *as5600, uint8_t *agc);

/**
 * @brief Get magnetic field magnitude
 * 
 * @param[in,out] as5600 Pointer to AS5600 instance
 * @param[out] magnitude Pointer to store magnitude value
 */
void as5600_get_magnitude(as5600_t *as5600, uint16_t *magnitude);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H