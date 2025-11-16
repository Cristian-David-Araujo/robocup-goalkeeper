/**
 * @file as5600.h
 * @brief AS5600 magnetic encoder driver for ESP32-S3
 * @details
 * 
 * Driver for the AS5600 12-bit magnetic rotary position sensor.
 * 
 * About the OUT pin in the AS5600 sensor:
 * The ADC of the ESP32 is connected to the OUT pin of the AS5600 sensor.
 * The OUT pin can be configured to output a 10%-90% (VCC) analog signal.
 * Since the ESP32 ADC can only read 0-3.3V, the VCC of the AS5600 sensor must be 3.3V.
 * But there is another problem. The characteristic graph of the ADC (Voltage vs. Digital Value) is not linear on all
 * the range (0-3.3V). It is linear only on the 5%-90% range, approximately.
 * That is why the OUT pin must be configured to output a 10%-90% signal.
 * 
 * @author MaverickST
 * @version 0.0.5
 * @date 2024-11-16
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

#define VCC_3V3_MV          3300        ///< VCC in mV
#define VCC_3V3_MIN_RR_MV   330         ///< VCC minimum range in mV -> 10% of VCC
#define VCC_3V3_MAX_RR_MV   2970        ///< VCC maximum range in mV -> 90% of VCC

#define MAP(val, in_min, in_max, out_min, out_max) ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) ///< Map function
#define ADC_TO_VOLTAGE(val) MAP(val, 0, AS5600_ADC_RESOLUTION_12_BIT, 0, VCC_3V3_MV) ///< ADC to voltage conversion
#define LIMIT(a, min, max) (a < min ? min : (a > max ? max : a)) ///< Limit a value between min and max

#define I2C_MASTER_FREQ_HZ  400000      ///< I2C master clock frequency (400 kHz)
#define AS5600_SENSOR_ADDR  0x36        ///< Slave address for AS5600 sensor

typedef struct
{
    as5600_config_t conf; ///< AS5600 configuration
    as5600_reg_t reg;
    uint8_t out;         ///< GPIO pin connected to the OUT pin of the AS5600 sensor

    // Peripheral handles
    i2c_t i2c_handle;   ///< I2C handle for the AS5600 sensor
    adc_t adc_handle;   ///< ADC handle for the OUT pin
    gpio_t gpio_handle; ///< GPIO handle for the OUT pin

} as5600_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c_num I2C port number
 */
void as5600_init(as5600_t *as5600, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, uint8_t out);

/**
 * @brief Deinitialize the I2C master driver
 * 
 */
void as5600_deinit(as5600_t *as5600);

/**
 * @brief Get angle in degrees from the AS5600 sensor by ADC.
 * Also take into account the range of the OUT pin of the AS5600 sensor, which is 10%-90% of VCC.
 * 
 * @param as5600 
 */
float as5600_adc_get_angle(as5600_t *as5600);

/**
 * @brief The host microcontroller can perform a permanent programming of ZPOS and MPOS with a BURN_ANGLE command.
 * To perform a BURN_ANGLE command, write the value 0x80 into register 0xFF. 
 * The BURN_ANGLE command can be executed up to 3 times
 * ZMCO shows how many times ZPOS and MPOS have been permanently written. 
 * This command may only be executed if the presence of the magnet is detected (MD = 1).
 * 
 * @param as5600 
 */
void as5600_burn_angle_command(as5600_t *as5600);

/**
 * @brief The host microcontroller can perform a permanent writing of MANG and CONFIG with a BURN_SETTING command. 
 * To perform a BURN_SETTING command, write the value 0x40 into register 0xFF. 
 * MANG can be written only if ZPOS and MPOS have never been permanently written (ZMCO = 00). 
 * The BURN_ SETTING command can be performed only one time.
 * 
 * @param as5600 
 */
void as5600_burn_setting_command(as5600_t *as5600);

/**
 * @brief Convert register string to register address
 * 
 * @param reg_str Register string
 * @return as5600_reg_t Register address
 */
as5600_reg_t as5600_reg_str_to_addr(as5600_t *as5600, const char *reg_str);

// --------------------------------------------------------------
// ------------------ GPIO and ADC FUNCTIONS --------------------
// --------------------------------------------------------------

/**
 * @brief Initialize the ADC driver
 * 
 * @param as5600 
 */
void as5600_init_adc(as5600_t *as5600);

/**
 * @brief Initialize the ADC driver with a shared handle
 * 
 * @param as5600 
 * @param shared_handle Shared ADC handle
 */
void as5600_init_adc_shared(as5600_t *as5600, adc_oneshot_unit_handle_t shared_handle);

/**
 * @brief Deinitialize the ADC driver
 * 
 * @param as5600 
 */
void as5600_deinit_adc(as5600_t *as5600);

/**
 * @brief Initialize the GPIO driver
 * 
 * @param as5600 
 */
void as5600_init_gpio(as5600_t *as5600);

/**
 * @brief Deinitialize the GPIO driver
 * 
 * @param as5600 
 */
void as5600_deinit_gpio(as5600_t *as5600);

/**
 * @brief Set the GPIO pin to the specified value
 * 
 * @param value Value to set (0 or 1)
 */
void as5600_set_gpio(as5600_t *as5600, uint8_t value);

// -------------------------------------------------------------
// ---------------------- I2C FUNCTIONS ------------------------
// -------------------------------------------------------------

/**
 * @brief Read register
 * 
 * @param reg Register address
 * @param data Pointer to the data
 */
void as5600_read_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t *data);

/**
 * @brief Write register
 * 
 * @param reg Register address
 * @param data Data to write
 */
void as5600_write_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t data);

/**
 * @brief Check if the register is valid for reading
 * 
 * @param reg Register address
 * @return true if the register is valid
 * @return false if the register is invalid
 */
bool as5600_is_valid_read_reg(as5600_t *as5600, as5600_reg_t reg);

/**
 * @brief Check if the register is valid for writing
 * 
 * @param reg Register address
 * @return true if the register is valid
 * @return false if the register is invalid
 */
bool as5600_is_valid_write_reg(as5600_t *as5600, as5600_reg_t reg);

// -------------------------------------------------------------
// ---------------------- CONFIG REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Set the start position by writing the ZPOS register
 * 
 * @param start_position 
 */
void as5600_set_start_position(as5600_t *as5600, uint16_t start_position);

/**
 * @brief Get the start position by reading the ZPOS register
 * 
 * @param start_position 
 */
void as5600_get_start_position(as5600_t *as5600, uint16_t *start_position);

/**
 * @brief Set the stop position by writing the MPOS register
 * 
 * @param stop_position 
 */
void as5600_set_stop_position(as5600_t *as5600, uint16_t stop_position);

/**
 * @brief Get the stop position by reading the MPOS register
 * 
 * @param stop_position 
 */
void as5600_get_stop_position(as5600_t *as5600, uint16_t *stop_position);

/**
 * @brief Set the maximum angle by writing the MANG register
 * 
 * @param max_angle 
 */
void as5600_set_max_angle(as5600_t *as5600, uint16_t max_angle);

/**
 * @brief Get the maximum angle by reading the MANG register
 * 
 * @param max_angle 
 */
void as5600_get_max_angle(as5600_t *as5600, uint16_t *max_angle);

/**
 * @brief Set the configuration by writing the CONF register
 * 
 * @param conf Configuration
 */
void as5600_set_conf(as5600_t *as5600, as5600_config_t conf);

/**
 * @brief Get the configuration by reading the CONF register
 * 
 * @param conf Configuration
 */
void as5600_get_conf(as5600_t *as5600, as5600_config_t *conf);


// -------------------------------------------------------------
// ---------------------- OUTPUT REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Read RAW ANGLE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_raw_angle(as5600_t *as5600, uint16_t *raw_angle);

/**
 * @brief Read ANGLE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_angle(as5600_t *as5600, uint16_t *angle);

// -------------------------------------------------------------
// ---------------------- STATUS REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Read STATUS register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_status(as5600_t *as5600, uint8_t *status);

/**
 * @brief Read AGC register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_agc(as5600_t *as5600, uint8_t *agc);

/**
 * @brief Read MAGNITUDE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_magnitude(as5600_t *as5600, uint16_t *magnitude);


#endif // AS5600_H