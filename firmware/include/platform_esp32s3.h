/**
 * @file platform_esp32s3.h
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PLATFORM_ESP32S3_H
#define PLATFORM_ESP32S3_H

// Standard C includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// ESP32-S3 specific includes
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_check.h"


// Constants for UART configuration
#define UART_PIN_NO_USE UART_PIN_NO_CHANGE // Do not use pin for UART configuration UART_PIN_NO_CHANGE is from SDK or ESP-IDF

/**
 * @brief Structure for UART configuration
 * 
 */
typedef struct {
    uint32_t baud_rate;         // Baud rate for UART communication
    uint16_t buffer_size;       // Buffer size for UART communication
    uint8_t gpio_tx;            // GPIO pin for UART TX
    uint8_t gpio_rx;            // GPIO pin for UART RX
    uint8_t gpio_rts;           // GPIO pin for UART RTS
    uint8_t gpio_cts;           // GPIO pin for UART CTS

    // UART configuration structure for ESP32-S3
    uart_port_t uart_num;       // UART port number
    uart_config_t uart_config;  // UART configuration structure
    uart_event_t uart_event;    // UART event structure
    uint32_t event_queue_size;  // Event queue size for UART communication
    QueueHandle_t event_queue;  // Event queue for UART communication
} uart_t;

/**
 * @brief Initialize UART peripheral with custom configuration
 * 
 * @param uart_config UART configuration structure
 * @param baud_rate Baud rate for UART communication
 * @param buffer_size Buffer size for UART communication
 * @param gpio_tx GPIO pin for UART TX
 * @param gpio_rx GPIO pin for UART RX
 * @param gpio_rts GPIO pin for UART RTS
 * @param gpio_cts GPIO pin for UART CTS
 * 
 * @return true if UART initialization was successful and false if UART initialization failed
 */
int uart_init(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, int8_t gpio_tx, int8_t gpio_rx, int8_t gpio_rts, int8_t gpio_cts);

/**
 * @brief Initialize UART peripheral with default configuration
 * 
 * @param uart_t UART configuration structure
 * @param baud_rate Baud rate for UART communication
 * @param buffer_size Buffer size for UART communication
 * @param gpio_tx GPIO pin for UART TX
 * @param gpio_rx GPIO pin for UART RX
 * 
 * @return true if UART initialization was successful and false if UART initialization failed
 */
static inline bool uart_init_with_defaults(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, int8_t gpio_tx, int8_t gpio_rx) {
    return uart_init(uart_config, baud_rate, buffer_size, gpio_tx, gpio_rx, UART_PIN_NO_USE, UART_PIN_NO_USE);
}

/**
 * @brief Write data to the UART
 * 
 * @param uart_config Pointer to the uart_t structure with initialized configuration
 * @param data Pointer to the data buffer to be written
 * @param length Length of the data buffer
 * @return int Number of bytes written, or -1 if an error occurred
 */
int uart_write(uart_t *uart_config, const uint8_t *data, size_t length);

/**
 * @brief Read data from the UART
 * 
 * @param uart_config Pointer to the uart_t structure with initialized configuration
 * @param buffer Pointer to the buffer to store the read data
 * @param length Maximum number of bytes to read into the buffer
 * @param timeout_ms Time in milliseconds to wait for data
 * @return int Number of bytes read, or -1 if an error occurred
 */
int uart_read(uart_t *uart_config, uint8_t *buffer, size_t length, int timeout_ms);

/**
 * @brief Flush the UART buffer
 * 
 * @param uart_config Pointer to the uart_t structure with initialized configuration
 * @return int 0 if the buffer was flushed successfully, or -1 if an error occurred
 */
int uart_clear(uart_t *uart_config);



// -------------------------------------------------------------
// ---------------------- I2C MASTER ---------------------------
// -------------------------------------------------------------

#include "driver/i2c_master.h"
#define I2C_TIMEOUT_MS 100

static const char* TAG_I2C = "i2c";

/**
 * @brief I2C master driver structure
 * 
 */
typedef struct 
{
    uint8_t addr; // I2C device address
    uint32_t clk_speed_hz;

    i2c_port_t i2c_num; // I2C port number
    uint8_t gpio_scl;
    uint8_t gpio_sda;

    // I2C configuration structure for ESP32 - S3
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle;
} i2c_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c 
 * @param i2c_num 
 * @param gpio_scl 
 * @param gpio_sda 
 * @param clk_speed_hz 
 * @param addr 
 * @return true if the I2C is initialized correctly
 * @return false if the I2C initialization failed
 */
bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr);

/**
 * @brief Deinitialize the I2C master driver
 * 
 * @param i2c 
 */
void i2c_deinit(i2c_t *i2c);

/**
 * @brief Given a register address, read the data from the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 * @return true  if the read operation was successful
 * @return false  if the read operation failed
 */
bool i2c_read_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Given a register address, write the data to the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 * @return true if the write operation was successful
 * @return false if the write operation failed
 */
bool i2c_write_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Write data to the I2C bus
 * 
 * @param i2c 
 * @param data 
 * @param len 
 */
void i2c_write(i2c_t *i2c, uint8_t *data, size_t len);



// -------------------------------------------------------------
// ---------------------- ADC ---------------------------------
// -------------------------------------------------------------

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "sdkconfig.h"
#include "stdatomic.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#define ADC_CONF_UNIT           ADC_UNIT_1   /*!< ADC unit for ADC1 */
#define ADC_RESOLUTION_12_BIT   4095         /*!< 12-bit resolution for ADC */  

#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_CHANNEL_COUNT       1

static const char* TAG_ADC = "adc";

typedef struct
{
    uint8_t gpio_out;
    adc_channel_t chan;
    adc_unit_t unit;
    bool is_calibrated;

    adc_cali_handle_t adc_cali_handle;
    adc_oneshot_unit_handle_t adc_handle;
} adc_t;


/**
 * @brief Create a new ADC unit
 * 
 * @param handle Pointer to the ADC unit handle
 * @return true if the ADC unit is created successfully
 * @return false if the ADC unit creation failed
 */
bool adc_create_unit(adc_oneshot_unit_handle_t *handle);

/**
 * @brief Configure the ADC channel with the specified GPIO pin and shared ADC handle
 * 
 * @param adc struct to store the ADC configuration
 * @param gpio_out GPIO pin connected to the ADC output
 * @param shared_handle Shared ADC handle for calibration
 * @return true if the ADC channel is configured correctly
 * @return false if the ADC channel configuration failed
 */
bool adc_config_channel(adc_t *adc, uint8_t gpio_out, adc_oneshot_unit_handle_t shared_handle);

/**
 * @brief Initialize the ADC driver
 * 
 * @param adc struct to store the ADC configuration
 * @param gpio_out GPIO pin connected to the ADC output
 * @return true if the ADC is initialized correctly
 * @return false if the ADC initialization failed
 */
bool adc_init(adc_t *adc, uint8_t gpio_out);

bool adc_new_unit(adc_t *adc, uint8_t gpio_out);

bool adc_new_channel_cali(adc_t *adc, uint8_t gpio_out, adc_oneshot_unit_handle_t adc_handle);

/**
 * @brief Deinitialize the ADC driver
 * 
 * @param adc struct to store the ADC configuration
 * @return true if the ADC is deinitialized correctly
 * @return false if the ADC deinitialization failed
 */
bool adc_deinit(adc_t *adc);

/**
 * @brief Read the raw ADC value. Range: 0 - 4095
 * 
 * @param adc 
 * @param raw 
 */
void adc_read_raw(adc_t *adc, int *raw);

/**
 * @brief Read the ADC value and convert it to millivolts
 * 
 * @param adc 
 * @param raw 
 * @param angle 
 */
void adc_read_mvolt(adc_t *adc, uint16_t *mvolt);


// -------------------------------------------------------------
// ---------------------- GPIO ---------------------------------
// -------------------------------------------------------------

#include "driver/gpio.h"
#include "esp_err.h"

static const char* TAG_GPIO = "gpio";

typedef struct
{
    uint8_t gpio_num;
    gpio_int_type_t intr_type;
    gpio_mode_t mode;
    gpio_pull_mode_t pullup_en;
    gpio_pull_mode_t pulldown_en;
} gpio_t;

/**
 * @brief Initialize the GPIO driver
 * 
 * @param gpio struct to store the GPIO configuration
 * @param gpio_num GPIO pin number
 * @param mode 1 = INPUT, 2 = OUTPUT, 
 * @param pulldown_en 0 = disable, 1 = enable
 * @param pullup_en 0 = disable, 1 = enable
 * @return true if the GPIO is initialized correctly
 * @return false if the GPIO initialization failed
 */
bool gpio_init_basic(gpio_t *gpio, uint8_t gpio_num, uint8_t mode, bool pulldown_en, bool pullup_en);

/**
 * @brief Deinitialize the GPIO driver
 * 
 * @param gpio struct to store the GPIO configuration
 * @return true if the GPIO is deinitialized correctly
 * @return false if the GPIO deinitialization failed
 */
bool gpio_deinit(gpio_t *gpio);

/**
 * @brief Set the GPIO pin to HIGH
 * 
 * @param gpio struct to store the GPIO configuration
 */
void gpio_set_high(gpio_t *gpio);

/**
 * @brief Set the GPIO pin to LOW
 * 
 * @param gpio struct to store the GPIO configuration
 */
void gpio_set_low(gpio_t *gpio);


#endif // PLATFORM_ESP32S3_H