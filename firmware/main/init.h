#ifndef INIT_H
#define INIT_H




/* Header file for sensor initialization */
#include "as5600.h"
#include "bno055.h"
#include "platform_esp32s3.h"

#include "utils/gpio_utils.h"


// Define macros for error handling
#define INIT_SUCCESS 0
#define INIT_ERROR_AS5600 1
#define INIT_ERROR_BNO055 2




extern AS5600_t as5600_0; ///< AS5600 sensor structure
extern AS5600_t as5600_1; ///< AS5600 sensor structure
extern AS5600_t as5600_2; ///< AS5600 sensor structure

extern BNO055_t bno055; ///< BNO055 sensor structure

/**
 * @brief Initialize the sensors
 * This function initializes the AS5600 and BNO055 sensors, sets up the UART for communication,
 * and configures the GPIO pins for the sensors.
 * @param AS5600_t *as5600 Pointer to the AS5600 sensor structure
 * @param BNO055_t *bno055 Pointer to the BNO055 sensor structure
 * @return int
 *         - INIT_SUCCESS if initialization is successful
 *        - INIT_ERROR_AS5600 if AS5600 initialization fails
 *        - INIT_ERROR_BNO055 if BNO055 initialization fails
 */
int init_sensors(void);


#endif // INIT_H