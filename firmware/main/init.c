#include "init.h"

int init_sensors(void)
{

    /*Set the parameters for the AS5600 sensors*/
    as5600_0.out = GPIO_ENCODER_0_IN_ANALOG; // Set the OUT pin for AS5600 0
    as5600_1.out = GPIO_ENCODER_1_IN_ANALOG; // Set the OUT pin for AS5600 1
    as5600_2.out = GPIO_ENCODER_2_IN_ANALOG; // Set the OUT pin for AS5600 2

    int status = INIT_SUCCESS;

    // Initialize AS5600 sensors
    
    status = AS5600_Init_ADC(&as5600_0);
    if (status != INIT_SUCCESS) {
        return INIT_ERROR_AS5600; // Return error if AS5600 0 initialization fails
    }
    status = AS5600_Init_ADC(&as5600_1);
    if (status != INIT_SUCCESS) {
        return INIT_ERROR_AS5600; // Return error if AS5600 1 initialization fails
    }
    status = AS5600_Init_ADC(&as5600_2);
    if (status != INIT_SUCCESS) {
        return INIT_ERROR_AS5600; // Return error if AS5600 2 initialization fails
    }


    // Initialize BNO055 sensor
    status = BNO055_Init(&bno055, GPIO_IMU_UART_TX, GPIO_IMU_UART_RX);
    if (status != INIT_SUCCESS) {
        return INIT_ERROR_BNO055; // Return error if BNO055 initialization fails
    }

    
    return status; // Return success if all sensors are initialized successfully

}
