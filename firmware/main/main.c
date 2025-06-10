#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <stdio.h>

#include "pid.h"
#include "init.h"
#include "motor.h"
#include "as5600.h"

motor_brushless_t motor_0; // Motor 0 structure
AS5600_t as5600_0; // AS5600 sensor structure|

void app_main(void)
{
    init_sensors(); // Initialize sensors
    init_motors();  // Initialize motors
    motor_calibration(&motor_0); // Calibrate the motor

    int speed = 0; // Initial speed for the motor
    while (1)
    {
        printf("Actual position: %f\n", AS5600_ADC_GetAngle(&as5600_0)); // Read and print the angle from AS5600
        printf("Motor speed: %d\n", speed); // Print the current motor speed

        motor_set_speed(&motor_0, speed); // Set the motor speed
        speed += 10; // Increment speed by 10
        if (speed > 100) // If speed exceeds 1000, reset to 0
        {
            speed = -100; // Reset speed to -100
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }


}