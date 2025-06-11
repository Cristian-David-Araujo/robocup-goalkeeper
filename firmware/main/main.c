#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <stdio.h>

#include "pid.h"
#include "init.h"
#include "motor.h"
#include "as5600.h"
#include "types.h"

// Forward declaration of the sensor reading task function
void vTaskReadSensors(void *pvParameters);
void vTaskControl(void *pvParameters);

motor_brushless_t motor_0; // Motor 0 structure
AS5600_t as5600_0; // AS5600 sensor structure|
pid_block_handle_t pid;

RawSensorData sensor_data = {0};  // Initialize with zeros
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control


void app_main(void)
{
    init_sensors(); // Initialize sensors
    init_motors();  // Initialize motors
    motor_calibration(&motor_0); // Calibrate the motor

    // Initialize the shared data mutex
    xSensorDataMutex = xSemaphoreCreateMutex();
    xPidMutex = xSemaphoreCreateMutex();

    // Start the sensor reading task
    xTaskCreate(vTaskReadSensors, "SensorTask", 2048, NULL, 5, NULL);

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