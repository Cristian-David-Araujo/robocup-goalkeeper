#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <stdio.h>

#include "pid.h"
#include "init.h"
#include "motor.h"
#include "as5600.h"
#include "types.h"

// Forward declaration for UART initialization if not included by a header
void uart_init_task(void);

// Forward declaration of the sensor reading task function
void vTaskReadSensors(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskUart(void* arg);
void vTaskUartHandler(void *arg);
void vTaskUartParser(void *arg);

motor_brushless_t motor_0; // Motor 0 structure
AS5600_t as5600_0; // AS5600 sensor structure|
pid_block_handle_t pid;
pid_parameter_t pid_param = {
        .kp = PID_MOTOR_KP,
        .ki = PID_MOTOR_KI,
        .kd = PID_MOTOR_KD,
        .max_output = PID_MOTOR_MAX_OUTPUT, // Set maximum output for PID controller
        .min_output = PID_MOTOR_MIN_OUTPUT, // Set minimum output for PID controller
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .beta = PID_MOTOR_BETA // Set beta filter coefficient for derivative term
    };

RawSensorData sensor_data = {0};  // Initialize with zeros
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control
SemaphoreHandle_t xADCMutex = NULL; // Mutex for ADC operations

TaskHandle_t xHandleParserTask;

void app_main(void)
{
    init_sensors(); // Initialize sensors
    init_motors();  // Initialize motors
    init_pid(); // Initialize PID controller
    uart_init_task(); // Initialize UART
    motor_calibration(&motor_0); // Calibrate the motor

    // Initialize the shared data mutex
    xSensorDataMutex = xSemaphoreCreateMutex();
    xPidMutex = xSemaphoreCreateMutex();
    xADCMutex = xSemaphoreCreateMutex(); // Create mutex for ADC operations

    

    // Start the sensor reading task with higher priority
    xTaskCreate(vTaskReadSensors, "Sensor Task", 2048, NULL, 6, NULL);
    // Start the control task with medium priority
    xTaskCreate(vTaskControl, "Control Task", 4096, NULL, 5, NULL);
    // Start the UART tunning the pid parameters task
    xTaskCreate(vTaskUartHandler, "uart_handler", 4096, NULL, 10, NULL);
    xTaskCreate(vTaskUartParser, "uart_parser", 4096, NULL, 10, &xHandleParserTask);
}