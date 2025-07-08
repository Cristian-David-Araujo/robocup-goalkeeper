#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include <stdio.h>

#include "pid.h"
#include "init.h"
#include "motor.h"
#include "as5600.h"
#include "types_utils.h"

// Forward declaration for UART initialization if not included by a header
void uart_init_task(void);

// Forward declaration of the sensor reading task function
void vTaskReadSensors(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskUart(void* arg);
void vTaskUartHandler(void *arg);
void vTaskUartParser(void *arg);
void vTaskInverseKinematics(void *pvParameters);

motor_brushless_t motor[3]; ///< Array of brushless motors
AS5600_t as5600[3]; ///< Array of AS5600 sensors
adc_oneshot_unit_handle_t shared_adc_handle;


pid_block_handle_t pid[3]; ///< Array of PID controllers for each motor
pid_parameter_t pid_param = {
        .kp = PID_MOTOR_KP,
        .ki = PID_MOTOR_KI,
        .kd = PID_MOTOR_KD,
        .max_output = PID_MOTOR_MAX_OUTPUT, // Set maximum output for PID controller
        .min_output = PID_MOTOR_MIN_OUTPUT, // Set minimum output for PID controller
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .beta = PID_MOTOR_BETA // Set beta filter coefficient for derivative term
    };

RawSensorData sensor_data = {0};  //< Initialize with zeros
Velocity robot_command;           //< {vx, vy, wz}
WheelSpeeds wheel_targets;  //< φ̇_1, φ̇_2, φ̇_3

SemaphoreHandle_t xCmdMutex;
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xPidMutex = NULL; // Mutex for PID control
SemaphoreHandle_t xADCMutex = NULL; // Mutex for ADC operations

TaskHandle_t xHandleParserTask;

#define CIRCULAR_RADIUS 1.0f
#define OMEGA_CIRC 0.5f                // Velocidad angular (rad/s)
#define DT_SECONDS 0.02f               // Periodo de actualización (segundos)
#define TASK_PERIOD_MS ((int)(DT_SECONDS * 1000))

/**
 * @brief Task that moves the motors forward and backward periodically.
 * 
 * This task alternates between setting the motors to move forward and backward
 * every second, using predefined setpoints for each motor.
 * 
 * @param arg Unused
 */
void vTaskMove(void* arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float t = 0.0f;
    float prev_angle = 0.0f;

    while (1) {
        // Calculate the command velocities based on a circular trajectory
        float vx_cmd = -CIRCULAR_RADIUS * OMEGA_CIRC * sinf(OMEGA_CIRC * t);
        float vy_cmd =  CIRCULAR_RADIUS * OMEGA_CIRC * cosf(OMEGA_CIRC * t);
        float angle = atan2f(vy_cmd, vx_cmd);
        float omega_cmd = (angle - prev_angle) / DT_SECONDS;

        if (omega_cmd > M_PI / DT_SECONDS) omega_cmd -= 2 * M_PI / DT_SECONDS;
        if (omega_cmd < -M_PI / DT_SECONDS) omega_cmd += 2 * M_PI / DT_SECONDS;

        prev_angle = angle;

        // Write the command to the shared robot_command structure
        if (xCmdMutex && xSemaphoreTake(xCmdMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            robot_command.vx = vx_cmd;
            robot_command.vy = vy_cmd;
            robot_command.wz = omega_cmd;
            xSemaphoreGive(xCmdMutex);
        }

        // Log the command for debugging
        printf("t=%.2f  cmd: vx=%.3f, vy=%.3f, wz=%.3f \n", t, vx_cmd, vy_cmd, omega_cmd);

        // Wait for the next cycle
        t += DT_SECONDS;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

void app_main(void)
{
    init_sensors(); // Initialize sensors
    init_motors();  // Initialize motors
    init_pid(); // Initialize PID controller
    // uart_init_task(); // Initialize UART
    // motor_calibration3(&motor_0, &motor_1, &motor_2); // Calibrate all motors
    vTaskDelay(pdMS_TO_TICKS(3000)); // Delay to allow sensors to stabilize

    // Initialize the shared data mutex
    xSensorDataMutex = xSemaphoreCreateMutex();
    xPidMutex = xSemaphoreCreateMutex();
    xADCMutex = xSemaphoreCreateMutex(); // Create mutex for ADC operations
    xCmdMutex = xSemaphoreCreateMutex(); // Create mutex for command data


    // Start the sensor reading task with higher priority
    xTaskCreate(vTaskReadSensors, "Sensor Task", 4096, NULL, 6, NULL);
    // Start the control task with medium priority
    xTaskCreate(vTaskControl, "Control Task", 4096, NULL, 3, NULL);
    // Start the inverse kinematics task with higher priority
    xTaskCreate(vTaskInverseKinematics, "IK", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskMove, "Move Task", 2048, NULL, 4, NULL); // Start the move task with lower priority
    // // Start the UART tunning the pid parameters task
    // xTaskCreate(vTaskUartHandler, "uart_handler", 4096, NULL, 10, NULL);
    // xTaskCreate(vTaskUartParser, "uart_parser", 4096, NULL, 10, &xHandleParserTask);

    // motor_set_speed(&motor[0], MOTOR_DIRECTION_FORWARD(0) * 20.0f); // Set motor 0 speed to 20% in forward direction
    // motor_set_speed(&motor[1], MOTOR_DIRECTION_FORWARD(1) * 20.0f); // Set motor 1 speed to 20% in forward direction
    // motor_set_speed(&motor[2], MOTOR_DIRECTION_FORWARD(2) * 20.0f); // Set motor 2 speed to 20% in forward direction
    
}