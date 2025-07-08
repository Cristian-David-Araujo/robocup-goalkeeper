#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#include <math.h>

#include "types_utils.h"
#include "config_utils.h"  // For robot configuration constants
#include "kinematics.h"  // For inverse kinematics functions
#include "pid.h"  // For PID control


// Shared commands
extern Velocity robot_command;           // {vx, vy, wz}
extern WheelSpeeds wheel_targets;        // φ̇_1, φ̇_2, φ̇_3
extern SemaphoreHandle_t xCmdMutex;

extern SemaphoreHandle_t xPidMutex; // Mutex for PID control
extern pid_block_handle_t pid[3]; ///< Array of PID controllers for each motor

TaskHandle_t xTaskKinematicsHandle = NULL;

void vTaskInverseKinematics(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t timestamp_us = 0;
    WheelSpeeds targets = {0}; // Initialize wheel speeds to zero
    Velocity cmd = {0}; // Initialize command to zero
    
    while (1) {
        // Get the latest robot command
        if (xSemaphoreTake(xCmdMutex, portMAX_DELAY) == pdTRUE) {
            cmd = robot_command;
            xSemaphoreGive(xCmdMutex);
        }

        // Compute wheel speeds (inverse kinematics)
        compute_inverse_kinematics(cmd, &targets);  // implemented below

        // Save to global
        if (xSemaphoreTake(xCmdMutex, portMAX_DELAY) == pdTRUE) {
            wheel_targets = targets;
            xSemaphoreGive(xCmdMutex);
        }

        // Set the setpoint for each motor using the PID controller
        if (xSemaphoreTake(xPidMutex, portMAX_DELAY) == pdTRUE) {
            pid_update_set_point(pid[0], targets.phi_dot[0]);
            pid_update_set_point(pid[1], targets.phi_dot[2]);
            pid_update_set_point(pid[2], targets.phi_dot[1]);
            xSemaphoreGive(xPidMutex);
        }

        // // Debug output every 1s
        // if ((timestamp_us % 1000000) == 0) { // every 20
        //     printf("IK,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
        //            timestamp_us,
        //            cmd.vx, cmd.vy, cmd.wz,
        //            targets.phi_dot[0], targets.phi_dot[1], targets.phi_dot[2]);
        // }

        // timestamp_us += KINEMATICS_TASK_PERIOD_MS * 1000; // Increment timestamp by task period in microseconds

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(KINEMATICS_TASK_PERIOD_MS));
    }
}