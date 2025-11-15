/**
 * @file task_inverse_kinematics.c
 * @brief Inverse kinematics computation task
 * 
 * This task converts desired robot velocities (vx, vy, wz) into target
 * wheel speeds using inverse kinematic equations. It sends wheel targets
 * to the control task via queue.
 * 
 * Features:
 * - Decouples trajectory planning from motor control
 * - Non-blocking queue-based communication
 * - Moderate update frequency (10 ms period)
 * 
 * Communication:
 * - Receives velocity commands via g_velocity_command_queue (from Trajectory task)
 * - Sends wheel targets via g_wheel_target_queue (to Control task)
 * - Updates PID setpoints via g_pid_mutex
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include <math.h>

#include "types_utils.h"
#include "config_utils.h"
#include "kinematics.h"
#include "pid.h"

// =============================================================================
// LOGGING
// =============================================================================

static const char *TAG = "IK_TASK";

// =============================================================================
// EXTERNAL SHARED DATA
// =============================================================================

extern QueueHandle_t g_velocity_command_queue;  // Input: velocity commands
extern QueueHandle_t g_wheel_target_queue;      // Output: wheel targets
extern SemaphoreHandle_t g_pid_mutex;           // PID controller access
extern pid_block_handle_t g_pid[3];

// =============================================================================
// TASK HANDLE (Optional - for external control)
// =============================================================================

TaskHandle_t g_task_kinematics_handle = NULL;

// =============================================================================
// TASK IMPLEMENTATION
// =============================================================================

/**
 * @brief Inverse kinematics computation task
 * 
 * This task runs at moderate frequency (10 ms) to:
 * 1. Receive desired robot velocity command from queue
 * 2. Compute required wheel speeds using inverse kinematics
 * 3. Send wheel targets to control task via queue
 * 4. Update PID controller setpoints
 * 
 * The task uses queues for efficient one-way data flow and only acquires
 * the PID mutex briefly to update setpoints.
 * 
 * @param pvParameters Unused task parameter
 */
void task_inverse_kinematics(void *pvParameters) 
{
    ESP_LOGI(TAG, "Inverse kinematics task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Local variables
    wheel_speeds_t targets = {0};  // Computed wheel speeds
    velocity_t cmd = {0};          // Desired robot velocity

    ESP_LOGI(TAG, "Entering main IK loop");

    // =================================================================
    // MAIN TASK LOOP
    // =================================================================
    
    while (1) {
        // -------------------------------------------------------------
        // 1. RECEIVE VELOCITY COMMAND FROM QUEUE
        // -------------------------------------------------------------
        
        // Try to receive latest command with short timeout
        // If queue is empty, use previous command (maintains last setpoint)
        if (xQueueReceive(g_velocity_command_queue, &cmd, pdMS_TO_TICKS(5)) == pdTRUE) {
            // New command received
        } else {
            // No new command, continue with previous cmd value
        }

        // -------------------------------------------------------------
        // 2. COMPUTE WHEEL SPEEDS (Inverse Kinematics)
        // -------------------------------------------------------------
        
        // Transform robot velocity to wheel angular velocities
        // Uses kinematic equations based on robot geometry
        compute_inverse_kinematics(cmd, &targets);

        // -------------------------------------------------------------
        // 3. SEND WHEEL TARGETS TO CONTROL TASK VIA QUEUE
        // -------------------------------------------------------------
        
        // Overwrite oldest item if queue is full (latest data is most relevant)
        if (xQueueOverwrite(g_wheel_target_queue, &targets) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send wheel targets");
        }

        // -------------------------------------------------------------
        // 4. UPDATE PID SETPOINTS (Thread-safe)
        // -------------------------------------------------------------
        
        // Note: Wheel indexing may differ from motor indexing
        // This mapping handles the physical layout of the robot
        if (xSemaphoreTake(g_pid_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            pid_update_set_point(g_pid[0], targets.phi_dot[0]);  // Motor 0 -> Wheel 0
            pid_update_set_point(g_pid[1], targets.phi_dot[2]);  // Motor 1 -> Wheel 2
            pid_update_set_point(g_pid[2], targets.phi_dot[1]);  // Motor 2 -> Wheel 1
            xSemaphoreGive(g_pid_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire PID mutex for setpoint update");
        }

        // -------------------------------------------------------------
        // 5. OPTIONAL LOGGING (Disabled for performance)
        // -------------------------------------------------------------
        
        // Uncomment for debugging:
        // ESP_LOGI(TAG, "Cmd: vx=%.2f vy=%.2f wz=%.2f | Wheels: [%.2f, %.2f, %.2f]",
        //          cmd.vx, cmd.vy, cmd.wz,
        //          targets.phi_dot[0], targets.phi_dot[1], targets.phi_dot[2]);

        // -------------------------------------------------------------
        // 6. WAIT FOR NEXT CYCLE
        // -------------------------------------------------------------
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(KINEMATICS_TASK_PERIOD_MS));
    }
}
