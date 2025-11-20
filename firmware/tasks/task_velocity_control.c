/**
 * @file task_velocity_control.c
 * @brief Robot velocity PID control task (outer loop)
 * 
 * This task implements a cascaded PID control architecture where:
 * - Outer loop (this task): Controls robot velocity (vx, vy, wz)
 * - Inner loop (task_control): Controls individual wheel speeds
 * 
 * The velocity controller:
 * 1. Receives desired velocity from trajectory task
 * 2. Reads measured velocity from SENSOR FUSION (IMU + kinematics) - v2.0
 * 3. Computes PID corrections for tracking errors
 * 4. Sends corrected velocity commands to IK task
 * 
 * Features:
 * - Three independent PID controllers (vx, vy, wz)
 * - Uses fused pose for 1.6× better velocity accuracy (v2.0)
 * - Timeout-based communication for robustness
 * - Graceful degradation on sensor/queue failures
 * - Moderate update frequency (10 ms period)
 * 
 * Communication:
 * - Receives desired velocity via g_desired_velocity_queue (from Trajectory task)
 * - Reads measured velocity via g_fused_pose_mutex (from Fusion task) - v2.0
 * - Sends corrected velocity via g_velocity_command_queue (to IK task)
 * - Accesses g_velocity_pid[] via g_velocity_pid_mutex
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include <math.h>

#include "types_utils.h"
#include "config_utils.h"
#include "pid.h"

// =============================================================================
// LOGGING
// =============================================================================

static const char *TAG = "VEL_CTRL";

// =============================================================================
// EXTERNAL SHARED DATA
// =============================================================================

extern QueueHandle_t g_desired_velocity_queue;   // Input: desired velocity from trajectory
extern QueueHandle_t g_velocity_command_queue;   // Output: corrected velocity to IK
extern SemaphoreHandle_t g_fused_pose_mutex;     // Fused pose access (v2.0)
extern SemaphoreHandle_t g_velocity_pid_mutex;   // Velocity PID controllers access
extern fused_pose_t g_fused_pose;                // Fused pose from sensor fusion (v2.0)
extern pid_block_handle_t g_velocity_pid[3];     // PID controllers: [vx, vy, wz]

// =============================================================================
// TASK HANDLE
// =============================================================================

TaskHandle_t g_task_velocity_control_handle = NULL;

// =============================================================================
// TASK IMPLEMENTATION
// =============================================================================

/**
 * @brief Velocity control task (outer PID loop)
 * 
 * This task runs at moderate frequency (10 ms) to:
 * 1. Receive desired robot velocity from trajectory task
 * 2. Read measured robot velocity from SENSOR FUSION (IMU + kinematics) - v2.0
 * 3. Compute PID corrections for tracking errors (vx, vy, wz)
 * 4. Send corrected velocity commands to IK task
 * 
 * The cascaded architecture allows:
 * - Trajectory planning to be decoupled from control
 * - Robust tracking even with model uncertainties
 * - Independent tuning of velocity vs. wheel control loops
 * - Enhanced accuracy via complementary filter (v2.0)
 * 
 * @param pvParameters Unused task parameter
 */
void task_velocity_control(void *pvParameters)
{
    ESP_LOGI(TAG, "Velocity control task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Local variables
    velocity_t desired = {0};      // Desired velocity from trajectory
    velocity_t measured = {0};     // Measured velocity from sensors
    velocity_t corrected = {0};    // PID-corrected velocity output
    
    uint32_t no_desired_count = 0; // Counter for missing desired velocity
    uint32_t no_measured_count = 0; // Counter for missing measurements

    ESP_LOGI(TAG, "Entering main velocity control loop");

    // =================================================================
    // MAIN TASK LOOP
    // =================================================================
    
    while (1) {
        // -------------------------------------------------------------
        // 1. RECEIVE DESIRED VELOCITY FROM TRAJECTORY TASK
        // -------------------------------------------------------------
        
        if (xQueueReceive(g_desired_velocity_queue, &desired, pdMS_TO_TICKS(5)) == pdTRUE) {
            no_desired_count = 0;
        } else {
            // No new desired velocity, continue with previous value
            no_desired_count++;
            if (no_desired_count % 500 == 0) {
                ESP_LOGW(TAG, "No desired velocity for %lu cycles", no_desired_count);
            }
        }

        // -------------------------------------------------------------
        // 2. READ MEASURED VELOCITY FROM SENSOR FUSION (v2.0)
        // -------------------------------------------------------------
        
        if (g_fused_pose_mutex && 
            xSemaphoreTake(g_fused_pose_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Extract velocity from fused pose (1.6× better accuracy)
            measured.vx = g_fused_pose.vel_x;
            measured.vy = g_fused_pose.vel_y;
            measured.wz = g_fused_pose.vel_angular;
            xSemaphoreGive(g_fused_pose_mutex);
            no_measured_count = 0;
        } else {
            // Timeout acquiring fused pose - use previous value
            no_measured_count++;
            if (no_measured_count % 500 == 0) {
                ESP_LOGW(TAG, "Cannot read fused velocity for %lu cycles", no_measured_count);
            }
        }

        // -------------------------------------------------------------
        // 3. COMPUTE PID CORRECTIONS FOR VELOCITY TRACKING
        // -------------------------------------------------------------
        
        // OPEN-LOOP TEST: PID disabled, pass through desired velocity
        // corrected = desired;
        
        //CLOSED-LOOP CONTROL (currently disabled):
        if (g_velocity_pid_mutex && 
            xSemaphoreTake(g_velocity_pid_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            // Update setpoints to desired velocities
            pid_update_set_point(g_velocity_pid[0], desired.vx);
            pid_update_set_point(g_velocity_pid[1], desired.vy);
            pid_update_set_point(g_velocity_pid[2], desired.wz);
            
            // Compute PID outputs (error = setpoint - measured)
            // PID outputs are velocity corrections
            float vx_correction, vy_correction, wz_correction;
            pid_compute(g_velocity_pid[0], measured.vx, &vx_correction);
            pid_compute(g_velocity_pid[1], measured.vy, &vy_correction);
            pid_compute(g_velocity_pid[2], measured.wz, &wz_correction);
            
            xSemaphoreGive(g_velocity_pid_mutex);
            
            // Corrected velocity = desired + PID correction
            // Note: PID already accounts for error internally, so output is the corrected command
            corrected.vx = vx_correction;
            corrected.vy = vy_correction;
            corrected.wz = wz_correction;
            
        } else {
            // Timeout acquiring PID mutex - pass through desired velocity
            ESP_LOGW(TAG, "PID mutex timeout - using feedforward control");
            corrected = desired;
        }
        

        // -------------------------------------------------------------
        // 4. SEND CORRECTED VELOCITY TO IK TASK
        // -------------------------------------------------------------
        
        if (xQueueSend(g_velocity_command_queue, &corrected, pdMS_TO_TICKS(5)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send corrected velocity (queue full)");
        }

        // -------------------------------------------------------------
        // 5. PERIODIC LOGGING (Every 1 second)
        // -------------------------------------------------------------
        
        static uint32_t log_counter = 0;
        log_counter++;
        if (log_counter % 100 == 0) {  // 10ms period * 100 = 1 second
            ESP_LOGI(TAG, "Desired: vx=%.3f vy=%.3f wz=%.3f", 
                     desired.vx, desired.vy, desired.wz);
            ESP_LOGI(TAG, "Measured: vx=%.3f vy=%.3f wz=%.3f", 
                     measured.vx, measured.vy, measured.wz);
            ESP_LOGI(TAG, "Corrected: vx=%.3f vy=%.3f wz=%.3f", 
                     corrected.vx, corrected.vy, corrected.wz);
        }

        // Wait for next control cycle
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(VELOCITY_CONTROL_TASK_PERIOD_MS));
    }
}
