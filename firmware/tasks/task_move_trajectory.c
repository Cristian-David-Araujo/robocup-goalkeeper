/**
 * @file task_move_trajectory.c
 * @brief Circular trajectory generation task implementation
 * 
 * This task generates desired velocity commands to make the robot follow
 * a circular trajectory. It computes vx, vy, and wz commands based on
 * parametric circular motion equations.
 * 
 * Task characteristics:
 * - Period: 20 ms (50 Hz)
 * - Priority: 2 (low - reference generation)
 * - Stack: 2048 bytes
 * 
 * Communication:
 * - Sends velocity commands via g_desired_velocity_queue
 * - Reads fused velocity for logging via g_fused_pose_mutex (v2.0)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

#include "main.h"
#include "types_utils.h"

// =============================================================================
// LOGGING TAG
// =============================================================================

static const char *TAG = "TRAJECTORY";

// =============================================================================
// TASK HANDLE
// =============================================================================

TaskHandle_t g_task_trajectory_handle = NULL;

// =============================================================================
// TRAJECTORY PARAMETERS
// =============================================================================

#define CIRCULAR_RADIUS 1.0f       ///< Radius of circular trajectory (m)
#define OMEGA_CIRC 0.5f            ///< Angular velocity for circular motion (rad/s)
#define DT_SECONDS 0.02f           ///< Control loop period (s)
#define TASK_PERIOD_MS ((int)(DT_SECONDS * 1000))  ///< Period in milliseconds

// =============================================================================
// TASK IMPLEMENTATION
// =============================================================================

/**
 * @brief Task that generates a circular trajectory for robot motion
 * 
 * This task computes desired velocities (vx, vy, wz) to make the robot
 * follow a circular path. It sends velocity commands via queue to the
 * velocity control task (outer PID loop).
 * 
 * Circular trajectory equations:
 * - Position: x(t) = R*cos(ωt), y(t) = R*sin(ωt)
 * - Velocity: vx(t) = -R*ω*sin(ωt), vy(t) = R*ω*cos(ωt)
 * - Angular velocity: computed from heading change
 * 
 * @param arg Unused task parameter
 */
void task_move_trajectory(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    float t = 0.0f;                      // Time accumulator
    float prev_angle = 0.0f;             // Previous trajectory angle
    velocity_t speed_estimated = {0};    // Local copy of estimated velocity
    velocity_t velocity_cmd = {0};       // Command to send to velocity control task

    ESP_LOGI(TAG, "Trajectory task started");

    while (1) {
        // =================================================================
        // CIRCULAR TRAJECTORY COMPUTATION
        // =================================================================
        
        // Parametric circular trajectory: x(t) = R*cos(ωt), y(t) = R*sin(ωt)
        // Velocity: vx(t) = -R*ω*sin(ωt), vy(t) = R*ω*cos(ωt)
        float vx_cmd = -CIRCULAR_RADIUS * OMEGA_CIRC * sinf(OMEGA_CIRC * t);
        float vy_cmd =  CIRCULAR_RADIUS * OMEGA_CIRC * cosf(OMEGA_CIRC * t);
        
        // Compute angular velocity from change in heading
        float angle = atan2f(vy_cmd, vx_cmd);
        float omega_cmd = (angle - prev_angle) / DT_SECONDS;
        
        // Handle angle wrap-around
        if (omega_cmd > M_PI / DT_SECONDS) {
            omega_cmd -= 2.0f * M_PI / DT_SECONDS;
        }
        if (omega_cmd < -M_PI / DT_SECONDS) {
            omega_cmd += 2.0f * M_PI / DT_SECONDS;
        }
        
        prev_angle = angle;

        // =================================================================
        // SEND VELOCITY COMMAND TO VELOCITY CONTROL TASK (Via Queue)
        // =================================================================
        
        velocity_cmd.vx = vx_cmd;
        velocity_cmd.vy = vy_cmd;
        velocity_cmd.wz = omega_cmd;
        
        // Send desired velocity to outer-loop velocity PID controller
        if (xQueueSend(g_desired_velocity_queue, &velocity_cmd, pdMS_TO_TICKS(5)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send desired velocity command (queue full)");
        }

        // =================================================================
        // READ FUSED VELOCITY FOR LOGGING (Thread-safe) - v2.0
        // =================================================================
        
        if (g_fused_pose_mutex && 
            xSemaphoreTake(g_fused_pose_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            speed_estimated.vx = g_fused_pose.vel_x;
            speed_estimated.vy = g_fused_pose.vel_y;
            speed_estimated.wz = g_fused_pose.vel_angular;
            xSemaphoreGive(g_fused_pose_mutex);
        }

        // =================================================================
        // PERIODIC LOGGING (Every 1 second)
        // =================================================================
        
        if ((int)(t * 1000) % 1000 == 0) {
            ESP_LOGI(TAG, "Move t=%.2f, cmd: vx=%.2f vy=%.2f wz=%.2f", 
                     t, vx_cmd, vy_cmd, omega_cmd);
            ESP_LOGI(TAG, "Estimated: vx=%.2f vy=%.2f wz=%.2f", 
                     speed_estimated.vx, speed_estimated.vy, speed_estimated.wz);
        }

        // Update time and wait for next cycle
        t += DT_SECONDS;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
