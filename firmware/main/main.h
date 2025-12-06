/**
 * @file main.h
 * @brief Global system-level declarations for RoboCup Goalkeeper firmware
 * 
 * This header provides access to all system-wide variables, handles, and
 * synchronization primitives used across multiple tasks.
 * 
 * All global variables are defined in main.c and declared extern here.
 * Tasks and modules should include this header to access shared resources.
 */

#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "as5600.h"
#include "bno055.h"
#include "motor.h"
#include "pid.h"
#include "types_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations for ESP-IDF types (avoid including heavy headers)
typedef struct adc_oneshot_unit_ctx_t *adc_oneshot_unit_handle_t;

// =============================================================================
// HARDWARE INSTANCE DECLARATIONS
// =============================================================================

/// @brief Array of three brushless motor controllers
extern motor_brushless_t g_motor[3];

/// @brief Array of three AS5600 magnetic encoders
extern as5600_t g_as5600[3];

/// @brief BNO055 IMU sensor instance (currently unused)
extern bno055_t g_bno055;

/// @brief Shared ADC handle for encoder analog reading
extern adc_oneshot_unit_handle_t g_shared_adc_handle;

/// @brief Array of three PID controller handles
extern pid_block_handle_t g_pid[3];

/// @brief PID controller tuning parameters (shared by all motors)
extern pid_parameter_t g_pid_param;

/// @brief Array of three velocity PID controller handles (vx, vy, wz)
extern pid_block_handle_t g_velocity_pid[3];

/// @brief Velocity PID controller tuning parameters (shared by vx, vy, wz)
extern pid_parameter_t g_velocity_pid_param;

// =============================================================================
// SHARED DATA STRUCTURES
// =============================================================================

/// @brief Raw sensor readings from all encoders
/// Protected by: g_sensor_data_mutex
/// Writers: Sensor task
/// Readers: Control task
extern raw_sensor_data_t g_sensor_data;

/// @brief Estimated robot velocity (from forward kinematics)
/// Protected by: g_estimated_data_mutex
/// Writers: Sensor task
/// Readers: Trajectory task (for logging)
extern velocity_t g_robot_estimated;

// =============================================================================
// SYNCHRONIZATION PRIMITIVES
// =============================================================================

/// @brief Queue for desired velocity commands (Trajectory → Velocity Control task)
/// Item type: velocity_t
/// Size: 2 items (only latest command matters)
extern QueueHandle_t g_desired_velocity_queue;

/// @brief Queue for corrected velocity commands (Velocity Control → IK task)
/// Item type: velocity_t
/// Size: 2 items (PID-corrected velocity commands)
extern QueueHandle_t g_velocity_command_queue;

/// @brief Queue for wheel speed targets (IK → Control task)
/// Item type: wheel_speeds_t
/// Size: 2 items (only latest targets matter)
extern QueueHandle_t g_wheel_target_queue;

/// @brief Mutex protecting g_sensor_data
/// Shared between: Sensor task (write), Control task (read)
extern SemaphoreHandle_t g_sensor_data_mutex;

/// @brief Mutex protecting g_pid array
/// Shared between: IK task (setpoint updates), Control task (compute)
extern SemaphoreHandle_t g_pid_mutex;

/// @brief Mutex protecting g_velocity_pid array
/// Shared between: Velocity Control task (compute and setpoint updates)
extern SemaphoreHandle_t g_velocity_pid_mutex;

/// @brief Mutex protecting shared ADC operations
/// Shared between: Sensor task (all encoders share one ADC unit)
extern SemaphoreHandle_t g_adc_mutex;

/// @brief Mutex protecting g_robot_estimated
/// Shared between: Sensor task (write), Trajectory task (read)
extern SemaphoreHandle_t g_estimated_data_mutex;

// =============================================================================
// TASK HANDLES
// =============================================================================

/// @brief Handle for sensor reading task
extern TaskHandle_t g_task_sensor_handle;

/// @brief Handle for motor control task
extern TaskHandle_t g_task_control_handle;

/// @brief Handle for inverse kinematics task
extern TaskHandle_t g_task_ik_handle;

/// @brief Handle for velocity control task (outer PID loop)
extern TaskHandle_t g_task_velocity_control_handle;

/// @brief Handle for trajectory generation task
extern TaskHandle_t g_task_trajectory_handle;

/// @brief Handle for UART parser task (used for parameter tuning)
extern TaskHandle_t g_handle_parser_task;

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
