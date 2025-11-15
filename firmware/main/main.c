/**
 * @file main.c
 * @brief Main application entry point for RoboCup Goalkeeper firmware
 * 
 * This file contains the application entry point and global variable definitions.
 * It coordinates initialization of hardware and creation of FreeRTOS tasks for:
 * - Sensor reading and filtering
 * - Motor control (PID loops)
 * - Inverse kinematics calculation
 * - Trajectory generation
 * 
 * Architecture:
 * - Uses FreeRTOS tasks for concurrent operation
 * - Inter-task communication via mutexes protecting shared data structures
 * - Task priorities configured to ensure real-time responsiveness
 * 
 * All identifiers follow snake_case naming convention.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <stdio.h>
#include <math.h>

#include "init.h"
#include "pid.h"
#include "motor.h"
#include "as5600.h"
#include "bno055.h"
#include "types_utils.h"
#include "config_utils.h"

// =============================================================================
// LOGGING TAG
// =============================================================================

static const char *TAG = "MAIN";

// =============================================================================
// HARDWARE INSTANCE DEFINITIONS
// =============================================================================

/// @brief Array of three brushless motor controllers
motor_brushless_t g_motor[3];

/// @brief Array of three AS5600 magnetic encoders
AS5600_t g_as5600[3];

/// @brief BNO055 IMU sensor instance (currently unused)
BNO055_t g_bno055;

/// @brief Shared ADC handle for encoder analog reading
adc_oneshot_unit_handle_t g_shared_adc_handle;

/// @brief Array of three PID controller handles
pid_block_handle_t g_pid[3];

/// @brief PID controller tuning parameters (shared by all motors)
pid_parameter_t g_pid_param = {
    .kp = PID_MOTOR_KP,
    .ki = PID_MOTOR_KI,
    .kd = PID_MOTOR_KD,
    .max_output = PID_MOTOR_MAX_OUTPUT,
    .min_output = PID_MOTOR_MIN_OUTPUT,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL,
    .beta = PID_MOTOR_BETA
};

// =============================================================================
// SHARED DATA STRUCTURES
// =============================================================================

/// @brief Raw sensor readings from all encoders
/// Protected by: g_sensor_data_mutex
/// Writers: Sensor task
/// Readers: Control task
raw_sensor_data_t g_sensor_data = {0};

/// @brief Estimated robot velocity (from forward kinematics)
/// Protected by: g_estimated_data_mutex
/// Writers: Sensor task
/// Readers: Trajectory task (for logging)
velocity_t g_robot_estimated = {0};

// =============================================================================
// SYNCHRONIZATION PRIMITIVES
// =============================================================================

/// @brief Queue for velocity commands (Trajectory → IK task)
/// Item type: velocity_t
/// Size: 2 items (only latest command matters)
/// Rationale: One-way data flow, producer-consumer pattern
QueueHandle_t g_velocity_command_queue = NULL;

/// @brief Queue for wheel speed targets (IK → Control task)
/// Item type: wheel_speeds_t
/// Size: 2 items (only latest targets matter)
/// Rationale: Decouples IK from control, allows buffering
QueueHandle_t g_wheel_target_queue = NULL;

/// @brief Mutex protecting g_sensor_data
/// Shared between: Sensor task (write), Control task (read)
SemaphoreHandle_t g_sensor_data_mutex = NULL;

/// @brief Mutex protecting g_pid array
/// Shared between: IK task (setpoint updates), Control task (compute)
SemaphoreHandle_t g_pid_mutex = NULL;

/// @brief Mutex protecting shared ADC operations
/// Shared between: Sensor task (all encoders share one ADC unit)
SemaphoreHandle_t g_adc_mutex = NULL;

/// @brief Mutex protecting g_robot_estimated
/// Shared between: Sensor task (write), Trajectory task (read)
SemaphoreHandle_t g_estimated_data_mutex = NULL;

// =============================================================================
// TASK HANDLES
// =============================================================================

/// @brief Handle for sensor reading task
TaskHandle_t g_task_sensor_handle = NULL;

/// @brief Handle for motor control task
TaskHandle_t g_task_control_handle = NULL;

/// @brief Handle for inverse kinematics task
TaskHandle_t g_task_ik_handle = NULL;

/// @brief Handle for trajectory generation task
TaskHandle_t g_task_trajectory_handle = NULL;

/// @brief Handle for UART parser task (used for parameter tuning)
TaskHandle_t g_handle_parser_task = NULL;

// =============================================================================
// TASK FORWARD DECLARATIONS
// =============================================================================

void task_read_sensors(void *pvParameters);
void task_control(void *pvParameters);
void task_inverse_kinematics(void *pvParameters);
void task_move_trajectory(void *pvParameters);

// Optional UART tasks for runtime PID tuning (currently disabled)
// void task_uart_handler(void *arg);
// void task_uart_parser(void *arg);

// =============================================================================
// CIRCULAR TRAJECTORY PARAMETERS
// =============================================================================

#define CIRCULAR_RADIUS 0.5f       ///< Radius of circular trajectory (m)
#define OMEGA_CIRC 0.5f            ///< Angular velocity for circular motion (rad/s)
#define DT_SECONDS 0.02f           ///< Control loop period (s)
#define TASK_PERIOD_MS ((int)(DT_SECONDS * 1000))  ///< Period in milliseconds

// =============================================================================
// TRAJECTORY GENERATION TASK
// =============================================================================

/**
 * @brief Task that generates a circular trajectory for robot motion
 * 
 * This task computes desired velocities (vx, vy, wz) to make the robot
 * follow a circular path. It sends velocity commands via queue to the
 * inverse kinematics task.
 * 
 * Task characteristics:
 * - Period: TASK_PERIOD_MS (20 ms)
 * - Priority: 4 (medium)
 * - Stack: 2048 bytes
 * 
 * Communication:
 * - Sends velocity commands to IK task via g_velocity_command_queue
 * - Reads estimated velocity for logging via g_estimated_data_mutex
 * 
 * @param arg Unused task parameter
 */
void task_move_trajectory(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    float t = 0.0f;                      // Time accumulator
    float prev_angle = 0.0f;             // Previous trajectory angle
    velocity_t speed_estimated = {0};    // Local copy of estimated velocity
    velocity_t velocity_cmd = {0};       // Command to send to IK task

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
        // SEND VELOCITY COMMAND TO IK TASK (Via Queue)
        // =================================================================
        
        velocity_cmd.vx = vx_cmd;
        velocity_cmd.vy = vy_cmd;
        velocity_cmd.wz = omega_cmd;
        
        // Send to queue with short timeout (overwrite if queue is full)
        if (xQueueSend(g_velocity_command_queue, &velocity_cmd, pdMS_TO_TICKS(5)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send velocity command (queue full)");
        }

        // =================================================================
        // READ ESTIMATED VELOCITY FOR LOGGING (Thread-safe)
        // =================================================================
        
        if (g_estimated_data_mutex && 
            xSemaphoreTake(g_estimated_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            speed_estimated = g_robot_estimated;
            xSemaphoreGive(g_estimated_data_mutex);
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

// =============================================================================
// APPLICATION MAIN ENTRY POINT
// =============================================================================

/**
 * @brief Main application entry point
 * 
 * Initializes hardware, creates synchronization primitives, and spawns
 * all application tasks.
 * 
 * Initialization sequence:
 * 1. Initialize motors (PWM setup)
 * 2. Initialize sensors (encoders and ADC)
 * 3. Initialize PID controllers
 * 4. Wait for sensor stabilization
 * 5. Create mutexes for thread-safe data sharing
 * 6. Create FreeRTOS tasks with appropriate priorities
 * 
 * Task priority hierarchy (higher number = higher priority):
 * - Priority 6: Sensor reading (most critical for real-time feedback)
 * - Priority 5: Inverse kinematics (must run before control)
 * - Priority 4: Trajectory generation
 * - Priority 3: Motor control (PID loops)
 */
void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  RoboCup Goalkeeper Firmware");
    ESP_LOGI(TAG, "  Version: %s", ROBOT_VERSION);
    ESP_LOGI(TAG, "==============================================");

    // =================================================================
    // HARDWARE INITIALIZATION
    // =================================================================
    
    ESP_LOGI(TAG, "Starting hardware initialization...");
    
    int ret = init_motors();
    if (ret != INIT_SUCCESS) {
        ESP_LOGE(TAG, "Motor initialization failed with code %d", ret);
        return;
    }
    
    ret = init_sensors();
    if (ret != INIT_SUCCESS) {
        ESP_LOGE(TAG, "Sensor initialization failed with code %d", ret);
        return;
    }
    
    ret = init_pid();
    if (ret != INIT_SUCCESS) {
        ESP_LOGE(TAG, "PID initialization failed with code %d", ret);
        return;
    }
    
    ESP_LOGI(TAG, "Hardware initialization complete");
    
    // Optional: Motor calibration sequence
    // motor_calibration3(&g_motor[0], &g_motor[1], &g_motor[2]);
    
    // Wait for sensor stabilization
    ESP_LOGI(TAG, "Waiting for sensor stabilization...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // =================================================================
    // CREATE SYNCHRONIZATION PRIMITIVES
    // =================================================================
    
    ESP_LOGI(TAG, "Creating synchronization primitives...");
    
    // Create mutexes for shared resources
    g_sensor_data_mutex = xSemaphoreCreateMutex();
    g_pid_mutex = xSemaphoreCreateMutex();
    g_adc_mutex = xSemaphoreCreateMutex();
    g_estimated_data_mutex = xSemaphoreCreateMutex();
    
    // Create queues for one-way data flow
    g_velocity_command_queue = xQueueCreate(2, sizeof(velocity_t));
    g_wheel_target_queue = xQueueCreate(2, sizeof(wheel_speeds_t));
    
    // Validate all primitives were created successfully
    if (!g_sensor_data_mutex || !g_pid_mutex || !g_adc_mutex || 
        !g_estimated_data_mutex || !g_velocity_command_queue || 
        !g_wheel_target_queue) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        ESP_LOGE(TAG, "  sensor_data_mutex: %p", g_sensor_data_mutex);
        ESP_LOGE(TAG, "  pid_mutex: %p", g_pid_mutex);
        ESP_LOGE(TAG, "  adc_mutex: %p", g_adc_mutex);
        ESP_LOGE(TAG, "  estimated_data_mutex: %p", g_estimated_data_mutex);
        ESP_LOGE(TAG, "  velocity_command_queue: %p", g_velocity_command_queue);
        ESP_LOGE(TAG, "  wheel_target_queue: %p", g_wheel_target_queue);
        return;
    }
    
    ESP_LOGI(TAG, "All synchronization primitives created successfully");

    // =================================================================
    // CREATE FREERTOS TASKS
    // =================================================================
    
    ESP_LOGI(TAG, "Creating application tasks...");
    
    BaseType_t xReturned;
    
    // High-priority sensor reading task (6)
    xReturned = xTaskCreate(
        task_read_sensors,
        "SensorTask",
        4096,
        NULL,
        6,
        &g_task_sensor_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ Sensor task created (priority 6)");
    
    // Inverse kinematics task (5) - must run before control
    xReturned = xTaskCreate(
        task_inverse_kinematics,
        "IK_Task",
        4096,
        NULL,
        5,
        &g_task_ik_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IK task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ IK task created (priority 5)");
    
    // Trajectory generation task (4)
    xReturned = xTaskCreate(
        task_move_trajectory,
        "MoveTask",
        2048,
        NULL,
        4,
        &g_task_trajectory_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create move task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ Trajectory task created (priority 4)");
    
    // Motor control task (3)
    xReturned = xTaskCreate(
        task_control,
        "ControlTask",
        4096,
        NULL,
        3,
        &g_task_control_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ Control task created (priority 3)");
    
    // Optional UART tuning tasks (currently disabled)
    // xTaskCreate(task_uart_handler, "uart_handler", 4096, NULL, 10, NULL);
    // xTaskCreate(task_uart_parser, "uart_parser", 4096, NULL, 10, &g_handle_parser_task);

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Task Communication Architecture:");
    ESP_LOGI(TAG, "  Trajectory → IK: Queue (velocity commands)");
    ESP_LOGI(TAG, "  IK → Control: Queue (wheel targets)");
    ESP_LOGI(TAG, "  Sensor → Control: Mutex (sensor data)");
    ESP_LOGI(TAG, "  Sensor → Trajectory: Mutex (estimates)");
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "System running...");
    
    // Optional: Set constant motor speeds for testing
    // motor_set_speed(&g_motor[0], MOTOR_DIRECTION_FORWARD(0) * 20.0f);
    // motor_set_speed(&g_motor[1], MOTOR_DIRECTION_FORWARD(1) * 20.0f);
    // motor_set_speed(&g_motor[2], MOTOR_DIRECTION_FORWARD(2) * 20.0f);
}
