/**
 * @file main.c
 * @brief Main application entry point for RoboCup Goalkeeper firmware
 * 
 * This file contains:
 * - Application entry point (app_main)
 * - Global variable definitions (all system-wide resources)
 * - Hardware initialization coordination
 * - FreeRTOS task creation and startup
 * 
 * Architecture:
 * - Uses FreeRTOS tasks for concurrent operation
 * - Inter-task communication via queues and mutexes
 * - Task priorities configured to ensure real-time responsiveness
 * 
 * All identifiers follow snake_case naming convention.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "main.h"
#include "init.h"
#include "config_utils.h"

// =============================================================================
// LOGGING TAG
// =============================================================================

static const char *TAG = "MAIN";

// =============================================================================
// HARDWARE INSTANCE DEFINITIONS
// =============================================================================

motor_brushless_t g_motor[3];
AS5600_t g_as5600[3];
BNO055_t g_bno055;
adc_oneshot_unit_handle_t g_shared_adc_handle;

// =============================================================================
// PID CONTROLLER DEFINITIONS
// =============================================================================

pid_block_handle_t g_pid[3];
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

pid_block_handle_t g_velocity_pid[3];
pid_parameter_t g_velocity_pid_param = {
    .kp = PID_VELOCITY_KP,
    .ki = PID_VELOCITY_KI,
    .kd = PID_VELOCITY_KD,
    .max_output = PID_VELOCITY_MAX_OUTPUT,
    .min_output = PID_VELOCITY_MIN_OUTPUT,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL,
    .beta = PID_VELOCITY_BETA
};

// =============================================================================
// SHARED DATA STRUCTURES
// =============================================================================

raw_sensor_data_t g_sensor_data = {0};
velocity_t g_robot_estimated = {0};

// =============================================================================
// SYNCHRONIZATION PRIMITIVES
// =============================================================================

QueueHandle_t g_desired_velocity_queue = NULL;
QueueHandle_t g_velocity_command_queue = NULL;
QueueHandle_t g_wheel_target_queue = NULL;

SemaphoreHandle_t g_sensor_data_mutex = NULL;
SemaphoreHandle_t g_pid_mutex = NULL;
SemaphoreHandle_t g_velocity_pid_mutex = NULL;
SemaphoreHandle_t g_adc_mutex = NULL;
SemaphoreHandle_t g_estimated_data_mutex = NULL;

// =============================================================================
// TASK HANDLES
// =============================================================================

TaskHandle_t g_task_sensor_handle = NULL;
TaskHandle_t g_task_control_handle = NULL;
TaskHandle_t g_handle_parser_task = NULL;

// =============================================================================
// TASK FORWARD DECLARATIONS
// =============================================================================

void task_read_sensors(void *pvParameters);
void task_motor_control(void *pvParameters);
void task_inverse_kinematics(void *pvParameters);
void task_velocity_control(void *pvParameters);
void task_move_trajectory(void *pvParameters);

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
 * 5. Create mutexes and queues
 * 6. Create FreeRTOS tasks
 * 
 * Task priority hierarchy (higher = more critical):
 * - Priority 6: Sensor reading
 * - Priority 5: Inverse kinematics
 * - Priority 4: Velocity control (outer PID)
 * - Priority 3: Trajectory generation
 * - Priority 2: Motor control (inner PID)
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
    g_velocity_pid_mutex = xSemaphoreCreateMutex();
    g_adc_mutex = xSemaphoreCreateMutex();
    g_estimated_data_mutex = xSemaphoreCreateMutex();
    
    // Create queues for cascaded control flow:
    // Trajectory → Velocity PID → IK → Wheel PID → Motors
    g_desired_velocity_queue = xQueueCreate(2, sizeof(velocity_t));
    g_velocity_command_queue = xQueueCreate(2, sizeof(velocity_t));
    g_wheel_target_queue = xQueueCreate(1, sizeof(wheel_speeds_t));  // Length 1 for xQueueOverwrite
    
    // Validate all primitives were created successfully
    if (!g_sensor_data_mutex || !g_pid_mutex || !g_velocity_pid_mutex ||
        !g_adc_mutex || !g_estimated_data_mutex || 
        !g_desired_velocity_queue || !g_velocity_command_queue || 
        !g_wheel_target_queue) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        ESP_LOGE(TAG, "  sensor_data_mutex: %p", g_sensor_data_mutex);
        ESP_LOGE(TAG, "  pid_mutex: %p", g_pid_mutex);
        ESP_LOGE(TAG, "  velocity_pid_mutex: %p", g_velocity_pid_mutex);
        ESP_LOGE(TAG, "  adc_mutex: %p", g_adc_mutex);
        ESP_LOGE(TAG, "  estimated_data_mutex: %p", g_estimated_data_mutex);
        ESP_LOGE(TAG, "  desired_velocity_queue: %p", g_desired_velocity_queue);
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
    
    // Velocity control task (4) - outer PID loop
    xReturned = xTaskCreate(
        task_velocity_control,
        "VelCtrlTask",
        4096,
        NULL,
        4,
        &g_task_velocity_control_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create velocity control task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ Velocity control task created (priority 4)");
    
    // Trajectory generation task (3)
    xReturned = xTaskCreate(
        task_move_trajectory,
        "MoveTask",
        4096,  // Increased from 2048 to prevent stack overflow
        NULL,
        3,
        &g_task_trajectory_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create move task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ Trajectory task created (priority 3)");
    
    // Motor control task (2) - inner PID loop
    xReturned = xTaskCreate(
        task_motor_control,
        "MotorCtrlTask",
        4096,
        NULL,
        2,
        &g_task_control_handle
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor control task");
        return;
    }
    ESP_LOGI(TAG, "  ✓ Motor control task created (priority 2)");
    
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Task Communication Architecture:");
    ESP_LOGI(TAG, "  Trajectory → Vel Control: Queue (desired velocity)");
    ESP_LOGI(TAG, "  Vel Control → IK: Queue (corrected velocity)");
    ESP_LOGI(TAG, "  IK → Control: Queue (wheel targets)");
    ESP_LOGI(TAG, "  Sensor ↔ Tasks: Mutexes (shared data)");
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "System running...");
}
