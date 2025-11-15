/**
 * @file task_control.c
 * @brief Motor control task with PID feedback loops
 * 
 * This task implements closed-loop motor speed control using PID controllers.
 * It reads encoder velocities and computes motor commands to track desired
 * wheel speeds received from the inverse kinematics task.
 * 
 * Features:
 * - Independent PID control for each motor
 * - Thread-safe access to sensor data and PID controllers
 * - High-frequency control loop (2 ms period)
 * - Robust timeout handling for all shared resource access
 * 
 * Communication:
 * - Receives wheel targets via g_wheel_target_queue (from IK task)
 * - Reads sensor data via g_sensor_data_mutex (from Sensor task)
 * - Accesses PID controllers via g_pid_mutex
 * 
 * Optional: UART-based runtime PID tuning (currently disabled)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "types_utils.h"
#include "motor.h"
#include "pid.h"
#include "config_utils.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

// =============================================================================
// LOGGING
// =============================================================================

static const char *TAG = "CONTROL_TASK";

// =============================================================================
// EXTERNAL SHARED DATA
// =============================================================================

extern raw_sensor_data_t g_sensor_data;
extern QueueHandle_t g_wheel_target_queue;
extern SemaphoreHandle_t g_sensor_data_mutex;
extern SemaphoreHandle_t g_pid_mutex;
extern pid_block_handle_t g_pid[3];
extern pid_parameter_t g_pid_param;
extern motor_brushless_t g_motor[3];
extern TaskHandle_t g_handle_parser_task;

// =============================================================================
// UART CONFIGURATION (for runtime PID tuning - optional)
// =============================================================================

#define UART_RX_BUFFER_SIZE 256
#define UART_QUEUE_SIZE 10

static QueueHandle_t uart_queue = NULL;
static char uart_buffer[UART_RX_BUFFER_SIZE];
static volatile int uart_buffer_index = 0;

// =============================================================================
// MAIN CONTROL TASK
// =============================================================================

/**
 * @brief Motor control task implementing PID feedback loops
 * 
 * This task runs at high frequency (2 ms) to:
 * 1. Receive target wheel speeds from IK task (via queue)
 * 2. Read current encoder angular velocities
 * 3. Compute PID control outputs for each motor
 * 4. Apply motor commands
 * 
 * The task maintains the last received setpoints if no new data arrives,
 * ensuring smooth control even with occasional queue delays.
 * 
 * @param pvParameters Unused task parameter
 */
void task_control(void *pvParameters) 
{
    ESP_LOGI(TAG, "Motor control task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();

    // Local variables
    encoder_reading_t encoder[3];          // Encoder readings (angle & velocity)
    wheel_speeds_t wheel_targets = {0};    // Target wheel speeds from IK task
    float pid_output[3] = {0.0f};          // PID controller outputs
    bool targets_received = false;         // Flag to track if we have valid targets
    uint32_t no_target_count = 0;          // Counter for missing target warnings

    ESP_LOGI(TAG, "Entering main control loop");

    // =================================================================
    // MAIN TASK LOOP
    // =================================================================
    
    while (1) {
        // -------------------------------------------------------------
        // 1. RECEIVE WHEEL TARGETS FROM QUEUE (Non-blocking)
        // -------------------------------------------------------------
        
        // Try to get latest wheel targets from IK task
        // Use short timeout to avoid blocking the control loop
        if (xQueueReceive(g_wheel_target_queue, &wheel_targets, pdMS_TO_TICKS(1)) == pdTRUE) {
            targets_received = true;
            no_target_count = 0;
        } else {
            // No new target - use previous values
            // This is normal during steady-state operation
            no_target_count++;
            
            // Warn if we haven't received targets for a while (>100ms)
            if (no_target_count == 50 && !targets_received) {
                ESP_LOGW(TAG, "No wheel targets received yet");
            } else if (targets_received && no_target_count % 500 == 0) {
                ESP_LOGW(TAG, "No new wheel targets for %d cycles", no_target_count);
            }
        }

        // -------------------------------------------------------------
        // 2. READ SENSOR DATA (Thread-safe with timeout)
        // -------------------------------------------------------------
        
        if (xSemaphoreTake(g_sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                encoder[i] = g_sensor_data.encoders[i];
            }
            xSemaphoreGive(g_sensor_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire sensor data mutex (timeout)");
            // Continue with previous encoder values
        }

        // -------------------------------------------------------------
        // 3. COMPUTE PID CONTROL OUTPUTS (Thread-safe with timeout)
        // -------------------------------------------------------------
        
        if (xSemaphoreTake(g_pid_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                // PID computes: output = PID(setpoint - measured_velocity)
                // Setpoints were updated by IK task
                pid_compute(g_pid[i], encoder[i].omega_rad, &pid_output[i]);
            }
            xSemaphoreGive(g_pid_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire PID mutex (timeout)");
            // Continue with previous PID outputs (motor commands unchanged)
        }

        // -------------------------------------------------------------
        // 4. APPLY MOTOR COMMANDS
        // -------------------------------------------------------------
        
        for (int i = 0; i < 3; i++) {
            // Apply direction correction and set motor speed
            // motor_set_speed handles saturation and safety limits
            motor_set_speed(&g_motor[i], 
                          MOTOR_DIRECTION_FORWARD(i) * pid_output[i]);
        }

        // -------------------------------------------------------------
        // 5. OPTIONAL LOGGING (Disabled for performance)
        // -------------------------------------------------------------
        
        // Uncomment for debugging:
        // ESP_LOGI(TAG, "Omega: [%.2f, %.2f, %.2f] | PID: [%.2f, %.2f, %.2f]",
        //          encoder[0].omega_rad, encoder[1].omega_rad, encoder[2].omega_rad,
        //          pid_output[0], pid_output[1], pid_output[2]);

        // -------------------------------------------------------------
        // 6. WAIT FOR NEXT CYCLE
        // -------------------------------------------------------------
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

// =============================================================================
// OPTIONAL UART TUNING FUNCTIONS (Currently disabled)
// =============================================================================

/**
 * @brief Initialize UART for PID parameter tuning
 * 
 * Configures UART0 for receiving parameter update commands.
 * Used for runtime tuning without recompiling.
 */
void uart_init_task(void) 
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, UART_RX_BUFFER_SIZE * 2, 0, 
                       UART_QUEUE_SIZE, &uart_queue, 0);
    uart_flush(UART_NUM_0);
    uart_enable_rx_intr(UART_NUM_0);
    
    ESP_LOGI(TAG, "UART initialized for parameter tuning");
}

/**
 * @brief UART event handler task
 * 
 * Receives UART data and buffers complete lines for parsing.
 * Notifies parser task when a complete command is received.
 * 
 * @param arg Unused task parameter
 */
void task_uart_handler(void *arg) 
{
    uart_event_t event;
    char c;

    ESP_LOGI(TAG, "UART handler task started");

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                for (int i = 0; i < event.size; ++i) {
                    if (uart_read_bytes(UART_NUM_0, (uint8_t *)&c, 1, 
                                       pdMS_TO_TICKS(10)) > 0) {
                        if (c == '\n' || c == '\r') {
                            // Complete line received
                            uart_buffer[uart_buffer_index] = '\0';
                            uart_buffer_index = 0;
                            // Notify parser task
                            xTaskNotifyGive(g_handle_parser_task);
                        } else if (uart_buffer_index < UART_RX_BUFFER_SIZE - 1) {
                            uart_buffer[uart_buffer_index++] = c;
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief UART command parser task
 * 
 * Parses received UART commands and updates PID parameters.
 * 
 * Expected format: {"default":"kp ki kd setpoint"}
 * Values are in hundredths (e.g., 10 = 0.1)
 * 
 * @param arg Unused task parameter
 */
void task_uart_parser(void *arg) 
{
    float kp, ki, kd, setpoint;
    char parsed[128];

    g_handle_parser_task = xTaskGetCurrentTaskHandle();
    
    ESP_LOGI(TAG, "UART parser task started");

    while (1) {
        // Wait for notification from handler task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Parse JSON-like format: {"default":"kp ki kd setpoint"}
        char *start = strstr(uart_buffer, "\"default\":\"");
        if (start) {
            start += strlen("\"default\":\"");
            char *end = strchr(start, '"');
            
            if (end && (end - start) < sizeof(parsed)) {
                strncpy(parsed, start, end - start);
                parsed[end - start] = '\0';

                // Parse four float values
                if (sscanf(parsed, "%f %f %f %f", &kp, &ki, &kd, &setpoint) == 4) {
                    // Update PID parameters (thread-safe)
                    if (xSemaphoreTake(g_pid_mutex, portMAX_DELAY) == pdTRUE) {
                        // Convert from hundredths to actual values
                        g_pid_param.kp = kp / 100.0f;
                        g_pid_param.ki = ki / 100.0f;
                        g_pid_param.kd = kd / 100.0f;
                        g_pid_param.set_point = setpoint;
                        
                        // Update all PID controllers
                        for (int i = 0; i < 3; i++) {
                            pid_update_parameters(g_pid[i], &g_pid_param);
                        }

                        xSemaphoreGive(g_pid_mutex);

                        ESP_LOGI(TAG, "PID updated: Kp=%.3f Ki=%.3f Kd=%.3f SP=%.2f",
                                g_pid_param.kp, g_pid_param.ki, 
                                g_pid_param.kd, g_pid_param.set_point);
                    } else {
                        ESP_LOGW(TAG, "Failed to acquire PID mutex");
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid parameter format");
                }
            } else {
                ESP_LOGW(TAG, "Parsed value too long or malformed");
            }
        } else {
            ESP_LOGW(TAG, "Expected format: {\"default\":\"kp ki kd setpoint\"}");
        }
    }
}
