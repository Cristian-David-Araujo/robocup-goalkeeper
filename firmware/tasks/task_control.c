#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#include "esp_timer.h"
#include "types_utils.h"
#include "motor.h"
#include "pid.h"
#include "config_utils.h" ///< Configuration utilities

#include <stdint.h>
#include <string.h>
#include <math.h>


// External shared sensor data and mutex
extern RawSensorData sensor_data;
extern SemaphoreHandle_t xSensorDataMutex;
extern SemaphoreHandle_t xPidMutex;

extern pid_block_handle_t pid[3]; ///< Array of PID controllers
extern pid_parameter_t pid_param; ///< Array of PID parameters for each motor
extern motor_brushless_t motor[3]; ///< Array of brushless motors

#define UART_RX_BUFFER_SIZE 256
#define UART_QUEUE_SIZE     10

static QueueHandle_t uart_queue;
static char uart_buffer[UART_RX_BUFFER_SIZE];
static volatile int uart_buffer_index = 0;
extern TaskHandle_t xHandleParserTask;


void vTaskControl(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();

    EncoderReading encoder[3]; // Array to hold encoder readings

    float out_pid_motor[3] = {0.0f, 0.0f, 0.0f}; // Output for each motor

    uint32_t timestamp_us = 1000000; // 1 second in microseconds

    while (1) {
        // Try to take the mutex
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            // Copy encoder data from shared sensor_data
            for (int i = 0; i < 3; i++) {
                encoder[i] = sensor_data.encoders[i]; // Copy encoder data
            }
            // Release the mutex
            xSemaphoreGive(xSensorDataMutex);
        }

        // pid_compute(pid, encoder.omega_rad, &out_pid_motor_0);
        if (xSemaphoreTake(xPidMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                pid_compute(pid[i], encoder[i].omega_rad, &out_pid_motor[i]);
            }
            xSemaphoreGive(xPidMutex);
        }

        for (int i = 0; i < 3; i++) {
            // Set motor speed based on PID output
            motor_set_speed(&motor[i], MOTOR_DIRECTION_FORWARD(i)*out_pid_motor[i]);
        }

        // // Print the output every 20 ms
        // if ((timestamp_us % 20000) == 0) { // cada 20 ms
        //     printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp_us, encoder[0].omega_rad, encoder[1].omega_rad, encoder[2].omega_rad, out_pid_motor[0], out_pid_motor[1], out_pid_motor[2]);
        // }

        // timestamp_us += CONTROL_TASK_PERIOD_MS * 1000; // Increment timestamp by task period in microseconds

        // Wait before next check (optional)
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

void uart_init_task() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, UART_RX_BUFFER_SIZE * 2, 0, UART_QUEUE_SIZE, &uart_queue, 0);
    uart_flush(UART_NUM_0);
    uart_enable_rx_intr(UART_NUM_0);
}

void vTaskUartHandler(void *arg) {
    uart_event_t event;
    char d;

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                for (int i = 0; i < event.size; ++i) {
                    if (uart_read_bytes(UART_NUM_0, (uint8_t *)&d, 1, pdMS_TO_TICKS(10)) > 0) {
                        if (d == '\n' || d == '\r') {
                            uart_buffer[uart_buffer_index] = '\0';
                            uart_buffer_index = 0;
                            // Notify parser task
                            xTaskNotifyGive(xHandleParserTask);
                        } else if (uart_buffer_index < UART_RX_BUFFER_SIZE - 1) {
                            uart_buffer[uart_buffer_index++] = d;
                        }
                    }
                }
            }
        }
    }
}

void vTaskUartParser(void *arg) {
    float kp, ki, kd, setpoint;
    char parsed[128];

    xHandleParserTask = xTaskGetCurrentTaskHandle();

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char *start = strstr(uart_buffer, "\"default\":\"");
        if (start) {
            start += strlen("\"default\":\"");
            char *end = strchr(start, '"');
            if (end && (end - start) < sizeof(parsed)) {
                strncpy(parsed, start, end - start);
                parsed[end - start] = '\0';

                if (sscanf(parsed, "%f %f %f %f", &kp, &ki, &kd, &setpoint) == 4) {
                    if (xSemaphoreTake(xPidMutex, portMAX_DELAY) == pdTRUE) {
                        pid_param.kp = kp / 100.0f;
                        pid_param.ki = ki / 100.0f;
                        pid_param.kd = kd / 100.0f;
                        pid_param.set_point = setpoint;
                        
                        // Update each PID controller with the new parameters
                        for (int i = 0; i < 3; i++) {
                            pid_update_parameters(pid[i], &pid_param);
                        }

                        xSemaphoreGive(xPidMutex);

                        printf("Updated PID: kp=%.2f, ki=%.2f, kd=%.2f, setpoint=%.2f\n", kp, ki, kd, setpoint);
                    } else {
                        printf("PID mutex busy.\n");
                    }
                } else {
                    printf("Invalid format inside 'default'.\n");
                }
            } else {
                printf("Value too long or malformed.\n");
            }
        } else {
            printf("Expected format: {\"default\":\"kp ki kd setpoint\"}\n");
        }
    }
}