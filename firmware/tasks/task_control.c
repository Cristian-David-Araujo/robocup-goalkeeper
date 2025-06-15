#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#include "esp_timer.h"
#include "types.h"
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

extern pid_block_handle_t pid;
extern pid_parameter_t pid_param;
extern motor_brushless_t motor_0;


void vTaskControl(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();

    EncoderReading encoder;

    float out_pid_motor_0;
    uint32_t timestamp_us = 1000000; // 1 second in microseconds

    while (1) {
        // Try to take the mutex
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            // Safely copy the data you need
            encoder = sensor_data.encoders[0];  // Index depends on your setup

            // Release the mutex
            xSemaphoreGive(xSensorDataMutex);
        }

        // pid_compute(pid, encoder.omega_rad, &out_pid_motor_0);
        if (xSemaphoreTake(xPidMutex, portMAX_DELAY) == pdTRUE) {
            pid_compute(pid, encoder.omega_rad, &out_pid_motor_0);
            xSemaphoreGive(xPidMutex);
        }

        motor_set_speed(&motor_0, out_pid_motor_0);

        printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp_us, encoder.omega_rad, pid_param.set_point, 0.0, out_pid_motor_0, 0.0, 0.0);
        timestamp_us += CONTROL_TASK_PERIOD_MS * 1000; // Increment timestamp by task period in microseconds

        // Wait before next check (optional)
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}


void vTaskUart(void* arg) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_flush(UART_NUM_0);

    char data[128];
    char parsed[128];
    float kp, ki, kd, setpoint;

    while (1) {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)data, sizeof(data) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';

            // Look for the string between quotes after "default":
            char *start = strstr(data, "\"default\":\"");
            if (start) {
                start += strlen("\"default\":\"");
                char *end = strchr(start, '"');
                if (end && (end - start) < sizeof(parsed)) {
                    strncpy(parsed, start, end - start);
                    parsed[end - start] = '\0';

                    if (sscanf(parsed, "%f %f %f %f", &kp, &ki, &kd, &setpoint) == 4) {
                        if (xSemaphoreTake(xPidMutex, portMAX_DELAY) == pdTRUE) {
                            pid_param.kp = kp/ 100.0f; // Scale down the kp value
                            pid_param.ki = ki / 100.0f; // Scale down the ki value
                            pid_param.kd = kd / 100.0f; // Scale down the kd value
                            pid_param.set_point = setpoint;

                            pid_update_parameters(pid, &pid_param);
                            xSemaphoreGive(xPidMutex);

                            printf("Updated PID: kp=%.2f, ki=%.2f, kd=%.2f, setpoint=%.2f\n", kp, ki, kd, setpoint);
                        } else {
                            printf("PID mutex busy. Parameters not updated.\n");
                        }
                    } else {
                        printf("Invalid PID format inside 'default'.\n");
                    }
                } else {
                    printf("Invalid or too long 'default' value.\n");
                }
            } else {
                printf("Invalid format. Expected: {\"default\":\"kp ki kd setpoint\"}\n");
            }
        }
    }
}
