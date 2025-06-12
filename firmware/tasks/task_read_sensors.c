#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "bno055.h"      ///< IMU driver
#include "as5600.h"      ///< AS5600 encoder driver
#include "config_utils.h" ///< Configuration utilities
#include "types.h"

#include <stdint.h>
#include <math.h>

/// @brief Angular velocity state structure
typedef struct {
    float last_angle_deg;      ///< Last angle in degrees [0, 360)
    int64_t last_time_us;      ///< Last time in microseconds
} angular_velocity_t;

// External shared sensor data and mutex
extern RawSensorData sensor_data;
extern SemaphoreHandle_t xSensorDataMutex;
extern SemaphoreHandle_t xADCMutex; // Mutex for ADC operations

// External AS5600 sensor instance
extern AS5600_t as5600_0;

/// Optional handle to manage task externally
TaskHandle_t xTaskReadSensorsHandle = NULL;

/**
 * @brief Computes angular velocity in rad/s based on new angle and timestamp.
 * 
 * @param sensor Pointer to angular velocity state
 * @param angle_deg Current angle in degrees [0, 360)
 * @param time_us Current time in microseconds
 * @return Angular velocity in radians per second
 */
static inline float compute_angular_velocity(angular_velocity_t *sensor, float angle_deg, int64_t time_us)
{
    float delta_deg = angle_deg - sensor->last_angle_deg;

    // Handle wrap-around (360 -> 0 or 0 -> 360)
    if (delta_deg > 180.0f) delta_deg -= 360.0f;
    else if (delta_deg < -180.0f) delta_deg += 360.0f;

    int64_t delta_time_us = time_us - sensor->last_time_us;
    if (delta_time_us <= 0) return 0.0f;

    float deg_per_sec = delta_deg * (1e6f / (float)delta_time_us);
    float rad_per_sec = deg_per_sec * (M_PI / 180.0f);

    sensor->last_angle_deg = angle_deg;
    sensor->last_time_us = time_us;

    return rad_per_sec;
}

/**
 * @brief Task that reads encoder and computes angular velocity periodically.
 * 
 * @param pvParameters Unused
 */
void vTaskReadSensors(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Initialize state
    angular_velocity_t encoder_state = {
        .last_angle_deg = AS5600_ADC_GetAngle(&as5600_0),
        .last_time_us   = esp_timer_get_time()
    };

    float beta = exp(-2 * M_PI * SENSOR_CUTOFF_FREQUENCY_OMEGA_HZ / SENSOR_TASK_SAMPLE_RATE_HZ);  // 10Hz cutoff frequency
    float filtered_omega_rad = 0.0f; // Filtered angular velocity

    float angle_deg = 0.0f; // Current angle in degrees
    int64_t now_us = 0; // Current time in microseconds
    float omega_rad = 0.0f; // Current angular velocity in rad/s

    // uint32_t timestamp_us = 1000000; // 1 second in microseconds

    while (true) {
        //Take mutex to read the encoder angle
        if (xSemaphoreTake(xADCMutex, portMAX_DELAY) == pdTRUE) {
            // Read the angle from the AS5600 sensor
            angle_deg = AS5600_ADC_GetAngle(&as5600_0);
            xSemaphoreGive(xADCMutex);
        }
        // angle_deg = AS5600_ADC_GetAngle(&as5600_0);
        now_us  = esp_timer_get_time();
        omega_rad = compute_angular_velocity(&encoder_state, angle_deg, now_us);

        // Apply low-pass filter to smooth the angle Vn = beta * Vn-1 + (1 - beta) * Vn
        filtered_omega_rad = beta * filtered_omega_rad + (1.0f - beta) * omega_rad;

        // Safely store the result
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            sensor_data.encoders[0].angle_deg = angle_deg;
            sensor_data.encoders[0].omega_rad = filtered_omega_rad;
            xSemaphoreGive(xSensorDataMutex);
        }
        

        // Print the result for debugging
        // printf("Encoder 0: Angle=%.2f deg, Omega=%.4f rad/s\n", angle_deg, filtered_omega_rad);
        // printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp_us, angle_deg, 0.0, 0.0, omega_rad, filtered_omega_rad, 0.0);
        // timestamp_us += SENSOR_TASK_PERIOD_MS * 1000; // Increment timestamp by task period in microseconds

        // Wait for the next cycle
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
