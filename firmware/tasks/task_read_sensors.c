#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "bno055.h"      ///< IMU driver
#include "as5600.h"      ///< AS5600 encoder driver
#include "types.h"

#include <stdint.h>
#include <math.h>

/// @brief Sensor reading period in milliseconds
#define SENSOR_TASK_PERIOD_MS 10

/// @brief Angular velocity state structure
typedef struct {
    float last_angle_deg;      ///< Last angle in degrees [0, 360)
    int64_t last_time_us;      ///< Last time in microseconds
} angular_velocity_t;

// External shared sensor data and mutex
extern RawSensorData sensor_data;
extern SemaphoreHandle_t xSensorDataMutex;

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

    while (true) {
        float angle_deg = AS5600_ADC_GetAngle(&as5600_0);
        int64_t now_us  = esp_timer_get_time();
        float omega_rad = compute_angular_velocity(&encoder_state, angle_deg, now_us);

        // Safely store the result
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            sensor_data.encoders[0].omega_rad = omega_rad;
            xSemaphoreGive(xSensorDataMutex);
        }

        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
