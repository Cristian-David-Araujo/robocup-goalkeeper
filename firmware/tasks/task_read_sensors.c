#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "bno055.h"      ///< IMU driver
#include "as5600.h"      ///< AS5600 encoder driver
#include "config_utils.h" ///< Configuration utilities
#include "types_utils.h"

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
extern AS5600_t as5600[3]; ///< Array of AS5600 sensors
/// Optional handle to manage task externally
TaskHandle_t xTaskReadSensorsHandle = NULL;

/// @brief Kalman filter state for 1D estimation
/// This structure holds the estimated value, estimation error covariance, process noise, and measurement noise.
typedef struct {
    float x;  // estimated value
    float P;  // estimation error covariance
    float Q;  // process noise
    float R;  // measurement noise
} Kalman1D;

/**
 * @brief Initializes a Kalman filter for 1D estimation.
 * 
 * @param kf Pointer to the Kalman1D structure
 * @param q Process noise covariance
 * @param r Measurement noise covariance
 */
static inline void kalman_init(Kalman1D *kf, float q, float r) {
    kf->x = 0.0f;
    kf->P = 1.0f;
    kf->Q = q;
    kf->R = r;
}

/**
 * @brief Updates the Kalman filter with a new measurement.
 * 
 * This function performs the prediction and update steps of the Kalman filter.
 * It returns the updated estimated value.
 * 
 * @param kf Pointer to the Kalman1D structure
 * @param measurement New measurement value
 * @return Updated estimated value
 */
static inline float kalman_update(Kalman1D *kf, float measurement) {
    kf->P += kf->Q;
    float K = kf->P / (kf->P + kf->R);
    kf->x += K * (measurement - kf->x);
    kf->P *= (1.0f - K);
    return kf->x;
}




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
/**
 * @brief Task that reads encoder and computes angular velocity periodically.
 * 
 * @param pvParameters Unused
 */
void vTaskReadSensors(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Initialize state
    angular_velocity_t encoder_state[3];
    int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < 3; i++) {
        encoder_state[i].last_angle_deg = AS5600_ADC_GetAngle(&as5600[i]);
        encoder_state[i].last_time_us = now_us;
    }

    // float beta = exp(-2 * M_PI * SENSOR_CUTOFF_FREQUENCY_OMEGA_HZ / SENSOR_TASK_SAMPLE_RATE_HZ);
    float filtered_omega_rad[3] = {0.0f, 0.0f, 0.0f}; // Filtered angular velocities for each encoder
    float angle_deg[3] = {0.0f, 0.0f, 0.0f}; // Current angles in degrees for each encoder
    float omega_rad[3] = {0.0f, 0.0f, 0.0f}; // Angular velocities in rad/s for each encoder
    // Kalman filters for each encoder
    Kalman1D kalman_filters[3];
    for (int i = 0; i < 3; i++) {
        kalman_init(&kalman_filters[i], SENSOR_KALMAN_Q, SENSOR_KALMAN_R); // Initialize Kalman filter for each encoder
    }

    // uint32_t timestamp_us = 1000000; // 1 second in microseconds
    // int print_counter = 0;

    while (true) {
        //Take mutex to read the encoder angle
        if (xSemaphoreTake(xADCMutex, portMAX_DELAY) == pdTRUE) {
            // Read the angle from the AS5600 sensor
            for (int i = 0; i < 3; i++) {
                angle_deg[i] = AS5600_ADC_GetAngle(&as5600[i]);
            }
            // Release the mutex after reading
            xSemaphoreGive(xADCMutex);
        }
        // angle_deg = AS5600_ADC_GetAngle(&as5600_0);
        now_us  = esp_timer_get_time(); ///< Get current time in microseconds

        // Compute angular velocity and apply filter
        for (int i = 0; i < 3; i++) {
            // Compute angular velocity in rad/s
            omega_rad[i] = compute_angular_velocity(&encoder_state[i], angle_deg[i], now_us);
            // Apply low-pass filter to smooth the angle Vn = beta * Vn-1 + (1 - beta) * Vn
            // filtered_omega_rad[i] = beta * filtered_omega_rad[i] + (1.0f - beta) * omega_rad[i];
            filtered_omega_rad[i] = kalman_update(&kalman_filters[i], omega_rad[i]);
        }
        

        // Safely store the result
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                sensor_data.encoders[i].angle_deg = angle_deg[i];
                sensor_data.encoders[i].omega_rad = SENSOR_ANGULAR_DIRECTION_FORWARD(i) * filtered_omega_rad[i]; // Forward direction
            }
            xSemaphoreGive(xSensorDataMutex);
        }
        

        // Print the result for debugging
        // if (++print_counter >= 10) {
        //     printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp_us, angle_deg[0], angle_deg[1], angle_deg[2], SENSOR_ANGULAR_DIRECTION_FORWARD(0)*filtered_omega_rad[0], SENSOR_ANGULAR_DIRECTION_FORWARD(1)*filtered_omega_rad[1], SENSOR_ANGULAR_DIRECTION_FORWARD(2)*filtered_omega_rad[2]);
        //     print_counter = 0;
        // }
        // timestamp_us += SENSOR_TASK_PERIOD_MS * 1000; // Increment timestamp by task period in microseconds

        // Wait for the next cycle
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
