/**
 * @file task_read_sensors.c
 * @brief Sensor reading and velocity estimation task
 * 
 * This task reads encoder angles, computes angular velocities, applies
 * Kalman filtering, and estimates robot velocity using forward kinematics.
 * 
 * Features:
 * - High-frequency encoder polling (2 ms period)
 * - Angular velocity calculation with wrap-around handling
 * - Kalman filtering for noise reduction
 * - Forward kinematics for robot velocity estimation
 * 
 * Thread-safety: Accesses shared data through mutexes
 * - g_adc_mutex: Protects ADC read operations
 * - g_sensor_data_mutex: Protects sensor data writes
 * - g_estimated_data_mutex: Protects velocity estimate writes
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "bno055.h"
#include "as5600.h"
#include "config_utils.h"
#include "types_utils.h"
#include "kinematics.h"

#include <stdint.h>
#include <math.h>

// =============================================================================
// LOGGING
// =============================================================================

static const char *TAG = "SENSOR_TASK";

// =============================================================================
// EXTERNAL SHARED DATA
// =============================================================================

extern raw_sensor_data_t g_sensor_data;
extern SemaphoreHandle_t g_sensor_data_mutex;
extern SemaphoreHandle_t g_adc_mutex;
extern SemaphoreHandle_t g_estimated_data_mutex;
extern as5600_t g_as5600[3];
extern velocity_t g_robot_estimated;

// =============================================================================
// INTERNAL DATA STRUCTURES
// =============================================================================

/**
 * @brief Angular velocity computation state for one encoder
 * 
 * Maintains history needed to compute angular velocity from
 * position measurements over time.
 */
typedef struct {
    float last_angle_deg;    ///< Previous angle measurement [0, 360)
    int64_t last_time_us;    ///< Previous timestamp in microseconds
} angular_velocity_state_t;

/**
 * @brief 1D Kalman filter state
 * 
 * Implements a simple Kalman filter for scalar measurements.
 * Used to filter noisy angular velocity estimates.
 */
typedef struct {
    float x;  ///< Estimated value (state)
    float P;  ///< Estimation error covariance
    float Q;  ///< Process noise covariance
    float R;  ///< Measurement noise covariance
} kalman_1d_t;

// =============================================================================
// INTERNAL FUNCTIONS
// =============================================================================

/**
 * @brief Initialize a 1D Kalman filter
 * 
 * @param kf Pointer to Kalman filter structure
 * @param q Process noise covariance
 * @param r Measurement noise covariance
 */
static inline void kalman_init(kalman_1d_t *kf, float q, float r) 
{
    kf->x = 0.0f;
    kf->P = 1.0f;
    kf->Q = q;
    kf->R = r;
}

/**
 * @brief Update Kalman filter with new measurement
 * 
 * Performs prediction and correction steps:
 * 1. Prediction: P = P + Q
 * 2. Kalman gain: K = P / (P + R)
 * 3. Correction: x = x + K*(measurement - x)
 * 4. Covariance update: P = P*(1 - K)
 * 
 * @param kf Pointer to Kalman filter structure
 * @param measurement New measurement value
 * @return Updated estimated value
 */
static inline float kalman_update(kalman_1d_t *kf, float measurement) 
{
    // Prediction step
    kf->P += kf->Q;
    
    // Compute Kalman gain
    float K = kf->P / (kf->P + kf->R);
    
    // Correction step
    kf->x += K * (measurement - kf->x);
    kf->P *= (1.0f - K);
    
    return kf->x;
}

/**
 * @brief Compute angular velocity from position measurements
 * 
 * Calculates angular velocity using finite difference approximation:
 * ω = Δθ / Δt
 * 
 * Handles angle wrap-around at 0°/360° boundary.
 * 
 * @param state Pointer to angular velocity state (maintains history)
 * @param angle_deg Current angle in degrees [0, 360)
 * @param time_us Current timestamp in microseconds
 * @return Angular velocity in radians per second
 */
static inline float compute_angular_velocity(angular_velocity_state_t *state, 
                                             float angle_deg, 
                                             int64_t time_us)
{
    // Compute angle change
    float delta_deg = angle_deg - state->last_angle_deg;

    // Handle wrap-around (crossing 0°/360° boundary)
    if (delta_deg > 180.0f) {
        delta_deg -= 360.0f;  // Forward wrap
    } else if (delta_deg < -180.0f) {
        delta_deg += 360.0f;  // Backward wrap
    }

    // Compute time difference
    int64_t delta_time_us = time_us - state->last_time_us;
    if (delta_time_us <= 0) {
        return 0.0f;  // No time elapsed, return zero velocity
    }

    // Convert to angular velocity in rad/s
    float deg_per_sec = delta_deg * (1e6f / (float)delta_time_us);
    float rad_per_sec = deg_per_sec * (M_PI / 180.0f);

    // Update state for next iteration
    state->last_angle_deg = angle_deg;
    state->last_time_us = time_us;

    return rad_per_sec;
}

// =============================================================================
// TASK IMPLEMENTATION
// =============================================================================

/**
 * @brief Main sensor reading task
 * 
 * This task runs at high frequency (2 ms period) to:
 * 1. Read encoder angles via ADC
 * 2. Compute angular velocities with wrap-around handling
 * 3. Apply Kalman filtering for noise reduction
 * 4. Estimate robot velocity using forward kinematics
 * 5. Update shared data structures (thread-safe with timeouts)
 * 
 * All shared resource accesses use timeouts to prevent deadlocks
 * and provide diagnostic warnings if access is delayed.
 * 
 * @param pvParameters Unused task parameter
 */
void task_read_sensors(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor reading task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();

    // =================================================================
    // INITIALIZE STATE VARIABLES
    // =================================================================
    
    angular_velocity_state_t encoder_state[3];
    kalman_1d_t kalman_filters[3];
    int64_t now_us = esp_timer_get_time();
    
    // Initialize angular velocity computation state for each encoder
    for (int i = 0; i < 3; i++) {
        encoder_state[i].last_angle_deg = as5600_adc_get_angle(&g_as5600[i]);
        encoder_state[i].last_time_us = now_us;
        kalman_init(&kalman_filters[i], SENSOR_KALMAN_Q, SENSOR_KALMAN_R);
    }

    // Local working variables
    float filtered_omega_rad[3] = {0.0f};  // Filtered angular velocities
    float angle_deg[3] = {0.0f};            // Current encoder angles
    float omega_rad[3] = {0.0f};            // Raw angular velocities
    wheel_speeds_t wheel_speeds_estimated = {0};  // Estimated wheel speeds
    velocity_t speed_estimated = {0};       // Estimated robot velocity

    ESP_LOGI(TAG, "Entering main sensor loop");

    // =================================================================
    // MAIN TASK LOOP
    // =================================================================
    
    while (1) {
        // -------------------------------------------------------------
        // 1. READ ENCODER ANGLES (Thread-safe ADC access with timeout)
        // -------------------------------------------------------------
        
        if (xSemaphoreTake(g_adc_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                angle_deg[i] = as5600_adc_get_angle(&g_as5600[i]);
            }
            xSemaphoreGive(g_adc_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire ADC mutex (timeout)");
            // Continue with previous angle values
        }
        
        now_us = esp_timer_get_time();

        // -------------------------------------------------------------
        // 2. COMPUTE AND FILTER ANGULAR VELOCITIES
        // -------------------------------------------------------------
        
        for (int i = 0; i < 3; i++) {
            // Compute raw angular velocity
            omega_rad[i] = compute_angular_velocity(&encoder_state[i], 
                                                   angle_deg[i], 
                                                   now_us);
            
            // Apply Kalman filter and direction correction
            filtered_omega_rad[i] = SENSOR_ANGULAR_DIRECTION_FORWARD(i) * 
                                   kalman_update(&kalman_filters[i], omega_rad[i]);
        }

        // -------------------------------------------------------------
        // 3. ESTIMATE ROBOT VELOCITY (Forward Kinematics)
        // -------------------------------------------------------------
        
        // Map encoder readings to wheel speeds (handle wheel indexing)
        wheel_speeds_estimated.phi_dot[0] = filtered_omega_rad[0];  // Wheel 1
        wheel_speeds_estimated.phi_dot[1] = filtered_omega_rad[2];  // Wheel 2
        wheel_speeds_estimated.phi_dot[2] = filtered_omega_rad[1];  // Wheel 3
        
        // Compute robot velocity from wheel speeds
        compute_forward_kinematics(wheel_speeds_estimated, &speed_estimated);

        // -------------------------------------------------------------
        // 4. UPDATE SHARED DATA (Thread-safe with timeout)
        // -------------------------------------------------------------
        
        // Update estimated robot velocity
        if (xSemaphoreTake(g_estimated_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_robot_estimated.vx = speed_estimated.vx;
            g_robot_estimated.vy = speed_estimated.vy;
            g_robot_estimated.wz = speed_estimated.wz;
            xSemaphoreGive(g_estimated_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire estimated_data mutex (timeout)");
        }

        // Update sensor data (angles and velocities)
        if (xSemaphoreTake(g_sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int i = 0; i < 3; i++) {
                g_sensor_data.encoders[i].angle_deg = angle_deg[i];
                g_sensor_data.encoders[i].omega_rad = filtered_omega_rad[i];
            }
            xSemaphoreGive(g_sensor_data_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to acquire sensor_data mutex (timeout)");
        }

        // -------------------------------------------------------------
        // 5. OPTIONAL LOGGING (Disabled for performance)
        // -------------------------------------------------------------
        
        // Uncomment for debugging:
        // ESP_LOGI(TAG, "Angles: %.2f %.2f %.2f | Omega: %.2f %.2f %.2f",
        //          angle_deg[0], angle_deg[1], angle_deg[2],
        //          filtered_omega_rad[0], filtered_omega_rad[1], filtered_omega_rad[2]);

        // -------------------------------------------------------------
        // 6. WAIT FOR NEXT CYCLE
        // -------------------------------------------------------------
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
