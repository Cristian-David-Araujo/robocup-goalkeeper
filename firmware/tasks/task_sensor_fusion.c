/**
 * @file task_sensor_fusion.c
 * @brief Sensor fusion task for IMU and kinematics integration
 * 
 * This task is responsible for:
 * 1. Reading IMU data from BNO055 at 100 Hz (10 ms period)
 * 2. Obtaining forward kinematics velocity estimates
 * 3. Running the complementary filter fusion algorithm
 * 4. Publishing fused pose data for consumption by other tasks
 * 
 * Task characteristics:
 * - Priority: 5 (high - between sensor reading and inverse kinematics)
 * - Period: 10 ms (100 Hz update rate)
 * - Stack: 4096 bytes
 * 
 * Communication:
 * - Reads IMU data via I2C (BNO055)
 * - Reads kinematics velocity via g_estimated_data_mutex
 * - Publishes fused pose via g_fused_pose_mutex
 * 
 * Thread-safety:
 * - Uses mutexes for all shared data access
 * - Non-blocking with timeouts to prevent deadlocks
 * 
 * @author Cristian David Araujo A.
 * @version 1.0
 * @date 2025-11-18
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "sensor_fusion.h"
#include "bno055.h"
#include "types_utils.h"
#include "config_utils.h"
#include "gpio_utils.h"

#include <math.h>
#include <string.h>

// =============================================================================
// LOGGING
// =============================================================================

static const char *TAG = "FUSION_TASK";

// =============================================================================
// EXTERNAL SHARED DATA
// =============================================================================

extern bno055_t g_bno055;                      // IMU sensor instance
extern velocity_t g_robot_estimated;           // Forward kinematics velocity
extern SemaphoreHandle_t g_estimated_data_mutex;  // Protects g_robot_estimated
extern SemaphoreHandle_t g_fused_pose_mutex;   // Protects g_fused_pose
extern fused_pose_t g_fused_pose;              // Published fused pose

// =============================================================================
// MODULE-LEVEL STATE
// =============================================================================

static sensor_fusion_handle_t s_fusion_handle = NULL;
static bool s_imu_initialized = false;

// =============================================================================
// INTERNAL HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Read IMU data from BNO055 sensor
 * 
 * Reads orientation (Euler angles), linear acceleration, and angular velocity
 * from the BNO055 IMU sensor. Handles I2C communication errors gracefully.
 * 
 * @param[out] imu_data Pointer to structure to store IMU data
 * @return true if read successful, false on error
 */
static bool read_imu_data(imu_data_t *imu_data)
{
    if (!s_imu_initialized || imu_data == NULL) {
        return false;
    }
    
    float yaw, pitch, roll;
    float gx, gy, gz;
    float ax, ay, az;
    
    // Read Euler angles (absolute orientation from sensor fusion)
    // BNO055 API uses direct pointer parameters
    bno055_get_euler_angles(&g_bno055, &yaw, &pitch, &roll);
    
    // Read gyroscope data (angular velocity)
    bno055_get_gyro(&g_bno055, &gx, &gy, &gz);
    
    // Read linear acceleration (with gravity removed if in NDOF mode)
    // Note: Use bno055_get_acceleration for raw accel if needed
    bno055_get_acceleration(&g_bno055, &ax, &ay, &az);
    
    // Populate IMU data structure
    // BNO055 units configured as: radians for angles, m/s² for accel, rad/s for gyro
    imu_data->yaw_rad = yaw;
    imu_data->pitch_rad = pitch;
    imu_data->roll_rad = roll;
    
    // Gyroscope (rad/s)
    imu_data->gyro_x = gx;
    imu_data->gyro_y = gy;
    imu_data->gyro_z = gz;
    
    // Linear acceleration (m/s²)
    imu_data->accel_x = ax;
    imu_data->accel_y = ay;
    imu_data->accel_z = az;
    
    // Timestamp
    imu_data->timestamp_us = esp_timer_get_time();
    
    return true;
}

/**
 * @brief Initialize BNO055 IMU sensor
 * 
 * Configures the BNO055 in NDOF mode (9-DOF sensor fusion) with appropriate
 * units and power settings.
 * 
 * @return true if initialization successful, false otherwise
 */
static bool initialize_imu_sensor(void)
{
    ESP_LOGI(TAG, "Initializing BNO055 IMU sensor...");
    
    // Initialize BNO055 with I2C configuration (GPIO 17=SDA, 18=SCL)
    int status = bno055_init(&g_bno055, GPIO_IMU_I2C_SDA, GPIO_IMU_I2C_SCL, 
                             I2C_NUM_0);
    
    if (status != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "BNO055 initialization failed with status %d", status);
        return false;
    }
    
    // Set operation mode to NDOF (Nine Degrees of Freedom - full sensor fusion)
    status = bno055_set_operation_mode(&g_bno055, BNO055_MODE_NDOF);
    if (status != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set NDOF mode");
        return false;
    }
    
    // Configure units: radians for angles, m/s² for acceleration, rad/s for gyro
    // Note: Parameter order is accel, gyro, euler, temp, orientation
    status = bno055_set_unit(&g_bno055, 
                            BNO055_ACCEL_UNIT_MSQ,     // Accel in m/s²
                            BNO055_GYRO_UNIT_RPS,      // Gyro in rad/s
                            BNO055_EULER_UNIT_RAD,     // Euler angles in radians
                            BNO055_TEMP_UNIT_CELSIUS,  // Temperature in Celsius
                            0x00);                     // Windows orientation format
    if (status != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to set units (using defaults)");
        // Continue anyway - not critical
    }
    
    // Set to normal power mode
    status = bno055_set_power_mode(&g_bno055, BNO055_POWER_NORMAL);
    if (status != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to set power mode");
    }
    
    // Wait for sensor calibration to stabilize
    ESP_LOGI(TAG, "Waiting for IMU calibration...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Check calibration status
    status = bno055_get_calibration_status(&g_bno055);
    if (status == BNO055_SUCCESS) {
        // Extract calibration levels from status byte
        uint8_t sys = (g_bno055.calib_stat >> 6) & 0x03;
        uint8_t gyro = (g_bno055.calib_stat >> 4) & 0x03;
        uint8_t accel = (g_bno055.calib_stat >> 2) & 0x03;
        uint8_t mag = g_bno055.calib_stat & 0x03;
        
        ESP_LOGI(TAG, "IMU Calibration Status:");
        ESP_LOGI(TAG, "  System: %d/3", sys);
        ESP_LOGI(TAG, "  Gyro: %d/3", gyro);
        ESP_LOGI(TAG, "  Accel: %d/3", accel);
        ESP_LOGI(TAG, "  Mag: %d/3", mag);
    }
    
    s_imu_initialized = true;
    ESP_LOGI(TAG, "BNO055 IMU initialized successfully");
    
    return true;
}

// =============================================================================
// TASK IMPLEMENTATION
// =============================================================================

/**
 * @brief Sensor fusion task main loop
 * 
 * This task runs at 100 Hz (10 ms period) and performs the following:
 * 1. Read IMU data from BNO055
 * 2. Read forward kinematics velocity estimate
 * 3. Update fusion algorithm with both inputs
 * 4. Compute fused pose
 * 5. Publish fused pose for other tasks
 * 
 * The task uses timeouts on all mutex operations to prevent deadlocks
 * and provides diagnostic logging on errors.
 * 
 * @param pvParameters Unused task parameter
 */
void task_sensor_fusion(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor fusion task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    
    // Initialize IMU sensor
    if (!initialize_imu_sensor()) {
        ESP_LOGE(TAG, "IMU initialization failed - fusion task will exit");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize fusion algorithm with default configuration
    fusion_config_t fusion_config = {
        .alpha_orientation = FUSION_ALPHA_ORIENTATION,
        .alpha_velocity = FUSION_ALPHA_VELOCITY,
        .gyro_drift_compensation = FUSION_GYRO_DRIFT_COMP,
        .acceleration_threshold = FUSION_ACCEL_THRESHOLD,
        .enable_velocity_fusion = true,
        .enable_position_integration = true
    };
    
    if (!sensor_fusion_init(&fusion_config, &s_fusion_handle)) {
        ESP_LOGE(TAG, "Sensor fusion initialization failed - task will exit");
        vTaskDelete(NULL);
        return;
    }
    
    // Local variables
    imu_data_t imu_data;
    velocity_t kinematics_velocity;
    fused_pose_t fused_pose;
    
    uint32_t cycle_count = 0;
    uint32_t imu_read_errors = 0;
    uint32_t kinematics_read_errors = 0;
    
    ESP_LOGI(TAG, "Entering main fusion loop (period: %d ms)", FUSION_TASK_PERIOD_MS);
    
    // =========================================================================
    // MAIN TASK LOOP
    // =========================================================================
    
    while (1) {
        // ---------------------------------------------------------------------
        // STEP 1: READ IMU DATA
        // ---------------------------------------------------------------------
        
        if (read_imu_data(&imu_data)) {
            // Successfully read IMU, update fusion
            sensor_fusion_update_imu(s_fusion_handle, &imu_data);
        } else {
            imu_read_errors++;
            if (imu_read_errors % 100 == 0) {
                ESP_LOGW(TAG, "IMU read errors: %lu", imu_read_errors);
            }
        }
        
        // ---------------------------------------------------------------------
        // STEP 2: READ FORWARD KINEMATICS VELOCITY
        // ---------------------------------------------------------------------
        
        if (g_estimated_data_mutex && 
            xSemaphoreTake(g_estimated_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            kinematics_velocity = g_robot_estimated;
            xSemaphoreGive(g_estimated_data_mutex);
            
            // Update fusion with kinematics data
            sensor_fusion_update_kinematics(s_fusion_handle, &kinematics_velocity);
        } else {
            kinematics_read_errors++;
            if (kinematics_read_errors % 100 == 0) {
                ESP_LOGW(TAG, "Kinematics read errors: %lu", kinematics_read_errors);
            }
        }
        
        // ---------------------------------------------------------------------
        // STEP 3: COMPUTE FUSED POSE
        // ---------------------------------------------------------------------
        
        if (sensor_fusion_compute(s_fusion_handle, &fused_pose)) {
            // Fusion successful, publish result
            
            if (g_fused_pose_mutex && 
                xSemaphoreTake(g_fused_pose_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                memcpy(&g_fused_pose, &fused_pose, sizeof(fused_pose_t));
                xSemaphoreGive(g_fused_pose_mutex);
            } else {
                ESP_LOGW(TAG, "Failed to publish fused pose (mutex timeout)");
            }
        }
        
        // ---------------------------------------------------------------------
        // STEP 4: PERIODIC LOGGING (Every 1 second)
        // ---------------------------------------------------------------------
        
        cycle_count++;
        if (cycle_count % (1000 / FUSION_TASK_PERIOD_MS) == 0) {
            // Log fusion statistics
            uint32_t imu_updates, kin_updates, fusion_computes;
            uint64_t last_update_us;
            
            if (sensor_fusion_get_stats(s_fusion_handle, &imu_updates, &kin_updates, 
                                       &fusion_computes, &last_update_us)) {
                ESP_LOGI(TAG, "Fusion Stats: IMU=%lu, Kin=%lu, Compute=%lu", 
                         imu_updates, kin_updates, fusion_computes);
                ESP_LOGI(TAG, "Pose: pos=(%.2f, %.2f) vel=(%.2f, %.2f) yaw=%.2f°",
                         fused_pose.pos_x, fused_pose.pos_y,
                         fused_pose.vel_x, fused_pose.vel_y,
                         fused_pose.yaw_rad * 180.0f / M_PI);
                ESP_LOGI(TAG, "Confidence: orientation=%.2f, velocity=%.2f",
                         fused_pose.orientation_confidence,
                         fused_pose.velocity_confidence);
            }
            
            // Log error counts if any
            if (imu_read_errors > 0 || kinematics_read_errors > 0) {
                ESP_LOGW(TAG, "Errors - IMU: %lu, Kinematics: %lu", 
                         imu_read_errors, kinematics_read_errors);
            }
        }
        
        // ---------------------------------------------------------------------
        // STEP 5: WAIT FOR NEXT CYCLE
        // ---------------------------------------------------------------------
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FUSION_TASK_PERIOD_MS));
    }
    
    // Should never reach here
    sensor_fusion_deinit(s_fusion_handle);
    vTaskDelete(NULL);
}
