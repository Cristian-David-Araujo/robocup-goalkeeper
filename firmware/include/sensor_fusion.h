/**
 * @file sensor_fusion.h
 * @brief Sensor fusion interface for IMU and kinematics integration
 * 
 * This module implements a complementary filter-based sensor fusion algorithm
 * that combines high-rate IMU data (100 Hz, 10 ms period) with forward kinematics
 * estimates to produce a robust, stable pose estimate for the robot.
 * 
 * Algorithm: Complementary Filter
 * --------------------------------
 * The complementary filter fuses two sources:
 * 1. IMU (BNO055): Provides high-frequency orientation (pitch, roll, yaw) and
 *    linear acceleration data. Subject to drift over time but excellent
 *    short-term accuracy.
 * 2. Forward Kinematics: Computed from wheel encoders, provides velocity
 *    estimates. Subject to slip and accumulated position error but stable
 *    long-term reference.
 * 
 * The filter applies:
 * - High-pass filter on IMU: Captures fast dynamics, rejects low-freq drift
 * - Low-pass filter on kinematics: Provides stable long-term reference
 * - Weighted fusion: pose_fused = α·IMU + (1-α)·kinematics
 * 
 * Update rates:
 * - IMU reading: 10 ms (100 Hz)
 * - Kinematics: Updated by sensor task at 2 ms (500 Hz), sampled at 10 ms
 * - Fusion output: 10 ms (100 Hz)
 * 
 * Thread-safety:
 * - All public functions are thread-safe
 * - Uses internal mutexes for state protection
 * - Designed for single-writer, multiple-reader access pattern
 * 
 * @author Cristian David Araujo A.
 * @version 1.0
 * @date 2025-11-18
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>
#include "types_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

// Note: imu_data_t and fused_pose_t are defined in types_utils.h
// to avoid circular dependencies and allow usage across modules

/**
 * @brief Complementary filter configuration parameters
 * 
 * Tuning parameters for the complementary filter algorithm.
 */
typedef struct {
    float alpha_orientation;  ///< Weight for IMU orientation [0.0, 1.0]. Typical: 0.98
    float alpha_velocity;     ///< Weight for IMU velocity [0.0, 1.0]. Typical: 0.02
    float gyro_drift_compensation;  ///< Gyro drift compensation factor. Typical: 0.001
    float acceleration_threshold;   ///< Min acceleration to integrate (m/s²). Typical: 0.1
    bool enable_velocity_fusion;    ///< Enable velocity fusion (vs orientation-only)
    bool enable_position_integration;  ///< Enable position integration
} fusion_config_t;

/**
 * @brief Sensor fusion handle (opaque pointer)
 * 
 * Handle to sensor fusion state. Internal implementation hidden.
 */
typedef struct sensor_fusion_state_s *sensor_fusion_handle_t;

// =============================================================================
// INITIALIZATION AND CONFIGURATION
// =============================================================================

/**
 * @brief Initialize sensor fusion module
 * 
 * Creates and initializes the sensor fusion state with default or custom
 * configuration parameters. Must be called before any other fusion functions.
 * 
 * Default configuration:
 * - alpha_orientation = 0.98 (98% IMU, 2% kinematics for orientation)
 * - alpha_velocity = 0.05 (5% IMU acceleration, 95% kinematics for velocity)
 * - gyro_drift_compensation = 0.001
 * - acceleration_threshold = 0.1 m/s²
 * - enable_velocity_fusion = true
 * - enable_position_integration = true
 * 
 * @param[in] config Pointer to configuration structure. If NULL, uses defaults.
 * @param[out] handle Pointer to store created fusion handle
 * 
 * @return true if initialization successful, false otherwise
 * 
 * @note This function allocates memory. Call sensor_fusion_deinit() to free.
 * @note Thread-safe: Can be called from any task
 */
bool sensor_fusion_init(const fusion_config_t *config, sensor_fusion_handle_t *handle);

/**
 * @brief Deinitialize sensor fusion module
 * 
 * Frees all resources associated with the sensor fusion handle.
 * 
 * @param[in] handle Fusion handle to deinitialize
 * 
 * @note Thread-safe: Can be called from any task
 * @note After calling this, the handle becomes invalid
 */
void sensor_fusion_deinit(sensor_fusion_handle_t handle);

/**
 * @brief Update fusion configuration parameters at runtime
 * 
 * Allows tuning of fusion parameters without reinitializing the module.
 * Useful for online calibration and parameter optimization.
 * 
 * @param[in] handle Fusion handle
 * @param[in] config New configuration parameters
 * 
 * @return true if update successful, false if handle invalid
 * 
 * @note Thread-safe: Can be called from any task
 */
bool sensor_fusion_update_config(sensor_fusion_handle_t handle, 
                                 const fusion_config_t *config);

/**
 * @brief Reset fusion state to initial conditions
 * 
 * Clears all integrated state (position, velocity) and resets filters.
 * Useful when robot position is manually reset or known discontinuity occurs.
 * 
 * @param[in] handle Fusion handle
 * 
 * @return true if reset successful, false if handle invalid
 * 
 * @note Thread-safe: Can be called from any task
 */
bool sensor_fusion_reset(sensor_fusion_handle_t handle);

// =============================================================================
// FUSION UPDATE FUNCTIONS
// =============================================================================

/**
 * @brief Update fusion with new IMU measurement
 * 
 * Processes new IMU data and updates the internal filter state.
 * Should be called at IMU update rate (100 Hz / 10 ms).
 * 
 * @param[in] handle Fusion handle
 * @param[in] imu_data Pointer to IMU measurement data
 * 
 * @return true if update successful, false if handle or data invalid
 * 
 * @note Thread-safe: Can be called from IMU reading task
 * @note Internally timestamps the update for proper integration
 */
bool sensor_fusion_update_imu(sensor_fusion_handle_t handle, 
                              const imu_data_t *imu_data);

/**
 * @brief Update fusion with new kinematics estimate
 * 
 * Provides forward kinematics velocity estimate to the fusion filter.
 * Should be called at kinematics update rate (typically 100 Hz / 10 ms).
 * 
 * @param[in] handle Fusion handle
 * @param[in] velocity Pointer to velocity estimate from forward kinematics
 * 
 * @return true if update successful, false if handle or data invalid
 * 
 * @note Thread-safe: Can be called from sensor/kinematics task
 */
bool sensor_fusion_update_kinematics(sensor_fusion_handle_t handle, 
                                     const velocity_t *velocity);

/**
 * @brief Compute fused pose estimate
 * 
 * Executes the complementary filter algorithm to produce a fused pose estimate
 * combining IMU and kinematics data. Should be called after updating both
 * IMU and kinematics data.
 * 
 * Algorithm steps:
 * 1. High-pass filter on IMU orientation change
 * 2. Low-pass filter on integrated kinematics orientation
 * 3. Complementary fusion: pose = α·IMU + (1-α)·kinematics
 * 4. Velocity fusion from kinematics with IMU acceleration correction
 * 5. Position integration from fused velocity
 * 
 * @param[in] handle Fusion handle
 * @param[out] fused_pose Pointer to store computed fused pose
 * 
 * @return true if computation successful, false if handle or output invalid
 * 
 * @note Thread-safe: Can be called from any task
 * @note Returns most recent fusion result even if no new data available
 */
bool sensor_fusion_compute(sensor_fusion_handle_t handle, 
                          fused_pose_t *fused_pose);

// =============================================================================
// QUERY FUNCTIONS
// =============================================================================

/**
 * @brief Get latest fused pose estimate (non-blocking)
 * 
 * Retrieves the most recently computed fused pose without triggering
 * a new computation. Useful for multiple consumers reading the same data.
 * 
 * @param[in] handle Fusion handle
 * @param[out] fused_pose Pointer to store latest fused pose
 * 
 * @return true if data retrieved successfully, false if handle invalid
 * 
 * @note Thread-safe: Can be called from any task
 * @note Does not trigger new computation, returns cached result
 */
bool sensor_fusion_get_pose(sensor_fusion_handle_t handle, 
                           fused_pose_t *fused_pose);

/**
 * @brief Get fusion algorithm statistics
 * 
 * Retrieves diagnostic information about the fusion algorithm performance.
 * 
 * @param[in] handle Fusion handle
 * @param[out] imu_update_count Total number of IMU updates processed
 * @param[out] kinematics_update_count Total number of kinematics updates
 * @param[out] fusion_compute_count Total number of fusion computations
 * @param[out] last_update_us Timestamp of last fusion update (μs)
 * 
 * @return true if statistics retrieved, false if handle invalid
 * 
 * @note Thread-safe: Can be called from any task
 */
bool sensor_fusion_get_stats(sensor_fusion_handle_t handle,
                            uint32_t *imu_update_count,
                            uint32_t *kinematics_update_count,
                            uint32_t *fusion_compute_count,
                            uint64_t *last_update_us);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_FUSION_H
