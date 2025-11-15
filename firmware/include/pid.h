/**
 * @file pid.h
 * @brief PID controller interface for motor control
 * 
 * This module provides a flexible PID (Proportional-Integral-Derivative) controller
 * implementation supporting both positional and incremental calculation modes.
 * 
 * Features:
 * - Positional and incremental PID algorithms
 * - Anti-windup protection
 * - Derivative filtering with beta coefficient
 * - Configurable output limits
 * 
 * Thread-safety: Functions are NOT thread-safe. External synchronization required
 * if PID blocks are accessed from multiple tasks.
 * 
 * @note All identifiers follow snake_case naming convention
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// ERROR CODES
// =============================================================================

#define PID_OK 0                   ///< Operation completed successfully
#define PID_ERR_INVALID_ARG -1     ///< Invalid argument provided
#define PID_ERR_NO_MEM -2          ///< Memory allocation failed

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

/**
 * @brief PID calculation algorithm type
 * 
 * Defines which PID calculation method to use:
 * - POSITIONAL: Output is absolute control value u(n) = Kp*e(n) + Ki*Σe + Kd*Δe
 * - INCREMENTAL: Output is change in control Δu(n) = Kp*Δe + Ki*e + Kd*Δ²e
 */
typedef enum {
    PID_CAL_TYPE_INCREMENTAL,  ///< Incremental PID (outputs control change)
    PID_CAL_TYPE_POSITIONAL,   ///< Positional PID (outputs absolute control value)
} pid_calculate_type_t;

/**
 * @brief Opaque handle to PID control block
 * 
 * This handle is returned by pid_new_control_block() and must be passed
 * to all other PID functions. The internal structure is implementation-defined.
 */
typedef struct pid_block_t *pid_block_handle_t;

/**
 * @brief PID controller tuning parameters
 * 
 * Contains all configuration parameters for PID controller behavior.
 * Thread-safety: Not thread-safe. Do not modify while PID is active.
 */
typedef struct {
    float kp;                       ///< Proportional gain coefficient
    float ki;                       ///< Integral gain coefficient
    float kd;                       ///< Derivative gain coefficient
    float max_output;               ///< Maximum output limit
    float min_output;               ///< Minimum output limit
    float set_point;                ///< Target setpoint value
    pid_calculate_type_t cal_type;  ///< PID calculation algorithm
    float beta;                     ///< Derivative term filter coefficient [0.0, 1.0]
} pid_parameter_t;

/**
 * @brief PID controller initialization configuration
 */
typedef struct {
    pid_parameter_t init_param;  ///< Initial PID parameters
} pid_config_t;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

/**
 * @brief Create a new PID control block
 *
 * Allocates and initializes a new PID controller with the specified configuration.
 * The returned handle must be freed with pid_del_control_block() when no longer needed.
 *
 * @param[in] config Pointer to PID configuration structure
 * @param[out] ret_pid Pointer to receive the created PID handle
 * @return 
 *      - PID_OK: Successfully created
 *      - PID_ERR_INVALID_ARG: Invalid config or ret_pid pointer
 *      - PID_ERR_NO_MEM: Memory allocation failed
 * 
 * Thread-safety: Not thread-safe
 */
int pid_new_control_block(const pid_config_t *config, pid_block_handle_t *ret_pid);

/**
 * @brief Delete a PID control block
 *
 * Frees all resources associated with a PID controller.
 *
 * @param[in] pid PID control block handle to delete
 * @return 
 *      - PID_OK: Successfully deleted
 *      - PID_ERR_INVALID_ARG: Invalid PID handle
 * 
 * Thread-safety: Not thread-safe
 */
int pid_del_control_block(pid_block_handle_t pid);

/**
 * @brief Update PID controller parameters
 *
 * Updates the tuning parameters of an existing PID controller. This can be used
 * for runtime parameter adjustment or mode switching.
 *
 * @param[in] pid PID control block handle
 * @param[in] params Pointer to new parameters
 * @return 
 *      - PID_OK: Successfully updated
 *      - PID_ERR_INVALID_ARG: Invalid PID handle or params pointer
 * 
 * Thread-safety: Not thread-safe. Ensure exclusive access during update.
 */
int pid_update_parameters(pid_block_handle_t pid, const pid_parameter_t *params);

/**
 * @brief Update the PID setpoint
 *
 * Changes the target value that the PID controller tries to achieve.
 * Does not reset error accumulators.
 *
 * @param[in] pid PID control block handle
 * @param[in] set_point New target setpoint value
 * @return 
 *      - PID_OK: Successfully updated
 *      - PID_ERR_INVALID_ARG: Invalid PID handle
 * 
 * Thread-safety: Not thread-safe. Ensure exclusive access during update.
 */
int pid_update_set_point(pid_block_handle_t pid, float set_point);

/**
 * @brief Compute PID output
 *
 * Performs one iteration of the PID control algorithm using the current
 * process variable (input) and returns the control output.
 *
 * @param[in] pid PID control block handle
 * @param[in] input Current process variable (measured value)
 * @param[out] output Pointer to receive computed control output
 * @return 
 *      - PID_OK: Successfully computed
 *      - PID_ERR_INVALID_ARG: Invalid PID handle or output pointer
 * 
 * Thread-safety: Not thread-safe. Ensure exclusive access during computation.
 */
int pid_compute(pid_block_handle_t pid, float input, float *output);

/**
 * @brief Reset PID controller state
 *
 * Clears all error accumulators and history in the PID controller,
 * returning it to initial state. Does not change tuning parameters.
 *
 * @param[in] pid PID control block handle
 * @return 
 *      - PID_OK: Successfully reset
 *      - PID_ERR_INVALID_ARG: Invalid PID handle
 * 
 * Thread-safety: Not thread-safe. Ensure exclusive access during reset.
 */
int pid_reset_block(pid_block_handle_t pid);

#ifdef __cplusplus
}
#endif

#endif // PID_H