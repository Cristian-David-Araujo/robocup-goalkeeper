/**
 * @file pid.c
 * @brief PID controller implementation
 * 
 * Implements positional and incremental PID control algorithms with
 * anti-windup protection and derivative filtering.
 */

#include "pid.h"
#include <stdbool.h>
#include <sys/param.h>
#include <stdlib.h>
#include <stdio.h>

// =============================================================================
// INTERNAL TYPES
// =============================================================================

/// @brief Function pointer type for PID calculation algorithms
typedef void (*pid_cal_func_t)(struct pid_block_t *pid);

/**
 * @brief Internal PID control block structure
 * 
 * Contains all state variables and configuration for a PID controller.
 * This structure is opaque to external users.
 */
struct pid_block_t {
    // Configuration parameters
    float kp;                          ///< Proportional gain
    float ki;                          ///< Integral gain
    float kd;                          ///< Derivative gain
    float max_output;                  ///< Maximum output limit
    float min_output;                  ///< Minimum output limit
    float set_point;                   ///< Target setpoint
    float beta;                        ///< Derivative filter coefficient
    
    // State variables
    float error;                       ///< Current error e(n)
    float previous_err1;               ///< Previous error e(n-1)
    float previous_err2;               ///< Error two steps ago e(n-2)
    float integral_err;                ///< Accumulated integral error
    float derivative_err;              ///< Current derivative term
    float previous_derivative_err;     ///< Previous derivative (for filtering)
    float previous_output;             ///< Previous output u(n-1) (incremental mode)
    float output;                      ///< Current output
    
    // Algorithm selector
    pid_cal_func_t calculate_func;     ///< Calculation function pointer
};

// =============================================================================
// INTERNAL FUNCTION DECLARATIONS
// =============================================================================

static void pid_calc_positional(struct pid_block_t *pid);
static void pid_calc_incremental(struct pid_block_t *pid);

// =============================================================================
// PID CALCULATION ALGORITHMS
// =============================================================================

/**
 * @brief Positional PID calculation
 * 
 * Computes absolute control output using:
 * u(n) = Kp*e(n) + Ki*Σe + Kd*Δe
 * 
 * Features anti-windup: integral only accumulates when output is not saturated
 * or when error is driving output back toward valid range.
 * 
 * @param pid Pointer to PID block
 */
static void pid_calc_positional(struct pid_block_t *pid)
{
    // Anti-windup: Only integrate if output not saturated OR error helps unsaturate
    bool output_saturated_high = (pid->output >= pid->max_output && pid->error > 0.0f);
    bool output_saturated_low = (pid->output <= pid->min_output && pid->error < 0.0f);
    bool output_in_range = (pid->output < pid->max_output && pid->output > pid->min_output);
    
    if (output_in_range || (!output_saturated_high && !output_saturated_low)) {
        pid->integral_err += pid->error;
    }

    // Derivative term with first-order low-pass filter (beta filtering)
    float raw_derivative = pid->error - pid->previous_err1;
    pid->derivative_err = pid->beta * pid->previous_derivative_err + 
                         (1.0f - pid->beta) * raw_derivative;

    // Calculate output: P + I + D
    float output = pid->kp * pid->error +
                   pid->ki * pid->integral_err +
                   pid->kd * pid->derivative_err;

    // Clamp output to configured limits
    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    // Update state for next iteration
    pid->previous_err1 = pid->error;
    pid->previous_derivative_err = pid->derivative_err;
    pid->output = output;
}

/**
 * @brief Incremental PID calculation
 * 
 * Computes change in control output using:
 * Δu(n) = Kp*Δe + Ki*e(n) + Kd*Δ²e
 * u(n) = u(n-1) + Δu(n)
 * 
 * Incremental mode is useful when controlling actuators that accept
 * velocity/change commands rather than absolute position commands.
 * 
 * @param pid Pointer to PID block
 */
static void pid_calc_incremental(struct pid_block_t *pid)
{
    // Calculate incremental output change
    // Δu = Kp*(e(n)-e(n-1)) + Ki*e(n) + Kd*(e(n)-2*e(n-1)+e(n-2))
    float delta_output = pid->kp * (pid->error - pid->previous_err1) +
                        pid->ki * pid->error +
                        pid->kd * (pid->error - 2.0f * pid->previous_err1 + pid->previous_err2);

    // Add increment to previous output
    float output = pid->previous_output + delta_output;

    // Clamp to limits
    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    // Update state
    pid->previous_err2 = pid->previous_err1;
    pid->previous_err1 = pid->error;
    pid->previous_output = output;
    pid->output = output;
}

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

int pid_new_control_block(const pid_config_t *config, pid_block_handle_t *ret_pid)
{
    // Validate input parameters
    if (!config || !ret_pid) {
        return PID_ERR_INVALID_ARG;
    }

    // Allocate memory for PID control block
    struct pid_block_t *pid = calloc(1, sizeof(struct pid_block_t));
    if (!pid) {
        return PID_ERR_NO_MEM;
    }

    // Initialize parameters
    int ret = pid_update_parameters(pid, &config->init_param);
    if (ret != PID_OK) {
        free(pid);
        return ret;
    }

    *ret_pid = pid;
    return PID_OK;
}

int pid_del_control_block(pid_block_handle_t pid)
{
    if (!pid) {
        return PID_ERR_INVALID_ARG;
    }

    free(pid);
    return PID_OK;
}

int pid_compute(pid_block_handle_t pid, float input, float *output)
{
    if (!pid || !output) {
        return PID_ERR_INVALID_ARG;
    }
    
    // Calculate error and run PID algorithm
    pid->error = pid->set_point - input;
    pid->calculate_func(pid);
    *output = pid->output;
    
    return PID_OK;
}

int pid_update_parameters(pid_block_handle_t pid, const pid_parameter_t *params)
{
    if (!pid || !params) {
        return PID_ERR_INVALID_ARG;
    }

    // Update tuning parameters
    pid->kp = params->kp;
    pid->ki = params->ki;
    pid->kd = params->kd;
    pid->max_output = params->max_output;
    pid->min_output = params->min_output;
    pid->set_point = params->set_point;
    pid->beta = params->beta;

    // Select calculation function based on type
    switch (params->cal_type) {
        case PID_CAL_TYPE_INCREMENTAL:
            pid->calculate_func = pid_calc_incremental;
            break;
        case PID_CAL_TYPE_POSITIONAL:
            pid->calculate_func = pid_calc_positional;
            break;
        default:
            return PID_ERR_INVALID_ARG;
    }
    
    return PID_OK;
}

int pid_update_set_point(pid_block_handle_t pid, float set_point)
{
    if (!pid) {
        return PID_ERR_INVALID_ARG;
    }

    pid->set_point = set_point;
    
    // Note: We deliberately do NOT reset error accumulators here.
    // This allows smooth setpoint changes without control discontinuities.
    // If reset is desired, call pid_reset_block() explicitly.
    
    return PID_OK;
}

int pid_reset_block(pid_block_handle_t pid)
{
    if (!pid) {
        return PID_ERR_INVALID_ARG;
    }

    // Clear all state variables
    pid->error = 0.0f;
    pid->previous_err1 = 0.0f;
    pid->previous_err2 = 0.0f;
    pid->integral_err = 0.0f;
    pid->derivative_err = 0.0f;
    pid->previous_derivative_err = 0.0f;
    pid->previous_output = 0.0f;
    pid->output = 0.0f;

    return PID_OK;
}