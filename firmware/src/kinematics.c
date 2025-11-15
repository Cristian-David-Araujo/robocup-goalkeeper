/**
 * @file kinematics.c
 * @brief Kinematic transformations for three-wheeled omnidirectional robot
 * 
 * Implements forward and inverse kinematics using matrix-based approach.
 * The forward kinematics uses a precomputed pseudo-inverse matrix for efficiency.
 */

#include "kinematics.h"
#include <math.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

/// @brief Wheel orientation angles in radians (fixed by robot geometry)
static const float g_alpha[3] = {
    ROBOT_WHEEL_0_OFFSET,
    ROBOT_WHEEL_1_OFFSET,
    ROBOT_WHEEL_2_OFFSET
};

// =============================================================================
// FORWARD KINEMATICS STATE
// =============================================================================

/// @brief Pseudo-inverse matrix A⁺ for forward kinematics (computed once)
static float g_aplus[3][3];

/// @brief Initialization flag for pseudo-inverse matrix
static bool g_aplus_initialized = false;

// =============================================================================
// INTERNAL FUNCTIONS
// =============================================================================

/**
 * @brief Initialize the pseudo-inverse matrix for forward kinematics
 *
 * Computes A⁺ = (AᵀA)⁻¹Aᵀ where A is the Jacobian matrix relating wheel
 * velocities to robot velocity. This is computed once on first use and
 * cached for subsequent calls.
 *
 * The Jacobian matrix A is:
 * A = [[-sin(α₀)/r,  cos(α₀)/r,  L/r],
 *      [-sin(α₁)/r,  cos(α₁)/r,  L/r],
 *      [-sin(α₂)/r,  cos(α₂)/r,  L/r]]
 * 
 * where αᵢ are wheel angles, r is wheel radius, L is body radius.
 */
static void init_forward_kinematics_matrix(void) 
{
    float a[3][3];

    // Construct the Jacobian matrix A
    for (int i = 0; i < 3; ++i) {
        float alpha = g_alpha[i];
        a[i][0] = -sinf(alpha) / ROBOT_WHEEL_RADIUS;
        a[i][1] =  cosf(alpha) / ROBOT_WHEEL_RADIUS;
        a[i][2] =  ROBOT_BODY_RADIUS / ROBOT_WHEEL_RADIUS;
    }

    // Compute AᵀA (3x3 matrix)
    float ata[3][3] = {{0.0f}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                ata[i][j] += a[k][i] * a[k][j];
            }
        }
    }

    // Compute determinant of AᵀA
    float det = ata[0][0] * (ata[1][1] * ata[2][2] - ata[1][2] * ata[2][1])
              - ata[0][1] * (ata[1][0] * ata[2][2] - ata[1][2] * ata[2][0])
              + ata[0][2] * (ata[1][0] * ata[2][1] - ata[1][1] * ata[2][0]);

    // Check for singular matrix
    if (fabsf(det) < 1e-6f) {
        // Matrix is singular, cannot compute inverse
        // Leave g_aplus_initialized as false
        return;
    }

    float inv_det = 1.0f / det;

    // Compute (AᵀA)⁻¹ using cofactor method
    float inv_ata[3][3];
    inv_ata[0][0] =  (ata[1][1] * ata[2][2] - ata[1][2] * ata[2][1]) * inv_det;
    inv_ata[0][1] = -(ata[0][1] * ata[2][2] - ata[0][2] * ata[2][1]) * inv_det;
    inv_ata[0][2] =  (ata[0][1] * ata[1][2] - ata[0][2] * ata[1][1]) * inv_det;
    inv_ata[1][0] = -(ata[1][0] * ata[2][2] - ata[1][2] * ata[2][0]) * inv_det;
    inv_ata[1][1] =  (ata[0][0] * ata[2][2] - ata[0][2] * ata[2][0]) * inv_det;
    inv_ata[1][2] = -(ata[0][0] * ata[1][2] - ata[0][2] * ata[1][0]) * inv_det;
    inv_ata[2][0] =  (ata[1][0] * ata[2][1] - ata[1][1] * ata[2][0]) * inv_det;
    inv_ata[2][1] = -(ata[0][0] * ata[2][1] - ata[0][1] * ata[2][0]) * inv_det;
    inv_ata[2][2] =  (ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0]) * inv_det;

    // Compute Aᵀ (transpose of A)
    float at[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            at[i][j] = a[j][i];
        }
    }

    // Compute A⁺ = (AᵀA)⁻¹ * Aᵀ
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            g_aplus[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                g_aplus[i][j] += inv_ata[i][k] * at[k][j];
            }
        }
    }

    g_aplus_initialized = true;
}

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

void compute_inverse_kinematics(velocity_t v, wheel_speeds_t *w) 
{
    // Apply inverse kinematic equation for each wheel
    // φ̇ᵢ = (-sin(αᵢ)*vx + cos(αᵢ)*vy + L*wz) / r
    for (int i = 0; i < 3; ++i) {
        float alpha = g_alpha[i];
        w->phi_dot[i] = (-sinf(alpha) * v.vx + 
                         cosf(alpha) * v.vy + 
                         ROBOT_BODY_RADIUS * v.wz) / ROBOT_WHEEL_RADIUS;
    }
}

void compute_forward_kinematics(wheel_speeds_t w, velocity_t *v) 
{
    // Initialize pseudo-inverse matrix on first call
    if (!g_aplus_initialized) {
        init_forward_kinematics_matrix();
        
        // If initialization failed, return zero velocity
        if (!g_aplus_initialized) {
            v->vx = 0.0f;
            v->vy = 0.0f;
            v->wz = 0.0f;
            return;
        }
    }

    // Multiply A⁺ by wheel speed vector to get robot velocity
    // [vx, vy, wz]ᵀ = A⁺ * [φ̇₁, φ̇₂, φ̇₃]ᵀ
    v->vx = 0.0f;
    v->vy = 0.0f;
    v->wz = 0.0f;
    
    for (int i = 0; i < 3; ++i) {
        v->vx += g_aplus[0][i] * w.phi_dot[i];
        v->vy += g_aplus[1][i] * w.phi_dot[i];
        v->wz += g_aplus[2][i] * w.phi_dot[i];
    }
}
