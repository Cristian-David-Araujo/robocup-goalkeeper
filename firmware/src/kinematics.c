#include "kinematics.h"
#include <math.h>

const float alpha[3] = {ROBOT_WHEEL_0_OFFSET, ROBOT_WHEEL_1_OFFSET, ROBOT_WHEEL_2_OFFSET};  // Wheel angles in radians

void compute_inverse_kinematics(Velocity v, WheelSpeeds *w) {
    for (int i = 0; i < 3; ++i) {
        float a = alpha[i];
        w->phi_dot[i] = (-sinf(a) * v.vx + cosf(a) * v.vy + ROBOT_BODY_RADIUS * v.wz) / ROBOT_WHEEL_RADIUS;
    }
}


// Matrix A⁺ = (Aᵀ·A)⁻¹·Aᵀ — hardcoded inverse for this geometry
// You can also calculate it dynamically with matrix ops if needed

void compute_forward_kinematics(WheelSpeeds w, Velocity *v) {
    // Build matrix A (3x3) and pseudo-invert manually:
    // A = [
    //   [-sin(α₁)/r,  cos(α₁)/r, R/r],
    //   [-sin(α₂)/r,  cos(α₂)/r, R/r],
    //   [-sin(α₃)/r,  cos(α₃)/r, R/r]
    // ]

    // We compute A⁺ * φ̇ = [vx, vy, ω]
    // This is done with hardcoded pseudo-inverse for speed (3x3 system)

    float A[3][3];
    for (int i = 0; i < 3; ++i) {
        float a = alpha[i];
        A[i][0] = -sinf(a) / ROBOT_WHEEL_RADIUS;
        A[i][1] =  cosf(a) / ROBOT_WHEEL_RADIUS;
        A[i][2] =  ROBOT_BODY_RADIUS / ROBOT_WHEEL_RADIUS;
    }

    // Compute Aᵗ * A
    float AtA[3][3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                AtA[i][j] += A[k][i] * A[k][j];

    // Invert AtA (3x3 matrix inversion, only done once here)
    // For simplicity, assume symmetric positive definite => use basic inverse
    float det = AtA[0][0]*(AtA[1][1]*AtA[2][2] - AtA[1][2]*AtA[2][1])
              - AtA[0][1]*(AtA[1][0]*AtA[2][2] - AtA[1][2]*AtA[2][0])
              + AtA[0][2]*(AtA[1][0]*AtA[2][1] - AtA[1][1]*AtA[2][0]);

    if (fabsf(det) < 1e-6f) {
        v->vx = v->vy = v->wz = 0;
        return;
    }

    float invDet = 1.0f / det;

    float invAtA[3][3];
    invAtA[0][0] =  (AtA[1][1]*AtA[2][2] - AtA[1][2]*AtA[2][1]) * invDet;
    invAtA[0][1] = -(AtA[0][1]*AtA[2][2] - AtA[0][2]*AtA[2][1]) * invDet;
    invAtA[0][2] =  (AtA[0][1]*AtA[1][2] - AtA[0][2]*AtA[1][1]) * invDet;
    invAtA[1][0] = -(AtA[1][0]*AtA[2][2] - AtA[1][2]*AtA[2][0]) * invDet;
    invAtA[1][1] =  (AtA[0][0]*AtA[2][2] - AtA[0][2]*AtA[2][0]) * invDet;
    invAtA[1][2] = -(AtA[0][0]*AtA[1][2] - AtA[0][2]*AtA[1][0]) * invDet;
    invAtA[2][0] =  (AtA[1][0]*AtA[2][1] - AtA[1][1]*AtA[2][0]) * invDet;
    invAtA[2][1] = -(AtA[0][0]*AtA[2][1] - AtA[0][1]*AtA[2][0]) * invDet;
    invAtA[2][2] =  (AtA[0][0]*AtA[1][1] - AtA[0][1]*AtA[1][0]) * invDet;

    // Compute A⁺ = (AᵗA)⁻¹ Aᵗ and multiply with w.phi_dot
    float At[3][3];
    for (int i = 0; i < 3; ++i)      // transpose A
        for (int j = 0; j < 3; ++j)
            At[i][j] = A[j][i];

    float Aplus[3][3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                Aplus[i][j] += invAtA[i][k] * At[k][j];

    // Compute v = A⁺ * φ̇
    v->vx = v->vy = v->wz = 0;
    for (int i = 0; i < 3; ++i) {
        v->vx += Aplus[0][i] * w.phi_dot[i];
        v->vy += Aplus[1][i] * w.phi_dot[i];
        v->wz += Aplus[2][i] * w.phi_dot[i];
    }
}
