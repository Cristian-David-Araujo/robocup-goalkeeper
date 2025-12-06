#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#define ROBOT_BODY_RADIUS 0.08f // Radius of the robot in meters
#define ROBOT_WHEEL_RADIUS 0.03f // Radius of the robot wheels in meters
#define ROBOT_WHEEL_0_OFFSET M_PI/6 // Offset angle for wheel 0 in radians
#define ROBOT_WHEEL_1_OFFSET 5*M_PI/6 // Offset angle for wheel 1 in radians
#define ROBOT_WHEEL_2_OFFSET 3*M_PI/2 // Offset angle for wheel 2 in radians
#define TOLERANCE 1e-4f
#define TEST_COUNT 1000

typedef struct {
    float vx;
    float vy;
    float wz;
} Velocity;

typedef struct {
    float phi_dot[3];
} WheelSpeeds;

// Constants
const float alpha[3] = {ROBOT_WHEEL_0_OFFSET, ROBOT_WHEEL_1_OFFSET, ROBOT_WHEEL_2_OFFSET};

// Precomputed pseudo-inverse A⁺ (A⁺ = (AᵗA)⁻¹ Aᵗ)
static float Aplus[3][3];

// Flag to check if Aplus has been initialized
static int Aplus_initialized = 0;

void compute_inverse_kinematics(Velocity v, WheelSpeeds *w) {
    for (int i = 0; i < 3; ++i) {
        float a = alpha[i];
        w->phi_dot[i] = (-sinf(a) * v.vx + cosf(a) * v.vy + ROBOT_BODY_RADIUS * v.wz) / ROBOT_WHEEL_RADIUS;
    }
}

void init_forward_kinematics_matrix() {
    float A[3][3];

    // Build A
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

    // Compute determinant
    float det = AtA[0][0]*(AtA[1][1]*AtA[2][2] - AtA[1][2]*AtA[2][1])
              - AtA[0][1]*(AtA[1][0]*AtA[2][2] - AtA[1][2]*AtA[2][0])
              + AtA[0][2]*(AtA[1][0]*AtA[2][1] - AtA[1][1]*AtA[2][0]);

    if (fabsf(det) < 1e-6f) return; // Singular matrix

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

    // Transpose A
    float At[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            At[i][j] = A[j][i];

    // Compute Aplus = (AtA)^-1 * At
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            Aplus[i][j] = 0;
            for (int k = 0; k < 3; ++k)
                Aplus[i][j] += invAtA[i][k] * At[k][j];
        }

    Aplus_initialized = 1;
}

void compute_forward_kinematics(WheelSpeeds w, Velocity *v) {
    if (!Aplus_initialized) {
        init_forward_kinematics_matrix();
        if (!Aplus_initialized) {
            v->vx = v->vy = v->wz = 0;
            return;
        }
    }

    v->vx = v->vy = v->wz = 0;
    for (int i = 0; i < 3; ++i) {
        v->vx += Aplus[0][i] * w.phi_dot[i];
        v->vy += Aplus[1][i] * w.phi_dot[i];
        v->wz += Aplus[2][i] * w.phi_dot[i];
    }
}


float rand_float(float min, float max) {
    return min + (float)rand() / RAND_MAX * (max - min);
}

void run_bulk_tests(int count) {
    int passed = 0;
    for (int i = 0; i < count; ++i) {
        Velocity v = {
            .vx = rand_float(-3.0f, 3.0f),
            .vy = rand_float(-3.0f, 3.0f),
            .wz = rand_float(-6.0f, 6.0f)
        };

        WheelSpeeds ws;
        compute_inverse_kinematics(v, &ws);

        Velocity recovered;
        compute_forward_kinematics(ws, &recovered);

        float err_vx = fabsf(v.vx - recovered.vx);
        float err_vy = fabsf(v.vy - recovered.vy);
        float err_wz = fabsf(v.wz - recovered.wz);

        printf("Test %3d | Input: [vx=%.3f, vy=%.3f, wz=%.3f] → ", i+1, v.vx, v.vy, v.wz);
        printf("Recovered: [vx=%.3f, vy=%.3f, wz=%.3f] ", recovered.vx, recovered.vy, recovered.wz);

        if (err_vx < TOLERANCE && err_vy < TOLERANCE && err_wz < TOLERANCE) {
            printf("✅ PASS\n");
            passed++;
        } else {
            printf("❌ FAIL (err vx=%.5f, vy=%.5f, wz=%.5f)\n", err_vx, err_vy, err_wz);
        }
    }
    printf("\n🎯 %d / %d tests passed.\n", passed, count);
}

int main() {
    srand((unsigned)time(NULL));
    printf("Running %d randomized round-trip kinematics tests...\n\n", TEST_COUNT);
    run_bulk_tests(TEST_COUNT);
    return 0;
}
