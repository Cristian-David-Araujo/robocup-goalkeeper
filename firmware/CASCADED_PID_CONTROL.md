# Cascaded PID Control Implementation

## Overview

This document describes the implementation of a **cascaded (two-layer) PID control architecture** for the RoboCup Goalkeeper robot. The system now features:

- **Outer Loop:** Robot velocity control (vx, vy, wz)
- **Inner Loop:** Individual wheel speed control (φ̇₁, φ̇₂, φ̇₃)

## Architecture

### Control Flow

```
Trajectory Task → Velocity PID Task → IK Task → Wheel PID Task → Motors
     (20ms)           (10ms)          (10ms)        (2ms)
```

### Data Flow

1. **Trajectory Task** generates desired robot velocities
2. **Velocity PID Task** compares desired vs. measured velocities, outputs corrections
3. **IK Task** converts corrected velocities to wheel speed targets
4. **Wheel PID Task** controls individual motors to achieve wheel targets
5. **Sensor Task** provides feedback at both levels (wheel speeds and robot velocity)

## Benefits of Cascaded Control

### 1. **Improved Tracking Accuracy**
- Outer loop compensates for kinematic model errors
- Feedback at robot velocity level corrects for slip, disturbances
- Better trajectory following in real-world conditions

### 2. **Modular Tuning**
- Velocity PID and wheel PID can be tuned independently
- Outer loop focuses on trajectory tracking
- Inner loop focuses on motor control dynamics

### 3. **Robustness**
- System tolerates model uncertainties
- Disturbances (floor friction, battery voltage) are rejected
- More stable behavior under varying loads

### 4. **Industry Standard**
- Standard approach in mobile robotics
- Well-understood design patterns
- Easier to debug and maintain

## Implementation Details

### New Files

**tasks/task_velocity_control.c**
- Implements outer-loop velocity PID control
- Priority: 4 (medium)
- Period: 10 ms
- Three PID controllers for vx, vy, wz

### Modified Files

**main/main.c**
- Added `g_velocity_pid[3]` and `g_velocity_pid_param`
- Added `g_desired_velocity_queue` and updated `g_velocity_command_queue`
- Added `g_velocity_pid_mutex`
- Created velocity control task with priority 4
- Updated task priorities (Wheel PID now priority 2, Trajectory now priority 3)

**main/init.c**
- Added initialization for velocity PID controllers
- Updated logging to distinguish wheel vs. velocity PIDs

**utils/config_utils.h**
- Added velocity PID tuning parameters:
  - `PID_VELOCITY_KP = 1.0`
  - `PID_VELOCITY_KI = 0.1`
  - `PID_VELOCITY_KD = 0.05`
  - `PID_VELOCITY_MAX_OUTPUT = 1.0`
  - `PID_VELOCITY_MIN_OUTPUT = -1.0`
- Added `VELOCITY_CONTROL_TASK_PERIOD_MS = 10`

**README.md**
- Updated architecture diagrams
- Added velocity control task description
- Updated task priorities and communication patterns
- Added design rationale for cascaded control

### Queue Communication

| Queue | From → To | Item Type | Size |
|-------|-----------|-----------|------|
| `g_desired_velocity_queue` | Trajectory → Velocity PID | `velocity_t` | 2 |
| `g_velocity_command_queue` | Velocity PID → IK | `velocity_t` | 2 |
| `g_wheel_target_queue` | IK → Wheel PID | `wheel_speeds_t` | 2 |

### Task Priorities (Updated)

| Priority | Task | Purpose |
|----------|------|---------|
| 6 | Sensor | Encoder reading and filtering |
| 5 | IK | Kinematic transformation |
| 4 | **Velocity PID** (NEW) | **Robot velocity control** |
| 3 | Trajectory | Reference generation |
| 2 | Wheel PID | Motor control |

## Tuning Guidelines

### Velocity PID (Outer Loop)

**Current Settings:**
- Kp = 1.0 (moderate response)
- Ki = 0.1 (slow integral correction)
- Kd = 0.05 (light damping)

**Tuning Process:**
1. Start with P-only control (Ki=0, Kd=0)
2. Increase Kp until oscillations appear, then reduce by 50%
3. Add Ki slowly to eliminate steady-state error
4. Add Kd if overshoot is excessive

**Typical Issues:**
- **Too much oscillation:** Reduce Kp or increase Kd
- **Steady-state error:** Increase Ki
- **Sluggish response:** Increase Kp
- **Overshoot:** Increase Kd

### Wheel PID (Inner Loop)

**Current Settings:** (unchanged)
- Kp = 0.1
- Ki = 0.006
- Kd = 0.0

**Note:** Inner loop should be tuned first, then outer loop

## Testing Recommendations

### 1. **Open-Loop Test**
```c
// In task_velocity_control.c, temporarily disable PID:
corrected = desired;  // Pass through without correction
```
Verify kinematic model is working correctly.

### 2. **Step Response Test**
Command a sudden velocity change (e.g., vx = 0 → 0.5 m/s) and observe:
- Rise time
- Overshoot
- Settling time
- Steady-state error

### 3. **Circular Trajectory Test**
Run existing circular trajectory and compare:
- Commanded path vs. actual path
- Tracking error over time
- Response to disturbances

### 4. **Logging**
Monitor these values:
- Desired velocity (from trajectory)
- Measured velocity (from sensors)
- Corrected velocity (PID output)
- Tracking error (desired - measured)

## Expected Behavior

### Without Velocity PID (Open Loop)
- Trajectory may drift due to model errors
- No correction for wheel slip or disturbances
- Path accuracy depends on kinematic model accuracy

### With Velocity PID (Closed Loop)
- Active correction of tracking errors
- Compensation for model uncertainties
- Better path following even with disturbances
- Slight increase in control effort (more actuator activity)

## Troubleshooting

### Issue: Robot oscillates
**Cause:** Velocity PID gains too high (especially Kp or Kd)
**Solution:** Reduce `PID_VELOCITY_KP` and/or `PID_VELOCITY_KD`

### Issue: Slow response, large tracking error
**Cause:** Velocity PID gains too low
**Solution:** Increase `PID_VELOCITY_KP`

### Issue: Steady-state offset
**Cause:** Insufficient integral action
**Solution:** Increase `PID_VELOCITY_KI`

### Issue: System unstable after adding velocity PID
**Cause:** Interaction between loops, improper tuning order
**Solution:** 
1. Verify wheel PID is stable first
2. Start with low velocity PID gains
3. Tune velocity PID gradually

### Issue: Tasks timing out on mutexes/queues
**Cause:** Priority inversion or insufficient CPU time
**Solution:** 
- Check task watermarks with `vTaskGetRunTimeStats()`
- Verify priorities are correctly assigned
- Increase timeout values if necessary

## Performance Metrics

Monitor these to validate improvement:

1. **Tracking Error RMS:** Should decrease with cascaded control
2. **Path Deviation:** Maximum deviation from desired trajectory
3. **Convergence Time:** Time to reach steady-state after command change
4. **Control Effort:** Motor command activity (should be reasonable, not excessive)

## Future Improvements

1. **Adaptive Tuning:** Adjust PID gains based on operating conditions
2. **Feedforward:** Add model-based feedforward to reduce tracking error
3. **Acceleration Limits:** Constrain velocity PID output rate for smoother motion
4. **Anti-windup:** Enhanced anti-windup strategies for velocity PIDs
5. **Gain Scheduling:** Different PID gains for different velocity ranges

## References

- Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2014). *Feedback Control of Dynamic Systems*
- Siegwart, R., & Nourbakhsh, I. R. (2004). *Introduction to Autonomous Mobile Robots*
- ESP-IDF FreeRTOS Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html

---

**Implementation Date:** November 2025
**Version:** 1.1
**Status:** Ready for testing
