# RoboCup Goalkeeper Firmware

**Version:** 1.0  
**Platform:** ESP32-S3  
**RTOS:** FreeRTOS (via ESP-IDF)

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Components](#hardware-components)
4. [Software Architecture](#software-architecture)
5. [Task Descriptions](#task-descriptions)
6. [Inter-Task Communication](#inter-task-communication)
7. [Module Descriptions](#module-descriptions)
8. [Build and Flash Instructions](#build-and-flash-instructions)
9. [Configuration](#configuration)
10. [Coding Conventions](#coding-conventions)
11. [Development Workflow](#development-workflow)
12. [Troubleshooting](#troubleshooting)

---

## Overview

This firmware controls a three-wheeled omnidirectional robot for RoboCup goalkeeper applications. The system implements:

- **Closed-loop motor control** with PID feedback
- **Sensor fusion** from magnetic encoders
- **Real-time kinematics** (forward and inverse transformations)
- **Multi-task architecture** using FreeRTOS for concurrent operation
- **Thread-safe data sharing** via mutexes

### Key Features

- ✅ Three brushless DC motors with independent PID control
- ✅ Three AS5600 magnetic encoders for position/velocity feedback
- ✅ Kalman filtering for noise reduction
- ✅ Omnidirectional motion control
- ✅ Modular, maintainable architecture
- ✅ Comprehensive documentation and error handling

---

## System Architecture

### High-Level Block Diagram

```
┌──────────────────┐
│  Trajectory Gen  │  ← Generates desired robot velocity
└────────┬─────────┘
         │ (vx, vy, wz)
         ↓
┌──────────────────┐
│ Inverse Kinematics│  ← Computes target wheel speeds
└────────┬─────────┘
         │ (φ̇₁, φ̇₂, φ̇₃)
         ↓
┌──────────────────┐
│   PID Control    │  ← Computes motor commands
└────────┬─────────┘
         │ (PWM signals)
         ↓
┌──────────────────┐
│   Motors         │  ← Physical actuators
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│   Encoders       │  ← Measure wheel velocities
└────────┬─────────┘
         │ (measured φ̇)
         ↓
┌──────────────────┐
│ Forward Kinematics│ ← Estimates robot velocity
└──────────────────┘
```

### Task Interaction Diagram

```
┌────────────────────────────────────────────────────────────┐
│                   FreeRTOS Scheduler                       │
└────────────────────────────────────────────────────────────┘
         │              │              │              │
         ↓              ↓              ↓              ↓
    ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
    │ Sensor  │   │  Move   │   │   IK    │   │ Control │
    │ Task    │   │ Task    │   │  Task   │   │  Task   │
    │ (P=6)   │   │ (P=4)   │   │ (P=5)   │   │ (P=3)   │
    └────┬────┘   └────┬────┘   └────┬────┘   └────┬────┘
         │             │             │             │
         │             │ Queue       │ Queue       │
         │   Mutex     ↓             ↓    Mutex    │ Mutex
         │        ┌────────┐    ┌────────┐         │
         │        │velocity│    │ wheel  │         │
         │        │command │    │targets │         │
         │        └────────┘    └────────┘         │
         ├──────────────────────┬──────────────────┤
         ↓                      ↓                  ↓
    ┌──────────────────────────────────────────────────┐
    │          Shared Resources (Mutex-Protected)       │
    │  • sensor_data  • robot_estimated  • pid[]       │
    └──────────────────────────────────────────────────┘
```

**Communication Patterns:**
- **Trajectory → IK:** Queue-based (one-way, producer-consumer)
- **IK → Control:** Queue-based (one-way, decoupled setpoints)
- **Sensor → Control:** Mutex-protected shared memory (bi-directional read)
- **Sensor → Trajectory:** Mutex-protected shared memory (for logging)

**Design Rationale:**
- Queues for unidirectional data flow (reduces contention)
- Mutexes only for truly shared resources (ADC, PID array)
- Timeout-based acquisition (prevents deadlocks)
- Task priorities ordered by criticality

---

## Hardware Components

### ESP32-S3 Microcontroller

- **CPU:** Dual-core Xtensa LX7 @ 240 MHz
- **RAM:** 512 KB SRAM
- **Flash:** External (size depends on module)
- **Peripherals:** LEDC PWM, ADC, I2C, UART

### Motors

- **Type:** Brushless DC (SKYWALKER series)
- **Quantity:** 3
- **Control:** PWM speed + PWM direction signals
- **Configuration:**
  - Motor 0: GPIO 7 (speed), GPIO 8 (reverse)
  - Motor 1: GPIO 15 (speed), GPIO 3 (reverse)
  - Motor 2: GPIO 16 (speed), GPIO 46 (reverse)

### Encoders

- **Type:** AS5600 12-bit magnetic rotary encoders
- **Quantity:** 3 (one per motor)
- **Interface:** Analog output (via ADC)
- **Configuration:**
  - Encoder 0: GPIO 4 (analog input)
  - Encoder 1: GPIO 5 (analog input)
  - Encoder 2: GPIO 6 (analog input)

### IMU (Optional)

- **Type:** BNO055 9-DOF IMU
- **Interface:** I2C
- **Pins:** GPIO 17 (SDA), GPIO 18 (SCL)
- **Status:** Currently disabled in firmware

---

## Software Architecture

### Directory Structure

```
firmware/
├── main/
│   ├── main.c              # Application entry point
│   ├── init.c              # Hardware initialization
│   └── init.h              # Initialization interface
├── tasks/
│   ├── task_read_sensors.c      # Sensor reading + filtering
│   ├── task_control.c           # Motor PID control
│   └── task_inverse_kinematics.c # IK computation
├── src/
│   ├── motor.c             # Motor driver implementation
│   ├── pid.c               # PID controller implementation
│   ├── kinematics.c        # Forward/inverse kinematics
│   ├── as5600.c            # Encoder driver
│   └── bno055.c            # IMU driver
├── include/
│   ├── motor.h             # Motor driver interface
│   ├── pid.h               # PID controller interface
│   ├── kinematics.h        # Kinematics interface
│   ├── as5600.h            # Encoder interface
│   └── bno055.h            # IMU interface
├── utils/
│   ├── types_utils.h       # Common type definitions
│   ├── config_utils.h      # Configuration constants
│   └── gpio_utils.h        # GPIO pin assignments
└── CMakeLists.txt          # Build configuration
```

### Module Layering

```
┌─────────────────────────────────────────┐
│        Application Layer (main.c)       │
│  • Task creation • Initialization       │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│         Task Layer (tasks/)             │
│  • Sensor reading • Control • IK        │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│      Algorithm Layer (src/)             │
│  • PID • Kinematics • Filtering         │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│       Driver Layer (src/)               │
│  • Motor • Encoder • IMU                │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│         HAL Layer (ESP-IDF)             │
│  • LEDC • ADC • I2C • GPIO              │
└─────────────────────────────────────────┘
```

---

## Task Descriptions

### 1. Sensor Reading Task (`task_read_sensors`)

**Purpose:** Reads encoder angles and computes filtered angular velocities

**Characteristics:**
- **Priority:** 6 (highest)
- **Period:** 2 ms
- **Stack:** 4096 bytes
- **Core:** Any (managed by FreeRTOS scheduler)

**Responsibilities:**
1. Read encoder angles via ADC (thread-safe with `g_adc_mutex` and timeout)
2. Compute angular velocities using finite difference with wrap-around handling
3. Apply Kalman filtering for noise reduction
4. Estimate robot velocity using forward kinematics
5. Update shared data structures (thread-safe with timeouts)

**Communication:**
- **Outputs:** `g_sensor_data` (via mutex), `g_robot_estimated` (via mutex)
- **Timeout:** 10ms for ADC, 5ms for data updates
- **Error Handling:** Logs warnings on timeout, continues with previous values

**Rationale:** Highest priority ensures timely sensor feedback for control loops. Timeout handling prevents deadlocks.

---

### 2. Trajectory Generation Task (`task_move_trajectory`)

**Purpose:** Generates desired robot velocity commands

**Characteristics:**
- **Priority:** 4 (medium)
- **Period:** 20 ms
- **Stack:** 2048 bytes

**Responsibilities:**
1. Compute desired velocities (currently: circular trajectory)
2. Send velocity commands to IK task via queue
3. Read estimated velocity for logging (non-critical)

**Communication:**
- **Outputs:** `g_velocity_command_queue` (2-item queue)
- **Inputs:** `g_robot_estimated` (via mutex, for logging only)
- **Timeout:** 5ms for queue send, continues on failure

**Rationale:** Queue-based output decouples trajectory generation from IK computation. Non-blocking design prevents starvation.

---

### 3. Inverse Kinematics Task (`task_inverse_kinematics`)

**Purpose:** Converts robot velocity commands to wheel speed targets

**Characteristics:**
- **Priority:** 5 (high)
- **Period:** 10 ms
- **Stack:** 4096 bytes

**Responsibilities:**
1. Receive desired robot velocity from queue (non-blocking)
2. Compute wheel speeds using inverse kinematic equations
3. Send wheel targets to control task via queue
4. Update PID controller setpoints (thread-safe)

**Communication:**
- **Inputs:** `g_velocity_command_queue` (receives from Trajectory task)
- **Outputs:** `g_wheel_target_queue` (sends to Control task)
- **Shared:** `g_pid_mutex` (for setpoint updates)
- **Timeout:** 5ms for queue receive, 10ms for PID mutex

**Rationale:** Higher priority than trajectory but lower than sensor ensures proper execution order. Queue-based I/O provides decoupling.

---

### 4. Motor Control Task (`task_control`)

**Purpose:** Implements PID feedback control for each motor

**Characteristics:**
- **Priority:** 3 (medium-low)
- **Period:** 2 ms
- **Stack:** 4096 bytes

**Responsibilities:**
1. Receive target wheel speeds from IK task via queue
2. Read current encoder velocities from sensor data
3. Compute PID outputs (error = setpoint - measured)
4. Apply motor commands via PWM

**Communication:**
- **Inputs:** `g_wheel_target_queue` (receives from IK task), `g_sensor_data` (via mutex)
- **Shared:** `g_pid_mutex` (for PID computation)
- **Timeout:** 1ms for queue, 5ms for mutexes
- **Error Handling:** Uses previous values on timeout, logs warnings

**Rationale:** Lower priority acceptable because PID maintains last setpoint. Fast period ensures stable control. Timeout handling provides robustness.

---

## Inter-Task Communication

### Communication Architecture Overview

This system uses a **hybrid communication model** combining FreeRTOS queues and mutexes:

**Queues (One-Way Data Flow):**
- Used for producer-consumer relationships
- Non-blocking with timeouts
- Only latest data matters (size=2, allows buffering)

**Mutexes (Shared Resources):**
- Used for resources accessed by multiple tasks
- Timeout-based acquisition (no portMAX_DELAY)
- Prevents deadlocks and provides diagnostic warnings

### Synchronization Primitives

| Primitive | Type | Purpose | Users | Timeout |
|-----------|------|---------|-------|---------|
| `g_velocity_command_queue` | Queue (size=2) | Velocity commands | Trajectory→IK | 5ms |
| `g_wheel_target_queue` | Queue (size=2) | Wheel targets | IK→Control | 1ms |
| `g_sensor_data_mutex` | Mutex | Sensor readings | Sensor(W), Control(R) | 5ms |
| `g_estimated_data_mutex` | Mutex | Robot velocity estimate | Sensor(W), Trajectory(R) | 5ms |
| `g_pid_mutex` | Mutex | PID controller array | IK(W), Control(R) | 5-10ms |
| `g_adc_mutex` | Mutex | Shared ADC hardware | Sensor task only | 10ms |

### Data Flow Diagram

```
┌──────────────┐
│  Trajectory  │
│    Task      │
└──────┬───────┘
       │ velocity_t via QUEUE
       ↓
┌──────────────┐
│   IK Task    │
└──────┬───────┘
       │ wheel_speeds_t via QUEUE
       │
       ↓              ┌──────────────┐
┌──────────────┐     │   Sensor     │
│   Control    │◄────┤    Task      │ sensor_data via MUTEX
│    Task      │     └──────────────┘
└──────┬───────┘            │
       │                    │ estimated velocity via MUTEX
       │                    ↓
       │             ┌──────────────┐
       └────────────►│  Trajectory  │ (for logging)
         PID mutex   │    Task      │
                     └──────────────┘
```

### Design Decisions and Rationale

**1. Why Queues for Trajectory→IK and IK→Control?**

**Decision:** Use FreeRTOS queues instead of mutex-protected globals

**Rationale:**
- **One-way data flow:** Producer-consumer pattern, no need for bidirectional access
- **Decoupling:** Tasks don't block each other waiting for mutex
- **Natural buffering:** Queue size of 2 allows one item being processed while next arrives
- **Latest-value semantics:** If queue fills, use `xQueueOverwrite()` to replace oldest
- **Lower contention:** Tasks interact via kernel queue primitive, not shared memory

**Alternative Rejected:** Mutexes would cause unnecessary blocking and tighter coupling

---

**2. Why Mutexes for Sensor Data and PID Array?**

**Decision:** Continue using mutexes for `g_sensor_data`, `g_pid[]`, and `g_robot_estimated`

**Rationale:**
- **Multiple readers:** Control task needs sensor data; trajectory task needs estimates
- **State representation:** Not message-passing, but shared system state
- **Atomic updates:** Sensor data has multiple fields that must be updated together
- **Small critical sections:** Quick copy operations, low hold time

---

**3. Why Timeout-Based Mutex Acquisition?**

**Decision:** Replace `portMAX_DELAY` with timeouts (5-10ms)

**Rationale:**
- **Deadlock prevention:** System can detect and log stuck tasks
- **Graceful degradation:** Tasks continue with stale data rather than blocking forever
- **Diagnostic visibility:** Timeout warnings indicate synchronization problems
- **Real-time guarantee:** High-priority tasks won't be blocked indefinitely

**Implementation Pattern:**
```c
if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Critical section
    xSemaphoreGive(mutex);
} else {
    ESP_LOGW(TAG, "Mutex timeout - continuing with previous data");
    // Graceful degradation
}
```

---

**4. Task Priority Assignment**

**Decision:**
- Sensor Task: 6 (highest)
- IK Task: 5
- Trajectory Task: 4
- Control Task: 3

**Rationale:**
- **Sensor (6):** Must sample encoders at precise intervals for accurate velocity estimation
- **IK (5):** Must process commands before control loop to update setpoints
- **Trajectory (4):** Can tolerate jitter; generates commands at lower rate (20ms)
- **Control (3):** Runs fast (2ms) but can be preempted; maintains last setpoint if delayed

**Validation:** No priority inversion observed; task watermarks show sufficient stack

---

### Error Handling Strategy

**All tasks implement:**
1. **Timeout-based resource acquisition** (no infinite blocking)
2. **Graceful degradation** (use previous values on failure)
3. **Diagnostic logging** (ESP_LOGW for timeouts, ESP_LOGE for critical errors)
4. **Validation at startup** (check all mutexes/queues created successfully)

**Example from Control Task:**
```c
if (xQueueReceive(g_wheel_target_queue, &wheel_targets, pdMS_TO_TICKS(1)) != pdTRUE) {
    // No new target - use previous wheel_targets value
    no_target_count++;
    if (no_target_count % 500 == 0) {
        ESP_LOGW(TAG, "No new targets for %d cycles", no_target_count);
    }
}
// Task continues with last known good values
```

### Concurrency Safety Guarantees

**Data Race Prevention:**
- ✅ All shared variables accessed via synchronization primitives
- ✅ No direct global reads/writes without protection
- ✅ Local copies made inside critical sections

**Deadlock Prevention:**
- ✅ No nested mutex acquisition (each task acquires one mutex at a time)
- ✅ Timeout-based waits (no infinite blocking)
- ✅ Queue operations are non-blocking or short-timeout

**Priority Inversion Mitigation:**
- ✅ FreeRTOS mutexes use priority inheritance by default
- ✅ Task priorities configured to minimize blocking chains
- ✅ Critical sections kept short (copy data and release)

**Testing Recommendations:**
- Monitor task watermarks: `vTaskGetRunTimeStats()`
- Check mutex hold times during stress testing
- Validate queue depths never consistently fill
- Run extended tests (>30 min) to detect rare race conditions

---

## Module Descriptions

### Motor Module (`motor.h`/`motor.c`)

**Purpose:** Brushless motor control using ESP32 LEDC PWM

**Key Functions:**
- `motor_init()`: Initialize PWM channels
- `motor_set_speed()`: Set signed speed (-100% to +100%)
- `motor_stop()`: Stop motor
- `motor_calibration()`: Calibration sequence

**Features:**
- Bidirectional control (forward/reverse PWM signals)
- Speed scaling with configurable limits
- Thread-unsafe: caller must synchronize

---

### PID Module (`pid.h`/`pid.c`)

**Purpose:** Generic PID controller implementation

**Key Functions:**
- `pid_new_control_block()`: Create PID instance
- `pid_compute()`: Compute control output
- `pid_update_set_point()`: Change target setpoint
- `pid_update_parameters()`: Retune controller
- `pid_reset_block()`: Clear error accumulators

**Features:**
- Positional and incremental modes
- Anti-windup protection
- Derivative filtering
- Tunable via UART (optional)

---

### Kinematics Module (`kinematics.h`/`kinematics.c`)

**Purpose:** Forward and inverse kinematic transformations

**Key Functions:**
- `compute_inverse_kinematics()`: Robot velocity → Wheel speeds
- `compute_forward_kinematics()`: Wheel speeds → Robot velocity

**Implementation:**
- Inverse: Direct computation using Jacobian
- Forward: Pseudo-inverse matrix (computed once, cached)

**Thread-Safety:** Reentrant (no shared state after initialization)

---

## Build and Flash Instructions

### Prerequisites

1. **Install ESP-IDF:**
   ```bash
   # Follow official ESP-IDF installation guide
   # https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/
   
   # Recommended version: v5.1 or later
   ```

2. **Set up environment:**
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

### Build Process

1. **Configure project:**
   ```bash
   cd firmware
   idf.py set-target esp32s3
   idf.py menuconfig  # Optional: adjust settings
   ```

2. **Build firmware:**
   ```bash
   idf.py build
   ```

3. **Flash to device:**
   ```bash
   idf.py -p /dev/ttyUSB0 flash
   ```

4. **Monitor output:**
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   ```

5. **Flash and monitor (combined):**
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

### Troubleshooting Build Issues

- **Missing headers:** Ensure all submodules are initialized
- **Linker errors:** Check CMakeLists.txt includes all source files
- **Flash errors:** Verify correct serial port and permissions

---

## Configuration

### Tuning Parameters

All configuration constants are centralized in `utils/config_utils.h`:

#### Robot Geometry
```c
#define ROBOT_BODY_RADIUS 0.08f      // Chassis radius (m)
#define ROBOT_WHEEL_RADIUS 0.03f     // Wheel radius (m)
#define ROBOT_WHEEL_0_OFFSET (M_PI/6.0f)  // Wheel angles (rad)
```

#### Motor Limits
```c
#define MOTOR_MAX_SPEED_PERCENT 12   // Maximum speed (%)
#define MOTOR_MIN_SPEED_PERCENT 5    // Minimum speed (%)
```

#### PID Tuning
```c
#define PID_MOTOR_KP 0.1f            // Proportional gain
#define PID_MOTOR_KI 0.006f          // Integral gain
#define PID_MOTOR_KD 0.0f            // Derivative gain
#define PID_MOTOR_MAX_OUTPUT 80.0f   // Output limit
```

#### Task Timing
```c
#define SENSOR_TASK_PERIOD_MS 2      // Sensor loop period
#define CONTROL_TASK_PERIOD_MS 2     // Control loop period
#define KINEMATICS_TASK_PERIOD_MS 10 // IK loop period
```

#### Kalman Filter
```c
#define SENSOR_KALMAN_Q 0.001f       // Process noise
#define SENSOR_KALMAN_R 10.0f        // Measurement noise
```

### GPIO Pin Assignments

Pin mappings are defined in `utils/gpio_utils.h`. Update these if hardware changes:

```c
// Motors
#define GPIO_MOTOR_0_SIGNAL_OUT_PWM 7
#define GPIO_MOTOR_0_REVERSE_OUT_PWM 8
// ... (see file for complete list)

// Encoders
#define GPIO_ENCODER_0_IN_ANALOG 4
// ... (see file for complete list)
```

---

## Coding Conventions

### Naming Conventions

This codebase follows **snake_case** naming for all identifiers:

| Item | Convention | Example |
|------|-----------|---------|
| Variables | `snake_case` | `sensor_data`, `wheel_speeds` |
| Functions | `snake_case` | `motor_set_speed()`, `pid_compute()` |
| Types | `snake_case_t` | `velocity_t`, `motor_brushless_t` |
| Macros | `SCREAMING_SNAKE_CASE` | `MOTOR_MAX_SPEED_PERCENT` |
| Enums | `snake_case` (values SCREAMING) | `pid_calculate_type_t`, `PID_CAL_TYPE_POSITIONAL` |
| Global variables | `g_prefix` | `g_motor[3]`, `g_sensor_data` |
| Static variables | `s_prefix` or no prefix | `s_kalman_filters[]` |

### File Organization

- **Headers (`.h`)**: Interface declarations, type definitions, documentation
- **Sources (`.c`)**: Implementation, internal functions (marked `static`)

### Documentation Standards

- **All public functions:** Doxygen-style comments
- **Parameters:** `@param[in]`, `@param[out]`, `@param[in,out]`
- **Return values:** `@return` with possible values
- **Thread-safety:** Always documented

Example:
```c
/**
 * @brief Brief description
 * 
 * Detailed description with usage notes.
 * 
 * @param[in] input Input parameter description
 * @param[out] output Output parameter description
 * @return Status code: 0 on success, negative on error
 * 
 * Thread-safety: Not thread-safe. Use mutex for shared access.
 */
int function_name(int input, int *output);
```

### Error Handling

- **Return codes:** Use `int` return values (0 = success, negative = error)
- **Logging:** Use ESP-IDF logging (`ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE`)
- **Null checks:** Always validate pointer parameters

---

## Development Workflow

### Adding a New Task

1. Create task file in `tasks/` directory
2. Declare task function in `main.c`
3. Create task in `app_main()` with appropriate priority
4. Document task characteristics in this README
5. Add any new shared data structures
6. Create/use appropriate mutexes for synchronization

### Adding a New Module

1. Create header in `include/`
2. Create implementation in `src/`
3. Add to CMakeLists.txt
4. Document module in this README
5. Follow snake_case naming conventions

### Tuning PID Controllers

**Method 1: Recompile (Permanent)**
1. Edit `config_utils.h`
2. Rebuild and flash

**Method 2: UART Runtime (Temporary)**
1. Enable UART tasks in `main.c` (uncomment)
2. Send JSON command: `{"default":"10 6 0 50"}`
   - Format: `kp ki kd setpoint` (in hundredths)
3. Parameters reset on reboot

---

## Troubleshooting

### Motors Not Spinning

- ✅ Check power supply to motors
- ✅ Verify PWM signals with oscilloscope
- ✅ Check `MOTOR_MIN_SPEED_PERCENT` is appropriate for ESCs
- ✅ Run motor calibration sequence
- ✅ Check GPIO pin assignments match hardware

### Erratic Sensor Readings

- ✅ Verify encoder magnets are properly aligned
- ✅ Increase Kalman filter measurement noise (`SENSOR_KALMAN_R`)
- ✅ Check ADC reference voltage
- ✅ Inspect encoder wiring for noise

### Robot Not Following Trajectory

- ✅ Check PID gains are reasonable
- ✅ Verify kinematics constants (wheel radius, body radius)
- ✅ Inspect wheel angle offsets
- ✅ Log estimated vs. commanded velocities
- ✅ Check for motor saturation (PID output at limits)

### Build Errors

- ✅ Ensure ESP-IDF environment is sourced
- ✅ Check ESP-IDF version compatibility (v5.1+)
- ✅ Clean build: `idf.py fullclean && idf.py build`
- ✅ Verify all files are added to CMakeLists.txt

### Task Starvation

- ✅ Review task priorities (higher number = higher priority)
- ✅ Check for infinite loops without delays
- ✅ Monitor stack usage: `idf.py monitor` (stack watermarks)
- ✅ Ensure mutexes are always released

---

## License

[Specify license here - e.g., MIT, GPL, proprietary]

## Contributors

- [Your name/team]

## References

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [AS5600 Datasheet](https://ams.com/as5600)
- [RoboCup Rules](https://www.robocup.org/)

---

**Last Updated:** November 2025  
**Firmware Version:** 1.0  
**Maintained by:** [Your team name]
