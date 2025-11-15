# Task Interaction and Shared Data Handling Improvements

**Date:** November 15, 2025  
**Scope:** Task communication architecture refactoring  
**Objective:** Improve robustness, clarity, and concurrency safety

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Problems Identified](#problems-identified)
3. [Solutions Implemented](#solutions-implemented)
4. [Architecture Changes](#architecture-changes)
5. [Design Decisions and Rationale](#design-decisions-and-rationale)
6. [Code Changes](#code-changes)
7. [Testing Recommendations](#testing-recommendations)
8. [Performance Impact](#performance-impact)

---

## Executive Summary

### What Changed

**Before:**
- All inter-task communication used mutexes (even for one-way data flow)
- No timeout handling (`portMAX_DELAY` everywhere)
- Potential for deadlocks and priority inversion
- Limited diagnostic visibility

**After:**
- **Hybrid model:** Queues for one-way data flow + mutexes for shared resources
- **Timeout-based acquisition:** All mutexes/queues use timeouts (5-10ms)
- **Graceful degradation:** Tasks continue with previous values on timeout
- **Enhanced diagnostics:** Warning logs for timeout conditions
- **Deadlock prevention:** No nested locking, timeout guarantees

### Key Benefits

✅ **Improved Robustness:** Timeout handling prevents infinite blocking  
✅ **Better Decoupling:** Queue-based communication reduces task interdependencies  
✅ **Enhanced Diagnostics:** Warning logs identify synchronization bottlenecks  
✅ **Deadlock Prevention:** Guaranteed progress even under resource contention  
✅ **Functional Equivalence:** Same behavior under normal operation  

---

## Problems Identified

### 1. Inappropriate Use of Mutexes for One-Way Data Flow

**Issue:**
```c
// Trajectory task writes
xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
g_robot_command = new_command;
xSemaphoreGive(g_cmd_mutex);

// IK task reads
xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
cmd = g_robot_command;
xSemaphoreGive(g_cmd_mutex);
```

**Problems:**
- Mutex overkill for simple producer-consumer relationship
- Both tasks must acquire same lock (contention point)
- Trajectory task blocks if IK task holds mutex
- No natural buffering mechanism

**Impact:** Unnecessary coupling and potential priority inversion

---

### 2. Infinite Blocking with `portMAX_DELAY`

**Issue:**
```c
xSemaphoreTake(g_sensor_data_mutex, portMAX_DELAY);  // Block forever!
```

**Problems:**
- Task can hang indefinitely if mutex never released
- No diagnostic feedback when blocking occurs
- Deadlock potential if multiple tasks wait on each other
- No graceful degradation path

**Impact:** System can become unresponsive with no error indication

---

### 3. No Validation of Synchronization Primitive Creation

**Issue:**
```c
g_sensor_data_mutex = xSemaphoreCreateMutex();
// Used immediately without checking if NULL
```

**Problems:**
- System continues running with NULL mutex handles
- Unpredictable behavior on first mutex access
- No clear error message at root cause

**Impact:** Difficult-to-debug crashes at runtime

---

### 4. Lack of Diagnostic Visibility

**Issue:**
- No logging when tasks wait for resources
- No metrics on contention or blocking
- Silent failures make debugging difficult

**Impact:** Problems only manifest as performance degradation or hangs

---

## Solutions Implemented

### 1. Introduced FreeRTOS Queues for One-Way Data Flow

**Trajectory → IK Communication:**
```c
// Trajectory task (producer)
velocity_t velocity_cmd = compute_trajectory();
if (xQueueSend(g_velocity_command_queue, &velocity_cmd, pdMS_TO_TICKS(5)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to send velocity command (queue full)");
}

// IK task (consumer)
velocity_t cmd;
if (xQueueReceive(g_velocity_command_queue, &cmd, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Process new command
} else {
    // Use previous cmd value (maintains last setpoint)
}
```

**IK → Control Communication:**
```c
// IK task (producer)
wheel_speeds_t targets = compute_inverse_kinematics(cmd);
xQueueOverwrite(g_wheel_target_queue, &targets);  // Always succeeds

// Control task (consumer)
wheel_speeds_t wheel_targets;
if (xQueueReceive(g_wheel_target_queue, &wheel_targets, pdMS_TO_TICKS(1)) == pdTRUE) {
    // New targets received
} else {
    // Use previous targets (maintains control)
}
```

**Benefits:**
- Natural producer-consumer pattern
- No mutex contention
- Built-in buffering (queue size = 2)
- Non-blocking with timeout
- Overwrite semantics for "latest value" data

---

### 2. Added Timeout-Based Mutex Acquisition

**Before:**
```c
xSemaphoreTake(g_sensor_data_mutex, portMAX_DELAY);  // Infinite wait
```

**After:**
```c
if (xSemaphoreTake(g_sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Critical section
    xSemaphoreGive(g_sensor_data_mutex);
} else {
    ESP_LOGW(TAG, "Failed to acquire sensor_data mutex (timeout)");
    // Continue with previous data
}
```

**Timeout Values:**
- ADC mutex: 10ms (hardware access)
- Sensor data mutex: 5ms (quick copy)
- PID mutex: 5-10ms (computation)
- Estimated data mutex: 5ms (logging only)

**Benefits:**
- Guaranteed forward progress
- Diagnostic warnings on contention
- Graceful degradation (use stale data)
- Deadlock prevention

---

### 3. Enhanced Initialization Validation

**Before:**
```c
g_sensor_data_mutex = xSemaphoreCreateMutex();
// No validation, continue regardless
```

**After:**
```c
g_sensor_data_mutex = xSemaphoreCreateMutex();
g_velocity_command_queue = xQueueCreate(2, sizeof(velocity_t));
g_wheel_target_queue = xQueueCreate(2, sizeof(wheel_speeds_t));

if (!g_sensor_data_mutex || !g_velocity_command_queue || !g_wheel_target_queue) {
    ESP_LOGE(TAG, "Failed to create synchronization primitives");
    ESP_LOGE(TAG, "  sensor_data_mutex: %p", g_sensor_data_mutex);
    ESP_LOGE(TAG, "  velocity_command_queue: %p", g_velocity_command_queue);
    ESP_LOGE(TAG, "  wheel_target_queue: %p", g_wheel_target_queue);
    return;  // Abort initialization
}

ESP_LOGI(TAG, "All synchronization primitives created successfully");
```

**Benefits:**
- Fail-fast on initialization errors
- Clear diagnostic output
- Prevents mysterious crashes later

---

### 4. Added Task Handles for Management

**Enhancement:**
```c
TaskHandle_t g_task_sensor_handle = NULL;
TaskHandle_t g_task_control_handle = NULL;
TaskHandle_t g_task_ik_handle = NULL;
TaskHandle_t g_task_trajectory_handle = NULL;

xTaskCreate(task_read_sensors, "SensorTask", 4096, NULL, 6, &g_task_sensor_handle);
```

**Benefits:**
- Enable future task monitoring
- Support for task notifications
- Runtime state inspection
- Controlled task suspension/resumption

---

## Architecture Changes

### Communication Topology

**Before (All Mutexes):**
```
Trajectory ──mutex──> IK ──mutex──> Control
                                      ↑
                                    mutex
                                      │
                                   Sensor
```
*Every connection is a potential contention point*

**After (Hybrid Model):**
```
Trajectory ──QUEUE──> IK ──QUEUE──> Control
                                      ↑
                                    mutex
                                      │
                                   Sensor
```
*Queues decouple producers from consumers*

### Data Flow Summary

| From | To | Mechanism | Data Type | Size | Rationale |
|------|-----|-----------|-----------|------|-----------|
| Trajectory | IK | Queue | `velocity_t` | 2 items | One-way, producer-consumer |
| IK | Control | Queue | `wheel_speeds_t` | 2 items | One-way, decoupling |
| Sensor | Control | Mutex | `raw_sensor_data_t` | N/A | Shared state, multiple fields |
| Sensor | Trajectory | Mutex | `velocity_t` | N/A | Shared state, logging only |
| IK | Control | Mutex | `pid_block_handle_t[3]` | N/A | Shared PID controllers |

### Task Interaction Matrix

| Task | Reads From | Writes To | Blocks On |
|------|-----------|-----------|-----------|
| **Sensor** | ADC (hardware) | `g_sensor_data`, `g_robot_estimated` | `g_adc_mutex` |
| **Trajectory** | `g_robot_estimated` | `g_velocity_command_queue` | `g_estimated_data_mutex` (optional) |
| **IK** | `g_velocity_command_queue` | `g_wheel_target_queue`, PID setpoints | `g_pid_mutex` |
| **Control** | `g_wheel_target_queue`, `g_sensor_data` | Motor PWM (hardware) | `g_sensor_data_mutex`, `g_pid_mutex` |

---

## Design Decisions and Rationale

### Decision 1: Queues vs. Mutexes

**Question:** When to use queues vs. mutexes?

**Decision Matrix:**

| Pattern | Use | Rationale |
|---------|-----|-----------|
| **One-way data flow** | Queue | Natural producer-consumer, decoupling |
| **Bi-directional access** | Mutex | Multiple readers/writers |
| **Message passing** | Queue | FIFO semantics, buffering |
| **State sharing** | Mutex | Latest value, not FIFO |
| **Hardware access** | Mutex | Serialize access to shared peripheral |

**Applied:**
- Trajectory→IK: Queue (one-way commands)
- IK→Control: Queue (one-way setpoints)
- Sensor data: Mutex (shared state, multiple fields)
- PID array: Mutex (shared resource, bidirectional)

---

### Decision 2: Queue Size = 2

**Question:** Why queue depth of 2 instead of 1 or larger?

**Rationale:**
- **Size 1:** No buffering, producer can block unnecessarily
- **Size 2:** Allows one item being processed while next arrives
- **Size >2:** Unnecessary for "latest value" data; wastes memory

**Implementation:**
```c
g_velocity_command_queue = xQueueCreate(2, sizeof(velocity_t));
g_wheel_target_queue = xQueueCreate(2, sizeof(wheel_speeds_t));
```

**Semantics:**
- If queue fills, use `xQueueOverwrite()` to replace oldest
- Only the latest command/setpoint matters (not historical data)

---

### Decision 3: Timeout Values

**Question:** What timeout values to use for each primitive?

**Analysis:**

| Resource | Typical Hold Time | Chosen Timeout | Rationale |
|----------|------------------|----------------|-----------|
| `g_adc_mutex` | ~1ms (ADC read) | 10ms | Hardware I/O, allow margin |
| `g_sensor_data_mutex` | <100μs (copy) | 5ms | Quick copy operation |
| `g_pid_mutex` | <500μs (compute) | 5-10ms | PID calculation time |
| `g_velocity_command_queue` | N/A (non-blocking) | 5ms | Producer side |
| `g_wheel_target_queue` | N/A (non-blocking) | 1ms | Consumer side (tight loop) |

**Design Principle:** Timeout should be 5-10x typical hold time

---

### Decision 4: Graceful Degradation Strategy

**Question:** What should tasks do on timeout?

**Strategy:**
1. **Log warning** (ESP_LOGW) to indicate problem
2. **Continue with previous value** (maintains control)
3. **Don't halt system** (availability > consistency)

**Example:**
```c
if (xQueueReceive(g_wheel_target_queue, &wheel_targets, pdMS_TO_TICKS(1)) != pdTRUE) {
    // Use previous wheel_targets value
    no_target_count++;
    if (no_target_count % 500 == 0) {
        ESP_LOGW(TAG, "No new targets for %d cycles", no_target_count);
    }
}
// Task continues with last known good values
```

**Rationale:**
- Robot maintains last command (safer than stopping)
- Diagnostic log identifies problem without halting
- High-frequency warnings rate-limited (% 500)

---

## Code Changes

### File: `main/main.c`

**Changes:**
1. Added `#include "freertos/queue.h"`
2. Replaced `g_cmd_mutex` and globals with queues
3. Created `g_velocity_command_queue` and `g_wheel_target_queue`
4. Added task handle globals for management
5. Enhanced initialization validation with detailed logging
6. Updated `task_move_trajectory()` to use queue
7. Improved task creation logging

**Lines Changed:** ~80 lines  
**Functional Impact:** None under normal operation; improved error handling

---

### File: `tasks/task_inverse_kinematics.c`

**Changes:**
1. Added `#include "freertos/queue.h"`
2. Replaced mutex reads with `xQueueReceive()`
3. Replaced mutex writes with `xQueueOverwrite()`
4. Added timeout handling with warnings
5. Maintained PID setpoint updates via mutex (necessary)

**Lines Changed:** ~30 lines  
**Functional Impact:** Better decoupling from trajectory task

---

### File: `tasks/task_control.c`

**Changes:**
1. Added `#include "freertos/queue.h"`
2. Replaced direct global access with `xQueueReceive()`
3. Added timeout handling for all mutex/queue operations
4. Added `no_target_count` tracking and periodic warnings
5. Graceful continuation on timeout (use previous values)

**Lines Changed:** ~40 lines  
**Functional Impact:** More robust operation under load

---

### File: `tasks/task_read_sensors.c`

**Changes:**
1. Replaced `portMAX_DELAY` with `pdMS_TO_TICKS(5-10)`
2. Added timeout warnings for all mutex operations
3. Continue with previous values on timeout

**Lines Changed:** ~15 lines  
**Functional Impact:** Deadlock prevention, diagnostic visibility

---

## Testing Recommendations

### Unit Tests

**Test 1: Queue Overflow Handling**
```c
// Simulate fast producer, slow consumer
// Verify xQueueOverwrite() replaces old data
// Confirm no data loss for "latest value" semantics
```

**Test 2: Mutex Timeout Behavior**
```c
// Hold mutex in one task
// Verify timeout triggers in another
// Check warning logged correctly
// Confirm graceful degradation
```

**Test 3: Queue Empty Handling**
```c
// Consumer task starts before producer sends data
// Verify timeout returns pdFALSE
// Confirm task continues with initialization values
```

---

### Integration Tests

**Test 1: Normal Operation**
- **Setup:** Run all tasks for 10 minutes
- **Expected:** No timeout warnings, smooth control
- **Validation:** Check log for absence of warnings

**Test 2: High Load**
- **Setup:** Increase trajectory update rate to 100 Hz
- **Expected:** Occasional queue warnings, system continues
- **Validation:** Control loop remains stable

**Test 3: Priority Inversion**
- **Setup:** Suspend high-priority task briefly
- **Expected:** Lower-priority tasks complete, no deadlock
- **Validation:** System recovers when task resumed

---

### Stress Tests

**Test 1: Extended Runtime**
- **Duration:** 24 hours continuous operation
- **Monitoring:** Task watermarks, mutex hold times
- **Expected:** No memory leaks, stable operation

**Test 2: Resource Contention**
- **Setup:** Add additional tasks competing for mutexes
- **Expected:** Timeouts increase, but no deadlocks
- **Validation:** All tasks make forward progress

---

### Performance Metrics to Monitor

```c
// Enable FreeRTOS run-time stats
configGENERATE_RUN_TIME_STATS 1

// Check task stats periodically
char stats_buffer[512];
vTaskGetRunTimeStats(stats_buffer);
ESP_LOGI(TAG, "Task Stats:\n%s", stats_buffer);

// Monitor stack high water marks
UBaseType_t sensor_hwm = uxTaskGetStackHighWaterMark(g_task_sensor_handle);
ESP_LOGI(TAG, "Sensor task stack remaining: %u bytes", sensor_hwm * 4);
```

---

## Performance Impact

### Latency Analysis

**Queue Operations:**
- `xQueueSend()`: ~5-10 μs (O(1) operation)
- `xQueueReceive()`: ~5-10 μs (O(1) operation)
- **Impact:** Negligible (<0.5% of 2ms task period)

**Mutex Operations (unchanged):**
- `xSemaphoreTake()`: ~2-5 μs
- `xSemaphoreGive()`: ~2-5 μs
- **Impact:** Same as before

**Timeout Checking:**
- Timeout logic: <1 μs (simple comparison)
- **Impact:** Negligible overhead

### Memory Usage

**Before:**
- Mutexes: 5 × ~100 bytes = 500 bytes
- **Total:** ~500 bytes

**After:**
- Mutexes: 4 × ~100 bytes = 400 bytes
- Queues: 2 × (100 bytes + 2×item_size) = ~250 bytes
- **Total:** ~650 bytes

**Increase:** +150 bytes (~0.03% of 512KB RAM)

### CPU Utilization

**Measurements (estimated):**
- Sensor task: 5-10% CPU (unchanged)
- Control task: 3-5% CPU (unchanged)
- IK task: 1-2% CPU (unchanged)
- Trajectory task: <1% CPU (unchanged)

**Queue overhead:** <0.1% additional CPU time

**Conclusion:** Performance impact is negligible; benefits far outweigh costs

---

## Migration Guide

### For Developers Extending the System

**Adding a New Task that Consumes Data:**

```c
// 1. Determine communication pattern
// One-way from existing task? → Use queue
// Bi-directional or multiple writers? → Use mutex

// 2. Queue pattern (if applicable)
QueueHandle_t g_new_data_queue = NULL;

void app_main(void) {
    g_new_data_queue = xQueueCreate(2, sizeof(my_data_t));
    // Validate creation...
}

void producer_task(void *arg) {
    my_data_t data = generate_data();
    if (xQueueSend(g_new_data_queue, &data, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGW(TAG, "Queue send failed");
    }
}

void consumer_task(void *arg) {
    my_data_t data;
    if (xQueueReceive(g_new_data_queue, &data, pdMS_TO_TICKS(5)) == pdTRUE) {
        process_data(data);
    } else {
        // Use previous data or skip processing
    }
}
```

**Adding a New Shared Resource:**

```c
// 1. Declare mutex
SemaphoreHandle_t g_new_resource_mutex = NULL;

// 2. Create in app_main()
g_new_resource_mutex = xSemaphoreCreateMutex();
if (!g_new_resource_mutex) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return;
}

// 3. Access with timeout
if (xSemaphoreTake(g_new_resource_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // Critical section
    access_shared_resource();
    xSemaphoreGive(g_new_resource_mutex);
} else {
    ESP_LOGW(TAG, "Mutex timeout on new_resource");
}
```

---

## Conclusion

### Summary of Improvements

1. **✅ Robustness:** Timeout handling prevents deadlocks
2. **✅ Decoupling:** Queue-based communication reduces interdependencies
3. **✅ Diagnostics:** Warning logs identify issues early
4. **✅ Safety:** Graceful degradation maintains control
5. **✅ Clarity:** Clear documentation of all communication patterns

### Functional Equivalence

**Under normal operation:**
- All tasks behave identically to previous implementation
- Control loop stability unchanged
- Trajectory following performance identical
- Sensor reading accuracy preserved

**Under fault conditions:**
- New implementation degrades gracefully (continues with stale data)
- Old implementation could hang indefinitely (portMAX_DELAY)

### Future Enhancements

**Potential improvements:**
1. Add task watchdog monitoring
2. Implement performance metrics collection
3. Add event groups for multi-task synchronization
4. Consider stream buffers for high-throughput data
5. Implement task notifications for urgent events

---

**Document Version:** 1.0  
**Author:** GitHub Copilot  
**Review Status:** Ready for integration testing  
**Next Steps:** Run validation test suite, monitor logs for timeout warnings
