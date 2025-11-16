# ESP32-S3 Goalkeeper Firmware - Refactoring Plan

## Overview
This document outlines the comprehensive refactoring of the RoboCup Goalkeeper firmware to align with FreeRTOS and ESP-IDF best practices.

## Naming Conventions Standard

### 1. Type Names
**Rule:** All custom types use `snake_case_t` suffix

**Before → After:**
- `AS5600_t` → `as5600_t`
- `BNO055_t` → `bno055_t`  
- `motor_brushless_t` → ✅ (already correct)
- `pid_block_handle_t` → ✅ (already correct)
- `encoder_reading_t` → ✅ (already correct)

### 2. Global Variables
**Rule:** Prefix with `g_` followed by `snake_case`

**Already correct:** Most globals already use `g_` prefix
- `g_motor`, `g_as5600`, `g_bno055`
- `g_pid`, `g_velocity_pid`
- `g_sensor_data`, `g_robot_estimated`
- All task handles: `g_task_*_handle`
- All FreeRTOS primitives: `g_*_mutex`, `g_*_queue`

### 3. Static Variables
**Rule:** Prefix with `s_` followed by `snake_case`

**Example:**
```c
static const char *s_tag = "MODULE";
static uint32_t s_sample_count = 0;
```

### 4. Function Names
**Rule:** `module_action_object()` pattern

**Already mostly correct:**
- `motor_init()`, `motor_set_speed()`, `motor_stop()`
- `pid_init()`, `pid_compute()`
- `kinematics_forward()`, `kinematics_inverse()`

**Needs adjustment:**
- Driver-specific functions should keep hardware prefix:
  - `as5600_init()` → ✅
  - `bno055_init()` → ✅

### 5. Macros and Constants
**Rule:** `UPPER_SNAKE_CASE`

**Already correct:**
- `NUM_ENCODERS`
- `PID_MOTOR_KP`, `PID_MOTOR_KI`, `PID_MOTOR_KD`
- `INIT_SUCCESS`, `INIT_ERROR_*`

### 6. File Names
**Rule:** All lowercase `snake_case.c` or `snake_case.h`

**Already correct:**
- `task_motor_control.c`
- `types_utils.h`
- `config_utils.h`

## Header Guards

### Standard Format
```c
#ifndef MODULE_NAME_H
#define MODULE_NAME_H
// content
#endif // MODULE_NAME_H
```

**Apply to all headers:**
- Use simple `MODULE_NAME_H` format
- Include trailing comment `// MODULE_NAME_H`
- Match exactly to filename

## Function Organization

### Order Within Files
1. Includes
2. Macros/Constants
3. Type definitions
4. Static variables
5. Static function declarations
6. Public functions
7. Static functions

### Documentation
**Rule:** Every public function needs Doxygen comment

**Format:**
```c
/**
 * @brief Brief one-line description
 * 
 * Detailed description if needed.
 * 
 * @param[in] input Description
 * @param[out] output Description
 * @param[in,out] both Description
 * @return Description of return value
 * 
 * @note Special notes
 * @warning Important warnings
 */
```

## Task Naming

### Task Functions
**Pattern:** `task_<purpose>(void *pvParameters)`

**Current (all correct):**
- `task_read_sensors()`
- `task_motor_control()`
- `task_inverse_kinematics()`
- `task_velocity_control()`
- `task_move_trajectory()`

### Task Handles
**Pattern:** `g_task_<purpose>_handle`

**Current (all correct):**
- `g_task_sensor_handle`
- `g_task_control_handle`
- `g_task_ik_handle`
- `g_task_velocity_control_handle`
- `g_task_trajectory_handle`

## FreeRTOS Best Practices

### 1. Handle Naming
**Rule:** Clear, descriptive names with proper suffix

✅ **Current usage is good:**
- Queues: `g_*_queue`
- Mutexes: `g_*_mutex`
- Task handles: `g_task_*_handle`

### 2. Timeout Values
**Rule:** Always use explicit timeouts, never `portMAX_DELAY` in production

✅ **Already implemented:** All code uses timeouts with `pdMS_TO_TICKS()`

### 3. Critical Sections
**Rule:** Keep critical sections as short as possible

✅ **Current code:** Uses mutexes appropriately with timeouts

## Module Refactoring Checklist

### Files to Review and Refactor

#### Type Definitions Headers
- [ ] `include/as5600.h` - Change `AS5600_t` → `as5600_t`
- [ ] `include/bno055.h` - Change `BNO055_t` → `bno055_t`
- [ ] `include/motor.h` - ✅ Already correct
- [ ] `include/pid.h` - Review and ensure consistency
- [ ] `include/kinematics.h` - Review structure
- [ ] `utils/types_utils.h` - ✅ Already correct

#### Implementation Files
- [ ] `src/as5600.c` - Update type references
- [ ] `src/bno055.c` - Update type references
- [ ] `src/motor.c` - Check static variable naming
- [ ] `src/pid.c` - Check implementation
- [ ] `src/kinematics.c` - Check implementation
- [ ] `src/platform_esp32s3.c` - Review platform abstractions

#### Task Files
- [ ] `tasks/task_read_sensors.c` - Check static variables
- [ ] `tasks/task_motor_control.c` - Check static variables
- [ ] `tasks/task_inverse_kinematics.c` - Check static variables
- [ ] `tasks/task_velocity_control.c` - Check static variables
- [ ] `tasks/task_move_trajectory.c` - Check static variables

#### Main Files
- [ ] `main/main.c` - Review global definitions
- [ ] `main/main.h` - Update type references
- [ ] `main/init.c` - Update type references
- [ ] `main/init.h` - Update type references

## Priority Refactorings

### High Priority (Type Consistency)
1. **AS5600_t → as5600_t** throughout codebase
2. **BNO055_t → bno055_t** throughout codebase
3. Ensure all static variables use `s_` prefix
4. Ensure all constants use `UPPER_SNAKE_CASE`

### Medium Priority (Code Quality)
1. Add missing Doxygen comments
2. Standardize header guards
3. Consistent error handling patterns
4. Add static assertions where helpful

### Low Priority (Nice to Have)
1. Extract magic numbers to named constants
2. Consider adding assertion macros
3. Add runtime parameter validation
4. Improve comment consistency

## ESP-IDF Specific Conventions

### Logging
**Rule:** Use ESP-IDF logging macros consistently

✅ **Already used:**
```c
static const char *TAG = "MODULE_NAME";
ESP_LOGI(TAG, "Message");
ESP_LOGW(TAG, "Warning");
ESP_LOGE(TAG, "Error");
```

### Error Handling
**Rule:** Use ESP-IDF error codes where appropriate

**Consider adding:**
```c
esp_err_t module_function(void) {
    if (error) return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}
```

### Memory Management
**Rule:** Use heap_caps_* for ESP32-specific allocations

**Current:** Uses FreeRTOS allocation (correct for task stacks)

## Testing Strategy

After each refactoring stage:
1. Build project: `idf.py build`
2. Check for compilation errors
3. Verify no functional changes (logic preservation)
4. Test basic functionality if possible

## Implementation Order

1. **Phase 1:** Type name refactoring (AS5600_t, BNO055_t)
2. **Phase 2:** Static variable prefix additions
3. **Phase 3:** Header guard standardization
4. **Phase 4:** Documentation improvements
5. **Phase 5:** Final verification and testing

## Notes

- Preserve all existing functionality
- Keep git history clean with atomic commits
- Each phase should compile successfully
- Document any breaking changes
