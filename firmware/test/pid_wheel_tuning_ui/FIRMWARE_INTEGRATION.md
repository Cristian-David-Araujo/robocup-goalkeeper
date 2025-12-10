# Firmware Integration Guide for Wheel PID Tuning

This document describes the firmware modifications required to support the wheel PID tuning interface.

## Overview

The firmware must support:
1. **Tuning mode flag** - Enable/disable wheel tuning mode
2. **Command parsing** - Process tuning commands from UDP
3. **Module control** - Disable interfering high-level modules during tuning
4. **Telemetry streaming** - Send wheel data at 50 Hz via UDP
5. **Runtime PID updates** - Accept and apply new PID constants

## Required Changes

### 1. Add Tuning Mode State

Add global state variables in `main/main.h`:

```c
// Tuning mode flags
extern volatile bool g_wheel_tuning_active;
extern volatile bool g_body_tuning_active;

// Tuning mode setpoints (overrides normal control)
extern float g_wheel_tuning_setpoint;
```

In `main/main.c`:

```c
volatile bool g_wheel_tuning_active = false;
volatile bool g_body_tuning_active = false;
float g_wheel_tuning_setpoint = 0.0f;
```

### 2. Modify WiFi Communication Task

Update `tasks/task_wifi_comm.c` to handle tuning commands:

```c
static void parse_tuning_command(const cJSON *json, struct sockaddr_in *source_addr)
{
    const cJSON *command = cJSON_GetObjectItem(json, "command");
    if (!command || !cJSON_IsString(command)) {
        return;
    }
    
    const char *cmd_str = command->valuestring;
    
    if (strcmp(cmd_str, "start_tuning") == 0) {
        // Check mode
        const cJSON *mode = cJSON_GetObjectItem(json, "mode");
        if (mode && strcmp(mode->valuestring, "wheel") == 0) {
            g_wheel_tuning_active = true;
            ESP_LOGI(TAG, "Wheel tuning mode activated");
        }
        
    } else if (strcmp(cmd_str, "stop_tuning") == 0) {
        g_wheel_tuning_active = false;
        g_body_tuning_active = false;
        g_wheel_tuning_setpoint = 0.0f;
        ESP_LOGI(TAG, "Tuning mode deactivated");
        
    } else if (strcmp(cmd_str, "apply_pid") == 0) {
        const cJSON *params = cJSON_GetObjectItem(json, "params");
        if (params) {
            float kp = cJSON_GetObjectItem(params, "kp")->valuedouble;
            float ki = cJSON_GetObjectItem(params, "ki")->valuedouble;
            float kd = cJSON_GetObjectItem(params, "kd")->valuedouble;
            
            // Apply to all wheel PIDs
            if (g_pid_mutex && xSemaphoreTake(g_pid_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (int i = 0; i < 3; i++) {
                    pid_update_param(g_pid[i], kp, ki, kd);
                }
                xSemaphoreGive(g_pid_mutex);
                ESP_LOGI(TAG, "PID updated: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp, ki, kd);
            }
        }
        
    } else if (strcmp(cmd_str, "set_setpoint") == 0) {
        const cJSON *params = cJSON_GetObjectItem(json, "params");
        if (params) {
            const cJSON *setpoint = cJSON_GetObjectItem(params, "setpoint");
            if (setpoint) {
                g_wheel_tuning_setpoint = (float)setpoint->valuedouble;
                ESP_LOGI(TAG, "Setpoint updated: %.3f rad/s", g_wheel_tuning_setpoint);
            }
        }
        
    } else if (strcmp(cmd_str, "emergency_stop") == 0) {
        g_wheel_tuning_active = false;
        g_body_tuning_active = false;
        g_wheel_tuning_setpoint = 0.0f;
        
        // Stop all motors immediately
        for (int i = 0; i < 3; i++) {
            motor_stop(&g_motor[i]);
        }
        ESP_LOGW(TAG, "EMERGENCY STOP via tuning interface");
    }
}

// In the WiFi receive loop, add detection for tuning commands:
static void wifi_receive_task(void *pvParameters)
{
    // ... existing code ...
    
    while (1) {
        // Receive UDP packet
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                          (struct sockaddr *)&source_addr, &socklen);
        
        if (len > 0) {
            rx_buffer[len] = 0;
            
            // Parse JSON
            cJSON *json = cJSON_Parse(rx_buffer);
            if (json) {
                const cJSON *type = cJSON_GetObjectItem(json, "type");
                
                if (type && strcmp(type->valuestring, "tuning_command") == 0) {
                    // Handle tuning command
                    parse_tuning_command(json, &source_addr);
                } else {
                    // Handle normal velocity command
                    parse_velocity_command(json);
                }
                
                cJSON_Delete(json);
            }
        }
    }
}
```

### 3. Add PID Parameter Update Function

Add to `src/pid.c`:

```c
esp_err_t pid_update_param(pid_block_handle_t handle, float kp, float ki, float kd)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    pid_block_t *block = (pid_block_t *)handle;
    
    block->param.kp = kp;
    block->param.ki = ki;
    block->param.kd = kd;
    
    // Reset integral to avoid windup with new parameters
    block->integral = 0.0f;
    
    return ESP_OK;
}
```

Add declaration to `include/pid.h`:

```c
/**
 * @brief Update PID parameters at runtime
 * 
 * @param handle PID controller handle
 * @param kp New proportional gain
 * @param ki New integral gain
 * @param kd New derivative gain
 * @return ESP_OK on success
 */
esp_err_t pid_update_param(pid_block_handle_t handle, float kp, float ki, float kd);
```

### 4. Modify Motor Control Task

Update `tasks/task_motor_control.c` to respect tuning mode:

```c
void task_motor_control(void *pvParameters) 
{
    // ... existing initialization ...
    
    while (1) {
        // Check if in wheel tuning mode
        if (g_wheel_tuning_active) {
            // TUNING MODE: Direct wheel control
            // Use the same setpoint for all wheels
            wheel_targets.wheel1 = g_wheel_tuning_setpoint;
            wheel_targets.wheel2 = g_wheel_tuning_setpoint;
            wheel_targets.wheel3 = g_wheel_tuning_setpoint;
            
            // Skip queue receive - use tuning setpoint directly
        } else {
            // NORMAL MODE: Receive targets from IK task
            if (xQueueReceive(g_wheel_target_queue, &wheel_targets, pdMS_TO_TICKS(1)) == pdTRUE) {
                targets_received = true;
                no_target_count = 0;
            }
        }
        
        // ... rest of control loop (PID computation, motor commands) ...
    }
}
```

### 5. Disable Interfering Modules

Update velocity control task `tasks/task_velocity_control.c`:

```c
void task_velocity_control(void *pvParameters)
{
    // ... existing code ...
    
    while (1) {
        // Skip velocity control if wheel tuning is active
        if (g_wheel_tuning_active) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // ... normal velocity control logic ...
    }
}
```

Update inverse kinematics task `tasks/task_inverse_kinematics.c`:

```c
void task_inverse_kinematics(void *pvParameters)
{
    // ... existing code ...
    
    while (1) {
        // Skip IK if wheel tuning is active
        if (g_wheel_tuning_active) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        // ... normal IK logic ...
    }
}
```

### 6. Add Telemetry Streaming

Create new function in `tasks/task_wifi_comm.c`:

```c
static void send_wheel_telemetry(int sock, struct sockaddr_in *dest_addr)
{
    if (!g_wheel_tuning_active) {
        return;  // Only send telemetry during tuning
    }
    
    // Gather current wheel data
    raw_sensor_data_t sensor_data;
    float pid_outputs[3] = {0};
    
    if (g_sensor_data_mutex && xSemaphoreTake(g_sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        sensor_data = g_sensor_data;
        xSemaphoreGive(g_sensor_data_mutex);
    }
    
    // Build JSON telemetry
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "wheel_telemetry");
    cJSON_AddNumberToObject(root, "timestamp", (double)xTaskGetTickCount());
    
    cJSON *wheels = cJSON_CreateArray();
    
    for (int i = 0; i < 3; i++) {
        cJSON *wheel = cJSON_CreateObject();
        cJSON_AddNumberToObject(wheel, "id", i + 1);
        cJSON_AddNumberToObject(wheel, "velocity", sensor_data.encoders[i].angular_velocity);
        cJSON_AddNumberToObject(wheel, "setpoint", g_wheel_tuning_setpoint);
        
        // Get PID output (motor command)
        // This requires exposing the last PID output from motor control task
        cJSON_AddNumberToObject(wheel, "control", pid_outputs[i]);
        
        cJSON_AddItemToArray(wheels, wheel);
    }
    
    cJSON_AddItemToObject(root, "wheels", wheels);
    
    // Send via UDP
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        sendto(sock, json_str, strlen(json_str), 0,
               (struct sockaddr *)dest_addr, sizeof(*dest_addr));
        free(json_str);
    }
    
    cJSON_Delete(root);
}

// Add telemetry task
static void telemetry_stream_task(void *pvParameters)
{
    const TickType_t telemetry_period = pdMS_TO_TICKS(20);  // 50 Hz
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Get socket and destination from parent task
    // (This requires passing these as task parameters)
    
    while (1) {
        if (g_wheel_tuning_active && g_wifi_state.connected) {
            send_wheel_telemetry(sock, &dest_addr);
        }
        
        vTaskDelayUntil(&last_wake_time, telemetry_period);
    }
}
```

### 7. Expose PID Outputs

Modify `tasks/task_motor_control.c` to expose control outputs:

```c
// Add global array for PID outputs
float g_pid_outputs[3] = {0.0f, 0.0f, 0.0f};

// In the control loop, after computing PID:
for (int i = 0; i < 3; i++) {
    pid_output[i] = pid_compute(g_pid[i], encoder[i].angular_velocity);
    g_pid_outputs[i] = pid_output[i];  // Store for telemetry
    
    // Convert to motor command
    motor_set_speed(&g_motor[i], pid_output[i]);
}
```

Declare in `main/main.h`:

```c
extern float g_pid_outputs[3];
```

## Testing Procedure

### 1. Compile and Flash

```bash
idf.py build
idf.py flash monitor
```

### 2. Start Tuning UI

```bash
cd test/pid_wheel_tuning_ui
python app.py
```

### 3. Verify Communication

1. Check logs for "Wheel tuning mode activated"
2. Verify telemetry is being sent (50 Hz)
3. Confirm PID constants update correctly
4. Test emergency stop functionality

### 4. Safety Checks

- Emergency stop should immediately halt all wheels
- Tuning mode should disable high-level control
- Connection loss should trigger automatic stop
- Setpoints should be clamped to safe limits

## Troubleshooting

**Telemetry not received:**
- Check `dest_addr` has correct UI server address
- Verify UDP socket is sending to correct port
- Check firewall rules

**PID updates not applying:**
- Verify mutex is acquired successfully
- Check PID handles are valid
- Ensure `pid_update_param` function exists

**Modules still interfering:**
- Add logging to confirm `g_wheel_tuning_active` flag is set
- Check all control tasks are checking the flag
- Verify flag is volatile to prevent optimization

## Protocol Summary

### UI → Firmware (UDP to robot)

```json
{"type": "tuning_command", "command": "start_tuning", "mode": "wheel"}
{"type": "tuning_command", "command": "apply_pid", "params": {"kp": 1.0, "ki": 0.1, "kd": 0.05}}
{"type": "tuning_command", "command": "set_setpoint", "params": {"setpoint": 10.0}}
{"type": "tuning_command", "command": "emergency_stop"}
```

### Firmware → UI (UDP from robot)

```json
{
  "type": "wheel_telemetry",
  "timestamp": 123456,
  "wheels": [
    {"id": 1, "velocity": 12.5, "setpoint": 10.0, "control": 45.3},
    {"id": 2, "velocity": 12.3, "setpoint": 10.0, "control": 44.8},
    {"id": 3, "velocity": 12.7, "setpoint": 10.0, "control": 46.1}
  ]
}
```

## Files Modified

- `main/main.h` - Add tuning mode globals
- `main/main.c` - Initialize tuning mode variables
- `include/pid.h` - Add `pid_update_param` declaration
- `src/pid.c` - Implement `pid_update_param`
- `tasks/task_wifi_comm.c` - Parse tuning commands, send telemetry
- `tasks/task_motor_control.c` - Respect tuning mode, expose PID outputs
- `tasks/task_velocity_control.c` - Disable during wheel tuning
- `tasks/task_inverse_kinematics.c` - Disable during wheel tuning

## Next Steps

After wheel tuning is working:
1. Implement body PID tuning mode (similar approach)
2. Add parameter persistence (save tuned values to flash)
3. Add auto-tuning algorithms (Ziegler-Nichols, etc.)
4. Implement per-wheel setpoints for asymmetric testing
