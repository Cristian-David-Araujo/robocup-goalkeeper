# Body PID Tuning - Firmware Integration Guide

This document explains how to integrate the body PID tuning interface with the ESP32 firmware.

## Overview

The body tuning interface controls the **body-level velocity PID controllers** (vx, vy, wz). When active:

1. **Trajectory planning is disabled** (no autonomous movement)
2. **Body velocity setpoints are set directly** from the UI
3. **Velocity PIDs remain active** to track commanded body velocities
4. **Inverse kinematics converts** body velocities to wheel speeds
5. **Wheel PIDs track** the wheel speed targets

## Architecture During Body Tuning

```
┌─────────────┐
│   Tuning    │  Setpoints: vx, vy, wz
│     UI      │  Commands: PID gains, E-stop
└──────┬──────┘
       │ UDP
       │
┌──────▼──────────────────────────────────────────┐
│               ESP32 Firmware                      │
│                                                   │
│  ┌────────────────┐    DISABLED during tuning    │
│  │  Trajectory    │                               │
│  │   Planning     │                               │
│  └────────────────┘                               │
│                                                   │
│  ┌────────────────┐                               │
│  │   Velocity     │  ◄── Uses g_body_tuning_     │
│  │  PID Control   │      setpoint[] as input     │
│  │  (vx, vy, wz)  │                               │
│  └───────┬────────┘                               │
│          │ velocity_command_queue                 │
│          ▼                                        │
│  ┌────────────────┐                               │
│  │   Inverse      │                               │
│  │  Kinematics    │                               │
│  └───────┬────────┘                               │
│          │ wheel_target_queue                     │
│          ▼                                        │
│  ┌────────────────┐                               │
│  │   Wheel PID    │                               │
│  │    Control     │                               │
│  └───────┬────────┘                               │
│          │                                        │
│          ▼                                        │
│     [Motors]                                      │
└───────────────────────────────────────────────────┘
```

## Global Variables (Already Declared)

These variables are already defined in `main/main.h` and `main/main.c`:

```c
// In main.h (extern declarations)
extern volatile bool g_body_tuning_active;
extern float g_body_tuning_setpoint[3];  // [0]=vx, [1]=vy, [2]=wz
extern pid_block_handle_t g_velocity_pid[3];
extern velocity_t g_robot_estimated;

// In main.c (definitions)
volatile bool g_body_tuning_active = false;
float g_body_tuning_setpoint[3] = {0.0f, 0.0f, 0.0f};
```

## Required Firmware Modifications

### 1. WiFi Task Modifications

File: `tasks/task_wifi_comm.c`

#### Add Command Parsing

```c
/**
 * @brief Parse body tuning command from JSON
 */
static void parse_body_tuning_command(cJSON *json) {
    const char *action = cJSON_GetObjectItem(json, "action")->valuestring;
    
    if (strcmp(action, "start") == 0) {
        // Start body tuning mode
        g_body_tuning_active = true;
        
        // Get setpoints
        cJSON *vx_sp = cJSON_GetObjectItem(json, "vx_setpoint");
        cJSON *vy_sp = cJSON_GetObjectItem(json, "vy_setpoint");
        cJSON *wz_sp = cJSON_GetObjectItem(json, "wz_setpoint");
        
        if (vx_sp) g_body_tuning_setpoint[0] = (float)vx_sp->valuedouble;
        if (vy_sp) g_body_tuning_setpoint[1] = (float)vy_sp->valuedouble;
        if (wz_sp) g_body_tuning_setpoint[2] = (float)wz_sp->valuedouble;
        
        ESP_LOGI(TAG, "Body tuning started: vx=%.2f, vy=%.2f, wz=%.2f",
                 g_body_tuning_setpoint[0],
                 g_body_tuning_setpoint[1],
                 g_body_tuning_setpoint[2]);
    }
    else if (strcmp(action, "stop") == 0) {
        // Stop body tuning mode
        g_body_tuning_active = false;
        g_body_tuning_setpoint[0] = 0.0f;
        g_body_tuning_setpoint[1] = 0.0f;
        g_body_tuning_setpoint[2] = 0.0f;
        
        ESP_LOGI(TAG, "Body tuning stopped");
    }
    else if (strcmp(action, "update_pid") == 0) {
        // Update PID gains for specified axis
        const char *axis = cJSON_GetObjectItem(json, "axis")->valuestring;
        float kp = (float)cJSON_GetObjectItem(json, "kp")->valuedouble;
        float ki = (float)cJSON_GetObjectItem(json, "ki")->valuedouble;
        float kd = (float)cJSON_GetObjectItem(json, "kd")->valuedouble;
        
        int axis_index = -1;
        if (strcmp(axis, "vx") == 0) axis_index = 0;
        else if (strcmp(axis, "vy") == 0) axis_index = 1;
        else if (strcmp(axis, "wz") == 0) axis_index = 2;
        
        if (axis_index >= 0) {
            // Acquire mutex for thread-safe PID update
            if (xSemaphoreTake(g_velocity_pid_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                pid_update_gains(g_velocity_pid[axis_index], kp, ki, kd);
                xSemaphoreGive(g_velocity_pid_mutex);
                
                ESP_LOGI(TAG, "Updated %s PID: Kp=%.3f, Ki=%.3f, Kd=%.3f",
                         axis, kp, ki, kd);
            }
        }
    }
    else if (strcmp(action, "set_setpoint") == 0) {
        // Update single axis setpoint
        const char *axis = cJSON_GetObjectItem(json, "axis")->valuestring;
        float setpoint = (float)cJSON_GetObjectItem(json, "setpoint")->valuedouble;
        
        if (strcmp(axis, "vx") == 0) g_body_tuning_setpoint[0] = setpoint;
        else if (strcmp(axis, "vy") == 0) g_body_tuning_setpoint[1] = setpoint;
        else if (strcmp(axis, "wz") == 0) g_body_tuning_setpoint[2] = setpoint;
        
        ESP_LOGI(TAG, "Updated %s setpoint: %.2f", axis, setpoint);
    }
    else if (strcmp(action, "emergency_stop") == 0) {
        // Emergency stop
        g_body_tuning_active = false;
        g_body_tuning_setpoint[0] = 0.0f;
        g_body_tuning_setpoint[1] = 0.0f;
        g_body_tuning_setpoint[2] = 0.0f;
        
        // Stop all motors immediately
        for (int i = 0; i < 3; i++) {
            motor_set_speed(&g_motor[i], 0);
        }
        
        ESP_LOGW(TAG, "EMERGENCY STOP activated");
    }
}
```

#### Add Telemetry Streaming

```c
/**
 * @brief Send body telemetry data via UDP
 * 
 * Sends current body velocity state for real-time visualization.
 * Call this at 50Hz from the WiFi task main loop.
 */
static void send_body_telemetry(void) {
    if (!g_body_tuning_active) return;
    
    // Read estimated body velocity (from forward kinematics)
    velocity_t estimated;
    if (xSemaphoreTake(g_estimated_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        estimated = g_robot_estimated;
        xSemaphoreGive(g_estimated_data_mutex);
    } else {
        // Timeout - use zero values
        estimated.vx = 0.0f;
        estimated.vy = 0.0f;
        estimated.wz = 0.0f;
    }
    
    // Get PID control outputs
    // These would need to be exposed from velocity control task
    // For now, use placeholder values
    float vx_control = 0.0f;  // TODO: Export from velocity task
    float vy_control = 0.0f;
    float wz_control = 0.0f;
    
    // Build JSON telemetry message
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "body_telemetry");
    
    // VX axis
    cJSON_AddNumberToObject(root, "vx_setpoint", g_body_tuning_setpoint[0]);
    cJSON_AddNumberToObject(root, "vx_measured", estimated.vx);
    cJSON_AddNumberToObject(root, "vx_control", vx_control);
    
    // VY axis
    cJSON_AddNumberToObject(root, "vy_setpoint", g_body_tuning_setpoint[1]);
    cJSON_AddNumberToObject(root, "vy_measured", estimated.vy);
    cJSON_AddNumberToObject(root, "vy_control", vy_control);
    
    // WZ axis
    cJSON_AddNumberToObject(root, "wz_setpoint", g_body_tuning_setpoint[2]);
    cJSON_AddNumberToObject(root, "wz_measured", estimated.wz);
    cJSON_AddNumberToObject(root, "wz_control", wz_control);
    
    // Send UDP packet
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        sendto(udp_sock, json_str, strlen(json_str), 0,
               (struct sockaddr *)&client_addr, sizeof(client_addr));
        free(json_str);
    }
    
    cJSON_Delete(root);
}
```

#### Integrate into Message Handler

```c
// In your existing UDP receive handler
void handle_udp_message(const char *data, size_t len) {
    cJSON *json = cJSON_Parse(data);
    if (json == NULL) {
        ESP_LOGW(TAG, "Failed to parse JSON");
        return;
    }
    
    const char *type = cJSON_GetObjectItem(json, "type")->valuestring;
    
    if (strcmp(type, "tuning_command") == 0) {
        const char *mode = cJSON_GetObjectItem(json, "mode")->valuestring;
        
        if (strcmp(mode, "body") == 0) {
            parse_body_tuning_command(json);
        }
        else if (strcmp(mode, "wheel") == 0) {
            // Existing wheel tuning handler
            parse_wheel_tuning_command(json);
        }
    }
    else if (strcmp(type, "velocity_command") == 0) {
        // Existing velocity command handler
        parse_velocity_command(json);
    }
    
    cJSON_Delete(json);
}
```

#### Add Telemetry Task Loop

```c
// In task_wifi_comm main loop
void task_wifi_comm(void *pvParameters) {
    // ... initialization code ...
    
    TickType_t last_telemetry_time = xTaskGetTickCount();
    const TickType_t telemetry_period = pdMS_TO_TICKS(20);  // 50Hz = 20ms
    
    while (1) {
        // Handle incoming UDP messages
        // ... existing receive code ...
        
        // Send telemetry at regular intervals
        TickType_t now = xTaskGetTickCount();
        if ((now - last_telemetry_time) >= telemetry_period) {
            if (g_body_tuning_active) {
                send_body_telemetry();
            }
            else if (g_wheel_tuning_active) {
                send_wheel_telemetry();
            }
            last_telemetry_time = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

### 2. Velocity Control Task Modifications

File: `tasks/task_velocity_control.c`

Modify to use body tuning setpoints when active:

```c
void task_velocity_control(void *pvParameters) {
    // ... initialization ...
    
    while (1) {
        velocity_t desired;
        
        // Check if in body tuning mode
        if (g_body_tuning_active) {
            // TUNING MODE: Use setpoints from tuning interface
            desired.vx = g_body_tuning_setpoint[0];
            desired.vy = g_body_tuning_setpoint[1];
            desired.wz = g_body_tuning_setpoint[2];
            
            // Don't wait for queue - we have direct setpoints
        } else {
            // NORMAL MODE: Get desired velocity from trajectory task
            if (xQueueReceive(g_desired_velocity_queue, &desired, pdMS_TO_TICKS(10)) != pdTRUE) {
                // No command - use zero velocity
                desired.vx = 0.0f;
                desired.vy = 0.0f;
                desired.wz = 0.0f;
            }
        }
        
        // ... rest of PID computation remains the same ...
        // The velocity PIDs will track the tuning setpoints
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### 3. Trajectory Task Modifications

File: `tasks/task_move_trajectory.c`

Skip trajectory planning during body tuning:

```c
void task_move_trajectory(void *pvParameters) {
    // ... initialization ...
    
    while (1) {
        // Skip trajectory planning during body tuning
        if (g_body_tuning_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // ... normal trajectory planning code ...
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

### 4. Export Velocity PID Outputs (Optional Enhancement)

To display control outputs in the UI, export PID outputs from velocity control task:

#### In main.h:
```c
/// @brief Array of velocity PID outputs for telemetry
/// Index: [0]=vx_output, [1]=vy_output, [2]=wz_output
extern float g_velocity_pid_outputs[3];
```

#### In main.c:
```c
float g_velocity_pid_outputs[3] = {0.0f, 0.0f, 0.0f};
```

#### In task_velocity_control.c:
```c
// After computing PID outputs
float vx_output = pid_compute(g_velocity_pid[0], ...);
float vy_output = pid_compute(g_velocity_pid[1], ...);
float wz_output = pid_compute(g_velocity_pid[2], ...);

// Store for telemetry
g_velocity_pid_outputs[0] = vx_output;
g_velocity_pid_outputs[1] = vy_output;
g_velocity_pid_outputs[2] = wz_output;
```

#### In WiFi task telemetry:
```c
cJSON_AddNumberToObject(root, "vx_control", g_velocity_pid_outputs[0]);
cJSON_AddNumberToObject(root, "vy_control", g_velocity_pid_outputs[1]);
cJSON_AddNumberToObject(root, "wz_control", g_velocity_pid_outputs[2]);
```

## Testing Procedure

### 1. Build and Flash Firmware

```bash
cd firmware
idf.py build flash monitor
```

### 2. Start Tuning Interface

**With Docker:**
```bash
cd test/pid_body_tuning_ui
docker-compose up -d
```

**Without Docker:**
```bash
cd test/pid_body_tuning_ui
python app.py
```

### 3. Connect to Robot WiFi

Connect computer to robot's WiFi access point or ensure both are on same network.

### 4. Access Web Interface

Open browser to: `http://localhost:8082`

### 5. Verify Communication

- Check WebSocket connection indicator (should be green)
- Check telemetry rate (should show ~50 Hz when tuning active)
- Check robot status display

### 6. Basic Tuning Test

1. **Set initial PID gains**: Use conservative values (Kp=1.0, Ki=0.1, Kd=0.05)
2. **Apply gains**: Click "Apply XX PID" for each axis
3. **Set small setpoint**: Start with vx=0.3 m/s, others at 0
4. **Start tuning**: Click "▶️ Start Tuning"
5. **Observe response**: Watch vx velocity graph
6. **Adjust gains**: Tune based on response characteristics
7. **Test other axes**: Repeat for vy and wz

## Communication Protocol Summary

### Commands (UI → Robot)

| Action | JSON Structure |
|--------|---------------|
| Start Tuning | `{"type":"tuning_command","mode":"body","action":"start","vx_setpoint":1.0,"vy_setpoint":0.5,"wz_setpoint":0.0}` |
| Stop Tuning | `{"type":"tuning_command","mode":"body","action":"stop"}` |
| Update PID | `{"type":"tuning_command","mode":"body","action":"update_pid","axis":"vx","kp":1.5,"ki":0.2,"kd":0.08}` |
| Set Setpoint | `{"type":"tuning_command","mode":"body","action":"set_setpoint","axis":"vx","setpoint":1.2}` |
| E-Stop | `{"type":"tuning_command","mode":"body","action":"emergency_stop"}` |

### Telemetry (Robot → UI)

```json
{
  "type": "body_telemetry",
  "vx_setpoint": 1.0,
  "vx_measured": 0.95,
  "vx_control": 0.12,
  "vy_setpoint": 0.5,
  "vy_measured": 0.48,
  "vy_control": 0.06,
  "wz_setpoint": 0.0,
  "wz_measured": 0.02,
  "wz_control": -0.01
}
```

Sent at 50Hz (20ms period) when `g_body_tuning_active == true`.

## Safety Considerations

1. **E-Stop Handler**: Immediately zeros all setpoints and stops motors
2. **Tuning Flag Check**: All control loops check `g_body_tuning_active`
3. **Mutex Protection**: PID gain updates use mutexes for thread safety
4. **Parameter Validation**: Validate PID gains are non-negative
5. **Timeout Handling**: Implement communication timeout detection

## Troubleshooting

| Issue | Likely Cause | Solution |
|-------|-------------|----------|
| No telemetry | Robot not calling `send_body_telemetry()` | Add telemetry call in WiFi task loop |
| PID updates ignored | Missing `pid_update_gains()` function | Implement in `src/pid.c` |
| Jerky motion | Telemetry rate too slow | Ensure 50Hz rate (20ms period) |
| Oscillations | PID gains too aggressive | Reduce Kp, add Kd damping |
| Steady-state error | Insufficient integral action | Increase Ki (carefully) |

## Next Steps

1. Implement WiFi task modifications
2. Test with simulator (`test_udp.py`)
3. Test on hardware with small setpoints
4. Tune PID gains systematically
5. Document final tuned parameters

## Related Files

- `main/main.h` - Global variable declarations
- `main/main.c` - Global variable definitions
- `tasks/task_wifi_comm.c` - WiFi communication (to be modified)
- `tasks/task_velocity_control.c` - Velocity PID control (to be modified)
- `tasks/task_move_trajectory.c` - Trajectory planning (to be modified)
- `include/pid.h`, `src/pid.c` - PID controller implementation

## References

- **CASCADED_PID_CONTROL.md** - Overall control architecture
- **WIFI_VECTOR_CONTROL.md** - WiFi communication protocol
- **test/pid_wheel_tuning_ui/** - Wheel tuning interface (similar implementation)
