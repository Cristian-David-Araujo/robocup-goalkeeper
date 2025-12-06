# WiFi Vector Control Feature

## Overview
This feature enables remote control of the robot through WiFi by receiving velocity vector commands (vx, vy, wz) from an external controller or application.

## Architecture

### Communication Protocol
- **Transport**: UDP (low latency) or TCP (reliable)
- **Port**: 3333 (configurable)
- **Message Format**: JSON or binary packed struct

### Command Structure
```json
{
  "vx": 0.5,      // Linear velocity in X direction (m/s)
  "vy": 0.3,      // Linear velocity in Y direction (m/s)
  "wz": 0.2,      // Angular velocity (rad/s)
  "timestamp": 1234567890
}
```

### Binary Format (Alternative)
```c
typedef struct {
    float vx;           // 4 bytes
    float vy;           // 4 bytes
    float wz;           // 4 bytes
    uint32_t timestamp; // 4 bytes
    uint16_t checksum;  // 2 bytes
} __attribute__((packed)) wifi_cmd_t; // Total: 18 bytes
```

## Implementation Components

### 1. WiFi Communication Task (`task_wifi_comm.c`)
- **Purpose**: Manage WiFi connection and receive velocity commands
- **Priority**: 3 (medium-high)
- **Period**: Event-driven (socket receive)
- **Responsibilities**:
  - Establish WiFi connection
  - Listen for incoming UDP/TCP packets
  - Parse velocity commands
  - Validate commands (range checking, timeout)
  - Send commands to velocity control queue

### 2. Command Validation
- **Velocity Limits**:
  - `vx, vy`: ±1.0 m/s (configurable)
  - `wz`: ±2.0 rad/s (configurable)
- **Timeout**: 500 ms (command expiration)
- **Safety**: Automatic stop if no commands received

### 3. Integration with Control System
```
WiFi Command → WiFi Task → Queue → Velocity Control Task → Motor Control
```

### 4. Configuration
```c
#define WIFI_SSID "RoboCup_Network"
#define WIFI_PASSWORD "goalkeeper2024"
#define WIFI_CMD_PORT 3333
#define WIFI_CMD_TIMEOUT_MS 500
#define WIFI_MAX_VX 1.0f
#define WIFI_MAX_VY 1.0f
#define WIFI_MAX_WZ 2.0f
```

## Usage

### Starting WiFi Control Mode
1. Robot connects to WiFi network on startup
2. WiFi task starts listening on configured port
3. External controller sends velocity commands
4. Robot executes received commands

### Safety Features
- Command timeout: Robot stops if no command received within 500ms
- Velocity clamping: Commands exceeding limits are capped
- Emergency stop: Special command (all zeros with flag) for immediate stop
- Connection monitoring: Robot stops if WiFi disconnects

## Testing

### Test Command (UDP)
```bash
# Send velocity command via netcat
echo '{"vx":0.5,"vy":0.3,"wz":0.1}' | nc -u <robot_ip> 3333
```

### Python Test Script
```python
import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
robot_ip = "192.168.1.100"
port = 3333

while True:
    cmd = {"vx": 0.5, "vy": 0.0, "wz": 0.0, "timestamp": int(time.time())}
    sock.sendto(json.dumps(cmd).encode(), (robot_ip, port))
    time.sleep(0.1)
```

## File Structure
```
firmware/
├── tasks/
│   └── task_wifi_comm.c          # WiFi communication task
├── include/
│   └── wifi_control.h             # WiFi control definitions
├── utils/
│   └── wifi_config_utils.h        # WiFi configuration
└── WIFI_VECTOR_CONTROL.md         # This documentation
```

## Future Enhancements
- [ ] Bidirectional communication (send robot state back)
- [ ] Multiple command modes (position, trajectory, velocity)
- [ ] Command queue with interpolation
- [ ] WebSocket support for web-based control
- [ ] Secure authentication
- [ ] Network diagnostics (latency, packet loss)
