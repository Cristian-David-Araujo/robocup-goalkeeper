# Wheel PID Tuning Interface

A real-time web interface for tuning individual wheel PID controllers on the RoboCup goalkeeper robot.

## Features

- **Real-time PID Tuning**: Adjust Kp, Ki, Kd constants for all three wheels simultaneously
- **Live Telemetry**: View setpoint, measured velocity, and control output in real-time
- **Interactive Graphs**: Six synchronized graphs showing velocity tracking and control signals
- **Session Logging**: Record all tuning data with timestamps for offline analysis
- **CSV Export**: Download session logs for further processing
- **Safety Controls**: Emergency stop and automatic timeout protection

## Architecture

### Communication Protocol

The interface uses WebSocket for real-time bidirectional communication and UDP for robot telemetry.

#### Client → Server Commands

All commands are sent as JSON via WebSocket:

```json
{
  "command": "start_tuning" | "stop_tuning" | "apply_pid" | "set_setpoint" | "emergency_stop",
  "kp": 1.0,      // For apply_pid
  "ki": 0.1,      // For apply_pid
  "kd": 0.05,     // For apply_pid
  "setpoint": 10.0 // For set_setpoint (rad/s)
}
```

#### Server → Client Messages

**Configuration** (sent on connection):
```json
{
  "type": "config",
  "sampling_rate_hz": 50,
  "max_wheel_velocity": 50.0,
  "max_control_output": 100.0,
  "robot_connected": true
}
```

**Telemetry** (sent at sampling rate):
```json
{
  "type": "telemetry",
  "timestamp": "2024-12-09T10:30:15.123Z",
  "wheels": {
    "wheel1": {"velocity": 12.5, "setpoint": 10.0, "control": 45.3},
    "wheel2": {"velocity": 12.3, "setpoint": 10.0, "control": 44.8},
    "wheel3": {"velocity": 12.7, "setpoint": 10.0, "control": 46.1}
  },
  "robot_connected": true
}
```

**Acknowledgment** (response to commands):
```json
{
  "type": "ack",
  "command": "apply_pid",
  "success": true,
  "error": null
}
```

#### Robot ← Server Commands (UDP)

Commands sent to robot via UDP (port 3333):

```json
{
  "type": "tuning_command",
  "command": "start_tuning" | "stop_tuning" | "apply_pid" | "set_setpoint" | "emergency_stop",
  "mode": "wheel",
  "params": {
    "kp": 1.0,
    "ki": 0.1,
    "kd": 0.05,
    "setpoint": 10.0,
    "wheels": "all"
  }
}
```

#### Robot → Server Telemetry (UDP)

Telemetry sent from robot at 50 Hz:

```json
{
  "type": "wheel_telemetry",
  "timestamp": 1234567890,
  "wheels": [
    {"id": 1, "velocity": 12.5, "setpoint": 10.0, "control": 45.3},
    {"id": 2, "velocity": 12.3, "setpoint": 10.0, "control": 44.8},
    {"id": 3, "velocity": 12.7, "setpoint": 10.0, "control": 46.1}
  ]
}
```

## Installation & Usage

### Prerequisites

- Docker and Docker Compose (recommended) OR Python 3.8+
- Robot connected to the same network
- Robot IP address configured (default: 192.168.1.100)

### Option 1: Docker (Recommended)

1. Configure robot IP (create `.env` file):
```bash
cp .env.example .env
# Edit .env and set ROBOT_IP
```

2. Start with Docker Compose:
```bash
docker-compose up -d
```

3. View logs:
```bash
docker-compose logs -f
```

4. Stop:
```bash
docker-compose down
```

5. Open browser:
```
http://localhost:8081
```

### Option 2: Python Virtual Environment

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Configure robot IP (optional):
```bash
export ROBOT_IP=192.168.1.100
export ROBOT_PORT=3333
```

3. Start the server:
```bash
python app.py
```

Or use the convenience script:
```bash
# Linux/Mac
chmod +x start.sh
./start.sh

# Windows
start.bat
```

4. Open browser:
```
http://localhost:8081
```

### Configuration Options

Environment variables:

- `PID_TUNING_HOST`: Server bind address (default: 0.0.0.0)
- `PID_TUNING_PORT`: Server port (default: 8081)
- `ROBOT_IP`: Robot IP address (default: 192.168.1.100)
- `ROBOT_PORT`: Robot UDP port (default: 3333)
- `SAMPLING_RATE_HZ`: Telemetry rate (default: 50)
- `MAX_WHEEL_VEL`: Maximum wheel velocity in rad/s (default: 50.0)

## Tuning Workflow

### 1. Preparation

- Secure robot on test stand (wheels off ground)
- Verify robot is powered and connected to network
- Check robot IP address in configuration

### 2. Start Tuning Session

1. Open UI in browser
2. Verify "Robot: Connected" status is green
3. Click **"Start Tuning"** button
   - This disables high-level control (kinematics, body stabilization)
   - Enables direct wheel velocity control
   - Wheels enter tuning mode

### 3. Set PID Constants

1. Enter Kp, Ki, Kd values in input fields
2. Click **"Apply PID Constants"**
3. Constants are applied to all three wheels simultaneously

### 4. Test Response

1. Enter target velocity in "Set Point" field (e.g., 10.0 rad/s)
2. Click **"Send Set Point"**
3. Observe graphs:
   - **Blue dashed line**: Target setpoint
   - **Green solid line**: Measured velocity from encoders
   - **Orange line**: Control output (PWM %)

### 5. Tune Parameters

Iteratively adjust PID constants:

- **Kp (Proportional)**: Increases response speed, may cause oscillation if too high
- **Ki (Integral)**: Eliminates steady-state error, may cause overshoot
- **Kd (Derivative)**: Reduces overshoot and oscillation, may amplify noise

### 6. Evaluate Performance

Monitor graphs for:
- Rise time (how quickly velocity reaches setpoint)
- Overshoot (how much velocity exceeds setpoint)
- Settling time (time to stabilize within ±5% of setpoint)
- Steady-state error (final tracking error)

### 7. Save Session

1. Click **"Download Log"** to export CSV
2. Log includes all telemetry with timestamps
3. Analyze offline using Excel/Python/MATLAB

### 8. Stop Tuning

1. Click **"Emergency Stop"** or normal stop
2. Tuning mode is disabled
3. Robot returns to normal operation

## Firmware Integration

### Required Changes

The robot firmware must be modified to support tuning mode:

1. **Add tuning mode flag** in control task
2. **Parse tuning commands** from UDP
3. **Disable interfering modules** when tuning active:
   - Inverse kinematics
   - Body velocity controller
   - Trajectory planning
4. **Send telemetry** at 50 Hz via UDP
5. **Accept PID updates** during runtime

### Example Integration

See `firmware_integration_example.c` for detailed implementation guide.

## Safety Features

- **Emergency Stop**: Immediately halts all wheels and exits tuning mode
- **Connection Timeout**: Auto-stops if UI disconnects
- **Velocity Limits**: Setpoints clamped to maximum safe velocity
- **Parameter Validation**: Rejects negative or extreme PID values
- **Window Blur Detection**: Pauses tuning if browser loses focus

## Troubleshooting

### Robot Not Connecting

1. Verify robot is powered and on network
2. Check robot IP: `ping <ROBOT_IP>`
3. Verify UDP port 3333 is not blocked
4. Check firmware has WiFi control enabled

### No Telemetry Data

1. Confirm "Robot: Connected" status
2. Check firmware is sending telemetry
3. Verify UDP feedback listener is active (check logs)
4. Ensure firewall allows UDP on server port

### Graphs Not Updating

1. Check WebSocket connection (should show "Connected")
2. Verify sampling rate is reasonable (10-100 Hz)
3. Check browser console for JavaScript errors
4. Try reducing buffer size if performance is slow

### Poor PID Performance

1. Start with conservative values (Kp=0.5, Ki=0.05, Kd=0.01)
2. Increase Kp until oscillation appears, then reduce by 30%
3. Add Ki slowly to eliminate steady-state error
4. Add Kd to reduce overshoot if needed
5. Ensure mechanical system is in good condition (no friction, backlash)

## Development

### Testing Without Robot

Use the test UDP simulator:

```bash
python test_udp.py
```

This simulates robot telemetry for UI development.

### Project Structure

```
pid_wheel_tuning_ui/
├── app.py                 # FastAPI backend server
├── requirements.txt       # Python dependencies
├── README.md             # This file
├── start.sh              # Linux/Mac startup script
├── start.bat             # Windows startup script
└── static/
    ├── index.html        # UI layout and styling
    └── main.js          # Client-side logic and WebSocket handling
```

## License

MIT License - See LICENSE file for details

## Contributors

RoboCup Goalkeeper Team
Cristian David Araujo A.

## References

- FastAPI Documentation: https://fastapi.tiangolo.com/
- WebSocket Protocol: https://developer.mozilla.org/en-US/docs/Web/API/WebSocket
- PID Tuning Guide: https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
