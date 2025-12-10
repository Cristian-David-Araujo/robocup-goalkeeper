# Body PID Tuning Interface

Real-time web-based interface for tuning body-level PID controllers (vx, vy, wz) on the RoboCup goalkeeper robot.

## Features

### 🎯 Body Velocity Control
- **VX Controller**: Linear velocity in X direction
- **VY Controller**: Linear velocity in Y direction
- **WZ Controller**: Angular velocity around Z axis

### 📊 Real-Time Visualization
- **6 Interactive Graphs**: 
  - 3 velocity graphs (setpoint vs measured)
  - 3 control output graphs
- **Live Telemetry**: 50Hz update rate
- **Auto-scaling axes** with smooth animations

### 🎛️ PID Tuning Controls
- Independent PID parameters for each axis
- Real-time gain updates without restart
- Setpoint adjustment during tuning
- Visual feedback of current values

### 📝 Session Management
- **Data Logging**: Timestamped telemetry capture
- **CSV Export**: Download session data for analysis
- **Session Controls**: Start, pause, reset, emergency stop
- **Session Timer**: Track tuning duration

### 🔒 Safety Features
- **Emergency Stop**: Immediate motor shutdown
- **Connection Monitoring**: Real-time WebSocket status
- **Parameter Validation**: Prevent invalid inputs
- **Telemetry Rate Display**: Monitor communication health

## Architecture

```
┌─────────────────┐       WebSocket        ┌──────────────────┐       UDP          ┌──────────┐
│   Web Browser   │ ◄──────────────────► │  FastAPI Server  │ ◄────────────────► │  ESP32   │
│   (Frontend)    │   JSON (Commands)      │    (Backend)     │   JSON (Telemetry) │  Robot   │
└─────────────────┘   JSON (Telemetry)     └──────────────────┘   JSON (Commands)  └──────────┘
```

### Communication Flow

1. **UI → Server**: PID parameters, setpoints, control commands
2. **Server → Robot**: UDP JSON commands
3. **Robot → Server**: UDP JSON telemetry (50Hz)
4. **Server → UI**: WebSocket telemetry broadcast

### Message Protocol

#### Body Tuning Command (UI → Robot)
```json
{
  "type": "tuning_command",
  "mode": "body",
  "action": "start",
  "vx_setpoint": 1.0,
  "vy_setpoint": 0.5,
  "wz_setpoint": 0.0
}
```

#### PID Update Command
```json
{
  "type": "tuning_command",
  "mode": "body",
  "action": "update_pid",
  "axis": "vx",
  "kp": 1.5,
  "ki": 0.2,
  "kd": 0.08
}
```

#### Body Telemetry (Robot → UI)
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

## Installation

### Option 1: Docker (Recommended)

```bash
# Build and start container
docker-compose up -d

# View logs
docker-compose logs -f

# Stop container
docker-compose down
```

### Option 2: Manual Setup

```bash
# Install Python dependencies
pip install -r requirements.txt

# Run server
python app.py
```

## Usage

### 1. Start the Server

**Docker:**
```bash
docker-compose up -d
```

**Manual:**
```bash
python app.py
```

The interface will be available at `http://localhost:8082`

### 2. Configure Robot Connection

Edit `.env` file (copy from `.env.example`):
```bash
ROBOT_IP=192.168.4.1      # ESP32 IP address
ROBOT_PORT=12345          # Robot UDP port
LOCAL_LISTEN_PORT=12346   # Server listening port
TELEMETRY_RATE_HZ=50      # Telemetry frequency
```

### 3. Test Without Hardware

Use the included UDP simulator:
```bash
# Terminal 1: Start simulator
python test_udp.py

# Terminal 2: Start server
python app.py

# Terminal 3: Access UI
# Open browser to http://localhost:8082
```

### 4. Tune PID Controllers

1. **Set Initial PID Values**: Enter Kp, Ki, Kd for each axis
2. **Apply PID Gains**: Click "Apply XX PID" button for each axis
3. **Set Setpoints**: Enter desired velocities
4. **Start Tuning**: Click "▶️ Start Tuning"
5. **Observe Response**: Monitor graphs for tracking performance
6. **Adjust Gains**: Fine-tune parameters based on response
7. **Download Data**: Export CSV for further analysis

## Tuning Guidelines

### VX/VY Controller (Linear Velocity)

**Step 1: Proportional Gain (Kp)**
- Start with Kp = 1.0, Ki = 0, Kd = 0
- Increase Kp until response is fast but oscillates
- Reduce Kp by 30-50%

**Step 2: Integral Gain (Ki)**
- Add small Ki (0.05-0.2) to eliminate steady-state error
- If overshoot increases, reduce Ki

**Step 3: Derivative Gain (Kd)**
- Add small Kd (0.01-0.1) to reduce overshoot
- Too much Kd causes noise amplification

### WZ Controller (Angular Velocity)

Similar process to VX/VY, but typical values may differ:
- Angular dynamics usually faster than linear
- May need higher Kp and lower Ki
- Monitor for mechanical resonances

### Performance Metrics

- **Rise Time**: Time to reach 90% of setpoint
- **Settling Time**: Time to stay within ±2% of setpoint
- **Overshoot**: Maximum deviation above setpoint
- **Steady-State Error**: Final tracking error

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | 192.168.4.1 | ESP32 robot IP address |
| `ROBOT_PORT` | 12345 | UDP port for robot commands |
| `LOCAL_LISTEN_PORT` | 12346 | Server UDP listening port |
| `TELEMETRY_RATE_HZ` | 50 | Telemetry update frequency |

### Server Configuration (app.py)

```python
HTTP_PORT = 8082              # Web server port
TELEMETRY_RATE_HZ = 50        # Telemetry frequency
```

## File Structure

```
pid_body_tuning_ui/
├── app.py                    # FastAPI backend server
├── static/
│   ├── index.html            # Web UI
│   └── main.js               # Frontend JavaScript
├── logs/                     # Session logs (auto-created)
├── requirements.txt          # Python dependencies
├── Dockerfile                # Container build config
├── docker-compose.yml        # Container orchestration
├── .env.example              # Environment template
├── .gitignore               # Git exclusions
├── .dockerignore            # Docker exclusions
├── test_udp.py              # Robot simulator
└── README.md                # This file
```

## Troubleshooting

### WebSocket Won't Connect
- Check server is running: `docker-compose ps` or `ps aux | grep python`
- Verify port 8082 is not in use: `netstat -an | grep 8082`
- Check browser console for errors

### No Telemetry Data
- Verify robot IP in `.env` matches actual robot
- Check robot is sending UDP packets (Wireshark)
- Ensure firewall allows UDP on configured ports
- Run `test_udp.py` to simulate robot

### Graphs Not Updating
- Check telemetry rate display (should be ~50 Hz)
- Verify "Start Tuning" was clicked
- Check browser console for JavaScript errors
- Ensure WebSocket connection is active (green indicator)

### PID Updates Not Applied
- Verify WebSocket connection is active
- Check server logs for UDP send errors
- Ensure robot firmware is processing commands
- Test with simulator first

## Data Export Format

CSV columns:
```
Timestamp, Elapsed(s), VX_Setpoint, VX_Measured, VX_Control,
VY_Setpoint, VY_Measured, VY_Control, WZ_Setpoint, WZ_Measured, WZ_Control
```

Example:
```csv
2024-12-09T10:30:00.000Z,0.020,1.0000,0.0500,0.9500,0.5000,0.0200,0.4800,0.0000,0.0100,-0.0100
2024-12-09T10:30:00.020Z,0.040,1.0000,0.1200,0.8800,0.5000,0.0450,0.4550,0.0000,0.0050,-0.0050
```

## Performance Notes

- **Telemetry Rate**: 50Hz provides smooth visualization without overwhelming network
- **Graph Window**: 10 seconds of history (500 points max)
- **WebSocket**: Low latency for control commands (<10ms typical)
- **UDP**: Reliable on local WiFi, may need tuning for longer distances

## Safety Considerations

⚠️ **IMPORTANT SAFETY REMINDERS**:

1. **Emergency Stop**: Always keep E-STOP button accessible
2. **Safe Area**: Ensure robot has clear operating space
3. **Parameter Limits**: Start with conservative PID gains
4. **Supervision**: Never leave robot unattended during tuning
5. **Power Control**: Have physical power switch accessible

## Integration with Firmware

See `FIRMWARE_INTEGRATION.md` for detailed firmware implementation guide.

## Related Documentation

- `CASCADED_PID_CONTROL.md` - Overall PID architecture
- `WIFI_VECTOR_CONTROL.md` - WiFi communication protocol
- `test/pid_wheel_tuning_ui/` - Wheel-level PID tuning interface

## License

Part of the RoboCup Goalkeeper firmware project.

## Support

For issues or questions, refer to the main project repository.
