# Robot Teleoperation Web UI

A real-time web-based teleoperation interface for controlling omnidirectional robots via keyboard. Built with FastAPI, WebSockets, and vanilla JavaScript for low-latency control.

## Features

- 🎮 **Real-time keyboard control** - Game-like WASD + QE controls
- 🔌 **WebSocket communication** - Low latency command streaming
- 🛡️ **Safety features**:
  - Emergency stop button
  - Automatic stop on connection loss
  - Command timeout (1 second default)
  - Velocity clamping
- 📊 **Live monitoring** - Real-time velocity display and update rate
- 🐳 **Docker ready** - Easy deployment with Docker/Docker Compose
- ⚙️ **Configurable** - Environment variables for all settings

## Key Mapping

| Key | Action | Velocity Component |
|-----|--------|-------------------|
| **W** | Move Forward | +Linear X |
| **S** | Move Backward | -Linear X |
| **A** | Strafe Left | -Linear Y |
| **D** | Strafe Right | +Linear Y |
| **Q** | Rotate Left | -Angular Z |
| **E** | Rotate Right | +Angular Z |

Multiple keys can be pressed simultaneously for combined movements (e.g., W+D = forward-right diagonal).

## Quick Start

### Option 1: Docker Compose (Recommended)

1. **Update robot IP in `docker-compose.yml`**:
   ```yaml
   environment:
     - ROBOT_IP=192.168.1.100  # Change to your robot's IP
   ```

2. **Build and run**:
   ```bash
   docker-compose up --build
   ```

3. **Open browser**:
   ```
   http://localhost:8080
   ```

### Option 2: Docker

```bash
# Build image
docker build -t robot-teleop-ui .

# Run container
docker run -d \
  --name robot-teleop \
  -p 8080:8080 \
  -e ROBOT_IP=192.168.1.100 \
  -e ROBOT_PORT=3333 \
  -e MAX_LINEAR_VEL=1.0 \
  -e MAX_ANGULAR_VEL=2.0 \
  robot-teleop-ui
```

### Option 3: Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Set environment variables (optional)
export ROBOT_IP=192.168.1.100
export ROBOT_PORT=3333
export MAX_LINEAR_VEL=1.0
export MAX_ANGULAR_VEL=2.0

# Run application
python app.py
```

Then open http://localhost:8080 in your browser.

## Configuration

All settings can be configured via environment variables:

### Server Configuration

| Variable | Default | Description |
|----------|---------|-------------|
| `TELEOP_HOST` | `0.0.0.0` | Server bind address |
| `TELEOP_PORT` | `8080` | Server port |

### Robot Connection

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `192.168.1.100` | Robot's IP address |
| `ROBOT_PORT` | `3333` | Robot's UDP port |

### Velocity Limits

| Variable | Default | Description |
|----------|---------|-------------|
| `MAX_LINEAR_VEL` | `1.0` | Max linear velocity (m/s) |
| `MAX_ANGULAR_VEL` | `2.0` | Max angular velocity (rad/s) |

### Safety Configuration

| Variable | Default | Description |
|----------|---------|-------------|
| `COMMAND_TIMEOUT` | `1.0` | Command timeout in seconds |
| `UPDATE_RATE_HZ` | `20` | Command update rate (Hz) |

## Communication Protocol

The application sends velocity commands to the robot via UDP in JSON format:

```json
{
  "vx": 0.5,   // Linear X velocity (m/s)
  "vy": 0.3,   // Linear Y velocity (m/s)
  "wz": 0.1    // Angular Z velocity (rad/s)
}
```

This matches the protocol expected by the robot's WiFi control system (see `WIFI_VECTOR_CONTROL.md` in the main firmware documentation).

## Adapting to Different Robot Interfaces

The `RobotAdapter` class in `app.py` handles communication with the robot. To adapt it to different interfaces:

### 1. ROS/ROS2

```python
# Add rclpy to requirements.txt
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotAdapter:
    def __init__(self):
        self.node = Node('teleop_adapter')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
    
    def send_command(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub.publish(msg)
        return True
```

### 2. MQTT

```python
# Add paho-mqtt to requirements.txt
import paho.mqtt.client as mqtt
import json

class RobotAdapter:
    def __init__(self, broker_ip, broker_port):
        self.client = mqtt.Client()
        self.client.connect(broker_ip, broker_port)
        self.client.loop_start()
    
    def send_command(self, vx, vy, wz):
        msg = json.dumps({"vx": vx, "vy": vy, "wz": wz})
        self.client.publish("robot/cmd_vel", msg)
        return True
```

### 3. TCP Socket

```python
import socket
import json

class RobotAdapter:
    def __init__(self, robot_ip, robot_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((robot_ip, robot_port))
    
    def send_command(self, vx, vy, wz):
        msg = json.dumps({"vx": vx, "vy": vy, "wz": wz}) + "\n"
        self.sock.sendall(msg.encode())
        return True
```

### 4. Serial Port

```python
# Add pyserial to requirements.txt
import serial
import json

class RobotAdapter:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate)
    
    def send_command(self, vx, vy, wz):
        msg = json.dumps({"vx": vx, "vy": vy, "wz": wz}) + "\n"
        self.ser.write(msg.encode())
        return True
```

## API Endpoints

### Web Interface
- `GET /` - Main teleoperation UI

### REST API
- `GET /health` - Health check and status
- `GET /api/config` - Get current configuration
- `POST /api/stop` - Emergency stop

### WebSocket
- `WS /ws` - Real-time velocity command streaming

## Safety Features

1. **Emergency Stop**: Red button immediately sends zero velocities
2. **Connection Monitoring**: Robot stops if WebSocket disconnects
3. **Command Timeout**: Robot stops if no commands received for 1 second
4. **Velocity Clamping**: All commands are clamped to configured limits
5. **Window Blur Detection**: Robot stops when browser loses focus
6. **Page Visibility**: Robot stops when tab is hidden

## Testing

### Health Check
```bash
curl http://localhost:8080/health
```

### Emergency Stop
```bash
curl -X POST http://localhost:8080/api/stop
```

### Manual Velocity Command (for testing)
You can test the robot interface without the UI:
```bash
echo '{"vx":0.5,"vy":0.0,"wz":0.0}' | nc -u <ROBOT_IP> 3333
```

## Network Configuration

### Same Network as Robot
If the container is on the same network as the robot, use the robot's IP directly.

### Host Network Mode
To use host networking (simpler for local development):
```bash
docker run --network host -e ROBOT_IP=192.168.1.100 robot-teleop-ui
```

Or in `docker-compose.yml`:
```yaml
services:
  robot-teleop-ui:
    network_mode: host
```

## Troubleshooting

### Robot not responding
1. Check robot IP and port configuration
2. Verify robot is connected to network: `ping <ROBOT_IP>`
3. Check robot's WiFi control task is running
4. Verify firewall allows UDP traffic on port 3333
5. Check logs: `docker logs robot-teleop-ui`

### High latency
1. Reduce update rate: `UPDATE_RATE_HZ=10`
2. Use wired connection instead of WiFi
3. Check network congestion
4. Run container on same machine/network as robot

### WebSocket connection fails
1. Check browser console for errors
2. Verify firewall allows WebSocket connections
3. Try disabling browser extensions
4. Check server logs for connection attempts

### Keys not responding
1. Click on the web page to ensure focus
2. Check browser console for JavaScript errors
3. Verify keyboard is not captured by other apps
4. Try pressing keys one at a time first

## Development

### Project Structure
```
robot_teleop_ui/
├── app.py                 # FastAPI application & WebSocket server
├── static/
│   ├── index.html        # Web UI
│   └── main.js           # Client-side JavaScript
├── requirements.txt       # Python dependencies
├── Dockerfile            # Container image definition
├── docker-compose.yml    # Docker Compose configuration
└── README.md            # This file
```

### Adding Features

1. **Add logging**: Logs are already configured with Python's logging module
2. **Add authentication**: Use FastAPI's security utilities
3. **Add HTTPS**: Configure uvicorn with SSL certificates
4. **Add recording**: Log commands to file for replay/analysis

## License

MIT License - See main repository for details

## Authors

RoboCup Goalkeeper Team

## Related Documentation

- Main firmware: `../firmware/README.md`
- WiFi control protocol: `../firmware/WIFI_VECTOR_CONTROL.md`
- Robot configuration: `../firmware/utils/config_utils.h`
