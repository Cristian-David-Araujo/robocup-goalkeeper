# Body PID Tuning - Quick Start Guide

Get up and running in 5 minutes.

## Prerequisites

- Docker and Docker Compose installed
- OR Python 3.11+ with pip
- Robot with WiFi configured
- Computer connected to robot's network

## Quick Start (Docker - Recommended)

```bash
# 1. Navigate to directory
cd test/pid_body_tuning_ui

# 2. Copy environment template
cp .env.example .env

# 3. Edit robot IP if needed
nano .env  # or your editor
# Change ROBOT_IP=192.168.4.1 to your robot's IP

# 4. Start container
docker-compose up -d

# 5. Open browser
# Navigate to: http://localhost:8082

# 6. View logs (optional)
docker-compose logs -f

# 7. Stop when done
docker-compose down
```

## Quick Start (Manual)

```bash
# 1. Navigate to directory
cd test/pid_body_tuning_ui

# 2. Install dependencies
pip install -r requirements.txt

# 3. Set environment variables (optional)
export ROBOT_IP=192.168.4.1
export ROBOT_PORT=12345

# 4. Start server
python app.py

# 5. Open browser
# Navigate to: http://localhost:8082
```

## Test Without Robot

```bash
# Terminal 1: Start simulator
python test_udp.py

# Terminal 2: Start server
python app.py  # or docker-compose up

# Terminal 3: Open browser
# Navigate to: http://localhost:8082
```

## Basic Tuning Workflow

### 1. Set Initial PID Gains
```
VX: Kp=1.0, Ki=0.1, Kd=0.05
VY: Kp=1.0, Ki=0.1, Kd=0.05
WZ: Kp=1.0, Ki=0.1, Kd=0.05
```

Click "Apply XX PID" for each axis.

### 2. Set Conservative Setpoints
```
VX: 0.3 m/s
VY: 0.0 m/s
WZ: 0.0 rad/s
```

### 3. Start Tuning Session

Click "▶️ Start Tuning"

### 4. Observe Response

Watch VX velocity graph:
- **Good**: Smooth approach to setpoint, minimal overshoot
- **Too slow**: Increase Kp slightly
- **Oscillating**: Reduce Kp, increase Kd
- **Steady error**: Increase Ki carefully

### 5. Tune Each Axis

Test each axis independently:
1. Set only one axis to non-zero setpoint
2. Observe response
3. Adjust PID gains
4. Repeat

### 6. Download Data

Click "💾 Download CSV" to export session data for analysis.

## Common Setpoint Values

| Axis | Safe Test | Moderate | Aggressive |
|------|-----------|----------|------------|
| VX | 0.3 m/s | 0.8 m/s | 1.5 m/s |
| VY | 0.2 m/s | 0.6 m/s | 1.2 m/s |
| WZ | 0.5 rad/s | 1.5 rad/s | 3.0 rad/s |

## Typical PID Values

Based on similar omni-directional robots:

### Linear Velocity (VX, VY)
```
Kp: 0.8 - 2.0
Ki: 0.05 - 0.3
Kd: 0.01 - 0.15
```

### Angular Velocity (WZ)
```
Kp: 1.0 - 3.0
Ki: 0.1 - 0.5
Kd: 0.02 - 0.2
```

## Status Indicators

| Indicator | Meaning |
|-----------|---------|
| 🟢 Connected | WebSocket connection active |
| 🔴 Disconnected | No connection to server |
| 🤖 Tuning Active | Body tuning mode enabled |
| 📊 50 Hz | Telemetry rate (healthy) |
| 📊 0 Hz | No telemetry (check robot) |

## Emergency Stop

**Click 🛑 E-STOP button** or press emergency stop on robot.

This immediately:
- Stops all motors
- Clears all setpoints
- Exits tuning mode

## Troubleshooting (30 seconds)

### No Connection
```bash
# Check server is running
docker ps  # or ps aux | grep python

# Restart server
docker-compose restart
```

### No Telemetry
1. Check robot is powered on
2. Verify network connection
3. Check ROBOT_IP in `.env`
4. Test with simulator first

### Graphs Not Moving
1. Click "▶️ Start Tuning"
2. Set non-zero setpoint
3. Check telemetry rate indicator

## Configuration Reference

### Environment Variables (.env)
```bash
ROBOT_IP=192.168.4.1      # Robot IP
ROBOT_PORT=12345          # Robot UDP port
LOCAL_LISTEN_PORT=12346   # Server port
TELEMETRY_RATE_HZ=50      # Update frequency
```

### Docker Commands
```bash
# Start
docker-compose up -d

# Stop
docker-compose down

# Logs
docker-compose logs -f

# Restart
docker-compose restart

# Rebuild
docker-compose build --no-cache
docker-compose up -d
```

## Files Overview

```
pid_body_tuning_ui/
├── app.py                 # Backend server
├── static/
│   ├── index.html         # Web interface
│   └── main.js           # Frontend logic
├── docker-compose.yml     # Docker config
├── requirements.txt       # Python deps
├── test_udp.py           # Robot simulator
└── README.md             # Full documentation
```

## Safety Checklist

✅ Clear operating area  
✅ E-Stop button accessible  
✅ Start with small setpoints  
✅ Test simulator first  
✅ Supervise robot during tuning  

## Next Steps

1. ✅ Get system running (you're here!)
2. 📖 Read [README.md](README.md) for details
3. 🔧 Read [FIRMWARE_INTEGRATION.md](FIRMWARE_INTEGRATION.md) for firmware integration
4. 📊 Analyze exported CSV data
5. 📝 Document your final PID values

## Support

- Full docs: `README.md`
- Firmware integration: `FIRMWARE_INTEGRATION.md`
- Main project: `../../README.md`

---

**Ready to tune!** 🎯

Start Docker, open browser to `localhost:8082`, and begin tuning.
