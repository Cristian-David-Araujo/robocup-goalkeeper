# Wheel PID Tuning - Quick Start

## 🚀 Start the Interface

```bash
cd test/pid_wheel_tuning_ui

# With Docker (Recommended)
docker-compose up -d

# Or with Python
python app.py
```

**Access**: http://localhost:8081

## 🎛️ Tuning Workflow

1. **Prepare Robot**
   - Mount on test stand (wheels off ground)
   - Connect to same network as computer
   - Verify IP address matches `.env` file

2. **Start Tuning Session**
   - Open UI in browser
   - Check "Robot: Connected" status
   - Click **"Start Tuning"**

3. **Configure PID**
   - Enter Kp, Ki, Kd values
   - Start conservative: `Kp=0.5, Ki=0.05, Kd=0.01`
   - Click **"Apply PID Constants"**

4. **Test Response**
   - Enter target velocity (e.g., `10.0` rad/s)
   - Click **"Send Set Point"**
   - Watch graphs: Blue=target, Green=measured, Orange=control

5. **Iterate**
   - Increase Kp until slight oscillation
   - Add Ki to eliminate steady-state error
   - Add Kd to reduce overshoot
   - Click **"Download Log"** to save data

6. **Stop**
   - Click **"Emergency Stop"** when done
   - Robot exits tuning mode

## 📊 Graph Interpretation

**Top Row** (Velocity):
- Blue dashed line = Target setpoint
- Green solid line = Measured velocity
- Good tracking = lines overlap

**Bottom Row** (Control):
- Orange line = PWM output (%)
- Should be smooth, not oscillating wildly

## ⚙️ Docker Commands

```bash
# Start
docker-compose up -d

# View logs
docker-compose logs -f

# Stop
docker-compose down

# Rebuild after changes
docker-compose up -d --build
```

## 🔧 Configuration

Edit `.env` file:
```bash
ROBOT_IP=192.168.1.100      # Robot IP address
ROBOT_PORT=3333             # Robot UDP port
SAMPLING_RATE_HZ=50         # Telemetry rate
```

## 🆘 Troubleshooting

**Robot not connecting:**
- Check `ROBOT_IP` in `.env`
- Ping robot: `ping 192.168.1.100`
- Verify robot WiFi is active
- Check firewall allows UDP port 3333

**No telemetry data:**
- Ensure tuning mode is active (click "Start Tuning")
- Check firmware has telemetry streaming enabled
- View server logs: `docker-compose logs -f`

**Graphs not updating:**
- Check WebSocket connection (should show "Connected")
- Refresh browser page
- Check browser console for errors (F12)

## 🛡️ Safety

- ⚠️ Always secure robot on test stand
- ⚠️ Emergency stop halts all wheels immediately
- ⚠️ Connection loss triggers automatic stop
- ⚠️ Start with low setpoints (< 5 rad/s)

## 📝 PID Tuning Tips

1. **Start Conservative**: Low gains prevent instability
2. **One at a Time**: Adjust Kp first, then Ki, then Kd
3. **Small Steps**: Change by 10-20% increments
4. **Watch Control Output**: Should not saturate (±100%)
5. **Test Different Setpoints**: Verify at low and high speeds
6. **Save Data**: Download logs for offline analysis

## 📞 Support

See full documentation:
- `README.md` - Complete guide
- `FIRMWARE_INTEGRATION.md` - Firmware changes
- `IMPLEMENTATION_SUMMARY.md` - Architecture overview
