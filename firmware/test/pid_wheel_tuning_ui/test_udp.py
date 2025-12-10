"""
UDP Telemetry Simulator for Wheel PID Tuning Interface

Simulates robot telemetry for testing the UI without a real robot.
Generates realistic wheel velocity responses with PID-like behavior.
"""

import socket
import json
import time
import math
import random

# Configuration
SERVER_IP = "127.0.0.1"  # Send to localhost
SERVER_PORT = 8081  # This should match the port where app.py is listening
ROBOT_PORT = 3333  # Robot's listening port (not used in simulator)
TELEMETRY_RATE_HZ = 50

# Simulation state
wheel_state = {
    1: {"velocity": 0.0, "setpoint": 0.0, "control": 0.0},
    2: {"velocity": 0.0, "setpoint": 0.0, "control": 0.0},
    3: {"velocity": 0.0, "setpoint": 0.0, "control": 0.0}
}

# PID parameters (simulated)
pid_params = {"kp": 1.0, "ki": 0.1, "kd": 0.05}

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', ROBOT_PORT))
sock.settimeout(0.01)  # Non-blocking with short timeout

print(f"UDP Telemetry Simulator Started")
print(f"Listening on port {ROBOT_PORT} for commands")
print(f"Sending telemetry to {SERVER_IP}:{SERVER_PORT}")
print(f"Telemetry rate: {TELEMETRY_RATE_HZ} Hz")
print("-" * 50)

def simulate_wheel_response(wheel_id, dt):
    """
    Simulate realistic wheel velocity response to setpoint.
    Includes inertia, friction, and PID control behavior.
    """
    wheel = wheel_state[wheel_id]
    
    # Error
    error = wheel["setpoint"] - wheel["velocity"]
    
    # Simulated PID control (simplified)
    control = pid_params["kp"] * error
    control = max(-100, min(100, control))  # Clamp to PWM range
    
    wheel["control"] = control
    
    # Simulate motor dynamics (first-order system with noise)
    tau = 0.1  # Time constant
    acceleration = (control * 0.5 - wheel["velocity"]) / tau
    
    # Add realistic noise
    noise = random.gauss(0, 0.1)
    
    # Update velocity
    wheel["velocity"] += (acceleration * dt) + noise
    
    # Add friction/damping
    wheel["velocity"] *= 0.99

def process_command(data):
    """Process incoming commands from UI."""
    try:
        cmd = json.loads(data.decode('utf-8'))
        
        if cmd.get("type") == "tuning_command":
            command = cmd.get("command")
            
            if command == "start_tuning":
                print("✓ Tuning mode activated")
                
            elif command == "stop_tuning":
                print("✓ Tuning mode deactivated")
                for wheel_id in wheel_state:
                    wheel_state[wheel_id]["setpoint"] = 0.0
                    
            elif command == "apply_pid":
                params = cmd.get("params", {})
                pid_params["kp"] = params.get("kp", pid_params["kp"])
                pid_params["ki"] = params.get("ki", pid_params["ki"])
                pid_params["kd"] = params.get("kd", pid_params["kd"])
                print(f"✓ PID updated: Kp={pid_params['kp']}, Ki={pid_params['ki']}, Kd={pid_params['kd']}")
                
            elif command == "set_setpoint":
                params = cmd.get("params", {})
                setpoint = params.get("setpoint", 0.0)
                for wheel_id in wheel_state:
                    wheel_state[wheel_id]["setpoint"] = setpoint
                print(f"✓ Setpoint updated: {setpoint} rad/s")
                
            elif command == "emergency_stop":
                print("⚠ EMERGENCY STOP")
                for wheel_id in wheel_state:
                    wheel_state[wheel_id]["setpoint"] = 0.0
                    wheel_state[wheel_id]["velocity"] = 0.0
                    wheel_state[wheel_id]["control"] = 0.0
                    
    except json.JSONDecodeError:
        pass  # Ignore malformed commands
    except Exception as e:
        print(f"Error processing command: {e}")

def send_telemetry():
    """Send simulated telemetry to server."""
    telemetry = {
        "type": "wheel_telemetry",
        "timestamp": int(time.time() * 1000),
        "wheels": [
            {
                "id": wheel_id,
                "velocity": round(wheel["velocity"], 3),
                "setpoint": round(wheel["setpoint"], 3),
                "control": round(wheel["control"], 3)
            }
            for wheel_id, wheel in wheel_state.items()
        ]
    }
    
    message = json.dumps(telemetry).encode('utf-8')
    sock.sendto(message, (SERVER_IP, SERVER_PORT))

# Main simulation loop
interval = 1.0 / TELEMETRY_RATE_HZ
last_time = time.time()
iteration = 0

try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        
        # Check for incoming commands
        try:
            data, addr = sock.recvfrom(1024)
            process_command(data)
        except socket.timeout:
            pass
        
        # Simulate wheel dynamics
        for wheel_id in wheel_state:
            simulate_wheel_response(wheel_id, dt)
        
        # Send telemetry
        send_telemetry()
        
        # Status update
        iteration += 1
        if iteration % 50 == 0:
            print(f"[{iteration}] W1: {wheel_state[1]['velocity']:6.2f} → {wheel_state[1]['setpoint']:6.2f} rad/s")
        
        # Maintain sampling rate
        last_time = current_time
        sleep_time = max(0, interval - (time.time() - current_time))
        time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n\nSimulator stopped by user")
    sock.close()
