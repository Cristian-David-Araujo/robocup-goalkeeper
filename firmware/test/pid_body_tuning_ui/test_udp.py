"""
UDP Robot Simulator for Body PID Tuning
========================================

Simulates ESP32 robot responses for testing the body tuning UI without hardware.
Generates realistic body velocity telemetry with simulated PID behavior.
"""

import socket
import json
import time
import math
import random

# Configuration
LISTEN_PORT = 12345
SERVER_IP = "127.0.0.1"
SERVER_PORT = 12346

# Simulation parameters
UPDATE_RATE_HZ = 50
UPDATE_PERIOD = 1.0 / UPDATE_RATE_HZ

# Body dynamics simulation
class BodySimulator:
    def __init__(self):
        # Current state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # Setpoints
        self.vx_setpoint = 0.0
        self.vy_setpoint = 0.0
        self.wz_setpoint = 0.0
        
        # PID gains (initial values)
        self.vx_pid = {'kp': 1.0, 'ki': 0.1, 'kd': 0.05}
        self.vy_pid = {'kp': 1.0, 'ki': 0.1, 'kd': 0.05}
        self.wz_pid = {'kp': 1.0, 'ki': 0.1, 'kd': 0.05}
        
        # Integral and derivative terms
        self.vx_integral = 0.0
        self.vy_integral = 0.0
        self.wz_integral = 0.0
        
        self.vx_prev_error = 0.0
        self.vy_prev_error = 0.0
        self.wz_prev_error = 0.0
        
        # Control outputs
        self.vx_control = 0.0
        self.vy_control = 0.0
        self.wz_control = 0.0
        
        # Tuning active flag
        self.tuning_active = False
        
    def compute_pid(self, setpoint, measured, pid_gains, integral, prev_error, dt):
        """Compute PID control output"""
        error = setpoint - measured
        
        # Proportional
        p_term = pid_gains['kp'] * error
        
        # Integral with anti-windup
        integral += error * dt
        integral = max(min(integral, 10.0), -10.0)  # Clamp integral
        i_term = pid_gains['ki'] * integral
        
        # Derivative
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        d_term = pid_gains['kd'] * derivative
        
        # Total output
        output = p_term + i_term + d_term
        output = max(min(output, 10.0), -10.0)  # Saturation
        
        return output, integral, error
    
    def update(self, dt):
        """Update simulation state"""
        if not self.tuning_active:
            # Decay to zero when not tuning
            self.vx *= 0.95
            self.vy *= 0.95
            self.wz *= 0.95
            return
        
        # Compute PID outputs
        self.vx_control, self.vx_integral, self.vx_prev_error = self.compute_pid(
            self.vx_setpoint, self.vx, self.vx_pid, self.vx_integral, self.vx_prev_error, dt
        )
        
        self.vy_control, self.vy_integral, self.vy_prev_error = self.compute_pid(
            self.vy_setpoint, self.vy, self.vy_pid, self.vy_integral, self.vy_prev_error, dt
        )
        
        self.wz_control, self.wz_integral, self.wz_prev_error = self.compute_pid(
            self.wz_setpoint, self.wz, self.wz_pid, self.wz_integral, self.wz_prev_error, dt
        )
        
        # Simulate body dynamics (first-order system with time constant)
        tau = 0.2  # Time constant (200ms)
        alpha = dt / (tau + dt)
        
        # Update velocities towards control outputs (simulating motor response)
        self.vx += alpha * (self.vx_control - self.vx)
        self.vy += alpha * (self.vy_control - self.vy)
        self.wz += alpha * (self.wz_control - self.wz)
        
        # Add noise
        noise_level = 0.02
        self.vx += random.gauss(0, noise_level)
        self.vy += random.gauss(0, noise_level)
        self.wz += random.gauss(0, noise_level)
    
    def start_tuning(self, vx_sp, vy_sp, wz_sp):
        """Start tuning mode"""
        self.tuning_active = True
        self.vx_setpoint = vx_sp
        self.vy_setpoint = vy_sp
        self.wz_setpoint = wz_sp
        
        # Reset integrals
        self.vx_integral = 0.0
        self.vy_integral = 0.0
        self.wz_integral = 0.0
        
        print(f"Tuning started: VX={vx_sp}, VY={vy_sp}, WZ={wz_sp}")
    
    def stop_tuning(self):
        """Stop tuning mode"""
        self.tuning_active = False
        print("Tuning stopped")
    
    def update_pid_gains(self, axis, kp, ki, kd):
        """Update PID gains for specified axis"""
        if axis == 'vx':
            self.vx_pid = {'kp': kp, 'ki': ki, 'kd': kd}
            self.vx_integral = 0.0  # Reset integral
        elif axis == 'vy':
            self.vy_pid = {'kp': kp, 'ki': ki, 'kd': kd}
            self.vy_integral = 0.0
        elif axis == 'wz':
            self.wz_pid = {'kp': kp, 'ki': ki, 'kd': kd}
            self.wz_integral = 0.0
        
        print(f"Updated {axis.upper()} PID: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def update_setpoint(self, axis, setpoint):
        """Update setpoint for specified axis"""
        if axis == 'vx':
            self.vx_setpoint = setpoint
        elif axis == 'vy':
            self.vy_setpoint = setpoint
        elif axis == 'wz':
            self.wz_setpoint = setpoint
        
        print(f"Updated {axis.upper()} setpoint: {setpoint}")
    
    def emergency_stop(self):
        """Emergency stop"""
        self.tuning_active = False
        self.vx_setpoint = 0.0
        self.vy_setpoint = 0.0
        self.wz_setpoint = 0.0
        print("EMERGENCY STOP")

def main():
    print("=" * 60)
    print("Body PID Tuning - UDP Robot Simulator")
    print("=" * 60)
    print(f"Listening on port {LISTEN_PORT}")
    print(f"Sending telemetry to {SERVER_IP}:{SERVER_PORT}")
    print(f"Update rate: {UPDATE_RATE_HZ} Hz")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', LISTEN_PORT))
    sock.settimeout(0.01)  # 10ms timeout for non-blocking
    
    # Create simulator
    simulator = BodySimulator()
    
    last_update_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_update_time
            
            # Handle incoming commands
            try:
                data, addr = sock.recvfrom(4096)
                message = json.loads(data.decode('utf-8'))
                
                if message['type'] == 'tuning_command':
                    action = message.get('action')
                    
                    if action == 'start':
                        simulator.start_tuning(
                            message.get('vx_setpoint', 0.0),
                            message.get('vy_setpoint', 0.0),
                            message.get('wz_setpoint', 0.0)
                        )
                    
                    elif action == 'stop':
                        simulator.stop_tuning()
                    
                    elif action == 'update_pid':
                        simulator.update_pid_gains(
                            message.get('axis'),
                            message.get('kp', 0.0),
                            message.get('ki', 0.0),
                            message.get('kd', 0.0)
                        )
                    
                    elif action == 'set_setpoint':
                        simulator.update_setpoint(
                            message.get('axis'),
                            message.get('setpoint', 0.0)
                        )
                    
                    elif action == 'emergency_stop':
                        simulator.emergency_stop()
                
            except socket.timeout:
                pass
            except Exception as e:
                print(f"Error receiving command: {e}")
            
            # Update simulation at fixed rate
            if dt >= UPDATE_PERIOD:
                simulator.update(dt)
                last_update_time = current_time
                
                # Send telemetry
                telemetry = {
                    'type': 'body_telemetry',
                    'vx_setpoint': simulator.vx_setpoint,
                    'vx_measured': simulator.vx,
                    'vx_control': simulator.vx_control,
                    'vy_setpoint': simulator.vy_setpoint,
                    'vy_measured': simulator.vy,
                    'vy_control': simulator.vy_control,
                    'wz_setpoint': simulator.wz_setpoint,
                    'wz_measured': simulator.wz,
                    'wz_control': simulator.wz_control
                }
                
                try:
                    sock.sendto(
                        json.dumps(telemetry).encode('utf-8'),
                        (SERVER_IP, SERVER_PORT)
                    )
                except Exception as e:
                    print(f"Error sending telemetry: {e}")
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\nShutting down simulator...")
    finally:
        sock.close()
        print("Simulator stopped")

if __name__ == "__main__":
    main()
