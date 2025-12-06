"""
Test script for robot teleoperation UI

Tests the UDP communication and basic functionality without needing a real robot.
"""

import socket
import json
import time
from datetime import datetime

def test_udp_listener(port=3333, duration=30):
    """
    Listen for UDP commands on the specified port.
    Simulates the robot's receiver for testing the UI.
    
    Args:
        port: UDP port to listen on
        duration: How long to listen (seconds)
    """
    print(f"Starting UDP listener on port {port}")
    print(f"Will listen for {duration} seconds")
    print("Open the web UI and send commands...\n")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(1.0)  # 1 second timeout for checking duration
    
    start_time = time.time()
    command_count = 0
    last_command = None
    
    try:
        while time.time() - start_time < duration:
            try:
                data, addr = sock.recvfrom(1024)
                command_count += 1
                
                try:
                    cmd = json.loads(data.decode('utf-8'))
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    # Only print if command changed
                    if cmd != last_command:
                        print(f"[{timestamp}] From {addr[0]}:{addr[1]}")
                        print(f"  vx: {cmd.get('vx', 0):+.3f} m/s")
                        print(f"  vy: {cmd.get('vy', 0):+.3f} m/s")
                        print(f"  wz: {cmd.get('wz', 0):+.3f} rad/s")
                        print(f"  Total commands: {command_count}\n")
                        last_command = cmd
                    
                except json.JSONDecodeError:
                    print(f"[ERROR] Invalid JSON: {data}")
                    
            except socket.timeout:
                continue
                
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    finally:
        sock.close()
        elapsed = time.time() - start_time
        print(f"\n{'='*50}")
        print(f"Test Summary:")
        print(f"  Duration: {elapsed:.1f} seconds")
        print(f"  Total commands received: {command_count}")
        if command_count > 0:
            print(f"  Average rate: {command_count/elapsed:.1f} commands/sec")
        print(f"{'='*50}")


def test_send_commands(robot_ip='127.0.0.1', robot_port=3333, count=10):
    """
    Send test commands to the robot.
    Useful for testing without the web UI.
    
    Args:
        robot_ip: Robot's IP address
        robot_port: Robot's UDP port
        count: Number of commands to send
    """
    print(f"Sending {count} test commands to {robot_ip}:{robot_port}\n")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    test_commands = [
        {"vx": 0.5, "vy": 0.0, "wz": 0.0},   # Forward
        {"vx": -0.5, "vy": 0.0, "wz": 0.0},  # Backward
        {"vx": 0.0, "vy": 0.5, "wz": 0.0},   # Right
        {"vx": 0.0, "vy": -0.5, "wz": 0.0},  # Left
        {"vx": 0.0, "vy": 0.0, "wz": 1.0},   # Rotate right
        {"vx": 0.0, "vy": 0.0, "wz": -1.0},  # Rotate left
        {"vx": 0.5, "vy": 0.5, "wz": 0.0},   # Diagonal
        {"vx": 0.0, "vy": 0.0, "wz": 0.0},   # Stop
    ]
    
    try:
        for i in range(count):
            cmd = test_commands[i % len(test_commands)]
            message = json.dumps(cmd).encode('utf-8')
            
            sock.sendto(message, (robot_ip, robot_port))
            print(f"Sent: {cmd}")
            
            time.sleep(0.5)
            
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        sock.close()
        print("\nTest completed")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "send":
        # Send test commands
        robot_ip = sys.argv[2] if len(sys.argv) > 2 else '127.0.0.1'
        test_send_commands(robot_ip=robot_ip)
    else:
        # Listen for commands (default)
        test_udp_listener()
