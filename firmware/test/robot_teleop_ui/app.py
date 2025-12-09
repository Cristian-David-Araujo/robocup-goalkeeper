"""
Robot Teleoperation Web UI
==========================

A real-time web-based teleoperation interface for controlling robots via keyboard.
Uses WebSocket for low-latency command streaming.

Author: RoboCup Goalkeeper Team
License: MIT
"""

import asyncio
import json
import logging
import os
import socket
from datetime import datetime
from typing import Dict, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse

# =============================================================================
# CONFIGURATION
# =============================================================================

# Server configuration
HOST = os.getenv("TELEOP_HOST", "0.0.0.0")
PORT = int(os.getenv("TELEOP_PORT", "8080"))

# Robot velocity limits (m/s and rad/s)
MAX_LINEAR_VELOCITY = float(os.getenv("MAX_LINEAR_VEL", "1.0"))
MAX_ANGULAR_VELOCITY = float(os.getenv("MAX_ANGULAR_VEL", "2.0"))

# Robot communication
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.1.100")
ROBOT_PORT = int(os.getenv("ROBOT_PORT", "3333"))

# Safety configuration
COMMAND_TIMEOUT_SEC = float(os.getenv("COMMAND_TIMEOUT", "1.0"))
UPDATE_RATE_HZ = int(os.getenv("UPDATE_RATE_HZ", "20"))

# Velocity ramping (gradual acceleration)
MAX_ACCELERATION = float(os.getenv("MAX_ACCELERATION", "2.0"))  # m/s² or rad/s²
RAMP_ENABLED = os.getenv("RAMP_ENABLED", "true").lower() == "true"

# =============================================================================
# LOGGING SETUP
# =============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# =============================================================================
# FASTAPI APPLICATION
# =============================================================================

app = FastAPI(
    title="Robot Teleoperation UI",
    description="Real-time keyboard control for omnidirectional robots",
    version="1.0.0"
)

# Mount static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# =============================================================================
# GLOBAL STATE
# =============================================================================

# Active WebSocket connections
active_connections: Set[WebSocket] = set()

# Current velocity command
current_velocity = {
    "vx": 0.0,
    "vy": 0.0,
    "wz": 0.0,
    "timestamp": None
}

# Target velocity (what user wants)
target_velocity = {
    "vx": 0.0,
    "vy": 0.0,
    "wz": 0.0
}

# Last command time for timeout detection
last_command_time = None

# Robot connection status
robot_connected = False
last_robot_response_time = None

# =============================================================================
# ROBOT COMMUNICATION
# =============================================================================

class RobotAdapter:
    """
    Adapter for sending velocity commands to the robot.
    
    This class handles the communication protocol with the robot.
    Modify the send_command method to match your robot's interface.
    """
    
    def __init__(self, robot_ip: str, robot_port: int):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)  # 500ms timeout for connection check
        self.last_send_time = datetime.now()
        logger.info(f"Robot adapter initialized for {robot_ip}:{robot_port}")
    
    def send_command(self, vx: float, vy: float, wz: float) -> bool:
        """
        Send velocity command to robot via UDP.
        
        Args:
            vx: Linear velocity X (m/s)
            vy: Linear velocity Y (m/s)
            wz: Angular velocity Z (rad/s)
        
        Returns:
            True if command sent successfully, False otherwise
        """
        try:
            # Format: JSON command matching robot's WiFi control protocol
            command = {
                "vx": round(vx, 3),
                "vy": round(vy, 3),
                "wz": round(wz, 3)
            }
            
            message = json.dumps(command).encode('utf-8')
            self.sock.sendto(message, (self.robot_ip, self.robot_port))
            
            logger.debug(f"Sent command: {command}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False
    
    def stop(self):
        """Send stop command (zero velocities)."""
        return self.send_command(0.0, 0.0, 0.0)
    
    def close(self):
        """Close the UDP socket."""
        self.sock.close()
    
    def check_connection(self) -> bool:
        """
        Check if robot is reachable.
        
        Returns:
            True if robot is responsive, False otherwise
        """
        try:
            # Send a ping command (zero velocity as heartbeat)
            test_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
            message = json.dumps(test_cmd).encode('utf-8')
            self.sock.sendto(message, (self.robot_ip, self.robot_port))
            return True
        except Exception as e:
            logger.debug(f"Connection check failed: {e}")
            return False


# Global robot adapter instance
robot_adapter = RobotAdapter(ROBOT_IP, ROBOT_PORT)

# =============================================================================
# VELOCITY RAMPING
# =============================================================================

def apply_velocity_ramp(current: Dict[str, float], target: Dict[str, float], dt: float) -> Dict[str, float]:
    """
    Apply gradual velocity ramping to avoid sudden movements.
    
    Special behavior: Angular velocity (wz) goes directly to 0 when stopping,
    without ramping, for immediate rotation stop.
    
    Args:
        current: Current velocity {vx, vy, wz}
        target: Target velocity {vx, vy, wz}
        dt: Time delta in seconds
    
    Returns:
        New velocity after applying acceleration limits
    """
    if not RAMP_ENABLED:
        return target
    
    max_delta = MAX_ACCELERATION * dt
    new_velocity = {}
    
    for key in ['vx', 'vy', 'wz']:
        # Special case: wz goes directly to 0 without ramping when stopping
        if key == 'wz' and target[key] == 0.0:
            new_velocity[key] = 0.0
            continue
        
        delta = target[key] - current[key]
        
        if abs(delta) <= max_delta:
            new_velocity[key] = target[key]
        else:
            new_velocity[key] = current[key] + (max_delta if delta > 0 else -max_delta)
    
    return new_velocity

# =============================================================================
# API ENDPOINTS
# =============================================================================

@app.get("/", response_class=HTMLResponse)
async def get_index():
    """Serve the main teleoperation UI."""
    with open("static/index.html", "r") as f:
        return HTMLResponse(content=f.read())


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return JSONResponse({
        "status": "healthy",
        "active_connections": len(active_connections),
        "current_velocity": current_velocity,
        "config": {
            "max_linear_vel": MAX_LINEAR_VELOCITY,
            "max_angular_vel": MAX_ANGULAR_VELOCITY,
            "robot_ip": ROBOT_IP,
            "robot_port": ROBOT_PORT,
            "update_rate_hz": UPDATE_RATE_HZ
        }
    })


@app.get("/api/config")
async def get_config():
    """Get current configuration."""
    return JSONResponse({
        "max_linear_velocity": MAX_LINEAR_VELOCITY,
        "max_angular_velocity": MAX_ANGULAR_VELOCITY,
        "update_rate_hz": UPDATE_RATE_HZ,
        "command_timeout_sec": COMMAND_TIMEOUT_SEC,
        "max_acceleration": MAX_ACCELERATION,
        "ramp_enabled": RAMP_ENABLED,
        "robot_connected": robot_connected
    })


@app.post("/api/stop")
async def emergency_stop():
    """Emergency stop endpoint - immediately sends zero velocities."""
    global current_velocity, last_command_time
    
    current_velocity = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "timestamp": datetime.now().isoformat()}
    last_command_time = None
    
    robot_adapter.stop()
    
    logger.warning("EMERGENCY STOP triggered")
    return JSONResponse({"status": "stopped", "velocity": current_velocity})

# =============================================================================
# WEBSOCKET ENDPOINT
# =============================================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time velocity commands.
    
    Receives JSON messages with velocity commands:
    {
        "vx": float,  // Linear X velocity (m/s)
        "vy": float,  // Linear Y velocity (m/s)
        "wz": float   // Angular Z velocity (rad/s)
    }
    """
    global current_velocity, last_command_time
    
    await websocket.accept()
    active_connections.add(websocket)
    client_id = id(websocket)
    
    logger.info(f"Client {client_id} connected. Active connections: {len(active_connections)}")
    
    try:
        # Send initial config to client
        await websocket.send_json({
            "type": "config",
            "max_linear_velocity": MAX_LINEAR_VELOCITY,
            "max_angular_velocity": MAX_ANGULAR_VELOCITY,
            "update_rate_hz": UPDATE_RATE_HZ,
            "max_acceleration": MAX_ACCELERATION,
            "ramp_enabled": RAMP_ENABLED,
            "robot_connected": robot_connected
        })
        
        while True:
            # Receive velocity command from client
            data = await websocket.receive_json()
            
            # Validate and clamp target velocities
            target_velocity["vx"] = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, data.get("vx", 0.0)))
            target_velocity["vy"] = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, data.get("vy", 0.0)))
            target_velocity["wz"] = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, data.get("wz", 0.0)))
            
            # Apply velocity ramping
            dt = 1.0 / UPDATE_RATE_HZ
            ramped_velocity = apply_velocity_ramp(
                {"vx": current_velocity["vx"], "vy": current_velocity["vy"], "wz": current_velocity["wz"]},
                target_velocity,
                dt
            )
            
            # Update current velocity
            current_velocity = {
                "vx": ramped_velocity["vx"],
                "vy": ramped_velocity["vy"],
                "wz": ramped_velocity["wz"],
                "timestamp": datetime.now().isoformat()
            }
            last_command_time = datetime.now()
            
            # Send command to robot
            success = robot_adapter.send_command(
                current_velocity["vx"],
                current_velocity["vy"],
                current_velocity["wz"]
            )
            
            # Send acknowledgment back to client
            await websocket.send_json({
                "type": "ack",
                "success": success,
                "velocity": current_velocity,
                "target_velocity": target_velocity,
                "robot_connected": robot_connected
            })
    
    except WebSocketDisconnect:
        logger.info(f"Client {client_id} disconnected")
    
    except Exception as e:
        logger.error(f"WebSocket error for client {client_id}: {e}")
    
    finally:
        # Clean up connection
        active_connections.discard(websocket)
        
        # Safety: Stop robot if no more active connections
        if len(active_connections) == 0:
            logger.warning("No active connections - sending stop command")
            robot_adapter.stop()
            current_velocity = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "timestamp": datetime.now().isoformat()}
            last_command_time = None
        
        logger.info(f"Client {client_id} cleanup complete. Active connections: {len(active_connections)}")

# =============================================================================
# BACKGROUND TASKS
# =============================================================================

@app.on_event("startup")
async def startup_event():
    """Initialize background tasks on startup."""
    asyncio.create_task(safety_monitor())
    asyncio.create_task(robot_connection_monitor())
    logger.info(f"Teleoperation server starting on {HOST}:{PORT}")
    logger.info(f"Robot target: {ROBOT_IP}:{ROBOT_PORT}")
    logger.info(f"Max velocities: linear={MAX_LINEAR_VELOCITY} m/s, angular={MAX_ANGULAR_VELOCITY} rad/s")
    logger.info(f"Velocity ramping: {RAMP_ENABLED} (max accel: {MAX_ACCELERATION})")


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up on shutdown."""
    logger.info("Shutting down - sending stop command to robot")
    robot_adapter.stop()
    robot_adapter.close()


async def safety_monitor():
    """
    Background task to monitor command timeout.
    
    If no commands received within COMMAND_TIMEOUT_SEC, sends stop command.
    """
    global current_velocity, target_velocity, last_command_time
    
    logger.info("Safety monitor started")
    
    while True:
        await asyncio.sleep(0.1)  # Check every 100ms
        
        if last_command_time is not None:
            time_since_last_command = (datetime.now() - last_command_time).total_seconds()
            
            if time_since_last_command > COMMAND_TIMEOUT_SEC:
                if current_velocity["vx"] != 0.0 or current_velocity["vy"] != 0.0 or current_velocity["wz"] != 0.0:
                    logger.warning(f"Command timeout ({time_since_last_command:.2f}s) - stopping robot")
                    
                    current_velocity = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "timestamp": datetime.now().isoformat()}
                    target_velocity = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                    robot_adapter.stop()
                    last_command_time = None


async def robot_connection_monitor():
    """
    Background task to monitor robot connection status.
    
    Periodically checks if robot is responsive.
    """
    global robot_connected, last_robot_response_time
    
    logger.info("Robot connection monitor started")
    
    while True:
        await asyncio.sleep(2.0)  # Check every 2 seconds
        
        try:
            # Attempt to check connection
            is_connected = robot_adapter.check_connection()
            
            if is_connected:
                if not robot_connected:
                    logger.info("Robot connection established")
                robot_connected = True
                last_robot_response_time = datetime.now()
            else:
                if robot_connected:
                    logger.warning("Robot connection lost")
                robot_connected = False
        
        except Exception as e:
            logger.error(f"Connection monitor error: {e}")
            robot_connected = False

# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    import uvicorn
    
    uvicorn.run(
        app,
        host=HOST,
        port=PORT,
        log_level="info"
    )
