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

# Last command time for timeout detection
last_command_time = None

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


# Global robot adapter instance
robot_adapter = RobotAdapter(ROBOT_IP, ROBOT_PORT)

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
        "command_timeout_sec": COMMAND_TIMEOUT_SEC
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
            "update_rate_hz": UPDATE_RATE_HZ
        })
        
        while True:
            # Receive velocity command from client
            data = await websocket.receive_json()
            
            # Validate and clamp velocities
            vx = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, data.get("vx", 0.0)))
            vy = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, data.get("vy", 0.0)))
            wz = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, data.get("wz", 0.0)))
            
            # Update current velocity
            current_velocity = {
                "vx": vx,
                "vy": vy,
                "wz": wz,
                "timestamp": datetime.now().isoformat()
            }
            last_command_time = datetime.now()
            
            # Send command to robot
            success = robot_adapter.send_command(vx, vy, wz)
            
            # Send acknowledgment back to client
            await websocket.send_json({
                "type": "ack",
                "success": success,
                "velocity": current_velocity
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
    logger.info(f"Teleoperation server starting on {HOST}:{PORT}")
    logger.info(f"Robot target: {ROBOT_IP}:{ROBOT_PORT}")
    logger.info(f"Max velocities: linear={MAX_LINEAR_VELOCITY} m/s, angular={MAX_ANGULAR_VELOCITY} rad/s")


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
    global current_velocity, last_command_time
    
    logger.info("Safety monitor started")
    
    while True:
        await asyncio.sleep(0.1)  # Check every 100ms
        
        if last_command_time is not None:
            time_since_last_command = (datetime.now() - last_command_time).total_seconds()
            
            if time_since_last_command > COMMAND_TIMEOUT_SEC:
                if current_velocity["vx"] != 0.0 or current_velocity["vy"] != 0.0 or current_velocity["wz"] != 0.0:
                    logger.warning(f"Command timeout ({time_since_last_command:.2f}s) - stopping robot")
                    
                    current_velocity = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "timestamp": datetime.now().isoformat()}
                    robot_adapter.stop()
                    last_command_time = None

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
