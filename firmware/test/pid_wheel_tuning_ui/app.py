"""
Wheel PID Tuning Interface - Backend Server
===========================================

A real-time web interface for tuning individual wheel PID controllers.
Provides telemetry streaming and command interface for PID parameter adjustment.

Author: RoboCup Goalkeeper Team
License: MIT
"""

import asyncio
import json
import logging
import os
import socket
from datetime import datetime
from typing import Dict, Set, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse

# =============================================================================
# CONFIGURATION
# =============================================================================

# Server configuration
HOST = os.getenv("PID_TUNING_HOST", "0.0.0.0")
PORT = int(os.getenv("PID_TUNING_PORT", "8081"))

# Robot communication
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.1.100")
ROBOT_PORT = int(os.getenv("ROBOT_PORT", "3333"))

# Tuning configuration
SAMPLING_RATE_HZ = int(os.getenv("SAMPLING_RATE_HZ", "50"))
MAX_WHEEL_VELOCITY = float(os.getenv("MAX_WHEEL_VEL", "50.0"))  # rad/s
MAX_CONTROL_OUTPUT = float(os.getenv("MAX_CONTROL", "100.0"))  # PWM %

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
    title="Wheel PID Tuning Interface",
    description="Real-time PID tuning for individual wheel controllers",
    version="1.0.0"
)

# Mount static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# =============================================================================
# GLOBAL STATE
# =============================================================================

# Active WebSocket connections
active_connections: Set[WebSocket] = set()

# Tuning state
tuning_active = False
robot_connected = False
last_robot_response_time = None

# Current PID parameters
current_pid = {
    "kp": 1.0,
    "ki": 0.1,
    "kd": 0.05
}

# Current setpoint (target velocity for all wheels)
current_setpoint = 0.0

# Latest wheel telemetry
wheel_telemetry = {
    "wheel1": {"velocity": 0.0, "setpoint": 0.0, "control": 0.0},
    "wheel2": {"velocity": 0.0, "setpoint": 0.0, "control": 0.0},
    "wheel3": {"velocity": 0.0, "setpoint": 0.0, "control": 0.0}
}

# =============================================================================
# ROBOT COMMUNICATION
# =============================================================================

class WheelTuningAdapter:
    """
    Adapter for communicating with robot during wheel PID tuning.
    
    Protocol:
    - Sends commands via UDP (JSON format)
    - Receives telemetry feedback via UDP
    """
    
    def __init__(self, robot_ip: str, robot_port: int):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', 0))  # Bind to any available port
        self.sock.settimeout(0.5)
        
        local_port = self.sock.getsockname()[1]
        logger.info(f"Wheel tuning adapter initialized for {robot_ip}:{robot_port}, listening on port {local_port}")
    
    def send_command(self, command: Dict) -> bool:
        """
        Send command to robot.
        
        Command format:
        {
            "type": "tuning_command",
            "command": "start_tuning" | "stop_tuning" | "apply_pid" | "set_setpoint" | "emergency_stop",
            "params": {...}
        }
        """
        try:
            message = json.dumps(command).encode('utf-8')
            bytes_sent = self.sock.sendto(message, (self.robot_ip, self.robot_port))
            logger.debug(f"Sent command: {command} ({bytes_sent} bytes)")
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False
    
    def start_tuning(self) -> bool:
        """Enable wheel tuning mode on robot."""
        return self.send_command({
            "type": "tuning_command",
            "command": "start_tuning",
            "mode": "wheel"
        })
    
    def stop_tuning(self) -> bool:
        """Disable wheel tuning mode on robot."""
        return self.send_command({
            "type": "tuning_command",
            "command": "stop_tuning"
        })
    
    def apply_pid(self, kp: float, ki: float, kd: float) -> bool:
        """Apply PID constants to all wheels."""
        return self.send_command({
            "type": "tuning_command",
            "command": "apply_pid",
            "params": {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "wheels": "all"  # Apply to all three wheels
            }
        })
    
    def set_setpoint(self, setpoint: float) -> bool:
        """Set target velocity for all wheels."""
        return self.send_command({
            "type": "tuning_command",
            "command": "set_setpoint",
            "params": {
                "setpoint": setpoint,
                "wheels": "all"  # Apply to all three wheels
            }
        })
    
    def emergency_stop(self) -> bool:
        """Emergency stop - halt all wheels."""
        return self.send_command({
            "type": "tuning_command",
            "command": "emergency_stop"
        })
    
    def close(self):
        """Close the UDP socket."""
        self.sock.close()


# Global adapter instance
robot_adapter = WheelTuningAdapter(ROBOT_IP, ROBOT_PORT)

# =============================================================================
# API ENDPOINTS
# =============================================================================

@app.get("/", response_class=HTMLResponse)
async def get_index():
    """Serve the main tuning UI."""
    with open("static/index.html", "r") as f:
        return HTMLResponse(content=f.read())


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return JSONResponse({
        "status": "healthy",
        "active_connections": len(active_connections),
        "tuning_active": tuning_active,
        "robot_connected": robot_connected,
        "config": {
            "sampling_rate_hz": SAMPLING_RATE_HZ,
            "max_wheel_velocity": MAX_WHEEL_VELOCITY,
            "robot_ip": ROBOT_IP,
            "robot_port": ROBOT_PORT
        }
    })


@app.get("/api/config")
async def get_config():
    """Get current configuration."""
    return JSONResponse({
        "sampling_rate_hz": SAMPLING_RATE_HZ,
        "max_wheel_velocity": MAX_WHEEL_VELOCITY,
        "max_control_output": MAX_CONTROL_OUTPUT,
        "robot_connected": robot_connected,
        "tuning_active": tuning_active,
        "current_pid": current_pid,
        "current_setpoint": current_setpoint
    })

# =============================================================================
# WEBSOCKET ENDPOINT
# =============================================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time tuning interface.
    
    Receives commands:
    - start_tuning: Enable tuning mode
    - stop_tuning: Disable tuning mode
    - apply_pid: Update PID constants
    - set_setpoint: Update target velocity
    - emergency_stop: Halt all wheels
    
    Sends telemetry:
    - config: Initial configuration
    - telemetry: Real-time wheel data
    - ack: Command acknowledgments
    """
    global tuning_active, current_pid, current_setpoint
    
    await websocket.accept()
    active_connections.add(websocket)
    client_id = id(websocket)
    
    logger.info(f"Client {client_id} connected. Active connections: {len(active_connections)}")
    
    try:
        # Send initial config
        await websocket.send_json({
            "type": "config",
            "sampling_rate_hz": SAMPLING_RATE_HZ,
            "max_wheel_velocity": MAX_WHEEL_VELOCITY,
            "max_control_output": MAX_CONTROL_OUTPUT,
            "robot_connected": robot_connected
        })
        
        while True:
            # Receive command from client
            data = await websocket.receive_json()
            command = data.get("command")
            
            if command == "start_tuning":
                success = robot_adapter.start_tuning()
                tuning_active = success
                
                await websocket.send_json({
                    "type": "ack",
                    "command": "start_tuning",
                    "success": success,
                    "error": None if success else "Failed to enable tuning mode"
                })
                
                if success:
                    logger.info("Tuning mode activated")
                
            elif command == "pause_tuning":
                # For pause, we just stop sending setpoints but keep tuning active
                await websocket.send_json({
                    "type": "ack",
                    "command": "pause_tuning",
                    "success": True
                })
                logger.info("Tuning paused (setpoints cleared)")
                
            elif command == "stop_tuning":
                success = robot_adapter.stop_tuning()
                tuning_active = False
                
                await websocket.send_json({
                    "type": "ack",
                    "command": "stop_tuning",
                    "success": success
                })
                logger.info("Tuning mode deactivated")
                
            elif command == "apply_pid":
                kp = data.get("kp", current_pid["kp"])
                ki = data.get("ki", current_pid["ki"])
                kd = data.get("kd", current_pid["kd"])
                
                # Validate
                if kp < 0 or ki < 0 or kd < 0:
                    await websocket.send_json({
                        "type": "ack",
                        "command": "apply_pid",
                        "success": False,
                        "error": "PID constants must be non-negative"
                    })
                    continue
                
                success = robot_adapter.apply_pid(kp, ki, kd)
                
                if success:
                    current_pid = {"kp": kp, "ki": ki, "kd": kd}
                
                await websocket.send_json({
                    "type": "ack",
                    "command": "apply_pid",
                    "success": success,
                    "error": None if success else "Failed to apply PID constants"
                })
                
                logger.info(f"PID constants updated: Kp={kp}, Ki={ki}, Kd={kd}")
                
            elif command == "set_setpoint":
                setpoint = data.get("setpoint", 0.0)
                
                # Validate
                if abs(setpoint) > MAX_WHEEL_VELOCITY:
                    await websocket.send_json({
                        "type": "ack",
                        "command": "set_setpoint",
                        "success": False,
                        "error": f"Setpoint exceeds maximum velocity ({MAX_WHEEL_VELOCITY} rad/s)"
                    })
                    continue
                
                success = robot_adapter.set_setpoint(setpoint)
                
                if success:
                    current_setpoint = setpoint
                
                await websocket.send_json({
                    "type": "ack",
                    "command": "set_setpoint",
                    "success": success,
                    "error": None if success else "Failed to set setpoint"
                })
                
                logger.info(f"Setpoint updated: {setpoint} rad/s")
                
            elif command == "emergency_stop":
                robot_adapter.emergency_stop()
                tuning_active = False
                current_setpoint = 0.0
                
                await websocket.send_json({
                    "type": "ack",
                    "command": "emergency_stop",
                    "success": True
                })
                
                logger.warning("EMERGENCY STOP triggered")
    
    except WebSocketDisconnect:
        logger.info(f"Client {client_id} disconnected")
    
    except Exception as e:
        logger.error(f"WebSocket error for client {client_id}: {e}")
    
    finally:
        active_connections.discard(websocket)
        
        # Safety: Stop tuning if no active connections
        if len(active_connections) == 0 and tuning_active:
            logger.warning("No active connections - disabling tuning mode")
            robot_adapter.stop_tuning()
            tuning_active = False
        
        logger.info(f"Client {client_id} cleanup complete. Active connections: {len(active_connections)}")

# =============================================================================
# BACKGROUND TASKS
# =============================================================================

@app.on_event("startup")
async def startup_event():
    """Initialize background tasks on startup."""
    asyncio.create_task(telemetry_broadcaster())
    asyncio.create_task(udp_telemetry_listener())
    asyncio.create_task(robot_connection_monitor())
    logger.info(f"Wheel PID Tuning server starting on {HOST}:{PORT}")
    logger.info(f"Robot target: {ROBOT_IP}:{ROBOT_PORT}")
    logger.info(f"Sampling rate: {SAMPLING_RATE_HZ} Hz")


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up on shutdown."""
    logger.info("Shutting down - stopping tuning mode")
    if tuning_active:
        robot_adapter.stop_tuning()
    robot_adapter.close()


async def telemetry_broadcaster():
    """
    Broadcast wheel telemetry to all connected clients.
    Runs at configured sampling rate.
    """
    logger.info("Telemetry broadcaster started")
    interval = 1.0 / SAMPLING_RATE_HZ
    
    while True:
        await asyncio.sleep(interval)
        
        if len(active_connections) == 0:
            continue
        
        # Prepare telemetry message
        telemetry_msg = {
            "type": "telemetry",
            "timestamp": datetime.now().isoformat(),
            "wheels": wheel_telemetry,
            "robot_connected": robot_connected
        }
        
        # Broadcast to all clients
        disconnected = set()
        for websocket in active_connections:
            try:
                await websocket.send_json(telemetry_msg)
            except Exception as e:
                logger.error(f"Error sending telemetry: {e}")
                disconnected.add(websocket)
        
        # Clean up disconnected clients
        active_connections.difference_update(disconnected)


async def udp_telemetry_listener():
    """
    Listen for UDP telemetry feedback from robot.
    
    Expected format:
    {
        "type": "wheel_telemetry",
        "wheels": [
            {"id": 1, "velocity": float, "setpoint": float, "control": float},
            {"id": 2, "velocity": float, "setpoint": float, "control": float},
            {"id": 3, "velocity": float, "setpoint": float, "control": float}
        ]
    }
    """
    global wheel_telemetry, robot_connected, last_robot_response_time
    
    logger.info("UDP telemetry listener started")
    
    feedback_socket = robot_adapter.sock
    feedback_socket.setblocking(False)
    
    local_addr = feedback_socket.getsockname()
    logger.info(f"Listening for telemetry on {local_addr[0]}:{local_addr[1]}")
    
    telemetry_count = 0
    
    while True:
        try:
            data, addr = feedback_socket.recvfrom(2048)
            
            if telemetry_count == 0:
                logger.info(f"First telemetry received from {addr}")
            
            try:
                telemetry = json.loads(data.decode('utf-8'))
                
                if telemetry.get("type") == "wheel_telemetry" and "wheels" in telemetry:
                    # Parse wheel data
                    for wheel_data in telemetry["wheels"]:
                        wheel_id = wheel_data.get("id")
                        if wheel_id in [1, 2, 3]:
                            wheel_key = f"wheel{wheel_id}"
                            wheel_telemetry[wheel_key] = {
                                "velocity": wheel_data.get("velocity", 0.0),
                                "setpoint": wheel_data.get("setpoint", 0.0),
                                "control": wheel_data.get("control", 0.0)
                            }
                    
                    last_robot_response_time = datetime.now()
                    robot_connected = True
                    telemetry_count += 1
                    
                    if telemetry_count % 100 == 0:
                        logger.debug(f"Telemetry #{telemetry_count} received")
                        
            except json.JSONDecodeError:
                logger.warning(f"Invalid JSON telemetry: {data}")
                
        except BlockingIOError:
            await asyncio.sleep(0.01)
        except Exception as e:
            logger.error(f"Error in telemetry listener: {e}")
            await asyncio.sleep(0.1)


async def robot_connection_monitor():
    """Monitor robot connection status based on telemetry reception."""
    global robot_connected, last_robot_response_time
    
    logger.info("Robot connection monitor started")
    
    while True:
        await asyncio.sleep(1.0)
        
        try:
            if last_robot_response_time:
                time_since_response = (datetime.now() - last_robot_response_time).total_seconds()
                if time_since_response > 2.0:
                    if robot_connected:
                        logger.warning("Robot connection lost (no telemetry)")
                    robot_connected = False
                elif not robot_connected:
                    logger.info("Robot connection established")
                    robot_connected = True
            else:
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
