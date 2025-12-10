"""
Body PID Tuning Interface - Backend Server
===========================================

FastAPI server providing:
- WebSocket interface for real-time UI communication
- UDP communication with ESP32 robot
- Body velocity PID tuning (vx, vy, wz controllers)
- Telemetry broadcasting at configurable rates

Architecture:
- WebSocket handles UI commands and streams telemetry
- UDP adapter manages robot communication
- Async tasks for telemetry collection and broadcasting
"""

import asyncio
import json
import logging
import socket
from datetime import datetime
from typing import Dict, Set, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import uvicorn

# ============================================================================
# Configuration
# ============================================================================

# Server configuration
HTTP_PORT = 8082
WS_PATH = "/ws"

# Robot communication
ROBOT_IP = "192.168.4.1"  # ESP32 AP mode default IP
ROBOT_PORT = 12345
LOCAL_LISTEN_PORT = 12346

# Telemetry settings
TELEMETRY_RATE_HZ = 50  # 50Hz telemetry rate
TELEMETRY_PERIOD = 1.0 / TELEMETRY_RATE_HZ

# Logging
LOG_LEVEL = logging.INFO

# ============================================================================
# Logging Setup
# ============================================================================

logging.basicConfig(
    level=LOG_LEVEL,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ============================================================================
# Body Tuning Adapter (UDP Communication)
# ============================================================================

class BodyTuningAdapter:
    """Handles UDP communication with the ESP32 robot for body tuning"""
    
    def __init__(self, robot_ip: str, robot_port: int, local_port: int):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.local_port = local_port
        
        self.sock: Optional[socket.socket] = None
        self.running = False
        self.last_telemetry: Dict = {}
        
        # Tuning state
        self.tuning_active = False
        self.setpoints = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        
    def start(self):
        """Initialize UDP socket"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', self.local_port))
            self.sock.settimeout(0.1)  # Non-blocking with timeout
            self.running = True
            logger.info(f"UDP socket listening on port {self.local_port}")
        except Exception as e:
            logger.error(f"Failed to start UDP socket: {e}")
            raise
    
    def stop(self):
        """Close UDP socket"""
        self.running = False
        if self.sock:
            self.sock.close()
            logger.info("UDP socket closed")
    
    def send_command(self, command: Dict):
        """Send command to robot via UDP"""
        if not self.sock:
            logger.error("UDP socket not initialized")
            return False
        
        try:
            message = json.dumps(command).encode('utf-8')
            self.sock.sendto(message, (self.robot_ip, self.robot_port))
            logger.debug(f"Sent command: {command['type']}")
            return True
        except Exception as e:
            logger.error(f"Failed to send UDP command: {e}")
            return False
    
    def receive_telemetry(self) -> Optional[Dict]:
        """Receive telemetry from robot (non-blocking)"""
        if not self.sock or not self.running:
            return None
        
        try:
            data, addr = self.sock.recvfrom(4096)
            message = json.loads(data.decode('utf-8'))
            
            if message.get('type') == 'body_telemetry':
                self.last_telemetry = message
                return message
                
        except socket.timeout:
            # Normal timeout, no data available
            pass
        except Exception as e:
            logger.error(f"Error receiving telemetry: {e}")
        
        return None
    
    def start_tuning(self, vx_setpoint: float, vy_setpoint: float, wz_setpoint: float):
        """Start body tuning mode"""
        self.tuning_active = True
        self.setpoints = {'vx': vx_setpoint, 'vy': vy_setpoint, 'wz': wz_setpoint}
        
        command = {
            'type': 'tuning_command',
            'mode': 'body',
            'action': 'start',
            'vx_setpoint': vx_setpoint,
            'vy_setpoint': vy_setpoint,
            'wz_setpoint': wz_setpoint
        }
        
        return self.send_command(command)
    
    def stop_tuning(self):
        """Stop body tuning mode"""
        self.tuning_active = False
        
        command = {
            'type': 'tuning_command',
            'mode': 'body',
            'action': 'stop'
        }
        
        return self.send_command(command)
    
    def update_pid_gains(self, axis: str, kp: float, ki: float, kd: float):
        """Update PID gains for specified axis"""
        command = {
            'type': 'tuning_command',
            'mode': 'body',
            'action': 'update_pid',
            'axis': axis,
            'kp': kp,
            'ki': ki,
            'kd': kd
        }
        
        return self.send_command(command)
    
    def update_setpoint(self, axis: str, setpoint: float):
        """Update setpoint for specified axis"""
        self.setpoints[axis] = setpoint
        
        command = {
            'type': 'tuning_command',
            'mode': 'body',
            'action': 'set_setpoint',
            'axis': axis,
            'setpoint': setpoint
        }
        
        return self.send_command(command)
    
    def emergency_stop(self):
        """Send emergency stop command"""
        self.tuning_active = False
        
        command = {
            'type': 'tuning_command',
            'mode': 'body',
            'action': 'emergency_stop'
        }
        
        return self.send_command(command)

# ============================================================================
# WebSocket Connection Manager
# ============================================================================

class ConnectionManager:
    """Manages WebSocket connections and broadcasts"""
    
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.add(websocket)
        logger.info(f"WebSocket connected. Active connections: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        self.active_connections.discard(websocket)
        logger.info(f"WebSocket disconnected. Active connections: {len(self.active_connections)}")
    
    async def send_message(self, websocket: WebSocket, message: Dict):
        """Send message to specific websocket"""
        try:
            await websocket.send_json(message)
        except Exception as e:
            logger.error(f"Error sending message: {e}")
            self.disconnect(websocket)
    
    async def broadcast(self, message: Dict):
        """Broadcast message to all connected clients"""
        disconnected = set()
        
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {e}")
                disconnected.add(connection)
        
        # Clean up disconnected clients
        for connection in disconnected:
            self.disconnect(connection)

# ============================================================================
# Global State
# ============================================================================

robot_adapter: Optional[BodyTuningAdapter] = None
connection_manager = ConnectionManager()
telemetry_task: Optional[asyncio.Task] = None

# ============================================================================
# Background Tasks
# ============================================================================

async def telemetry_listener():
    """Background task to receive and broadcast telemetry"""
    logger.info("Telemetry listener started")
    
    while True:
        try:
            # Receive telemetry from robot
            telemetry = robot_adapter.receive_telemetry()
            
            if telemetry and connection_manager.active_connections:
                # Broadcast to all connected WebSocket clients
                await connection_manager.broadcast({
                    'type': 'telemetry',
                    'vx_setpoint': telemetry.get('vx_setpoint', 0),
                    'vx_measured': telemetry.get('vx_measured', 0),
                    'vx_control': telemetry.get('vx_control', 0),
                    'vy_setpoint': telemetry.get('vy_setpoint', 0),
                    'vy_measured': telemetry.get('vy_measured', 0),
                    'vy_control': telemetry.get('vy_control', 0),
                    'wz_setpoint': telemetry.get('wz_setpoint', 0),
                    'wz_measured': telemetry.get('wz_measured', 0),
                    'wz_control': telemetry.get('wz_control', 0)
                })
            
            # Rate limiting
            await asyncio.sleep(TELEMETRY_PERIOD)
            
        except Exception as e:
            logger.error(f"Error in telemetry listener: {e}")
            await asyncio.sleep(0.1)

# ============================================================================
# FastAPI Application
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    global robot_adapter, telemetry_task
    
    # Startup
    logger.info("Starting Body PID Tuning Server")
    robot_adapter = BodyTuningAdapter(ROBOT_IP, ROBOT_PORT, LOCAL_LISTEN_PORT)
    
    try:
        robot_adapter.start()
    except Exception as e:
        logger.error(f"Failed to initialize robot adapter: {e}")
    
    # Start telemetry listener
    telemetry_task = asyncio.create_task(telemetry_listener())
    
    logger.info(f"Server ready on http://0.0.0.0:{HTTP_PORT}")
    logger.info(f"Robot UDP endpoint: {ROBOT_IP}:{ROBOT_PORT}")
    logger.info(f"Telemetry rate: {TELEMETRY_RATE_HZ} Hz")
    
    yield
    
    # Shutdown
    logger.info("Shutting down server")
    
    if telemetry_task:
        telemetry_task.cancel()
        try:
            await telemetry_task
        except asyncio.CancelledError:
            pass
    
    if robot_adapter:
        robot_adapter.stop()
    
    logger.info("Server stopped")

app = FastAPI(title="Body PID Tuning Server", lifespan=lifespan)

# Mount static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# ============================================================================
# HTTP Endpoints
# ============================================================================

@app.get("/", response_class=HTMLResponse)
async def root():
    """Serve the main HTML page"""
    with open("static/index.html", "r") as f:
        return f.read()

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "robot_connected": robot_adapter.running if robot_adapter else False,
        "active_connections": len(connection_manager.active_connections),
        "tuning_active": robot_adapter.tuning_active if robot_adapter else False
    }

# ============================================================================
# WebSocket Endpoint
# ============================================================================

@app.websocket(WS_PATH)
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time communication"""
    await connection_manager.connect(websocket)
    
    try:
        # Send initial status
        await connection_manager.send_message(websocket, {
            'type': 'status',
            'robot_status': 'Connected' if robot_adapter.running else 'Disconnected',
            'tuning_active': robot_adapter.tuning_active
        })
        
        # Message handling loop
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            
            await handle_websocket_message(websocket, message)
            
    except WebSocketDisconnect:
        connection_manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        connection_manager.disconnect(websocket)

async def handle_websocket_message(websocket: WebSocket, message: Dict):
    """Handle incoming WebSocket messages"""
    msg_type = message.get('type')
    
    if msg_type == 'start_tuning':
        vx_setpoint = message.get('vx_setpoint', 0.0)
        vy_setpoint = message.get('vy_setpoint', 0.0)
        wz_setpoint = message.get('wz_setpoint', 0.0)
        
        success = robot_adapter.start_tuning(vx_setpoint, vy_setpoint, wz_setpoint)
        
        await connection_manager.send_message(websocket, {
            'type': 'ack' if success else 'error',
            'message': f'Body tuning started' if success else 'Failed to start tuning'
        })
        
    elif msg_type == 'stop_tuning':
        success = robot_adapter.stop_tuning()
        
        await connection_manager.send_message(websocket, {
            'type': 'ack' if success else 'error',
            'message': 'Body tuning stopped' if success else 'Failed to stop tuning'
        })
        
    elif msg_type == 'apply_pid':
        axis = message.get('axis')
        kp = message.get('kp', 0.0)
        ki = message.get('ki', 0.0)
        kd = message.get('kd', 0.0)
        
        if axis not in ['vx', 'vy', 'wz']:
            await connection_manager.send_message(websocket, {
                'type': 'error',
                'message': f'Invalid axis: {axis}'
            })
            return
        
        success = robot_adapter.update_pid_gains(axis, kp, ki, kd)
        
        await connection_manager.send_message(websocket, {
            'type': 'ack' if success else 'error',
            'message': f'PID gains updated for {axis.upper()}' if success else f'Failed to update PID for {axis.upper()}'
        })
        
    elif msg_type == 'set_setpoint':
        axis = message.get('axis')
        setpoint = message.get('setpoint', 0.0)
        
        if axis not in ['vx', 'vy', 'wz']:
            await connection_manager.send_message(websocket, {
                'type': 'error',
                'message': f'Invalid axis: {axis}'
            })
            return
        
        success = robot_adapter.update_setpoint(axis, setpoint)
        
        await connection_manager.send_message(websocket, {
            'type': 'ack' if success else 'error',
            'message': f'Setpoint updated for {axis.upper()}' if success else f'Failed to update setpoint for {axis.upper()}'
        })
        
    elif msg_type == 'emergency_stop':
        success = robot_adapter.emergency_stop()
        
        await connection_manager.broadcast({
            'type': 'status',
            'robot_status': 'Emergency Stop'
        })
        
        await connection_manager.send_message(websocket, {
            'type': 'ack' if success else 'error',
            'message': 'Emergency stop activated' if success else 'Failed to activate emergency stop'
        })
        
    else:
        await connection_manager.send_message(websocket, {
            'type': 'error',
            'message': f'Unknown message type: {msg_type}'
        })

# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    uvicorn.run(
        "app:app",
        host="0.0.0.0",
        port=HTTP_PORT,
        log_level="info",
        reload=False
    )
