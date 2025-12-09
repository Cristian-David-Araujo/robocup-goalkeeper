/**
 * Robot Teleoperation Web UI - Client JavaScript
 * 
 * Handles keyboard input, WebSocket communication, and UI updates.
 */

// =============================================================================
// CONFIGURATION
// =============================================================================

let config = {
    maxLinearVelocity: 1.0,
    maxAngularVelocity: 2.0,
    updateRateHz: 20,
    maxAcceleration: 2.0,
    rampEnabled: true
};

// =============================================================================
// STATE MANAGEMENT
// =============================================================================

// WebSocket connection
let ws = null;
let isConnected = false;

// Robot connection status
let robotConnected = false;

// Key state tracking
const keyState = {
    w: false,
    s: false,
    a: false,
    d: false,
    q: false,
    e: false
};

// Target velocities (what user wants from keyboard)
let targetVelocity = {
    vx: 0.0,
    vy: 0.0,
    wz: 0.0
};

// Actual velocities (measured from robot sensors)
let actualVelocity = {
    vx: 0.0,
    vy: 0.0,
    wz: 0.0
};

// Ramped velocity (server-side acceleration limiting)
let rampedVelocity = {
    vx: 0.0,
    vy: 0.0,
    wz: 0.0
};

// Update rate tracking
let updateCount = 0;
let lastUpdateTime = Date.now();
let updateRateDisplay = 0;

// =============================================================================
// GRAPH DATA
// =============================================================================

const GRAPH_HISTORY_SIZE = 100; // Number of data points to keep
const GRAPH_UPDATE_INTERVAL = 100; // Update every 100ms (10 Hz)

// Graph data buffers
const graphData = {
    vx: { target: [], actual: [] },
    vy: { target: [], actual: [] },
    wz: { target: [], actual: [] }
};

// Canvas contexts
let chartContexts = {
    vx: null,
    vy: null,
    wz: null
};

let lastGraphUpdate = Date.now();

// =============================================================================
// WEBSOCKET CONNECTION
// =============================================================================

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    console.log('Connecting to WebSocket:', wsUrl);
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        console.log('WebSocket connected');
        isConnected = true;
        updateConnectionStatus(true);
    };
    
    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        } catch (error) {
            console.error('Error parsing WebSocket message:', error);
        }
    };
    
    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateConnectionStatus(false);
    };
    
    ws.onclose = () => {
        console.log('WebSocket disconnected');
        isConnected = false;
        updateConnectionStatus(false);
        
        // Attempt reconnection after 2 seconds
        setTimeout(connectWebSocket, 2000);
    };
}

function handleWebSocketMessage(data) {
    if (data.type === 'config') {
        config.maxLinearVelocity = data.max_linear_velocity;
        config.maxAngularVelocity = data.max_angular_velocity;
        config.updateRateHz = data.update_rate_hz;
        config.maxAcceleration = data.max_acceleration || 2.0;
        config.rampEnabled = data.ramp_enabled !== false;
        robotConnected = data.robot_connected || false;
        updateRobotConnectionStatus(robotConnected);
        console.log('Received config:', config);
    } else if (data.type === 'ack') {
        // Ramped velocity (server-side acceleration limiting)
        if (data.velocity) {
            rampedVelocity = {
                vx: data.velocity.vx,
                vy: data.velocity.vy,
                wz: data.velocity.wz
            };
        }
        // Target velocity (user keyboard input)
        if (data.target_velocity) {
            targetVelocity = {
                vx: data.target_velocity.vx,
                vy: data.target_velocity.vy,
                wz: data.target_velocity.wz
            };
        }
        // Actual velocity from robot sensors
        if (data.actual_velocity) {
            actualVelocity = {
                vx: data.actual_velocity.vx,
                vy: data.actual_velocity.vy,
                wz: data.actual_velocity.wz
            };
            // Debug: log occasionally to check if receiving data
            if (Math.random() < 0.01) { // 1% chance = ~every 5 seconds at 20Hz
                console.log('Actual velocity from robot:', actualVelocity);
            }
        }
        if (data.robot_connected !== undefined) {
            robotConnected = data.robot_connected;
            updateRobotConnectionStatus(robotConnected);
            // Debug: log connection status changes
            if (Math.random() < 0.02) {
                console.log('Robot connected:', robotConnected, 'actual_velocity:', data.actual_velocity);
            }
        }
        updateVelocityDisplay();
    }
}

function sendVelocityCommand(vx, vy, wz) {
    if (!isConnected || !ws || ws.readyState !== WebSocket.OPEN) {
        return;
    }
    
    const command = {
        vx: vx,
        vy: vy,
        wz: wz
    };
    
    ws.send(JSON.stringify(command));
    
    // Update rate tracking
    updateCount++;
    const now = Date.now();
    if (now - lastUpdateTime >= 1000) {
        updateRateDisplay = updateCount;
        updateCount = 0;
        lastUpdateTime = now;
        document.getElementById('updateRate').textContent = `${updateRateDisplay} Hz`;
    }
}

// =============================================================================
// KEYBOARD HANDLING
// =============================================================================

function computeVelocityFromKeys() {
    let vx = 0.0;
    let vy = 0.0;
    let wz = 0.0;
    
    // Linear X (strafe left/right)
    if (keyState.a) vx -= config.maxLinearVelocity;
    if (keyState.d) vx += config.maxLinearVelocity;
    
    // Linear Y (forward/backward)
    if (keyState.w) vy += config.maxLinearVelocity;
    if (keyState.s) vy -= config.maxLinearVelocity;
    
    // Angular Z (rotate)
    // Q = counter-clockwise (positive), E = clockwise (negative)
    if (keyState.q) wz += config.maxAngularVelocity;
    if (keyState.e) wz -= config.maxAngularVelocity;
    
    return { vx, vy, wz };
}

function onKeyDown(event) {
    const key = event.key.toLowerCase();
    
    if (key in keyState && !keyState[key]) {
        keyState[key] = true;
        updateKeyVisual(key, true);
        event.preventDefault();
    }
}

function onKeyUp(event) {
    const key = event.key.toLowerCase();
    
    if (key in keyState) {
        keyState[key] = false;
        updateKeyVisual(key, false);
        event.preventDefault();
    }
}

function updateKeyVisual(key, isActive) {
    const keyElement = document.querySelector(`.key-item[data-key="${key}"]`);
    if (keyElement) {
        if (isActive) {
            keyElement.classList.add('active');
        } else {
            keyElement.classList.remove('active');
        }
    }
}

// =============================================================================
// UI UPDATES
// =============================================================================

function updateConnectionStatus(connected) {
    const statusDot = document.getElementById('statusDot');
    const statusText = document.getElementById('statusText');
    
    if (connected) {
        statusDot.classList.add('connected');
        statusText.textContent = 'Connected';
        statusText.style.color = '#28a745';
    } else {
        statusDot.classList.remove('connected');
        statusText.textContent = 'Disconnected';
        statusText.style.color = '#dc3545';
    }
}

function updateRobotConnectionStatus(connected) {
    const robotStatusDot = document.getElementById('robotStatusDot');
    const robotStatusText = document.getElementById('robotStatusText');
    
    if (connected) {
        robotStatusDot.classList.add('connected');
        robotStatusText.textContent = 'Robot: Connected';
        robotStatusText.style.color = '#28a745';
    } else {
        robotStatusDot.classList.remove('connected');
        robotStatusText.textContent = 'Robot: Disconnected';
        robotStatusText.style.color = '#dc3545';
    }
}

function updateVelocityDisplay() {
    // Update actual velocity from robot sensors
    document.getElementById('vxDisplay').textContent = actualVelocity.vx.toFixed(2);
    document.getElementById('vyDisplay').textContent = actualVelocity.vy.toFixed(2);
    document.getElementById('wzDisplay').textContent = actualVelocity.wz.toFixed(2);
    
    // Update target velocity (what user wants)
    const vxTargetEl = document.getElementById('vxTarget');
    const vyTargetEl = document.getElementById('vyTarget');
    const wzTargetEl = document.getElementById('wzTarget');
    
    if (vxTargetEl) {
        const showTarget = Math.abs(targetVelocity.vx - actualVelocity.vx) > 0.01;
        vxTargetEl.textContent = showTarget ? `→ ${targetVelocity.vx.toFixed(2)}` : '';
    }
    
    if (vyTargetEl) {
        const showTarget = Math.abs(targetVelocity.vy - actualVelocity.vy) > 0.01;
        vyTargetEl.textContent = showTarget ? `→ ${targetVelocity.vy.toFixed(2)}` : '';
    }
    
    if (wzTargetEl) {
        const showTarget = Math.abs(targetVelocity.wz - actualVelocity.wz) > 0.01;
        wzTargetEl.textContent = showTarget ? `→ ${targetVelocity.wz.toFixed(2)}` : '';
    }
    
    // Update graphs periodically
    updateGraphs();
}

// =============================================================================
// GRAPH FUNCTIONS
// =============================================================================

function initGraphs() {
    chartContexts.vx = document.getElementById('vxChart').getContext('2d');
    chartContexts.vy = document.getElementById('vyChart').getContext('2d');
    chartContexts.wz = document.getElementById('wzChart').getContext('2d');
}

function updateGraphData() {
    const now = Date.now();
    if (now - lastGraphUpdate < GRAPH_UPDATE_INTERVAL) {
        return;
    }
    lastGraphUpdate = now;
    
    // Add new data points: commanded (ramped) and actual (from sensors)
    ['vx', 'vy', 'wz'].forEach(key => {
        graphData[key].target.push(rampedVelocity[key]);  // Command sent to robot
        graphData[key].actual.push(actualVelocity[key]);  // Measured by robot sensors
        
        // Keep only recent history
        if (graphData[key].target.length > GRAPH_HISTORY_SIZE) {
            graphData[key].target.shift();
            graphData[key].actual.shift();
        }
    });
}

function updateGraphs() {
    updateGraphData();
    
    drawGraph(chartContexts.vx, graphData.vx, 'Linear X', config.maxLinearVelocity);
    drawGraph(chartContexts.vy, graphData.vy, 'Linear Y', config.maxLinearVelocity);
    drawGraph(chartContexts.wz, graphData.wz, 'Angular Z', config.maxAngularVelocity);
}

function drawGraph(ctx, data, label, maxValue) {
    if (!ctx) return;
    
    const canvas = ctx.canvas;
    const width = canvas.width;
    const height = canvas.height;
    const padding = 45;
    const graphWidth = width - 2 * padding;
    const graphHeight = height - 2 * padding;
    
    // Clear canvas
    ctx.clearRect(0, 0, width, height);
    
    // Dark background gradient
    const gradient = ctx.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, '#242735');
    gradient.addColorStop(1, '#1a1d29');
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 0.5;
    
    // Horizontal grid lines
    for (let i = 0; i <= 4; i++) {
        const y = padding + (graphHeight / 4) * i;
        ctx.beginPath();
        ctx.moveTo(padding, y);
        ctx.lineTo(width - padding, y);
        ctx.stroke();
    }
    
    // Vertical grid lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    for (let i = 0; i <= 10; i++) {
        const x = padding + (graphWidth / 10) * i;
        ctx.beginPath();
        ctx.moveTo(x, padding);
        ctx.lineTo(x, height - padding);
        ctx.stroke();
    }
    
    // Draw axes
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(padding, padding);
    ctx.lineTo(padding, height - padding);
    ctx.lineTo(width - padding, height - padding);
    ctx.stroke();
    
    // Draw zero line with gradient effect
    const zeroY = padding + graphHeight / 2;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([8, 4]);
    ctx.beginPath();
    ctx.moveTo(padding, zeroY);
    ctx.lineTo(width - padding, zeroY);
    ctx.stroke();
    ctx.setLineDash([]);
    
    if (data.target.length === 0) return;
    
    // Scale factor
    const scale = graphHeight / (2 * maxValue);
    
    // Draw commanded velocity line (dashed blue)
    ctx.shadowColor = 'rgba(102, 126, 234, 0.6)';
    ctx.shadowBlur = 8;
    ctx.strokeStyle = '#7c8ef8';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    for (let i = 0; i < data.target.length; i++) {
        const x = padding + (graphWidth / GRAPH_HISTORY_SIZE) * i;
        const y = zeroY - data.target[i] * scale;
        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    }
    ctx.stroke();
    ctx.setLineDash([]);
    
    // Draw actual velocity from sensors (solid green)
    ctx.shadowColor = 'rgba(40, 220, 100, 0.6)';
    ctx.shadowBlur = 8;
    ctx.strokeStyle = '#3ddc84';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    for (let i = 0; i < data.actual.length; i++) {
        const x = padding + (graphWidth / GRAPH_HISTORY_SIZE) * i;
        const y = zeroY - data.actual[i] * scale;
        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
    
    // Draw labels
    ctx.fillStyle = '#e0e0e0';
    ctx.font = 'bold 10px Segoe UI';
    ctx.textAlign = 'right';
    ctx.fillText(maxValue.toFixed(1), padding - 8, padding + 5);
    ctx.fillText('0', padding - 8, zeroY + 5);
    ctx.fillText((-maxValue).toFixed(1), padding - 8, height - padding + 5);
    
    // Title label
    ctx.font = 'bold 13px Segoe UI';
    ctx.textAlign = 'center';
    ctx.fillStyle = '#e0e0e0';
    ctx.fillText(label, width / 2, padding - 20);
    
    // Legend with dark background
    const legendX = width - padding - 120;
    const legendY = padding + 5;
    ctx.fillStyle = 'rgba(26, 29, 41, 0.95)';
    ctx.fillRect(legendX - 5, legendY - 2, 115, 42);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1;
    ctx.strokeRect(legendX - 5, legendY - 2, 115, 42);
    
    ctx.font = 'bold 10px Segoe UI';
    ctx.textAlign = 'left';
    
    // Command legend (dashed blue)
    ctx.shadowColor = 'rgba(124, 142, 248, 0.6)';
    ctx.shadowBlur = 4;
    ctx.fillStyle = '#7c8ef8';
    ctx.beginPath();
    ctx.arc(legendX + 5, legendY + 8, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#e0e0e0';
    ctx.fillText('Command', legendX + 15, legendY + 12);
    
    // Actual legend (solid green)
    ctx.shadowColor = 'rgba(61, 220, 132, 0.6)';
    ctx.shadowBlur = 4;
    ctx.fillStyle = '#3ddc84';
    ctx.beginPath();
    ctx.arc(legendX + 5, legendY + 28, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#e0e0e0';
    ctx.fillText('Actual', legendX + 15, legendY + 32);
}

function emergencyStop() {
    // Reset all key states
    for (const key in keyState) {
        keyState[key] = false;
        updateKeyVisual(key, false);
    }
    
    // Send zero velocities
    sendVelocityCommand(0, 0, 0);
    
    // Also call the server emergency stop endpoint
    fetch('/api/stop', { method: 'POST' })
        .then(response => response.json())
        .then(data => {
            console.log('Emergency stop response:', data);
        })
        .catch(error => {
            console.error('Emergency stop error:', error);
        });
}

// =============================================================================
// PERIODIC UPDATE LOOP
// =============================================================================

function updateLoop() {
    const velocity = computeVelocityFromKeys();
    
    // Always send command to ensure immediate response
    // This is especially important for stopping rotation when releasing Q/E
    sendVelocityCommand(velocity.vx, velocity.vy, velocity.wz);
    
    // Update local state
    targetVelocity = velocity;
}

// =============================================================================
// EVENT LISTENERS
// =============================================================================

document.addEventListener('DOMContentLoaded', () => {
    // Initialize graphs
    initGraphs();
    
    // Connect WebSocket
    connectWebSocket();
    
    // Keyboard events
    document.addEventListener('keydown', onKeyDown);
    document.addEventListener('keyup', onKeyUp);
    
    // Prevent default behavior for controlled keys
    document.addEventListener('keydown', (event) => {
        if (event.key.toLowerCase() in keyState) {
            event.preventDefault();
        }
    });
    
    // Emergency stop button
    document.getElementById('stopBtn').addEventListener('click', emergencyStop);
    
    // Info button
    document.getElementById('infoBtn').addEventListener('click', () => {
        const infoPanel = document.getElementById('infoPanel');
        if (infoPanel.style.display === 'none') {
            infoPanel.style.display = 'block';
        } else {
            infoPanel.style.display = 'none';
        }
    });
    
    // Window blur - stop robot when window loses focus
    window.addEventListener('blur', () => {
        console.log('Window lost focus - stopping robot');
        emergencyStop();
    });
    
    // Start periodic update loop
    const updateInterval = 1000 / config.updateRateHz;
    setInterval(updateLoop, updateInterval);
    
    console.log('Teleoperation UI initialized');
    console.log(`Update rate: ${config.updateRateHz} Hz (${updateInterval.toFixed(1)} ms)`);
});

// =============================================================================
// VISIBILITY CHANGE HANDLER
// =============================================================================

document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        console.log('Page hidden - stopping robot');
        emergencyStop();
    }
});
