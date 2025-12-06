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
    updateRateHz: 20
};

// =============================================================================
// STATE MANAGEMENT
// =============================================================================

// WebSocket connection
let ws = null;
let isConnected = false;

// Key state tracking
const keyState = {
    w: false,
    s: false,
    a: false,
    d: false,
    q: false,
    e: false
};

// Current velocities
let currentVelocity = {
    vx: 0.0,
    vy: 0.0,
    wz: 0.0
};

// Update rate tracking
let updateCount = 0;
let lastUpdateTime = Date.now();
let updateRateDisplay = 0;

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
        console.log('Received config:', config);
    } else if (data.type === 'ack') {
        if (data.velocity) {
            currentVelocity = {
                vx: data.velocity.vx,
                vy: data.velocity.vy,
                wz: data.velocity.wz
            };
            updateVelocityDisplay();
        }
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
    
    // Linear X (forward/backward)
    if (keyState.w) vx += config.maxLinearVelocity;
    if (keyState.s) vx -= config.maxLinearVelocity;
    
    // Linear Y (strafe left/right)
    if (keyState.a) vy -= config.maxLinearVelocity;
    if (keyState.d) vy += config.maxLinearVelocity;
    
    // Angular Z (rotate)
    if (keyState.q) wz -= config.maxAngularVelocity;
    if (keyState.e) wz += config.maxAngularVelocity;
    
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

function updateVelocityDisplay() {
    document.getElementById('vxDisplay').textContent = currentVelocity.vx.toFixed(2);
    document.getElementById('vyDisplay').textContent = currentVelocity.vy.toFixed(2);
    document.getElementById('wzDisplay').textContent = currentVelocity.wz.toFixed(2);
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
    
    // Only send if there's a change or if velocities are non-zero
    if (velocity.vx !== currentVelocity.vx || 
        velocity.vy !== currentVelocity.vy || 
        velocity.wz !== currentVelocity.wz ||
        velocity.vx !== 0 || velocity.vy !== 0 || velocity.wz !== 0) {
        
        sendVelocityCommand(velocity.vx, velocity.vy, velocity.wz);
        currentVelocity = velocity;
        updateVelocityDisplay();
    }
}

// =============================================================================
// EVENT LISTENERS
// =============================================================================

document.addEventListener('DOMContentLoaded', () => {
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
