/**
 * Wheel PID Tuning Interface - Client JavaScript
 * 
 * Handles WebSocket communication, real-time graphing, and UI updates
 * for tuning individual wheel PID controllers.
 */

// =============================================================================
// CONFIGURATION
// =============================================================================

let config = {
    samplingRateHz: 50,
    bufferSizeSeconds: 10,
    maxWheelVelocity: 50.0, // rad/s
    maxControlOutput: 100.0  // PWM %
};

// =============================================================================
// STATE MANAGEMENT
// =============================================================================

// WebSocket connection
let ws = null;
let isConnected = false;

// Robot and tuning status
let robotConnected = false;
let tuningActive = false;

// PID parameters
let pidParams = {
    kp: 1.0,
    ki: 0.1,
    kd: 0.05,
    setpoint: 0.0
};

// Wheel data
let wheelData = {
    wheel1: { velocity: 0.0, setpoint: 0.0, control: 0.0 },
    wheel2: { velocity: 0.0, setpoint: 0.0, control: 0.0 },
    wheel3: { velocity: 0.0, setpoint: 0.0, control: 0.0 }
};

// Update rate tracking
let updateCount = 0;
let lastUpdateTime = Date.now();
let updateRateDisplay = 0;

// Session log
let sessionLog = [];
let sessionStartTime = null;

// =============================================================================
// GRAPH DATA
// =============================================================================

const graphData = {
    wheel1: {
        setpoint: [],
        measured: [],
        control: []
    },
    wheel2: {
        setpoint: [],
        measured: [],
        control: []
    },
    wheel3: {
        setpoint: [],
        measured: [],
        control: []
    }
};

let maxDataPoints = 500; // Will be recalculated based on buffer size

// Canvas contexts
let chartContexts = {
    wheel1Vel: null,
    wheel2Vel: null,
    wheel3Vel: null,
    wheel1Ctrl: null,
    wheel2Ctrl: null,
    wheel3Ctrl: null
};

let lastGraphUpdate = Date.now();
const GRAPH_UPDATE_INTERVAL = 50; // Update every 50ms (20 Hz)

// =============================================================================
// WEBSOCKET CONNECTION
// =============================================================================

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    console.log('Connecting to WebSocket:', wsUrl);
    addLogEntry('Connecting to server...');
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        console.log('WebSocket connected');
        isConnected = true;
        updateConnectionStatus(true);
        addLogEntry('✓ Connected to server');
    };
    
    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        } catch (error) {
            console.error('Error parsing WebSocket message:', error);
            addLogEntry('⚠ Error parsing message: ' + error.message);
        }
    };
    
    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateConnectionStatus(false);
        addLogEntry('✗ WebSocket error');
    };
    
    ws.onclose = () => {
        console.log('WebSocket disconnected');
        isConnected = false;
        updateConnectionStatus(false);
        addLogEntry('✗ Disconnected from server');
        
        // Attempt reconnection after 2 seconds
        setTimeout(connectWebSocket, 2000);
    };
}

function handleWebSocketMessage(data) {
    if (data.type === 'config') {
        // Initial configuration from server
        config.samplingRateHz = data.sampling_rate_hz || 50;
        config.maxWheelVelocity = data.max_wheel_velocity || 50.0;
        robotConnected = data.robot_connected || false;
        updateRobotConnectionStatus(robotConnected);
        addLogEntry('📋 Received configuration from server');
        console.log('Config:', config);
        
    } else if (data.type === 'telemetry') {
        // Real-time telemetry data
        if (data.wheels) {
            wheelData.wheel1 = {
                velocity: data.wheels.wheel1?.velocity || 0.0,
                setpoint: data.wheels.wheel1?.setpoint || 0.0,
                control: data.wheels.wheel1?.control || 0.0
            };
            wheelData.wheel2 = {
                velocity: data.wheels.wheel2?.velocity || 0.0,
                setpoint: data.wheels.wheel2?.setpoint || 0.0,
                control: data.wheels.wheel2?.control || 0.0
            };
            wheelData.wheel3 = {
                velocity: data.wheels.wheel3?.velocity || 0.0,
                setpoint: data.wheels.wheel3?.setpoint || 0.0,
                control: data.wheels.wheel3?.control || 0.0
            };
            
            updateWheelDisplay();
            updateGraphData();
        }
        
        if (data.robot_connected !== undefined) {
            robotConnected = data.robot_connected;
            updateRobotConnectionStatus(robotConnected);
        }
        
        // Update rate tracking
        updateCount++;
        const now = Date.now();
        if (now - lastUpdateTime >= 1000) {
            updateRateDisplay = updateCount;
            updateCount = 0;
            lastUpdateTime = now;
            document.getElementById('updateRate').textContent = `${updateRateDisplay} Hz`;
        }
        
    } else if (data.type === 'ack') {
        // Command acknowledgment
        if (data.command === 'start_tuning') {
            tuningActive = data.success;
            updateTuningStatus(tuningActive);
            if (data.success) {
                addLogEntry('✓ Tuning mode activated');
            } else {
                addLogEntry('✗ Failed to activate tuning mode: ' + (data.error || 'Unknown error'));
            }
        } else if (data.command === 'stop_tuning') {
            tuningActive = false;
            updateTuningStatus(false);
            addLogEntry('✓ Tuning mode deactivated');
            
        } else if (data.command === 'apply_pid') {
            if (data.success) {
                addLogEntry(`✓ PID constants applied: Kp=${pidParams.kp}, Ki=${pidParams.ki}, Kd=${pidParams.kd}`);
            } else {
                addLogEntry('✗ Failed to apply PID constants: ' + (data.error || 'Unknown error'));
            }
            
        } else if (data.command === 'set_setpoint') {
            if (data.success) {
                addLogEntry(`✓ Set point updated: ${pidParams.setpoint.toFixed(2)} rad/s`);
            } else {
                addLogEntry('✗ Failed to update set point: ' + (data.error || 'Unknown error'));
            }
        }
        
    } else if (data.type === 'error') {
        addLogEntry('⚠ Server error: ' + data.message);
        console.error('Server error:', data.message);
    }
}

function sendCommand(command, params = {}) {
    if (!isConnected || !ws || ws.readyState !== WebSocket.OPEN) {
        addLogEntry('✗ Cannot send command: Not connected');
        return;
    }
    
    const message = {
        command: command,
        ...params
    };
    
    ws.send(JSON.stringify(message));
    console.log('Sent command:', message);
}

// =============================================================================
// UI EVENT HANDLERS
// =============================================================================

function onStartTuning() {
    if (!robotConnected) {
        alert('Robot is not connected. Please check the connection.');
        return;
    }
    
    sendCommand('start_tuning');
    document.getElementById('startTuningBtn').disabled = true;
    document.getElementById('pauseTuningBtn').disabled = false;
    
    // Initialize session
    sessionStartTime = Date.now();
    addLogEntry('🚀 Starting tuning session...');
}

function onPauseTuning() {
    sendCommand('pause_tuning');
    document.getElementById('startTuningBtn').disabled = false;
    document.getElementById('pauseTuningBtn').disabled = true;
    addLogEntry('⏸ Tuning paused');
}

function onStopTuning() {
    sendCommand('stop_tuning');
    document.getElementById('startTuningBtn').disabled = false;
    document.getElementById('pauseTuningBtn').disabled = true;
    tuningActive = false;
    updateTuningStatus(false);
    addLogEntry('⛔ Tuning stopped');
}

function onApplyPID() {
    // Read PID values from inputs
    pidParams.kp = parseFloat(document.getElementById('kpInput').value) || 0.0;
    pidParams.ki = parseFloat(document.getElementById('kiInput').value) || 0.0;
    pidParams.kd = parseFloat(document.getElementById('kdInput').value) || 0.0;
    
    // Validate
    if (pidParams.kp < 0 || pidParams.ki < 0 || pidParams.kd < 0) {
        alert('PID constants must be non-negative');
        return;
    }
    
    sendCommand('apply_pid', {
        kp: pidParams.kp,
        ki: pidParams.ki,
        kd: pidParams.kd
    });
    
    addLogEntry(`→ Applying PID: Kp=${pidParams.kp}, Ki=${pidParams.ki}, Kd=${pidParams.kd}`);
}

function onApplySetpoint() {
    pidParams.setpoint = parseFloat(document.getElementById('setpointInput').value) || 0.0;
    
    // Validate
    if (Math.abs(pidParams.setpoint) > config.maxWheelVelocity) {
        alert(`Set point must be between -${config.maxWheelVelocity} and ${config.maxWheelVelocity} rad/s`);
        return;
    }
    
    sendCommand('set_setpoint', {
        setpoint: pidParams.setpoint
    });
    
    addLogEntry(`→ Setting target: ${pidParams.setpoint.toFixed(2)} rad/s`);
}

function onEmergencyStop() {
    sendCommand('emergency_stop');
    onStopTuning();
    addLogEntry('⚠⚠⚠ EMERGENCY STOP ⚠⚠⚠');
}

function onResetPlot() {
    // Clear all graph data
    for (const wheel in graphData) {
        graphData[wheel].setpoint = [];
        graphData[wheel].measured = [];
        graphData[wheel].control = [];
    }
    
    // Redraw empty graphs
    updateGraphs();
    addLogEntry('🔄 Plot data reset');
}

function onDownloadLog() {
    if (sessionLog.length === 0) {
        alert('No log data to download');
        return;
    }
    
    // Generate CSV content
    let csvContent = 'Timestamp,Wheel,Setpoint,Measured,Control,Event\n';
    
    sessionLog.forEach(entry => {
        if (entry.data) {
            // Telemetry data
            for (const wheel in entry.data) {
                const d = entry.data[wheel];
                csvContent += `${entry.timestamp},${wheel},${d.setpoint},${d.velocity},${d.control},\n`;
            }
        } else {
            // Event log
            csvContent += `${entry.timestamp},,,,,${entry.message}\n`;
        }
    });
    
    // Create download
    const blob = new Blob([csvContent], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `wheel_pid_tuning_${new Date().toISOString().replace(/:/g, '-')}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    addLogEntry('💾 Log downloaded');
}

function onClearLog() {
    const logContent = document.getElementById('logContent');
    logContent.innerHTML = '<div class="log-entry"><span class="log-timestamp">[00:00:00]</span><span>Log cleared</span></div>';
    sessionLog = [];
    sessionStartTime = Date.now();
    addLogEntry('🗑 Log cleared');
}

function onSamplingRateChange() {
    const samplingRate = parseInt(document.getElementById('samplingRateInput').value) || 50;
    const bufferSize = parseInt(document.getElementById('bufferSizeInput').value) || 10;
    
    config.samplingRateHz = samplingRate;
    config.bufferSizeSeconds = bufferSize;
    maxDataPoints = samplingRate * bufferSize;
    
    addLogEntry(`⚙ Sampling: ${samplingRate} Hz, Buffer: ${bufferSize}s`);
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

function updateTuningStatus(active) {
    const tuningStatusDot = document.getElementById('tuningStatusDot');
    const tuningStatusText = document.getElementById('tuningStatusText');
    
    if (active) {
        tuningStatusDot.classList.add('tuning');
        tuningStatusText.textContent = 'Tuning: Active';
        tuningStatusText.style.color = '#ffc107';
    } else {
        tuningStatusDot.classList.remove('tuning');
        tuningStatusText.textContent = 'Tuning: Inactive';
        tuningStatusText.style.color = '#808080';
    }
}

function updateWheelDisplay() {
    document.getElementById('wheel1Vel').textContent = wheelData.wheel1.velocity.toFixed(2);
    document.getElementById('wheel2Vel').textContent = wheelData.wheel2.velocity.toFixed(2);
    document.getElementById('wheel3Vel').textContent = wheelData.wheel3.velocity.toFixed(2);
}

function addLogEntry(message) {
    const logContent = document.getElementById('logContent');
    const timestamp = formatTimestamp();
    
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    entry.innerHTML = `<span class="log-timestamp">[${timestamp}]</span><span>${message}</span>`;
    
    logContent.appendChild(entry);
    logContent.scrollTop = logContent.scrollHeight;
    
    // Add to session log
    sessionLog.push({
        timestamp: new Date().toISOString(),
        message: message
    });
}

function formatTimestamp() {
    if (!sessionStartTime) {
        return '00:00:00';
    }
    
    const elapsed = Math.floor((Date.now() - sessionStartTime) / 1000);
    const hours = Math.floor(elapsed / 3600);
    const minutes = Math.floor((elapsed % 3600) / 60);
    const seconds = elapsed % 60;
    
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
}

// =============================================================================
// GRAPH FUNCTIONS
// =============================================================================

function initGraphs() {
    chartContexts.wheel1Vel = document.getElementById('wheel1VelChart').getContext('2d');
    chartContexts.wheel2Vel = document.getElementById('wheel2VelChart').getContext('2d');
    chartContexts.wheel3Vel = document.getElementById('wheel3VelChart').getContext('2d');
    chartContexts.wheel1Ctrl = document.getElementById('wheel1CtrlChart').getContext('2d');
    chartContexts.wheel2Ctrl = document.getElementById('wheel2CtrlChart').getContext('2d');
    chartContexts.wheel3Ctrl = document.getElementById('wheel3CtrlChart').getContext('2d');
}

function updateGraphData() {
    const now = Date.now();
    if (now - lastGraphUpdate < GRAPH_UPDATE_INTERVAL) {
        return;
    }
    lastGraphUpdate = now;
    
    // Add new data points
    ['wheel1', 'wheel2', 'wheel3'].forEach(wheel => {
        const data = wheelData[wheel];
        graphData[wheel].setpoint.push(data.setpoint);
        graphData[wheel].measured.push(data.velocity);
        graphData[wheel].control.push(data.control);
        
        // Keep only recent history
        if (graphData[wheel].setpoint.length > maxDataPoints) {
            graphData[wheel].setpoint.shift();
            graphData[wheel].measured.shift();
            graphData[wheel].control.shift();
        }
    });
    
    // Log telemetry data
    if (tuningActive && sessionLog.length % 50 === 0) { // Log every 50 updates (~1 second at 50Hz)
        sessionLog.push({
            timestamp: new Date().toISOString(),
            data: JSON.parse(JSON.stringify(wheelData)) // Deep copy
        });
    }
    
    updateGraphs();
}

function updateGraphs() {
    drawVelocityGraph(chartContexts.wheel1Vel, graphData.wheel1, 'Wheel 1');
    drawVelocityGraph(chartContexts.wheel2Vel, graphData.wheel2, 'Wheel 2');
    drawVelocityGraph(chartContexts.wheel3Vel, graphData.wheel3, 'Wheel 3');
    
    drawControlGraph(chartContexts.wheel1Ctrl, graphData.wheel1, 'Wheel 1');
    drawControlGraph(chartContexts.wheel2Ctrl, graphData.wheel2, 'Wheel 2');
    drawControlGraph(chartContexts.wheel3Ctrl, graphData.wheel3, 'Wheel 3');
}

function drawVelocityGraph(ctx, data, label) {
    if (!ctx) return;
    
    const canvas = ctx.canvas;
    const width = canvas.width;
    const height = canvas.height;
    const padding = 50;
    const graphWidth = width - 2 * padding;
    const graphHeight = height - 2 * padding;
    
    // Clear canvas
    ctx.clearRect(0, 0, width, height);
    
    // Background gradient
    const gradient = ctx.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, '#242735');
    gradient.addColorStop(1, '#1a1d29');
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 0.5;
    
    for (let i = 0; i <= 4; i++) {
        const y = padding + (graphHeight / 4) * i;
        ctx.beginPath();
        ctx.moveTo(padding, y);
        ctx.lineTo(width - padding, y);
        ctx.stroke();
    }
    
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
    
    // Zero line
    const zeroY = padding + graphHeight / 2;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([8, 4]);
    ctx.beginPath();
    ctx.moveTo(padding, zeroY);
    ctx.lineTo(width - padding, zeroY);
    ctx.stroke();
    ctx.setLineDash([]);
    
    if (data.setpoint.length === 0) return;
    
    // Auto-scale based on data
    const allValues = [...data.setpoint, ...data.measured];
    const maxVal = Math.max(...allValues, config.maxWheelVelocity * 0.1);
    const minVal = Math.min(...allValues, -config.maxWheelVelocity * 0.1);
    const range = Math.max(Math.abs(maxVal), Math.abs(minVal));
    const scale = graphHeight / (2 * range);
    
    // Draw setpoint line (dashed blue)
    ctx.shadowColor = 'rgba(102, 126, 234, 0.6)';
    ctx.shadowBlur = 8;
    ctx.strokeStyle = '#7c8ef8';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    for (let i = 0; i < data.setpoint.length; i++) {
        const x = padding + (graphWidth / maxDataPoints) * i;
        const y = zeroY - data.setpoint[i] * scale;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    
    // Draw measured velocity (solid green)
    ctx.shadowColor = 'rgba(61, 220, 132, 0.6)';
    ctx.shadowBlur = 8;
    ctx.strokeStyle = '#3ddc84';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    for (let i = 0; i < data.measured.length; i++) {
        const x = padding + (graphWidth / maxDataPoints) * i;
        const y = zeroY - data.measured[i] * scale;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
    
    // Labels
    ctx.fillStyle = '#e0e0e0';
    ctx.font = 'bold 10px Segoe UI';
    ctx.textAlign = 'right';
    ctx.fillText(range.toFixed(1), padding - 8, padding + 5);
    ctx.fillText('0', padding - 8, zeroY + 5);
    ctx.fillText((-range).toFixed(1), padding - 8, height - padding + 5);
    
    // Legend
    const legendX = width - padding - 120;
    const legendY = padding + 5;
    ctx.fillStyle = 'rgba(26, 29, 41, 0.95)';
    ctx.fillRect(legendX - 5, legendY - 2, 115, 42);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1;
    ctx.strokeRect(legendX - 5, legendY - 2, 115, 42);
    
    ctx.font = 'bold 10px Segoe UI';
    ctx.textAlign = 'left';
    
    // Setpoint
    ctx.shadowColor = 'rgba(124, 142, 248, 0.6)';
    ctx.shadowBlur = 4;
    ctx.fillStyle = '#7c8ef8';
    ctx.beginPath();
    ctx.arc(legendX + 5, legendY + 8, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#e0e0e0';
    ctx.fillText('Setpoint', legendX + 15, legendY + 12);
    
    // Measured
    ctx.shadowColor = 'rgba(61, 220, 132, 0.6)';
    ctx.shadowBlur = 4;
    ctx.fillStyle = '#3ddc84';
    ctx.beginPath();
    ctx.arc(legendX + 5, legendY + 28, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#e0e0e0';
    ctx.fillText('Measured', legendX + 15, legendY + 32);
}

function drawControlGraph(ctx, data, label) {
    if (!ctx) return;
    
    const canvas = ctx.canvas;
    const width = canvas.width;
    const height = canvas.height;
    const padding = 50;
    const graphWidth = width - 2 * padding;
    const graphHeight = height - 2 * padding;
    
    // Clear canvas
    ctx.clearRect(0, 0, width, height);
    
    // Background
    const gradient = ctx.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, '#242735');
    gradient.addColorStop(1, '#1a1d29');
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, width, height);
    
    // Grid
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 4; i++) {
        const y = padding + (graphHeight / 4) * i;
        ctx.beginPath();
        ctx.moveTo(padding, y);
        ctx.lineTo(width - padding, y);
        ctx.stroke();
    }
    
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    for (let i = 0; i <= 10; i++) {
        const x = padding + (graphWidth / 10) * i;
        ctx.beginPath();
        ctx.moveTo(x, padding);
        ctx.lineTo(x, height - padding);
        ctx.stroke();
    }
    
    // Axes
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(padding, padding);
    ctx.lineTo(padding, height - padding);
    ctx.lineTo(width - padding, height - padding);
    ctx.stroke();
    
    // Zero line
    const zeroY = padding + graphHeight / 2;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([8, 4]);
    ctx.beginPath();
    ctx.moveTo(padding, zeroY);
    ctx.lineTo(width - padding, zeroY);
    ctx.stroke();
    ctx.setLineDash([]);
    
    if (data.control.length === 0) return;
    
    // Scale
    const maxControl = config.maxControlOutput;
    const scale = graphHeight / (2 * maxControl);
    
    // Draw control output (orange)
    ctx.shadowColor = 'rgba(255, 159, 64, 0.6)';
    ctx.shadowBlur = 8;
    ctx.strokeStyle = '#ff9f40';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    for (let i = 0; i < data.control.length; i++) {
        const x = padding + (graphWidth / maxDataPoints) * i;
        const y = zeroY - data.control[i] * scale;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
    
    // Labels
    ctx.fillStyle = '#e0e0e0';
    ctx.font = 'bold 10px Segoe UI';
    ctx.textAlign = 'right';
    ctx.fillText(maxControl.toFixed(0), padding - 8, padding + 5);
    ctx.fillText('0', padding - 8, zeroY + 5);
    ctx.fillText((-maxControl).toFixed(0), padding - 8, height - padding + 5);
    
    // Legend
    const legendX = width - padding - 100;
    const legendY = padding + 5;
    ctx.fillStyle = 'rgba(26, 29, 41, 0.95)';
    ctx.fillRect(legendX - 5, legendY - 2, 95, 25);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1;
    ctx.strokeRect(legendX - 5, legendY - 2, 95, 25);
    
    ctx.font = 'bold 10px Segoe UI';
    ctx.textAlign = 'left';
    ctx.shadowColor = 'rgba(255, 159, 64, 0.6)';
    ctx.shadowBlur = 4;
    ctx.fillStyle = '#ff9f40';
    ctx.beginPath();
    ctx.arc(legendX + 5, legendY + 10, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#e0e0e0';
    ctx.fillText('Control', legendX + 15, legendY + 14);
}

// =============================================================================
// INITIALIZATION
// =============================================================================

document.addEventListener('DOMContentLoaded', () => {
    // Initialize graphs
    initGraphs();
    
    // Connect WebSocket
    connectWebSocket();
    
    // Event listeners
    document.getElementById('startTuningBtn').addEventListener('click', onStartTuning);
    document.getElementById('pauseTuningBtn').addEventListener('click', onPauseTuning);
    document.getElementById('stopBtn').addEventListener('click', onEmergencyStop);
    document.getElementById('resetPlotBtn').addEventListener('click', onResetPlot);
    document.getElementById('applyPidBtn').addEventListener('click', onApplyPID);
    document.getElementById('applySetpointBtn').addEventListener('click', onApplySetpoint);
    document.getElementById('downloadLogBtn').addEventListener('click', onDownloadLog);
    document.getElementById('clearLogBtn').addEventListener('click', onClearLog);
    
    document.getElementById('samplingRateInput').addEventListener('change', onSamplingRateChange);
    document.getElementById('bufferSizeInput').addEventListener('change', onSamplingRateChange);
    
    // Initialize sampling configuration
    onSamplingRateChange();
    
    // Window blur - stop tuning
    window.addEventListener('blur', () => {
        if (tuningActive) {
            console.log('Window lost focus - pausing tuning');
            onPauseTuning();
        }
    });
    
    console.log('Wheel PID Tuning UI initialized');
});

// Visibility change handler
document.addEventListener('visibilitychange', () => {
    if (document.hidden && tuningActive) {
        console.log('Page hidden - pausing tuning');
        onPauseTuning();
    }
});
