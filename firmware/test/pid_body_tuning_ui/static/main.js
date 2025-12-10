// ============================================================================
// Body PID Tuning Interface - Main JavaScript
// ============================================================================

// WebSocket connection
let ws = null;
let reconnectInterval = null;
let telemetryRateCounter = 0;
let lastTelemetryRateUpdate = Date.now();

// Session state
let sessionActive = false;
let sessionStartTime = null;
let sessionPaused = false;
let dataLog = [];

// Graph configurations
const GRAPH_CONFIG = {
    timeWindow: 10000, // 10 seconds of data
    maxDataPoints: 500,
    gridColor: '#e5e7eb',
    setpointColor: '#f59e0b',
    measuredColor: '#3b82f6',
    controlColor: '#22c55e',
    lineWidth: 2
};

// Graph data storage
const graphData = {
    vx: { time: [], setpoint: [], measured: [], control: [] },
    vy: { time: [], setpoint: [], measured: [], control: [] },
    wz: { time: [], setpoint: [], measured: [], control: [] }
};

// Canvas contexts
let canvases = {};

// ============================================================================
// Initialization
// ============================================================================

document.addEventListener('DOMContentLoaded', () => {
    initializeCanvases();
    initializeInputHandlers();
    connectWebSocket();
    startTelemetryRateMonitor();
    startSessionTimer();
});

function initializeCanvases() {
    const canvasIds = [
        'vxVelocityCanvas', 'vxControlCanvas',
        'vyVelocityCanvas', 'vyControlCanvas',
        'wzVelocityCanvas', 'wzControlCanvas'
    ];

    canvasIds.forEach(id => {
        const canvas = document.getElementById(id);
        canvases[id] = canvas.getContext('2d');
        // Set high DPI scaling
        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        canvases[id].scale(dpr, dpr);
        canvas.style.width = rect.width + 'px';
        canvas.style.height = rect.height + 'px';
    });
}

function initializeInputHandlers() {
    // Update value displays when inputs change
    const axes = ['vx', 'vy', 'wz'];
    const params = ['kp', 'ki', 'kd', 'setpoint'];

    axes.forEach(axis => {
        params.forEach(param => {
            const input = document.getElementById(`${axis}_${param}`);
            const display = document.getElementById(`${axis}_${param}_val`);
            
            if (input && display) {
                input.addEventListener('input', (e) => {
                    const value = parseFloat(e.target.value) || 0;
                    if (param === 'setpoint') {
                        display.textContent = value.toFixed(2) + (axis === 'wz' ? ' rad/s' : ' m/s');
                    } else {
                        display.textContent = value.toFixed(2);
                    }
                });
            }
        });
    });
}

// ============================================================================
// WebSocket Connection
// ============================================================================

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    log('Connecting to server...', 'info');

    try {
        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            log('✓ Connected to server', 'success');
            updateConnectionStatus(true);
            
            if (reconnectInterval) {
                clearInterval(reconnectInterval);
                reconnectInterval = null;
            }
        };

        ws.onclose = () => {
            log('✗ Disconnected from server', 'error');
            updateConnectionStatus(false);
            
            // Attempt reconnection every 3 seconds
            if (!reconnectInterval) {
                reconnectInterval = setInterval(() => {
                    log('Attempting to reconnect...', 'info');
                    connectWebSocket();
                }, 3000);
            }
        };

        ws.onerror = (error) => {
            log('WebSocket error occurred', 'error');
            console.error('WebSocket error:', error);
        };

        ws.onmessage = (event) => {
            handleWebSocketMessage(event.data);
        };

    } catch (error) {
        log(`Connection failed: ${error.message}`, 'error');
        updateConnectionStatus(false);
    }
}

function updateConnectionStatus(connected) {
    const indicator = document.getElementById('wsStatus');
    const text = document.getElementById('wsStatusText');

    if (connected) {
        indicator.classList.remove('disconnected');
        indicator.classList.add('connected');
        text.textContent = 'Connected';
    } else {
        indicator.classList.remove('connected');
        indicator.classList.add('disconnected');
        text.textContent = 'Disconnected';
    }
}

// ============================================================================
// WebSocket Message Handling
// ============================================================================

function handleWebSocketMessage(data) {
    try {
        const message = JSON.parse(data);

        switch (message.type) {
            case 'telemetry':
                handleTelemetry(message);
                break;
            case 'status':
                handleStatus(message);
                break;
            case 'error':
                log(`Error: ${message.message}`, 'error');
                break;
            case 'ack':
                log(`✓ ${message.message}`, 'success');
                break;
            default:
                console.warn('Unknown message type:', message.type);
        }
    } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
    }
}

function handleTelemetry(message) {
    if (!sessionActive || sessionPaused) return;

    const timestamp = Date.now();
    telemetryRateCounter++;

    // Extract body velocity data
    const data = {
        vx: {
            setpoint: message.vx_setpoint || 0,
            measured: message.vx_measured || 0,
            control: message.vx_control || 0
        },
        vy: {
            setpoint: message.vy_setpoint || 0,
            measured: message.vy_measured || 0,
            control: message.vy_control || 0
        },
        wz: {
            setpoint: message.wz_setpoint || 0,
            measured: message.wz_measured || 0,
            control: message.wz_control || 0
        }
    };

    // Update graph data
    ['vx', 'vy', 'wz'].forEach(axis => {
        graphData[axis].time.push(timestamp);
        graphData[axis].setpoint.push(data[axis].setpoint);
        graphData[axis].measured.push(data[axis].measured);
        graphData[axis].control.push(data[axis].control);

        // Trim old data points
        const cutoffTime = timestamp - GRAPH_CONFIG.timeWindow;
        while (graphData[axis].time.length > 0 && graphData[axis].time[0] < cutoffTime) {
            graphData[axis].time.shift();
            graphData[axis].setpoint.shift();
            graphData[axis].measured.shift();
            graphData[axis].control.shift();
        }

        // Limit to max data points
        if (graphData[axis].time.length > GRAPH_CONFIG.maxDataPoints) {
            graphData[axis].time.shift();
            graphData[axis].setpoint.shift();
            graphData[axis].measured.shift();
            graphData[axis].control.shift();
        }
    });

    // Update current value displays
    document.getElementById('vx_current').textContent = data.vx.measured.toFixed(3);
    document.getElementById('vx_control_current').textContent = data.vx.control.toFixed(3);
    document.getElementById('vy_current').textContent = data.vy.measured.toFixed(3);
    document.getElementById('vy_control_current').textContent = data.vy.control.toFixed(3);
    document.getElementById('wz_current').textContent = data.wz.measured.toFixed(3);
    document.getElementById('wz_control_current').textContent = data.wz.control.toFixed(3);

    // Log data point
    dataLog.push({
        timestamp: new Date().toISOString(),
        elapsed: sessionStartTime ? (timestamp - sessionStartTime) / 1000 : 0,
        ...data
    });

    // Update data points counter
    document.getElementById('dataPoints').textContent = dataLog.length;

    // Render graphs
    renderGraphs();
}

function handleStatus(message) {
    if (message.robot_status) {
        document.getElementById('robotStatus').textContent = message.robot_status;
    }
}

// ============================================================================
// Graph Rendering
// ============================================================================

function renderGraphs() {
    renderVelocityGraph('vxVelocityCanvas', graphData.vx, 'VX');
    renderControlGraph('vxControlCanvas', graphData.vx);
    
    renderVelocityGraph('vyVelocityCanvas', graphData.vy, 'VY');
    renderControlGraph('vyControlCanvas', graphData.vy);
    
    renderVelocityGraph('wzVelocityCanvas', graphData.wz, 'WZ');
    renderControlGraph('wzControlCanvas', graphData.wz);
}

function renderVelocityGraph(canvasId, data, label) {
    const canvas = document.getElementById(canvasId);
    const ctx = canvases[canvasId];
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    // Clear canvas
    ctx.clearRect(0, 0, width, height);

    if (data.time.length === 0) {
        drawNoDataMessage(ctx, width, height);
        return;
    }

    // Calculate scales
    const timeMin = data.time[0];
    const timeMax = data.time[data.time.length - 1];
    const timeRange = timeMax - timeMin || 1;

    const allValues = [...data.setpoint, ...data.measured];
    const valueMin = Math.min(...allValues);
    const valueMax = Math.max(...allValues);
    const valueRange = valueMax - valueMin || 1;
    const valuePadding = valueRange * 0.1;

    const graphPadding = { left: 50, right: 20, top: 20, bottom: 30 };
    const graphWidth = width - graphPadding.left - graphPadding.right;
    const graphHeight = height - graphPadding.top - graphPadding.bottom;

    // Draw grid
    drawGrid(ctx, graphPadding, graphWidth, graphHeight, width, height);

    // Draw axes
    drawAxes(ctx, graphPadding, graphWidth, graphHeight, valueMin - valuePadding, valueMax + valuePadding);

    // Plot setpoint line
    plotLine(ctx, data.time, data.setpoint, timeMin, timeRange, valueMin - valuePadding, valueRange + 2 * valuePadding,
             graphPadding, graphWidth, graphHeight, GRAPH_CONFIG.setpointColor, 2, [5, 5]);

    // Plot measured line
    plotLine(ctx, data.time, data.measured, timeMin, timeRange, valueMin - valuePadding, valueRange + 2 * valuePadding,
             graphPadding, graphWidth, graphHeight, GRAPH_CONFIG.measuredColor, GRAPH_CONFIG.lineWidth);

    // Draw legend
    drawLegend(ctx, width, graphPadding.top, ['Setpoint', 'Measured'], 
               [GRAPH_CONFIG.setpointColor, GRAPH_CONFIG.measuredColor]);
}

function renderControlGraph(canvasId, data) {
    const canvas = document.getElementById(canvasId);
    const ctx = canvases[canvasId];
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    // Clear canvas
    ctx.clearRect(0, 0, width, height);

    if (data.time.length === 0) {
        drawNoDataMessage(ctx, width, height);
        return;
    }

    // Calculate scales
    const timeMin = data.time[0];
    const timeMax = data.time[data.time.length - 1];
    const timeRange = timeMax - timeMin || 1;

    const controlMin = Math.min(...data.control);
    const controlMax = Math.max(...data.control);
    const controlRange = controlMax - controlMin || 1;
    const controlPadding = controlRange * 0.1;

    const graphPadding = { left: 50, right: 20, top: 20, bottom: 30 };
    const graphWidth = width - graphPadding.left - graphPadding.right;
    const graphHeight = height - graphPadding.top - graphPadding.bottom;

    // Draw grid
    drawGrid(ctx, graphPadding, graphWidth, graphHeight, width, height);

    // Draw axes
    drawAxes(ctx, graphPadding, graphWidth, graphHeight, controlMin - controlPadding, controlMax + controlPadding);

    // Plot control line
    plotLine(ctx, data.time, data.control, timeMin, timeRange, controlMin - controlPadding, controlRange + 2 * controlPadding,
             graphPadding, graphWidth, graphHeight, GRAPH_CONFIG.controlColor, GRAPH_CONFIG.lineWidth);

    // Draw legend
    drawLegend(ctx, width, graphPadding.top, ['Control Output'], [GRAPH_CONFIG.controlColor]);
}

function drawGrid(ctx, padding, graphWidth, graphHeight, totalWidth, totalHeight) {
    ctx.strokeStyle = GRAPH_CONFIG.gridColor;
    ctx.lineWidth = 1;

    // Horizontal grid lines
    for (let i = 0; i <= 5; i++) {
        const y = padding.top + (graphHeight / 5) * i;
        ctx.beginPath();
        ctx.moveTo(padding.left, y);
        ctx.lineTo(padding.left + graphWidth, y);
        ctx.stroke();
    }

    // Vertical grid lines
    for (let i = 0; i <= 10; i++) {
        const x = padding.left + (graphWidth / 10) * i;
        ctx.beginPath();
        ctx.moveTo(x, padding.top);
        ctx.lineTo(x, padding.top + graphHeight);
        ctx.stroke();
    }
}

function drawAxes(ctx, padding, graphWidth, graphHeight, minValue, maxValue) {
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2;

    // Y-axis
    ctx.beginPath();
    ctx.moveTo(padding.left, padding.top);
    ctx.lineTo(padding.left, padding.top + graphHeight);
    ctx.stroke();

    // X-axis
    ctx.beginPath();
    ctx.moveTo(padding.left, padding.top + graphHeight);
    ctx.lineTo(padding.left + graphWidth, padding.top + graphHeight);
    ctx.stroke();

    // Y-axis labels
    ctx.fillStyle = '#666';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    for (let i = 0; i <= 5; i++) {
        const y = padding.top + (graphHeight / 5) * i;
        const value = maxValue - ((maxValue - minValue) / 5) * i;
        ctx.fillText(value.toFixed(2), padding.left - 5, y);
    }
}

function plotLine(ctx, timeData, valueData, timeMin, timeRange, valueMin, valueRange,
                  padding, graphWidth, graphHeight, color, lineWidth, dash = []) {
    if (timeData.length === 0) return;

    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.setLineDash(dash);
    ctx.beginPath();

    for (let i = 0; i < timeData.length; i++) {
        const x = padding.left + ((timeData[i] - timeMin) / timeRange) * graphWidth;
        const y = padding.top + graphHeight - ((valueData[i] - valueMin) / valueRange) * graphHeight;

        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    }

    ctx.stroke();
    ctx.setLineDash([]);
}

function drawNoDataMessage(ctx, width, height) {
    ctx.fillStyle = '#999';
    ctx.font = '14px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('No data available. Start tuning to see graphs.', width / 2, height / 2);
}

function drawLegend(ctx, width, topPadding, labels, colors) {
    const legendX = width - 150;
    const legendY = topPadding + 5;
    const lineLength = 20;
    const spacing = 80;

    ctx.font = '11px sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';

    labels.forEach((label, i) => {
        const x = legendX + i * spacing;
        
        // Draw line
        ctx.strokeStyle = colors[i];
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, legendY);
        ctx.lineTo(x + lineLength, legendY);
        ctx.stroke();

        // Draw label
        ctx.fillStyle = '#333';
        ctx.fillText(label, x + lineLength + 5, legendY);
    });
}

// ============================================================================
// Control Functions
// ============================================================================

function applyPID(axis) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        log('Cannot apply PID: Not connected to server', 'error');
        return;
    }

    const kp = parseFloat(document.getElementById(`${axis}_kp`).value) || 0;
    const ki = parseFloat(document.getElementById(`${axis}_ki`).value) || 0;
    const kd = parseFloat(document.getElementById(`${axis}_kd`).value) || 0;

    if (kp < 0 || ki < 0 || kd < 0) {
        log('PID parameters must be non-negative', 'error');
        return;
    }

    const message = {
        type: 'apply_pid',
        axis: axis,
        kp: kp,
        ki: ki,
        kd: kd
    };

    ws.send(JSON.stringify(message));
    log(`Applied ${axis.toUpperCase()} PID: Kp=${kp.toFixed(2)}, Ki=${ki.toFixed(2)}, Kd=${kd.toFixed(2)}`, 'info');
}

function startTuning() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        log('Cannot start tuning: Not connected to server', 'error');
        return;
    }

    // Get all setpoints
    const vx_setpoint = parseFloat(document.getElementById('vx_setpoint').value) || 0;
    const vy_setpoint = parseFloat(document.getElementById('vy_setpoint').value) || 0;
    const wz_setpoint = parseFloat(document.getElementById('wz_setpoint').value) || 0;

    const message = {
        type: 'start_tuning',
        vx_setpoint: vx_setpoint,
        vy_setpoint: vy_setpoint,
        wz_setpoint: wz_setpoint
    };

    ws.send(JSON.stringify(message));
    
    sessionActive = true;
    sessionPaused = false;
    sessionStartTime = Date.now();
    
    document.getElementById('sessionState').textContent = 'Active';
    document.getElementById('robotStatus').textContent = 'Tuning Active';
    
    log('Body tuning session started', 'success');
}

function pauseTuning() {
    sessionPaused = !sessionPaused;
    document.getElementById('sessionState').textContent = sessionPaused ? 'Paused' : 'Active';
    log(sessionPaused ? 'Session paused' : 'Session resumed', 'info');
}

function resetSession() {
    if (!confirm('Reset session? This will clear all graph data and logs.')) {
        return;
    }

    // Clear graph data
    ['vx', 'vy', 'wz'].forEach(axis => {
        graphData[axis].time = [];
        graphData[axis].setpoint = [];
        graphData[axis].measured = [];
        graphData[axis].control = [];
    });

    // Clear data log
    dataLog = [];
    
    // Reset session state
    sessionActive = false;
    sessionPaused = false;
    sessionStartTime = null;
    
    document.getElementById('sessionState').textContent = 'Idle';
    document.getElementById('dataPoints').textContent = '0';
    document.getElementById('sessionTime').textContent = '00:00:00';
    
    // Re-render empty graphs
    renderGraphs();
    
    log('Session reset complete', 'info');
}

function emergencyStop() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        log('Cannot send E-STOP: Not connected to server', 'error');
        return;
    }

    const message = { type: 'emergency_stop' };
    ws.send(JSON.stringify(message));

    sessionActive = false;
    sessionPaused = false;
    
    document.getElementById('sessionState').textContent = 'Stopped';
    document.getElementById('robotStatus').textContent = 'Emergency Stop';
    
    log('🛑 EMERGENCY STOP activated', 'error');
}

function downloadLog() {
    if (dataLog.length === 0) {
        log('No data to download', 'warning');
        return;
    }

    // Create CSV header
    let csv = 'Timestamp,Elapsed(s),VX_Setpoint,VX_Measured,VX_Control,VY_Setpoint,VY_Measured,VY_Control,WZ_Setpoint,WZ_Measured,WZ_Control\n';

    // Add data rows
    dataLog.forEach(entry => {
        csv += `${entry.timestamp},${entry.elapsed.toFixed(3)},`;
        csv += `${entry.vx.setpoint.toFixed(4)},${entry.vx.measured.toFixed(4)},${entry.vx.control.toFixed(4)},`;
        csv += `${entry.vy.setpoint.toFixed(4)},${entry.vy.measured.toFixed(4)},${entry.vy.control.toFixed(4)},`;
        csv += `${entry.wz.setpoint.toFixed(4)},${entry.wz.measured.toFixed(4)},${entry.wz.control.toFixed(4)}\n`;
    });

    // Create download link
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `body_tuning_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);

    log(`Downloaded ${dataLog.length} data points as CSV`, 'success');
}

// ============================================================================
// Utility Functions
// ============================================================================

function log(message, type = 'info') {
    const logDisplay = document.getElementById('logDisplay');
    const timestamp = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry log-type-${type}`;
    entry.innerHTML = `<span class="log-timestamp">[${timestamp}]</span> ${message}`;
    logDisplay.appendChild(entry);
    logDisplay.scrollTop = logDisplay.scrollHeight;

    // Keep only last 100 log entries
    while (logDisplay.children.length > 100) {
        logDisplay.removeChild(logDisplay.firstChild);
    }
}

function clearLog() {
    const logDisplay = document.getElementById('logDisplay');
    logDisplay.innerHTML = '<div class="log-entry log-type-info"><span class="log-timestamp"></span>Log cleared</div>';
}

function startTelemetryRateMonitor() {
    setInterval(() => {
        const now = Date.now();
        const elapsed = (now - lastTelemetryRateUpdate) / 1000;
        const rate = telemetryRateCounter / elapsed;
        
        document.getElementById('telemetryRate').textContent = rate.toFixed(1) + ' Hz';
        
        telemetryRateCounter = 0;
        lastTelemetryRateUpdate = now;
    }, 1000);
}

function startSessionTimer() {
    setInterval(() => {
        if (sessionActive && !sessionPaused && sessionStartTime) {
            const elapsed = Math.floor((Date.now() - sessionStartTime) / 1000);
            const hours = Math.floor(elapsed / 3600);
            const minutes = Math.floor((elapsed % 3600) / 60);
            const seconds = elapsed % 60;
            
            document.getElementById('sessionTime').textContent = 
                `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
        }
    }, 1000);
}
