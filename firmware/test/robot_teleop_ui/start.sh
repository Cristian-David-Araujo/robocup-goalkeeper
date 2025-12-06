#!/bin/bash

# Quick start script for Robot Teleoperation UI

set -e

echo "=================================="
echo "Robot Teleoperation UI - Quick Start"
echo "=================================="
echo ""

# Check if .env exists, if not create from example
if [ ! -f .env ]; then
    echo "Creating .env file from .env.example..."
    cp .env.example .env
    echo "✓ Created .env file"
    echo "  Please edit .env to set your ROBOT_IP"
    echo ""
fi

# Check for Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed"
    exit 1
fi

echo "Python version: $(python3 --version)"
echo ""

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    echo "✓ Virtual environment created"
    echo ""
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install -q --upgrade pip
pip install -q -r requirements.txt
echo "✓ Dependencies installed"
echo ""

# Load environment variables
if [ -f .env ]; then
    export $(cat .env | grep -v '^#' | xargs)
fi

echo "=================================="
echo "Configuration:"
echo "  Server: ${TELEOP_HOST:-0.0.0.0}:${TELEOP_PORT:-8080}"
echo "  Robot: ${ROBOT_IP:-192.168.1.100}:${ROBOT_PORT:-3333}"
echo "  Max Linear Vel: ${MAX_LINEAR_VEL:-1.0} m/s"
echo "  Max Angular Vel: ${MAX_ANGULAR_VEL:-2.0} rad/s"
echo "=================================="
echo ""

echo "Starting server..."
echo "Open your browser to: http://localhost:${TELEOP_PORT:-8080}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the application
python app.py
