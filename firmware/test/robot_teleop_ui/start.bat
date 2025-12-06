@echo off
REM Quick start script for Robot Teleoperation UI (Windows)

echo ==================================
echo Robot Teleoperation UI - Quick Start
echo ==================================
echo.

REM Check if .env exists, if not create from example
if not exist .env (
    echo Creating .env file from .env.example...
    copy .env.example .env
    echo [OK] Created .env file
    echo   Please edit .env to set your ROBOT_IP
    echo.
)

REM Check for Python
where python >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed or not in PATH
    pause
    exit /b 1
)

python --version
echo.

REM Create virtual environment if it doesn't exist
if not exist venv (
    echo Creating virtual environment...
    python -m venv venv
    echo [OK] Virtual environment created
    echo.
)

REM Activate virtual environment
echo Activating virtual environment...
call venv\Scripts\activate.bat

REM Install dependencies
echo Installing dependencies...
pip install -q --upgrade pip
pip install -q -r requirements.txt
echo [OK] Dependencies installed
echo.

REM Load environment variables from .env file
if exist .env (
    for /f "tokens=*" %%a in (.env) do (
        set "line=%%a"
        if not "!line:~0,1!"=="#" (
            set "%%a"
        )
    )
)

echo ==================================
echo Configuration:
echo   Server: %TELEOP_HOST%:%TELEOP_PORT%
echo   Robot: %ROBOT_IP%:%ROBOT_PORT%
echo   Max Linear Vel: %MAX_LINEAR_VEL% m/s
echo   Max Angular Vel: %MAX_ANGULAR_VEL% rad/s
echo ==================================
echo.

echo Starting server...
echo Open your browser to: http://localhost:%TELEOP_PORT%
echo.
echo Press Ctrl+C to stop
echo.

REM Run the application
python app.py
