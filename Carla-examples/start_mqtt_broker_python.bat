@echo off
echo Starting Python MQTT Broker...
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH!
    echo Please install Python and try again.
    pause
    exit /b 1
)

REM Check if hbmqtt is installed
python -c "import hbmqtt" >nul 2>&1
if errorlevel 1 (
    echo hbmqtt library not found. Installing...
    echo.
    pip install hbmqtt
    if errorlevel 1 (
        echo.
        echo ERROR: Failed to install hbmqtt!
        echo Please install manually: pip install hbmqtt
        pause
        exit /b 1
    )
    echo.
    echo hbmqtt installed successfully!
    echo.
)

echo Starting MQTT broker...
echo.
python mqtt_broker.py

pause
