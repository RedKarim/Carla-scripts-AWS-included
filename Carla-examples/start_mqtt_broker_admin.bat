@echo off
REM This script requires Administrator privileges
REM Right-click and select "Run as administrator"

echo Checking for Administrator privileges...
net session >nul 2>&1
if errorlevel 1 (
    echo.
    echo ERROR: This script requires Administrator privileges!
    echo.
    echo Please:
    echo 1. Right-click this file
    echo 2. Select "Run as administrator"
    echo.
    pause
    exit /b 1
)

echo Administrator privileges confirmed.
echo.

REM Check if Mosquitto service is already running
sc query mosquitto | find "RUNNING" >nul 2>&1
if not errorlevel 1 (
    echo MQTT Broker (Mosquitto) is already running!
    echo Broker running on localhost:1883
    pause
    exit /b 0
)

REM Check if service exists
sc query mosquitto >nul 2>&1
if errorlevel 1 (
    echo Mosquitto service not found. Installing service...
    cd /d "C:\Program Files\Mosquitto" 2>nul
    if errorlevel 1 (
        cd /d "C:\Program Files (x86)\Mosquitto" 2>nul
        if errorlevel 1 (
            echo ERROR: Cannot find Mosquitto installation directory.
            echo Please navigate to Mosquitto folder and run: mosquitto.exe install
            pause
            exit /b 1
        )
    )
    mosquitto.exe install
    if errorlevel 1 (
        echo ERROR: Failed to install Mosquitto service.
        pause
        exit /b 1
    )
    echo Mosquitto service installed successfully.
)

REM Start the service
echo Starting Mosquitto service...
net start mosquitto
if errorlevel 1 (
    echo ERROR: Failed to start Mosquitto service.
    echo.
    echo Try checking the service status:
    echo   sc query mosquitto
    pause
    exit /b 1
)

echo.
echo ========================================
echo MQTT Broker started successfully!
echo Broker running on localhost:1883
echo ========================================
echo.
echo You can now run your CARLA manual control script.
echo.
pause
