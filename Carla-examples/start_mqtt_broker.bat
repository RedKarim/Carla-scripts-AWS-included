@echo off
echo Starting MQTT Broker...
echo.

REM Check if Mosquitto service is already running
sc query mosquitto 2>nul | find "RUNNING" >nul
if %errorlevel% equ 0 (
    echo MQTT Broker (Mosquitto) is already running!
    echo Broker running on localhost:1883
    pause
    exit /b 0
)

REM Try to start Mosquitto as a service
echo Attempting to start Mosquitto service...
net start mosquitto 2>nul
if %errorlevel% equ 0 (
    echo MQTT Broker (Mosquitto) started successfully!
    echo Broker running on localhost:1883
    pause
    exit /b 0
)

REM Check if service exists
sc query mosquitto >nul 2>&1
if %errorlevel% equ 0 (
    echo Mosquitto service found but failed to start.
    echo.
    echo Please run as Administrator:
    echo   1. Right-click "start_mqtt_broker_admin.bat"
    echo   2. Select "Run as administrator"
    echo.
    echo Or manually run: net start mosquitto
    pause
    exit /b 1
)

REM Try to find and run mosquitto.exe
set MOSQUITTO_FOUND=0

if exist "C:\Program Files\mosquitto\mosquitto.exe" (
    set "MOSQUITTO_PATH=C:\Program Files\mosquitto\mosquitto.exe"
    set MOSQUITTO_FOUND=1
    goto :start_mosquitto
)

if exist "C:\Program Files (x86)\mosquitto\mosquitto.exe" (
    set "MOSQUITTO_PATH=C:\Program Files (x86)\mosquitto\mosquitto.exe"
    set MOSQUITTO_FOUND=1
    goto :start_mosquitto
)

if exist "%ProgramFiles%\mosquitto\mosquitto.exe" (
    set "MOSQUITTO_PATH=%ProgramFiles%\mosquitto\mosquitto.exe"
    set MOSQUITTO_FOUND=1
    goto :start_mosquitto
)

REM Try to run from PATH
where mosquitto >nul 2>&1
if %errorlevel% equ 0 (
    echo Starting Mosquitto broker directly...
    start "Mosquitto MQTT Broker" mosquitto
    timeout /t 2 >nul
    echo MQTT Broker started!
    echo Broker running on localhost:1883
    pause
    exit /b 0
)

REM Not found
goto :not_found

:start_mosquitto
if %MOSQUITTO_FOUND% equ 1 (
    echo Starting Mosquitto broker from: %MOSQUITTO_PATH%
    start "Mosquitto MQTT Broker" "%MOSQUITTO_PATH%"
    timeout /t 2 >nul
    echo MQTT Broker started!
    echo Broker running on localhost:1883
    pause
    exit /b 0
)

:not_found
echo.
echo ERROR: Mosquitto not found in PATH or common locations!
echo.
echo Please try one of these methods:
echo.
echo Method 1 - Use the Admin script (EASIEST):
echo   1. Right-click "start_mqtt_broker_admin.bat"
echo   2. Select "Run as administrator"
echo.
echo Method 2 - Start as Administrator manually:
echo   1. Right-click Command Prompt
echo   2. Select "Run as administrator"
echo   3. Navigate to this folder
echo   4. Run: net start mosquitto
echo.
echo Method 3 - Start manually:
echo   1. Open Command Prompt as Administrator
echo   2. Navigate to Mosquitto installation folder
echo   3. Run: mosquitto.exe
echo.
pause
