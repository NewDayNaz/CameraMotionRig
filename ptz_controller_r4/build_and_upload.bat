@echo off
REM Build and upload script for PTZ Controller (Arduino Uno R4 Minima)
REM Usage: build_and_upload.bat [COM_PORT]
REM Example: build_and_upload.bat COM3

setlocal

REM Get COM port from command line argument, or prompt
if "%1"=="" (
    echo Please specify COM port: build_and_upload.bat COM3
    echo Or edit this script and set COM_PORT below
    set COM_PORT=COM3
) else (
    set COM_PORT=%1
)

echo ========================================
echo PTZ Controller Build and Upload
echo Arduino Uno R4 Minima
echo ========================================
echo.
echo Using COM port: %COM_PORT%
echo.

REM Check if Arduino CLI is installed
where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo ERROR: Arduino CLI not found in PATH
    echo.
    echo Option 1: Install Arduino CLI
    echo   Download from: https://arduino.github.io/arduino-cli/
    echo   Or install via: winget install Arduino.ArduinoCLI
    echo.
    echo Option 2: Use Arduino IDE
    echo   Open ptz_controller_r4.ino in Arduino IDE
    echo   Select board: Arduino Uno R4 Minima
    echo   Select port: %COM_PORT%
    echo   Click Upload
    echo.
    pause
    exit /b 1
)

echo Step 1: Checking Arduino CLI configuration...
arduino-cli config init 2>nul
arduino-cli core update-index
if errorlevel 1 (
    echo Failed to update core index
    pause
    exit /b 1
)

echo.
echo Step 2: Installing/Updating Arduino Uno R4 Minima core...
arduino-cli core install arduino:renesas_uno
if errorlevel 1 (
    echo Failed to install core
    pause
    exit /b 1
)

REM Try to update to latest version
arduino-cli core upgrade arduino:renesas_uno

echo.
echo NOTE: If you encounter "internal compiler error" in Binary.h, this is a known
echo bug in the Renesas core. Try:
echo   1. Update Arduino CLI: arduino-cli core upgrade arduino:renesas_uno
echo   2. Use Arduino IDE 2.3.0 or later (may have better compiler)
echo   3. Check Arduino forum for latest fixes
echo.

echo.
echo Step 3: Compiling firmware...
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi ptz_controller_r4.ino
if errorlevel 1 (
    echo.
    echo Trying alternative FQBN for R4 Minima...
    arduino-cli compile --fqbn arduino:renesas_uno:unor4_minima ptz_controller_r4.ino
    if errorlevel 1 (
        echo Compilation failed!
        echo.
        echo Make sure:
        echo   - All source files are in the same directory as ptz_controller_r4.ino
        echo   - Arduino Uno R4 core is installed
        pause
        exit /b 1
    )
    set FQBN=arduino:renesas_uno:unor4_minima
) else (
    set FQBN=arduino:renesas_uno:unor4wifi
)

echo.
echo Step 4: Uploading firmware to %COM_PORT%...
arduino-cli upload -p %COM_PORT% --fqbn %FQBN% ptz_controller_r4.ino
if errorlevel 1 (
    echo Upload failed!
    echo.
    echo Make sure:
    echo   - Board is connected to %COM_PORT%
    echo   - USB cable supports data transfer
    echo   - No other program is using the serial port
    echo   - Board is in upload mode (may need to press RESET button)
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo Upload successful!
echo ========================================
echo.
echo Step 5: Opening serial monitor...
echo Press Ctrl+C to exit monitor
echo.
timeout /t 2 /nobreak >nul
arduino-cli monitor -p %COM_PORT% -c baudrate=115200

pause

