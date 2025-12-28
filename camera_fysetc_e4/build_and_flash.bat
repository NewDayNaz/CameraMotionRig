@echo off
REM Build and flash script for FYSETC E4 firmware (Windows)
REM Usage: build_and_flash.bat [COM_PORT]
REM Example: build_and_flash.bat COM3

setlocal enabledelayedexpansion

REM Get COM port from command line argument, or prompt
if "%1"=="" (
    echo Please specify COM port: build_and_flash.bat COM3
    echo Or edit this script and set COM_PORT below
    set COM_PORT=COM3
) else (
    set COM_PORT=%1
)

echo ========================================
echo FYSETC E4 Firmware Build and Flash
echo ========================================
echo.
echo Using COM port: %COM_PORT%
echo.

REM Check if ESP-IDF environment is set
where idf.py >nul 2>&1
if errorlevel 1 (
    echo ERROR: ESP-IDF not found in PATH
    echo Please run ESP-IDF Command Prompt or set ESP-IDF environment
    echo.
    echo Trying to set ESP-IDF environment automatically...
    if exist "%userprofile%\esp\esp-idf\export.bat" (
        call "%userprofile%\esp\esp-idf\export.bat"
    ) else (
        echo ESP-IDF not found at default location: %userprofile%\esp\esp-idf\
        echo Please install ESP-IDF or set environment manually
        pause
        exit /b 1
    )
)

echo.
echo Step 1: Building firmware...
idf.py build
if errorlevel 1 (
    echo Build failed!
    pause
    exit /b 1
)

echo.
echo Step 2: Erasing flash (skipped by default)...
REM Uncomment the next line to erase flash (recommended for first-time flashing)
REM idf.py -p %COM_PORT% erase-flash

echo.
echo Step 3: Flashing firmware to %COM_PORT%...
echo.
echo Attempting to send BOOTLOADER command to enter download mode...
echo.
REM Try to send BOOTLOADER command via PowerShell
powershell -NoProfile -Command "$comPort = '%COM_PORT%'; $port = new-Object System.IO.Ports.SerialPort $comPort,115200,None,8,one; try { $port.Open(); $port.Write('BOOTLOADER'); $port.Write([char]10); Start-Sleep -Milliseconds 200; $port.Close(); Write-Host 'BOOTLOADER command sent successfully'; exit 0 } catch { Write-Host 'Note: Could not send BOOTLOADER command - device may not be connected or already in bootloader mode'; exit 1 }"
if errorlevel 1 (
    echo.
    echo Note: Could not send BOOTLOADER command automatically.
    echo The device may already be in bootloader mode, or you may need to:
    echo   1. Connect GPIO0 pin to GND using a jumper wire
    echo   2. Press and release the RESET button
    echo   3. Remove the GPIO0 to GND connection
    echo   4. Wait 1 second
    echo.
    echo Press any key to continue with flashing attempt...
    pause >nul
) else (
    echo BOOTLOADER command sent. Waiting for device to reboot into bootloader mode...
    timeout /t 3 /nobreak >nul
)
echo.
echo Using baud rate 115200 for more reliable flashing
timeout /t 1 /nobreak >nul
idf.py -p %COM_PORT% flash -b 115200

echo.
echo Step 4: Starting serial monitor...
echo Press Ctrl+] to exit monitor
echo.
idf.py -p %COM_PORT% monitor

pause

