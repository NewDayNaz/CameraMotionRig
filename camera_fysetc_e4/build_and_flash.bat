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
echo ========================================
echo MANUAL BOOTLOADER MODE REQUIRED
echo ========================================
echo FYSETC E4 board doesn't have a BOOT button.
echo To enter bootloader mode:
echo.
echo METHOD 1 - GPIO0 to GND (Recommended):
echo   1. Connect GPIO0 pin to GND using a jumper wire
echo   2. Press and release the RESET button (or power cycle)
echo   3. Keep GPIO0 connected to GND for 1 second
echo   4. Remove the GPIO0 to GND connection
echo   5. Wait 1 second
echo.
echo METHOD 2 - Try automatic reset (if USB supports DTR/RTS):
echo   Just press RESET button right before flashing
echo   (esptool may be able to auto-enter bootloader mode)
echo.
echo Then press any key here to continue with flashing...
pause >nul
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

