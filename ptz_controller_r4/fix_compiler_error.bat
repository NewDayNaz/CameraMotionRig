@echo off
REM Quick fix for internal compiler error in Binary.h
REM This updates the Arduino R4 core to the latest version

echo ========================================
echo Fixing Arduino R4 Compiler Error
echo ========================================
echo.
echo This script will update the Arduino R4 core to the latest version
echo which may fix the internal compiler error in Binary.h
echo.

REM Check if Arduino CLI is installed
where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo ERROR: Arduino CLI not found
    echo Please install Arduino CLI first
    pause
    exit /b 1
)

echo Step 1: Updating core index...
arduino-cli core update-index
if errorlevel 1 (
    echo Failed to update index
    pause
    exit /b 1
)

echo.
echo Step 2: Upgrading Arduino R4 core to latest version...
arduino-cli core upgrade arduino:renesas_uno
if errorlevel 1 (
    echo Failed to upgrade core
    echo Trying to reinstall...
    arduino-cli core uninstall arduino:renesas_uno
    arduino-cli core install arduino:renesas_uno
)

echo.
echo Step 3: Checking installed version...
arduino-cli core list

echo.
echo ========================================
echo Fix complete!
echo ========================================
echo.
echo If the error persists, try:
echo   1. Use Arduino IDE 2.3.0+ instead
echo   2. Check Arduino forum for latest fixes
echo   3. See TROUBLESHOOTING.md for more options
echo.
pause


