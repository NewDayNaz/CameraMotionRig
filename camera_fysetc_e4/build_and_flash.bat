@echo off
REM Build and flash script for FYSETC E4 firmware (Windows)
REM Usage: build_and_flash.bat [COM_PORT|OTA:IP_ADDRESS]
REM Example: build_and_flash.bat COM3
REM Example: build_and_flash.bat OTA:192.168.1.100

setlocal enabledelayedexpansion

REM Get COM port or OTA address from command line argument
set FLASH_MODE=SERIAL
set TARGET=%1

if "%1"=="" (
    echo Please specify target:
    echo   Serial flash: build_and_flash.bat COM3
    echo   OTA upload:   build_and_flash.bat OTA:192.168.1.100
    echo.
    set /p TARGET="Enter target (COM port or OTA:IP): "
)

REM Check if OTA mode
echo %TARGET% | findstr /C:"OTA:" >nul
if %errorlevel%==0 (
    set FLASH_MODE=OTA
    set OTA_IP=%TARGET:OTA:=%
    echo OTA mode: %OTA_IP%
) else (
    set COM_PORT=%TARGET%
    echo Serial mode: %COM_PORT%
)

echo ========================================
echo FYSETC E4 Firmware Build and Flash
echo ========================================
echo.
if "%FLASH_MODE%"=="OTA" (
    echo Mode: OTA Update
    echo Target IP: %OTA_IP%
) else (
    echo Mode: Serial Flash
    echo Using COM port: %COM_PORT%
)
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

if "%FLASH_MODE%"=="OTA" (
    echo.
    echo Step 2: Uploading firmware via OTA...
    echo.
    
    REM Find the firmware binary
    set FIRMWARE_BIN=build\camera_fysetc_e4.bin
    if not exist "%FIRMWARE_BIN%" (
        echo ERROR: Firmware binary not found at %FIRMWARE_BIN%
        echo Make sure the build completed successfully.
        pause
        exit /b 1
    )
    
    echo Uploading %FIRMWARE_BIN% to http://%OTA_IP%/api/update
    echo.
    
    REM Use PowerShell to upload the firmware
    powershell -NoProfile -Command "$firmwarePath = '%FIRMWARE_BIN%'; $ip = '%OTA_IP%'; $uri = 'http://' + $ip + '/api/update'; try { $fileBytes = [System.IO.File]::ReadAllBytes($firmwarePath); $fileSize = $fileBytes.Length; Write-Host \"Uploading $fileSize bytes...\"; $webClient = New-Object System.Net.WebClient; $webClient.Headers.Add('Content-Type', 'application/octet-stream'); $response = $webClient.UploadData($uri, 'POST', $fileBytes); $responseText = [System.Text.Encoding]::UTF8.GetString($response); Write-Host \"Response: $responseText\"; if ($responseText -match '\"status\":\"ok\"') { Write-Host \"OTA update successful! Device will reboot.\"; exit 0 } else { Write-Host \"OTA update may have failed. Check response above.\"; exit 1 } } catch { Write-Host \"Error: $_\"; exit 1 }"
    
    if errorlevel 1 (
        echo.
        echo OTA upload failed!
        echo Make sure:
        echo   - Device is connected to WiFi
        echo   - Device IP address is correct: %OTA_IP%
        echo   - Device is accessible on the network
        echo   - HTTP server is running on the device
        pause
        exit /b 1
    )
    
    echo.
    echo OTA update completed successfully!
    echo Device will reboot automatically. Wait 30 seconds and check the device.
    echo.
) else (
    echo.
    echo Step 2: Erasing flash (skipped by default)...
    REM Uncomment the next line to erase flash (recommended for first-time flashing)
    REM idf.py -p %COM_PORT% erase-flash
    
    echo.
    echo Step 3: Flashing firmware to %COM_PORT%...
    echo.
    echo Note: If flashing fails, you may need to manually enter bootloader mode:
    echo   1. Connect GPIO0 pin to GND using a jumper wire
    echo   2. Press and release the RESET button
    echo   3. Remove the GPIO0 to GND connection
    echo   4. Wait 1 second, then run: idf.py -p %COM_PORT% flash
    echo.
    echo Using baud rate 115200 for more reliable flashing
    timeout /t 1 /nobreak >nul
    idf.py -p %COM_PORT% flash -b 115200
    
    if errorlevel 1 (
        echo.
        echo Flash failed!
        pause
        exit /b 1
    )
    
    echo.
    echo Step 4: Starting serial monitor...
    echo Press Ctrl+] to exit monitor
    echo.
    idf.py -p %COM_PORT% monitor
)

pause

