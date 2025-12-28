@echo off
REM OTA upload script for FYSETC E4 firmware (Windows)
REM Usage: ota_upload.bat [IP_ADDRESS] [FIRMWARE_BIN]
REM Example: ota_upload.bat 192.168.1.100
REM Example: ota_upload.bat 192.168.1.100 build\camera_fysetc_e4.bin

setlocal enabledelayedexpansion

REM Get IP address and optional firmware path
if "%1"=="" (
    echo Usage: ota_upload.bat [IP_ADDRESS] [FIRMWARE_BIN]
    echo Example: ota_upload.bat 192.168.1.100
    echo.
    set /p OTA_IP="Enter device IP address: "
) else (
    set OTA_IP=%1
)

if "%2"=="" (
    set FIRMWARE_BIN=build\camera_fysetc_e4.bin
) else (
    set FIRMWARE_BIN=%2
)

if not exist "%FIRMWARE_BIN%" (
    echo ERROR: Firmware binary not found at %FIRMWARE_BIN%
    echo Make sure the build completed successfully or specify the correct path.
    pause
    exit /b 1
)

echo ========================================
echo OTA Firmware Upload
echo ========================================
echo.
echo Target IP: %OTA_IP%
echo Firmware: %FIRMWARE_BIN%
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
pause

