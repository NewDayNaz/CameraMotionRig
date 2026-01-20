@echo off
cd /D "%~dp0"

:: Service name (must match the one in install_service.bat)
set SERVICE_NAME=node-camera-midi-bridge

echo Restarting service: %SERVICE_NAME%

:: Check for admin privileges
net session >nul 2>&1
if %errorLevel% == 0 (
    echo Administrative permissions confirmed.
) else (
    echo Warning: Administrative permissions may be required. Continuing anyway...
)

:: Try to restart using nssm (if available)
where nssm >nul 2>&1
if %errorLevel% == 0 (
    echo Using NSSM to restart service...
    nssm restart %SERVICE_NAME%
    if %errorLevel% == 0 (
        echo Service restarted successfully.
    ) else (
        echo Failed to restart service using NSSM.
        echo Attempting stop/start method...
        nssm stop %SERVICE_NAME%
        timeout /t 2 /nobreak >nul
        nssm start %SERVICE_NAME%
        if %errorLevel% == 0 (
            echo Service restarted successfully.
        ) else (
            echo Failed to restart service. Please check service status manually.
            exit /B 1
        )
    )
) else (
    :: Fall back to sc command if nssm is not in PATH
    echo NSSM not found in PATH, using sc command...
    sc stop %SERVICE_NAME%
    timeout /t 2 /nobreak >nul
    sc start %SERVICE_NAME%
    if %errorLevel% == 0 (
        echo Service restarted successfully.
    ) else (
        echo Failed to restart service. Please check service status manually.
        exit /B 1
    )
)

pause
