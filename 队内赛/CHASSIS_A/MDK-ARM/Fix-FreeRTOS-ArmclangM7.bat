@echo off
setlocal

echo FreeRTOS Armclang Cortex-M7 automatic repair
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Use-FreeRTOS-ArmclangM7.ps1" -Offline
set "repairExitCode=%ERRORLEVEL%"

echo.
if not "%repairExitCode%"=="0" (
    if /I "%~1"=="--non-interactive" exit /b %repairExitCode%
    echo Repair failed. Read the error above, then press any key to close.
    pause >nul
    exit /b %repairExitCode%
)

if /I "%~1"=="--non-interactive" exit /b 0
echo Repair completed. Press any key to close.
pause >nul
exit /b 0
