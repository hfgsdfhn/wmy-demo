@echo off
setlocal

echo FreeRTOS Armclang Cortex-M7 automatic repair
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Use-FreeRTOS-ArmclangM7.ps1"
set "repairExitCode=%ERRORLEVEL%"

echo.
if not "%repairExitCode%"=="0" (
    echo Repair failed. Read the error above, then press any key to close.
    pause >nul
    exit /b %repairExitCode%
)

echo Repair completed. Press any key to close.
pause >nul
exit /b 0
