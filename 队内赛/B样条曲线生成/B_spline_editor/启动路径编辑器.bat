@echo off
setlocal
set "PYTHON_EXE=%LocalAppData%\Programs\Python\Python314\python.exe"
if not exist "%PYTHON_EXE%" (
    echo Python 3.14 was not found.
    pause
    exit /b 1
)
start "Robocon B-Spline Editor" /D "%~dp0" "%PYTHON_EXE%" "%~dp0main.py"
exit /b 0
