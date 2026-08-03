@echo off
set "PYTHON_EXE=%LocalAppData%\Programs\Python\Python314\python.exe"
if not exist "%PYTHON_EXE%" (
    echo 未找到 Python 3.14。请先安装 Python，或编辑本文件中的 PYTHON_EXE 路径。
    pause
    exit /b 1
)
"%PYTHON_EXE%" "%~dp0main.py"
