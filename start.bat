@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set "PPG_PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%PPG_PYTHON%" set "PPG_PYTHON=%~dp0..\..\.venv\Scripts\python.exe"
if not exist "%PPG_PYTHON%" (
    echo Project environment missing. See docs for setup instructions.
    pause
    exit /b 1
)
"%PPG_PYTHON%" "%~dp0tools\monitor\main.py" %*
if errorlevel 1 pause
