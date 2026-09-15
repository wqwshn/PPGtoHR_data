@echo off
setlocal
cd /d "%~dp0"
call "%~dp0scripts\env.bat"
if defined PPG_PYTHON goto run
set "PPG_PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%PPG_PYTHON%" set "PPG_PYTHON=%~dp0..\..\.venv\Scripts\python.exe"
if not exist "%PPG_PYTHON%" set "PPG_PYTHON=python"
:run
"%PPG_PYTHON%" -c "import PyQt5, pyqtgraph, serial, numpy" >nul 2>&1
if errorlevel 1 (
    echo Python dependencies missing. Activate your project environment or set PPG_PYTHON.
    echo See docs/README.md for setup instructions.
    pause
    exit /b 1
)
"%PPG_PYTHON%" "%~dp0tools\monitor\main.py" %*
if errorlevel 1 pause
