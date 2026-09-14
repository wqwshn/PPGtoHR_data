@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
if not exist "%~dp0.venv\Scripts\python.exe" (
    echo Project environment missing. See the workbench guide in docs.
    pause
    exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\monitor\main.py" %*
if errorlevel 1 pause

