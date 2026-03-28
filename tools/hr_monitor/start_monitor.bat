@echo off
REM PPG Heart Rate Monitor Launcher
REM Usage:
REM   start_monitor.bat          - Serial port mode
REM   start_monitor.bat --simulate - Simulated data mode (no hardware needed)

cd /d "%~dp0"

REM Activate conda environment
call conda activate ppg_prj

REM Launch the monitor
python main.py %*

REM Deactivate on exit
call conda deactivate
pause
