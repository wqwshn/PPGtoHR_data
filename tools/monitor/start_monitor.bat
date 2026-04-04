@echo off
REM PPG Monitor Launcher
REM Usage:
REM   start_monitor.bat              - Serial port mode
REM   start_monitor.bat --simulate   - HR simulated data mode
REM   start_monitor.bat --raw-simulate - Raw data simulated mode (125Hz)

cd /d "%~dp0"

REM Activate conda environment
call conda activate ppg_prj

REM Launch the monitor
python main.py %*

REM Deactivate on exit
call conda deactivate
pause
