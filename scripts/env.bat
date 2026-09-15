@echo off
set "PATH=%~dp0..\.venv\Scripts;C:\Program Files\Git\cmd;%PATH%"
for /d %%D in ("%~dp0..\.local-tools\xpack-arm-none-eabi-gcc-*") do set "PATH=%%~fD\bin;%PATH%"
for /d %%D in ("%~dp0..\.local-tools\xpack-openocd-*") do set "PATH=%%~fD\bin;%PATH%"
set PYTHONUTF8=1
