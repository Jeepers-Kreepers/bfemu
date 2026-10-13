@echo off
rem Serves the virtual FC on a real serial port (com0com pair: COM10 ^<->^ COM11).
rem Usage:  run_windows_com.bat COM11
setlocal
title bfemu - COM bridge
cd /d "%~dp0"

set COMPORT=%~1
if "%COMPORT%"=="" set COMPORT=COM11

echo ============================================================
echo   Virtual FC on serial port %COMPORT%
echo   Connect Betaflight Configurator to the TWIN port (e.g. COM10).
echo   Create the pair first:  com0com Setup Command Prompt
echo       install PortName=COM10 PortName=COM11
echo ============================================================
echo.

set PY=
where python >nul 2>nul
if not errorlevel 1 set PY=python
if not defined PY (
    where py >nul 2>nul
    if not errorlevel 1 set PY=py
)
if not defined PY (
    echo [X] Python not found in PATH. Install Python 3.10+ and tick "Add to PATH".
    goto hold
)
for /f "delims=" %%v in ('%PY% --version 2^>^&1') do echo [i] %%~v
chcp 65001 >nul 2>nul
%PY% -u -m bfemu --com %COMPORT% --http 8088 --eeprom "%~dp0eeprom.json" --log "%~dp0logs\bfemu-com.log"
echo.
echo [i] Stopped (exit code %ERRORLEVEL%). Log: logs\bfemu-com.log

:hold
echo.
echo Window kept open. Press any key to close.
pause >nul
