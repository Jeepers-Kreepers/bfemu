@echo off
rem ===================================================================
rem  bfemu - virtual Betaflight flight controller (MSP + CLI)
rem  This file is intentionally ASCII-only and uses CRLF line endings:
rem  cmd.exe mis-parses LF-only batch files, which closes the window.
rem ===================================================================
setlocal
title bfemu - virtual Betaflight FC
cd /d "%~dp0"

rem --- port can be overridden:  run_windows.bat 6761 -----------------
set PORT=%~1
if "%PORT%"=="" set PORT=5761

echo ============================================================
echo   bfemu - virtual Betaflight 4.5.4 (MSP API 1.46)
echo.
echo   Configurator: manual connection -^> tcp://127.0.0.1:%PORT%
echo   Web monitor : http://127.0.0.1:8088
echo   Virtual TX  : http://127.0.0.1:8088/tx   (move sticks, channels go live)
echo   Config kept in: eeprom.json (delete it or run --wipe for factory reset)
echo.
echo   Keep this window open. Ctrl+C stops the emulator.
echo ============================================================
echo.

rem --- 1. locate the package (bfemu\__main__.py) ---------------------
set ROOT=%~dp0
if not exist "%ROOT%bfemu\__main__.py" if exist "%ROOT%bfemu\bfemu\__main__.py" set ROOT=%~dp0bfemu\
if not exist "%ROOT%bfemu\__main__.py" (
    echo [X] Package not found.
    echo     Expected: %ROOT%bfemu\__main__.py
    echo     Unpack the archive so that run_windows.bat and the "bfemu"
    echo     folder sit next to each other, then run the bat again.
    goto hold
)
cd /d "%ROOT%"

rem --- 2. locate Python 3.10+ ----------------------------------------
set PY=
where python >nul 2>nul
if not errorlevel 1 set PY=python
if not defined PY (
    where py >nul 2>nul
    if not errorlevel 1 set PY=py
)
if not defined PY (
    echo [X] Python not found in PATH.
    echo     Install Python 3.10+ from https://www.python.org/downloads/
    echo     and tick "Add python.exe to PATH" during installation.
    echo     Then close this window and run the bat again.
    goto hold
)
%PY% -c "import sys; raise SystemExit(0 if sys.version_info>=(3,10) else 1)" >nul 2>nul
if errorlevel 1 (
    echo [X] Python 3.10 or newer is required.
    goto hold
)
for /f "delims=" %%v in ('%PY% --version 2^>^&1') do echo [i] %%~v

rem --- 3. make sure the console speaks UTF-8 -------------------------
chcp 65001 >nul 2>nul

rem --- 4. run ---------------------------------------------------------
echo [i] Starting virtual FC on TCP port %PORT% ...
echo [i] Log file: %~dp0logs\bfemu.log
echo.
%PY% -u -m bfemu --tcp 127.0.0.1:%PORT% --http 8088 --eeprom "%~dp0eeprom.json" --log "%~dp0logs\bfemu.log"
set RC=%ERRORLEVEL%
echo.
if "%RC%"=="3" echo [X] A port is busy - read the hint above, or run:  run_windows.bat 6761
if "%RC%"=="1" echo [X] The emulator stopped with an error - see logs\bfemu.log
if "%RC%"=="0" echo [i] The emulator stopped normally.

:hold
echo.
echo ------------------------------------------------------------
echo Window kept open on purpose. Press any key to close.
pause >nul
