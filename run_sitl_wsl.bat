@echo off
rem WSL launcher: ASCII with CRLF, for reliable cmd.exe execution.
setlocal
chcp 65001 >nul 2>nul
title bfemu - Betaflight SITL via WSL
cd /d "%~dp0"
echo ===============================================================
echo   bfemu / Betaflight SITL - Windows via Ubuntu WSL
echo   Browser: http://127.0.0.1:18788/aircraft#sitl-experiment
echo   Keep this window open. Windows FC settings are not changed.
echo ===============================================================
echo.
where wsl.exe >nul 2>nul
if errorlevel 1 goto no_wsl
if not exist "tools\run_wsl_sitl.sh" goto no_project
wsl.exe --cd "%~dp0." --exec bash tools/run_wsl_sitl.sh
set RC=%ERRORLEVEL%
echo.
if not "%RC%"=="0" (
    echo [X] WSL startup failed. Check the message above.
    echo     Ubuntu WSL must be installed and opened at least once.
    echo     First install: run PowerShell as Administrator, then:
    echo       wsl --install -d Ubuntu
    echo     Restart Windows if requested, open Ubuntu and create a user.
    echo     If --cd is not supported, update WSL: wsl --update
)
goto hold
:no_wsl
echo [X] WSL was not found. Windows 10 version 2004 or newer is required.
echo     Open PowerShell as Administrator and run: wsl --install -d Ubuntu
echo     Restart Windows if requested, open Ubuntu and create a user.
goto hold
:no_project
echo [X] Full project files were not found. Extract the complete bfemu folder.
:hold
echo.
echo Press any key to close this window.
pause >nul
