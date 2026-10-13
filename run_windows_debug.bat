@echo off
rem Debug launcher: shows every step and never closes the window.
setlocal
title bfemu - DEBUG
cd /d "%~dp0"
echo [debug] script dir: %~dp0
echo [debug] files here:
dir /b "%~dp0"
echo.
set PY=
where python >nul 2>nul
if not errorlevel 1 set PY=python
if not defined PY where py >nul 2>nul
if not defined PY set PY=py
echo [debug] python launcher: %PY%
%PY% --version
echo.
echo [debug] unpack check:
if exist "%~dp0bfemu\__main__.py" echo   OK %~dp0bfemu\__main__.py
if not exist "%~dp0bfemu\__main__.py" echo   MISSING %~dp0bfemu\__main__.py
echo.
echo [debug] starting emulator (Ctrl+C to stop)...
chcp 65001 >nul 2>nul
%PY% -u -m bfemu --tcp 127.0.0.1:5761 --http 8088 --log "%~dp0logs\debug.log"
echo.
echo [debug] exit code: %ERRORLEVEL%
echo [debug] log: %~dp0logs\debug.log
echo.
cmd /k
