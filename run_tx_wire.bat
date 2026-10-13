@echo off
rem ---------------------------------------------------------------------------
rem  Read a REAL receiver from a COM port (USB-UART adapter) and lead the
rem  emulator channels with it - the real link and the real bind.
rem  CRSF 420000 / SBUS 100000 are detected automatically.
rem  Ctrl+C stops the script.
rem ---------------------------------------------------------------------------
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py

%PY% -c "import serial" >nul 2>nul
if errorlevel 1 (
    echo pyserial is not installed - installing now ...
    %PY% -m pip install pyserial
    if errorlevel 1 (
        echo.
        echo Could not install pyserial. Run manually:  %PY% -m pip install pyserial
        pause
        exit /b 1
    )
)

echo.
echo Available serial ports:
%PY% tools\tx.py ports
echo.
set /p COMPORT=Type the COM port of the receiver (e.g. COM5): 
if "%COMPORT%"=="" (
    echo No port given - nothing to do.
    pause
    exit /b 1
)
echo.
echo Reading receiver on %COMPORT% - CRSF and SBUS are detected automatically.
echo Ctrl+C stops.
echo.
%PY% tools\tx.py wire --port %COMPORT%
echo.
pause
