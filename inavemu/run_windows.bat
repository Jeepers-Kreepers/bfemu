@echo off
rem Запуск виртуального FC INAV (Windows)
cd /d "%~dp0"
python -m inavemu --tcp 127.0.0.1:5761 --http 8088 %*
