#!/bin/bash
# Запуск виртуального FC INAV (Linux/macOS)
cd "$(dirname "$0")"
python3 -m inavemu --tcp 127.0.0.1:5761 --http 8088 "$@"
