#!/bin/sh
# bfemu — запуск на Linux/macOS.
#
# TCP слушает 127.0.0.1 (как и .bat-файлы на Windows): эмулятор отдаёт не только телеметрию,
# но и запись конфигурации, тест моторов и /api/rc — в общей сети это чужие руки на стиках.
# Нужен доступ с другой машины — запустите с явным адресом:
#     python3 -m bfemu --tcp 0.0.0.0:5761 --http 8088 --http-host 0.0.0.0
set -eu
cd "$(dirname "$0")" || exit 1
PORT="${1:-5761}"
HOST="${BFEMU_HOST:-127.0.0.1}"
mkdir -p logs
exec python3 -u -m bfemu --tcp "$HOST:$PORT" --http "${BFEMU_HTTP:-8088}" --log logs/bfemu.log
