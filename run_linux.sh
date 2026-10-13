#!/bin/sh
# bfemu — запуск на Linux/macOS
cd "$(dirname "$0")" || exit 1
PORT="${1:-5761}"
exec python3 -u -m bfemu --tcp "0.0.0.0:$PORT" --http 8088 --log logs/bfemu.log
