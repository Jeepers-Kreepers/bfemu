#!/bin/sh
# Веб-тренажёр и эксперимент SITL целиком внутри WSL. Windows FC не изменяется.
set -eu
cd "$(dirname "$0")/.."

printf '\n%s\n' 'bfemu + Betaflight SITL (WSL)' \
  'Адрес в браузере Windows: http://127.0.0.1:18788/aircraft#sitl-experiment' \
  'Не закрывайте это окно, пока пользуетесь тренажёром.' \
  'В этом запуске используются отдельные настройки FC и отдельное хранилище браузера.'

if ! command -v python3 >/dev/null 2>&1 || ! python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
  printf '\n%s\n' 'Нужен Python 3.10 или новее внутри Ubuntu (WSL).'
  printf '%s\n' 'Для Ubuntu установите Python: sudo apt update && sudo apt install python3'
  exit 1
fi
if ! command -v gcc >/dev/null 2>&1 || ! command -v make >/dev/null 2>&1; then
  printf '\n%s\n' 'В Ubuntu (WSL) не найдены gcc или make, необходимые для сборки SITL.'
  printf '%s\n' 'В терминале Ubuntu выполните: sudo apt update && sudo apt install build-essential'
  exit 1
fi
if [ ! -f tools/betaflight_sitl.py ] || [ ! -f bfemu/__main__.py ]; then
  printf '\n%s\n' 'Не найдены файлы проекта. Запускайте run_sitl_wsl.bat из полной папки репозитория.'
  exit 1
fi
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/bfemu-wsl"
mkdir -p "$state_dir"
printf '\n%s\n' 'При первом запуске эксперимента браузер покажет состояние загрузки и сборки SITL.'
printf '%s\n\n' 'Текущий Windows FC и его EEPROM не изменяются.'
exec python3 -u -m bfemu --tcp 127.0.0.1:15777 --http 18788 --http-strict --eeprom "$state_dir/eeprom.json"
