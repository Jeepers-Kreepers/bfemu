"""Сравнение повторных экспериментов Betaflight SITL; не выполняет подбор PID."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import sys
import time

from . import aircraft_sitl
from .flight_physics import PhysicsError


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='новый файл JSONL; существующий не перезаписывается')
    parser.add_argument('--aircraft', type=Path, help='сохранённый профиль; по умолчанию теоретический Build 03')
    parser.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='roll')
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), required=True)
    parser.add_argument('--periods-ms', nargs='+', type=int, default=[4, 2, 8], help='порядок периодов: 2, 4 или 8 мс')
    parser.add_argument('--repeats', type=int, default=3, help='число циклов с каждым периодом, от 1 до 10')
    args = parser.parse_args(argv)
    if (any(value not in range(251) for value in args.pid) or not 1 <= args.repeats <= 10 or
            len(set(args.periods_ms)) != len(args.periods_ms) or
            not all(period in (2, 4, 8) for period in args.periods_ms)):
        parser.error('P/I/D Max — целые 0–250, периоды — разные значения 2, 4 или 8 мс, число циклов — 1–10')
    try:
        path = args.aircraft or Path(__file__).parent / 'data' / 'aircraft_build03.json'
        aircraft = json.loads(path.read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(aircraft)  # До создания файла проверить модель.
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала подготовьте Betaflight SITL через тренажёр или tools/betaflight_sitl.py build.')
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        failures = 0
        with args.output.open('x', encoding='utf-8') as output:
            def emit(record):
                line = json.dumps(record, ensure_ascii=False, allow_nan=False)
                output.write(line+'\n')
                output.flush()
                print(line, flush=True)
            emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                  'firmware': 'Betaflight 4.5.4 SITL', 'revision': aircraft_sitl.REVISION,
                  'platform': platform.platform(), 'aircraft': aircraft, 'axis': args.axis, 'pid': pid,
                  'periods_ms': args.periods_ms, 'repeats': args.repeats,
                  'note': 'Порядок периодов повторяется в каждом цикле; неуспешные запуски не повторяются автоматически. '
                          'Остальные PID-настройки и фильтры SITL заводские.'})
            for cycle in range(1, args.repeats+1):
                for period in args.periods_ms:
                    start = time.monotonic()
                    record = {'type': 'run', 'cycle': cycle, 'axis': args.axis, 'sensor_period_ms': period}
                    try:
                        result = aircraft_sitl.experiment(aircraft, axis=args.axis, pid=pid, sensor_period_ms=period)
                        record.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'])
                    except (PhysicsError, OSError) as exc:
                        failures += 1
                        record.update(ok=False, error=str(exc))
                    record['wall_seconds'] = round(time.monotonic()-start, 3)
                    emit(record)
            emit({'type': 'summary', 'attempts': args.repeats*len(args.periods_ms), 'not_completed': failures})
        return 1 if failures else 0
    except (OSError, ValueError) as exc:
        print(f'Исследование SITL: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование SITL прервано; файл может быть неполным.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
