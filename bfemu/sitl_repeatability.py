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
from .sitl_timing import KernelQueueProbe, timing_supported


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='новый файл JSONL; существующий не перезаписывается')
    parser.add_argument('--aircraft', type=Path, help='сохранённый профиль; по умолчанию теоретический Build 03')
    parser.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='roll')
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), required=True)
    parser.add_argument('--periods-ms', nargs='+', type=int, default=[4, 2, 8], help='порядок периодов: 2, 4 или 8 мс')
    parser.add_argument('--modes', nargs='+', choices=('poll', 'event'), default=['poll'],
                        help='режимы чтения моторов в каждом периоде: poll и/или event')
    parser.add_argument('--kernel-queue-timing', action='store_true',
                        help='измерять возраст UDP при чтении с метками ядра Linux')
    parser.add_argument('--pid-denominators', nargs='+', type=int, help='экспериментальные делители PID 1–16')
    parser.add_argument('--alternate-pid-order', action='store_true', help='менять порядок делителей в чётных циклах')
    parser.add_argument('--include-samples', action='store_true',
                        help='сохранять отсчёты датчиков и команд моторов в успешных запусках JSONL')
    parser.add_argument('--repeats', type=int, default=3, help='число циклов с каждым периодом, от 1 до 10')
    args = parser.parse_args(argv)
    if (any(value not in range(251) for value in args.pid) or not 1 <= args.repeats <= 10 or
            len(set(args.periods_ms)) != len(args.periods_ms) or
            not all(period in (2, 4, 8) for period in args.periods_ms) or len(set(args.modes)) != len(args.modes) or
            (args.pid_denominators is not None and (len(set(args.pid_denominators)) != len(args.pid_denominators) or
                                                    any(not 1 <= d <= 16 for d in args.pid_denominators))) or
            (args.alternate_pid_order and args.pid_denominators is None)):
        parser.error('P/I/D Max — 0–250; периоды, режимы и делители 1–16 не должны повторяться; циклы — 1–10')
    try:
        if args.kernel_queue_timing and not timing_supported():
            raise PhysicsError('Метки UDP ядра поддерживаются только на 64-битном Linux x86_64 или aarch64.')
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
                  'periods_ms': args.periods_ms, 'modes': args.modes, 'repeats': args.repeats,
                  'kernel_queue_timing': args.kernel_queue_timing, 'include_samples': args.include_samples,
                  'pid_denominators': args.pid_denominators, 'alternate_pid_order': args.alternate_pid_order,
                  'note': 'Порядок периодов повторяется в каждом цикле; неуспешные запуски не повторяются автоматически. '
                          'Остальные PID-настройки и фильтры SITL заводские.'})
            for cycle in range(1, args.repeats+1):
                for period in args.periods_ms:
                    denominators = args.pid_denominators or [None]
                    if args.alternate_pid_order and cycle % 2 == 0:
                        denominators = list(reversed(denominators))
                    for mode in args.modes:
                        for denom in denominators:
                            start = time.monotonic()
                            record = {'type': 'run', 'cycle': cycle, 'axis': args.axis,
                                      'sensor_period_ms': period, 'motor_mode': mode}
                            if denom is not None:
                                record['pid_process_denom'] = denom
                            probe = KernelQueueProbe() if args.kernel_queue_timing else None
                            try:
                                extra = {'_pid_process_denom': denom} if denom is not None else {}
                                result = aircraft_sitl.experiment(aircraft, axis=args.axis, pid=pid,
                                                                  sensor_period_ms=period, motor_mode=mode,
                                                                  timing_probe=probe, **extra)
                                if denom is not None and result.get('verified_pid_denom') != denom:
                                    raise PhysicsError('SITL вернул неверный делитель PID.')
                                record.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'])
                                if args.include_samples:
                                    record.update(samples=result['samples'], command_window_s=result['command_window_s'],
                                                  disarm_requested_s=result['disarm_requested_s'])
                            except (PhysicsError, OSError) as exc:
                                failures += 1
                                record.update(ok=False, error=str(exc))
                            if probe is not None:
                                record['kernel_queue'] = probe.summary()
                            record['wall_seconds'] = round(time.monotonic()-start, 3)
                            emit(record)
            emit({'type': 'summary', 'attempts': args.repeats*len(args.periods_ms)*len(args.modes)*len(args.pid_denominators or [None]),
                  'not_completed': failures})
        return 1 if failures else 0
    except (OSError, ValueError) as exc:
        print(f'Исследование SITL: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование SITL прервано; файл может быть неполным.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
