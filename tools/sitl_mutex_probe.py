"""Исследовательская трассировка PWM в конкретном Linux x86-64 SITL, не подбор PID.

Только известный по SHA-256 ELF; другие сборки нужно исследовать заново. LD_PRELOAD
влияет на планирование, наблюдает лишь успешно отправленные пакеты, не все PID-циклы.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import struct
import subprocess
import sys
import tempfile

# Скрипт запускается из корня проекта как python tools/sitl_mutex_probe.py.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_physics import PhysicsError

ELF_HASH = 'a14fdcfa67b6a3ac28ffafa73e9ca7ee1fccf4a06a4127bf2538b28bdae6efe1'
RECORD = struct.Struct('<ddII4f4f5fiI')
RECORD_V1 = struct.Struct('<ddII4f4f5fi')


def analyze_trace(data, start, window, *, format_version=2):
    """Оценить отправки в окне команды; маски относятся только к последней записи моторов."""
    record_format = {1: RECORD_V1, 2: RECORD}[format_version]
    if len(data) % record_format.size:
        raise ValueError('Оборванный бинарный файл трассировки.')
    begin, end = (start + t for t in window)
    rows = [r for r in record_format.iter_unpack(data) if begin <= r[0] <= end]
    if not rows:
        raise ValueError('В окне команды нет записанных отправок моторов.')
    if any(not all(map(math.isfinite, r[:2]+r[4:17])) for r in rows):
        raise ValueError('Трассировка содержит недопустимые числовые значения.')
    masks = Counter((r[2], r[3]) for r in rows)
    differences = []
    for r in rows:
        packet = r[4:8]
        # PWM выхода FC 1–4 передаётся в UDP элементах 3, 0, 1, 2.
        remapped = (packet[3], packet[0], packet[1], packet[2])
        expected = [int(value-r[17])/1000 for value in r[8:12]]
        differences.append(max(abs(a-b) for a, b in zip(remapped, expected, strict=True)))
    result = {'active_sends': len(rows),
            'write_count_and_success_mask': {f'{count}:{mask:04b}': n for (count, mask), n in sorted(masks.items())},
            'different_from_current_motor_gt_0_002': sum(d > .002 for d in differences),
            'max_abs_difference': max(differences), 'idle_pulse_values': sorted({r[17] for r in rows}),
            'median_abs_roll_d_internal': statistics.median(abs(r[14]) for r in rows),
            'max_abs_roll_d_internal': max(abs(r[14]) for r in rows),
            'median_simrate_snapshot': statistics.median(r[1] for r in rows)}
    if format_version == 2 and len(rows) > 1 and rows[-1][0] > rows[0][0]:
        result['motor_update_attempts_per_second'] = sum(r[18] for r in rows[1:])/(rows[-1][0]-rows[0][0])
        result['median_attempts_per_packet'] = statistics.median(r[18] for r in rows[1:])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True, help='новая папка для JSONL и бинарных данных')
    parser.add_argument('--repeats', type=int, default=4, help='число чередующихся пар, 1–10')
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 10:
        parser.error('Число пар должно быть от 1 до 10.')
    if sys.platform != 'linux' or platform.machine() != 'x86_64':
        parser.error('Эта трассировка предназначена только для Linux x86-64.')
    try:
        if os.environ.get('LD_PRELOAD'):
            raise PhysicsError('Внешний LD_PRELOAD должен быть отключён до исследования.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите SITL через tools/betaflight_sitl.py build.')
        if hashlib.sha256(Path(aircraft_sitl.BINARY).read_bytes()).hexdigest() != ELF_HASH:
            raise PhysicsError('Хеш ELF не совпадает с исследуемой сборкой; адреса трассировки неприменимы.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data'/'aircraft_build03.json').read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory(prefix='bfemu-mutex-probe-') as tmp:
            library = Path(tmp)/'probe.so'
            subprocess.run(['gcc', '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-o', str(library),
                            str(Path(__file__).with_suffix('.c')), '-ldl', '-pthread'], check=True)
            args.output_dir.mkdir(parents=True, exist_ok=False)
            original_popen, original_loop = aircraft_sitl.subprocess.Popen, aircraft_sitl.run_loop
            failures = 0
            with (args.output_dir/'summary.jsonl').open('x', encoding='utf-8') as output:
                def emit(data):
                    line = json.dumps(data, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'schema': 1, 'trace_format': 2, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'firmware': aircraft_sitl.REVISION, 'elf_sha256': ELF_HASH, 'platform': platform.platform(),
                      'axis': 'roll', 'pid': {'p': 45, 'i': 80, 'd': 40}, 'period_ms': 4,
                      'motor_mode': 'poll', 'repeats': args.repeats, 'aircraft': profile,
                      'note': 'Трассировка меняет нагрузку. Записаны отправки моторов, не все PID-циклы.'})
                for cycle in range(1, args.repeats+1):
                    for trace in ([False, True] if cycle % 2 else [True, False]):
                        path = (args.output_dir/f'trace-{cycle}.bin').resolve()
                        starts = []
                        def launch(*a, _path=path, **kw):
                            env = os.environ.copy()
                            env.update(LD_PRELOAD=str(library), BFEMU_MASKTRACE=str(_path))
                            kw['env'] = env
                            return original_popen(*a, **kw)
                        def observed_loop(*a, _starts=starts, **kw):
                            clock = kw['clock']
                            def observed_clock():
                                now = clock()
                                if not _starts:
                                    _starts.append(now)
                                return now
                            kw['clock'] = observed_clock
                            return original_loop(*a, **kw)
                        aircraft_sitl.subprocess.Popen = launch if trace else original_popen
                        aircraft_sitl.run_loop = observed_loop
                        row = {'type': 'run', 'cycle': cycle, 'trace': trace, 'file': path.name if trace else None}
                        try:
                            result = aircraft_sitl.experiment(profile, axis='roll', pid={'p': 45, 'i': 80, 'd': 40},
                                                              sensor_period_ms=4, motor_mode='poll')
                            row.update(verified_pid=result['verified_pid'], diagnostics=result['diagnostics'],
                                       start_monotonic=starts[0], command_window_s=result['command_window_s'])
                            if trace:
                                row['trace_stats'] = analyze_trace(path.read_bytes(), starts[0], result['command_window_s'])
                            row['ok'] = True
                        except (PhysicsError, OSError, ValueError) as exc:
                            failures += 1
                            row.update(ok=False, error=str(exc))
                        finally:
                            aircraft_sitl.subprocess.Popen = original_popen
                            aircraft_sitl.run_loop = original_loop
                        emit(row)
                emit({'type': 'summary', 'attempts': args.repeats*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Исследование синхронизации: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование прервано; файл результатов может быть неполным.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
