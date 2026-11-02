"""Измерение входов в задачу PID Betaflight SITL на отдельной инструментированной сборке.

Измеритель добавляет системные вызовы и меняет время выполнения. В штатную сборку
и пользовательскую EEPROM ничего не записывается. Не использовать на реальном FC.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import itertools
import os
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_physics import PhysicsError

# Записи последовательных непересекающихся 100 мс интервалов; монотонные часы Linux.
RECORD = struct.Struct('<QQQQQQII')
CORE_HASH = 'bae269e35860b94863e42609b87a7c04cf633d274f518d2bbe076a5e0eacea4f'
SOURCE_FILE = Path(__file__).with_suffix('.inc')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def instrument_core(path):
    """Добавить только наблюдение перед вызовом штатного subTaskPidController."""
    if digest(path) != CORE_HASH:
        raise PhysicsError('Код core.c отличается от проверенной ревизии; автоматическое внедрение запрещено.')
    text = path.read_text(encoding='utf-8')
    include, call = '#include <stdbool.h>', '    subTaskRcCommand(currentTimeUs);\n    subTaskPidController(currentTimeUs);'
    if text.count(include) != 1 or text.count(call) != 1:
        raise PhysicsError('Найдено неожиданное число точек внедрения измерителя PID.')
    text = text.replace(include, '#include "bfemu_pid_probe.inc"\n\n'+include)
    text = text.replace(call, '    subTaskRcCommand(currentTimeUs);\n    bfemu_pid_probe(currentTimeUs);\n'
                             '    subTaskPidController(currentTimeUs);')
    path.write_text(text, encoding='utf-8')


def analyze_trace(data, start, window):
    """Выбрать полные непрерывные интервалы внутри окна команды стика."""
    if len(data) % RECORD.size:
        raise ValueError('Трассировка PID оборвана посреди записи.')
    begin, end = ((start+t)*1e9 for t in window)
    rows = [r for r in RECORD.iter_unpack(data) if begin <= r[0] and r[1] <= end]
    if not rows:
        raise ValueError('Нет полных измерительных интервалов в окне команды стика.')
    if any(r[1] <= r[0] or r[2] == 0 or r[3] > r[4] or r[6] > r[7] for r in rows):
        raise ValueError('В трассировке PID некорректный интервал или счётчик.')
    if any(a[1] != b[0] for a, b in itertools.pairwise(rows)):
        raise ValueError('Измерительные интервалы PID не непрерывны.')
    entries = sum(r[2] for r in rows)
    elapsed_ns = sum(r[1]-r[0] for r in rows)
    sim_us = sum(r[5] for r in rows)
    # Большой unsigned-интервал может означать не переход через wrap, а скачок часов назад.
    sim_clock_valid = max(r[7] for r in rows) < 1_000_000
    return {'bins': len(rows), 'duration_s': elapsed_ns/1e9, 'pid_task_entries': entries,
            'host_frequency_hz': entries*1e9/elapsed_ns,
            'simulated_frequency_hz': entries*1e6/sim_us if sim_us and sim_clock_valid else None,
            'sim_clock_valid': sim_clock_valid,
            'min_host_interval_us': min(r[3] for r in rows)/1000,
            'max_host_interval_us': max(r[4] for r in rows)/1000,
            'min_sim_interval_us': min(r[6] for r in rows),
            'max_sim_interval_us': max(r[7] for r in rows)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True, help='новый каталог измерений')
    parser.add_argument('--repeats', type=int, default=4, help='число пар для каждого делителя, 1–10')
    parser.add_argument('--denominators', type=int, nargs='+', default=[1, 16], help='делители PID от 1 до 16')
    parser.add_argument('--period-ms', type=int, choices=(2, 4, 8), default=4)
    parser.add_argument('--mode', choices=('poll', 'event'), default='poll')
    parser.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='roll')
    parser.add_argument('--pid', type=int, nargs=3, metavar=('P', 'I', 'D_MAX'), default=[45, 80, 40])
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args(argv)
    if (not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32 or len(args.denominators) > 4 or
            len(set(args.denominators)) != len(args.denominators) or
            any(not 1 <= d <= 16 for d in args.denominators) or any(v not in range(251) for v in args.pid)):
        parser.error('PID: 0–250, делители: разные 1–16 (до четырёх), пары: 1–10, jobs: 1–32.')
    if sys.platform != 'linux' or sys.byteorder != 'little':
        parser.error('Измерительная сборка требует Linux с порядком байтов little-endian.')
    try:
        if os.environ.get('LD_PRELOAD') or os.environ.get('BFEMU_PIDTRACE'):
            raise PhysicsError('Перед опытом отключите внешние LD_PRELOAD и BFEMU_PIDTRACE.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала подготовьте штатный Betaflight SITL.')
        if not shutil.which('make') or not shutil.which('gcc'):
            raise PhysicsError('Для исследовательской сборки нужны make и gcc.')
        core = aircraft_sitl.CACHE/'src/main/fc/core.c'
        if digest(core) != CORE_HASH:
            raise PhysicsError('core.c отличается от проверенного исходника Betaflight 4.5.4.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data'/'aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        normal = aircraft_sitl.BINARY
        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
        with tempfile.TemporaryDirectory(prefix='bfemu-pid-cycle-') as tmp:
            work = Path(tmp)
            source = work/'source'
            # Никогда не менять и не пересобирать общий кэш SITL.
            shutil.copytree(aircraft_sitl.CACHE, source)
            instrument_core(source/'src/main/fc/core.c')
            # Не позволять старому объектному файлу скрыть добавленную точку замера.
            (source/'obj/main/SITL/fc/core.o').unlink(missing_ok=True)
            shutil.copy2(SOURCE_FILE, source/'src/main/fc/bfemu_pid_probe.inc')
            sdk = work/'sdk'
            sdk.mkdir()
            print('Сборка изолированной исследовательской копии SITL...', flush=True)
            with (output_dir/'build.log').open('x', encoding='utf-8') as build_log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=build_log, stderr=subprocess.STDOUT, check=True)
            instrument = source/'obj/main/betaflight_SITL.elf'
            if digest(instrument) == digest(normal):
                raise PhysicsError('Исследовательский ELF не изменился; измеритель не был собран.')
            old_popen = aircraft_sitl.subprocess.Popen
            old_loop = aircraft_sitl.run_loop if args.mode == 'poll' else aircraft_sitl.run_event_loop
            failures = 0
            with (output_dir/'summary.jsonl').open('x', encoding='utf-8') as output:
                def emit(row):
                    line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'revision': aircraft_sitl.REVISION, 'normal_elf_sha256': digest(normal),
                      'instrumented_elf_sha256': digest(instrument), 'core_sha256': CORE_HASH,
                      'probe_source_sha256': digest(SOURCE_FILE), 'platform': platform.platform(), 'aircraft': profile,
                      'axis': args.axis, 'pid': pid, 'period_ms': args.period_ms, 'mode': args.mode,
                      'denominators': args.denominators, 'repeats': args.repeats,
                      'note': 'Измеряется вход в подзадачу PID до её вызова, а не время её выполнения. '
                              'Измерительная сборка может менять планирование.'})
                for cycle in range(1, args.repeats+1):
                    denoms = args.denominators if cycle % 2 else list(reversed(args.denominators))
                    for denom in denoms:
                        index = args.denominators.index(denom)
                        order = [False, True] if (cycle+index) % 2 else [True, False]
                        for trace in order:
                            path = output_dir/f'trace-{cycle}-{denom}.bin'
                            starts = []
                            def launch(*a, _path=path, **kw):
                                env = os.environ.copy()
                                env['BFEMU_PIDTRACE'] = str(_path)
                                kw['env'] = env
                                return old_popen(*a, **kw)
                            def measured_loop(*a, _starts=starts, **kw):
                                clock = kw['clock']
                                def observed_clock():
                                    now = clock()
                                    if not _starts:
                                        _starts.append(now)
                                    return now
                                kw['clock'] = observed_clock
                                return old_loop(*a, **kw)
                            aircraft_sitl.BINARY = instrument if trace else normal
                            aircraft_sitl.subprocess.Popen = launch if trace else old_popen
                            loop_name = 'run_loop' if args.mode == 'poll' else 'run_event_loop'
                            setattr(aircraft_sitl, loop_name, measured_loop)
                            row = {'type': 'run', 'cycle': cycle, 'denom': denom, 'instrumented': trace,
                                   'file': path.name if trace else None}
                            started = time.monotonic()
                            try:
                                result = aircraft_sitl.experiment(profile, axis=args.axis, pid=pid,
                                                                  sensor_period_ms=args.period_ms, motor_mode=args.mode,
                                                                  _pid_process_denom=denom)
                                row.update(verified_pid=result['verified_pid'], verified_denom=result['verified_pid_denom'],
                                           start_monotonic=starts[0], command_window_s=result['command_window_s'],
                                           diagnostics=result['diagnostics'])
                                if trace:
                                    row['measurement'] = analyze_trace(path.read_bytes(), starts[0], result['command_window_s'])
                                row['ok'] = True
                            except (PhysicsError, OSError, ValueError) as exc:
                                failures += 1
                                row.update(ok=False, error=str(exc))
                            finally:
                                aircraft_sitl.BINARY = normal
                                aircraft_sitl.subprocess.Popen = old_popen
                                setattr(aircraft_sitl, loop_name, old_loop)
                            row['wall_seconds'] = round(time.monotonic()-started, 3)
                            emit(row)
                emit({'type': 'summary', 'attempts': args.repeats*len(args.denominators)*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Измерение PID: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Измерение прервано; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
