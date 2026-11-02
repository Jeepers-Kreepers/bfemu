"""Снять события драйвера виртуального гироскопа в отдельной сборке SITL.

Счётчики показывают операции драйвера, а не трассу PID. Съём меняет тайминг;
штатный и инструментированный ELF запускаются поочерёдно.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import itertools
import math
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
from tools.sitl_pid_cycle_probe import digest

GYRO_HASH = 'bc898cec34b55709da6d0d53881c675de70033a278acddcf760430e1d4383f61'
SOURCE_FILE = Path(__file__).with_suffix('.inc')
RECORD = struct.Struct('<QIIIIhhhH')


def instrument_gyro(path):
    if digest(path) != GYRO_HASH:
        raise PhysicsError('accgyro_virtual.c отличается от проверенной ревизии Betaflight.')
    text = path.read_text(encoding='utf-8')
    edits = (
        ('#ifdef USE_VIRTUAL_GYRO\n', '#ifdef USE_VIRTUAL_GYRO\n#include "bfemu_gyro_probe.inc"\n'),
        ('    virtualGyroADC[X] = x;', '    bfemu_gyro_set(gyro->dataReady, x, y, z);\n    virtualGyroADC[X] = x;'),
        ('    if (gyro->dataReady == false) {\n        gyroDevUnLock(gyro);',
         '    if (gyro->dataReady == false) {\n        bfemu_gyro_read(false);\n        gyroDevUnLock(gyro);'),
        ('    gyro->dataReady = false;\n\n    gyro->gyroADCRaw[X]',
         '    bfemu_gyro_read(true);\n    gyro->dataReady = false;\n\n    gyro->gyroADCRaw[X]'),
    )
    for old, new in edits:
        if text.count(old) != 1:
            raise PhysicsError('Не найдены уникальные точки виртуального гироскопа.')
        text = text.replace(old, new)
    path.write_text(text, encoding='utf-8')


def decode_trace(data, start, window):
    """События входа FDM в драйвер в окне команды; не трасса PID."""
    if (not math.isfinite(start) or len(window) != 2 or
            not all(math.isfinite(x) for x in window) or not 0 <= window[0] < window[1]):
        raise ValueError('Некорректное окно измерения.')
    if not data or len(data) % (32*RECORD.size):
        raise ValueError('Файл гироскопа пуст или оборван посреди блока.')
    selected, last_time, last_seq = [], 0, 0
    for ns, seq, ok, empty, overwritten, x, y, z, reserved in RECORD.iter_unpack(data):
        if ns <= last_time or seq != last_seq+1 or overwritten not in (0, 1) or reserved:
            raise ValueError('Нарушена последовательность событий гироскопа.')
        last_time, last_seq = ns, seq
        if (start+window[0])*1e9 <= ns <= (start+window[1])*1e9:
            selected.append((ns, ok, empty, overwritten, x, y, z))
    if len(selected) < 2:
        raise ValueError('Недостаточно событий в окне команды.')
    gaps = [(b[0]-a[0])/1e6 for a, b in itertools.pairwise(selected)]
    return {'frames': len(selected), 'first_s': selected[0][0]/1e9-start,
            'last_s': selected[-1][0]/1e9-start, 'max_gap_ms': max(gaps),
            'reads_ok': sum(r[1] for r in selected), 'reads_empty': sum(r[2] for r in selected),
            'overwritten_pending': sum(r[3] for r in selected),
            'intervals_without_successful_read': sum(r[1] == 0 for r in selected),
            'intervals_without_read_calls': sum(r[1] == 0 and r[2] == 0 for r in selected),
            'missing_read_without_pending_flag': sum(r[1] == 0 and r[3] == 0 for r in selected),
            'intervals_multiple_reads': sum(r[1] > 1 for r in selected)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='pitch')
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), default=[49, 84, 46])
    parser.add_argument('--period-ms', choices=(2, 4, 8), type=int, default=4)
    parser.add_argument('--mode', choices=('poll', 'event'), default='poll')
    parser.add_argument('--repeats', type=int, default=4)
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32 or any(p not in range(251) for p in args.pid):
        parser.error('PID: 0–250, повторения: 1–10, jobs: 1–32.')
    if sys.platform != 'linux' or sys.byteorder != 'little':
        parser.error('Для съёма нужен Linux little-endian.')
    try:
        if os.environ.get('LD_PRELOAD') or os.environ.get('BFEMU_GYRO_TRACE'):
            raise PhysicsError('Отключите внешнее вмешательство LD_PRELOAD и BFEMU_GYRO_TRACE.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        if not shutil.which('make') or not shutil.which('gcc'):
            raise PhysicsError('Для исследовательской сборки нужны make и gcc.')
        target = aircraft_sitl.CACHE/'src/main/drivers/accgyro/accgyro_virtual.c'
        if digest(target) != GYRO_HASH:
            raise PhysicsError('accgyro_virtual.c отличается от проверенного исходника Betaflight.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        normal = aircraft_sitl.BINARY
        outdir = args.output_dir.resolve()
        outdir.mkdir(parents=True, exist_ok=False)
        with tempfile.TemporaryDirectory(prefix='bfemu-gyro-') as tmp:
            work = Path(tmp)
            source = work/'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            instrument_gyro(source/'src/main/drivers/accgyro/accgyro_virtual.c')
            shutil.copy2(SOURCE_FILE, source/'src/main/drivers/accgyro/bfemu_gyro_probe.inc')
            (source/'obj/main/SITL/drivers/accgyro/accgyro_virtual.o').unlink(missing_ok=True)
            sdk = work/'sdk'
            sdk.mkdir()
            print('Сборка отдельной копии с диагностикой гироскопа...', flush=True)
            with (outdir/'build.log').open('x', encoding='utf-8') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
            modified = source/'obj/main/betaflight_SITL.elf'
            if digest(modified) == digest(normal):
                raise PhysicsError('Экспериментальный ELF не изменился.')
            loop_name = 'run_loop' if args.mode == 'poll' else 'run_event_loop'
            old_popen, old_loop = aircraft_sitl.subprocess.Popen, getattr(aircraft_sitl, loop_name)
            failures = 0
            with (outdir/'summary.jsonl').open('x', encoding='utf-8') as output:
                def emit(row):
                    line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'revision': aircraft_sitl.REVISION, 'normal_elf_sha256': digest(normal),
                      'instrumented_elf_sha256': digest(modified), 'gyro_sha256': GYRO_HASH,
                      'probe_source_sha256': digest(SOURCE_FILE), 'platform': platform.platform(),
                      'aircraft': profile, 'axis': args.axis, 'pid': pid,
                      'period_ms': args.period_ms, 'mode': args.mode, 'repeats': args.repeats,
                      'note': 'Съём и запись меняют тайминг. Драйвер гироскопа — не трасса PID; '
                              'хвост до 31 события не записывается при остановке.'})
                for cycle in range(1, args.repeats+1):
                    for traced in ([False, True] if cycle % 2 else [True, False]):
                        row = {'type': 'run', 'cycle': cycle, 'instrumented': traced}
                        trace = outdir/f'trace-{cycle}.bin'
                        starts = []
                        def launch(*a, _trace=trace, **kw):
                            env = os.environ.copy()
                            env['BFEMU_GYRO_TRACE'] = str(_trace)
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
                        aircraft_sitl.BINARY = modified if traced else normal
                        aircraft_sitl.subprocess.Popen = launch if traced else old_popen
                        setattr(aircraft_sitl, loop_name, measured_loop)
                        started = time.monotonic()
                        try:
                            result = aircraft_sitl.experiment(profile, axis=args.axis, pid=pid,
                                                              sensor_period_ms=args.period_ms, motor_mode=args.mode)
                            row.update(verified_pid=result['verified_pid'], diagnostics=result['diagnostics'],
                                       start_monotonic=starts[0], command_window_s=result['command_window_s'])
                            path = outdir/f'physics-{cycle}-{int(traced)}.json'
                            with path.open('x', encoding='utf-8') as f:
                                json.dump({'samples': result['samples'], 'command_window_s': result['command_window_s'],
                                           'disarm_requested_s': result['disarm_requested_s']}, f,
                                          ensure_ascii=False, allow_nan=False)
                            row['physics_file'] = path.name
                            if traced:
                                row['coverage'] = decode_trace(trace.read_bytes(), starts[0], result['command_window_s'])
                            row['ok'] = True
                        except (PhysicsError, OSError, ValueError, IndexError) as exc:
                            failures += 1
                            row.update(ok=False, error=str(exc))
                        finally:
                            aircraft_sitl.BINARY = normal
                            aircraft_sitl.subprocess.Popen = old_popen
                            setattr(aircraft_sitl, loop_name, old_loop)
                        if traced and trace.exists():
                            row['trace_file'] = trace.name
                        row['wall_seconds'] = round(time.monotonic()-started, 3)
                        emit(row)
                emit({'type': 'summary', 'attempts': args.repeats*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Исследование гироскопа: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование гироскопа прервано; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
