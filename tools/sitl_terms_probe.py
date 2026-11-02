"""Снять реальные составляющие Betaflight PID в отдельной исследовательской сборке.

Не используется веб-тренажёром. Инструментирование меняет планирование; сравниваются
штатный и инструментированный ELF. Составляющие не доказывают причину отклика.
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
from tools.sitl_pid_cycle_probe import CORE_HASH, digest

SOURCE_FILE = Path(__file__).with_suffix('.inc')
RECORD = struct.Struct('<QII21f')
FIELDS = ('p', 'i', 'd', 'f', 'sum', 'gyro_dps', 'setpoint_dps')
AXES = ('roll', 'pitch', 'yaw')


def instrument_core(path):
    """Вставить наблюдатель после реального PID и перед расчётом моторов."""
    if digest(path) != CORE_HASH:
        raise PhysicsError('core.c отличается от проверенной ревизии Betaflight.')
    text = path.read_text(encoding='utf-8')
    point = 'FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)'
    call = '    subTaskPidController(currentTimeUs);\n    subTaskMotorUpdate(currentTimeUs);'
    if text.count(point) != 1 or text.count(call) != 1:
        raise PhysicsError('Не найдены уникальные точки наблюдения PID.')
    text = text.replace(point, '#include "bfemu_terms_probe.inc"\n\n'+point)
    text = text.replace(call, '    subTaskPidController(currentTimeUs);\n'
                              '    bfemu_terms_probe(currentTimeUs);\n    subTaskMotorUpdate(currentTimeUs);')
    path.write_text(text, encoding='utf-8')


def decode_trace(data, start_monotonic, window, axis):
    """Оставить записи в окне команды; отсутствующие выборки не интерполируются."""
    if axis not in AXES or len(window) != 2 or not (math.isfinite(start_monotonic) and
            all(math.isfinite(t) for t in window) and 0 <= window[0] < window[1]):
        raise ValueError('Некорректная ось или границы окна.')
    if len(data) % RECORD.size:
        raise ValueError('Файл оборван посреди записи PID.')
    begin, end = (int((start_monotonic+t)*1e9) for t in window)
    samples, last_ns = [], 0
    offset = 3 + 7*AXES.index(axis)
    for row in RECORD.iter_unpack(data):
        host_ns, sim_us, armed = row[:3]
        if host_ns <= last_ns or armed not in (0, 1) or not all(math.isfinite(v) for v in row[3:]):
            raise ValueError('В файле PID неверное время, флаг ARM или числовое значение.')
        last_ns = host_ns
        if begin <= host_ns <= end:
            values = dict(zip(FIELDS, row[offset:offset+7], strict=True))
            samples.append({'t': (host_ns/1e9-start_monotonic), 'sim_us': sim_us, 'armed': bool(armed), **values})
    if not samples:
        raise ValueError('Нет отсчётов PID в окне команды.')
    gaps = [b['t']-a['t'] for a, b in itertools.pairwise(samples)]
    coverage = {'samples': len(samples), 'first_s': samples[0]['t'], 'last_s': samples[-1]['t'],
                'max_gap_ms': max(gaps, default=0)*1000, 'armed_samples': sum(s['armed'] for s in samples)}
    return samples, coverage


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--axis', choices=AXES, default='pitch')
    parser.add_argument('--before', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), default=[49, 84, 46])
    parser.add_argument('--after', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), default=[55, 84, 46])
    parser.add_argument('--period-ms', type=int, choices=(2, 4, 8), default=4)
    parser.add_argument('--mode', choices=('poll', 'event'), default='poll')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32 or any(v not in range(251) for v in args.before+args.after):
        parser.error('PID: 0–250, повторения: 1–10, jobs: 1–32.')
    if sys.platform != 'linux' or sys.byteorder != 'little':
        parser.error('Требуется Linux little-endian.')
    try:
        if os.environ.get('LD_PRELOAD') or os.environ.get('BFEMU_TERMS_TRACE'):
            raise PhysicsError('Отключите внешние LD_PRELOAD и BFEMU_TERMS_TRACE.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        if not shutil.which('make') or not shutil.which('gcc'):
            raise PhysicsError('Для исследовательской сборки нужны make и gcc.')
        core = aircraft_sitl.CACHE/'src/main/fc/core.c'
        if digest(core) != CORE_HASH:
            raise PhysicsError('core.c отличается от проверенной ревизии Betaflight.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        configs = {name: dict(zip(('p', 'i', 'd'), values, strict=True))
                   for name, values in (('before', args.before), ('after', args.after))}
        normal = aircraft_sitl.BINARY
        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
        with tempfile.TemporaryDirectory(prefix='bfemu-terms-') as tmp:
            work = Path(tmp)
            source = work/'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            instrument_core(source/'src/main/fc/core.c')
            shutil.copy2(SOURCE_FILE, source/'src/main/fc/bfemu_terms_probe.inc')
            (source/'obj/main/SITL/fc/core.o').unlink(missing_ok=True)
            sdk = work/'sdk'
            sdk.mkdir()
            print('Сборка отдельной исследовательской копии...', flush=True)
            with (output_dir/'build.log').open('x', encoding='utf-8') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
            instrument = source/'obj/main/betaflight_SITL.elf'
            if digest(instrument) == digest(normal):
                raise PhysicsError('Исследовательский ELF не изменился.')
            old_popen = aircraft_sitl.subprocess.Popen
            loop_name = 'run_loop' if args.mode == 'poll' else 'run_event_loop'
            old_loop = getattr(aircraft_sitl, loop_name)
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
                      'axis': args.axis, 'pid': configs, 'period_ms': args.period_ms, 'mode': args.mode,
                      'repeats': args.repeats,
                      'note': 'Съём каждые 16 вызовов PID до микширования. Запись может влиять на планирование; '
                              'она не является трассой штатного ELF.'})
                for cycle in range(1, args.repeats+1):
                    conditions = list(configs) if cycle % 2 else list(reversed(configs))
                    for condition in conditions:
                        order = [False, True] if (cycle+list(configs).index(condition)) % 2 else [True, False]
                        for traced in order:
                            trace = output_dir/f'trace-{cycle}-{condition}.bin'
                            starts = []
                            def launch(*a, _trace=trace, **kw):
                                env = os.environ.copy()
                                env['BFEMU_TERMS_TRACE'] = str(_trace)
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
                            aircraft_sitl.BINARY = instrument if traced else normal
                            aircraft_sitl.subprocess.Popen = launch if traced else old_popen
                            setattr(aircraft_sitl, loop_name, measured_loop)
                            row = {'type': 'run', 'cycle': cycle, 'condition': condition,
                                   'pid': configs[condition], 'instrumented': traced,
                                   'file': trace.name if traced else None}
                            started = time.monotonic()
                            try:
                                result = aircraft_sitl.experiment(profile, axis=args.axis, pid=configs[condition],
                                                                  sensor_period_ms=args.period_ms, motor_mode=args.mode)
                                row.update(verified_pid=result['verified_pid'], start_monotonic=starts[0],
                                           command_window_s=result['command_window_s'], diagnostics=result['diagnostics'])
                                physics_path = output_dir/f'physics-{cycle}-{condition}-{int(traced)}.json'
                                with physics_path.open('x', encoding='utf-8') as physics:
                                    json.dump({'samples': result['samples'], 'command_window_s': result['command_window_s'],
                                               'disarm_requested_s': result['disarm_requested_s']}, physics,
                                              ensure_ascii=False, allow_nan=False)
                                row['physics_file'] = physics_path.name
                                if traced:
                                    _, row['coverage'] = decode_trace(trace.read_bytes(), starts[0],
                                                                      result['command_window_s'], args.axis)
                                row['ok'] = True
                            except (PhysicsError, OSError, ValueError, IndexError) as exc:
                                failures += 1
                                row.update(ok=False, error=str(exc))
                            finally:
                                aircraft_sitl.BINARY = normal
                                aircraft_sitl.subprocess.Popen = old_popen
                                setattr(aircraft_sitl, loop_name, old_loop)
                            row['wall_seconds'] = round(time.monotonic()-started, 3)
                            emit(row)
                emit({'type': 'summary', 'attempts': args.repeats*len(configs)*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Измерение PID: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Измерение прервано; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
