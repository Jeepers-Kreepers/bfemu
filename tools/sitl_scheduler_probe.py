"""Проверка влияния паузы планировщика SITL при разнесении процессов по ядрам.

Исследовательская копия target, PID не меняется. Занимает одно ядро на 100% и не
обеспечивает lockstep. Запуски с ошибками не повторяются и попадают в журнал.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_physics import PhysicsError
from tools.sitl_pid_cycle_probe import digest

MAIN_HASH = '7a2e83ae3fa8893c85c1baa53f2dfe49c192ee42aa1cdf1a8231acacc23ed577'


def remove_sitl_sleep(path):
    if digest(path) != MAIN_HASH:
        raise PhysicsError('main.c отличается от проверенной ревизии Betaflight.')
    text = path.read_text(encoding='utf-8')
    old = '        delayMicroseconds_real(50); // max rate 20kHz'
    if text.count(old) != 1:
        raise PhysicsError('Не найдена единственная пауза SITL.')
    path.write_text(text.replace(old, '        /* Исследование bfemu: без паузы планировщика SITL. */'), encoding='utf-8')


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
    if sys.platform != 'linux' or not hasattr(os, 'sched_getaffinity'):
        parser.error('Проверка разнесения по ядрам доступна только на Linux.')
    original_affinity = os.sched_getaffinity(0)
    try:
        if len(original_affinity) < 2:
            raise PhysicsError('Для опыта нужны хотя бы два доступных ядра.')
        if os.environ.get('LD_PRELOAD'):
            raise PhysicsError('Отключите внешнее вмешательство LD_PRELOAD.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        taskset = shutil.which('taskset')
        if not taskset or not shutil.which('make') or not shutil.which('gcc'):
            raise PhysicsError('Для опыта нужны taskset, make и gcc.')
        main_source = aircraft_sitl.CACHE/'src/main/main.c'
        if digest(main_source) != MAIN_HASH:
            raise PhysicsError('main.c отличается от проверенного исходника Betaflight.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        normal = aircraft_sitl.BINARY
        cpu_physics, cpu_fc = sorted(original_affinity)[:2]
        outdir = args.output_dir.resolve()
        outdir.mkdir(parents=True, exist_ok=False)
        with tempfile.TemporaryDirectory(prefix='bfemu-scheduler-') as tmp:
            work = Path(tmp)
            source = work/'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            remove_sitl_sleep(source/'src/main/main.c')
            (source/'obj/main/SITL/main.o').unlink(missing_ok=True)
            sdk = work/'sdk'
            sdk.mkdir()
            print('Сборка отдельной копии без паузы планировщика SITL...', flush=True)
            with (outdir/'build.log').open('x', encoding='utf-8') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
            modified = source/'obj/main/betaflight_SITL.elf'
            if digest(modified) == digest(normal):
                raise PhysicsError('Экспериментальный ELF не изменился.')
            old_popen = aircraft_sitl.subprocess.Popen
            def pinned_launch(cmd, *a, **kw):
                return old_popen([taskset, '-c', str(cpu_fc), *cmd], *a, **kw)
            failures = 0
            with (outdir/'summary.jsonl').open('x', encoding='utf-8') as output:
                def emit(row):
                    line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'revision': aircraft_sitl.REVISION, 'normal_elf_sha256': digest(normal),
                      'busy_elf_sha256': digest(modified), 'main_sha256': MAIN_HASH,
                      'platform': platform.platform(), 'cpu_physics': cpu_physics, 'cpu_sitl': cpu_fc,
                      'affinity_available': sorted(original_affinity), 'aircraft': profile, 'axis': args.axis,
                      'pid': pid, 'period_ms': args.period_ms, 'mode': args.mode, 'repeats': args.repeats,
                      'note': 'Физика закреплена за одним ядром, все потоки SITL за другим. '
                              'Без паузы SITL занимает ядро; равномерный PID или lockstep не доказаны.'})
                os.sched_setaffinity(0, {cpu_physics})
                aircraft_sitl.subprocess.Popen = pinned_launch
                try:
                    for cycle in range(1, args.repeats+1):
                        for busy in ([False, True] if cycle % 2 else [True, False]):
                            row = {'type': 'run', 'cycle': cycle, 'busy': busy}
                            aircraft_sitl.BINARY = modified if busy else normal
                            started = time.monotonic()
                            try:
                                result = aircraft_sitl.experiment(profile, axis=args.axis, pid=pid,
                                                                  sensor_period_ms=args.period_ms, motor_mode=args.mode)
                                row.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'])
                                path = outdir/f'physics-{cycle}-{int(busy)}.json'
                                with path.open('x', encoding='utf-8') as f:
                                    json.dump({'samples': result['samples'], 'command_window_s': result['command_window_s'],
                                               'disarm_requested_s': result['disarm_requested_s']}, f,
                                              ensure_ascii=False, allow_nan=False)
                                row['physics_file'] = path.name
                            except (PhysicsError, OSError, ValueError) as exc:
                                failures += 1
                                row.update(ok=False, error=str(exc))
                            finally:
                                aircraft_sitl.BINARY = normal
                            row['wall_seconds'] = round(time.monotonic()-started, 3)
                            emit(row)
                finally:
                    aircraft_sitl.subprocess.Popen = old_popen
                    os.sched_setaffinity(0, original_affinity)
                emit({'type': 'summary', 'attempts': args.repeats*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Исследование планировщика: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование прервано; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
