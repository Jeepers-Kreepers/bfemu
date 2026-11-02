"""Сравнить штатные часы SITL с часовым коэффициентом 1 в отдельной сборке.

Это исследование, не lockstep: FDM, RC, планировщик и моторы остаются асинхронными.
При коэффициенте 1 время FC идёт по часам хоста, а не по скорости поступления FDM.
Изменённый ELF не используется в веб-интерфейсе.
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

SITL_HASH = 'd1160e3bedce2f7dfe1798408cc271d9aeca4d11955f22dc49092f5cc9204be5'


def use_host_clock(path):
    if digest(path) != SITL_HASH:
        raise PhysicsError('sitl.c отличается от проверенной сетевой версии Betaflight.')
    text = path.read_text(encoding='utf-8')
    old = '        simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);'
    if text.count(old) != 1:
        raise PhysicsError('Не найдено единственное присваивание simRate.')
    path.write_text(text.replace(old, '        simRate = 1.0; // Исследование bfemu: часы хоста, не lockstep.'), encoding='utf-8')


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
    if sys.platform != 'linux':
        parser.error('Сборка поддерживается только на Linux.')
    try:
        if os.environ.get('LD_PRELOAD'):
            raise PhysicsError('Отключите внешнее вмешательство LD_PRELOAD.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        if not shutil.which('make') or not shutil.which('gcc'):
            raise PhysicsError('Для исследовательской сборки нужны make и gcc.')
        target = aircraft_sitl.CACHE/'src/main/target/SITL/sitl.c'
        if digest(target) != SITL_HASH:
            raise PhysicsError('sitl.c отличается от проверенного исходника Betaflight.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        normal = aircraft_sitl.BINARY
        outdir = args.output_dir.resolve()
        outdir.mkdir(parents=True, exist_ok=False)
        with tempfile.TemporaryDirectory(prefix='bfemu-clock-') as tmp:
            work = Path(tmp)
            source = work/'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            use_host_clock(source/'src/main/target/SITL/sitl.c')
            (source/'obj/main/SITL/sitl.o').unlink(missing_ok=True)
            sdk = work/'sdk'
            sdk.mkdir()
            print('Сборка отдельной копии с коэффициентом часов 1...', flush=True)
            with (outdir/'build.log').open('x', encoding='utf-8') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
            modified = source/'obj/main/betaflight_SITL.elf'
            if digest(modified) == digest(normal):
                raise PhysicsError('Экспериментальный ELF не изменился.')
            failures = 0
            with (outdir/'summary.jsonl').open('x', encoding='utf-8') as output:
                def emit(row):
                    line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'revision': aircraft_sitl.REVISION, 'normal_elf_sha256': digest(normal),
                      'host_clock_elf_sha256': digest(modified), 'sitl_sha256': SITL_HASH,
                      'platform': platform.platform(), 'aircraft': profile, 'axis': args.axis, 'pid': pid,
                      'period_ms': args.period_ms, 'mode': args.mode, 'repeats': args.repeats,
                      'note': 'Фиксированный simRate не синхронизирует FDM, RC, PID и моторы; это не lockstep.'})
                for cycle in range(1, args.repeats+1):
                    for host_clock in ([False, True] if cycle % 2 else [True, False]):
                        row = {'type': 'run', 'cycle': cycle, 'host_clock': host_clock}
                        aircraft_sitl.BINARY = modified if host_clock else normal
                        started = time.monotonic()
                        try:
                            result = aircraft_sitl.experiment(profile, axis=args.axis, pid=pid,
                                                              sensor_period_ms=args.period_ms, motor_mode=args.mode)
                            row.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'])
                            path = outdir/f'physics-{cycle}-{int(host_clock)}.json'
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
                emit({'type': 'summary', 'attempts': args.repeats*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Исследование часов: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование часов прервано; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
