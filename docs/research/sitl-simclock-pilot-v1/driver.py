"""Изолированный прототип моделируемого времени SITL с FDM и RC в одном кадре.

Исследование меняет target, планировщик и часы Betaflight SITL, не расчёт PID.
Инициализация и настройка пока происходят по часам хоста, поэтому полная
детерминированность не предполагается. Не используется веб-тренажёром.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import select
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_actuators import Actuators, decode_sitl_motors, output_mapping, read_curves
from bfemu.flight_physics import Aircraft, PhysicsError, State, step
from bfemu.sitl_feedback import sensor_packet
from tools.sitl_pid_cycle_probe import digest
from tools.sitl_scheduler_probe import MAIN_HASH
from tools.sitl_step_probe import patch_sources as patch_step, replace_unique

SCHEDULER_HASH = '82d9f8a366271f3484ec1d2cb48e271afd739f651fb0cbcd9a1dedaa71f3eee1'
ROOT = Path(__file__).resolve().parent
INPUT_HEADER = struct.Struct('=II')
RC = struct.Struct('=d16H')
OUTPUT = struct.Struct('=IIIIIIQ4f')
FRAME_US = 4000


def patch_sources(source):
    main = source/'src/main/main.c'
    sched = source/'src/main/scheduler/scheduler.c'
    if digest(main) != MAIN_HASH or digest(sched) != SCHEDULER_HASH:
        raise PhysicsError('Планировщик или main.c отличаются от проверенной ревизии.')
    patch_step(source)
    target = source/'src/main/target/SITL/sitl.c'
    inc_dir = target.parent
    t = target.read_text()
    t = replace_unique(t, '#include "bfemu_step_probe.inc"', '#include "bfemu_simclock_probe.inc"')
    t = replace_unique(t, '    if (udpInit(&bfemu_step_link, "127.0.0.1", 9005, false) != 0) exit(1);',
                       '    if (udpInit(&bfemu_sync_input, "127.0.0.1", 9010, true) != 0 ||\n'
                       '        udpInit(&bfemu_sync_output, "127.0.0.1", 9011, false) != 0) exit(1);')
    t = replace_unique(t, '    if (realtime_now > last_realtime + 500*1e3) {',
                       '    if (!bfemuSimclockActive() && realtime_now > last_realtime + 500*1e3) {')
    t = replace_unique(t, 'static void* tcpThread(void* data)',
                       '#include "bfemu_simclock_target.inc"\n\nstatic void* tcpThread(void* data)')
    t = replace_unique(t, 'uint64_t micros64(void)\n{',
                       'uint64_t micros64(void)\n{\n    if (bfemuSimclockActive()) return __atomic_load_n(&bfemu_simulated_us, __ATOMIC_RELAXED);')
    t = replace_unique(t, 'uint64_t millis64(void)\n{',
                       'uint64_t millis64(void)\n{\n    if (bfemuSimclockActive()) return __atomic_load_n(&bfemu_simulated_us, __ATOMIC_RELAXED) / 1000;')
    # Функция объявляется до updateState, реализация находится после RC-обработчиков.
    t = replace_unique(t, '#include "bfemu_simclock_probe.inc"',
                       '#include "bfemu_simclock_probe.inc"\nbool bfemuSimclockActive(void);')
    m = main.read_text()
    m = replace_unique(m, 'void FAST_CODE run(void)\n{',
                       'extern bool bfemuSimclockActive(void);\nextern bool bfemuSimclockInput(void);\n'
                       'extern void bfemuSimclockAdvance(void);\nextern void bfemuSimclockReply(void);\n'
                       'void FAST_CODE run(void)\n{')
    m = replace_unique(m, '''        scheduler();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif''', '''        if (bfemuSimclockInput()) {
            for (int tick = 0; tick < 160; tick++) {
                bfemuSimclockAdvance();
                scheduler();
            }
            bfemuSimclockReply();
        } else if (bfemuSimclockActive()) {
            delayMicroseconds_real(100);
        } else {
            scheduler();
            delayMicroseconds_real(50);
        }''')
    sc = sched.read_text()
    sc = replace_unique(sc, '#include "scheduler.h"',
                        '#include "scheduler.h"\nextern bool bfemuSimclockActive(void);')
    sc = replace_unique(sc, '        if (schedLoopRemainingCycles < schedLoopStartCycles) {',
                        '        if (bfemuSimclockActive() ? schedLoopRemainingCycles <= 0 : '
                        'schedLoopRemainingCycles < schedLoopStartCycles) {')
    for p, text in ((main, m), (target, t), (sched, sc)):
        p.write_text(text)
    for f in ('sitl_simclock_probe.inc', 'sitl_simclock_target.inc'):
        shutil.copy2(ROOT/f, inc_dir/f.replace('sitl_', 'bfemu_', 1))
    for o in ('main.o', 'sitl.o', 'scheduler/scheduler.o'):
        (source/'obj/main/SITL'/o).unlink(missing_ok=True)


def decode_response(data, expected, mapping, previous_clock):
    if len(data) != OUTPUT.size:
        raise PhysicsError('Неверный размер подтверждения модели времени.')
    seq, gyro_seq, mask, pid_cycles, armed, flags, sim_us, *_ = OUTPUT.unpack(data)
    if seq != expected or gyro_seq != seq or mask & 15 != 15 or not pid_cycles or armed not in (0, 1):
        raise PhysicsError(f'Неверный кадр {seq}: gyro={gyro_seq}, маска={mask}, PID={pid_cycles}, ARM={armed}.')
    if previous_clock is not None and sim_us-previous_clock != FRAME_US:
        raise PhysicsError('Моделируемые часы прошивки изменились не на 4000 мкс.')
    motors = decode_sitl_motors(data[-16:], mapping)
    return {'sequence': seq, 'read_sequence': gyro_seq, 'mask': mask, 'pid_cycles': pid_cycles,
            'armed': bool(armed), 'disable_flags': flags, 'sim_us': sim_us, 'motors': motors}


def run_stationary(profile, frames, *, timeout=3.):
    """Проверка последовательности на стоящей модели, без команды на ARM."""
    aircraft = Aircraft.from_profile(profile)
    mapping = output_mapping(profile)
    actuators = Actuators(read_curves(aircraft_sitl.theoretical_curves(profile)))
    state = State()
    records, last_clock, commands = [], None, (0., 0., 0., 0.)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx:
        rx.bind(('127.0.0.1', 9011))
        for seq in range(1, frames+1):
            thrust, reaction = actuators.advance(commands, FRAME_US/1e6)
            state = step(aircraft, state, thrust, reaction, FRAME_US/1e6)
            timestamp = seq*FRAME_US/1e6
            fdm = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=timestamp, pressure_pa=101325)
            rc = RC.pack(timestamp, *([1500, 1500, 1000, 1500, 1000]+[1000]*11))
            tx.sendto(INPUT_HEADER.pack(seq, 0)+fdm+rc, ('127.0.0.1', 9010))
            if not select.select([rx], [], [], timeout)[0]:
                raise PhysicsError(f'Нет подтверждения кадра {seq}; получено {len(records)}.')
            data, sender = rx.recvfrom(65535)
            if sender[0] != '127.0.0.1':
                raise PhysicsError('Ответ получен не с localhost.')
            rec = decode_response(data, seq, mapping, last_clock)
            if rec['armed']:
                raise PhysicsError('Неожиданный ARM при проверке неподвижного аппарата.')
            last_clock, commands = rec['sim_us'], rec['motors']
            records.append({'fdm_time_s': timestamp, 'state': asdict(state), **rec})
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=500)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args(argv)
    if not 2 <= args.frames <= 20000 or not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32:
        parser.error('Кадры: 2–20000, повторы: 1–10, jobs: 1–32.')
    if sys.platform != 'linux' or sys.byteorder != 'little':
        parser.error('Требуется Linux little-endian.')
    try:
        if os.environ.get('LD_PRELOAD'):
            raise PhysicsError('Отключите LD_PRELOAD.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text())
        aircraft_sitl.theoretical_curves(profile)
        normal = aircraft_sitl.BINARY
        outdir = args.output_dir.resolve()
        outdir.mkdir(parents=True, exist_ok=False)
        with tempfile.TemporaryDirectory(prefix='bfemu-simclock-') as tmp:
            work = Path(tmp)
            source = work/'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            patch_sources(source)
            sdk = work/'sdk'
            sdk.mkdir()
            print('Сборка отдельной копии с моделируемыми часами...', flush=True)
            with (outdir/'build.log').open('x') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
            binary = source/'obj/main/betaflight_SITL.elf'
            if digest(binary) == digest(normal):
                raise PhysicsError('Исследовательский ELF не изменился.')
            failures = 0
            with (outdir/'summary.jsonl').open('x') as output:
                def emit(row):
                    line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'revision': aircraft_sitl.REVISION, 'normal_elf_sha256': digest(normal),
                      'simclock_elf_sha256': digest(binary), 'scheduler_sha256': SCHEDULER_HASH,
                      'driver_sha256': digest(Path(__file__)),
                      'includes_sha256': {p.name: digest(p) for p in (ROOT/'sitl_simclock_probe.inc', ROOT/'sitl_simclock_target.inc')},
                      'platform': platform.platform(), 'aircraft': profile, 'frames': args.frames,
                      'repeats': args.repeats, 'pid': {'p': 49, 'i': 84, 'd': 46},
                      'note': 'Проверка неподвижной модели без ARM. Инициализация и MSP асинхронны, планировщик изменён.'})
                for cycle in range(1, args.repeats+1):
                    row = {'type': 'run', 'cycle': cycle}
                    started = time.monotonic()
                    process = None
                    try:
                        aircraft_sitl.BINARY = binary
                        for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9003),
                                           (socket.SOCK_DGRAM, 9004), (socket.SOCK_DGRAM, 9010), (socket.SOCK_DGRAM, 9011)):
                            with socket.socket(socket.AF_INET, kind) as s:
                                s.bind(('127.0.0.1', port))
                        with tempfile.TemporaryDirectory(prefix='bfemu-simclock-eeprom-') as state:
                            def start():
                                return subprocess.Popen([str(binary)], cwd=state, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            process = start()
                            aircraft_sitl._configure_pwm(process, pid_process_denom=1)
                            process = start()
                            client = aircraft_sitl._connect(process)
                            try:
                                client.identify()
                                aircraft_sitl.verify_pid_denom(client, 1)
                                row['verified_pid'] = aircraft_sitl.set_pid_snapshot(client, 'pitch', {'p': 49, 'i': 84, 'd': 46})
                                aircraft_sitl._write(client, 35, bytes([0, 0, 0, 32, 48]))
                            finally:
                                client.close()
                            result = run_stationary(profile, args.frames)
                        row['ok'] = True
                        row['frames'] = len(result)
                        row['pid_cycles_min'] = min(x['pid_cycles'] for x in result)
                        row['pid_cycles_max'] = max(x['pid_cycles'] for x in result)
                        row['flags_first'] = result[0]['disable_flags']
                        row['flags_last'] = result[-1]['disable_flags']
                        path = outdir/f'frames-{cycle}.jsonl'
                        with path.open('x') as f:
                            for sample in result:
                                f.write(json.dumps(sample, ensure_ascii=False, allow_nan=False)+'\n')
                        row['file'] = path.name
                    except (PhysicsError, OSError, ValueError, subprocess.SubprocessError) as exc:
                        failures += 1
                        row.update(ok=False, error=str(exc))
                    finally:
                        aircraft_sitl._stop(process)
                        aircraft_sitl.BINARY = normal
                    row['wall_seconds'] = round(time.monotonic()-started, 3)
                    emit(row)
                emit({'type': 'summary', 'attempts': args.repeats, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Исследование моделируемого времени: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
