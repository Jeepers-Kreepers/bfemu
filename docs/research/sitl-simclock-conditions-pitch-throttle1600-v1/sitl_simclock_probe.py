"""Изолированный прототип моделируемого времени SITL с FDM и RC в одном кадре.

Исследование меняет target, планировщик и часы Betaflight SITL, не расчёт PID.
Инициализация и настройка пока происходят по часам хоста, поэтому полная
детерминированность не предполагается. Не используется веб-тренажёром.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import math
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
OUTPUT = struct.Struct('=IIIIIIQ4fIIII24fIII4f')
FRAME_US = 4000
DEFAULT_PID = {'roll': (45, 80, 40), 'pitch': (49, 84, 46), 'yaw': (45, 80, 0)}


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
    sc = replace_unique(sc, 'static uint32_t lastTargetCycles;',
                        'static uint32_t lastTargetCycles;\n'
                        'void bfemuSchedRebase(void) { lastTargetCycles = getCycleCounter(); }\n'
                        'uint32_t bfemuSchedEnabled(void) { return gyroEnabled; }\n'
                        'uint32_t bfemuSchedPeriod(void) { return desiredPeriodCycles; }\n'
                        'uint32_t bfemuSchedTarget(void) { return lastTargetCycles; }\n'
                        'uint32_t bfemuSchedNow(void) { return getCycleCounter(); }')
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
    seq, gyro_seq, mask, pid_cycles, armed, flags, sim_us, *tail = OUTPUT.unpack(data)
    enabled, period, target, now = tail[4:8]
    rc_axes, setpoints, gyros, pid = tail[8:11], tail[11:14], tail[14:17], tail[17:32]
    pid_us, gyro_us, gyro_sample_us, pid_dt, *ki = tail[32:]
    if seq != expected or gyro_seq != seq or mask & 15 != 15 or not pid_cycles or armed not in (0, 1):
        raise PhysicsError(f'Неверный кадр {seq}: gyro={gyro_seq}, маска={mask}, PID={pid_cycles}, ARM={armed}, флаги={flags:#x}, время={sim_us} мкс, scheduler={enabled}/{period}/{target}/{now}.')
    if previous_clock is not None and sim_us-previous_clock != FRAME_US:
        raise PhysicsError('Моделируемые часы прошивки изменились не на 4000 мкс.')
    motors = decode_sitl_motors(data[32:48], mapping)
    if not all(math.isfinite(value) for value in (*rc_axes, *setpoints, *gyros, *pid, pid_dt, *ki)):
        raise PhysicsError('Некорректный снимок RC, задания, гироскопа или PID FC.')
    return {'sequence': seq, 'read_sequence': gyro_seq, 'mask': mask, 'pid_cycles': pid_cycles,
            'armed': bool(armed), 'disable_flags': flags, 'sim_us': sim_us, 'motors': motors,
            'rc_axes': rc_axes, 'setpoint_axes_dps': setpoints, 'gyro_axes_dps': gyros,
            'pid_axes': [dict(zip(('p', 'i', 'd', 'f', 'sum'), pid[i:i+5], strict=True))
                         for i in range(0, 15, 5)],
            'pid_loop_us': pid_us, 'gyro_loop_us': gyro_us, 'gyro_sample_us': gyro_sample_us,
            'pid_dt_s': pid_dt, 'pid_ki': ki}


def run_frames(profile, frames, *, scenario='stationary', timeout=3., on_frame=None,
               pause_every=0, pause_ms=0, axis='pitch', stick=1650, throttle=1300,
               impulse_dps=(0., 0., 0.), prearm_frames=0):
    """Сценарии: свободное падение без ARM либо ступень выбранной оси после ARM."""
    if axis not in aircraft_sitl.AXES:
        raise PhysicsError('Неизвестная ось.')
    if scenario not in ('stationary', 'flight'):
        raise PhysicsError('Неизвестный сценарий моделируемого времени.')
    if (type(pause_every) is not int or type(pause_ms) is not int or
            not 0 <= pause_every <= 10000 or not 0 <= pause_ms <= 1000 or bool(pause_every) != bool(pause_ms)):
        raise PhysicsError('Период и длительность паузы должны быть заданы вместе.')
    if (type(stick) is not int or not 1000 <= stick <= 2000 or type(throttle) is not int or
            not 1000 <= throttle <= 2000 or type(prearm_frames) is not int or not 0 <= prearm_frames <= 2000):
        raise PhysicsError('Некорректная команда стика, газ или ожидание перед ARM.')
    if (not isinstance(impulse_dps, (list, tuple)) or len(impulse_dps) != 3 or
            any(not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 720 for v in impulse_dps)):
        raise PhysicsError('Импульс должен содержать три конечные скорости до 720 °/с.')
    aircraft = Aircraft.from_profile(profile)
    mapping = output_mapping(profile)
    actuators = Actuators(read_curves(aircraft_sitl.theoretical_curves(profile)))
    state = State()
    records, last_clock, commands = [], None, (0., 0., 0., 0.)
    arm_request = armed_at = command_start = command_end = disarm_request = ready_at = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx:
        rx.bind(('127.0.0.1', 9011))
        for seq in range(1, frames+1):
            if pause_every and seq > 1 and (seq-1) % pause_every == 0:
                time.sleep(pause_ms/1000)
            channels = [1500, 1500, 1000, 1500, 1000]+[1000]*11
            if arm_request is not None:
                channels[4] = 1900
            if armed_at is not None and seq > armed_at:
                channels[2] = throttle
                if command_start <= seq < command_end:
                    channels[aircraft_sitl.AXES[axis][1]] = stick
                if seq >= disarm_request:
                    channels[2], channels[4] = 1000, 1000
            if seq == command_start and any(impulse_dps):
                state = replace(state, angular_velocity_rad_s=tuple(
                    v+math.radians(delta) for v, delta in zip(state.angular_velocity_rad_s, impulse_dps, strict=True)))
            thrust, reaction = actuators.advance(commands, FRAME_US/1e6)
            state = step(aircraft, state, thrust, reaction, FRAME_US/1e6)
            timestamp = seq*FRAME_US/1e6
            fdm = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=timestamp, pressure_pa=101325)
            rc = RC.pack(timestamp, *channels)
            tx.sendto(INPUT_HEADER.pack(seq, 0)+fdm+rc, ('127.0.0.1', 9010))
            if not select.select([rx], [], [], timeout)[0]:
                raise PhysicsError(f'Нет подтверждения кадра {seq}; получено {len(records)}.')
            data, sender = rx.recvfrom(65535)
            if sender[0] != '127.0.0.1':
                raise PhysicsError('Ответ получен не с localhost.')
            rec = decode_response(data, seq, mapping, last_clock)
            last_clock = rec['sim_us']
            row = {'fdm_time_s': timestamp, 'channels': channels, 'applied_commands': commands,
                   'state': asdict(state), **rec}
            if seq == command_start and any(impulse_dps):
                row['impulse_dps'] = list(impulse_dps)
            records.append(row)
            if on_frame:
                on_frame(row)
            commands = rec['motors']
            if scenario == 'stationary':
                if rec['armed']:
                    raise PhysicsError('Неожиданный ARM при проверке без ARM.')
                continue
            if arm_request is None:
                if rec['armed']:
                    raise PhysicsError('ARM произошёл до команды.')
                if rec['disable_flags'] == 0:
                    if ready_at is None:
                        ready_at = seq
                    if seq-ready_at >= prearm_frames:
                        arm_request = seq+1
                else:
                    ready_at = None
            elif armed_at is None:
                if rec['armed']:
                    armed_at = seq
                    command_start = seq+151  # ровно 150 кадров газа до команды
                    command_end = command_start+300
                    disarm_request = command_end+125
                elif seq-arm_request >= 500:
                    raise PhysicsError(f'SITL не подтвердил ARM; флаги {rec["disable_flags"]:#x}.')
            elif seq < disarm_request and not rec['armed']:
                raise PhysicsError(f'SITL вышел из ARM на кадре {seq}; флаги {rec["disable_flags"]:#x}.')
            elif seq >= disarm_request and not rec['armed']:
                return records, {'ready_at': ready_at, 'arm_request': arm_request, 'armed_at': armed_at,
                                 'command_start': command_start, 'command_end': command_end,
                                 'disarm_request': disarm_request, 'disarmed_at': seq}
            elif seq >= disarm_request+250:
                raise PhysicsError('SITL не подтвердил дизарм за 250 кадров.')
    if scenario == 'flight':
        raise PhysicsError(f'Сценарий не завершён за {frames} кадров; флаги {records[-1]["disable_flags"]:#x}.')
    return records, None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=500)
    parser.add_argument('--scenario', choices=('stationary', 'flight'), default='stationary')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--axis', choices=tuple(aircraft_sitl.AXES), default='pitch')
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'),
                        help='P, I и D Max оси; по умолчанию значения исследовательского сценария')
    parser.add_argument('--stick', type=int, default=1650, help='RC выбранной оси во время ступени, 1000–2000')
    parser.add_argument('--throttle', type=int, default=1300, help='Газ после ARM до дизарма, 1000–2000')
    parser.add_argument('--impulse-dps', nargs=3, type=float, metavar=('ROLL', 'PITCH', 'YAW'), default=[0., 0., 0.],
                        help='Мгновенное приращение физической угловой скорости при начале команды, °/с')
    parser.add_argument('--prearm-frames', type=int, default=0, help='Дополнительные кадры без запрета ARM')
    parser.add_argument('--pause-every', type=int, default=0, help='вставлять задержку хоста каждые N кадров')
    parser.add_argument('--pause-ms', type=int, default=0, help='длительность задержки хоста в миллисекундах')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args(argv)
    if args.pid is None:
        args.pid = DEFAULT_PID[args.axis]
    if (not 2 <= args.frames <= 20000 or not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32 or any(v not in range(251) for v in args.pid) or
            not 0 <= args.pause_every <= 10000 or not 0 <= args.pause_ms <= 1000 or
            bool(args.pause_every) != bool(args.pause_ms) or not 1000 <= args.stick <= 2000 or
            not 1000 <= args.throttle <= 2000 or not 0 <= args.prearm_frames <= 2000 or
            any(not math.isfinite(v) or abs(v) > 720 for v in args.impulse_dps)):
        parser.error('Недопустимые кадры, повторы, PID, газ, стик, пауза или импульс.')
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
                      'platform': platform.platform(), 'aircraft': profile, 'frames': args.frames, 'scenario': args.scenario,
                      'repeats': args.repeats, 'axis': args.axis, 'pid': dict(zip(('p', 'i', 'd'), args.pid, strict=True)),
                      'pause_every': args.pause_every, 'pause_ms': args.pause_ms,
                      'stick': args.stick, 'throttle': args.throttle, 'impulse_dps': args.impulse_dps,
                      'prearm_frames': args.prearm_frames,
                      'note': 'Модель свободного тела без земли. Инициализация и MSP асинхронны, планировщик изменён.'})
                for cycle in range(1, args.repeats+1):
                    row = {'type': 'run', 'cycle': cycle}
                    started = time.monotonic()
                    process = None
                    stage = 'проверка портов'
                    try:
                        aircraft_sitl.BINARY = binary
                        for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9003),
                                           (socket.SOCK_DGRAM, 9004), (socket.SOCK_DGRAM, 9010), (socket.SOCK_DGRAM, 9011)):
                            with socket.socket(socket.AF_INET, kind) as s:
                                s.bind(('127.0.0.1', port))
                        with tempfile.TemporaryDirectory(prefix='bfemu-simclock-eeprom-') as state:
                            def start():
                                return subprocess.Popen([str(binary)], cwd=state, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                            stage = 'запуск и настройка PWM'
                            process = start()
                            aircraft_sitl._configure_pwm(process, pid_process_denom=1)
                            stage = 'повторный запуск и подключение MSP'
                            process = start()
                            client = aircraft_sitl._connect(process)
                            try:
                                stage = 'идентификация и проверка PID'
                                client.identify()
                                aircraft_sitl.verify_pid_denom(client, 1)
                                row['verified_pid'] = aircraft_sitl.set_pid_snapshot(
                                    client, args.axis, dict(zip(('p', 'i', 'd'), args.pid, strict=True)))
                                stage = 'настройка ARM и ожидание датчиков'
                                aircraft_sitl._write(client, 35, bytes([0, 0, 0, 32, 48]))
                                deadline = time.monotonic()+8
                                while True:
                                    status = client.request(101)
                                    if len(status) >= 6 and struct.unpack_from('<H', status, 4)[0] & 0x21 == 0x21:
                                        break
                                    if time.monotonic() >= deadline:
                                        raise PhysicsError('SITL не инициализировал датчики.')
                                    time.sleep(.1)
                            finally:
                                client.close()
                            stage = 'обмен кадрами и сценарий'
                            path = outdir/f'frames-{cycle}.jsonl'
                            row['file'] = path.name
                            with path.open('x') as f:
                                def save_frame(sample):
                                    f.write(json.dumps(sample, ensure_ascii=False, allow_nan=False)+'\n')
                                result, events = run_frames(profile, args.frames, scenario=args.scenario, on_frame=save_frame,
                                                            pause_every=args.pause_every, pause_ms=args.pause_ms,
                                                            axis=args.axis, stick=args.stick, throttle=args.throttle,
                                                            impulse_dps=args.impulse_dps, prearm_frames=args.prearm_frames)
                            row['events'] = events
                        row['ok'] = True
                        row['frames'] = len(result)
                        row['pid_cycles_min'] = min(x['pid_cycles'] for x in result)
                        row['pid_cycles_max'] = max(x['pid_cycles'] for x in result)
                        row['flags_first'] = result[0]['disable_flags']
                        row['flags_last'] = result[-1]['disable_flags']
                    except (PhysicsError, OSError, ValueError, subprocess.SubprocessError) as exc:
                        failures += 1
                        row.update(ok=False, error=str(exc), error_stage=stage)
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
