"""Изолированный прототип моделируемого времени SITL с FDM и RC в одном кадре.

Исследование меняет target, планировщик и часы Betaflight SITL, не расчёт PID.
Инициализация и настройка пока происходят по часам хоста, поэтому полная
детерминированность не предполагается. Экспериментальный режим тренажёра использует
тот же кадровый протокол и отдельную сборку; CLI сохраняет исследовательские повторы.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
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
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_actuators import Actuators, decode_sitl_motors, output_mapping, read_curves
from bfemu.flight_physics import Aircraft, PhysicsError, State, step
from bfemu.sitl_feedback import sensor_packet
from bfemu.sitl_capture import CaptureError
from tools.sitl_pid_cycle_probe import digest
from tools.sitl_scheduler_probe import MAIN_HASH
from tools.sitl_step_probe import patch_sources as patch_step, replace_unique

SCHEDULER_HASH = '82d9f8a366271f3484ec1d2cb48e271afd739f651fb0cbcd9a1dedaa71f3eee1'
ROOT = Path(__file__).resolve().parent
INPUT_HEADER = struct.Struct('=II')
RC = struct.Struct('=d16H')
OUTPUT = struct.Struct('=IIIIIIQ4fIIIIff')
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
    rc_pitch, setpoint_pitch = tail[8:]
    if seq != expected or gyro_seq != seq or mask & 15 != 15 or not pid_cycles or armed not in (0, 1):
        raise PhysicsError(f'Неверный кадр {seq}: gyro={gyro_seq}, маска={mask}, PID={pid_cycles}, ARM={armed}, флаги={flags:#x}, время={sim_us} мкс, scheduler={enabled}/{period}/{target}/{now}.')
    if previous_clock is not None and sim_us-previous_clock != FRAME_US:
        raise PhysicsError('Моделируемые часы прошивки изменились не на 4000 мкс.')
    motors = decode_sitl_motors(data[32:48], mapping)
    if not math.isfinite(rc_pitch) or not math.isfinite(setpoint_pitch):
        raise PhysicsError('Некорректный снимок RC или задания угловой скорости FC.')
    return {'sequence': seq, 'read_sequence': gyro_seq, 'mask': mask, 'pid_cycles': pid_cycles,
            'armed': bool(armed), 'disable_flags': flags, 'sim_us': sim_us, 'motors': motors,
            'rc_axis': rc_pitch, 'setpoint_axis_dps': setpoint_pitch}


def run_frames(profile, frames, *, scenario='stationary', axis='pitch', timeout=3., on_frame=None,
               pause_every=0, pause_ms=0):
    """Сценарии: свободное падение без ARM либо ступень PITCH после ARM."""
    if scenario not in ('stationary', 'flight'):
        raise PhysicsError('Неизвестный сценарий моделируемого времени.')
    if axis not in aircraft_sitl.AXES:
        raise PhysicsError('Неизвестная ось моделируемого времени.')
    if (type(pause_every) is not int or type(pause_ms) is not int or
            not 0 <= pause_every <= 10000 or not 0 <= pause_ms <= 1000 or bool(pause_every) != bool(pause_ms)):
        raise PhysicsError('Период и длительность паузы должны быть заданы вместе.')
    aircraft = Aircraft.from_profile(profile)
    mapping = output_mapping(profile)
    actuators = Actuators(read_curves(aircraft_sitl.theoretical_curves(profile)))
    state = State()
    records, last_clock, commands = [], None, (0., 0., 0., 0.)
    axis_index, rc_index = aircraft_sitl.AXES[axis]
    arm_request = armed_at = command_start = command_end = disarm_request = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx:
        rx.bind(('127.0.0.1', 9011))
        for seq in range(1, frames+1):
            if pause_every and seq > 1 and (seq-1) % pause_every == 0:
                time.sleep(pause_ms/1000)
            channels = [1500, 1500, 1000, 1500, 1000]+[1000]*11
            if arm_request is not None:
                channels[4] = 1900
            if armed_at is not None and seq > armed_at:
                channels[2] = 1300
                if command_start <= seq < command_end:
                    channels[rc_index] = 1650
                if seq >= disarm_request:
                    channels[2], channels[4] = 1000, 1000
            thrust, reaction = actuators.advance(commands, FRAME_US/1e6)
            state = step(aircraft, state, thrust, reaction, FRAME_US/1e6)
            timestamp = seq*FRAME_US/1e6
            fdm = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=timestamp, pressure_pa=101325)
            rc = RC.pack(timestamp, *channels)
            tx.sendto(INPUT_HEADER.pack(seq, axis_index)+fdm+rc, ('127.0.0.1', 9010))
            if not select.select([rx], [], [], timeout)[0]:
                raise PhysicsError(f'Нет подтверждения кадра {seq}; получено {len(records)}.')
            data, sender = rx.recvfrom(65535)
            if sender[0] != '127.0.0.1':
                raise PhysicsError('Ответ получен не с localhost.')
            rec = decode_response(data, seq, mapping, last_clock)
            if axis == 'pitch':  # Keep the archived CLI analyzer's historical field names.
                rec['rc_pitch'] = rec['rc_axis']
                rec['setpoint_pitch_dps'] = rec['setpoint_axis_dps']
            last_clock = rec['sim_us']
            row = {'fdm_time_s': timestamp, 'channels': channels, 'applied_commands': commands,
                   'state': asdict(state), **rec}
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
                    arm_request = seq+1
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
                return records, {'arm_request': arm_request, 'armed_at': armed_at,
                                 'command_start': command_start, 'command_end': command_end,
                                 'disarm_request': disarm_request, 'disarmed_at': seq}
            elif seq >= disarm_request+250:
                raise PhysicsError('SITL не подтвердил дизарм за 250 кадров.')
    if scenario == 'flight':
        raise PhysicsError(f'Сценарий не завершён за {frames} кадров; флаги {records[-1]["disable_flags"]:#x}.')
    return records, None



_SIMCLOCK_CACHE = aircraft_sitl.CACHE.parent / 'simclock' / aircraft_sitl.REVISION
_SIMCLOCK_BINARY = _SIMCLOCK_CACHE / 'betaflight_SITL.elf'
_SIMCLOCK_MARKER = _SIMCLOCK_CACHE / 'bfemu-simclock.json'
_SIMCLOCK_GUARD = threading.Lock()
_SIMCLOCK_THREAD = None
_SIMCLOCK_ERROR = None


def _simclock_build_inputs():
    return {
        'schema': 1,
        'revision': aircraft_sitl.REVISION,
        'normal_elf_sha256': digest(aircraft_sitl.BINARY),
        'driver_sha256': digest(Path(__file__)),
        'includes_sha256': {
            name: digest(Path(__file__).with_name(name))
            for name in ('sitl_simclock_probe.inc', 'sitl_simclock_target.inc')
        },
        'patch_helpers_sha256': {
            name: digest(Path(__file__).with_name(name))
            for name in ('sitl_scheduler_probe.py', 'sitl_step_probe.py')
        },
        'scheduler_sha256': SCHEDULER_HASH,
    }


def lockstep_build_status():
    """Return status for the optional deterministic SITL target."""
    global _SIMCLOCK_THREAD
    try:
        expected = _simclock_build_inputs()
        marker = json.loads(_SIMCLOCK_MARKER.read_text(encoding='utf-8'))
        if _SIMCLOCK_BINARY.is_file() and marker.get('inputs') == expected and marker.get('elf_sha256') == digest(_SIMCLOCK_BINARY):
            return {'state': 'ready'}
    except (OSError, ValueError):
        pass
    with _SIMCLOCK_GUARD:
        if _SIMCLOCK_THREAD is not None and _SIMCLOCK_THREAD.is_alive():
            return {'state': 'building'}
        if _SIMCLOCK_ERROR:
            return {'state': 'error', 'error': _SIMCLOCK_ERROR}
    return {'state': 'missing'}


def _build_lockstep_binary(jobs=4):
    """Build/cache a separate frame-stepped SITL. Never modify the normal build."""
    if sys.platform != 'linux' or sys.byteorder != 'little':
        raise PhysicsError('Lockstep SITL сейчас поддерживается только на Linux/WSL x86 little-endian.')
    if not aircraft_sitl.sitl_available():
        raise PhysicsError('Сначала нужно подготовить штатный Betaflight SITL.')
    inputs = _simclock_build_inputs()
    _SIMCLOCK_CACHE.mkdir(parents=True, exist_ok=True)
    lock_path = _SIMCLOCK_CACHE.parent / '.simclock-build.lock'
    try:
        import fcntl
    except ImportError as exc:
        raise PhysicsError('Для безопасной сборки lockstep нужен Linux flock.') from exc
    with lock_path.open('a') as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX)
        try:
            marker = json.loads(_SIMCLOCK_MARKER.read_text(encoding='utf-8'))
            if (_SIMCLOCK_BINARY.is_file() and marker.get('inputs') == inputs
                    and marker.get('elf_sha256') == digest(_SIMCLOCK_BINARY)):
                return _SIMCLOCK_BINARY
        except (OSError, ValueError):
            pass
        with tempfile.TemporaryDirectory(prefix='bfemu-simclock-build-') as tmp:
            work = Path(tmp)
            source = work / 'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            patch_sources(source)
            sdk = work / 'sdk'
            sdk.mkdir()
            log_path = _SIMCLOCK_CACHE / 'build.log'
            with log_path.open('w', encoding='utf-8') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{jobs}'],
                               cwd=source, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=600)
            binary = source / 'obj/main/betaflight_SITL.elf'
            if not binary.is_file() or digest(binary) == digest(aircraft_sitl.BINARY):
                raise PhysicsError('Сборка lockstep не отличается от штатного SITL.')
            staged = _SIMCLOCK_CACHE / 'betaflight_SITL.elf.tmp'
            shutil.copy2(binary, staged)
            os.replace(staged, _SIMCLOCK_BINARY)
            record = {'inputs': inputs, 'elf_sha256': digest(_SIMCLOCK_BINARY)}
            tmp_marker = _SIMCLOCK_CACHE / 'bfemu-simclock.json.tmp'
            tmp_marker.write_text(json.dumps(record, sort_keys=True), encoding='utf-8')
            os.replace(tmp_marker, _SIMCLOCK_MARKER)
    return _SIMCLOCK_BINARY


def _lockstep_build_worker():
    global _SIMCLOCK_ERROR
    try:
        result = aircraft_sitl.start_sitl_build()
        if result.get('state') == 'error':
            raise PhysicsError(result.get('error', 'Не удалось подготовить штатный SITL.'))
        deadline = time.monotonic() + 660
        while not aircraft_sitl.sitl_available():
            state = aircraft_sitl.build_status()
            if state.get('state') == 'error':
                raise PhysicsError(state.get('error', 'Не удалось собрать штатный SITL.'))
            if time.monotonic() >= deadline:
                raise PhysicsError('Подготовка штатного SITL заняла слишком много времени.')
            time.sleep(.25)
        _build_lockstep_binary()
    except Exception as exc:
        with _SIMCLOCK_GUARD:
            _SIMCLOCK_ERROR = str(exc)


def start_lockstep_build():
    """Start an explicit background build for the experimental deterministic mode."""
    global _SIMCLOCK_THREAD, _SIMCLOCK_ERROR
    if sys.platform != 'linux' or sys.byteorder != 'little':
        return {'state': 'error', 'error': 'Lockstep SITL сейчас доступен только на Linux/WSL x86 little-endian.'}
    state = lockstep_build_status()
    if state['state'] == 'ready' or state['state'] == 'building':
        return state
    with _SIMCLOCK_GUARD:
        if _SIMCLOCK_THREAD is not None and _SIMCLOCK_THREAD.is_alive():
            return {'state': 'building'}
        _SIMCLOCK_ERROR = None
        _SIMCLOCK_THREAD = threading.Thread(target=_lockstep_build_worker, name='sitl-lockstep-build', daemon=True)
        _SIMCLOCK_THREAD.start()
    return {'state': 'building'}


def _assert_lockstep_ports():
    for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9003),
                       (socket.SOCK_DGRAM, 9004), (socket.SOCK_DGRAM, 9010),
                       (socket.SOCK_DGRAM, 9011)):
        with socket.socket(socket.AF_INET, kind) as probe:
            try:
                probe.bind(('127.0.0.1', port))
            except OSError as exc:
                raise PhysicsError(f'Порт {port} занят. Для lockstep нужны TCP 5761 и UDP 9003, 9004, 9010, 9011.') from exc


def experiment_lockstep(profile, pid, axis):
    """Run one fixed-step 4 ms Betaflight SITL experiment and return raw frame data."""
    if axis not in aircraft_sitl.AXES:
        raise PhysicsError('Неизвестная ось lockstep.')
    if not isinstance(pid, dict) or set(pid) != {'p', 'i', 'd'} or any(
            type(pid[key]) is not int or not 0 <= pid[key] <= 250 for key in ('p', 'i', 'd')):
        raise PhysicsError('P, I и D Max должны быть целыми числами от 0 до 250.')
    if lockstep_build_status()['state'] != 'ready':
        raise PhysicsError('Lockstep SITL ещё не подготовлен. Нажмите подготовку и повторите запуск.')
    aircraft_sitl.theoretical_curves(profile)
    if not aircraft_sitl._LOCK.acquire(blocking=False):
        raise PhysicsError('Другой эксперимент SITL уже выполняется.')
    original_binary = aircraft_sitl.BINARY
    process = None
    try:
        _assert_lockstep_ports()
        aircraft_sitl.BINARY = _SIMCLOCK_BINARY
        with tempfile.TemporaryDirectory(prefix='bfemu-simclock-run-') as state_dir:
            def start():
                return subprocess.Popen([str(_SIMCLOCK_BINARY)], cwd=state_dir,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            process = start()
            aircraft_sitl._configure_pwm(process, pid_process_denom=1)
            process = start()
            client = aircraft_sitl._connect(process)
            try:
                client.identify()
                aircraft_sitl.verify_pid_denom(client, 1)
                verified_pid = aircraft_sitl.set_pid_snapshot(client, axis, pid)
                aircraft_sitl._write(client, 35, bytes([0, 0, 0, 32, 48]))
                deadline = time.monotonic() + 8
                while True:
                    status = client.request(101)
                    if len(status) >= 6 and struct.unpack_from('<H', status, 4)[0] & 0x21 == 0x21:
                        break
                    if time.monotonic() >= deadline:
                        raise PhysicsError('Lockstep SITL не инициализировал датчики.')
                    time.sleep(.05)
            finally:
                client.close()
            records, events = run_frames(profile, 2500, scenario='flight', axis=axis)
        start_t, end_t = events['command_start'] * .004, events['command_end'] * .004
        selected = [row for row in records if events['command_start'] <= row['sequence'] < events['command_end']]
        axis_index = aircraft_sitl.AXES[axis][0]
        peak = max(abs(math.degrees(row['state']['angular_velocity_rad_s'][axis_index])) for row in selected)
        pid_cycles = [row['pid_cycles'] for row in records]
        motor_saturated = sum(any(value <= 0.0001 or value >= .9999 for value in row['motors']) for row in selected)
        points = []
        for row in records:
            points.append({
                't': round(row['fdm_time_s'], 4),
                'rates': [round(math.degrees(v), 2) for v in row['state']['angular_velocity_rad_s']],
                'motors': [round(v, 4) for v in row['motors']],
                'applied_motors': [round(v, 4) for v in row['applied_commands']],
                'rc_us': row['rc_axis'],
                'setpoint_dps': round(row['setpoint_axis_dps'], 3),
                'pid_cycles': row['pid_cycles'],
                'frame': row['sequence'],
            })
        return {
            'mode': 'lockstep', 'firmware': 'Betaflight 4.5.4 SITL',
            'source_revision': aircraft_sitl.REVISION,
            'simclock_elf_sha256': digest(_SIMCLOCK_BINARY),
            'profile': profile['values'].get('profile_name', ''), 'axis': axis,
            'verified_pid': verified_pid, 'sensor_period_ms': 4, 'motor_mode': 'lockstep',
            'pid_process_denom': 1,
            'command_window_s': [round(start_t, 4), round(end_t, 4)],
            'disarm_requested_s': round(events['disarm_request'] * .004, 4),
            'diagnostics': {
                'peak_abs_dps': round(peak, 2), 'samples': len(selected),
                'pid_cycles_min': min(pid_cycles), 'pid_cycles_max': max(pid_cycles),
                'motor_saturated_samples': motor_saturated,
            },
            'samples': points,
            'note': ('Кадровый обмен и время SITL синхронизированы шагом 4 мс; код PID Betaflight сохранён. '
                     'Изменены target/планировщик для виртуального времени. Это повторяемый эксперимент на теоретической '
                     'модели приводов без земли, сопротивления воздуха, шума IMU и батареи; не прогноз полёта реального аппарата.'),
        }
    except (OSError, subprocess.SubprocessError, ValueError, CaptureError) as exc:
        raise PhysicsError(f'Lockstep эксперимент SITL не выполнен: {exc}') from exc
    finally:
        aircraft_sitl.BINARY = original_binary
        try:
            aircraft_sitl._stop(process)
        finally:
            aircraft_sitl._LOCK.release()

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=500)
    parser.add_argument('--scenario', choices=('stationary', 'flight'), default='stationary')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), default=[49, 84, 46])
    parser.add_argument('--axis', choices=tuple(aircraft_sitl.AXES), default='pitch')
    parser.add_argument('--pause-every', type=int, default=0, help='вставлять задержку хоста каждые N кадров')
    parser.add_argument('--pause-ms', type=int, default=0, help='длительность задержки хоста в миллисекундах')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args(argv)
    if (not 2 <= args.frames <= 20000 or not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32 or any(v not in range(251) for v in args.pid) or
            not 0 <= args.pause_every <= 10000 or not 0 <= args.pause_ms <= 1000 or
            bool(args.pause_every) != bool(args.pause_ms)):
        parser.error('Кадры: 2–20000, повторы: 1–10, jobs: 1–32, PID: 0–250.')
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
                      'note': 'Модель свободного тела без земли. Инициализация и MSP асинхронны, планировщик изменён.'})
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
                                row['verified_pid'] = aircraft_sitl.set_pid_snapshot(
                                    client, args.axis, dict(zip(('p', 'i', 'd'), args.pid, strict=True)))
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
                            path = outdir/f'frames-{cycle}.jsonl'
                            row['file'] = path.name
                            with path.open('x') as f:
                                def save_frame(sample):
                                    f.write(json.dumps(sample, ensure_ascii=False, allow_nan=False)+'\n')
                                result, events = run_frames(profile, args.frames, scenario=args.scenario, axis=args.axis, on_frame=save_frame,
                                                            pause_every=args.pause_every, pause_ms=args.pause_ms)
                            row['events'] = events
                        row['ok'] = True
                        row['frames'] = len(result)
                        row['pid_cycles_min'] = min(x['pid_cycles'] for x in result)
                        row['pid_cycles_max'] = max(x['pid_cycles'] for x in result)
                        row['flags_first'] = result[0]['disable_flags']
                        row['flags_last'] = result[-1]['disable_flags']
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
