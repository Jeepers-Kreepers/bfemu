"""Опытный обмен FDM → чтение gyro → PID/моторы с подтверждением кадра.

Это НЕ lockstep: часы FC и RC асинхронны; изменён PWM-адаптер SITL, но не PID.
Отдельная временная сборка никогда не подключается к веб-тренажёру.
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
from bfemu.sitl_closed_loop import _validate_loop
from bfemu.sitl_feedback import sensor_packet
from tools.sitl_clock_probe import SITL_HASH
from tools.sitl_gyro_probe import GYRO_HASH
from tools.sitl_pid_cycle_probe import CORE_HASH, digest

SOURCE_FILE = Path(__file__).with_suffix('.inc')
ACK = struct.Struct('=II4f')


def replace_unique(text, old, new):
    if text.count(old) != 1:
        raise PhysicsError('Не найдена единственная точка изменения экспериментальной сборки.')
    return text.replace(old, new)


def patch_sources(source):
    gyro = source/'src/main/drivers/accgyro/accgyro_virtual.c'
    core = source/'src/main/fc/core.c'
    sitl = source/'src/main/target/SITL/sitl.c'
    if any(digest(path) != expected for path, expected in ((gyro, GYRO_HASH), (core, CORE_HASH), (sitl, SITL_HASH))):
        raise PhysicsError('Исходники Betaflight отличаются от проверенной ревизии.')
    g = gyro.read_text()
    g = replace_unique(g, 'static int16_t virtualGyroADC[XYZ_AXIS_COUNT];',
                       'uint32_t bfemu_gyro_consumed_sequence;\nstatic uint32_t bfemu_gyro_sequence;\n'
                       'static int16_t virtualGyroADC[XYZ_AXIS_COUNT];')
    g = replace_unique(g, '    virtualGyroADC[X] = x;', '    bfemu_gyro_sequence++;\n    virtualGyroADC[X] = x;')
    g = replace_unique(g, '    gyro->dataReady = false;\n\n    gyro->gyroADCRaw[X]',
                       '    gyro->dataReady = false;\n'
                       '    __atomic_store_n(&bfemu_gyro_consumed_sequence, bfemu_gyro_sequence, __ATOMIC_RELEASE);\n'
                       '\n    gyro->gyroADCRaw[X]')
    c = core.read_text()
    c = replace_unique(c, 'FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)',
                       'extern void bfemu_step_begin(void);\nextern void bfemu_step_ack(void);\n'
                       'FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)')
    c = replace_unique(c, '    subTaskPidController(currentTimeUs);\n    subTaskMotorUpdate(currentTimeUs);',
                       '    subTaskPidController(currentTimeUs);\n    bfemu_step_begin();\n'
                       '    subTaskMotorUpdate(currentTimeUs);\n    bfemu_step_ack();')
    t = sitl.read_text()
    # Upstream при 500-мс reset не запоминает last_ts. При ожидании ответа
    # второй кадр делит свой интервал на время с загрузки хоста и почти
    # останавливает simRate, поэтому здесь инициализируем опору часов.
    t = replace_unique(t, '        last_timestamp = pkt->timestamp;\n        last_realtime = realtime_now;',
                       '        last_timestamp = pkt->timestamp;\n        last_ts = now_ts;\n'
                       '        last_realtime = realtime_now;')
    t = replace_unique(t, 'static servo_packet_raw pwmRawPkt;',
                       'static servo_packet_raw pwmRawPkt;\n#include "bfemu_step_probe.inc"')
    t = replace_unique(t, '    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);',
                       '    if (udpInit(&bfemu_step_link, "127.0.0.1", 9005, false) != 0) exit(1);\n'
                       '    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);')
    t = replace_unique(t, '    if (pthread_mutex_trylock(&updateLock) != 0) return;\n\n    if (index < MAX_SUPPORTED_MOTORS) {',
                       '    if (index < MAX_SUPPORTED_MOTORS) {\n        bfemu_step_mask |= 1U << index;')
    t = replace_unique(t, '    pthread_mutex_unlock(&updateLock); // can send PWM output now\n}\n\nstatic void pwmWriteMotorInt',
                       '}\n\nstatic void pwmWriteMotorInt')
    t = replace_unique(t, '''    // get one "fdm_packet" can only send one "servo_packet"!!
    if (pthread_mutex_trylock(&updateLock) != 0) return;
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
//    printf("[pwm]%u:%u,%u,%u,%u\\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
    udpSend(&pwmRawLink, &pwmRawPkt, sizeof(servo_packet_raw));''',
                       '    /* Отдельный опыт: ответ отправляется после PID, со счётчиком кадра. */')
    for path, text in ((gyro, g), (core, c), (sitl, t)):
        path.write_text(text)
    shutil.copy2(SOURCE_FILE, sitl.parent/'bfemu_step_probe.inc')
    for path in ('drivers/accgyro/accgyro_virtual.o', 'fc/core.o', 'sitl.o'):
        (source/'obj/main/SITL'/path).unlink(missing_ok=True)


def read_ack(sock, expected, mapping, timeout):
    ready, _, _ = select.select([sock], [], [], timeout)
    if not ready:
        raise PhysicsError('SITL не подтвердил обработку кадра gyro и выходов моторов.')
    data, sender = sock.recvfrom(65535)
    if sender[0] != '127.0.0.1' or len(data) != ACK.size:
        raise PhysicsError('Некорректное подтверждение кадра SITL.')
    seq, mask, *_ = ACK.unpack(data)
    if seq != expected or mask & 15 != 15:
        raise PhysicsError(f'Нарушена последовательность кадров SITL: ожидался {expected}, получен {seq}, маска {mask}.')
    return decode_sitl_motors(data[8:], mapping)


def run_step_loop(profile, model, motor_socket, state_socket, stream, *, ack_socket, duration, period, max_gap,
                  pressure_pa, clock=time.monotonic, sleep=time.sleep, stats=None):
    """Один кадр в полёте; реальное время физики/FC и асинхронный RC сохранены."""
    _validate_loop(duration, period, max_gap)
    aircraft, mapping = Aircraft.from_profile(profile), output_mapping(profile)
    actuators, state = Actuators(read_curves(model)), State()
    header = {'type': 'header', 'schema': 1, 'profile': profile, 'actuator_model': model,
              'time_basis': 'host_monotonic', 'sensor_period_s': period, 'pressure_pa': pressure_pa,
              'feedback_to_sitl': True,
              'note': 'Экспериментальный ответ после чтения gyro и PID/моторов; RC и часы асинхронны.'}
    stream.write(json.dumps(header, ensure_ascii=False, allow_nan=False)+'\n')
    stream.flush()
    first = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=0., pressure_pa=pressure_pa)
    start = clock()
    previous, next_tick = start, start+period
    commands, count = (0., 0., 0., 0.), 0
    if stats is not None:
        stats.update(acknowledged=0, max_ack_wait_ms=0., max_sensor_interval_ms=0.)
    state_socket.sendto(first, ('127.0.0.1', 9003))  # upstream 500-ms bootstrap: только сброс таймера
    while True:
        now = clock()
        if now < next_tick:
            sleep(next_tick-now)
            now = clock()
        elapsed = now-start
        if elapsed >= duration:
            break
        dt = now-previous
        if stats is not None:
            stats['max_sensor_interval_ms'] = max(stats['max_sensor_interval_ms'], dt*1000)
        if not 0 < dt < .02:
            raise PhysicsError(f'Нарушен темп расчёта: интервал {dt*1000:.3f} мс после {count} подтверждённых кадров вышел за предел 0–20 мс; связь остановлена.')
        thrust, reaction = actuators.advance(commands, dt)
        state = step(aircraft, state, thrust, reaction, dt)
        state_socket.sendto(sensor_packet(aircraft, state, actuators.thrust_n, timestamp=elapsed, pressure_pa=pressure_pa),
                            ('127.0.0.1', 9003))
        wait_start = time.monotonic()
        try:
            latest = read_ack(ack_socket, count+1, mapping, max_gap)
        finally:
            if stats is not None:
                stats['max_ack_wait_ms'] = max(stats['max_ack_wait_ms'], (time.monotonic()-wait_start)*1000)
        if stats is not None:
            stats['acknowledged'] = count+1
        stream.write(json.dumps({'type': 'sample', 'elapsed_s': elapsed, 'dt_s': dt,
                                 'applied_commands': commands, 'next_commands': latest,
                                 'thrust_n': actuators.thrust_n, 'reaction_nm': actuators.reaction_nm,
                                 'state': asdict(state)}, ensure_ascii=False, allow_nan=False)+'\n')
        count += 1
        commands, previous = latest, now
        next_tick = max(next_tick+period, now+period)
    stream.flush()
    if not count:
        raise PhysicsError('Недостаточно подтверждённых кадров SITL.')
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 10 or not 1 <= args.jobs <= 32:
        parser.error('Повторения: 1–10, jobs: 1–32.')
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
        with tempfile.TemporaryDirectory(prefix='bfemu-step-') as tmp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack:
            ack.bind(('127.0.0.1', 9005))
            source = Path(tmp)/'source'
            shutil.copytree(aircraft_sitl.CACHE, source)
            patch_sources(source)
            sdk = Path(tmp)/'sdk'
            sdk.mkdir()
            print('Сборка отдельной копии с подтверждением кадров...', flush=True)
            with (outdir/'build.log').open('x') as log:
                subprocess.run(['make', 'TARGET=SITL', f'ARM_SDK_DIR={sdk}', f'-j{args.jobs}'], cwd=source,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
            modified = source/'obj/main/betaflight_SITL.elf'
            if digest(modified) == digest(normal):
                raise PhysicsError('Экспериментальный ELF не изменился.')
            old_loop = aircraft_sitl.run_loop
            failures = 0
            with (outdir/'summary.jsonl').open('x') as output:
                def emit(row):
                    line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                    output.write(line+'\n')
                    output.flush()
                    print(line, flush=True)
                emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'revision': aircraft_sitl.REVISION, 'normal_elf_sha256': digest(normal),
                      'step_elf_sha256': digest(modified), 'core_sha256': CORE_HASH, 'gyro_sha256': GYRO_HASH,
                      'sitl_sha256': SITL_HASH, 'probe_source_sha256': digest(SOURCE_FILE),
                      'driver_source_sha256': digest(Path(__file__)),
                      'platform': platform.platform(), 'aircraft': profile, 'axis': 'pitch',
                      'pid': {'p': 49, 'i': 84, 'd': 46}, 'period_ms': 4, 'pid_denom': 1,
                      'repeats': args.repeats,
                      'note': 'Один FDM в полёте, подтверждение чтения gyro и записи четырёх моторов после PID. '
                              'Изменены PWM-адаптер и начальная опора simRate; часы FC, RC и физика остаются асинхронными.'})
                for cycle in range(1, args.repeats+1):
                    for sync in ([False, True] if cycle % 2 else [True, False]):
                        row = {'type': 'run', 'cycle': cycle, 'step': sync}
                        ack.setblocking(False)
                        while True:
                            try:
                                ack.recvfrom(65535)
                            except BlockingIOError:
                                break
                        stats = {}
                        raw_path = outdir/f'partial-{cycle}.jsonl'
                        def loop(*a, _stats=stats, _raw_path=raw_path, **kw):
                            try:
                                return run_step_loop(*a, ack_socket=ack, stats=_stats, **kw)
                            finally:
                                _raw_path.write_text(a[4].getvalue(), encoding='utf-8')
                        aircraft_sitl.BINARY = modified if sync else normal
                        aircraft_sitl.run_loop = loop if sync else old_loop
                        started = time.monotonic()
                        try:
                            result = aircraft_sitl.experiment(profile, axis='pitch', pid={'p': 49, 'i': 84, 'd': 46},
                                                              sensor_period_ms=4, motor_mode='poll', _pid_process_denom=1)
                            row.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'],
                                       verified_pid_denom=result['verified_pid_denom'])
                            path = outdir/f'physics-{cycle}-{int(sync)}.json'
                            with path.open('x') as f:
                                json.dump({'samples': result['samples'], 'command_window_s': result['command_window_s'],
                                           'disarm_requested_s': result['disarm_requested_s']}, f,
                                          ensure_ascii=False, allow_nan=False)
                            row['physics_file'] = path.name
                        except (PhysicsError, OSError, ValueError) as exc:
                            failures += 1
                            row.update(ok=False, error=str(exc))
                        finally:
                            aircraft_sitl.BINARY, aircraft_sitl.run_loop = normal, old_loop
                        if sync and stats:
                            row['step_diagnostics'] = stats
                        if sync and raw_path.exists():
                            row['raw_physics_file'] = raw_path.name
                        row['wall_seconds'] = round(time.monotonic()-started, 3)
                        emit(row)
                emit({'type': 'summary', 'attempts': args.repeats*2, 'not_completed': failures})
            return int(failures > 0)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'Опыт подтверждения кадров: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Опыт прерван; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
