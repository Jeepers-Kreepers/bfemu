"""Исследовательское сравнение фиксированного шага модели со штатным шагом по часам.

Фиксированный шаг не синхронизирует Betaflight с моделью: RC и UDP остаются
асинхронными. Крупные задержки хоста по-прежнему прерывают эксперимент.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_actuators import Actuators, decode_sitl_motors, output_mapping, read_curves
from bfemu.flight_physics import Aircraft, PhysicsError, State, step
from bfemu.sitl_closed_loop import _validate_loop
from bfemu.sitl_feedback import sensor_packet
from tools.sitl_pid_cycle_probe import digest


def run_fixed_loop(profile, model, motor_socket, state_socket, stream, *, duration, period, max_gap, pressure_pa,
                   clock=time.monotonic, sleep=time.sleep):
    """Шаг физики и метка FDM фиксированы; wall-время отдельно, UDP не синхронизирован."""
    _validate_loop(duration, period, max_gap)
    aircraft, mapping, curves = Aircraft.from_profile(profile), output_mapping(profile), read_curves(model)
    actuators, state = Actuators(curves), State()
    first = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=0., pressure_pa=pressure_pa)
    stream.write(json.dumps({'type': 'header', 'schema': 1, 'profile': profile, 'actuator_model': model,
                             'time_basis': 'host_monotonic', 'sensor_period_s': period,
                             'physics_time_basis': 'fixed_step', 'pressure_pa': pressure_pa,
                             'feedback_to_sitl': True, 'note': 'Фиксированный шаг не обеспечивает lockstep.'},
                            ensure_ascii=False, allow_nan=False)+'\n')
    stream.flush()
    motor_socket.setblocking(False)
    start = clock()
    previous, next_tick, last_motor = start, start+period, None
    commands = (0., 0., 0., 0.)
    state_socket.sendto(first, ('127.0.0.1', 9003))
    count = 0
    while True:
        now = clock()
        if now < next_tick:
            sleep(next_tick-now)
            now = clock()
        elapsed = now-start
        if elapsed >= duration:
            break
        wall_dt = now-previous
        if not 0 < wall_dt < .02:
            raise PhysicsError('Нарушен темп расчёта: интервал датчиков вышел за предел 0–0,02 с; связь остановлена.')
        latest = None
        for _ in range(4096):
            try:
                packet, sender = motor_socket.recvfrom(65535)
            except BlockingIOError:
                break
            if sender[0] != '127.0.0.1':
                raise PhysicsError('Пакет моторов пришёл не с localhost.')
            latest = decode_sitl_motors(packet, mapping)
        else:
            raise PhysicsError('Очередь моторов переполнена; связь остановлена.')
        if latest is not None:
            last_motor = now
        if now-(last_motor if last_motor is not None else start) > max_gap:
            raise PhysicsError('Нет пакетов моторов SITL: связь остановлена; проверьте выходы моторов.')
        thrust, reaction = actuators.advance(commands, period)
        state = step(aircraft, state, thrust, reaction, period)
        count += 1
        sim_time = count*period
        state_socket.sendto(sensor_packet(aircraft, state, actuators.thrust_n, timestamp=sim_time, pressure_pa=pressure_pa),
                            ('127.0.0.1', 9003))
        stream.write(json.dumps({'type': 'sample', 'elapsed_s': elapsed, 'dt_s': wall_dt, 'sim_time_s': sim_time,
                                 'applied_commands': commands, 'next_commands': latest,
                                 'thrust_n': actuators.thrust_n, 'reaction_nm': actuators.reaction_nm,
                                 'state': asdict(state)}, ensure_ascii=False, allow_nan=False)+'\n')
        if latest is not None:
            commands = latest
        previous = now
        next_tick = max(next_tick+period, now+period)
    stream.flush()
    if not count or last_motor is None:
        raise PhysicsError('Недостаточно данных от SITL для замкнутого расчёта.')
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='pitch')
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), default=[49, 84, 46])
    parser.add_argument('--period-ms', choices=(2, 4, 8), type=int, default=4)
    parser.add_argument('--repeats', type=int, default=4)
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 10 or any(p not in range(251) for p in args.pid):
        parser.error('PID: 0–250, повторения: 1–10.')
    if sys.platform != 'linux':
        parser.error('SITL работает на Linux.')
    try:
        if os.environ.get('LD_PRELOAD'):
            raise PhysicsError('Отключите внешнее вмешательство LD_PRELOAD.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
        old_loop = aircraft_sitl.run_loop
        failures = 0
        with (output_dir/'summary.jsonl').open('x', encoding='utf-8') as output:
            def emit(row):
                line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                output.write(line+'\n')
                output.flush()
                print(line, flush=True)
            emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                  'revision': aircraft_sitl.REVISION, 'elf_sha256': digest(aircraft_sitl.BINARY),
                  'platform': platform.platform(), 'aircraft': profile, 'axis': args.axis,
                  'pid': pid, 'period_ms': args.period_ms, 'motor_mode': 'poll', 'repeats': args.repeats,
                  'note': 'Без изменения ELF. Фиксированный шаг/FDM не синхронизирует UDP/RC/PID; '
                          'метки и длительность стика остаются по времени хоста.'})
            for cycle in range(1, args.repeats+1):
                for fixed in ([False, True] if cycle % 2 else [True, False]):
                    row = {'type': 'run', 'cycle': cycle, 'fixed': fixed}
                    timing = []
                    def observed_fixed(*a, _timing=timing, **kw):
                        count = run_fixed_loop(*a, **kw)
                        rows = [json.loads(line) for line in a[4].getvalue().splitlines()[1:]]
                        if not rows:
                            raise PhysicsError('Нет отсчётов фиксированного шага.')
                        _timing.append({'last_wall_s': rows[-1]['elapsed_s'], 'last_sim_s': rows[-1]['sim_time_s'],
                                       'sample_count': count})
                        return count
                    aircraft_sitl.run_loop = observed_fixed if fixed else old_loop
                    started = time.monotonic()
                    try:
                        result = aircraft_sitl.experiment(profile, axis=args.axis, pid=pid,
                                                          sensor_period_ms=args.period_ms, motor_mode='poll')
                        row.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'])
                        if fixed:
                            row['clock_comparison'] = timing[0]
                        path = output_dir/f'physics-{cycle}-{int(fixed)}.json'
                        with path.open('x', encoding='utf-8') as f:
                            json.dump({'samples': result['samples'], 'command_window_s': result['command_window_s'],
                                       'disarm_requested_s': result['disarm_requested_s']}, f,
                                      ensure_ascii=False, allow_nan=False)
                        row['physics_file'] = path.name
                    except (PhysicsError, OSError, ValueError, IndexError) as exc:
                        failures += 1
                        row.update(ok=False, error=str(exc))
                    finally:
                        aircraft_sitl.run_loop = old_loop
                    row['wall_seconds'] = round(time.monotonic()-started, 3)
                    emit(row)
            emit({'type': 'summary', 'attempts': args.repeats*2, 'not_completed': failures})
        return int(failures > 0)
    except (OSError, ValueError) as exc:
        print(f'Исследование фиксированного шага: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование прервано; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
