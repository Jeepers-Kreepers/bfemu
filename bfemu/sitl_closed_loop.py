"""Экспериментальная замкнутая связь свободного тела с Betaflight 4.5.4 SITL.

Только localhost:9002 (моторы) и localhost:9003 (состояние/IMU).
Возможна упрощённая среда ground-air-v1; батарея, шум, задержка IMU,
GPS и автоматическое управление RC здесь не моделируются.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import select
from pathlib import Path
import socket
import struct
import sys
import time

from .flight_actuators import Actuators, decode_sitl_motors, output_mapping, read_curves
from .flight_physics import GRAVITY, Aircraft, PhysicsError, State, add, step
from .flight_environment import ENVIRONMENT_NAME, ground_air_step, inverse_rotate
from .sitl_gust import advance_with_gust
from .sitl_feedback import sensor_packet
from .sitl_capture import CaptureError, SitlMspClient


def _validate_loop(duration, period, max_gap):
    if not math.isfinite(duration) or not 0 < duration <= 60:
        raise PhysicsError('Длительность должна быть больше нуля и не больше 60 секунд.')
    if not math.isfinite(period) or not .002 <= period <= .01:
        raise PhysicsError('Период датчиков должен быть от 0,002 до 0,01 секунды.')
    if not math.isfinite(max_gap) or not .05 <= max_gap <= 1:
        raise PhysicsError('Максимальный интервал без пакетов моторов должен быть от 0,05 до 1 секунды.')


def discard_queued_motors(motor_socket):
    """Удалить уже ожидающие в UDP-сокете моторы перед запуском источника FDM.

    UDP не содержит номера кадра и идентификатора процесса: очистка не доказывает,
    что все последующие пакеты относятся именно к текущему SITL.
    """
    motor_socket.setblocking(False)
    count = 0
    for _ in range(4096):
        try:
            motor_socket.recvfrom(65535)
        except BlockingIOError:
            return count
        count += 1
    raise PhysicsError('Очередь моторов переполнена до начала эксперимента.')


def run_loop(profile, model, motor_socket, state_socket, stream, *, duration, period, max_gap, pressure_pa,
             clock=time.monotonic, sleep=time.sleep, environment='free', gust_schedule=None):
    _validate_loop(duration, period, max_gap)
    if environment not in ('free', ENVIRONMENT_NAME) or (gust_schedule is not None and environment != ENVIRONMENT_NAME):
        raise PhysicsError('Неизвестная среда или недопустимое воздействие.')
    aircraft, mapping, curves = Aircraft.from_profile(profile), output_mapping(profile), read_curves(model)
    actuators, state = Actuators(curves), State()
    # Валидируем давление и начальное состояние до отправки и записи.
    first = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=0., pressure_pa=pressure_pa,
                          specific_force_m_s2=(0., 0., -GRAVITY) if environment == ENVIRONMENT_NAME else None)
    header = {'type': 'header', 'schema': 1, 'profile': profile, 'actuator_model': model,
              'time_basis': 'host_monotonic', 'sensor_period_s': period, 'pressure_pa': pressure_pa,
              'feedback_to_sitl': True, 'environment': environment, 'note': ('Плоская земля и приближённое сопротивление воздуха.'
              if environment == ENVIRONMENT_NAME else 'Свободное тело без земли и сопротивления воздуха.')+
              ' Идеальный IMU в центре масс; UDP-отправитель не подтверждён.'}
    stream.write(json.dumps(header, ensure_ascii=False, allow_nan=False)+'\n')
    stream.flush()
    motor_socket.setblocking(False)
    start = clock()
    previous = start
    last_motor = None
    commands = (0., 0., 0., 0.)
    next_tick = start+period
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
        dt = now-previous
        # В updateState SITL обновляет simRate только при 0 < deltaSim < 0.02 s.
        # Не пропускаем зависания и скачки времени как достоверный расчёт.
        if not 0 < dt < .02:
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
        average_thrust, average_reaction = actuators.advance(commands, dt)
        contact, specific, gust_active_s = None, None, 0.
        if gust_schedule is not None:
            state, contact, specific, gust_active_s = advance_with_gust(aircraft, state, average_thrust, average_reaction, previous, now, gust_schedule)
        elif environment == ENVIRONMENT_NAME:
            state, contact, specific = ground_air_step(aircraft, state, average_thrust, average_reaction, dt)
        else:
            state = step(aircraft, state, average_thrust, average_reaction, dt)
        state_socket.sendto(sensor_packet(aircraft, state, actuators.thrust_n, timestamp=elapsed, pressure_pa=pressure_pa,
                                          specific_force_m_s2=specific),
                            ('127.0.0.1', 9003))
        stream.write(json.dumps({'type': 'sample', 'elapsed_s': elapsed, 'dt_s': dt,
                                 'applied_commands': commands, 'next_commands': latest,
                                 'thrust_n': actuators.thrust_n, 'reaction_nm': actuators.reaction_nm,
                                 'state': asdict(state), 'ground_contact': contact, 'gust_active_s': gust_active_s}, ensure_ascii=False, allow_nan=False)+'\n')
        count += 1
        if latest is not None:
            commands = latest
        previous = now
        # При умеренном отставании не отправляем скопившиеся пакеты пачкой.
        next_tick = max(next_tick+period, now+period)
    stream.flush()
    if not count or last_motor is None:
        raise PhysicsError('Недостаточно данных от SITL для замкнутого расчёта.')
    return count


def run_event_loop(profile, model, motor_socket, state_socket, stream, *, duration, period, max_gap, pressure_pa,
                   clock=time.monotonic, wait=select.select, environment='free', gust_schedule=None):
    """Интегрировать модель при чтении команд и отправлять датчики по собственному таймеру.

    Время чтения сокета — не метка формирования команды FC. Пакеты UDP не имеют
    номера кадра, поэтому этот режим не обеспечивает lockstep с Betaflight.
    """
    _validate_loop(duration, period, max_gap)
    if environment not in ('free', ENVIRONMENT_NAME) or (gust_schedule is not None and environment != ENVIRONMENT_NAME):
        raise PhysicsError('Неизвестная среда или недопустимое воздействие.')
    aircraft, mapping, curves = Aircraft.from_profile(profile), output_mapping(profile), read_curves(model)
    actuators, state = Actuators(curves), State()
    first = sensor_packet(aircraft, state, actuators.thrust_n, timestamp=0., pressure_pa=pressure_pa,
                          specific_force_m_s2=(0., 0., -GRAVITY) if environment == ENVIRONMENT_NAME else None)
    header = {'type': 'header', 'schema': 1, 'profile': profile, 'actuator_model': model,
              'time_basis': 'host_monotonic', 'sensor_period_s': period, 'pressure_pa': pressure_pa,
              'feedback_to_sitl': True, 'motor_mode': 'event', 'environment': environment,
              'note': 'Команды применяются в момент чтения UDP на стороне модели; соответствие пакетам FDM неизвестно. '
              + ('Плоская земля, приближённое сопротивление воздуха.' if environment == ENVIRONMENT_NAME else
               'Свободное тело без земли и сопротивления воздуха.')}
    stream.write(json.dumps(header, ensure_ascii=False, allow_nan=False)+'\n')
    stream.flush()
    motor_socket.setblocking(False)
    start = clock()
    last_step = last_sensor = start
    last_sensor_state = state
    contact = environment == ENVIRONMENT_NAME
    gust_active_since_sensor = 0.
    contact_since_sensor = False
    last_motor = None
    next_tick = start + period
    commands = (0., 0., 0., 0.)
    latest = None
    full_output = False
    count = 0
    state_socket.sendto(first, ('127.0.0.1', 9003))

    def advance(now):
        nonlocal state, last_step, contact, contact_since_sensor, gust_active_since_sensor
        dt = now-last_step
        if dt < 0:
            raise PhysicsError('Время расчёта модели пошло назад.')
        if dt > 0:
            thrust, reaction = actuators.advance(commands, dt)
            if gust_schedule is not None:
                state, contact, _, active = advance_with_gust(aircraft, state, thrust, reaction, last_step, now, gust_schedule)
                gust_active_since_sensor += active
            elif environment == ENVIRONMENT_NAME:
                state, contact, _ = ground_air_step(aircraft, state, thrust, reaction, dt)
            else:
                state = step(aircraft, state, thrust, reaction, dt)
            contact_since_sensor = contact_since_sensor or bool(contact)
            last_step = now

    while True:
        now = clock()
        if now-start >= duration:
            break
        if now >= next_tick:
            interval = now-last_sensor
            if not 0 < interval < .02:
                raise PhysicsError('Нарушен темп расчёта: интервал датчиков вышел за предел 0–0,02 с; связь остановлена.')
            if now-(last_motor if last_motor is not None else start) > max_gap:
                raise PhysicsError('Нет пакетов моторов SITL: связь остановлена; проверьте выходы моторов.')
            advance(now)
            specific = None
            if environment == ENVIRONMENT_NAME:
                acceleration = tuple((a-b)/interval for a, b in zip(state.velocity_m_s, last_sensor_state.velocity_m_s, strict=True))
                specific = inverse_rotate(state.orientation, add(acceleration, (0., 0., -GRAVITY)))
            state_socket.sendto(sensor_packet(aircraft, state, actuators.thrust_n, timestamp=now-start,
                                              pressure_pa=pressure_pa, specific_force_m_s2=specific), ('127.0.0.1', 9003))
            stream.write(json.dumps({'type': 'sample', 'elapsed_s': now-start, 'dt_s': interval,
                                     'applied_commands': commands, 'next_commands': latest,
                                     'motor_full_output_since_sensor': full_output,
                                     'thrust_n': actuators.thrust_n, 'reaction_nm': actuators.reaction_nm,
                                     'state': asdict(state), 'ground_contact': contact_since_sensor if gust_schedule else contact,
                                     'gust_active_s': gust_active_since_sensor}, ensure_ascii=False, allow_nan=False)+'\n')
            last_sensor = now
            last_sensor_state = state
            gust_active_since_sensor = 0.
            contact_since_sensor = False
            latest = None
            full_output = any(value >= 1.0 for value in commands)
            count += 1
            next_tick = max(next_tick+period, now+period)
            continue
        readable, _, _ = wait([motor_socket], [], [], max(0., min(next_tick, start+duration)-now))
        if readable:
            for _ in range(4096):
                try:
                    packet, sender = motor_socket.recvfrom(65535)
                except BlockingIOError:
                    break
                if sender[0] != '127.0.0.1':
                    raise PhysicsError('Пакет моторов пришёл не с localhost.')
                received = clock()
                advance(received)
                commands = decode_sitl_motors(packet, mapping)
                latest = commands
                full_output = full_output or any(value >= 1.0 for value in commands)
                last_motor = received
            else:
                raise PhysicsError('Очередь моторов переполнена; связь остановлена.')
    stream.flush()
    if not count or last_motor is None:
        raise PhysicsError('Недостаточно данных от SITL для замкнутого расчёта.')
    return count


def check_ready(port=5761):
    """Проверить идентификатор и инициализацию датчиков, затем освободить MSP-порт."""
    client = SitlMspClient(port=port, timeout=2.)
    try:
        client.identify()
        status = client.request(101)
        if len(status) < 6:
            raise CaptureError('Слишком короткий ответ MSP_STATUS.')
        sensors = struct.unpack_from('<H', status, 4)[0]
        if sensors & 0x21 != 0x21:
            raise CaptureError('SITL ещё не инициализировал гироскоп и акселерометр; дождитесь запуска.')
    finally:
        client.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True, help='экспортированный JSON профиля аппарата')
    parser.add_argument('--actuators', type=Path, required=True, help='JSON кривых и постоянных времени приводов')
    parser.add_argument('--output', type=Path, required=True, help='новый файл результата JSONL')
    parser.add_argument('--pressure-pa', type=float, required=True, help='постоянное давление для виртуального барометра, Па')
    parser.add_argument('--duration', type=float, default=10, help='продолжительность, с (до 60)')
    parser.add_argument('--period', type=float, default=.004, help='целевой период пакетов датчиков, с')
    parser.add_argument('--max-gap', type=float, default=.25, help='остановка при потере пакетов моторов, с')
    parser.add_argument('--exclusive-state-source', action='store_true', required=True,
                        help='подтверждаю, что никакой другой источник не посылает данные на UDP 9003')
    parser.add_argument('--assume-ready', action='store_true', help='не проверять MSP: использовать только если Configurator занимает порт и SITL уже готов')
    parser.add_argument('--normal-motor-mode', action='store_true', required=True,
                        help='подтверждаю, что в SITL выключен 3D-режим моторов')
    args = parser.parse_args(argv)
    try:
        profile = json.loads(args.profile.read_text(encoding='utf-8'))
        model = json.loads(args.actuators.read_text(encoding='utf-8'))
        aircraft = Aircraft.from_profile(profile)
        output_mapping(profile)
        read_curves(model)
        sensor_packet(aircraft, State(), (0., 0., 0., 0.), timestamp=0., pressure_pa=args.pressure_pa)
        if (not math.isfinite(args.duration) or not 0 < args.duration <= 60 or not math.isfinite(args.period)
                or not .002 <= args.period <= .01 or not math.isfinite(args.max_gap) or not .05 <= args.max_gap <= 1):
            raise PhysicsError('Недопустимые длительность, период датчиков или интервал ожидания моторов.')
        if not args.assume_ready:
            try:
                check_ready()
            except (CaptureError, OSError) as exc:
                raise PhysicsError('Проверка готовности SITL не выполнена. Убедитесь, что SITL запущен и MSP 5761 свободен: '+str(exc)) from exc
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as motors, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fdm:
            motors.bind(('127.0.0.1', 9002))
            with args.output.open('x', encoding='utf-8') as stream:
                # После остановки прежнего источника дать SITL пройти его 500-мс таймаут.
                time.sleep(.55)
                discarded = discard_queued_motors(motors)
                if discarded:
                    print(f'Перед запуском отброшено старых пакетов моторов: {discarded}.', flush=True)
                print('Экспериментальная связь SITL: моторы 9002 → физика → идеальный IMU 9003. Земли нет.', flush=True)
                count = run_loop(profile, model, motors, fdm, stream, duration=args.duration, period=args.period,
                                 max_gap=args.max_gap, pressure_pa=args.pressure_pa)
        print(f'Записано {count} шагов: {args.output}')
        return 0
    except (OSError, ValueError, OverflowError) as exc:
        print(f'Связь SITL: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
