"""Односторонний эксперимент: команды UDP SITL → модель приводов → свободное тело.

Обратной связи в SITL нет. UDP 9002 может занимать только один локальный приёмник.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import socket
import sys
import time

from .flight_actuators import Actuators, decode_sitl_motors, output_mapping, read_curves
from .flight_physics import Aircraft, PhysicsError, State, step


def run_experiment(profile, model, sock, stream, *, duration, max_gap, clock=time.monotonic):
    if not math.isfinite(duration) or not 0 < duration <= 60:
        raise PhysicsError('Длительность должна быть больше нуля и не больше 60 секунд.')
    if not math.isfinite(max_gap) or not 0.01 <= max_gap <= 1:
        raise PhysicsError('Максимальный интервал между пакетами должен быть от 0,01 до 1 секунды.')
    aircraft, mapping, curves = Aircraft.from_profile(profile), output_mapping(profile), read_curves(model)
    actuators, state = Actuators(curves), State()
    stream.write(json.dumps({'type': 'header', 'schema': 1, 'profile': profile, 'actuator_model': model,
                             'time_basis': 'host_monotonic_packet_receive', 'feedback_to_sitl': False,
                             'note': 'Свободный полёт без земли, аэродинамики и датчиков. UDP-отправитель не подтверждён; это не замкнутая модель.'},
                            ensure_ascii=False, allow_nan=False) + '\n')
    stream.flush()
    started = clock()
    deadline = started + duration
    last_time, last_command = None, None
    count = 0
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            break
        sock.settimeout(min(max_gap, remaining))
        try:
            packet, sender = sock.recvfrom(65535)
        except TimeoutError:
            if clock() >= deadline:
                break
            raise PhysicsError('Нет пакетов моторов: расчёт остановлен, последнее значение не удерживается.') from None
        if sender[0] != '127.0.0.1':
            raise PhysicsError('Пакет моторов пришёл не с localhost.')
        received = clock()
        if received >= deadline:
            break
        command = decode_sitl_motors(packet, mapping)
        if last_time is not None:
            dt = received-last_time
            if not 0 <= dt <= max_gap:
                raise PhysicsError('Слишком большой интервал между пакетами моторов; расчёт остановлен.')
            if dt > 0:
                avg_thrust, avg_reaction = actuators.advance(last_command, dt)
                state = step(aircraft, state, avg_thrust, avg_reaction, dt)
                stream.write(json.dumps({'type': 'sample', 'received_s': received-started, 'dt_s': dt,
                                         'applied_commands': last_command, 'next_commands': command,
                                         'average_thrust_n': avg_thrust, 'average_reaction_nm': avg_reaction,
                                         'state': asdict(state)}, ensure_ascii=False, allow_nan=False) + '\n')
                count += 1
        last_time, last_command = received, command
    stream.flush()
    if count == 0:
        raise PhysicsError('Недостаточно пакетов моторов для расчёта.')
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True, help='JSON профиля аппарата')
    parser.add_argument('--actuators', type=Path, required=True, help='JSON кривых и постоянных времени приводов')
    parser.add_argument('--output', type=Path, required=True, help='новый файл результата JSONL')
    parser.add_argument('--duration', type=float, default=10, help='время приёма, секунд (до 60)')
    parser.add_argument('--max-gap', type=float, default=.1, help='остановка при отсутствии пакетов, секунд')
    args = parser.parse_args(argv)
    try:
        profile = json.loads(args.profile.read_text(encoding='utf-8'))
        model = json.loads(args.actuators.read_text(encoding='utf-8'))
        # Проверить данные до создания выходного файла и открытия порта.
        Aircraft.from_profile(profile)
        output_mapping(profile)
        read_curves(model)
        if not math.isfinite(args.duration) or not 0 < args.duration <= 60 or not math.isfinite(args.max_gap) or not .01 <= args.max_gap <= 1:
            raise PhysicsError('Недопустимое время работы или максимальный интервал между пакетами.')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', 9002))
            with args.output.open('x', encoding='utf-8') as stream:
                print('Получение команд моторов на localhost:9002. Обратной связи с SITL нет.', flush=True)
                count = run_experiment(profile, model, sock, stream, duration=args.duration, max_gap=args.max_gap)
        print(f'Записано {count} шагов: {args.output}')
        return 0
    except (OSError, ValueError, OverflowError) as exc:
        print(f'Эксперимент: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
