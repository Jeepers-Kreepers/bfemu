"""Модель приводов с явно заданными кривыми. Она не является кодом Betaflight."""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct

from .flight_physics import PhysicsError


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PhysicsError(f'{name}: требуется конечное число.')
    return float(value)


@dataclass(frozen=True)
class Point:
    command: float
    thrust_n: float
    reaction_nm: float  # знаковый момент вокруг +Z тела


@dataclass(frozen=True)
class MotorCurve:
    motor: int
    time_up_s: float
    time_down_s: float
    points: tuple[Point, ...]

    def target(self, command: float) -> tuple[float, float]:
        command = _number(command, 'Команда мотора')
        if not 0 <= command <= 1:
            raise PhysicsError('Команды вне 0–1 и режим 3D не поддерживаются.')
        for a, b in zip(self.points, self.points[1:], strict=False):
            if command <= b.command:
                ratio = (command-a.command)/(b.command-a.command)
                return (a.thrust_n+(b.thrust_n-a.thrust_n)*ratio,
                        a.reaction_nm+(b.reaction_nm-a.reaction_nm)*ratio)
        raise PhysicsError('Команда находится за пределами таблицы привода.')


def read_curves(config: dict) -> tuple[MotorCurve, MotorCurve, MotorCurve, MotorCurve]:
    if not isinstance(config, dict) or config.get('schema') != 1 or not isinstance(config.get('provenance'), str) or not config['provenance'].strip():
        raise PhysicsError('Модель приводов должна содержать schema=1 и непустой источник данных provenance.')
    data = config.get('motors')
    if not isinstance(data, list) or len(data) != 4:
        raise PhysicsError('Нужны отдельные данные четырёх моторов.')
    result = {}
    for item in data:
        if not isinstance(item, dict):
            raise PhysicsError('Некорректные данные мотора.')
        index = item.get('motor')
        if type(index) is not int or index not in range(1, 5) or index in result:
            raise PhysicsError('Номера моторов должны быть разными числами от 1 до 4.')
        up, down = (_number(item.get(key), key) for key in ('time_constant_up_s', 'time_constant_down_s'))
        if up <= 0 or down <= 0:
            raise PhysicsError('Постоянные времени модели должны быть положительными.')
        raw_points = item.get('curve')
        if not isinstance(raw_points, list) or not 2 <= len(raw_points) <= 1000:
            raise PhysicsError('Кривая мотора должна содержать от 2 до 1000 точек.')
        points = []
        for row in raw_points:
            if not isinstance(row, dict):
                raise PhysicsError('Каждая точка кривой должна быть объектом.')
            point = Point(*(_number(row.get(key), key) for key in ('command', 'thrust_n', 'reaction_nm')))
            if not 0 <= point.command <= 1 or point.thrust_n < 0:
                raise PhysicsError('Команда должна быть в диапазоне 0–1, тяга — неотрицательной.')
            if points and point.command <= points[-1].command:
                raise PhysicsError('Команды в таблице должны строго возрастать.')
            points.append(point)
        if points[0] != Point(0., 0., 0.) or points[-1].command != 1:
            raise PhysicsError('Кривая должна начинаться точкой (0, 0 Н, 0 Н·м) и заканчиваться командой 1.')
        result[index] = MotorCurve(index, up, down, tuple(points))
    return tuple(result[i] for i in range(1, 5))


def output_mapping(profile: dict) -> tuple[int, int, int, int]:
    """Индексы пакета 9002 для моторов профиля по исходному sitl.c Betaflight 4.5.4."""
    if not isinstance(profile, dict) or profile.get('schema') != 1 or not isinstance(profile.get('values'), dict):
        raise PhysicsError('Ожидается профиль аппарата со схемой 1.')
    result = []
    for motor in range(1, 5):
        output = profile['values'].get(f'motor_{motor}_output')
        if output not in ('1', '2', '3', '4'):
            raise PhysicsError(f'Для мотора {motor} необходим выход FC от 1 до 4.')
        result.append((3, 0, 1, 2)[int(output)-1])
    if len(set(result)) != 4:
        raise PhysicsError('Моторы должны быть подключены к разным выходам FC.')
    return tuple(result)


def decode_sitl_motors(packet: bytes, mapping: tuple[int, int, int, int]) -> tuple[float, float, float, float]:
    if len(packet) != 16:
        raise PhysicsError('Ожидался пакет SITL из четырёх float32 (16 байт).')
    values = struct.unpack('=4f', packet)
    commands = tuple(values[i] for i in mapping)
    if any(not math.isfinite(value) or value < 0 or value > 1 for value in commands):
        raise PhysicsError('Некорректная команда SITL или неподдерживаемый режим 3D; расчёт остановлен.')
    return commands


@dataclass
class Actuators:
    curves: tuple[MotorCurve, MotorCurve, MotorCurve, MotorCurve]
    thrust_n: tuple[float, float, float, float] = (0., 0., 0., 0.)
    reaction_nm: tuple[float, float, float, float] = (0., 0., 0., 0.)

    def advance(self, commands, dt):
        """Постоянная команда за dt: вернём средние силы, сохраним значения в конце шага."""
        if not math.isfinite(dt) or dt <= 0:
            raise PhysicsError('Шаг приводов должен быть положительным и конечным.')
        if len(commands) != 4:
            raise PhysicsError('Нужны команды четырёх моторов.')
        targets = [curve.target(command) for curve, command in zip(self.curves, commands, strict=True)]
        thrust_end, reaction_end, thrust_avg, reaction_avg = [], [], [], []
        for curve, old_thrust, old_reaction, (target_thrust, target_reaction) in zip(
                self.curves, self.thrust_n, self.reaction_nm, targets, strict=True):
            tau = curve.time_up_s if target_thrust >= old_thrust else curve.time_down_s
            # expm1 сохраняет точность для dt/tau около нуля.
            exponent = dt/tau
            one_minus_decay = -math.expm1(-exponent)
            average_fraction = one_minus_decay / exponent if exponent else 1.
            thrust_end.append(old_thrust+(target_thrust-old_thrust)*one_minus_decay)
            reaction_end.append(old_reaction+(target_reaction-old_reaction)*one_minus_decay)
            thrust_avg.append(target_thrust+(old_thrust-target_thrust)*average_fraction)
            reaction_avg.append(target_reaction+(old_reaction-target_reaction)*average_fraction)
        if not all(math.isfinite(value) for group in (thrust_end, reaction_end, thrust_avg, reaction_avg) for value in group):
            raise PhysicsError('Расчёт привода вышел за допустимый числовой диапазон.')
        self.thrust_n, self.reaction_nm = tuple(thrust_end), tuple(reaction_end)
        return tuple(thrust_avg), tuple(reaction_avg)
