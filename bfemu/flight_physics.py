"""Ядро динамики свободного твёрдого тела; регулятора и модели моторов здесь нет.

Система тела FRD (X вперёд, Y вправо, Z вниз), система мира NED. Тяга и
реактивные моменты должны быть заданы извне, а не вычислены из команды ESC.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from collections.abc import Sequence

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # w, x, y, z; вращение тела в мир
Mat3 = tuple[Vec3, Vec3, Vec3]
GRAVITY = 9.80665  # стандартное ускорение свободного падения, м/с²


class PhysicsError(ValueError):
    """Данных недостаточно либо они некорректны для этой модели."""


def vec(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise PhysicsError(f'{name}: нужны конечные числа.') from exc
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise PhysicsError(f'{name}: нужны {length} конечных чисел.')
    return result


def add(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    return tuple(x + y for x, y in zip(a, b, strict=True))


def scale(a: Sequence[float], n: float) -> tuple[float, ...]:
    return tuple(x * n for x in a)


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def matvec(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(x*y for x, y in zip(row, v, strict=True)) for row in m)


def det(m: Mat3) -> float:
    return sum(x*y for x, y in zip(m[0], cross(m[1], m[2]), strict=True))


def inverse(m: Mat3) -> Mat3:
    determinant = det(m)
    if not math.isfinite(determinant) or determinant <= 0:
        raise PhysicsError('Матрица инерции должна быть положительно определённой.')
    cols = (cross(m[1], m[2]), cross(m[2], m[0]), cross(m[0], m[1]))
    result = tuple(tuple(cols[j][i]/determinant for j in range(3)) for i in range(3))
    if not all(math.isfinite(x) for row in result for x in row):
        raise PhysicsError('Матрица инерции не допускает устойчивого обращения.')
    return result


def checked_inertia(values: Sequence[float]) -> Mat3:
    xx, yy, zz, xy, xz, yz = vec(values, 6, 'Инерция')
    inertia: Mat3 = ((xx, xy, xz), (xy, yy, yz), (xz, yz, zz))
    if xx <= 0 or xx*yy - xy*xy <= 0 or det(inertia) <= 0:
        raise PhysicsError('Матрица инерции должна быть положительно определённой.')
    # Матрица вторых моментов Q = tr(I)/2 * E - I должна быть неотрицательной.
    # Это ограничение в том числе обеспечивает неравенства треугольника для главных моментов.
    q: Mat3 = (((yy+zz-xx)/2, -xy, -xz), (-xy, (xx+zz-yy)/2, -yz), (-xz, -yz, (xx+yy-zz)/2))
    tol = max(xx, yy, zz) * 1e-12
    if (any(q[i][i] < -tol for i in range(3))
            or any(q[i][i]*q[j][j]-q[i][j]**2 < -tol**2 for i, j in ((0, 1), (0, 2), (1, 2)))
            or det(q) < -tol**3):
        raise PhysicsError('Матрица инерции не соответствует распределению массы твёрдого тела.')
    inverse(inertia)
    return inertia


def quat_normalize(q: Sequence[float]) -> Quat:
    w, x, y, z = vec(q, 4, 'Ориентация')
    norm = math.hypot(w, x, y, z)
    if not math.isfinite(norm) or norm == 0:
        raise PhysicsError('Кватернион ориентации должен быть ненулевым.')
    return (w/norm, x/norm, y/norm, z/norm)


def rotate(q: Quat, vector: Vec3) -> Vec3:
    w, x, y, z = quat_normalize(q)
    u: Vec3 = (x, y, z)
    uv = cross(u, vector)
    uuv = cross(u, uv)
    return add(vector, add(scale(uv, 2*w), scale(uuv, 2)))


def q_derivative(q: Quat, omega: Vec3) -> Quat:
    w, x, y, z = quat_normalize(q)
    p, r, s = omega
    return (-0.5*(x*p+y*r+z*s), 0.5*(w*p+y*s-z*r),
            0.5*(w*r+z*p-x*s), 0.5*(w*s+x*r-y*p))


@dataclass(frozen=True)
class Aircraft:
    mass_kg: float
    inertia: Mat3
    inverse_inertia: Mat3
    motor_positions_m: tuple[Vec3, Vec3, Vec3, Vec3]  # относительно центра масс

    @classmethod
    def from_profile(cls, profile: dict) -> Aircraft:
        if not isinstance(profile, dict) or profile.get('schema') != 1 or not isinstance(profile.get('values'), dict):
            raise PhysicsError('Ожидается JSON профиля аппарата со схемой 1.')
        values = profile['values']

        def number(name: str) -> float:
            value = values.get(name)
            if not isinstance(value, str):
                raise PhysicsError(f'Поле {name} отсутствует или не является строкой.')
            try:
                result = float(value)
            except (ValueError, OverflowError) as exc:
                raise PhysicsError(f'Для физики необходимо числовое значение {name}; «{value}» не подходит.') from exc
            if not math.isfinite(result):
                raise PhysicsError(f'Поле {name} должно быть конечным числом.')
            return result

        mass = number('mass_g') / 1000
        if mass <= 0:
            raise PhysicsError('Взлётная масса должна быть положительной.')
        cg = tuple(number('cg_' + axis)/1000 for axis in 'xyz')
        inertia = checked_inertia([number('inertia_' + axis) for axis in ('xx', 'yy', 'zz', 'xy', 'xz', 'yz')])
        positions = tuple(tuple(number(f'motor_{i}_{axis}')/1000 - cg[j] for j, axis in enumerate('xyz'))
                          for i in range(1, 5))
        return cls(mass, inertia, inverse(inertia), positions)


@dataclass(frozen=True)
class State:
    position_m: Vec3 = (0., 0., 0.)
    velocity_m_s: Vec3 = (0., 0., 0.)
    orientation: Quat = (1., 0., 0., 0.)
    angular_velocity_rad_s: Vec3 = (0., 0., 0.)


@dataclass(frozen=True)
class Derivative:
    position_m_s: Vec3
    acceleration_m_s2: Vec3
    orientation_per_s: Quat
    angular_acceleration_rad_s2: Vec3


def derivative(aircraft: Aircraft, state: State, thrust_n: Sequence[float], reaction_nm: Sequence[float]) -> Derivative:
    """Силы: тяга каждого ротора по -Z тела, заданный отдельно реактивный момент по +Z тела."""
    thrust = vec(thrust_n, 4, 'Тяга моторов')
    moments = vec(reaction_nm, 4, 'Реактивные моменты моторов')
    if any(t < 0 for t in thrust):
        raise PhysicsError('Отрицательная тяга не поддерживается этой моделью.')
    vel = vec(state.velocity_m_s, 3, 'Скорость')
    vec(state.position_m, 3, 'Положение')
    omega = vec(state.angular_velocity_rad_s, 3, 'Угловая скорость')
    q = quat_normalize(state.orientation)
    body_force: Vec3 = (0., 0., -sum(thrust))
    torque: Vec3 = (0., 0., sum(moments))
    for position, force in zip(aircraft.motor_positions_m, thrust, strict=True):
        torque = add(torque, cross(position, (0., 0., -force)))
    angular_momentum = matvec(aircraft.inertia, omega)
    angular_acceleration = matvec(aircraft.inverse_inertia, add(torque, scale(cross(omega, angular_momentum), -1)))
    acceleration = add(rotate(q, scale(body_force, 1/aircraft.mass_kg)), (0., 0., GRAVITY))
    result = Derivative(vel, acceleration, q_derivative(q, omega), angular_acceleration)
    for value in (result.position_m_s, result.acceleration_m_s2, result.orientation_per_s, result.angular_acceleration_rad_s2):
        if not all(math.isfinite(x) for x in value):
            raise PhysicsError('Расчёт вышел за допустимый числовой диапазон.')
    return result


def _state_add(state: State, d: Derivative, factor: float) -> State:
    return State(vec(add(state.position_m, scale(d.position_m_s, factor)), 3, 'Положение'),
                 vec(add(state.velocity_m_s, scale(d.acceleration_m_s2, factor)), 3, 'Скорость'),
                 quat_normalize(add(state.orientation, scale(d.orientation_per_s, factor))),
                 vec(add(state.angular_velocity_rad_s, scale(d.angular_acceleration_rad_s2, factor)), 3, 'Угловая скорость'))


def step(aircraft: Aircraft, state: State, thrust_n: Sequence[float], reaction_nm: Sequence[float], dt: float) -> State:
    """Один шаг RK4 при постоянных силах за шаг, без земли, сопротивления воздуха и датчиков."""
    if not math.isfinite(dt) or dt <= 0:
        raise PhysicsError('Шаг времени должен быть положительным и конечным.')
    k1 = derivative(aircraft, state, thrust_n, reaction_nm)
    k2 = derivative(aircraft, _state_add(state, k1, dt/2), thrust_n, reaction_nm)
    k3 = derivative(aircraft, _state_add(state, k2, dt/2), thrust_n, reaction_nm)
    k4 = derivative(aircraft, _state_add(state, k3, dt), thrust_n, reaction_nm)
    def mean(name: str) -> tuple[float, ...]:
        return tuple((a+2*b+2*c+d)/6 for a, b, c, d in zip(*(getattr(k, name) for k in (k1, k2, k3, k4)), strict=True))
    return _state_add(state, Derivative(mean('position_m_s'), mean('acceleration_m_s2'),
                                        mean('orientation_per_s'), mean('angular_acceleration_rad_s2')), dt)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Демонстрация ядра динамики: силы задаются явно, без Betaflight и модели ESC.')
    parser.add_argument('--profile', required=True, type=Path, help='Экспортированный JSON профиля аппарата')
    parser.add_argument('--thrust', required=True, nargs=4, type=float, metavar=('M1', 'M2', 'M3', 'M4'), help='Тяга четырёх моторов в Н')
    parser.add_argument('--reaction-torque', required=True, nargs=4, type=float, metavar=('M1', 'M2', 'M3', 'M4'), help='Знаковые реактивные моменты моторов относительно +Z в Н·м')
    parser.add_argument('--dt', required=True, type=float, help='Шаг интегрирования в секундах')
    parser.add_argument('--steps', type=int, default=1, help='Количество шагов')
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.steps <= 100000:
            raise PhysicsError('Число шагов должно быть от 1 до 100000.')
        aircraft = Aircraft.from_profile(json.loads(args.profile.read_text(encoding='utf-8')))
        state = State()
        initial = derivative(aircraft, state, args.thrust, args.reaction_torque)
        for _ in range(args.steps):
            state = step(aircraft, state, args.thrust, args.reaction_torque, args.dt)
        print(json.dumps({'initial_derivative': asdict(initial), 'state': asdict(state), 'time_s': args.dt*args.steps}, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, OverflowError) as exc:
        print(f'Физика: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
