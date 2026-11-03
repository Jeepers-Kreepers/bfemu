"""Формирование пакета датчиков Betaflight 4.5.4 SITL из состояния модели."""
from __future__ import annotations

import math
import struct
from collections.abc import Sequence

from .flight_physics import Aircraft, PhysicsError, State, quat_normalize, vec


# target/SITL/target.h: fdm_packet = 18 doubles в порядке полей C-структуры.
FDM_PACKET = struct.Struct('=18d')


def sensor_packet(aircraft: Aircraft, state: State, thrust_n: Sequence[float], *, timestamp: float, pressure_pa: float,
                  specific_force_m_s2: Sequence[float] | None = None) -> bytes:
    """Идеальный IMU в центре масс; давление задаётся вызывающим кодом как постоянное.

    Оси тела модели заданы в FRD. SITL инвертирует Y и Z из пакета: для
    Quad X Betaflight его гироскоп должен получать (p,-q,-r), чтобы знак
    обратной связи соответствовал микшеру. Поэтому в пакет идут (p,q,r).
    Кватернион передаётся напрямую; SITL отдельно преобразует углы ориентации.
    """
    if not math.isfinite(timestamp) or timestamp < 0:
        raise PhysicsError('Метка времени SITL должна быть неотрицательной и конечной.')
    if not math.isfinite(pressure_pa) or pressure_pa <= 0:
        raise PhysicsError('Давление должно быть положительным и конечным.')
    thrust = vec(thrust_n, 4, 'Тяга моторов')
    if any(value < 0 for value in thrust):
        raise PhysicsError('Отрицательная тяга не поддерживается.')
    p, q, r = vec(state.angular_velocity_rad_s, 3, 'Угловая скорость')
    orientation = quat_normalize(state.orientation)
    velocity = vec(state.velocity_m_s, 3, 'Скорость')
    position = vec(state.position_m, 3, 'Положение')
    # В свободном полёте акселерометр измеряет негравитационную силу на единицу массы;
    # в этой модели она состоит только из тяги вдоль -Z, а не из g или dV/dt в мире.
    accel = (0., 0., -sum(thrust)/aircraft.mass_kg) if specific_force_m_s2 is None else vec(specific_force_m_s2, 3, 'Удельная сила IMU')
    fields = (timestamp, p, q, r, *accel, *orientation, *velocity, *position, pressure_pa)
    if not all(math.isfinite(value) for value in fields):
        raise PhysicsError('Показания датчиков вышли за допустимый числовой диапазон.')
    return FDM_PACKET.pack(*fields)
