"""Упрощённые земля и воздух для опытов с SITL; не полная модель ArduPilot.

Закон сопротивления и наземное ограничение основаны на ArduPilot SITL
422fa5e64365cd446cafaaa2e2a7f06fee089c5a, SIM_Frame.cpp и SIM_Aircraft.cpp.
Параметры — общий пример, не измерения Build 03. Единицы и оси: FRD/NED.
"""
from dataclasses import dataclass, replace
import math

from .flight_physics import (GRAVITY, Aircraft, PhysicsError, State, _state_add, add,
                             derivative, quat_normalize, rotate, scale, vec)


@dataclass(frozen=True)
class Environment:
    # Округлённая оценка по эталонным параметрам 3-килограммового коптера
    # ArduPilot; для Build 03 не измерена. Моментное сопротивление винтов не учтено.
    area_cd_m2: float = .175
    air_density_kg_m3: float = 1.225
    # SIM_FRM_ROT_RATE = 120 в формуле ArduPilot: omega_dot -= omega * radians(400)/120.
    rotation_reference: float = 120.
    wind_ned_m_s: tuple[float, float, float] = (0., 0., 0.)

    def __post_init__(self):
        values = (self.area_cd_m2, self.air_density_kg_m3, self.rotation_reference)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise PhysicsError('Параметры воздуха должны быть конечными положительными числами.')
        vec(self.wind_ned_m_s, 3, 'Скорость ветра')


DEFAULT_ENVIRONMENT = Environment()
ENVIRONMENT_NAME = 'ground-air-v1'
SOURCE_REVISION = '422fa5e64365cd446cafaaa2e2a7f06fee089c5a'


def inverse_rotate(q, v):
    w, x, y, z = quat_normalize(q)
    return rotate((w, -x, -y, -z), v)


def air_derivative(aircraft, state, thrust, reaction, external_torque, environment):
    base = derivative(aircraft, state, thrust, reaction, external_torque_nm=external_torque)
    air_world = tuple(a-b for a, b in zip(state.velocity_m_s, environment.wind_ned_m_s, strict=True))
    air_body = inverse_rotate(state.orientation, air_world)
    drag_body = tuple(-.5*environment.air_density_kg_m3*environment.area_cd_m2*v*abs(v) for v in air_body)
    acceleration = add(base.acceleration_m_s2, rotate(state.orientation, scale(drag_body, 1/aircraft.mass_kg)))
    angular = add(base.angular_acceleration_rad_s2,
                  scale(state.angular_velocity_rad_s, -math.radians(400)/environment.rotation_reference))
    return replace(base, acceleration_m_s2=acceleration, angular_acceleration_rad_s2=angular)


def ground_air_step(aircraft: Aircraft, state: State, thrust, reaction, dt, *,
                    external_torque_nm=(0., 0., 0.), environment=DEFAULT_ENVIRONMENT):
    """Вернуть состояние, контакт и среднюю удельную силу IMU в теле за шаг.

    Плоская земля z=0 NED, жёсткая опора без отскока и скольжения;
    при контакте roll/pitch и угловая скорость обнуляются, yaw сохраняется.
    Ограничение дискретное, не является моделью удара или опорных ног.
    """
    if not math.isfinite(dt) or dt <= 0:
        raise PhysicsError('Шаг времени должен быть положительным и конечным.')
    k1 = air_derivative(aircraft, state, thrust, reaction, external_torque_nm, environment)
    k2 = air_derivative(aircraft, _state_add(state, k1, dt/2), thrust, reaction, external_torque_nm, environment)
    k3 = air_derivative(aircraft, _state_add(state, k2, dt/2), thrust, reaction, external_torque_nm, environment)
    k4 = air_derivative(aircraft, _state_add(state, k3, dt), thrust, reaction, external_torque_nm, environment)
    def mean(name):
        return tuple((a+2*b+2*c+d)/6 for a, b, c, d in zip(*(getattr(k, name) for k in (k1, k2, k3, k4)), strict=True))
    result = _state_add(state, type(k1)(mean('position_m_s'), mean('acceleration_m_s2'),
                                       mean('orientation_per_s'), mean('angular_acceleration_rad_s2')), dt)
    contact = result.position_m[2] >= 0
    if contact:
        w, x, y, z = quat_normalize(result.orientation)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        result = replace(result, position_m=(result.position_m[0], result.position_m[1], 0.),
                         velocity_m_s=(0., 0., min(0., result.velocity_m_s[2])),
                         orientation=(math.cos(yaw/2), 0., 0., math.sin(yaw/2)),
                         angular_velocity_rad_s=(0., 0., 0.))
    acceleration = tuple((a-b)/dt for a, b in zip(result.velocity_m_s, state.velocity_m_s, strict=True))
    specific = inverse_rotate(result.orientation, add(acceleration, (0., 0., -GRAVITY)))
    return result, contact, specific
