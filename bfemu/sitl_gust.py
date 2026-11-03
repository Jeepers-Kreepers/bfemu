"""Внешний момент для штатного (асинхронного) SITL по времени хоста."""
from __future__ import annotations
from itertools import pairwise

from .flight_environment import ground_air_step, inverse_rotate
from .flight_physics import GRAVITY, add
from .sitl_pair_scenario import gust_torque, validate_gust


class GustSchedule:
    """Однократное воздействие; окно публикуется целым неизменяемым кортежем."""
    def __init__(self, axis, moment_nm, duration_ms):
        if type(duration_ms) is not int or duration_ms % 4:
            raise ValueError('Длительность порыва должна задаваться с шагом 4 мс.')
        validate_gust(moment_nm, duration_ms//4)
        self.torque = gust_torque(axis, moment_nm)
        self.duration_s = duration_ms/1000
        self.window = None

    def schedule(self, start):
        if self.window is not None:
            raise ValueError('Порыв уже запланирован.')
        self.window = (start, start+self.duration_s)


def advance_with_gust(aircraft, state, thrust, reaction, start, end, schedule):
    """Разделить физический шаг на точных границах окна порыва."""
    window = schedule.window
    cuts = [start, end]
    if window is not None:
        cuts += [t for t in window if start < t < end]
    cuts.sort()
    initial = state
    contact = False
    active_s = 0.
    for left, right in pairwise(cuts):
        active = window is not None and window[0] <= left and right <= window[1]
        state, on_ground, _ = ground_air_step(aircraft, state, thrust, reaction, right-left,
                                               external_torque_nm=schedule.torque if active else (0., 0., 0.))
        contact = contact or on_ground
        if active:
            active_s += right-left
    dt = end-start
    acceleration = tuple((a-b)/dt for a, b in zip(state.velocity_m_s, initial.velocity_m_s, strict=True))
    specific = inverse_rotate(state.orientation, add(acceleration, (0., 0., -GRAVITY)))
    return state, contact, specific, active_s
