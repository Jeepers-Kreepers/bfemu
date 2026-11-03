"""Проверка земли, воздуха и удельной силы без FC."""
import math

import pytest

from bfemu.flight_environment import DEFAULT_ENVIRONMENT, Environment, air_derivative, ground_air_step
from bfemu.flight_physics import GRAVITY, Aircraft, State
from tests.test_flight_physics import profile


def test_disarmed_on_ground_is_still_and_imu_senses_support():
    a = Aircraft.from_profile(profile())
    s = State()
    for _ in range(250):
        s, contact, specific = ground_air_step(a, s, [0]*4, [0]*4, .004)
        assert contact
    assert s == State()
    assert specific == pytest.approx((0, 0, -GRAVITY))


def test_takeoff_hover_and_landing_with_no_penetration():
    a = Aircraft.from_profile(profile())
    s, contact, specific = ground_air_step(a, State(), [GRAVITY/2]*4, [0]*4, .1)
    assert not contact and s.position_m[2] < 0 and -2*GRAVITY < specific[2] < -GRAVITY
    falling = State(position_m=(0, 0, -.01), velocity_m_s=(0, 0, 1))
    landed, contact, specific = ground_air_step(a, falling, [0]*4, [0]*4, .1)
    assert contact and landed.position_m[2] == 0 and landed.velocity_m_s == (0, 0, 0)
    assert specific[2] == pytest.approx(-10-GRAVITY)


def test_rotational_air_damping_and_drag_are_opposite_motion():
    a = Aircraft.from_profile(profile())
    s = State(position_m=(0, 0, -100), velocity_m_s=(3, -4, 0), angular_velocity_rad_s=(0, 0, math.radians(100)))
    d = air_derivative(a, s, [0]*4, [0]*4, (0, 0, 0), DEFAULT_ENVIRONMENT)
    assert d.acceleration_m_s2[0] < 0 and d.acceleration_m_s2[1] > 0
    assert d.angular_acceleration_rad_s2[2] == pytest.approx(-math.radians(100)*math.radians(400)/120)
    for _ in range(250):
        s, contact, _ = ground_air_step(a, s, [0]*4, [0]*4, .004)
        assert not contact
    assert math.degrees(s.angular_velocity_rad_s[2]) == pytest.approx(100*math.exp(-math.radians(400)/120))


def test_wind_affects_relative_air_velocity_not_zero_ground_speed():
    a = Aircraft.from_profile(profile())
    env = Environment(wind_ned_m_s=(5, 0, 0))
    d = air_derivative(a, State(position_m=(0, 0, -2)), [0]*4, [0]*4, (0, 0, 0), env)
    assert d.acceleration_m_s2[0] > 0
