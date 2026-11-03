"""Аналитические проверки ядра твёрдого тела и профиля, без имитации FC."""
from __future__ import annotations

import json
import math
import subprocess
import sys

import pytest

from bfemu.flight_physics import GRAVITY, Aircraft, PhysicsError, State, derivative, step


def profile():
    values = {'mass_g': '1000', 'cg_x': '0', 'cg_y': '0', 'cg_z': '0',
              'inertia_xx': '2', 'inertia_yy': '3', 'inertia_zz': '4',
              'inertia_xy': '0', 'inertia_xz': '0', 'inertia_yz': '0'}
    for i, (x, y) in enumerate(((-1000, 1000), (1000, 1000), (-1000, -1000), (1000, -1000)), 1):
        values.update({f'motor_{i}_x': str(x), f'motor_{i}_y': str(y), f'motor_{i}_z': '0'})
    return {'schema': 1, 'values': values}


def test_hover_freefall_and_mass_affect_acceleration():
    p = profile()
    aircraft = Aircraft.from_profile(p)
    state = State()
    hover = derivative(aircraft, state, [GRAVITY/4]*4, [0]*4)
    assert hover.acceleration_m_s2 == pytest.approx((0, 0, 0), abs=1e-12)
    assert hover.angular_acceleration_rad_s2 == pytest.approx((0, 0, 0), abs=1e-12)
    fall = step(aircraft, state, [0]*4, [0]*4, .1)
    assert fall.position_m == pytest.approx((0, 0, GRAVITY*.1**2/2))
    assert fall.velocity_m_s == pytest.approx((0, 0, GRAVITY*.1))
    p['values']['mass_g'] = '2000'
    heavier = derivative(Aircraft.from_profile(p), state, [GRAVITY/4]*4, [0]*4)
    assert heavier.acceleration_m_s2 == pytest.approx((0, 0, GRAVITY/2))


def test_torque_geometry_center_inertia_and_signed_reaction():
    p = profile()
    aircraft = Aircraft.from_profile(p)
    d = derivative(aircraft, State(), [1, 0, 0, 0], [0, 0, 0, .4])
    # r=(-1,+1,0), F=(0,0,-1): tau=(-1,-1,0), reaction torque +0.4 around Z.
    assert d.angular_acceleration_rad_s2 == pytest.approx((-.5, -1/3, .1))
    p['values']['cg_y'] = '500'
    p['values']['inertia_xx'] = '1'
    changed = derivative(Aircraft.from_profile(p), State(), [1, 0, 0, 0], [0, 0, 0, .4])
    assert changed.angular_acceleration_rad_s2[0] == pytest.approx(-.5)
    p['values']['motor_1_y'] = '1500'
    changed = derivative(Aircraft.from_profile(p), State(), [1, 0, 0, 0], [0, 0, 0, .4])
    assert changed.angular_acceleration_rad_s2[0] == pytest.approx(-1)


def test_full_inertia_and_gyroscopic_coupling():
    p = profile()
    p['values']['inertia_xy'] = '.2'
    p['values']['inertia_xz'] = '.1'
    p['values']['inertia_yz'] = '-.1'
    aircraft = Aircraft.from_profile(p)
    state = State(angular_velocity_rad_s=(.8, -.3, .4))
    d = derivative(aircraft, state, [1, 0, 0, 0], [0, 0, 0, .4])
    # Проверяем уравнение Эйлера J*omega_dot + omega × J*omega = tau.
    w = state.angular_velocity_rad_s
    j = aircraft.inertia
    jw = tuple(sum(j[i][k]*w[k] for k in range(3)) for i in range(3))
    cross = (w[1]*jw[2]-w[2]*jw[1], w[2]*jw[0]-w[0]*jw[2], w[0]*jw[1]-w[1]*jw[0])
    torque = tuple(sum(j[i][k]*d.angular_acceleration_rad_s2[k] for k in range(3)) + cross[i] for i in range(3))
    assert torque == pytest.approx((-1, -1, .4))


def test_orientation_and_rk4_rotation():
    aircraft = Aircraft.from_profile(profile())
    q = (math.sqrt(.5), 0, math.sqrt(.5), 0)
    acceleration = derivative(aircraft, State(orientation=q), [1]*4, [0]*4).acceleration_m_s2
    assert acceleration == pytest.approx((-4, 0, GRAVITY), abs=1e-12)
    state = State(angular_velocity_rad_s=(1, 0, 0))
    for _ in range(100):
        state = step(aircraft, state, [0]*4, [0]*4, .01)
    assert state.orientation == pytest.approx((math.cos(.5), math.sin(.5), 0, 0), abs=1e-8)
    assert math.hypot(*state.orientation) == pytest.approx(1)


def test_rejects_unknown_incomplete_and_unphysical_profile():
    p = profile()
    p['values']['mass_g'] = 'Неизвестно'
    with pytest.raises(PhysicsError, match='mass_g'):
        Aircraft.from_profile(p)
    p = profile()
    del p['values']['motor_2_z']
    with pytest.raises(PhysicsError, match='motor_2_z'):
        Aircraft.from_profile(p)
    p = profile()
    p['values']['inertia_zz'] = '10'  # Izz > Ixx + Iyy: no distribution of mass can produce this.
    with pytest.raises(PhysicsError, match='распределению массы'):
        Aircraft.from_profile(p)
    p['values']['inertia_xy'] = '100'
    with pytest.raises(PhysicsError, match='положительно определённой'):
        Aircraft.from_profile(p)
    aircraft = Aircraft.from_profile(profile())
    with pytest.raises(PhysicsError, match='Отрицательная тяга'):
        derivative(aircraft, State(), [-1, 0, 0, 0], [0]*4)
    with pytest.raises(PhysicsError, match='Шаг времени'):
        step(aircraft, State(), [0]*4, [0]*4, 0)
    with pytest.raises(PhysicsError, match='Кватернион'):
        derivative(aircraft, State(orientation=(0, 0, 0, 0)), [0]*4, [0]*4)


def test_cli_needs_explicit_forces_and_reads_export(tmp_path):
    source = tmp_path / 'profile.json'
    source.write_text(json.dumps(profile()), encoding='utf-8')
    args = [sys.executable, '-m', 'bfemu.flight_physics', '--profile', str(source), '--thrust', '0', '0', '0', '0',
            '--reaction-torque', '0', '0', '0', '0', '--dt', '0.1', '--steps', '2']
    result = subprocess.run(args, text=True, capture_output=True, check=True)
    data = json.loads(result.stdout)
    assert data['time_s'] == pytest.approx(.2)
    assert data['state']['position_m'][2] == pytest.approx(GRAVITY*.2**2/2)
    no_forces = subprocess.run(args[:5], text=True, capture_output=True, check=False)
    assert no_forces.returncode != 0


def test_external_body_torque_affects_rotation_not_translation():
    aircraft = Aircraft.from_profile(profile())
    state = State()
    no_force = derivative(aircraft, state, [0]*4, [0]*4)
    forced = derivative(aircraft, state, [0]*4, [0]*4, external_torque_nm=(0, -0.6, 0))
    assert forced.acceleration_m_s2 == no_force.acceleration_m_s2
    assert forced.angular_acceleration_rad_s2 == pytest.approx((0, -0.2, 0))
    result = step(aircraft, state, [0]*4, [0]*4, .1, external_torque_nm=(0, -0.6, 0))
    assert result.angular_velocity_rad_s == pytest.approx((0, -.02, 0))
    with pytest.raises(PhysicsError):
        derivative(aircraft, state, [0]*4, [0]*4, external_torque_nm=(float('nan'), 0, 0))
