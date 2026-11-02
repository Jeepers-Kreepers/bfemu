"""Обычный виртуальный FC не подставляет динамику SITL в фигурку Configurator."""
import struct

from bfemu.fc import VirtualFC
from bfemu.fc_constants import MSP_ATTITUDE


def test_virtual_fc_reports_a_stationary_attitude_at_different_times(monkeypatch):
    fc = VirtualFC(eeprom_path=None)
    for now in (0., .5, 1., 10., 100.):
        monkeypatch.setattr(fc, '_t', lambda now=now: now)
        data = fc.handle(MSP_ATTITUDE, b'', 1)
        assert len(data) == 6
        assert struct.unpack('<hhh', data) == (0, 0, 0)


def test_free_body_keeps_rotating_after_motors_stop_without_drag():
    import math
    import pytest

    from bfemu.flight_physics import Aircraft, State, step
    from tests.test_flight_actuators import aircraft_profile

    aircraft = Aircraft.from_profile(aircraft_profile())
    state = State(angular_velocity_rad_s=(0., .5, 0.))
    for _ in range(250):
        state = step(aircraft, state, [0.]*4, [0.]*4, .004)
    assert state.angular_velocity_rad_s == pytest.approx((0., .5, 0.), abs=1e-12)
    assert state.orientation == pytest.approx((math.cos(.25), 0., math.sin(.25), 0.), abs=1e-8)
