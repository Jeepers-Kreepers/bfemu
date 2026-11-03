"""Границы и параметры порыва штатного асинхронного эксперимента."""
import io
import json
import struct

import pytest

from bfemu.flight_environment import ground_air_step
from bfemu.flight_physics import Aircraft, State
from bfemu.sitl_closed_loop import run_event_loop, run_loop
from bfemu.sitl_gust import GustSchedule, advance_with_gust
from tests.test_flight_actuators import aircraft_profile, model
from tests.test_sitl_feedback import FakeClock, FakeMotorSocket, FakeStateSocket


@pytest.mark.parametrize('moment,duration', [(0, 200), (True, 200), (.21, 200), (float('nan'), 200),
                                             (.05, 19), (.05, 21), (.05, True), (.05, 804)])
def test_gust_validation(moment, duration):
    with pytest.raises(ValueError):
        GustSchedule('pitch', moment, duration)


def test_gust_splits_both_boundaries_and_only_integrates_torque_inside():
    aircraft = Aircraft.from_profile(aircraft_profile())
    initial = State(position_m=(0, 0, -10))
    schedule = GustSchedule('pitch', -.08, 20)
    schedule.schedule(.005)
    actual, contact, _, applied = advance_with_gust(aircraft, initial, [0]*4, [0]*4, 0, .03, schedule)
    expected = initial
    for dt, torque in [(.005, (0, 0, 0)), (.02, (0, -.08, 0)), (.005, (0, 0, 0))]:
        expected, _, _ = ground_air_step(aircraft, expected, [0]*4, [0]*4, dt, external_torque_nm=torque)
    for field in ('position_m', 'velocity_m_s', 'orientation', 'angular_velocity_rad_s'):
        assert getattr(actual, field) == pytest.approx(getattr(expected, field), abs=1e-14)
    assert not contact and applied == pytest.approx(.02)
    before, _, _, active = advance_with_gust(aircraft, initial, [0]*4, [0]*4, 0, .004, schedule)
    no_gust, _, _ = ground_air_step(aircraft, initial, [0]*4, [0]*4, .004)
    assert before == no_gust and active == 0
    after, _, _, active = advance_with_gust(aircraft, initial, [0]*4, [0]*4, .026, .03, schedule)
    assert after.angular_velocity_rad_s == no_gust.angular_velocity_rad_s and active == 0
    with pytest.raises(ValueError):
        schedule.schedule(1)


@pytest.mark.parametrize('mode', ['poll', 'event'])
def test_loops_record_full_gust_duration(mode):
    clock = FakeClock()
    schedule = GustSchedule('roll', .05, 20)
    schedule.schedule(.005)
    output = io.StringIO()
    kw = dict(duration=.04, period=.004, max_gap=.1, pressure_pa=101325,
              clock=clock.clock, environment='ground-air-v1', gust_schedule=schedule)
    motor = FakeMotorSocket(struct.pack('=4f', 0, 0, 0, 0))
    if mode == 'poll':
        run_loop(aircraft_profile(), model(), motor, FakeStateSocket(), output, sleep=clock.sleep, **kw)
    else:
        def wait(readers, _write, _errors, timeout):
            clock.now += timeout
            return readers, [], []
        run_event_loop(aircraft_profile(), model(), motor, FakeStateSocket(), output, wait=wait, **kw)
    rows = [json.loads(line) for line in output.getvalue().splitlines()][1:]
    assert sum(row['gust_active_s'] for row in rows) == pytest.approx(.02)
    assert rows[0]['gust_active_s'] == 0
    assert any(0 < row['gust_active_s'] < row['dt_s'] for row in rows)
    assert rows[-1]['gust_active_s'] == 0


def test_gust_recovery_uses_last_out_of_band_sample():
    from bfemu.aircraft_sitl import _gust_metrics
    import math
    def row(t, rate):
        return {'elapsed_s': t, 'state': {'angular_velocity_rad_s': [0, math.radians(rate), 0]}}
    rows = [row(.2, 8), row(.3, 4), row(.4, 7), row(.5, 3), row(1.2, 2)]
    metrics = _gust_metrics(rows, 0, .2, 'pitch')
    assert metrics['gust_recovery_s'] == .3
    assert _gust_metrics([row(.2, 2), row(1.2, 4)], 0, .2, 'pitch')['gust_recovery_s'] == 0
    assert _gust_metrics([row(.2, 2), row(1.2, 8)], 0, .2, 'pitch')['gust_recovery_s'] is None


def test_gust_incomplete_or_ground_contact_is_rejected():
    from bfemu.aircraft_sitl import _gust_delivery
    from bfemu.flight_physics import PhysicsError
    rows = [{'elapsed_s': .21, 'dt_s': .02, 'gust_active_s': .2, 'ground_contact': False}]
    assert _gust_delivery(rows, 0, 1.2, .2) == pytest.approx(.2)
    with pytest.raises(PhysicsError, match='длительность'):
        _gust_delivery(rows, 0, 1.2, .3)
    # Контакт в интервале, заканчивающемся уже после окна, тоже опасен.
    rows.append({'elapsed_s': 1.21, 'dt_s': .02, 'gust_active_s': 0, 'ground_contact': True})
    with pytest.raises(PhysicsError, match='земли'):
        _gust_delivery(rows, 0, 1.2, .2)


def test_event_accumulates_gust_across_motor_events_between_sensors():
    clock = FakeClock()
    packets = [(t, struct.pack('=4f', 0, 0, 0, .1)) for t in (.001, .003, .006, .007, .019, .023)]
    class Motor:
        def setblocking(self, value):
            assert not value
        def recvfrom(self, _):
            if packets and packets[0][0] <= clock.now:
                return packets.pop(0)[1], ('127.0.0.1', 1234)
            raise BlockingIOError
    def wait(readers, _write, _errors, timeout):
        deadline = clock.now+timeout
        if packets and packets[0][0] <= deadline:
            clock.now = packets[0][0]
            return readers, [], []
        clock.now = deadline
        return [], [], []
    schedule = GustSchedule('roll', .05, 20)
    schedule.schedule(.002)
    output = io.StringIO()
    run_event_loop(aircraft_profile(), model(), Motor(), FakeStateSocket(), output,
                   duration=.032, period=.004, max_gap=.1, pressure_pa=101325,
                   clock=clock.clock, wait=wait, environment='ground-air-v1', gust_schedule=schedule)
    rows = [json.loads(line) for line in output.getvalue().splitlines()][1:]
    assert rows[0]['gust_active_s'] == pytest.approx(.002)
    assert rows[1]['gust_active_s'] == pytest.approx(.004)
    assert sum(r['gust_active_s'] for r in rows) == pytest.approx(.02)
