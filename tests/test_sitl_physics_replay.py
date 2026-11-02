"""Детерминированность ядра физики при одинаковом потоке внешних воздействий."""
import io
import json
import struct

import pytest

from bfemu.sitl_closed_loop import run_loop
from tests.test_flight_actuators import aircraft_profile, model
from tests.test_sitl_feedback import FakeClock, FakeMotorSocket, FakeStateSocket
from tools.sitl_physics_replay import replay


def fixture_log():
    clock, stream = FakeClock(oversleep=.001), io.StringIO()
    run_loop(aircraft_profile(), model(), FakeMotorSocket(struct.pack('=4f', 0, 0, 0, .5)),
             FakeStateSocket(), stream, duration=.2, period=.004, max_gap=.1, pressure_pa=101325,
             clock=clock.clock, sleep=clock.sleep)
    return stream.getvalue()


def test_physics_replays_the_same_input_without_numeric_difference():
    data = fixture_log()
    one, two = replay(data), replay(data)
    assert one == two
    assert one['samples'] > 20
    assert one['mismatched_values'] == 0 and one['first_mismatch'] is None and one['max_abs_difference'] == 0


def test_replay_detects_tampered_state_and_refuses_missing_inputs():
    rows = [json.loads(line) for line in fixture_log().splitlines()]
    rows[2]['state']['angular_velocity_rad_s'][0] += .5
    result = replay('\n'.join(json.dumps(row) for row in rows))
    assert result['mismatched_values'] == 1
    assert result['first_mismatch']['field'] == 'angular_velocity_rad_s'
    rows[2]['applied_commands'][0] = 0
    with pytest.raises(ValueError, match='не соответствует'):
        replay('\n'.join(json.dumps(row) for row in rows))
    rows = [json.loads(line) for line in fixture_log().splitlines()]
    rows[2]['dt_s'] += .001
    with pytest.raises(ValueError, match='временной шаг'):
        replay('\n'.join(json.dumps(row) for row in rows))


def test_archived_real_sitl_trajectory_replays_exactly_twice():
    import gzip
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]/'docs/research/physics-replay-v1'
    summary = json.loads((root/'summary.json').read_text(encoding='utf-8'))
    data = gzip.decompress((root/'physics.jsonl.gz').read_bytes()).decode('utf-8')
    first, second = replay(data), replay(data)
    assert first == second == summary['replay']
    assert summary['ok'] is True and summary['verified_pid']['values'] == summary['pid']
    assert first['samples'] == 2917 and first['mismatched_values'] == 0
