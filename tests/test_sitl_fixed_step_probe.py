"""Фиксированный шаг исследовательской модели не скрывает задержки хоста."""
import io
import json
import struct

import pytest

from bfemu.flight_physics import PhysicsError
from tests.test_flight_actuators import aircraft_profile, model
from tests.test_sitl_feedback import FakeClock, FakeMotorSocket, FakeStateSocket
from tools.sitl_fixed_step_probe import run_fixed_loop


def test_fixed_step_does_not_replace_wall_clock_and_applies_previous_motor_command():
    clock = FakeClock(oversleep=.001)
    receiver = FakeMotorSocket(struct.pack('=4f', 0, 0, 0, .5))
    sender, output = FakeStateSocket(), io.StringIO()
    count = run_fixed_loop(aircraft_profile(), model(), receiver, sender, output, duration=.03, period=.004,
                           max_gap=.1, pressure_pa=101325, clock=clock.clock, sleep=clock.sleep)
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert count == len(rows)-1 == len(sender.packets)-1
    assert rows[0]['physics_time_basis'] == 'fixed_step'
    assert rows[1]['elapsed_s'] == pytest.approx(.005)
    assert rows[1]['dt_s'] == pytest.approx(.005)
    assert rows[1]['sim_time_s'] == pytest.approx(.004)
    assert rows[1]['applied_commands'] == [0, 0, 0, 0]
    assert rows[2]['applied_commands'] == pytest.approx([.5, 0, 0, 0])
    assert rows[2]['state']['angular_velocity_rad_s'][0] < 0
    assert [packet[0] for packet in sender.packets] == pytest.approx([i*.004 for i in range(count+1)])
    assert sender.packets[2][1] == pytest.approx(rows[2]['state']['angular_velocity_rad_s'][0])


def test_fixed_step_still_rejects_hung_host_and_missing_motors():
    clock = FakeClock(oversleep=.02)
    with pytest.raises(PhysicsError, match='Нарушен темп'):
        run_fixed_loop(aircraft_profile(), model(), FakeMotorSocket(), FakeStateSocket(), io.StringIO(),
                       duration=.2, period=.004, max_gap=.1, pressure_pa=101325, clock=clock.clock, sleep=clock.sleep)
    clock = FakeClock()
    with pytest.raises(PhysicsError, match='Нет пакетов моторов'):
        run_fixed_loop(aircraft_profile(), model(), FakeMotorSocket(), FakeStateSocket(), io.StringIO(),
                       duration=.2, period=.004, max_gap=.05, pressure_pa=101325, clock=clock.clock, sleep=clock.sleep)


def test_archived_fixed_step_series_retains_every_run_and_clock_drift():
    import gzip
    from pathlib import Path

    base = Path(__file__).resolve().parents[1]/'docs/research'
    for name, count in (('fixed-step-v1', 8), ('fixed-step-replication', 16)):
        root = base/name
        rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
        assert rows[-1] == {'type': 'summary', 'attempts': count, 'not_completed': 0}
        assert all(row['ok'] and row['verified_pid']['values'] == rows[0]['pid'] for row in rows[1:-1])
        for row in rows[1:-1]:
            if row['fixed']:
                timing = row['clock_comparison']
                assert timing['last_sim_s'] == pytest.approx(timing['sample_count']*.004)
                assert timing['last_wall_s'] > timing['last_sim_s']
            data = json.loads(gzip.decompress((root/(row['physics_file']+'.gz')).read_bytes()))
            assert len(data['samples']) > 1000 and len(data['command_window_s']) == 2


def test_archived_two_millisecond_attempts_include_failures():
    import gzip
    from pathlib import Path

    path = Path(__file__).resolve().parents[1]/'docs/research/pitch-2ms-v1/series.jsonl.gz'
    rows = [json.loads(line) for line in gzip.decompress(path.read_bytes()).decode('utf-8').splitlines()]
    assert rows[0]['axis'] == 'pitch' and rows[0]['periods_ms'] == [2]
    assert rows[-1] == {'type': 'summary', 'attempts': 8, 'not_completed': 2}
    assert [row['cycle'] for row in rows[1:-1] if not row['ok']] == [5, 8]
    assert all(row['verified_pid']['values'] == rows[0]['pid'] for row in rows[1:-1] if row['ok'])
