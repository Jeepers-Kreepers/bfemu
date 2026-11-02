"""Контролируемая задержка поступающих команд, не модифицирующая прошивку SITL."""
import json
import os

import pytest

from bfemu import sitl_delay_probe
from bfemu.flight_physics import PhysicsError


class FakeClock:
    def __init__(self):
        self.now = 0.
    def __call__(self):
        return self.now


class ScheduledSocket:
    def __init__(self, clock):
        self.clock = clock
        self.packets = [(.001, b'first'), (.003, b'second')]
    def setblocking(self, value):
        assert value is False
    def recvfrom(self, size):
        if not self.packets or self.packets[0][0] > self.clock.now:
            raise BlockingIOError
        return self.packets.pop(0)[1], ('127.0.0.1', 1234)
    def wait(self, readers, _write, _errors, timeout):
        assert readers == [self]
        deadline = self.clock.now+timeout
        if self.packets and self.packets[0][0] <= deadline:
            self.clock.now = self.packets[0][0]
            return readers, [], []
        self.clock.now = deadline
        return [], [], []


def test_delay_queues_in_order_and_respects_sensor_deadline():
    clock = FakeClock()
    sock = ScheduledSocket(clock)
    probe = sitl_delay_probe.DelayedMotorInput(sock, .004, clock=clock, wait=sock.wait)
    probe.setblocking(False)
    assert probe.wait([probe], [], [], .002) == ([], [], [])
    assert clock.now == pytest.approx(.002)
    with pytest.raises(BlockingIOError):
        probe.recvfrom(100)
    assert probe.wait([probe], [], [], .004) == ([probe], [], [])
    assert clock.now == pytest.approx(.005)
    assert probe.recvfrom(100)[0] == b'first'
    with pytest.raises(BlockingIOError):
        probe.recvfrom(100)
    assert probe.wait([probe], [], [], .004) == ([probe], [], [])
    assert clock.now == pytest.approx(.007)
    assert probe.recvfrom(100)[0] == b'second'
    assert probe.summary() == {'count': 2, 'median_ms': 4.0, 'p95_ms': 4.0, 'max_ms': 4.0}


def test_invalid_delays_and_empty_summary():
    for delay in (-.001, .021, float('nan'), float('inf')):
        with pytest.raises(PhysicsError, match='задержка'):
            sitl_delay_probe.DelayedMotorInput(None, delay)
    assert sitl_delay_probe.DelayedMotorInput(None, 0).summary() == {'count': 0}


def test_cli_records_every_delay_and_failure_without_overwriting(tmp_path, monkeypatch):
    calls = []
    def fake(_profile, *, axis, pid, sensor_period_ms, motor_mode, _event_loop):
        calls.append((axis, pid, sensor_period_ms, motor_mode, _event_loop))
        if len(calls) == 2:
            raise PhysicsError('Нарушен темп')
        return {'verified_pid': {'axis': axis, 'values': pid}, 'diagnostics': {'peak_abs_dps': 90}}
    monkeypatch.setattr(sitl_delay_probe.aircraft_sitl, 'experiment', fake)
    monkeypatch.setattr(sitl_delay_probe.aircraft_sitl, 'sitl_available', lambda: True)
    output = tmp_path/'delay.jsonl'
    args = ['--output', str(output), '--pid', '45', '80', '40', '--delays-ms', '0', '4', '--repeats', '2', '--alternate-order']
    assert sitl_delay_probe.main(args) == 1
    rows = [json.loads(s) for s in output.read_text(encoding='utf-8').splitlines()]
    assert [row['requested_delay_ms'] for row in rows[1:-1]] == [0, 4, 4, 0]
    assert [row['ok'] for row in rows[1:-1]] == [True, False, True, True]
    assert rows[0]['alternate_order'] is True
    assert rows[2]['error'] == 'Нарушен темп'
    assert rows[-1] == {'type': 'summary', 'attempts': 4, 'not_completed': 1}
    assert len(calls) == 4 and all(row[3] == 'event' and callable(row[4]) for row in calls)
    before = output.read_bytes()
    assert sitl_delay_probe.main(args) == 1
    assert output.read_bytes() == before


def test_cli_rejects_duplicate_delay_before_creating_file(tmp_path):
    output = tmp_path/'delay.jsonl'
    with pytest.raises(SystemExit):
        sitl_delay_probe.main(['--output', str(output), '--pid', '45', '80', '40', '--delays-ms', '4', '4'])
    assert not output.exists()


def test_delayed_loop_passes_socket_and_wait_to_original_loop(monkeypatch):
    captured = {}
    def fake_loop(profile, model, motor, state, stream, **kwargs):
        captured.update(profile=profile, model=model, motor=motor, state=state, stream=stream, kwargs=kwargs)
        return 42
    monkeypatch.setattr(sitl_delay_probe, 'run_event_loop', fake_loop)
    probes = []
    loop = sitl_delay_probe.make_delayed_loop(.004, probes)
    assert loop('profile', 'model', 'socket', 'fdm', 'stream', period=.004) == 42
    assert len(probes) == 1 and captured['motor'] is probes[0]
    assert probes[0].sock == 'socket' and probes[0].delay_s == .004
    assert captured['kwargs']['wait'] == probes[0].wait and captured['kwargs']['period'] == .004


def test_flood_fails_closed_without_unbounded_queue():
    clock = FakeClock()
    class Flood:
        def recvfrom(self, _size):
            return b'data', ('127.0.0.1', 1234)
    sock = Flood()
    probe = sitl_delay_probe.DelayedMotorInput(sock, .004, clock=clock,
                                               wait=lambda readers, _w, _e, _t: (readers, [], []))
    with pytest.raises(PhysicsError, match='переполнена'):
        probe.wait([probe], [], [], .01)
    assert len(probe.pending) <= 4096


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный SITL и свободные порты')
def test_real_sitl_completes_delayed_and_control_run(tmp_path):
    output = tmp_path/'real-delay.jsonl'
    status = sitl_delay_probe.main(['--output', str(output), '--pid', '45', '80', '40', '--period-ms', '4',
                                    '--delays-ms', '0', '4', '--repeats', '1'])
    rows = [json.loads(s) for s in output.read_text(encoding='utf-8').splitlines()]
    assert status == 0 and rows[-1] == {'type': 'summary', 'attempts': 2, 'not_completed': 0}
    assert rows[1]['ok'] and rows[2]['ok']
    assert rows[1]['hold_delay']['count'] > 0
    assert rows[2]['hold_delay']['count'] > 0 and rows[2]['hold_delay']['median_ms'] >= 4
    assert rows[2]['verified_pid']['values'] == {'p': 45, 'i': 80, 'd': 40}
