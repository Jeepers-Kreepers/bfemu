"""Повторяемые серии без скрытых ошибок и без перезаписи данных."""
from __future__ import annotations

import json

import pytest

from bfemu import sitl_repeatability
from bfemu.flight_physics import PhysicsError


def test_series_interleaves_periods_and_records_failures(tmp_path, monkeypatch):
    calls = []
    def fake(aircraft, *, pid, axis, sensor_period_ms, motor_mode, timing_probe):
        calls.append((axis, pid, sensor_period_ms, motor_mode, timing_probe))
        if len(calls) == 2:
            raise PhysicsError('Нарушен темп')
        return {'verified_pid': {'axis': axis, 'values': pid}, 'diagnostics': {'peak_abs_dps': 123.0}}
    monkeypatch.setattr(sitl_repeatability.aircraft_sitl, 'sitl_available', lambda: True)
    monkeypatch.setattr(sitl_repeatability.aircraft_sitl, 'experiment', fake)
    output = tmp_path / 'runs.jsonl'
    status = sitl_repeatability.main(['--output', str(output), '--axis', 'pitch', '--pid', '45', '80', '40',
                                     '--periods-ms', '4', '2', '--repeats', '2'])
    assert status == 1
    rows = [json.loads(line) for line in output.read_text(encoding='utf-8').splitlines()]
    assert rows[0]['type'] == 'header' and rows[0]['aircraft']['values']['mass_g'] == '931'
    runs = rows[1:-1]
    assert [r['sensor_period_ms'] for r in runs] == [4, 2, 4, 2]
    assert [r['ok'] for r in runs] == [True, False, True, True]
    assert runs[1]['error'] == 'Нарушен темп'
    assert rows[-1] == {'type': 'summary', 'attempts': 4, 'not_completed': 1}
    assert len(calls) == 4 and all(c[0] == 'pitch' and c[1] == {'p': 45, 'i': 80, 'd': 40} for c in calls)
    before = output.read_bytes()
    assert sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40']) == 1
    assert output.read_bytes() == before


def test_series_validates_before_creating_output(tmp_path, monkeypatch):
    output = tmp_path / 'runs.jsonl'
    for options in (['--modes', 'poll', 'poll'], ['--periods-ms', '4', '4'], ['--periods-ms', '3'], ['--repeats', '11']):
        with pytest.raises(SystemExit):
            sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40', *options])
        assert not output.exists()
    monkeypatch.setattr(sitl_repeatability.aircraft_sitl, 'sitl_available', lambda: False)
    assert sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40']) == 1
    assert not output.exists()


def test_series_compares_modes_and_saves_queue_statistics(tmp_path, monkeypatch):
    class Probe:
        def summary(self):
            return {'valid': True, 'packets': 2, 'median_ms': .05, 'p95_ms': .1, 'max_ms': .2}
    calls = []
    def fake(_aircraft, *, pid, axis, sensor_period_ms, motor_mode, timing_probe):
        assert isinstance(timing_probe, Probe)
        calls.append((sensor_period_ms, motor_mode))
        return {'verified_pid': {'axis': axis, 'values': pid}, 'diagnostics': {'peak_abs_dps': 50}}
    monkeypatch.setattr(sitl_repeatability, 'timing_supported', lambda: True)
    monkeypatch.setattr(sitl_repeatability, 'KernelQueueProbe', Probe)
    monkeypatch.setattr(sitl_repeatability.aircraft_sitl, 'sitl_available', lambda: True)
    monkeypatch.setattr(sitl_repeatability.aircraft_sitl, 'experiment', fake)
    output = tmp_path / 'timing.jsonl'
    status = sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40', '--periods-ms', '4',
                                     '--modes', 'poll', 'event', '--kernel-queue-timing', '--repeats', '1'])
    rows = [json.loads(line) for line in output.read_text(encoding='utf-8').splitlines()]
    assert status == 0 and calls == [(4, 'poll'), (4, 'event')]
    assert rows[0]['kernel_queue_timing'] is True
    assert [row['motor_mode'] for row in rows[1:-1]] == ['poll', 'event']
    assert all(row['kernel_queue']['median_ms'] == .05 for row in rows[1:-1])
    assert rows[-1] == {'type': 'summary', 'attempts': 2, 'not_completed': 0}


def test_queue_timing_on_unsupported_platform_does_not_create_file(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(sitl_repeatability, 'timing_supported', lambda: False)
    output = tmp_path / 'timing.jsonl'
    assert sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40', '--kernel-queue-timing']) == 1
    assert not output.exists()
    assert 'Linux' in capsys.readouterr().err
