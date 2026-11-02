"""Повторяемые серии без скрытых ошибок и без перезаписи данных."""
from __future__ import annotations

import json

import pytest

from bfemu import sitl_repeatability
from bfemu.flight_physics import PhysicsError


def test_series_interleaves_periods_and_records_failures(tmp_path, monkeypatch):
    calls = []
    def fake(aircraft, *, pid, axis, sensor_period_ms):
        calls.append((axis, pid, sensor_period_ms))
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
    for options in (['--periods-ms', '4', '4'], ['--periods-ms', '3'], ['--repeats', '11']):
        with pytest.raises(SystemExit):
            sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40', *options])
        assert not output.exists()
    monkeypatch.setattr(sitl_repeatability.aircraft_sitl, 'sitl_available', lambda: False)
    assert sitl_repeatability.main(['--output', str(output), '--pid', '45', '80', '40']) == 1
    assert not output.exists()
