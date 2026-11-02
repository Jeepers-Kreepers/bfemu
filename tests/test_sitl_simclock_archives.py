"""Перепроверка сохранённых результатов, а не запуск FC в модульном тесте."""
import hashlib
import json
from pathlib import Path

import pytest

from tools.sitl_simclock_analyze import analyze, compare, load_series

ROOT = Path(__file__).resolve().parents[1]/'docs/research'


def test_archived_probe_sources_match_recorded_sha256():
    for directory in sorted(ROOT.glob('sitl-simclock-*')):
        header = json.loads((directory/'summary.jsonl').read_text().splitlines()[0])
        driver = directory/('sitl_simclock_probe.py' if (directory/'sitl_simclock_probe.py').exists() else 'driver.py')
        assert hashlib.sha256(driver.read_bytes()).hexdigest() == header['driver_sha256']
        for name, digest in header['includes_sha256'].items():
            path = directory/name
            if not path.exists():
                path = directory/('probe.inc' if name == 'sitl_simclock_probe.inc' else 'target.inc')
            assert hashlib.sha256(path.read_bytes()).hexdigest() == digest


def test_saved_flight_responses_are_repeatable_not_identical_across_p_values():
    p49 = ROOT/'sitl-simclock-p49-v2'
    p55 = ROOT/'sitl-simclock-p55-v2'
    pauses = ROOT/'sitl-simclock-pauses-v1'
    _, a, af = load_series(p49)
    _, b, bf = load_series(p55)
    _, c, cf = load_series(pauses)
    assert (len(a), len(b), len(c), len(af), len(bf), len(cf)) == (8, 8, 3, 0, 0, 1)
    for runs in (a, b, c):
        for run in runs[1:]:
            comparison = compare(runs[0], run)
            assert comparison['samples'] == 425
            assert comparison['rms_dps'] == comparison['max_abs_dps'] == 0
            assert comparison['motor_frames_differ'] == comparison['rc_frames_differ'] == 0
            assert comparison['setpoint_frames_differ'] == 0
    assert compare(a[0], c[0])['max_abs_dps'] == 0
    changed = compare(a[0], b[0])
    assert changed['rms_dps'] == pytest.approx(1.462893536587059)
    assert changed['max_abs_dps'] == pytest.approx(5.4431275334234)
    assert changed['motor_frames_differ'] == 132
    assert changed['setpoint_frames_differ'] == changed['rc_frames_differ'] == 0
    report = analyze([p49, p55, pauses])
    assert len(report['pairs']) == 171


def test_analyzer_refuses_mismatched_or_short_segments():
    row = {'channels': [1], 'motors': [0, 0, 0, 0], 'rc_pitch': 1,
           'setpoint_pitch_dps': 0, 'state': {'angular_velocity_rad_s': [0, 0, 0]}}
    with pytest.raises(ValueError, match='Интервалы'):
        compare({'segment': [row]}, {'segment': [row, row]})
    with pytest.raises(ValueError, match='Интервалы'):
        compare({'segment': [row]}, {'segment': [{**row, 'channels': [2]}]})
