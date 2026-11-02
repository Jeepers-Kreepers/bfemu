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


def test_comparison_uses_selected_axis_and_checks_controller_snapshots():
    def row(roll, pitch, yaw, rc, target):
        return {'channels': [1500] * 4, 'motors': [0.2] * 4, 'rc_axes': rc,
                'setpoint_axes_dps': target, 'state': {'angular_velocity_rad_s': [roll, pitch, yaw]}}

    first = {'axis': 'yaw', 'segment': [row(0, 0, 0, [1500] * 3, [0] * 3)]}
    second = {'axis': 'yaw', 'segment': [row(2, 3, 1, [1500, 1500, 1650], [0, 0, 75])]}
    difference = compare(first, second)
    assert difference['rms_dps'] == pytest.approx(57.29577951308232)
    assert difference['max_abs_dps'] == pytest.approx(57.29577951308232)
    assert difference['rc_frames_differ'] == difference['setpoint_frames_differ'] == 1
    assert difference['motor_frames_differ'] == 0
    with pytest.raises(ValueError, match='разные оси'):
        compare(first, {**second, 'axis': 'roll'})


def test_multiaxis_archives_and_i_d_changes():
    names = ('yaw-v2', 'roll-v1', 'pitch-base-v1', 'pitch-i105-v1', 'pitch-d55-v1', 'pitch-pauses-v2')
    all_runs = {}
    for name in names:
        _, runs, failures = load_series(ROOT/f'sitl-simclock-multiaxis-{name}')
        assert len(runs) == 6 and not failures
        all_runs[name] = runs
        for index, run in enumerate(runs):
            for other in runs[index+1:]:
                result = compare(run, other)
                assert result['samples'] == 425
                assert result['rms_dps'] == result['max_abs_dps'] == result['max_motor_difference'] == 0
                assert result['rc_frames_differ'] == result['setpoint_frames_differ'] == 0
    base = all_runs['pitch-base-v1'][0]
    assert compare(base, all_runs['pitch-pauses-v2'][0])['max_abs_dps'] == 0
    assert compare(base, all_runs['pitch-i105-v1'][0])['max_abs_dps'] == pytest.approx(2.3752805, abs=1e-5)
    assert compare(base, all_runs['pitch-d55-v1'][0])['max_abs_dps'] == pytest.approx(.650754, abs=1e-6)


def test_pid_term_snapshots_reveal_small_nonzero_difference_and_yaw_i_effect():
    def load(name):
        _, runs, failures = load_series(ROOT/f'sitl-simclock-terms-{name}')
        assert len(runs) == 4 and not failures
        return runs
    yaw = load('yaw-v1')
    yaw_zero = load('yaw-i0-v1')
    yaw_half = load('yaw-i40-v1')
    for first, other, peak in ((yaw[0], yaw_zero[0], 141.62163044889675),
                               (yaw[0], yaw_half[0], 132.4468438658951)):
        diff = compare(first, other)
        assert diff['max_abs_dps'] == pytest.approx(peak)
        assert diff['rc_frames_differ'] == diff['setpoint_frames_differ'] == 0
        assert diff['pid_term_max_abs_difference']['i'] > 0
    d = load('pitch-d55-v1')
    diff = compare(d[0], d[1])
    assert diff['max_abs_dps'] == diff['max_motor_difference'] == 0
    assert diff['pid_term_max_abs_difference']['d'] == pytest.approx(1.33514404296875e-5)


def test_failed_axis_build_archives_original_sources():
    directory = ROOT/'sitl-axis-build-failure-v1'
    metadata = json.loads((directory/'metadata.json').read_text())
    for name in ('sitl_simclock_probe.py', 'sitl_simclock_probe.inc', 'sitl_simclock_target.inc'):
        assert hashlib.sha256((directory/name).read_bytes()).hexdigest() == metadata[name]


def test_recorded_pid_and_gyro_periods_and_yaw_ki():
    for name, axis, expected_ki in (('yaw-i80-v1', 'yaw', 48.8761978),
                                    ('yaw-i0-v1', 'yaw', 0),
                                    ('pitch-v1', 'pitch', 20.5280037)):
        _, runs, failures = load_series(ROOT/f'sitl-simclock-timing-{name}')
        assert len(runs) == 2 and not failures
        index = {'yaw': 2, 'pitch': 1}[axis]
        for run in runs:
            for frame in run['segment']:
                assert frame['pid_loop_us'] == frame['gyro_loop_us'] == frame['gyro_sample_us'] == 125
                assert frame['pid_dt_s'] == pytest.approx(.000125)
                assert frame['pid_ki'][index] == pytest.approx(expected_ki)
        assert compare(runs[0], runs[1])['max_abs_dps'] == 0
