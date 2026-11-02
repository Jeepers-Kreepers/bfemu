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


def test_report_lists_incomparable_command_sequences_without_misleading_pair(monkeypatch):
    from tools import sitl_simclock_analyze
    def run(stick):
        row = {'channels': [1500, stick, 1300, 1500], 'motors': [0.2] * 4,
               'rc_axes': [1500, stick, 1500], 'setpoint_axes_dps': [0, 75, 0],
               'state': {'angular_velocity_rad_s': [0, 0, 0]}}
        return {'cycle': 1, 'pid': {'axis': 'pitch', 'values': {'p': 49, 'i': 84, 'd': 46}},
                'events': {'command_start': 1, 'command_end': 2, 'armed_at': 1},
                'axis': 'pitch', 'segment': [row]}
    def fake_load(name):
        stick = 1650 if name == 'positive' else 1350
        return {'pid': {}, 'stick': stick}, [run(stick)], []
    monkeypatch.setattr(sitl_simclock_analyze, 'load_series', fake_load)
    result = sitl_simclock_analyze.analyze(['positive', 'negative'])
    assert result['pairs'] == []
    assert result['skipped_pairs'] == {'different_axis': 0, 'different_commands_or_length': 1}
    assert [g['inputs']['stick'] for g in result['groups']] == [1650, 1350]


def test_conditions_archives_expose_response_and_position_differences():
    names = ('pitch-ref-v1', 'pitch-reverse-v1', 'pitch-throttle1600-v1', 'pitch-impulse-v1',
             'pitch-impulse-hold-v1', 'pitch-high-v1', 'yaw-reverse-v1', 'profile-ref-v1',
             'profile-iyy2-v1', 'cross-impulse-v1', 'cross-impulse-hold-v1')
    collected = {}
    for name in names:
        directory = ROOT/f'sitl-simclock-conditions-{name}'
        header, runs, failures = load_series(directory)
        assert len(runs) == 6 and not failures
        collected[name] = runs
        if 'profile_sha256' in header:
            assert hashlib.sha256((directory/'input-profile.json').read_bytes()).hexdigest() == header['profile_sha256']
        for i, run in enumerate(runs):
            for other in runs[i+1:]:
                diff = compare(run, other)
                assert all(value == 0 for value in diff['physical_rate_axes_max_abs_dps'].values())
                assert diff['max_motor_difference'] == diff['quaternion_max_component_difference'] == 0
    high = collected['pitch-high-v1'][0]['segment'][:300]
    assert sum(any(m >= 1 for m in row['motors']) for row in high) == 5
    inertia = compare(collected['profile-ref-v1'][0], collected['profile-iyy2-v1'][0])
    assert inertia['max_abs_dps'] == pytest.approx(26.100406707534308)
    cross = compare(collected['cross-impulse-v1'][0], collected['cross-impulse-hold-v1'][0])
    assert cross['max_abs_dps'] == 0
    # Угловые отклики совпадают, но до ARM модель свободно падала разное время.
    assert cross['position_axes_max_abs_m'][2] > 40


def test_bench_archives_repeat_complete_physics_despite_different_prearm_wait():
    results = {}
    for name in ('bench-base-v1', 'bench-impulse-v1', 'bench-impulse-hold-v1'):
        directory = ROOT/f'sitl-simclock-conditions-{name}'
        header, runs, failures = load_series(directory)
        assert len(runs) == 6 and not failures and header['prearm_bench']
        assert hashlib.sha256((directory/'input-profile.json').read_bytes()).hexdigest() == header['profile_sha256']
        results[name] = runs
        for index, run in enumerate(runs):
            for other in runs[index+1:]:
                diff = compare(run, other)
                assert diff['samples'] == 425
                assert all(value == 0 for value in diff['position_axes_max_abs_m'])
                assert all(value == 0 for value in diff['velocity_axes_max_abs_m_s'])
                assert all(value == 0 for value in diff['physical_rate_axes_max_abs_dps'].values())
                assert all(value == 0 for value in diff['gyro_axes_max_abs_dps'].values())
                assert diff['quaternion_max_component_difference'] == diff['max_motor_difference'] == 0
    for run in results['bench-impulse-v1']:
        for other in results['bench-impulse-hold-v1']:
            diff = compare(run, other)
            assert all(value == 0 for value in diff['position_axes_max_abs_m'])
            assert all(value == 0 for value in diff['velocity_axes_max_abs_m_s'])
            assert all(value == 0 for value in diff['physical_rate_axes_max_abs_dps'].values())
            assert all(value == 0 for value in diff['gyro_axes_max_abs_dps'].values())
            assert diff['quaternion_max_component_difference'] == diff['max_motor_difference'] == 0
            assert all(value == 0 for terms in diff['pid_axes_max_abs_difference'].values() for value in terms.values())


def test_bench_sensor_inputs_and_failed_short_scenario_are_preserved():
    import gzip

    failed = ROOT/'sitl-simclock-conditions-bench-sensor-v1'
    header, runs, failures = load_series(failed)
    assert header['frames'] == 500 and len(runs) == 0 and len(failures) == 2
    assert all('500 кадров' in failure['error'] for failure in failures)
    for number in (1, 2):
        with gzip.open(failed/f'frames-{number}.jsonl.gz', 'rt') as stream:
            assert sum(1 for _ in stream) == 500
    directory = ROOT/'sitl-simclock-conditions-bench-sensor-v2'
    header, runs, failures = load_series(directory)
    assert len(runs) == 2 and not failures and header['prearm_bench']
    assert hashlib.sha256((directory/'input-profile.json').read_bytes()).hexdigest() == header['profile_sha256']
    summary = [json.loads(line) for line in (directory/'summary.jsonl').read_text().splitlines()]
    for run in summary[1:-1]:
        with gzip.open(directory/(run['file']+'.gz'), 'rt') as stream:
            rows = [json.loads(line) for line in stream]
        armed_at = run['events']['armed_at']
        assert run['events']['bench_release'] == armed_at+1
        assert all(row['bench_held'] and row['fdm_accel_m_s2'] == [0, 0, -9.80665]
                   for row in rows[:armed_at])
        assert not rows[armed_at]['bench_held']
        assert rows[armed_at]['fdm_accel_m_s2'][2] != -9.80665
