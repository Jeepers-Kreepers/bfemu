"""Проверка агрегирования непосредственных измерений задачи PID."""
import platform

import pytest

from tools import sitl_pid_cycle_probe as probe


def sample(start_ns, end_ns, count, *, sim_us=100_000, host_min=100_000, host_max=200_000,
           sim_min=100, sim_max=200):
    return probe.RECORD.pack(start_ns, end_ns, count, host_min, host_max, sim_us, sim_min, sim_max)


def test_measurement_counts_complete_contiguous_intervals_only():
    records = (sample(9_900_000_000, 10_000_000_000, 99) +
               sample(10_000_000_000, 10_100_000_000, 500) +
               sample(10_100_000_000, 10_200_000_000, 700) +
               sample(10_200_000_000, 10_300_000_000, 99))
    result = probe.analyze_trace(records, 10, (0, .2))
    assert result['bins'] == 2 and result['duration_s'] == .2 and result['pid_task_entries'] == 1200
    assert result['host_frequency_hz'] == 6000 and result['simulated_frequency_hz'] == 6000
    assert result['sim_clock_valid'] is True
    assert result['min_host_interval_us'] == 100 and result['max_host_interval_us'] == 200


def test_measurement_rejects_corrupt_missing_and_noncontiguous_records():
    good = sample(10_000_000_000, 10_100_000_000, 500)
    broken = good + sample(10_101_000_000, 10_200_000_000, 500)
    for data, msg in ((b'1', 'оборвана'), (b'', 'Нет полных'), (broken, 'не непрерывны'),
                      (sample(10_000_000_000, 10_100_000_000, 0), 'некорректный')):
        with pytest.raises(ValueError, match=msg):
            probe.analyze_trace(data, 10, (0, .2))


def test_instrumentation_rejects_unrecognized_core(tmp_path):
    path = tmp_path/'core.c'
    path.write_text('#include <stdbool.h>\nsubTaskPidController();\n')
    before = path.read_bytes()
    with pytest.raises(probe.PhysicsError, match='автоматическое внедрение запрещено'):
        probe.instrument_core(path)
    assert path.read_bytes() == before


def test_rejects_existing_instrumentation_environment_before_output(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(probe.sys, 'platform', 'linux')
    monkeypatch.setenv('BFEMU_PIDTRACE', '/path')
    target = tmp_path/'new'
    assert probe.main(['--output-dir', str(target), '--repeats', '1']) == 1
    assert not target.exists() and 'BFEMU_PIDTRACE' in capsys.readouterr().err


def test_rejects_invalid_arguments_before_output(tmp_path, monkeypatch):
    monkeypatch.setattr(probe.sys, 'platform', 'linux')
    monkeypatch.setattr(platform, 'system', lambda: 'Linux')
    target = tmp_path/'new'
    for opts in (['--denominators', '0'], ['--denominators', '1', '1'], ['--repeats', '11'],
                 ['--pid', '45', '80', '251']):
        with pytest.raises(SystemExit):
            probe.main(['--output-dir', str(target), *opts])
        assert not target.exists()


def test_discontinuous_sim_clock_is_not_reported_as_a_frequency():
    data = sample(10_000_000_000, 10_100_000_000, 50, sim_us=4_294_967_000, sim_max=4_294_967_000)
    result = probe.analyze_trace(data, 10, (0, .2))
    assert result['simulated_frequency_hz'] is None and not result['sim_clock_valid']
    assert result['host_frequency_hz'] == 500


def test_instrumentation_inserts_only_two_expected_lines(tmp_path, monkeypatch):
    path = tmp_path/'core.c'
    path.write_text('#include <stdbool.h>\nvoid loop() {\n    subTaskRcCommand(currentTimeUs);\n'
                    '    subTaskPidController(currentTimeUs);\n}\n')
    monkeypatch.setattr(probe, 'CORE_HASH', probe.digest(path))
    probe.instrument_core(path)
    text = path.read_text()
    assert text.count('#include "bfemu_pid_probe.inc"') == 1
    assert 'subTaskRcCommand(currentTimeUs);\n    bfemu_pid_probe(currentTimeUs);\n    subTaskPidController(currentTimeUs);' in text
    with pytest.raises(probe.PhysicsError):
        probe.instrument_core(path)


def test_archived_measurements_reproduce_all_direct_counts():
    import json
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]/'docs'/'research'/'pid-cycle-direct'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['instrument_source_hash'] == probe.digest(probe.SOURCE_FILE)
    assert len(rows[1:]) == 16 and all(row['ok'] for row in rows[1:])
    instrumented = [row for row in rows[1:] if row['trace']]
    assert len(instrumented) == 8
    for row in instrumented:
        data = (root/row['file']).read_bytes()
        actual = probe.analyze_trace(data, row['start_monotonic'], row['command_window_s'])
        assert actual.pop('sim_clock_valid') is True
        assert actual == row['measurement']
        assert row['verified_denom'] == row['denom']


@pytest.mark.skipif(__import__('os').environ.get('BFEMU_TEST_REAL_SITL') != '1', reason='реальный SITL — opt-in')
def test_real_pid_cycle_probe_builds_separately_and_measures(tmp_path):
    import json
    if not probe.aircraft_sitl.sitl_available():
        pytest.skip('Требуется заранее собранный SITL.')
    target = tmp_path/'results'
    assert probe.main(['--output-dir', str(target), '--repeats', '1', '--denominators', '1']) == 0
    rows = [json.loads(line) for line in (target/'summary.jsonl').read_text().splitlines()]
    assert rows[-1] == {'type': 'summary', 'attempts': 2, 'not_completed': 0}
    result = next(row for row in rows if row.get('instrumented'))
    assert result['verified_denom'] == 1 and result['measurement']['pid_task_entries'] > 0
    assert result['measurement']['host_frequency_hz'] > 0
    assert (target/result['file']).is_file()
