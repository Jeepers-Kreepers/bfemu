"""Проверка снятия фактических составляющих отдельной сборки Betaflight."""
import gzip
import json
import math
from pathlib import Path

import pytest

from tools import sitl_terms_probe as probe


def sample(ns, sim=100, armed=1, values=None):
    return probe.RECORD.pack(ns, sim, armed, *(values or list(range(21))))


def test_decode_selects_window_and_actual_axis_without_interpolation():
    data = sample(9_900_000_000) + sample(10_100_000_000) + sample(10_200_000_000, sim=200) + sample(10_300_000_000)
    rows, coverage = probe.decode_trace(data, 10, (.1, .2), 'pitch')
    assert len(rows) == 2
    assert rows[0]['armed'] is True and rows[0]['sim_us'] == 100
    assert [rows[0][key] for key in probe.FIELDS] == list(range(7, 14))
    assert rows[1]['sim_us'] == 200 and coverage['samples'] == coverage['armed_samples'] == 2
    assert math.isclose(coverage['max_gap_ms'], 100)
    assert math.isclose(coverage['first_s'], .1) and math.isclose(coverage['last_s'], .2)


def test_decode_rejects_truncation_bad_order_flags_and_nan():
    good = sample(10_100_000_000)
    nan_values = list(range(21))
    nan_values[20] = float('nan')
    for data in (b'bad', good+good, sample(10_100_000_000, armed=2), sample(10_100_000_000, values=nan_values)):
        with pytest.raises(ValueError):
            probe.decode_trace(data, 10, (0, 1), 'roll')
    with pytest.raises(ValueError, match='Нет отсчётов'):
        probe.decode_trace(good, 20, (0, 1), 'roll')
    for window, axis in (((0, 1), 'bad'), ((1, 0), 'pitch'), ((0, float('inf')), 'roll')):
        with pytest.raises(ValueError):
            probe.decode_trace(good, 10, window, axis)


def test_core_only_instruments_verified_source_once(tmp_path, monkeypatch):
    path = tmp_path/'core.c'
    text = ('FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)\n{\n'
            '    subTaskPidController(currentTimeUs);\n    subTaskMotorUpdate(currentTimeUs);\n}\n')
    path.write_text(text)
    with pytest.raises(probe.PhysicsError):
        probe.instrument_core(path)
    assert path.read_text() == text
    monkeypatch.setattr(probe, 'CORE_HASH', probe.digest(path))
    probe.instrument_core(path)
    changed = path.read_text()
    assert changed.count('#include "bfemu_terms_probe.inc"') == 1
    assert 'subTaskPidController(currentTimeUs);\n    bfemu_terms_probe(currentTimeUs);\n    subTaskMotorUpdate(currentTimeUs);' in changed
    with pytest.raises(probe.PhysicsError):
        probe.instrument_core(path)


def test_archive_contains_verified_actual_pid_records_and_failures():
    root = Path(__file__).resolve().parents[1]/'docs/research/pid-terms-direct'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['probe_source_sha256'] == probe.digest(probe.SOURCE_FILE)
    assert rows[-1] == {'type': 'summary', 'attempts': 8, 'not_completed': 2}
    assert sum(row['ok'] for row in rows[1:-1]) == 6
    instrumented = [row for row in rows[1:-1] if row['instrumented']]
    assert len(instrumented) == 4 and all(row['ok'] for row in instrumented)
    for row in instrumented:
        data = gzip.decompress((root/(row['file']+'.gz')).read_bytes())
        samples, coverage = probe.decode_trace(data, row['start_monotonic'], row['command_window_s'], 'pitch')
        assert coverage == row['coverage']
        assert coverage['armed_samples'] == coverage['samples']
        assert row['verified_pid']['values'] == row['pid']
        assert any(abs(s['p']) > 1 for s in samples) and any(abs(s['d']) > 1 for s in samples)
        assert all(abs(s['sum'] - (s['p']+s['i']+s['d']+s['f'])) < .01 for s in samples)


def test_rejects_external_trace_and_invalid_arguments_before_creating_dir(tmp_path, monkeypatch):
    target = tmp_path/'results'
    monkeypatch.setenv('BFEMU_TERMS_TRACE', '/not-used')
    assert probe.main(['--output-dir', str(target)]) == 1
    assert not target.exists()
    monkeypatch.delenv('BFEMU_TERMS_TRACE')
    for args in (['--before', '251', '84', '46'], ['--repeats', '0']):
        with pytest.raises(SystemExit):
            probe.main(['--output-dir', str(target), *args])
        assert not target.exists()


def test_archive_saves_physics_and_pid_on_the_same_host_clock():
    import bisect

    root = Path(__file__).resolve().parents[1]/'docs/research/pid-terms-physics'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[-1] == {'type': 'summary', 'attempts': 4, 'not_completed': 1}
    assert sum(row['ok'] for row in rows[1:-1]) == 3
    row = next(row for row in rows if row.get('coverage'))
    trace = gzip.decompress((root/(row['file']+'.gz')).read_bytes())
    samples, coverage = probe.decode_trace(trace, row['start_monotonic'], row['command_window_s'], 'pitch')
    physics = json.loads(gzip.decompress((root/(row['physics_file']+'.gz')).read_bytes()))
    assert coverage == row['coverage'] and physics['command_window_s'] == row['command_window_s']
    times = [s['t'] for s in physics['samples']]
    assert all(min(abs(times[j]-sample['t']) for j in
                   range(max(0, bisect.bisect_left(times, sample['t'])-1),
                         min(len(times), bisect.bisect_left(times, sample['t'])+1))) < .005 for sample in samples)
    # Временная близость не устанавливает, какой FDM был использован PID.
