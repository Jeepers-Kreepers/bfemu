"""Исследовательский декодер снимков не должен выдумывать PID-циклы."""
import math
import platform

import pytest

from tools import sitl_mutex_probe as probe


def record(*, t=10.5, mask=15, packet=(.2, .3, .4, .1), d=30):
    return probe.RECORD.pack(t, 1., 4, mask, *packet, 1100, 1200, 1300, 1400,
                             10, 20, d, 40, 100, 1000, 40)


def test_analysis_checks_only_window_and_uses_correct_motor_order():
    data = record(t=9) + record() + record(t=10.7, mask=7, packet=(.8, .3, .4, .1), d=-50)
    result = probe.analyze_trace(data, 10, (0, 1))
    assert result['active_sends'] == 2
    assert result['write_count_and_success_mask'] == {'4:0111': 1, '4:1111': 1}
    assert result['different_from_current_motor_gt_0_002'] == 1
    assert result['max_abs_difference'] == pytest.approx(.6)
    assert result['median_abs_roll_d_internal'] == 40
    assert result['idle_pulse_values'] == [1000]
    assert result['motor_update_attempts_per_second'] == pytest.approx(200)
    assert result['median_attempts_per_packet'] == 40


def test_analysis_rejects_missing_truncated_and_nonfinite_data():
    for data, text in ((b'', 'нет записанных'), (record()[:-1], 'Оборванный'),
                       (record(d=math.nan), 'недопустимые')):
        with pytest.raises(ValueError, match=text):
            probe.analyze_trace(data, 10, (0, 1))


def test_probe_refuses_unknown_binary_without_output_dir(tmp_path, monkeypatch, capsys):
    binary = tmp_path/'sitl'
    binary.write_bytes(b'not the instrumented ELF')
    output = tmp_path/'new'
    monkeypatch.setattr(probe.sys, 'platform', 'linux')
    monkeypatch.setattr(platform, 'machine', lambda: 'x86_64')
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    monkeypatch.setattr(probe.aircraft_sitl, 'sitl_available', lambda: True)
    monkeypatch.setattr(probe.aircraft_sitl, 'BINARY', binary)
    assert probe.main(['--output-dir', str(output), '--repeats', '1']) == 1
    assert not output.exists() and 'Хеш ELF не совпадает' in capsys.readouterr().err


def test_probe_refuses_preload_without_output_dir(tmp_path, monkeypatch, capsys):
    output = tmp_path/'new'
    monkeypatch.setattr(probe.sys, 'platform', 'linux')
    monkeypatch.setattr(platform, 'machine', lambda: 'x86_64')
    monkeypatch.setenv('LD_PRELOAD', '/some/library')
    assert probe.main(['--output-dir', str(output), '--repeats', '1']) == 1
    assert not output.exists() and 'LD_PRELOAD' in capsys.readouterr().err


def test_can_decode_previous_trace_format():
    old = probe.RECORD_V1.pack(10.5, 1., 4, 15, .2, .3, .4, .1,
                               1100, 1200, 1300, 1400, 10, 20, 30, 40, 100, 1000)
    result = probe.analyze_trace(old, 10, (0, 1), format_version=1)
    assert result['active_sends'] == 1 and result['different_from_current_motor_gt_0_002'] == 0
    assert 'motor_update_attempts_per_second' not in result


def test_archived_trace_reproduces_motor_masks_and_diagnostics():
    import gzip
    import json
    from pathlib import Path
    folder = Path(__file__).resolve().parents[1]/'docs'/'research'/'mutex-probe-v1'
    rows = [json.loads(line) for line in (folder/'summary.jsonl').read_text().splitlines()]
    traced = [row for row in rows if row['trace'] and row['ok']]
    assert len(traced) == 4
    total = 0
    for row in traced:
        raw = gzip.decompress((folder/row['file']).read_bytes())
        data = probe.analyze_trace(raw, row['start_monotonic'], row['command_window_s'], format_version=1)
        assert data['active_sends'] == row['trace_stats']['active_sends']
        assert data['different_from_current_motor_gt_0_002'] == 0
        assert data['write_count_and_success_mask'] == {'4:1111': data['active_sends']}
        total += data['active_sends']
    assert total == 1069


def test_archived_v2_reproduces_observed_partial_write():
    import gzip
    import json
    from pathlib import Path
    folder = Path(__file__).resolve().parents[1]/'docs'/'research'/'mutex-probe-v2'
    rows = [json.loads(line) for line in (folder/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['trace_format'] == 2
    traced = [row for row in rows if row.get('trace') and row['ok']]
    assert len(traced) == 4
    counts = {}
    for row in traced:
        raw = gzip.decompress((folder/row['file']).read_bytes())
        data = probe.analyze_trace(raw, row['start_monotonic'], row['command_window_s'])
        assert data == row['trace_stats']
        for mask, n in data['write_count_and_success_mask'].items():
            counts[mask] = counts.get(mask, 0)+n
        assert data['different_from_current_motor_gt_0_002'] == 0
    assert counts == {'4:1111': 1055, '4:1110': 1}


@pytest.mark.skipif(__import__('os').environ.get('BFEMU_TEST_REAL_SITL') != '1', reason='реальный SITL — opt-in')
def test_real_probe_creates_control_and_trace(tmp_path):
    import hashlib
    from pathlib import Path
    binary = Path(probe.aircraft_sitl.BINARY)
    if not binary.is_file() or hashlib.sha256(binary.read_bytes()).hexdigest() != probe.ELF_HASH:
        pytest.skip('Требуется именно сборка ELF, указанная в исследовательском инструменте.')
    output = tmp_path/'probe'
    assert probe.main(['--output-dir', str(output), '--repeats', '1']) == 0
    import json
    rows = [json.loads(line) for line in (output/'summary.jsonl').read_text().splitlines()]
    assert rows[-1] == {'type': 'summary', 'attempts': 2, 'not_completed': 0}
    traced = rows[2]
    assert traced['trace'] and traced['trace_stats']['active_sends'] > 0
    assert traced['trace_stats']['motor_update_attempts_per_second'] > 0
    assert (output/traced['file']).is_file()
