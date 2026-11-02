"""Исследовательский съём виртуального гироскопа не меняет штатную сборку."""
import pytest

from tools import sitl_gyro_probe as probe


def test_instrumentation_is_restricted_to_known_source(tmp_path, monkeypatch):
    p = tmp_path/'accgyro_virtual.c'
    text = ('#ifdef USE_VIRTUAL_GYRO\n    virtualGyroADC[X] = x;\n'
            '    if (gyro->dataReady == false) {\n        gyroDevUnLock(gyro);\n'
            '    gyro->dataReady = false;\n\n    gyro->gyroADCRaw[X]')
    p.write_text(text)
    with pytest.raises(probe.PhysicsError):
        probe.instrument_gyro(p)
    assert p.read_text() == text
    monkeypatch.setattr(probe, 'GYRO_HASH', probe.digest(p))
    probe.instrument_gyro(p)
    result = p.read_text()
    assert result.count('bfemu_gyro_set(') == 1
    assert result.count('bfemu_gyro_read(false)') == 1
    assert result.count('bfemu_gyro_read(true)') == 1
    with pytest.raises(probe.PhysicsError):
        probe.instrument_gyro(p)


def records():
    return b''.join(probe.RECORD.pack(1_000_000_000+i*4_000_000, i+1, int(i != 4), i%3,
                                      int(i == 4), -1, 2, 3, 0) for i in range(32))


def test_decode_counters_and_window():
    r = probe.decode_trace(records(), 1, [0.008, .024])
    assert r['frames'] == 5
    assert r['reads_ok'] == 4 and r['overwritten_pending'] == 1
    assert r['intervals_without_successful_read'] == 1 and r['intervals_multiple_reads'] == 0
    assert r['max_gap_ms'] == 4


def test_decode_rejects_incomplete_or_corrupted_data():
    with pytest.raises(ValueError):
        probe.decode_trace(records()[:-1], 1, [0, .1])
    damaged = bytearray(records())
    damaged[8:12] = b'\x05\x00\x00\x00'
    with pytest.raises(ValueError):
        probe.decode_trace(damaged, 1, [0, .1])
    with pytest.raises(ValueError):
        probe.decode_trace(records(), 1, [.3, .4])


def test_rejects_preload_before_creating_output(tmp_path, monkeypatch):
    monkeypatch.setenv('LD_PRELOAD', '/external.so')
    out = tmp_path/'output'
    assert probe.main(['--output-dir', str(out)]) == 1
    assert not out.exists()


def test_archived_observations_match_summary():
    import gzip
    import json
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]/'docs/research/sitl-gyro-v1'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[-1] == {'type': 'summary', 'attempts': 8, 'not_completed': 0}
    for row in rows[1:-1]:
        assert row['ok'] and row['verified_pid']['values'] == rows[0]['pid']
        if row['instrumented']:
            raw = gzip.decompress((root/(row['trace_file']+'.gz')).read_bytes())
            stats = probe.decode_trace(raw, row['start_monotonic'], row['command_window_s'])
            assert all(stats[key] == value for key, value in row['coverage'].items())
            assert stats['intervals_without_read_calls'] == stats['overwritten_pending']
            assert stats['missing_read_without_pending_flag'] == 0
