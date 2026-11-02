"""Принудительный коэффициент часов используется только в отдельной копии SITL."""
import pytest

from tools import sitl_clock_probe as probe


def test_modified_source_is_hash_checked(tmp_path, monkeypatch):
    p = tmp_path/'sitl.c'
    text = 'before\n        simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);\nafter\n'
    p.write_text(text)
    with pytest.raises(probe.PhysicsError):
        probe.use_host_clock(p)
    assert p.read_text() == text
    monkeypatch.setattr(probe, 'SITL_HASH', probe.digest(p))
    probe.use_host_clock(p)
    assert p.read_text().replace('        simRate = 1.0; // Исследование bfemu: часы хоста, не lockstep.',
                                 '        simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);') == text
    with pytest.raises(probe.PhysicsError):
        probe.use_host_clock(p)


def test_rejects_invalid_parameters_and_external_preload(tmp_path, monkeypatch):
    target = tmp_path/'output'
    for opts in (['--pid', '49', '84', '251'], ['--repeats', '0'], ['--jobs', '33']):
        with pytest.raises(SystemExit):
            probe.main(['--output-dir', str(target), *opts])
        assert not target.exists()
    monkeypatch.setenv('LD_PRELOAD', '/external.so')
    assert probe.main(['--output-dir', str(target)]) == 1
    assert not target.exists()


def test_archived_series_includes_every_failure():
    import gzip
    import json
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]/'docs/research/sitl-host-clock-v1'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['sitl_sha256'] == probe.SITL_HASH
    assert rows[-1] == {'type': 'summary', 'attempts': 16, 'not_completed': 2}
    for row in rows[1:-1]:
        if row['ok']:
            assert row['verified_pid']['values'] == rows[0]['pid']
            data = json.loads(gzip.decompress((root/(row['physics_file']+'.gz')).read_bytes()))
            assert len(data['samples']) > 1000
        else:
            assert row['error']
