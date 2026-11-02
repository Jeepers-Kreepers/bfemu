"""Экспериментальная сборка не изменяет штатный исходник."""
import pytest

from tools import sitl_scheduler_probe as probe


def test_removes_only_verified_pause(tmp_path, monkeypatch):
    path = tmp_path/'main.c'
    text = 'void loop() {\n        delayMicroseconds_real(50); // max rate 20kHz\n}\n'
    path.write_text(text)
    with pytest.raises(probe.PhysicsError):
        probe.remove_sitl_sleep(path)
    assert path.read_text() == text
    monkeypatch.setattr(probe, 'MAIN_HASH', probe.digest(path))
    probe.remove_sitl_sleep(path)
    assert 'delayMicroseconds_real(50)' not in path.read_text()
    assert 'Исследование bfemu' in path.read_text()
    with pytest.raises(probe.PhysicsError):
        probe.remove_sitl_sleep(path)


def test_rejects_preload_before_creating_dir(tmp_path, monkeypatch):
    monkeypatch.setenv('LD_PRELOAD', '/not-used')
    monkeypatch.setattr(probe.os, 'sched_getaffinity', lambda _: {0, 1})
    target = tmp_path/'results'
    assert probe.main(['--output-dir', str(target)]) == 1
    assert not target.exists()


def test_rejects_one_cpu_and_invalid_arguments_before_creating_dir(tmp_path, monkeypatch):
    target = tmp_path/'results'
    monkeypatch.setattr(probe.os, 'sched_getaffinity', lambda _: {0})
    assert probe.main(['--output-dir', str(target)]) == 1
    assert not target.exists()
    with pytest.raises(SystemExit):
        probe.main(['--output-dir', str(target), '--repeats', '0'])
    assert not target.exists()


def test_archived_runs_include_all_failures_and_exact_conditions():
    import gzip
    import json
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]/'docs/research/scheduler-pinned-v1'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['main_sha256'] == probe.MAIN_HASH
    assert rows[0]['cpu_physics'] != rows[0]['cpu_sitl']
    assert rows[-1] == {'type': 'summary', 'attempts': 8, 'not_completed': 3}
    assert sum(row['ok'] for row in rows[1:-1]) == 5
    for row in rows[1:-1]:
        if row['ok']:
            data = json.loads(gzip.decompress((root/(row['physics_file']+'.gz')).read_bytes()))
            assert len(data['samples']) > 1000
            assert row['verified_pid']['values'] == rows[0]['pid']
