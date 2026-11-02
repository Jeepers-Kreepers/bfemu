"""Безопасное включение исследовательского макроса только в проверенной копии."""
import pytest

from tools import sitl_sync_probe as probe


def test_enable_sync_changes_only_the_intended_macro(tmp_path, monkeypatch):
    target = tmp_path/'target.h'
    text = 'first\n//#define SIMULATOR_GYROPID_SYNC\nlast\n'
    target.write_text(text)
    with pytest.raises(probe.PhysicsError):
        probe.enable_sync(target)
    assert target.read_text() == text
    monkeypatch.setattr(probe, 'TARGET_HASH', probe.digest(target))
    probe.enable_sync(target)
    assert target.read_text() == text.replace('//#define SIMULATOR_GYROPID_SYNC', '#define SIMULATOR_GYROPID_SYNC')
    with pytest.raises(probe.PhysicsError):
        probe.enable_sync(target)


def test_invalid_parameters_do_not_create_directory(tmp_path):
    target = tmp_path/'results'
    for opts in (['--pid', '49', '84', '251'], ['--repeats', '0'], ['--jobs', '33']):
        with pytest.raises(SystemExit):
            probe.main(['--output-dir', str(target), *opts])
        assert not target.exists()


def test_rejects_ld_preload_before_creating_directory(tmp_path, monkeypatch):
    target = tmp_path/'results'
    monkeypatch.setenv('LD_PRELOAD', '/external.so')
    assert probe.main(['--output-dir', str(target)]) == 1
    assert not target.exists()


def test_archive_preserves_successful_and_failed_trials():
    import gzip
    import json
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]/'docs/research/pid-sync-macro'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['target_sha256'] == probe.TARGET_HASH
    assert rows[-1] == {'type': 'summary', 'attempts': 8, 'not_completed': 5}
    assert [(row['sync'], row['ok']) for row in rows[1:-1]] == [
        (False, False), (True, False), (True, False), (False, True),
        (False, True), (True, False), (True, False), (False, True)]
    for row in rows[1:-1]:
        if row['ok']:
            data = json.loads(gzip.decompress((root/(row['physics_file']+'.gz')).read_bytes()))
            assert len(data['samples']) > 1000 and len(data['command_window_s']) == 2
            assert row['verified_pid']['values'] == rows[0]['pid']


def test_archive_includes_flags_without_claiming_cause():
    import json
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]/'docs/research/pid-sync-flags'
    rows = [json.loads(line) for line in (root/'summary.jsonl').read_text().splitlines()]
    sync = [row for row in rows[1:-1] if row['sync']]
    assert len(sync) == 2 and all(not row['ok'] for row in sync)
    assert all('0x020001A0' in row['error'] and 'RUNAWAY_TAKEOFF' in row['error'] and
               'не доказывают причину' in row['error'] for row in sync)
    assert rows[-1] == {'type': 'summary', 'attempts': 4, 'not_completed': 3}
