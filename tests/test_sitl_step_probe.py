"""Проверка границ опытного подтверждения кадров, без подмены PID."""
import io
import json
from pathlib import Path
import socket

import pytest

from bfemu.aircraft_sitl import theoretical_curves
from bfemu.flight_actuators import output_mapping
from tools import sitl_step_probe as probe


def test_replacements_require_exactly_one_match():
    assert probe.replace_unique('aXb', 'X', 'Y') == 'aYb'
    for s in ('ab', 'aXXb'):
        with pytest.raises(probe.PhysicsError):
            probe.replace_unique(s, 'X', 'Y')


def test_sources_must_match_before_writing(tmp_path):
    for name in ('drivers/accgyro/accgyro_virtual.c', 'fc/core.c', 'target/SITL/sitl.c'):
        p = tmp_path/'src/main'/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('unchanged')
    with pytest.raises(probe.PhysicsError):
        probe.patch_sources(tmp_path)
    assert (tmp_path/'src/main/fc/core.c').read_text() == 'unchanged'


def test_ack_sequence_mask_and_payload():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        receiver.bind(('127.0.0.1', 0))
        mapping = (3, 0, 1, 2)
        def send(seq, mask, motors=(.1, .2, .3, .4)):
            sender.sendto(probe.ACK.pack(seq, mask, *motors), receiver.getsockname())
        send(1, 15)
        assert probe.read_ack(receiver, 1, mapping, .1) == pytest.approx((.4, .1, .2, .3))
        send(3, 15)
        with pytest.raises(probe.PhysicsError, match='последовательность'):
            probe.read_ack(receiver, 2, mapping, .1)
        send(2, 7)
        with pytest.raises(probe.PhysicsError, match='маска'):
            probe.read_ack(receiver, 2, mapping, .1)
        send(2, 15, (float('nan'), 0, 0, 0))
        with pytest.raises(probe.PhysicsError, match='Некорректная команда'):
            probe.read_ack(receiver, 2, mapping, .1)
        with pytest.raises(probe.PhysicsError, match='не подтвердил'):
            probe.read_ack(receiver, 2, mapping, .001)


def test_loop_waits_for_each_ack_and_advances_only_recorded_commands(monkeypatch):
    profile = json.loads((Path(__file__).resolve().parents[1]/'bfemu/data/aircraft_build03.json').read_text())
    curves = theoretical_curves(profile)
    times = iter((0., .004, .008, .012))
    packets, expected = [], []
    class Sender:
        def sendto(self, data, address):
            packets.append((data, address))
    def ack(sock, seq, mapping, timeout):
        assert mapping == output_mapping(profile)
        expected.append(seq)
        return (.1*seq,)*4
    monkeypatch.setattr(probe, 'read_ack', ack)
    out = io.StringIO()
    count = probe.run_step_loop(profile, curves, None, Sender(), out, ack_socket=None, duration=.01,
                                period=.004, max_gap=.5, pressure_pa=101325, clock=lambda: next(times))
    rows = [json.loads(x) for x in out.getvalue().splitlines()]
    assert count == 2 and expected == [1, 2] and len(packets) == 3
    assert rows[1]['applied_commands'] == [0]*4
    assert rows[1]['next_commands'] == [.1]*4
    assert rows[2]['applied_commands'] == [.1]*4
    assert rows[2]['next_commands'] == [.2]*4


def test_loop_rejects_late_sensor_interval_instead_of_hiding_it(monkeypatch):
    profile = json.loads((Path(__file__).resolve().parents[1]/'bfemu/data/aircraft_build03.json').read_text())
    times = iter((0., .004, .029))
    class Sender:
        def sendto(self, data, address):
            pass
    monkeypatch.setattr(probe, 'read_ack', lambda *a: (0., 0., 0., 0.))
    out, stats = io.StringIO(), {}
    with pytest.raises(probe.PhysicsError, match=r'25.000 мс после 1 подтверждённых кадров'):
        probe.run_step_loop(profile, theoretical_curves(profile), None, Sender(), out, ack_socket=None,
                            duration=.1, period=.004, max_gap=.5, pressure_pa=101325,
                            clock=lambda: next(times), stats=stats)
    assert stats['acknowledged'] == 1
    assert stats['max_sensor_interval_ms'] == pytest.approx(25.)
    assert len(out.getvalue().splitlines()) == 2


def test_archived_step_failures_are_not_hidden():
    import gzip

    base = Path(__file__).resolve().parents[1]/'docs/research'
    rows = [json.loads(x) for x in (base/'sitl-step-v2/summary.jsonl').read_text().splitlines()]
    assert rows[-1] == {'type': 'summary', 'attempts': 6, 'not_completed': 3}
    assert all(not x['ok'] and '0–0,02 с' in x['error'] for x in rows[1:-1] if x['step'])
    assert all(x['ok'] for x in rows[1:-1] if not x['step'])
    timing = [json.loads(x) for x in (base/'sitl-step-timing-v2/summary.jsonl').read_text().splitlines()]
    step = next(x for x in timing if x.get('step'))
    assert not step['ok'] and step['step_diagnostics']['acknowledged'] == 369
    raw = gzip.decompress((base/'sitl-step-timing-v2'/(step['raw_physics_file']+'.gz')).read_bytes())
    assert len(raw.splitlines()) == step['step_diagnostics']['acknowledged']+1


def test_diagnostic_source_inserts_only_observation(monkeypatch, tmp_path):
    from tools import sitl_step_diagnostic as diag

    monkeypatch.setattr(probe, 'patch_sources', lambda source: None)
    gyro = tmp_path/'src/main/drivers/accgyro/accgyro_virtual.c'
    target = tmp_path/'src/main/target/SITL/sitl.c'
    include = target.parent/'bfemu_step_probe.inc'
    gyro.parent.mkdir(parents=True)
    target.parent.mkdir(parents=True)
    gyro.write_text('    bfemu_gyro_sequence++;\n'
                    '    __atomic_store_n(&bfemu_gyro_consumed_sequence, bfemu_gyro_sequence, __ATOMIC_RELEASE);')
    target.write_text('    const uint64_t realtime_now = micros64_real();')
    include.write_text('    if (!consumed || consumed == last_ack) return;')
    diag.add_diagnostics(tmp_path)
    assert '[STEP] SET' in gyro.read_text() and '[STEP] READ' in gyro.read_text()
    assert '[STEP] FDM' in target.read_text() and '[STEP] PID' in include.read_text()


def test_archived_safe_mutex_variant_records_partial_frames_and_hash():
    import gzip
    import hashlib

    root = Path(__file__).resolve().parents[1]/'docs/research/sitl-step-v3'
    rows = [json.loads(x) for x in (root/'summary.jsonl').read_text().splitlines()]
    assert rows[0]['driver_source_sha256'] == hashlib.sha256(Path(probe.__file__).read_bytes()).hexdigest()
    assert rows[-1] == {'type': 'summary', 'attempts': 4, 'not_completed': 2}
    for row in rows[1:-1]:
        if row['step']:
            assert not row['ok'] and row['step_diagnostics']['max_sensor_interval_ms'] >= 20
            assert row['step_diagnostics']['last_ack_wait_ms'] >= 20
            data = gzip.decompress((root/(row['raw_physics_file']+'.gz')).read_bytes())
            assert len(data.splitlines()) == row['step_diagnostics']['acknowledged']+1
        else:
            assert row['ok']


def test_archived_startup_diagnostic_is_available():
    root = Path(__file__).resolve().parents[1]/'docs/research/sitl-step-diagnostic-v2'
    log = (root/'sitl-stderr.txt').read_text()
    assert '[STEP] FDM 0.00000000' in log
    assert '[STEP] SET 1' in log and '[STEP] READ' not in log
