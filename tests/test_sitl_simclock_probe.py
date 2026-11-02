"""Проверка протокола исследовательских часов без подмены прошивки."""
import json
from pathlib import Path
import struct

import pytest

from tools import sitl_simclock_probe as probe


def test_decode_checks_sequence_gyro_pid_motor_mask_and_clock():
    def msg(seq=3, read=3, mask=15, pid=32, armed=1, clock=12000, rc=1650., sp=25.):
        return probe.OUTPUT.pack(seq, read, mask, pid, armed, 0, clock, .1, .2, .3, .4, 1, 125, 11000, 12000, rc, sp)
    record = probe.decode_response(msg(), 3, (3, 0, 1, 2), 8000)
    assert record['motors'] == pytest.approx((.4, .1, .2, .3))
    assert record['pid_cycles'] == 32 and record['setpoint_axis_dps'] == 25
    for data in (msg(seq=2), msg(read=2), msg(mask=7), msg(pid=0), msg(armed=2), msg(clock=12001),
                 msg(rc=float('nan')), msg(sp=float('nan')), msg()[:-1]):
        with pytest.raises(probe.PhysicsError):
            probe.decode_response(data, 3, (3, 0, 1, 2), 8000)


def test_run_uses_one_rc_command_per_frame_and_disarms(monkeypatch):
    profile = json.loads((Path(__file__).resolve().parents[1]/'bfemu/data/aircraft_build03.json').read_text())
    current = {}
    class FakeSocket:
        def __init__(self, *args):
            pass
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def bind(self, addr):
            assert addr == ('127.0.0.1', 9011)
        def sendto(self, payload, addr):
            assert addr == ('127.0.0.1', 9010)
            seq, axis = probe.INPUT_HEADER.unpack_from(payload)
            assert axis in (0, 1, 2) and len(payload) == 192
            current['seq'] = seq
            current['axis'] = axis
            current['channels'] = probe.RC.unpack(payload[-probe.RC.size:])[1:]
        def recvfrom(self, size):
            seq = current['seq']
            armed = int(current['channels'][4] == 1900)
            flags = 4 if seq == 1 else 0
            return (probe.OUTPUT.pack(seq, seq, 15, 32, armed, flags, 1000+seq*4000,
                                      0, 0, 0, 0, 1, 125, 0, 0, current['channels'][{0: 0, 1: 1, 2: 3}[current['axis']]], 0),
                    ('127.0.0.1', 9011))
    monkeypatch.setattr(probe.socket, 'socket', FakeSocket)
    monkeypatch.setattr(probe.select, 'select', lambda r, w, x, timeout: (r, [], []))
    captured = []
    rows, events = probe.run_frames(profile, 1000, scenario='flight', on_frame=captured.append)
    assert len(rows) == len(captured) == events['disarmed_at']
    assert events['arm_request'] == events['armed_at'] == 3
    assert events['command_start'] == 154 and events['command_end'] == 454
    assert events['disarm_request'] == events['disarmed_at'] == 579
    assert sum(row['channels'][1] == 1650 for row in rows) == 300
    assert rows[events['command_start']-1]['channels'][2] == 1300
    assert rows[-1]['channels'][4] == 1000 and not rows[-1]['armed']
    yaw_rows, yaw_events = probe.run_frames(profile, 1000, scenario='flight', axis='yaw')
    assert yaw_events['command_start'] == events['command_start']
    assert sum(row['channels'][3] == 1650 for row in yaw_rows) == 300
    assert all(row['rc_axis'] == row['channels'][3] for row in yaw_rows)
    roll_rows, _ = probe.run_frames(profile, 1000, scenario='flight', axis='roll')
    assert sum(row['channels'][0] == 1650 for row in roll_rows) == 300
    assert all(row['rc_axis'] == row['channels'][0] for row in roll_rows)
    # Это поддельный ответ в модульном тесте, а не результат полёта SITL.


def test_invalid_parameters_and_source_fail_before_changes(tmp_path):
    with pytest.raises(SystemExit):
        probe.main(['--output-dir', str(tmp_path/'out'), '--pid', '251', '84', '46'])
    assert not (tmp_path/'out').exists()
    for name in ('main.c', 'scheduler/scheduler.c'):
        p = tmp_path/'src/main'/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('unknown')
    with pytest.raises(probe.PhysicsError):
        probe.patch_sources(tmp_path)
    assert (tmp_path/'src/main/main.c').read_text() == 'unknown'


def test_protocol_frame_sizes_are_exact():
    assert probe.INPUT_HEADER.size+18*struct.calcsize('=d')+probe.RC.size == 192
    assert probe.OUTPUT.size == 72
