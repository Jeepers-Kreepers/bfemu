"""Проверка протокола исследовательских часов без подмены прошивки."""
import json
from pathlib import Path
import struct

import pytest

from tools import sitl_simclock_probe as probe


def test_decode_checks_sequence_gyro_pid_motor_mask_and_clock():
    def msg(seq=3, read=3, mask=15, pid=32, armed=1, clock=12000, rc=1650., sp=25.):
        return probe.OUTPUT.pack(seq, read, mask, pid, armed, 0, clock, .1, .2, .3, .4, 1, 125, 11000, 12000, 1500, rc, 1500, 0, sp, 0, *[0.] * 18, 125, 125, 125, 0.000125, 1., 2., 3.)
    record = probe.decode_response(msg(), 3, (3, 0, 1, 2), 8000)
    assert record['motors'] == pytest.approx((.4, .1, .2, .3))
    assert record['pid_cycles'] == 32 and record['setpoint_axes_dps'] == [0, 25, 0]
    assert record['gyro_axes_dps'] == [0., 0., 0.]
    assert record['pid_loop_us'] == record['gyro_loop_us'] == record['gyro_sample_us'] == 125
    assert record['pid_dt_s'] == pytest.approx(.000125) and record['pid_ki'] == [1., 2., 3.]
    assert record['pid_axes'][1] == {'p': 0., 'i': 0., 'd': 0., 'f': 0., 'sum': 0.}
    for data in (msg(seq=2), msg(read=2), msg(mask=7), msg(pid=0), msg(armed=2), msg(clock=12001),
                 msg(rc=float('nan')), msg(sp=float('nan')), msg()[:-1]):
        with pytest.raises(probe.PhysicsError):
            probe.decode_response(data, 3, (3, 0, 1, 2), 8000)
    # В новых полях тоже отклоняются неконечные значения.
    for offset in (88, 100, 172, 176):
        corrupt = bytearray(msg())
        struct.pack_into('=f', corrupt, offset, float('nan'))
        with pytest.raises(probe.PhysicsError):
            probe.decode_response(corrupt, 3, (3, 0, 1, 2), 8000)


@pytest.mark.parametrize('axis,channel', [('roll', 0), ('pitch', 1), ('yaw', 3)])
def test_run_uses_one_rc_command_per_frame_and_disarms(monkeypatch, axis, channel):
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
            seq, reserved = probe.INPUT_HEADER.unpack_from(payload)
            assert reserved == 0 and len(payload) == 192
            current['seq'] = seq
            current['channels'] = probe.RC.unpack(payload[-probe.RC.size:])[1:]
        def recvfrom(self, size):
            seq = current['seq']
            armed = int(current['channels'][4] == 1900)
            flags = 4 if seq == 1 else 0
            return (probe.OUTPUT.pack(seq, seq, 15, 32, armed, flags, 1000+seq*4000,
                                      0, 0, 0, 0, 1, 125, 0, 0, current['channels'][0], current['channels'][1],
                                      current['channels'][3], 0, 0, 0, *[0.] * 18, 125, 125, 125, .000125, 1., 2., 3.),
                    ('127.0.0.1', 9011))
    monkeypatch.setattr(probe.socket, 'socket', FakeSocket)
    monkeypatch.setattr(probe.select, 'select', lambda r, w, x, timeout: (r, [], []))
    captured = []
    rows, events = probe.run_frames(profile, 1000, scenario='flight', on_frame=captured.append, axis=axis)
    assert len(rows) == len(captured) == events['disarmed_at']
    assert events['arm_request'] == events['armed_at'] == 3
    assert events['command_start'] == 154 and events['command_end'] == 454
    assert events['disarm_request'] == events['disarmed_at'] == 579
    assert sum(row['channels'][channel] == 1650 for row in rows) == 300
    assert rows[events['command_start']-1]['channels'][2] == 1300
    assert rows[-1]['channels'][4] == 1000 and not rows[-1]['armed']
    # Это поддельный ответ в модульном тесте, а не результат полёта SITL.


def test_invalid_parameters_and_source_fail_before_changes(tmp_path):
    assert probe.DEFAULT_PID['yaw'] == (45, 80, 0)
    assert probe.DEFAULT_PID['roll'] == (45, 80, 40)
    assert probe.DEFAULT_PID['pitch'] == (49, 84, 46)
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
    assert probe.OUTPUT.size == 188
