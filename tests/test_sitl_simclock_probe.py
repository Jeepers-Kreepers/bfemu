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


@pytest.mark.parametrize('bench', [False, True])
@pytest.mark.parametrize('stick,throttle,impulse,hold', [(1650, 1300, (0., 0., 0.), 0),
                                                        (1350, 1600, (0., 25., 0.), 10)])
@pytest.mark.parametrize('axis,channel', [('roll', 0), ('pitch', 1), ('yaw', 3)])
def test_run_uses_one_rc_command_per_frame_and_disarms(monkeypatch, axis, channel, stick, throttle, impulse, hold, bench):
    profile = json.loads((Path(__file__).resolve().parents[1]/'bfemu/data/aircraft_build03.json').read_text())
    current = {}
    packets = []
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
            packets.append(probe.FDM_PACKET.unpack(payload[probe.INPUT_HEADER.size:probe.INPUT_HEADER.size+probe.FDM_PACKET.size]))
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
    rows, events = probe.run_frames(profile, 1000, scenario='flight', on_frame=captured.append, axis=axis,
                                    stick=stick, throttle=throttle, impulse_dps=impulse, prearm_frames=hold, prearm_bench=bench)
    assert len(rows) == len(captured) == events['disarmed_at']
    assert events['ready_at'] == 2
    assert events['arm_request'] == events['armed_at'] == 3+hold
    assert events['command_start'] == 154+hold and events['command_end'] == 454+hold
    assert events['disarm_request'] == events['disarmed_at'] == 579+hold
    assert sum(row['channels'][channel] == stick for row in rows) == 300
    if bench:
        assert events['bench_release'] == events['armed_at']+1
        assert all(row['bench_held'] and row['state']['position_m'] == (0., 0., 0.)
                   for row in rows[:events['armed_at']])
        assert all(packet[6] == pytest.approx(-probe.GRAVITY) for packet in packets[:events['armed_at']])
        assert all(row['fdm_accel_m_s2'] == [0., 0., -probe.GRAVITY] for row in rows[:events['armed_at']])
        assert not rows[events['armed_at']]['bench_held']
        assert packets[events['armed_at']][6] == pytest.approx(0.)
    else:
        assert events['bench_release'] is None and not any(row['bench_held'] for row in rows)
    command = rows[events['command_start']-1]
    assert command['channels'][2] == throttle
    if any(impulse):
        assert command['impulse_dps'] == list(impulse)
        assert command['state']['angular_velocity_rad_s'][1] == pytest.approx(25 * 3.141592653589793 / 180)
    else:
        assert 'impulse_dps' not in command
    assert rows[-1]['channels'][4] == 1000 and not rows[-1]['armed']
    # Это поддельный ответ в модульном тесте, а не результат полёта SITL.


def test_invalid_parameters_and_source_fail_before_changes(tmp_path):
    assert probe.DEFAULT_PID['yaw'] == (45, 80, 0)
    assert probe.DEFAULT_PID['roll'] == (45, 80, 40)
    assert probe.DEFAULT_PID['pitch'] == (49, 84, 46)
    with pytest.raises(SystemExit):
        probe.main(['--output-dir', str(tmp_path/'out'), '--pid', '251', '84', '46'])
    assert not (tmp_path/'out').exists()
    for args in (['--stick', '999'], ['--throttle', '2001'], ['--prearm-frames', '-1'],
                 ['--impulse-dps', '0', 'nan', '0'], ['--impulse-dps', '0', '721', '0']):
        with pytest.raises(SystemExit):
            probe.main(['--output-dir', str(tmp_path/'out'), *args])
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


def test_invalid_profile_is_rejected_before_build_or_output_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(probe.aircraft_sitl, 'sitl_available', lambda: True)
    profile = tmp_path/'invalid.json'
    profile.write_text('{"schema":1,"values":{"inertia_yy":"not a number"}}')
    out = tmp_path/'output'
    assert probe.main(['--output-dir', str(out), '--profile', str(profile)]) == 1
    assert not out.exists()


def test_default_frame_limit_depends_on_scenario():
    assert probe.parse_args(['--output-dir', 'x']).frames == 500
    assert probe.parse_args(['--output-dir', 'x', '--scenario', 'flight']).frames == 12000
    assert probe.parse_args(['--output-dir', 'x', '--scenario', 'flight', '--frames', '500']).frames == 500
