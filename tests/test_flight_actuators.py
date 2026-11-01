"""Кривые приводов и односторонний эксперимент с выходами Betaflight SITL."""
from __future__ import annotations

import io
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import pytest

from bfemu.flight_actuators import Actuators, decode_sitl_motors, output_mapping, read_curves
from bfemu.flight_physics import PhysicsError
from bfemu.msp import FROM_FC, MspParser, encode_v1
from bfemu.sitl_motor_experiment import run_experiment
from tests.test_flight_physics import profile
from tools import betaflight_sitl


def aircraft_profile():
    p = profile()
    for i in range(1, 5):
        p['values'][f'motor_{i}_output'] = str(i)
    return p


def model():
    return {'schema': 1, 'provenance': 'Синтетические входы для теста, не данные реального аппарата',
            'motors': [{'motor': i, 'time_constant_up_s': .1, 'time_constant_down_s': .2,
                        'curve': [{'command': 0, 'thrust_n': 0, 'reaction_nm': 0},
                                  {'command': .5, 'thrust_n': 3, 'reaction_nm': -.1},
                                  {'command': 1, 'thrust_n': 8, 'reaction_nm': -.2}]}
                       for i in range(1, 5)]}


def test_curve_interpolation_and_exact_first_order_lag():
    curves = read_curves(model())
    assert curves[0].target(.25) == pytest.approx((1.5, -.05))
    assert curves[0].target(.75) == pytest.approx((5.5, -.15))
    assert curves[0].target(1) == pytest.approx((8, -.2))
    actuators = Actuators(curves)
    average, reaction = actuators.advance([1]*4, .1)
    assert actuators.thrust_n == pytest.approx([8*(1-math.exp(-1))]*4)
    assert average == pytest.approx([8/math.e]*4)
    assert reaction == pytest.approx([-.2/math.e]*4)
    before = actuators.thrust_n[0]
    actuators.advance([0]*4, .2)
    assert actuators.thrust_n[0] == pytest.approx(before/math.e)


def test_mappings_are_correct_and_untrusted_packets_rejected():
    p = aircraft_profile()
    mapping = output_mapping(p)
    assert mapping == (3, 0, 1, 2)
    packet = struct.pack('<4f', .3, .4, .5, .2)
    assert decode_sitl_motors(packet, mapping) == pytest.approx((.2, .3, .4, .5))
    p['values']['motor_1_output'] = '2'
    with pytest.raises(PhysicsError, match='разным'):
        output_mapping(p)
    p['values']['motor_1_output'] = ''
    with pytest.raises(PhysicsError, match='выход FC'):
        output_mapping(p)
    with pytest.raises(PhysicsError, match='16 байт'):
        decode_sitl_motors(b'bad', mapping)
    with pytest.raises(PhysicsError, match='3D'):
        decode_sitl_motors(struct.pack('<4f', -1, 0, 0, 0), mapping)
    with pytest.raises(PhysicsError, match='Некорректная команда'):
        decode_sitl_motors(struct.pack('<4f', float('nan'), 0, 0, 0), mapping)


def test_refuses_unspecified_and_inconsistent_curves():
    p = model()
    p['provenance'] = ''
    with pytest.raises(PhysicsError, match='источник'):
        read_curves(p)
    p = model()
    p['motors'][0]['time_constant_up_s'] = 0
    with pytest.raises(PhysicsError, match='положительными'):
        read_curves(p)
    p = model()
    p['motors'][0]['curve'][0]['thrust_n'] = 1
    with pytest.raises(PhysicsError, match='начинаться'):
        read_curves(p)
    p = model()
    p['motors'][0]['curve'][1]['command'] = 0
    with pytest.raises(PhysicsError, match='возрастать'):
        read_curves(p)
    p = model()
    p['motors'][0]['curve'][1]['thrust_n'] = 9
    assert read_curves(p)[0].target(.5)[0] == 9  # Измеренная тяга может быть немонотонной.
    p = model()
    p['motors'][1]['motor'] = 1
    with pytest.raises(PhysicsError, match='разными'):
        read_curves(p)


def test_experiment_uses_previous_command_and_refuses_gaps():
    class FakeSocket:
        now = 0.
        sent = 0
        timeout = 0.

        def settimeout(self, timeout):
            self.timeout = timeout

        def recvfrom(self, _):
            self.now += .01
            if self.now > .05:
                self.now += self.timeout
                raise TimeoutError()
            self.sent += 1
            return struct.pack('<4f', 0, 0, 0, .5), ('127.0.0.1', 2222)

    sock = FakeSocket()
    output = io.StringIO()
    count = run_experiment(aircraft_profile(), model(), sock, output, duration=.05, max_gap=.05, clock=lambda: sock.now)
    records = [json.loads(row) for row in output.getvalue().splitlines()]
    assert count >= 1 and records[0]['feedback_to_sitl'] is False
    assert records[1]['applied_commands'] == pytest.approx([.5, 0, 0, 0])
    assert records[1]['average_thrust_n'][0] > 0
    assert records[1]['state']['angular_velocity_rad_s'][0] < 0
    with pytest.raises(PhysicsError, match='Нет пакетов'):
        sock = FakeSocket()
        run_experiment(aircraft_profile(), model(), sock, io.StringIO(), duration=.5, max_gap=.05, clock=lambda: sock.now)


def test_cli_does_not_overwrite_existing_file_or_guess_missing_values(tmp_path):
    p, curves, output = tmp_path / 'profile.json', tmp_path / 'curves.json', tmp_path / 'result.jsonl'
    p.write_text(json.dumps(aircraft_profile()), encoding='utf-8')
    curves.write_text(json.dumps(model()), encoding='utf-8')
    output.write_text('existing data', encoding='utf-8')
    args = [sys.executable, '-m', 'bfemu.sitl_motor_experiment', '--profile', str(p),
            '--actuators', str(curves), '--output', str(output), '--duration', '.1']
    result = subprocess.run(args, capture_output=True, text=True, check=False, timeout=5)
    assert result.returncode != 0
    assert output.read_text(encoding='utf-8') == 'existing data'
    output.unlink()
    curves.write_text('{}', encoding='utf-8')
    result = subprocess.run(args, capture_output=True, text=True, check=False, timeout=5)
    assert result.returncode != 0 and not output.exists()


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Требуется сборка настоящего Betaflight SITL')
def test_real_sitl_motor_commands_reach_physical_model(tmp_path):
    binary = betaflight_sitl.binary_path(betaflight_sitl.DEFAULT_CACHE)
    assert betaflight_sitl.built(betaflight_sitl.DEFAULT_CACHE)
    for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9002),
                       (socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004)):
        with socket.socket(socket.AF_INET, kind) as probe:
            try:
                probe.bind(('127.0.0.1', port))
            except OSError:
                pytest.skip(f'Порт {port} занят')

    def start():
        return subprocess.Popen([str(binary)], cwd=tmp_path, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def connect(process):
        end = time.monotonic() + 15
        while time.monotonic() < end:
            try:
                return socket.create_connection(('127.0.0.1', 5761), timeout=1)
            except OSError:
                if process.poll() is not None:
                    pytest.fail('SITL завершился до открытия MSP')
                time.sleep(.05)
        pytest.fail('SITL не открыл MSP')

    process = start()
    try:
        with connect(process) as cli:
            cli.settimeout(3)
            def read_until(marker):
                data = b''
                while marker not in data:
                    chunk = cli.recv(4096)
                    assert chunk
                    data += chunk
                return data
            cli.sendall(b'#')
            read_until(b'CLI Mode')
            cli.sendall(b'set motor_pwm_protocol = PWM\n')
            read_until(b'motor_pwm_protocol set to PWM')
            cli.sendall(b'save\n')
            read_until(b'saving')
        process.wait(timeout=10)
        process = start()
        with connect(process) as client, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as motors, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fdm:
            client.settimeout(3)
            motors.bind(('127.0.0.1', 9002))
            parser = MspParser()

            def exchange(code, payload=b''):
                client.sendall(encode_v1(code, payload))
                while True:
                    data = client.recv(4096)
                    assert data
                    parser.feed(data)
                    for reply in parser:
                        if reply.code == code:
                            assert reply.direction == FROM_FC
                            return reply.payload

            assert exchange(2) == b'BTFL' and exchange(3) == bytes((4, 5, 4))
            stop = threading.Event()

            def send_fdm():
                started = time.monotonic()
                while not stop.is_set():
                    packet = struct.pack('<18d', time.monotonic()-started, 0, 0, 0, 0, 0, -9.80665,
                                         1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 101325)
                    fdm.sendto(packet, ('127.0.0.1', 9003))
                    stop.wait(.005)

            feeder = threading.Thread(target=send_fdm)
            feeder.start()
            try:
                exchange(214, struct.pack('<4H', 1200, 1300, 1400, 1500))
                output = io.StringIO()
                count = run_experiment(aircraft_profile(), model(), motors, output, duration=1, max_gap=.5)
                rows = [json.loads(row) for row in output.getvalue().splitlines()[1:]]
                assert count > 0
                assert any(row['applied_commands'] == pytest.approx([.2, .3, .4, .5]) for row in rows)
                assert any(row['average_thrust_n'][0] > 0 for row in rows)
                exchange(214, struct.pack('<4H', 1000, 1000, 1000, 1000))
            finally:
                stop.set()
                feeder.join(timeout=2)
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
