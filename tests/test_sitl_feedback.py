"""Протокол датчиков и двусторонняя проверка с официальным Betaflight SITL."""
from __future__ import annotations

import io
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time

import pytest

from bfemu.flight_physics import GRAVITY, Aircraft, PhysicsError, State
from bfemu.msp import FROM_FC, MspParser, encode_v1
from bfemu.sitl_capture import decode_payload
from bfemu.sitl_closed_loop import run_loop
from bfemu.sitl_feedback import FDM_PACKET, sensor_packet
from tests.test_flight_actuators import aircraft_profile, model
from tools import betaflight_sitl


def test_fdm_packet_axes_gravity_quaternion_and_pressure():
    aircraft = Aircraft.from_profile(aircraft_profile())
    state = State(position_m=(1, 2, 3), velocity_m_s=(4, 5, 6), orientation=(2, 0, 0, 0),
                  angular_velocity_rad_s=(.1, -.2, .3))
    packet = sensor_packet(aircraft, state, [GRAVITY/4]*4, timestamp=1.25, pressure_pa=101325)
    assert len(packet) == 18*8
    fields = FDM_PACKET.unpack(packet)
    assert fields[:4] == pytest.approx((1.25, .1, -.2, .3))
    assert fields[4:7] == pytest.approx((0, 0, -GRAVITY))
    assert fields[7:11] == pytest.approx((1, 0, 0, 0))
    assert fields[11:17] == pytest.approx((4, 5, 6, 1, 2, 3))
    assert fields[17] == 101325
    # Удельная сила в свободном падении равна нулю, хотя ускорение тела равно g.
    freefall = FDM_PACKET.unpack(sensor_packet(aircraft, state, [0]*4, timestamp=0, pressure_pa=1))
    assert freefall[4:7] == (0, 0, 0)
    with pytest.raises(PhysicsError, match='Давление'):
        sensor_packet(aircraft, state, [0]*4, timestamp=0, pressure_pa=float('nan'))
    with pytest.raises(PhysicsError, match='Метка времени'):
        sensor_packet(aircraft, state, [0]*4, timestamp=-1, pressure_pa=101325)
    with pytest.raises(PhysicsError, match='Отрицательная тяга'):
        sensor_packet(aircraft, state, [-1, 0, 0, 0], timestamp=0, pressure_pa=101325)


class FakeClock:
    def __init__(self, oversleep=0):
        self.now = 0.
        self.oversleep = oversleep

    def clock(self):
        return self.now

    def sleep(self, interval):
        self.now += interval + self.oversleep


class FakeMotorSocket:
    def __init__(self, packet=None):
        self.packet = packet
        self.sent = False

    def setblocking(self, value):
        assert not value

    def recvfrom(self, _):
        if self.packet and not self.sent:
            self.sent = True
            return self.packet, ('127.0.0.1', 1234)
        self.sent = False
        raise BlockingIOError


class FakeStateSocket:
    def __init__(self):
        self.packets = []

    def sendto(self, packet, addr):
        assert addr == ('127.0.0.1', 9003)
        self.packets.append(FDM_PACKET.unpack(packet))
        return len(packet)


def test_loop_sends_timestamped_feedback_and_applies_previous_command():
    clock = FakeClock()
    receiver = FakeMotorSocket(struct.pack('=4f', 0, 0, 0, .5))
    sender, output = FakeStateSocket(), io.StringIO()
    count = run_loop(aircraft_profile(), model(), receiver, sender, output, duration=.03, period=.004,
                     max_gap=.1, pressure_pa=101325, clock=clock.clock, sleep=clock.sleep)
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert count == len(rows)-1 == len(sender.packets)-1
    assert rows[0]['feedback_to_sitl'] is True
    assert rows[1]['applied_commands'] == [0, 0, 0, 0]
    assert rows[2]['applied_commands'] == pytest.approx([.5, 0, 0, 0])
    assert rows[2]['state']['angular_velocity_rad_s'][0] < 0
    assert sender.packets[2][1] == pytest.approx(rows[2]['state']['angular_velocity_rad_s'][0])
    assert sender.packets[2][6] < 0
    assert [p[0] for p in sender.packets] == sorted({p[0] for p in sender.packets})


def test_loop_stops_on_timing_loss_and_missing_motor_data():
    clock = FakeClock(oversleep=.02)
    with pytest.raises(PhysicsError, match='Нарушен темп'):
        run_loop(aircraft_profile(), model(), FakeMotorSocket(), FakeStateSocket(), io.StringIO(),
                 duration=.2, period=.004, max_gap=.1, pressure_pa=101325, clock=clock.clock, sleep=clock.sleep)
    clock = FakeClock()
    with pytest.raises(PhysicsError, match='Нет пакетов моторов'):
        run_loop(aircraft_profile(), model(), FakeMotorSocket(), FakeStateSocket(), io.StringIO(),
                 duration=.2, period=.004, max_gap=.05, pressure_pa=101325, clock=clock.clock, sleep=clock.sleep)


def test_capture_converts_sitl_virtual_gyro_counts_to_dps():
    # sitl.c produces 30*16.4 raw counts; gyroRateDps divides by gyroDev.scale
    # and MSP writes those counts. For SITL virtual scale = 2000/32768.
    data = decode_payload(102, struct.pack('<9h', 0, 0, 256, 491, -327, 410, 0, 0, 0))
    assert data['gyro_raw'] == [491, -327, 410]
    assert data['gyro_dps'] == pytest.approx([30, -20, 25], abs=.1)


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный Betaflight SITL')
def test_real_sitl_reads_feedback_and_controls_motors(tmp_path):
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
        end = time.monotonic()+15
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
            cli.sendall(b'#')
            read_until(b'CLI Mode')
            cli.sendall(b'set motor_pwm_protocol = PWM\n')
            read_until(b'motor_pwm_protocol set to PWM')
            cli.sendall(b'save\n')
            read_until(b'saving')
        process.wait(timeout=10)
        process = start()
        with connect(process) as client, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as motors, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fdm, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rc:
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
            assert exchange(4)[:4] == b'SITL'
            # Настроить ARM на AUX1: 1700–2100. Все изменения только в изолированном процессе.
            exchange(35, bytes([0, 0, 0, 32, 48]))
            profile = aircraft_profile()
            values = profile['values']
            values.update({'mass_g': '931', 'inertia_xx': '0.00711', 'inertia_yy': '0.00754', 'inertia_zz': '0.01362'})
            for i, (x, y) in enumerate(((-143, 143), (143, 143), (-143, -143), (143, -143)), 1):
                values.update({f'motor_{i}_x': str(x), f'motor_{i}_y': str(y), f'motor_{i}_z': '-17.4'})
            curves = model()
            # Тестовая конфигурация, а не характеристики настоящего аппарата.
            for i, motor in enumerate(curves['motors']):
                for point in motor['curve']:
                    point['reaction_nm'] *= -1 if i in (0, 3) else 1
            rc_channels = [1500, 1500, 1000, 1500, 1000] + [1000]*11  # AETR, AUX1
            stop = threading.Event()
            result, errors, output = [], [], io.StringIO()

            def send_rc():
                beginning = time.monotonic()
                while not stop.is_set():
                    rc.sendto(struct.pack('=d16H', time.monotonic()-beginning, *rc_channels), ('127.0.0.1', 9004))
                    stop.wait(.02)

            def simulate():
                try:
                    result.append(run_loop(profile, curves, motors, fdm, output, duration=14, period=.004,
                                           max_gap=.5, pressure_pa=101325))
                except Exception as exc:
                    errors.append(exc)

            feeder = threading.Thread(target=send_rc)
            worker = threading.Thread(target=simulate)
            feeder.start()
            worker.start()
            try:
                # До arm гироскоп находится в покое и калибруется в настоящей прошивке.
                deadline = time.monotonic()+8
                last_status = exchange(101)
                while len(last_status) < 21 or struct.unpack_from('<I', last_status, 17)[0] != 0:
                    assert not errors, errors
                    assert time.monotonic() < deadline, f'SITL не готов к ARM: status={last_status.hex()}'
                    time.sleep(.1)
                    last_status = exchange(101)
                rc_channels[4] = 1900
                deadline = time.monotonic()+2
                last_status = exchange(101)
                while not (struct.unpack_from('<I', last_status, 6)[0] & 1):
                    assert time.monotonic() < deadline, f'SITL не включил ARM: status={last_status.hex()} errors={errors}'
                    time.sleep(.1)
                    last_status = exchange(101)
                rc_channels[2] = 1300
                time.sleep(.6)
                rc_channels[0] = 1650
                time.sleep(.7)
                imu = decode_payload(102, exchange(102))
                output_motors = decode_payload(104, exchange(104))['motor_output_msp'][:4]
                assert abs(imu['gyro_dps'][0]) > 5, imu
                assert max(output_motors)-min(output_motors) > 10, output_motors
                rc_channels[4] = 1000
                rc_channels[2] = 1000
                worker.join(timeout=15)
                assert not worker.is_alive() and not errors, errors
                rows = [json.loads(line) for line in output.getvalue().splitlines()]
                assert result[0] > 100 and rows[0]['feedback_to_sitl'] is True
                assert any(abs(row['state']['angular_velocity_rad_s'][0]) > .1 for row in rows[1:])
                assert any(max(row['applied_commands'])-min(row['applied_commands']) > .01 for row in rows[1:])
            finally:
                stop.set()
                feeder.join(timeout=2)
                worker.join(timeout=3)
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный Betaflight SITL')
def test_real_sitl_sensor_axes_and_units(tmp_path):
    from bfemu.sitl_capture import SitlMspClient

    binary = betaflight_sitl.binary_path(betaflight_sitl.DEFAULT_CACHE)
    assert betaflight_sitl.built(betaflight_sitl.DEFAULT_CACHE)
    for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004)):
        with socket.socket(socket.AF_INET, kind) as probe:
            try:
                probe.bind(('127.0.0.1', port))
            except OSError:
                pytest.skip(f'Порт {port} занят')
    process = subprocess.Popen([str(binary)], cwd=tmp_path, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    client = None
    state = State()
    forces = [GRAVITY/4]*4
    stop = threading.Event()
    aircraft = Aircraft.from_profile(aircraft_profile())
    transmitter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def feed():
        start = time.monotonic()
        while not stop.is_set():
            packet = sensor_packet(aircraft, state, forces, timestamp=time.monotonic()-start, pressure_pa=101325)
            transmitter.sendto(packet, ('127.0.0.1', 9003))
            stop.wait(.004)

    worker = threading.Thread(target=feed)
    try:
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            try:
                client = SitlMspClient(timeout=1)
                break
            except OSError:
                if process.poll() is not None:
                    pytest.fail('SITL завершился до открытия MSP')
                time.sleep(.05)
        assert client is not None
        client.identify()
        deadline = time.monotonic()+5
        while struct.unpack_from('<H', client.request(101), 4)[0] & 0x21 != 0x21:
            assert time.monotonic() < deadline, 'SITL не инициализировал датчики'
            time.sleep(.05)
        worker.start()
        time.sleep(4)  # Дать реальной прошивке откалибровать неподвижный гироскоп.
        imu = decode_payload(102, client.request(102))
        assert imu['acc_adc'] == pytest.approx([0, 0, 256], abs=3)
        for axis in range(3):
            angle = math.radians(20)/2
            quat = [math.cos(angle), 0., 0., 0.]
            quat[axis+1] = math.sin(angle)
            omega = [0., 0., 0.]
            omega[axis] = math.radians(30)
            state = State(orientation=tuple(quat), angular_velocity_rad_s=tuple(omega))
            time.sleep(.4)
            imu = decode_payload(102, client.request(102))
            expected = [0., 0., 0.]
            expected[axis] = 30. if axis == 0 else -30.
            assert imu['gyro_dps'] == pytest.approx(expected, abs=2)
            attitude = decode_payload(108, client.request(108))
            expected_attitude = [[200, 0, 0], [0, -200, 0], [0, 0, 20]][axis]
            assert [attitude['roll_decideg'], attitude['pitch_decideg'], attitude['yaw_deg']] == pytest.approx(expected_attitude, abs=3)
        state = State()
        forces = [0]*4
        time.sleep(.4)
        assert decode_payload(102, client.request(102))['acc_adc'] == pytest.approx([0, 0, 0], abs=3)
    finally:
        stop.set()
        if worker.ident is not None:
            worker.join(timeout=2)
        transmitter.close()
        if client:
            client.close()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def test_check_ready_verifies_sitl_and_initialized_sensors(monkeypatch):
    from bfemu import sitl_closed_loop
    from bfemu.sitl_capture import CaptureError

    clients = []

    class Fake:
        sensors = 0x21

        def __init__(self, port, timeout):
            assert port == 5761 and timeout == 2
            self.closed = False
            clients.append(self)

        def identify(self):
            return {'variant': 'BTFL', 'version': '4.5.4', 'board': 'SITL'}

        def request(self, code):
            assert code == 101
            return struct.pack('<HHH', 0, 0, self.sensors)

        def close(self):
            self.closed = True

    monkeypatch.setattr(sitl_closed_loop, 'SitlMspClient', Fake)
    sitl_closed_loop.check_ready()
    assert clients[-1].closed
    Fake.sensors = 0x20
    with pytest.raises(CaptureError, match='ещё не инициализировал'):
        sitl_closed_loop.check_ready()
    assert clients[-1].closed


def test_quad_x_mixer_and_gyro_signs_produce_negative_feedback():
    """Quad X Betaflight 4.5.4: положительная PID-сумма повышает одноимённое показание FC."""
    from pathlib import Path

    from bfemu.flight_physics import derivative

    profile = json.loads((Path(__file__).parents[1] / 'bfemu/data/aircraft_build03.json').read_text())
    aircraft = Aircraft.from_profile(profile)
    # mixer_init.c mixerQuadX, включая инверсию yaw при yaw_motors_reversed=OFF в mixer.c.
    mixer = ((-1, -1, 1, 1), (1, -1, 1, -1), (1, -1, -1, 1))
    for axis, factors in enumerate(mixer):
        thrust = [2 + .1 * factor for factor in factors]
        reaction = [(-1 if profile['values'][f'motor_{i+1}_direction'] == 'cw' else 1) * .016 * force
                    for i, force in enumerate(thrust)]
        acceleration = derivative(aircraft, State(), thrust, reaction).angular_acceleration_rad_s2
        packet = FDM_PACKET.unpack(sensor_packet(aircraft, State(angular_velocity_rad_s=acceleration), thrust,
                                                  timestamp=1, pressure_pa=101325))
        # sitl.c passes X unchanged and negates Y/Z before virtualGyroSet.
        gyro = (packet[1], -packet[2], -packet[3])
        assert gyro[axis] > 0
        assert [gyro[i] for i in range(3) if i != axis] == pytest.approx([0, 0], abs=1e-10)


def test_event_loop_applies_motors_when_read_before_next_sensor_tick():
    from bfemu.sitl_closed_loop import run_event_loop

    clock = FakeClock()
    commands = [(0.001, struct.pack('=4f', 0, 0, 0, 1.0)),
                (0.003, struct.pack('=4f', 0, 0, 0, .5))]

    class ScheduledMotor:
        def setblocking(self, value):
            assert not value
        def recvfrom(self, size):
            if commands and commands[0][0] <= clock.now:
                return commands.pop(0)[1], ('127.0.0.1', 1234)
            raise BlockingIOError

    motor = ScheduledMotor()
    def wait(readers, _write, _errors, timeout):
        deadline = clock.now + timeout
        if commands and commands[0][0] <= deadline:
            clock.now = commands[0][0]
            return readers, [], []
        clock.now = deadline
        return [], [], []

    sender, output = FakeStateSocket(), io.StringIO()
    count = run_event_loop(aircraft_profile(), model(), motor, sender, output, duration=.012, period=.004,
                           max_gap=.05, pressure_pa=101325, clock=clock.clock, wait=wait)
    rows = [json.loads(line) for line in output.getvalue().splitlines()]
    assert count == 2 and rows[0]['motor_mode'] == 'event'
    assert rows[1]['next_commands'] == pytest.approx([.5, 0, 0, 0])
    assert rows[1]['applied_commands'] == pytest.approx([.5, 0, 0, 0])
    # Обе команды были применены до первой отправки датчиков на 4 мс.
    assert rows[1]['state']['angular_velocity_rad_s'][0] < 0
    assert rows[1]['motor_full_output_since_sensor'] is True
    assert rows[2]['motor_full_output_since_sensor'] is False
    assert [p[0] for p in sender.packets] == pytest.approx([0, .004, .008])


def test_event_loop_still_stops_when_sensor_deadline_exceeds_limit():
    from bfemu.sitl_closed_loop import run_event_loop
    clock = FakeClock()
    def wait(_readers, _write, _errors, timeout):
        clock.now += timeout + .02
        return [], [], []
    with pytest.raises(PhysicsError, match='Нарушен темп'):
        run_event_loop(aircraft_profile(), model(), FakeMotorSocket(), FakeStateSocket(), io.StringIO(),
                       duration=.1, period=.004, max_gap=.05, pressure_pa=101325, clock=clock.clock, wait=wait)


def test_discard_stale_motor_packets_before_start():
    from bfemu.sitl_closed_loop import discard_queued_motors

    class Queued:
        def __init__(self):
            self.packets = [b'old1', b'old2']
        def setblocking(self, flag):
            assert not flag
        def recvfrom(self, _):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0), ('127.0.0.1', 1234)
    queued = Queued()
    assert discard_queued_motors(queued) == 2
    assert discard_queued_motors(queued) == 0
    queued.packets = [b'flood']*4096
    with pytest.raises(PhysicsError, match='Очередь'):
        discard_queued_motors(queued)
