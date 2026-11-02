"""Обычный виртуальный FC не подставляет динамику SITL в фигурку Configurator."""
import struct

from bfemu.fc import VirtualFC
from bfemu.fc_constants import MSP_ATTITUDE


def test_virtual_fc_reports_a_stationary_attitude_at_different_times(monkeypatch):
    fc = VirtualFC(eeprom_path=None)
    for now in (0., .5, 1., 10., 100.):
        monkeypatch.setattr(fc, '_t', lambda now=now: now)
        data = fc.handle(MSP_ATTITUDE, b'', 1)
        assert len(data) == 6
        assert struct.unpack('<hhh', data) == (0, 0, 0)


def test_free_body_keeps_rotating_after_motors_stop_without_drag():
    import math
    import pytest

    from bfemu.flight_physics import Aircraft, State, step
    from tests.test_flight_actuators import aircraft_profile

    aircraft = Aircraft.from_profile(aircraft_profile())
    state = State(angular_velocity_rad_s=(0., .5, 0.))
    for _ in range(250):
        state = step(aircraft, state, [0.]*4, [0.]*4, .004)
    assert state.angular_velocity_rad_s == pytest.approx((0., .5, 0.), abs=1e-12)
    assert state.orientation == pytest.approx((math.cos(.25), 0., math.sin(.25), 0.), abs=1e-8)


def test_attitude_over_tcp_is_stationary_in_virtual_flight_and_failsafe():
    """Проверяет байты по TCP, не имитируя интерфейс Configurator."""
    import socket
    import threading
    import time

    from bfemu.msp import FROM_FC, encode_v1
    from bfemu.server import TcpServer
    from tools import check_attitude

    fc = VirtualFC(eeprom_path=None)
    server = TcpServer(fc, '127.0.0.1', 0, boot_delay=0, verbose=False)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    deadline = time.monotonic() + 3
    while server._sock is None and time.monotonic() < deadline:
        time.sleep(.01)
    assert server._sock is not None
    port = server._sock.getsockname()[1]
    try:
        # Другое открытое MSP-соединение имитирует подключённый Configurator.
        with socket.create_connection(('127.0.0.1', port), timeout=2) as other:
            other.settimeout(2)
            other.sendall(encode_v1(MSP_ATTITUDE))
            expected = encode_v1(MSP_ATTITUDE, bytes(6), FROM_FC)
            received = b''
            while len(received) < len(expected):
                chunk = other.recv(len(expected)-len(received))
                assert chunk
                received += chunk
            assert received == expected
            for mode in ('обычный', 'виртуальный полёт', 'failsafe'):
                if mode == 'виртуальный полёт':
                    fc.set_sim_flight(True)
                elif mode == 'failsafe':
                    fc.set_sim_failsafe(True)
                assert list(check_attitude.samples(port, 5, .01)) == [(0., 0., 0)] * 5
    finally:
        server.stop()
        thread.join(timeout=2)


def test_attitude_diagnostic_decodes_and_checks_packet_size():
    import pytest

    from tools.check_attitude import decode_attitude

    assert decode_attitude(struct.pack('<hhh', -125, 65, 275)) == (-12.5, 6.5, 275)
    with pytest.raises(ValueError, match='6 байт'):
        decode_attitude(b'\0' * 5)
