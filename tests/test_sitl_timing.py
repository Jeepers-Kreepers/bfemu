"""Отдельный опциональный измеритель очереди не меняет протокол датаграммы."""
import socket
import time

import pytest

from bfemu.sitl_timing import KernelQueueProbe, timing_supported


@pytest.mark.skipif(not timing_supported(), reason='Нужны метки UDP Linux x86_64 или aarch64')
def test_kernel_queue_probe_measures_receive_age_and_preserves_payload():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        receiver.bind(('127.0.0.1', 0))
        probe = KernelQueueProbe()
        assert probe.summary()['valid'] is False
        wrapped = probe.wrap(receiver)
        tx.sendto(b'packet', receiver.getsockname())
        time.sleep(.003)
        assert wrapped.recvfrom(100)[0] == b'packet'
        result = probe.summary()
        assert result['valid'] and result['packets'] == 1
        assert result['median_ms'] >= 1
        assert result['median_ms'] == result['p95_ms'] == result['max_ms']
        assert wrapped.fileno() == receiver.fileno()


@pytest.mark.skipif(not timing_supported(), reason='Нужны метки UDP Linux x86_64 или aarch64')
def test_queue_probe_marks_clock_jump_invalid(monkeypatch):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        receiver.bind(('127.0.0.1', 0))
        probe = KernelQueueProbe().wrap(receiver)
        actual = time.time_ns
        jump = [0]
        monkeypatch.setattr(time, 'time_ns', lambda: actual() + jump[0])
        for adjustment in (0, 10_000_000):
            jump[0] = adjustment
            tx.sendto(b'x', receiver.getsockname())
            assert probe.recvfrom(20)[0] == b'x'
        summary = probe.summary()
        assert summary['valid'] is False and summary['packets'] == 2
        assert 'часы' in summary['reason']
