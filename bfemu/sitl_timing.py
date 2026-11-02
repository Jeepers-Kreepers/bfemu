"""Необязательное измерение ожидания UDP моторов в очереди ядра Linux."""
from __future__ import annotations

import math
import platform
import socket
import statistics
import struct
import sys
import time

from .flight_physics import PhysicsError

# Linux UAPI SO_TIMESTAMPNS_OLD; используется только с 64-битным struct timespec.
_SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)
_TIMESPEC = struct.Struct('@ll')


def timing_supported():
    return (sys.platform == 'linux' and platform.machine().lower() in ('x86_64', 'aarch64')
            and _TIMESPEC.size == 16 and hasattr(socket.socket, 'recvmsg'))


class KernelQueueProbe:
    """Измерить возраст датаграммы от поступления в сокет до чтения программой.

    Это не задержка реакции FC: UDP моторов не содержит идентификатора FDM.
    Абсолютные времена используют CLOCK_REALTIME, поэтому сдвиг системных часов
    делает всю статистику недействительной. Измерение добавляет небольшую нагрузку.
    """
    def __init__(self):
        if not timing_supported():
            raise PhysicsError('Метки UDP ядра поддерживаются только на 64-битном Linux x86_64 или aarch64.')
        self._socket = None
        self._ages = []
        self._offset_min = self._offset_max = None
        self._negative = 0

    def wrap(self, sock):
        if self._socket is not None:
            raise PhysicsError('Измеритель UDP уже подключён к сокету.')
        sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
        self._socket = sock
        return self

    def fileno(self):
        return self._socket.fileno()

    def setblocking(self, flag):
        return self._socket.setblocking(flag)

    def recvfrom(self, size):
        data, control, flags, address = self._socket.recvmsg(size, socket.CMSG_SPACE(_TIMESPEC.size))
        wall, mono = time.time_ns(), time.monotonic_ns()
        offset = wall-mono
        self._offset_min = offset if self._offset_min is None else min(self._offset_min, offset)
        self._offset_max = offset if self._offset_max is None else max(self._offset_max, offset)
        if flags & socket.MSG_CTRUNC:
            raise PhysicsError('Метка времени UDP усечена; измерение остановлено.')
        timestamps = [payload for level, kind, payload in control
                      if level == socket.SOL_SOCKET and kind == _SO_TIMESTAMPNS and len(payload) >= _TIMESPEC.size]
        if len(timestamps) != 1:
            raise PhysicsError('Ядро не вернуло единственную метку времени UDP.')
        sec, ns = _TIMESPEC.unpack_from(timestamps[0])
        if not 0 <= ns < 1_000_000_000:
            raise PhysicsError('Некорректная метка времени UDP от ядра.')
        age = wall-(sec*1_000_000_000+ns)
        if age < 0:
            self._negative += 1
        else:
            self._ages.append(age)
        return data, address

    def summary(self):
        count = len(self._ages)+self._negative
        if not count:
            return {'valid': False, 'packets': 0, 'reason': 'Нет полученных пакетов моторов.'}
        drift = (self._offset_max-self._offset_min)/1e6
        if self._negative or drift > 1.0:
            return {'valid': False, 'packets': count, 'clock_offset_span_ms': round(drift, 3),
                    'reason': 'Системные часы изменились во время измерения или возраст пакета оказался отрицательным.'}
        ages = sorted(self._ages)
        def ms(ns):
            return round(ns/1e6, 3)
        return {'valid': True, 'packets': count, 'clock_offset_span_ms': round(drift, 3),
                'median_ms': ms(statistics.median(ages)), 'p95_ms': ms(ages[math.ceil(.95*len(ages))-1]),
                'max_ms': ms(ages[-1])}
