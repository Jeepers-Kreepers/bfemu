"""Исследование чувствительности SITL к задержке команд моторов; не подбор PID.

Отдельная очередь задерживает применение уже полученных UDP-команд. Это не модель
ESC и не точная копия polling: очереди и квантизация двух режимов различаются.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import select
import statistics
import sys
import time

from . import aircraft_sitl
from .flight_physics import PhysicsError
from .sitl_closed_loop import run_event_loop


class DelayedMotorInput:
    """Фиксированная задержка после чтения UDP и до выдачи команды event-циклу."""
    def __init__(self, sock, delay_s, *, clock=time.monotonic, wait=select.select):
        if not math.isfinite(delay_s) or not 0 <= delay_s <= .02:
            raise PhysicsError('Исследуемая задержка команд должна быть от 0 до 20 мс.')
        self.sock, self.delay_s, self.clock, self._wait = sock, delay_s, clock, wait
        self.pending = deque()
        self.delays = []

    def setblocking(self, value):
        self.sock.setblocking(value)

    def recvfrom(self, size):
        now = self.clock()
        if not self.pending or self.pending[0][0] > now:
            raise BlockingIOError
        _, received, packet, sender = self.pending.popleft()
        self.delays.append(now-received)
        return packet[:size], sender

    def wait(self, readers, write, errors, timeout):
        if readers != [self] or write or errors:
            raise PhysicsError('Неожиданная конфигурация ожидания UDP.')
        deadline = self.clock()+timeout
        while True:
            now = self.clock()
            if self.pending and self.pending[0][0] <= now:
                return [self], [], []
            if now >= deadline:
                return [], [], []
            wake = min(deadline, self.pending[0][0] if self.pending else deadline)
            ready, _, _ = self._wait([self.sock], [], [], max(0., wake-now))
            if ready:
                for _ in range(4096):
                    try:
                        packet, sender = self.sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    received = self.clock()
                    if len(self.pending) >= 4096:
                        raise PhysicsError('Очередь задержанных команд переполнена; опыт остановлен.')
                    self.pending.append((received+self.delay_s, received, packet, sender))
                else:
                    raise PhysicsError('Очередь UDP моторов переполнена; опыт остановлен.')

    def summary(self):
        if not self.delays:
            return {'count': 0}
        values = sorted(self.delays)
        return {'count': len(values), 'median_ms': round(statistics.median(values)*1000, 3),
                'p95_ms': round(values[math.ceil(len(values)*.95)-1]*1000, 3),
                'max_ms': round(values[-1]*1000, 3)}


def make_delayed_loop(delay_s, probes):
    """Обернуть только вход моторов; физика, прошивка и FDM не модифицируются."""
    def loop(profile, model, motor_socket, state_socket, stream, **kwargs):
        delayed = DelayedMotorInput(motor_socket, delay_s)
        probes.append(delayed)
        return run_event_loop(profile, model, delayed, state_socket, stream, wait=delayed.wait, **kwargs)
    return loop


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='новый JSONL, существующий файл не перезаписывается')
    parser.add_argument('--aircraft', type=Path, help='профиль аппарата; по умолчанию теоретический Build 03')
    parser.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='roll')
    parser.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), required=True)
    parser.add_argument('--period-ms', type=int, choices=(2, 4, 8), default=4)
    parser.add_argument('--delays-ms', nargs='+', type=float, default=[0, 4], help='разные задержки, 0–20 мс')
    parser.add_argument('--repeats', type=int, default=4, help='циклы в указанном порядке, 1–10')
    parser.add_argument('--alternate-order', action='store_true', help='менять порядок задержек в чётных циклах')
    args = parser.parse_args(argv)
    if (any(x not in range(251) for x in args.pid) or not 1 <= args.repeats <= 10 or
            any(not math.isfinite(x) or x < 0 or x > 20 for x in args.delays_ms) or
            len(set(args.delays_ms)) != len(args.delays_ms)):
        parser.error('PID: 0–250; задержки: разные конечные значения 0–20 мс; циклы: 1–10')
    try:
        path = args.aircraft or Path(__file__).parent/'data'/'aircraft_build03.json'
        aircraft = json.loads(path.read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(aircraft)
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала подготовьте Betaflight SITL через тренажёр или tools/betaflight_sitl.py build.')
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        failures = 0
        with args.output.open('x', encoding='utf-8') as output:
            def emit(row):
                line = json.dumps(row, ensure_ascii=False, allow_nan=False)
                output.write(line+'\n')
                output.flush()
                print(line, flush=True)
            emit({'type': 'header', 'schema': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                  'firmware': 'Betaflight 4.5.4 SITL', 'revision': aircraft_sitl.REVISION,
                  'platform': platform.platform(), 'aircraft': aircraft, 'axis': args.axis, 'pid': pid,
                  'sensor_period_ms': args.period_ms, 'delays_ms': args.delays_ms, 'repeats': args.repeats,
                  'alternate_order': args.alternate_order,
                  'note': 'Искусственная очередь после чтения UDP, не модель ESC и не точная копия polling. '
                          'Порядок задержек может чередоваться; неуспешные опыты не повторяются.'})
            for cycle in range(1, args.repeats+1):
                delays = list(reversed(args.delays_ms)) if args.alternate_order and cycle % 2 == 0 else args.delays_ms
                for delay in delays:
                    probes = []
                    row = {'type': 'run', 'cycle': cycle, 'requested_delay_ms': delay}
                    started = time.monotonic()
                    try:
                        result = aircraft_sitl.experiment(aircraft, axis=args.axis, pid=pid,
                                                          sensor_period_ms=args.period_ms, motor_mode='event',
                                                          _event_loop=make_delayed_loop(delay/1000, probes))
                        row.update(ok=True, verified_pid=result['verified_pid'], diagnostics=result['diagnostics'])
                    except (PhysicsError, OSError) as exc:
                        failures += 1
                        row.update(ok=False, error=str(exc))
                    row['hold_delay'] = probes[0].summary() if probes else {'count': 0}
                    row['wall_seconds'] = round(time.monotonic()-started, 3)
                    emit(row)
            emit({'type': 'summary', 'attempts': args.repeats*len(args.delays_ms), 'not_completed': failures})
        return 1 if failures else 0
    except (OSError, ValueError) as exc:
        print(f'Исследование задержки SITL: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Исследование прервано; файл может быть неполным.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
