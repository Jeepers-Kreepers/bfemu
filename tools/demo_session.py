#!/usr/bin/env python3
"""
Демонстрация: подключаемся к запущенному виртуальному FC как Betaflight Configurator,
проходим рукопожатие, читаем телеметрию и выполняем команды CLI.

Запуск:
    python3 -m bfemu --tcp 0.0.0.0:5761 --http 8088     # в одном терминале
    python3 tools/demo_session.py                        # в другом
"""
from __future__ import annotations

import socket
import struct
import sys
import time

HOST, PORT = "127.0.0.1", 5761


class Link:
    def __init__(self):
        self.s = socket.create_connection((HOST, PORT), timeout=3)
        self.buf = bytearray()

    def send_msp(self, code: int, payload: bytes = b"", v2: bool = False) -> bytes:
        if v2:
            head = bytes([0, code & 0xFF, code >> 8, len(payload) & 0xFF, len(payload) >> 8]) + payload
            crc = 0
            for b in head:
                crc ^= b
                for _ in range(8):
                    crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
            frame = bytes([0x24, ord("X"), ord("<")]) + head + bytes([crc])
        else:
            body = bytes([len(payload), code]) + payload
            crc = 0
            for b in body:
                crc ^= b
            frame = bytes([0x24, ord("M"), ord("<")]) + body + bytes([crc])
        self.s.sendall(frame)
        return self._read_response()

    def _read_response(self) -> bytes:
        deadline = time.time() + 3
        while time.time() < deadline:
            self._parse()
            if getattr(self, "_ready", None) is not None:
                data = self._ready
                self._ready = None
                return data
            try:
                chunk = self.s.recv(65536)
            except socket.timeout:
                break
            self.buf += chunk
        raise TimeoutError("нет ответа")

    def _parse(self):
        self._ready = None
        b = self.buf
        while True:
            if not b or b[0] != 0x24:
                if b:
                    del b[0]
                    continue
                return
            if len(b) < 5:
                return
            proto = b[1]
            if proto == ord("M"):
                ln, code = b[3], b[4]
                total = 5 + ln + 1
                if len(b) < total:
                    return
                payload = bytes(b[5 : 5 + ln])
                del b[:total]
                self._ready = payload
                return
            if proto == ord("X"):
                if len(b) < 9:
                    return
                code = b[4] | (b[5] << 8)
                ln = b[6] | (b[7] << 8)
                total = 8 + ln + 1
                if len(b) < total:
                    return
                payload = bytes(b[8 : 8 + ln])
                del b[:total]
                self._ready = payload
                return
            del b[0]

    def cli(self, command: str) -> str:
        """CLI через MSP2_CLI_COMMAND с пейджингом."""
        out, offset = b"", 0
        while True:
            payload = command.encode() if offset == 0 else command.encode() + b"\x00" + struct.pack("<H", offset)
            resp = self.send_msp(0x3012, payload, v2=True)
            total = struct.unpack_from("<H", resp, 0)[0]
            out += resp[3:]
            offset += len(resp) - 3
            if offset >= total or len(resp) == 3:
                break
        return out.decode("latin-1", "replace")

    def raw_cli(self, command: str) -> str:
        """Сырой CLI-порт: STX + команда + LF + ETX (как MSP.send_cli_command)."""
        self.s.sendall(bytes([0x02]) + command.encode() + b"\n" + bytes([0x03]))
        data = b""
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                chunk = self.s.recv(65536)
            except socket.timeout:
                break
            data += chunk
            if data.startswith(bytes([0x02])) and data.endswith(bytes([0x03])):
                break
        return data.decode("latin-1", "replace")


def main():
    link = Link()
    print("== Подключение (рукопожатие как в configurator) ==")
    api = link.send_msp(1)
    print(f"  MSP API: {api[1]}.{api[2]}")
    print(f"  FC variant: {link.send_msp(2).decode()}")
    v = link.send_msp(3)
    print(f"  Прошивка: {v[0]}.{v[1]}.{v[2]}")
    board = link.send_msp(4)
    print(f"  Плата: {board[:4].decode()}, цель: {board[8:].split(bytes([0]))[0].decode()}")
    print(f"  UID: {''.join(f'{x:08x}' for x in struct.unpack('<III', link.send_msp(160)))}")

    print("\n== Телеметрия ==")
    analog = link.send_msp(110)
    print(f"  Напряжение: {struct.unpack_from('<H', analog, 7)[0] / 100:.2f} В, "
          f"ток: {struct.unpack_from('<h', analog, 5)[0] / 100:.2f} А")
    att = struct.unpack("<hhh", link.send_msp(108))
    print(f"  Крен/тангаж/курс: {att[0] / 10:.1f}° / {att[1] / 10:.1f}° / {att[2] / 10:.1f}°")
    gps = link.send_msp(106)
    lat, lon = struct.unpack_from("<ii", gps, 2)
    print(f"  GPS: fix={gps[0]} спутников={gps[1]} координаты={lat / 1e7:.5f}, {lon / 1e7:.5f}")
    rc = struct.unpack("<8H", link.send_msp(105))
    print(f"  RC-каналы: {rc}")

    print("\n== CLI поверх MSP (то, чем пользуются вкладки настроек) ==")
    print("  $ get gyro_lpf1_static_hz")
    for line in link.cli("get gyro_lpf1_static_hz").splitlines():
        print("   ", line)

    print("\n== Сырой CLI-порт (вкладка CLI) ==")
    print("  $ diff all  (первые строки)")
    out = link.raw_cli("diff all")
    for line in out.strip("\x02\x03").splitlines()[:14]:
        print("   ", line)

    print("\nГотово. Откройте монитор состояния в браузере — там видно все MSP-запросы.")


if __name__ == "__main__":
    sys.exit(main())
