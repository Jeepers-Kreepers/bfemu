#!/usr/bin/env python3
"""
Демо-сессия: «Configurator» в терминнале — рукопожатие, опрос вкладок, настройки.

    python3 tools/demo_session.py            # свой эмулятор на 127.0.0.1:5761
    python3 tools/demo_session.py --port 5762
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from inavemu import msp  # noqa: E402
from inavemu.fc import *  # noqa: E402,F403


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5761)
    args = ap.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=5)
    parser = msp.Parser()
    queue: list = []

    def ask(code, payload=b""):
        sock.sendall(msp.encode_v2(code, payload))
        deadline = time.time() + 5
        while time.time() < deadline:
            while queue:
                frame = queue.pop(0)
                if frame.code == code:
                    return frame.payload
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                break
            queue.extend(parser.feed(data))
        raise TimeoutError(f"нет ответа на 0x{code:04X}")

    print("— рукопожатие (как serial_backend.js) —")
    api = ask(MSP_API_VERSION)
    print(f"  MSP API      : {api[1]}.{api[2]}.0 (протокол {api[0]})")
    print(f"  вариант      : {ask(MSP_FC_VARIANT).decode()}")
    ver = ask(MSP_FC_VERSION)
    print(f"  версия FC    : {ver[0]}.{ver[1]}.{ver[2]}")
    print(f"  имя модели   : {ask(MSP_NAME).decode()}")
    print(f"  build        : {ask(MSP_BUILD_INFO)[:19].decode()}")
    board = ask(MSP_BOARD_INFO)
    tlen = board[8]
    print(f"  плата        : {board[:4].decode()} / таргет {board[9:9 + tlen].decode()}")
    uid = ask(MSP_UID)
    print(f"  UID          : {uid.hex()}")

    print("— вкладки —")
    boxes = ask(MSP_BOXIDS)
    print(f"  Modes        : {len(boxes)} боксов (первый id={boxes[0]})")
    ports = ask(MSP2_CF_SERIAL_CONFIG)
    print(f"  Ports        : {len(ports) // 9} портов по {9} байт")
    rc = ask(MSP_RC)
    ch = struct.unpack("<" + "H" * (len(rc) // 2), rc)
    print(f"  Receiver     : каналы {ch[:8]}")
    misc = ask(MSP2_INAV_MISC)
    midrc = struct.unpack_from("<h", misc, 0)[0]
    print(f"  Configuration: midrc={midrc}")
    st = ask(MSP2_INAV_STATUS)
    print(f"  Status       : cycleTime={struct.unpack_from('<H', st, 0)[0]} мкс")
    analog = ask(MSP2_INAV_ANALOG)
    voltage = struct.unpack_from("<H", analog, 1)[0] / 100
    print(f"  Батарея      : {voltage:.2f} В")
    att = ask(MSP_ATTITUDE)
    roll, pitch, yaw = struct.unpack("<hhh", att)
    print(f"  attitude     : roll={roll / 10:.1f}° pitch={pitch / 10:.1f}° yaw={yaw}°")

    print("— настройки —")
    for name in ("looptime", "gyro_filter_mode", "applied_defaults"):
        info = ask(MSP2_COMMON_SETTING_INFO, name.encode() + b"\x00")
        idx_off = len(name) + 1 + 2 + 1 + 1 + 1 + 4 + 4
        idx = struct.unpack_from("<H", info, idx_off)[0]
        val = ask(MSP2_COMMON_SETTING, bytes([0]) + struct.pack("<H", idx))
        print(f"  {name:22s} индекс {idx:3d}  значение {val.hex()}")

    sock.close()
    print("\nготово — эмулятор отвечает как настоящий FC INAV")


if __name__ == "__main__":
    main()
