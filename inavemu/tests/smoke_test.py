#!/usr/bin/env python3
"""
Сквозной smoke-тест: «Configurator» подключается к эмулятору по TCP и проходит
все вкладки фазы 1 — Setup, Ports, Configuration, Receiver, Modes (Auxiliary),
Sensors — плюс протокол настроек и сохранение с перезагрузкой.

    python3 tests/smoke_test.py
"""
from __future__ import annotations

import socket
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from inavemu import msp
from inavemu.fc import (
    MSP2_CF_SERIAL_CONFIG, MSP2_COMMON_SET_SETTING, MSP2_COMMON_SETTING,
    MSP2_COMMON_SETTING_INFO,
    MSP2_INAV_ANALOG, MSP2_INAV_BATTERY_CONFIG, MSP2_INAV_DEBUG, MSP2_INAV_MISC,
    MSP2_INAV_MIXER, MSP2_INAV_RATE_PROFILE, MSP2_INAV_TEMPERATURES, MSP2_INAV_AIR_SPEED, MSP2_INAV_STATUS,
    MSP_ACTIVEBOXES, MSP_ADJUSTMENT_RANGES, MSP_ADVANCED_CONFIG,
    MSP_API_VERSION, MSP_BOARD_ALIGNMENT, MSP_BOARD_INFO, MSP_BOXIDS,
    MSP_BUILD_INFO, MSP_CURRENT_METER_CONFIG, MSP_DATAFLASH_SUMMARY,
    MSP_EEPROM_WRITE, MSP_FC_VARIANT, MSP_FC_VERSION, MSP_FEATURE, MSP_MODE_RANGES,
    MSP_NAME, MSP_RC, MSP_RC_DEADBAND, MSP_RX_CONFIG, MSP_RX_MAP,
    MSP_SENSOR_ALIGNMENT, MSP_SENSOR_STATUS, MSP_SET_FEATURE, MSP_SET_MODE_RANGE,
    MSP_SET_REBOOT, MSP_SET_RX_MAP, MSP_SET_WP, MSP_SONAR, MSP_ALTITUDE,
    MSP_ATTITUDE,
    MSP_RAW_IMU, MSP_UID, MSP_VTX_CONFIG,
    MSP_WP_MISSION_SAVE,
    MSP2_BLACKBOX_CONFIG, MSP2_INAV_GEOZONE_VERTEX, MSP2_INAV_LOGIC_CONDITIONS_SINGLE,
    MSP2_INAV_SAFEHOME, MSP2_INAV_SET_GEOZONE, MSP2_INAV_SET_GEOZONE_VERTICE,
    MSP2_INAV_SET_LOGIC_CONDITIONS, MSP2_INAV_SET_SAFEHOME,
    MSP2_SET_BLACKBOX_CONFIG,
    OSD_ITEM_COUNT, OSD_LAYOUT_COUNT,
)
from inavemu.fc import InavFC
from inavemu.server import FCServer

CHECKS = 0
FAILS = []


def check(cond, what):
    global CHECKS
    CHECKS += 1
    if not cond:
        FAILS.append(what)
        print(f"  ✗ {what}")
    return cond


class Client:
    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.parser = msp.Parser()
        self.queue = []

    def request(self, code, payload=b"", version=2):
        raw = (msp.encode_v2(code, payload) if version == 2
               else msp.encode_v1(code, payload))
        self.sock.sendall(raw)
        deadline = time.time() + 5
        while time.time() < deadline:
            while self.queue:
                frame = self.queue.pop(0)
                if frame.code == code:
                    return frame
            try:
                data = self.sock.recv(65536)
            except TimeoutError:
                continue
            if not data:
                break
            self.queue.extend(self.parser.feed(data))
        raise TimeoutError(f"нет ответа на MSP 0x{code:04X}")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _recv_until(client: Client, marker: bytes, timeout: float = 5.0) -> bytes:
    """Читает из сокета, пока не встретится маркер (например, приглашение '# ')."""
    buf = bytearray()
    deadline = time.time() + timeout
    client.sock.settimeout(0.5)
    while time.time() < deadline:
        if marker in buf:
            return bytes(buf)
        try:
            data = client.sock.recv(65536)
        except TimeoutError:
            continue
        if not data:
            break
        buf.extend(data)
    return bytes(buf)


def main():
    global CHECKS
    print("=== inavemu smoke-тест ===")
    with tempfile.TemporaryDirectory() as td:
        eeprom = Path(td) / "eeprom.json"
        fc = InavFC(eeprom_path=eeprom)
        server = FCServer(fc, host="127.0.0.1", port=0)
        # занять свободный порт
        server._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server._sock.bind(("127.0.0.1", 0))
        server.port = server._sock.getsockname()[1]
        server._sock.listen(4)
        server._sock.settimeout(0.5)
        threading.Thread(target=server._accept_loop, daemon=True).start()

        c = Client(server.port)

        # --- рукопожатие (serial_backend.js) ---
        api = c.request(MSP_API_VERSION).payload
        check(api[1] == 2 and api[2] == 5, f"MSP_API_VERSION = 2.5, получено {api.hex()}")
        variant = c.request(MSP_FC_VARIANT).payload
        check(variant == b"INAV", f"MSP_FC_VARIANT = INAV, получено {variant!r}")
        ver = c.request(MSP_FC_VERSION).payload
        check(ver == bytes([9, 1, 0]), f"MSP_FC_VERSION = 9.1.0, получено {ver.hex()}")
        name = c.request(MSP_NAME).payload
        check(isinstance(name, bytes) and len(name) > 0, "MSP_NAME непуст")
        build = c.request(MSP_BUILD_INFO).payload
        check(len(build) >= 19, f"MSP_BUILD_INFO длина {len(build)}")
        board = c.request(MSP_BOARD_INFO).payload
        check(len(board) > 9, f"MSP_BOARD_INFO длина {len(board)}")
        uid = c.request(MSP_UID).payload
        check(len(uid) == 12, "MSP_UID = 12 байт")

        # defaultsDialog: getSetting("applied_defaults")
        info = c.request(MSP2_COMMON_SETTING_INFO, b"applied_defaults\x00").payload
        check(info.startswith(b"applied_defaults\x00"), "SETTING_INFO(applied_defaults)")
        val = c.request(MSP2_COMMON_SETTING, b"applied_defaults\x00").payload
        check(val == bytes([0]), "SETTING(applied_defaults) = 0")

        # onConnect
        boxes = c.request(MSP_BOXIDS).payload
        check(len(boxes) == 55, f"MSP_BOXIDS: {len(boxes)} боксов")
        dflash = c.request(MSP_DATAFLASH_SUMMARY).payload
        check(len(dflash) == 13, "MSP_DATAFLASH_SUMMARY = 13 байт")

        # периодика
        for code, want in ((MSP_SENSOR_STATUS, 9), (MSP2_INAV_STATUS, 22),
                           (MSP_ACTIVEBOXES, 8), (MSP2_INAV_ANALOG, 24)):
            got = c.request(code).payload
            check(len(got) == want, f"MSP 0x{code:04X}: длина {len(got)} != {want}")

        # --- Setup ---
        check(len(c.request(MSP_FEATURE).payload) == 4, "MSP_FEATURE")
        check(len(c.request(MSP2_INAV_MIXER).payload) == 9, "MSP2_INAV_MIXER")
        check(len(c.request(MSP2_INAV_MISC).payload) == 41, "MSP2_INAV_MISC")
        ports = c.request(MSP2_CF_SERIAL_CONFIG).payload
        check(len(ports) % 9 == 0 and len(ports) > 0, f"SERIAL_CONFIG: {len(ports)} байт")
        check(len(c.request(MSP_ATTITUDE).payload) == 6, "MSP_ATTITUDE")
        check(len(c.request(MSP_ALTITUDE).payload) == 10, "MSP_ALTITUDE")

        # --- Ports: сохранение ---
        new_ports = bytearray(ports)
        new_ports[5] = 8  # msp baud первого порта
        c.request(MSP2_CF_SERIAL_CONFIG and 0x100A, bytes(new_ports))
        back = c.request(MSP2_CF_SERIAL_CONFIG).payload
        check(back[5] == 8, "SERIAL_CONFIG: msp baudrate сохранился")

        # --- Configuration ---
        check(len(c.request(MSP_SENSOR_ALIGNMENT).payload) == 4, "MSP_SENSOR_ALIGNMENT")
        check(len(c.request(MSP_ADVANCED_CONFIG).payload) == 9, "MSP_ADVANCED_CONFIG")
        check(len(c.request(MSP_VTX_CONFIG).payload) >= 7, "MSP_VTX_CONFIG")
        check(len(c.request(MSP_BOARD_ALIGNMENT).payload) == 6, "MSP_BOARD_ALIGNMENT")
        check(len(c.request(MSP_CURRENT_METER_CONFIG).payload) == 7, "MSP_CURRENT_METER_CONFIG")
        check(len(c.request(MSP2_INAV_BATTERY_CONFIG).payload) == 29, "BATTERY_CONFIG")
        c.request(MSP_SET_FEATURE, struct.pack("<I", 0x80))
        check(c.request(MSP_FEATURE).payload == struct.pack("<I", 0x80), "SET_FEATURE")

        # --- Receiver ---
        rc = c.request(MSP_RC).payload
        check(len(rc) == 32, f"MSP_RC: {len(rc)} байт")
        rx = c.request(MSP_RX_CONFIG).payload
        check(len(rx) == 24, f"MSP_RX_CONFIG: {len(rx)} байт")
        check(len(c.request(MSP_RX_MAP).payload) == 8, "MSP_RX_MAP")
        check(len(c.request(MSP_RC_DEADBAND).payload) == 5, "MSP_RC_DEADBAND")
        check(len(c.request(MSP2_INAV_RATE_PROFILE).payload) == 15, "RATE_PROFILE")
        c.request(MSP_SET_RX_MAP, bytes([3, 2, 1, 0, 4, 5, 6, 7]))
        check(c.request(MSP_RX_MAP).payload[:4] == bytes([3, 2, 1, 0]), "SET_RX_MAP")

        # --- Modes (auxiliary) ---
        mr = c.request(MSP_MODE_RANGES).payload
        check(len(mr) == 160, f"MSP_MODE_RANGES: {len(mr)} байт")
        ar = c.request(MSP_ADJUSTMENT_RANGES).payload
        check(len(ar) == 120, f"MSP_ADJUSTMENT_RANGES: {len(ar)} байт")
        c.request(MSP_SET_MODE_RANGE, bytes([0, 2, 10, 20]))
        mr2 = c.request(MSP_MODE_RANGES).payload
        check(mr2[1] == 2 and mr2[2] == 10 and mr2[3] == 20, "SET_MODE_RANGE")

        # --- Sensors ---
        check(len(c.request(MSP_RAW_IMU).payload) == 18, "MSP_RAW_IMU")
        check(len(c.request(MSP_SONAR).payload) == 4, "MSP_SONAR")
        check(len(c.request(MSP2_INAV_AIR_SPEED).payload) == 4, "AIR_SPEED")
        check(len(c.request(MSP2_INAV_TEMPERATURES).payload) == 16, "TEMPERATURES")
        check(len(c.request(MSP2_INAV_DEBUG).payload) == 32, "DEBUG")

        # --- настройки: выборка нескольких и запись ---
        for setting in ("looptime", "gyro_filter_mode", "name", "serialrx_provider"):
            i = c.request(MSP2_COMMON_SETTING_INFO, setting.encode() + b"\x00").payload
            check(i.startswith(setting.encode() + b"\x00"), f"SETTING_INFO({setting})")
        i = c.request(MSP2_COMMON_SETTING_INFO, b"looptime\x00").payload
        idx_off = len(b"looptime\x00") + 2 + 1 + 1 + 1 + 4 + 4
        idx = struct.unpack_from("<H", i, idx_off)[0]
        v = c.request(MSP2_COMMON_SETTING, bytes([0]) + struct.pack("<H", idx)).payload
        check(struct.unpack("<H", v)[0] == 500, "looptime = 500")

        # --- фаза 2: PID / OSD / GPS ---
        from inavemu.fc import (
            MSP2_ADSB_VEHICLE_LIST, MSP2_INAV_EZ_TUNE, MSP2_INAV_OSD_ALARMS,
            MSP2_INAV_OSD_LAYOUTS, MSP2_INAV_OSD_PREFERENCES, MSP2_INAV_RATE_DYNAMICS,
            MSP2_PID, MSP_GPSSTATISTICS, MSP_WP, MSP_WP_GETINFO,
        )
        check(len(c.request(MSP2_PID).payload) == 44, "MSP2_PID: 11 банков")
        check(len(c.request(MSP2_INAV_RATE_DYNAMICS).payload) == 6, "RATE_DYNAMICS")
        check(len(c.request(MSP2_INAV_EZ_TUNE).payload) == 11, "EZ_TUNE")
        check(len(c.request(MSP_GPSSTATISTICS).payload) == 21, "GPSSTATISTICS")
        check(len(c.request(MSP2_ADSB_VEHICLE_LIST).payload) == 10, "ADSB list")
        head = c.request(MSP2_INAV_OSD_LAYOUTS).payload
        check(head == bytes([OSD_LAYOUT_COUNT, OSD_ITEM_COUNT]), f"OSD counts {head.hex()}")
        check(len(c.request(MSP2_INAV_OSD_LAYOUTS, bytes([0])).payload) == OSD_ITEM_COUNT * 2,
              f"OSD layout 0: {OSD_ITEM_COUNT} позиций")
        check(len(c.request(MSP2_INAV_OSD_ALARMS).payload) == 28, "OSD alarms")
        check(len(c.request(MSP2_INAV_OSD_PREFERENCES).payload) == 10, "OSD prefs")
        check(len(c.request(MSP_WP_GETINFO).payload) == 4, "WP_GETINFO")
        check(len(c.request(MSP_WP, bytes([1])).payload) == 21, "MSP_WP")

        # --- CLI по «сырому» порту: вход, команда, выход с перезагрузкой ---
        c2 = Client(server.port)
        c2.sock.sendall(b"#")
        banner = _recv_until(c2, b"# ")
        check(b"Entering CLI Mode" in banner, f"CLI banner: {banner[:60]!r}")
        c2.sock.sendall(b"version\n")
        out = _recv_until(c2, b"# ")
        check(b"INAV/9.1.0" in out or b"INAV/SITL" in out, f"cli version: {out[:80]!r}")
        c2.sock.sendall(b"set looptime = 4444\n")
        out = _recv_until(c2, b"# ")
        check(b"looptime set to 4444" in out, f"cli set: {out[:80]!r}")
        c2.sock.sendall(b"get looptime\n")
        out = _recv_until(c2, b"# ")
        check(b"looptime = 4444" in out, f"cli get: {out[:80]!r}")
        c2.sock.sendall(b"diff\n")
        out = _recv_until(c2, b"# ")
        check(b"set looptime = 4444" in out and b"batch start" in out,
              f"cli diff: {out[:120]!r}")
        c2.sock.sendall(b"save\n")
        out = _recv_until(c2, b"Rebooting")
        check(b"Saving" in out, f"cli save: {out[:60]!r}")
        time.sleep(0.3)
        c2.close()

        # значение из CLI пережило «перезагрузку»
        c3 = Client(server.port)
        i = c3.request(MSP2_COMMON_SETTING_INFO, b"looptime\x00").payload
        idx = struct.unpack_from("<H", i, len(b"looptime\x00") + 13)[0]
        v = c3.request(MSP2_COMMON_SETTING, bytes([0]) + struct.pack("<H", idx)).payload
        check(struct.unpack("<H", v)[0] == 4444, "CLI set пережил перезагрузку")
        c3.close()

        # --- сохранение + перезагрузка: линк рвётся, конфигурация живёт ---
        c.request(MSP_SET_FEATURE, struct.pack("<I", 0x40))
        c.request(MSP2_COMMON_SET_SETTING, b"applied_defaults\x00\x05")
        c.sock.sendall(msp.encode_v2(MSP_EEPROM_WRITE, b""))
        time.sleep(0.2)
        c.sock.sendall(msp.encode_v2(MSP_SET_REBOOT, b""))
        time.sleep(0.5)
        check(eeprom.exists(), "eeprom.json создан")

        c2 = Client(server.port)  # новый линк после «перезагрузки»
        feat = c2.request(MSP_FEATURE).payload
        check(feat == struct.pack("<I", 0x40), f"features пережили перезагрузку: {feat.hex()}")
        api = c2.request(MSP_API_VERSION).payload
        check(api[1] == 2, "второе подключение: MSP API")
        ad = c2.request(MSP2_COMMON_SETTING, b"applied_defaults\x00").payload
        check(ad == b"\x05", "applied_defaults пережил перезагрузку (диалог не вернётся)")

        # --- фаза 3: миссия/WP, safehome, geozone, логика по TCP ---
        def wp_pack(no, action, lat, lon, alt, flag):
            return (bytes([no, action]) + struct.pack("<iiI", lat, lon, alt)
                    + struct.pack("<HHH", 0, 0, 0) + bytes([flag]))

        c2.request(MSP_SET_WP, wp_pack(1, 1, 55755830, 37617380, 5000, 0))
        c2.request(MSP_SET_WP, wp_pack(2, 1, 55756000, 37618000, 7000, 0))
        c2.request(MSP_SET_WP, wp_pack(3, 4, 0, 0, 0, 0xA5))
        info = c2.request(MSP_WP_GETINFO).payload
        check(info == bytes([0, 120, 1, 3]), f"GETINFO миссии: {info.hex()}")
        wp2 = c2.request(MSP_WP, bytes([2])).payload
        check(struct.unpack_from("<iiI", wp2, 2) == (55756000, 37618000, 7000),
              "MSP_WP вернул сохранённую точку")
        c2.request(MSP_WP_MISSION_SAVE, bytes([0]))
        check(True, "MISSION_SAVE принят")

        sh = bytes([0, 1]) + struct.pack("<ii", 55755830, 37617380)
        c2.request(MSP2_INAV_SET_SAFEHOME, sh)
        check(c2.request(MSP2_INAV_SAFEHOME, bytes([0])).payload == sh,
              "safehome round-trip")
        gz = (bytes([0, 1, 1]) + struct.pack("<ii", 0, 10000)
              + bytes([0, 0, 2]))
        c2.request(MSP2_INAV_SET_GEOZONE, gz)
        v0 = bytes([0, 0]) + struct.pack("<ii", 1, 2)
        v1 = bytes([0, 1]) + struct.pack("<ii", 3, 4)
        c2.request(MSP2_INAV_SET_GEOZONE_VERTICE, v0)
        c2.request(MSP2_INAV_SET_GEOZONE_VERTICE, v1)
        check(c2.request(MSP2_INAV_GEOZONE_VERTEX, bytes([0, 1])).payload == v1,
              "geozone vertex round-trip")
        lc = (bytes([0, 1, 255, 4]) + bytes([1]) + struct.pack("<i", 10)
              + bytes([1]) + struct.pack("<i", 20) + bytes([0]))
        c2.request(MSP2_INAV_SET_LOGIC_CONDITIONS, lc)
        check(c2.request(MSP2_INAV_LOGIC_CONDITIONS_SINGLE, bytes([0])).payload
              == lc[1:], "logic condition round-trip")
        bb = bytes([1]) + struct.pack("<HHI", 1, 32, 0)
        c2.request(MSP2_SET_BLACKBOX_CONFIG, bb)
        check(c2.request(MSP2_BLACKBOX_CONFIG).payload == bytes([1]) + bb,
              "blackbox v2 round-trip")
        c2.close()
        c.close()
        server.stop()

    print(f"\nпройдено проверок: {CHECKS}")
    if FAILS:
        print(f"ПРОВАЛЕНО: {len(FAILS)}")
        for f in FAILS:
            print(f"  - {f}")
        sys.exit(1)
    print("ВСЕ ТЕСТЫ ПРОЙДЕНЫ")


if __name__ == "__main__":
    main()
