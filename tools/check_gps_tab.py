#!/usr/bin/env python3
"""Вкладка «GPS»: почему она появляется и что в ней видно.

Configurator показывает вкладку «GPS» только если прошивка в MSP_BUILD_INFO сообщила
build-опцию USE_GPS (src/components/sidebar/sidebar_items.js: buildOptions: ["USE_GPS"]).
Здесь проверяем ровно то, что делает вкладка GpsTab.vue:

  * USE_GPS (16412) есть в списке build options — иначе вкладки нет вовсе;
  * фича GPS (бит 7) включена, «Порты» держат GPS на UART1, датчик GPS виден в MSP_STATUS;
  * загрузка вкладки: MSP_FEATURE_CONFIG → MSP_GPS_CONFIG → и опрос
    MSP_RAW_GPS → MSP_COMP_GPS → MSP_GPSSVINFO → MSP_ATTITUDE → MSP_RAW_IMU;
  * данные «живые»: координаты/курс/дистанция меняются между опросами;
  * «Сохранить» во вкладке (MSP_SET_FEATURE_CONFIG + MSP_SET_GPS_CONFIG) переживает перезапуск;
  * страница монитора и /api/state показывают то же состояние GPS.

Запуск:  python3 tools/check_gps_tab.py
"""
from __future__ import annotations

import json
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bfemu.fc import VirtualFC
from bfemu.msp import MspParser, encode_v1, encode_v2
from bfemu.server import HttpMonitor, TcpServer

PORT = 15794
HTTP_PORT = 8089

ok_count = 0
fail_count = 0


def check(cond, msg):
    global ok_count, fail_count
    if cond:
        ok_count += 1
        print(f"   [OK ] {msg}")
    else:
        fail_count += 1
        print(f"   [FAIL] {msg}")


class Client:
    def __init__(self, port=PORT, timeout=3.0):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.parser = MspParser()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def request(self, code: int, payload: bytes = b"", version: int = 1) -> bytes:
        frame = encode_v1(code, payload) if version == 1 else encode_v2(code, payload)
        self.sock.sendall(frame)
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                break
            if not data:
                break
            self.parser.feed(data)
            for req in self.parser:
                if req.code == code:
                    return req.payload
        raise TimeoutError(f"нет ответа на MSP {code}")

    def expect_close(self, seconds=6.0) -> bool:
        self.sock.settimeout(seconds)
        try:
            while True:
                if not self.sock.recv(4096):
                    return True
        except (socket.timeout, OSError):
            return True


def cli_over_msp(cli: Client, command: str) -> str:
    out = b""
    offset = 0
    while True:
        payload = command.encode() if offset == 0 else command.encode() + b"\x00" + struct.pack("<H", offset)
        resp = cli.request(0x3012, payload, version=2)
        total = int.from_bytes(resp[0:2], "little")
        out += resp[3:]
        offset += len(resp) - 3
        if offset >= total or len(resp) - 3 == 0:
            break
    return out.decode("latin-1", "replace")


def build_options(build: bytes) -> list:
    codes, pos = [], 26
    while pos + 1 < len(build):
        code = int.from_bytes(build[pos : pos + 2], "little")
        pos += 2
        if code == 0:
            break
        codes.append(code)
    return codes


def raw_gps(payload: bytes) -> dict:
    lat = struct.unpack_from("<i", payload, 2)[0] / 1e7
    lon = struct.unpack_from("<i", payload, 6)[0] / 1e7
    return {
        "fix": payload[0],
        "sats": payload[1],
        "lat": lat,
        "lon": lon,
        "alt_m": int.from_bytes(payload[10:12], "little"),
        "speed_cms": int.from_bytes(payload[12:14], "little"),
        "course_deg": int.from_bytes(payload[14:16], "little") / 10,
        "pdop": int.from_bytes(payload[16:18], "little") / 100,
    }


def main() -> int:
    workdir = Path(tempfile.mkdtemp(prefix="bfemu-gps-"))
    eeprom = workdir / "eeprom.json"
    fc = VirtualFC(eeprom_path=eeprom)
    port, http_port = PORT, HTTP_PORT
    server = TcpServer(fc, "127.0.0.1", port, boot_delay=0.3, verbose=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    monitor = HttpMonitor(fc, http_port, strict_port=True)
    monitor.prepare()
    threading.Thread(target=monitor.serve_forever, daemon=True).start()
    time.sleep(0.7)
    try:
        cli = Client()

        print("\n1. Почему вкладки «GPS» не было и почему она теперь есть")
        codes = build_options(cli.request(5))
        check(16412 in codes,
              "MSP_BUILD_INFO сообщает USE_GPS (16412) — sidebar_items.js показывает вкладку «GPS»")
        check(16420 in codes, "остальные build-опции на месте (USE_SERVOS 16420 — вкладка «Сервоприводы»)")

        features = struct.unpack_from("<I", cli.request(36))[0]
        check(features & (1 << 7), f"фича GPS (бит 7) включена с завода (маска 0x{features:05x})")
        check("GPS" in cli_over_msp(cli, "feature"), "CLI 'feature': GPS в списке включённых")
        ports = cli.request(0x1009, version=2)
        uart1 = None
        pos = 1
        for _ in range(ports[0]):
            if ports[pos] == 0:
                uart1 = struct.unpack_from("<I", ports, pos + 1)[0]
            pos += 9
        check(uart1 is not None and uart1 & (1 << 1),
              "«Порты»: у UART1 включена функция GPS (бит 1, serialPortFunction_e)")
        sensors = struct.unpack_from("<H", cli.request(150), 4)[0]
        check(sensors & (1 << 3), f"MSP_STATUS: датчик GPS найден (маска 0x{sensors:02x})")

        print("\n2. Загрузка вкладки (порядок запросов как в GpsTab.vue)")
        feats = cli.request(36)
        check(len(feats) == 4, "MSP_FEATURE_CONFIG: 4 байта — вкладка берёт состояние фич")
        cfg = cli.request(132)
        check(len(cfg) == 6, "MSP_GPS_CONFIG: 6 байт (provider, sbas, auto_config, auto_baud, home_once, galileo)")
        check(cfg[0] == 1, f"провайдер {cfg[0]} = UBLOX (в модуле виртуальный UBLOX на UART1)")
        first = raw_gps(cli.request(106))
        check(first["fix"] == 2, "MSP_RAW_GPS: fix = 2 (STATE(GPS_FIX)) — вкладка покажет «фикс есть»")
        check(6 <= first["sats"] <= 12, f"спутников в решении: {first['sats']}")
        check(48.1 < first["lat"] < 48.2 and 11.5 < first["lon"] < 11.7,
              f"координаты: {first['lat']:.6f}, {first['lon']:.6f}")
        check(first["alt_m"] > 0 and 0 < first["speed_cms"] < 2000 and 0 <= first["course_deg"] < 360,
              f"высота {first['alt_m']} м, скорость {first['speed_cms']} см/с, курс {first['course_deg']:.0f}°")
        check(0.5 < first["pdop"] < 5, f"pdop = {first['pdop']} (вкладка показывает качество решения)")
        comp = cli.request(107)
        check(len(comp) == 5, "MSP_COMP_GPS: 5 байт")
        check(0 < int.from_bytes(comp[0:2], "little") < 1000,
              f"до дома {int.from_bytes(comp[0:2], 'little')} м, "
              f"направление {int.from_bytes(comp[2:4], 'little')}°")
        sv = cli.request(164)
        rows = [tuple(sv[1 + 4 * i : 5 + 4 * i]) for i in range(sv[0])]
        check(sv[0] == 24 and len(rows) == 24, f"MSP_GPSSVINFO: {sv[0]} спутников")
        check(sv[0] > 16,
              "спутников больше 16 — Configurator включает подробную таблицу сигналов "
              "(при 16 и меньше рисует упрощённую: прочерки в Gnss ID и числовое качество)")
        names = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 6: "Glonass"}
        systems = {names.get(r[0], "?") for r in rows}
        check(systems == {"GPS", "SBAS", "Galileo", "Glonass", "BeiDou"},
              f"все системы, которые знает Configurator (gnssArray): {sorted(systems)}")
        check(all(r[0] < 7 for r in rows), "нет строк со «система >= 7» — такие Configurator "
                                           "рисует как пустые (gnssId >= 7 → прочерк)")
        used = [r for r in rows if r[2] & 0x08]
        check(len(used) == first["sats"], f"«используется» ровно у {len(used)} спутников — "
                                          f"столько же, сколько в MSP_RAW_GPS (numSat)")
        strongest = max(rows, key=lambda r: r[3])
        weakest = min(rows, key=lambda r: r[3])
        check(strongest[2] & 0x08 and not weakest[2] & 0x08 and all(r[3] <= 55 for r in rows),
              f"в решении — самые сильные (C/N0 {strongest[3]}), самые слабые — нет (C/N0 {weakest[3]})")
        check(len(cli.request(108)) == 6 and len(cli.request(107)) and len(cli.request(110)) > 0,
              "MSP_ATTITUDE / MSP_RAW_IMU отвечают — вкладка не ждёт таймаут")

        print("\n3. Данные живые, а не замороженные")
        time.sleep(1.2)
        second = raw_gps(cli.request(106))
        comp2 = cli.request(107)
        d1, d2 = int.from_bytes(comp[0:2], "little"), int.from_bytes(comp2[0:2], "little")
        check(second["course_deg"] != first["course_deg"] or d2 != d1,
              f"курс/дистанция меняются: {first['course_deg']:.0f}° → {second['course_deg']:.0f}°, "
              f"{d1} м → {d2} м")

        print("\n4. «Сохранить» во вкладке: фичи и конфиг GPS переживают перезапуск")
        # так делает вкладка: сначала MSP_SET_FEATURE_CONFIG (переключатель GPS),
        # затем MSP_SET_GPS_CONFIG, затем обычное «сохранить и перезагрузить».
        cli.request(37, ((features & ~(1 << 7)) | (1 << 7)).to_bytes(4, "little"))   # GPS остаётся включён
        cli.request(223, bytes([1, 1, 1, 0, 1, 0]))    # UBLOX, SBAS EGNOS, auto_config, home_once, galileo OFF
        cli.request(250)
        time.sleep(0.3)
        cli.request(68, bytes([0]))
        cli.expect_close()
        cli.close()
        time.sleep(0.8)

        fc2 = VirtualFC(eeprom_path=eeprom)
        saved = fc2.settings
        check([saved["gps_provider"], saved["gps_sbas_mode"], saved["gps_auto_config"],
               saved["gps_auto_baud"], saved["gps_set_home_point_once"], saved["gps_ublox_use_galileo"]]
              == [1, 1, 1, 0, 1, 0],
              "конфигурация GPS из вкладки лежит во «флеше» (провайдер, SBAS, Galileo)")
        check("GPS" in fc2.features, "фича GPS после перезапуска на месте — вкладка не пропадёт")

        server2 = TcpServer(fc2, "127.0.0.1", port + 1, boot_delay=0.3, verbose=False)
        threading.Thread(target=server2.serve_forever, daemon=True).start()
        time.sleep(0.5)
        cli2 = Client(port=port + 1)
        check(list(cli2.request(132)) == [1, 1, 1, 0, 1, 0],
              "после перезапуска MSP_GPS_CONFIG отдаёт сохранённое")
        check(cli2.request(106)[0] == 2, "GPS по-прежнему с фиксом (приёмник не «отвалился»)")
        cli2.request(37, (features & ~(1 << 7)).to_bytes(4, "little"))     # выключить GPS, как в вкладке
        check(not (struct.unpack_from("<I", cli2.request(36))[0] & (1 << 7)),
              "переключатель GPS во вкладке выключает фичу (маска без бита 7)")
        feat_line = cli_over_msp(cli2, "feature").splitlines()[0]
        check(feat_line.startswith("Enabled:") and "GPS" not in feat_line,
              f"CLI 'feature': GPS больше не в списке включённых ({feat_line.strip()})")
        cli2.close()
        server2.stop()

        print("\n5. Монитор эмулятора показывает то же состояние")
        import urllib.request
        with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/state", timeout=5) as answer:
            state = json.loads(answer.read())
        gps = state.get("gps") or {}
        check(gps.get("use_gps_build_option") and gps.get("feature_gps"),
              "api/state: build-опция USE_GPS и фича GPS")
        check(gps.get("fix") == 2 and 6 <= gps.get("sats", 0) <= 24,
              f"api/state: fix={gps.get('fix')}, спутников {gps.get('sats')}")
        check(len(gps.get("satellites") or []) == 24, "api/state: список спутников из MSP_GPSSVINFO")
        page = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/", timeout=5).read().decode()
        check("GPS (приёмник на UART1)" in page and "USE_GPS" in page,
              "страница монитора: карточка GPS с build-опцией USE_GPS")

        print()
        if fail_count:
            print(f"ПРОВАЛЕНО проверок: {fail_count}")
            return 1
        print(f"ВКЛАДКА «GPS» РАБОТАЕТ: build-опция USE_GPS, живой фикс, спутники, сохранение — "
              f"{ok_count} проверок, 0 провалов")
        return 0
    finally:
        server.stop()
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
