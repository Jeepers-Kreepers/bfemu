#!/usr/bin/env python3
"""
Сквозной тест виртуального FC по TCP: повторяем ровно те последовательности,
которые делает Betaflight Configurator (см. src/js/serial_backend.js), затем
проверяем оба канала CLI.

Запуск:  python3 tests/smoke_test.py
"""
from __future__ import annotations

import builtins
import inspect
import json
import os
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
from bfemu.fc import (ADJUSTMENT_FUNCTION_COUNT, ADJUSTMENT_RANGE_COUNT, OSD_ITEM_COUNT,
                      OSD_PROFILE_COUNT, OSD_STAT_COUNT, OSD_TIMER_COUNT, OSD_VISIBLE,
                      OSD_WARNING_COUNT)

PORT = 15761
HOST = "127.0.0.1"

failures: list[str] = []


def check(cond, msg):
    status = "OK  " if cond else "FAIL"
    print(f"  [{status}] {msg}")
    if not cond:
        failures.append(msg)


class Client:
    """Мини-Configurator: тот же MSP-протокол, что в src/js/msp.js."""

    def __init__(self, host=HOST, port=PORT, timeout=3.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.parser = MspParser()
        self.pending: list = []
        self.raw = bytearray()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def send_raw(self, data: bytes):
        self.sock.sendall(data)

    def request(self, code: int, payload: bytes = b"", version: int = 1) -> bytes:
        """Отправляет MSP-запрос и ждёт ответ того же протокола."""
        frame = encode_v1(code, payload) if version == 1 else encode_v2(code, payload)
        self.sock.sendall(frame)
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                data = self.sock.recv(65536)
            except TimeoutError:
                break
            if not data:
                break
            self.parser.feed(data)
            for req in self.parser:
                if req.code == code:
                    return req.payload
        raise TimeoutError(f"нет ответа на MSP {code}")

    def expect_close(self, seconds=5.0) -> bool:
        """Ждёт разрыва соединения (ребут FC)."""
        self.sock.settimeout(seconds)
        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    return True
        except TimeoutError:
            return False
        except OSError:
            return True

    def read_some(self, seconds=1.0) -> bytes:
        self.sock.settimeout(seconds)
        try:
            return self.sock.recv(65536)
        except TimeoutError:
            return b""


def u16(b, off=0):
    return struct.unpack_from("<H", b, off)[0]


def u32(data: bytes, pos: int) -> int:
    return int.from_bytes(data[pos:pos + 4], "little")


def get_text(data, pos=0):
    """self.getText() из MSPHelper.js: сначала длина, потом символы. Возвращает (строка, новая позиция)."""
    size = data[pos]
    pos += 1
    return data[pos : pos + size].decode("latin-1"), pos + size


def i32(b, off=0):
    return struct.unpack_from("<i", b, off)[0]


def i16(b, off=0):
    return int.from_bytes(b[off : off + 2], "little", signed=True)


def i16_bytes(value):
    return int(value).to_bytes(2, "little", signed=True)


def u16_bytes(value):
    return int(value).to_bytes(2, "little")


def cli_over_msp(cli: Client, command: str, window=480) -> str:
    """Аналог MSP.send_cli_command: пейджинг через MSP2_CLI_COMMAND."""
    out = b""
    offset = 0
    while True:
        payload = command.encode() if offset == 0 else command.encode() + b"\x00" + struct.pack("<H", offset)
        resp = cli.request(0x3012, payload, version=2)
        total = u16(resp, 0)
        flags = resp[2]
        out += resp[3:]
        offset += len(resp) - 3
        if offset >= total or (flags & 1 and offset >= total):
            break
        if len(resp) - 3 == 0:
            break
    return out.decode("latin-1", "replace")


def main():
    fc = VirtualFC()
    from bfemu.server import TcpServer

    server = TcpServer(fc, "127.0.0.1", PORT, boot_delay=0.5, verbose=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.3)

    print("1. Handshake (как serial_backend.js onOpen)")
    cli = Client()
    payload = cli.request(1)  # MSP_API_VERSION
    check(len(payload) == 3, "MSP_API_VERSION: 3 байта")
    check(f"{payload[1]}.{payload[2]}.0" == "1.46.0", f"API 1.46.0 (получено {payload[1]}.{payload[2]}.0)")

    variant = cli.request(2)
    check(variant == b"BTFL", f"MSP_FC_VARIANT = BTFL (получено {variant!r})")

    ver = cli.request(3)
    check(f"{ver[0]}.{ver[1]}.{ver[2]}" >= "4.5.4", f"MSP_FC_VERSION = {ver[0]}.{ver[1]}.{ver[2]} (>=4.5.4 нужен для MSP CLI)")

    build = cli.request(5)
    # 11 (дата) + 8 (время) + 7 (git) + 9 build-опций по 2 байта + нулевой u16 в конце.
    # Опции читает MSPHelper.js при API >= 1.46 — от них зависит набор вкладок (например «Сервоприводы»).
    check(len(build) == 26 + 10 * 2 + 2, f"MSP_BUILD_INFO: {len(build)} байт (включая build options)")

    board = cli.request(4)
    check(board[:4] == b"S405", "MSP_BOARD_INFO: идентификатор платы S405")
    # строки в этом ответе — с префиксом длины (sbufWriteU8(strlen) в msp.c),
    # иначе Configurator показывает мусор вместо названия платы
    target_name, pos = get_text(board, 8)
    board_name, pos = get_text(board, pos)
    manufacturer, pos = get_text(board, pos)
    check(target_name == "STM32F405", f"targetName разобран: {target_name!r}")
    check(board_name == "VIRTUALFC", f"boardName разобран: {board_name!r}")
    check(manufacturer == "BFR", f"manufacturerId разобран: {manufacturer!r}")
    check(board[pos : pos + 32] == bytes(32), "подпись: 32 нулевых байта (как без USE_SIGNATURE)")
    caps = board[7]
    check(caps & 1, f"возможности платы: VCP (бит 0, caps=0x{caps:02x})")
    check(caps & 2, "возможности платы: HAS_SOFTSERIAL (бит 1) — нужен вкладке «Порты»")
    check(len(board) == pos + 32 + 1 + 1 + 2 + 4,
          f"MSP_BOARD_INFO целиком разобран: {len(board)} байт, хвост 8 байт (MCU/состояние/частота/проблемы)")

    status = cli.request(101)
    check(len(status) == 13, "MSP_STATUS: 13 байт (включая gyro cycle time)")
    sensors = u16(status, 4)
    # порядок битов задаёт msp.c: acc0, baro1, mag2, gps3, rangefinder4, gyro5
    check(sensors & (1 << 0), "маска: acc (бит 0)")
    check(sensors & (1 << 5), "маска: gyro (бит 5)")
    check(sensors & (1 << 3), "маска: GPS (бит 3) — иначе Configurator не запросит MSP_RAW_GPS")

    uid = cli.request(160)
    check(len(uid) == 12, "MSP_UID: 12 байт")

    text = cli.request(0x3006, bytes([2]), version=2)  # MSP2_GET_TEXT CRAFT_NAME
    check(text[0] == 2, "MSP2_GET_TEXT: тип CRAFT_NAME")
    craft, _ = get_text(text, 1)
    check(craft == "EMU-QUAD", f"MSP2_GET_TEXT разобран как в Configurator: {craft!r}")

    cli.request(246, bytes(8), version=1)  # MSP_SET_RTC
    check(True, "MSP_SET_RTC принят")

    print("2. Живые данные (requestLiveData)")
    analog = cli.request(110)
    check(len(analog) == 9, "MSP_ANALOG: 9 байт (vbat u8 + mAh u16 + rssi u16 + A i16 + V u16)")
    check(u16(analog, 7) > 1000, f"MSP_ANALOG: напряжение {u16(analog, 7) / 100:.2f} В")
    battery = cli.request(130)
    check(len(battery) == 11, "MSP_BATTERY_STATE: 11 байт")
    boxnames = cli.request(116)
    check(b"ARM;" in boxnames, "MSP_BOXNAMES содержит ARM")
    names = [n for n in boxnames.decode("latin-1").split(";") if n]
    check(len(names) == len(cli.request(119)) == len(fc.active_boxes) and "BEEPER" in names
          and "TELEMETRY" in names and "MAG" not in names and "LEDLOW" not in names,
          f"MSP_BOXNAMES/BOXIDS: {len(names)} режимов как initActiveBoxIds() — BEEPER есть, "
          f"выключенные (MAG/LEDLOW) нет")
    mode_ranges = cli.request(34)
    mode_extra = cli.request(238)
    check(len(mode_ranges) == 80, f"MSP_MODE_RANGES: {len(mode_ranges)} байт = 20 слотов × 4 (прошивка шлёт весь массив)")
    check(mode_ranges[:4] == b"\x00\x00\x00\x00",
          "пустой слот режима — «0 0 0 0» (ARM, 900/900), вкладка «Режимы» строк не рисует")
    check(len(mode_extra) == 61, f"MSP_MODE_RANGES_EXTRA: {len(mode_extra)} байт (count 20 + 20 × 3)")
    check(cli.request(101)[6:10] == b"\x00\x00\x00\x00",
          "MSP_STATUS.CONFIG.mode пуст: тумблеры внизу — ни один режим не включён")
    status_ex = cli.request(150)
    check(len(status_ex) == 24, f"MSP_STATUS_EX: {len(status_ex)} байт (раскладка API 1.46)")
    arm_count = status_ex[16]
    arm_flags = struct.unpack_from("<I", status_ex, 17)[0]
    check(arm_count == 26, f"счётчик флагов arming = {arm_count} (ARMING_DISABLE_FLAGS_COUNT)")
    check(arm_flags == 1 << 25, f"флаг arming = ARM_SWITCH (0x{arm_flags:08x})")
    config_state = status_ex[21]
    cpu_temp = u16(status_ex, 22)
    check(config_state == 0 and 0 < cpu_temp < 100, f"состояние конфигурации {config_state}, температура ядра {cpu_temp} °C")
    gps = cli.request(106)
    check(len(gps) == 18, f"MSP_RAW_GPS: {len(gps)} байт (API 1.46 => с DOP)")
    check(gps[0] == 2, f"MSP_RAW_GPS: fix = {gps[0]} (2 = STATE(GPS_FIX) = 1 << 1, как в runtime_config.h)")
    check(6 <= gps[1] <= len(fc.GPS_SATELLITES), f"MSP_RAW_GPS: спутников в решении {gps[1]}")
    lat, lon = i32(gps, 2) / 1e7, i32(gps, 6) / 1e7
    check(48.0 < lat < 48.3 and 11.4 < lon < 11.7,
          f"MSP_RAW_GPS: координаты правдоподобны ({lat:.6f}, {lon:.6f})")
    check(u16(gps, 12) > 0 and u16(gps, 14) >= 0 and u16(gps, 16) > 0,
          f"MSP_RAW_GPS: высота {u16(gps, 12)} м, скорость {u16(gps, 14)} см/с, курс {u16(gps, 16) / 10:.0f}°, pdop {u16(gps, 16)}")
    comp = cli.request(107)
    check(len(comp) == 5, f"MSP_COMP_GPS: {len(comp)} байт (дистанция, направление, признак обновления)")
    check(0 < u16(comp) < 1000 and u16(comp, 2) < 360,
          f"MSP_COMP_GPS: до дома {u16(comp)} м, направление {u16(comp, 2)}°")
    sv = cli.request(164)
    check(sv[0] == len(fc.GPS_SATELLITES) and len(sv) == 1 + 4 * sv[0],
          f"MSP_GPSSVINFO: {sv[0]} спутников по 4 байта (система, номер, качество, C/N0)")
    check(sv[0] > 16,
          "спутников больше 16 — только тогда Configurator рисует подробную таблицу сигналов "
          "(имена систем, зелёные метки «используется», подписи качества)")
    rows = [tuple(sv[1 + 4 * i : 5 + 4 * i]) for i in range(sv[0])]
    check(all(row[3] <= 55 for row in rows) and any(row[2] & 0x08 for row in rows),
          f"качество сигнала и бит «используется» на месте: {rows[:3]}")
    gcfg = cli.request(132)
    check(len(gcfg) == 6 and gcfg[0] == 1,
          f"MSP_GPS_CONFIG: {list(gcfg)} — provider, sbas, auto_config, auto_baud, home_once, galileo")
    check(list(gcfg) == [fc._setting("gps_provider", 1), fc._setting("gps_sbas_mode", 0),
                   fc._setting("gps_auto_config", 1), fc._setting("gps_auto_baud", 0),
                   fc._setting("gps_set_home_point_once", 0), fc._setting("gps_ublox_use_galileo", 0)],
          "MSP_GPS_CONFIG читается из настроек (то же, что 'get gps_*' в CLI)")
    att = cli.request(108)
    check(len(att) == 6, "MSP_ATTITUDE: 6 байт")

    print("2б. Вкладка «Порты» (MSP2_COMMON_SERIAL_CONFIG)")
    vtx = cli.request(88)
    check(len(vtx) == 15, f"MSP_VTX_CONFIG: {len(vtx)} байт (раскладка API 1.42+)")
    ports = cli.request(0x1009, version=2)
    count = ports[0]
    check(count == 6, f"MSP2_COMMON_SERIAL_CONFIG: {count} портов")
    pos, parsed, names = 1, [], {0: "UART1", 1: "UART2", 2: "UART3", 3: "UART4", 4: "UART5", 20: "VCP"}
    for _ in range(count):
        ident = ports[pos]
        mask = struct.unpack_from("<I", ports, pos + 1)[0]
        bauds = list(ports[pos + 5 : pos + 9])
        parsed.append((names.get(ident, str(ident)), mask, bauds))
        pos += 9
    check(pos == len(ports), f"все порты разобраны байт в байт ({pos} из {len(ports)})")
    check(("UART2", 1 << 6, [5, 4, 0, 5]) in parsed, "UART2 помечен как RX_SERIAL (бит 6)")
    check(any(name == "VCP" and mask & 1 for name, mask, _ in parsed), "VCP держит MSP (бит 0)")
    check(parsed[0][0] == "VCP",
          f"USB VCP идёт первой строкой — как serialPortIdentifiers[] в io/serial.c "
          f"(порядок строк: {[n for n, _, _ in parsed]})")
    check(all(bauds == [5, 4, 0, 5] for _, _, bauds in parsed),
          "скорости заводские, как pgResetFn_serialConfig(): MSP 115200, датчики 57600, телеметрия AUTO, ЧЯ 115200")

    # сохранение портов — как кнопкой «Сохранить и перезагрузить»
    save = bytes([2]) + bytes([1, 1 << 6, 0, 0, 0, 3, 1, 0, 5]) + bytes([20, 1, 0, 0, 0, 5, 1, 0, 5])
    cli.request(0x100A, save, version=2)
    again = cli.request(0x1009, version=2)
    check(again[0] == 2, "после сохранения конфигурация портов обновилась (2 порта)")
    check("serial 20 1 115200" in cli_over_msp(cli, "serial"), "CLI dump показывает VCP с MSP")

    print("2б-3. Чёрный ящик — это настройка, а не фича (как в прошивке 4.5.4)")
    # В featureNames[] прошивки на месте BLACKBOX стоит пустая строка: фичи такой нет,
    # логирование включается настройкой blackbox_device (cli/settings.c: lookupTableBlackboxDevice).
    feature_list = cli_over_msp(cli, "feature list")
    check("BLACKBOX" not in feature_list,
          "в 'feature list' нет BLACKBOX — фичи BLACKBOX в прошивке 4.5.4 не существует")
    check("BLACKBOX" not in fc.feature_bits_by_name and "BLACKBOX" not in fc.features,
          "BLACKBOX нет и во внутреннем списке фич (бит 11 в featureNames[] пустой)")
    check("INVALID NAME: BLACKBOX" in cli_over_msp(cli, "feature BLACKBOX"),
          "'feature BLACKBOX' отвечает ошибкой — как cli.c: ERROR_INVALID_NAME")
    device = cli_over_msp(cli, "get blackbox_device")
    check("blackbox_device = SPIFLASH" in device and "NONE, SPIFLASH, SDCARD, SERIAL" in device,
          f"логирование включено настройкой: {device.strip().splitlines()[0]}")
    dump = cli_over_msp(cli, "dump")
    check("set blackbox_device = SPIFLASH" in dump and "set blackbox_sample_rate" in dump,
          "dump показывает set blackbox_device = SPIFLASH (и скорость записи)")
    unknown = cli_over_msp(cli, "aux 8 11 1 1300 1700").strip()
    check(unknown == "aux 8 0 0 900 900 0 0",
          f"unknown permanentId 11 (удалённый GPS HOLD) не пишется — разобрано 3 значения, "
          f"слот обнуляется: {unknown!r}")
    check(cli_over_msp(cli, "aux 8 13 2 1300 1700 0 0").strip() == "aux 8 13 2 1300 1700 0 0",
          "верный permanentId (BEEPER = 13) пишется, как 'aux' в 4.5.4")
    cli_over_msp(cli, "aux 8 0 0 900 900")
    aux_dump = [line for line in dump.splitlines() if line.startswith("aux ")]
    check(len(aux_dump) == 20 and aux_dump[0] == "aux 0 0 0 900 900 0 0",
          f"CLI: 20 строк 'aux' по семь чисел, пустой слот — {aux_dump[0]!r} (printAux из cli.c)")

    print("2б-2. Биты фич совпадают с прошивкой (feature.h / Features.js)")
    for name, bit in (("SOFTSERIAL", 6), ("GPS", 7), ("LED_STRIP", 16), ("OSD", 18),
                      ("RX_SERIAL", 3), ("TELEMETRY", 10), ("AIRMODE", 22), ("ANTI_GRAVITY", 28)):
        got = fc.feature_bits_by_name.get(name)
        check(got == 1 << bit, f"{name} = бит {bit} (получено 0x{got or 0:08x})")

    print("2в. SOFTSERIAL: фича добавляет порты (как на железе)")
    ports_before = [ports[1 + 9 * i] for i in range(count)]
    check(30 not in ports_before, f"при выключенной фиче портов SOFTSERIAL нет: {ports_before}")
    feature_mask = (1 << 6) | (1 << 18)  # SOFTSERIAL | OSD (биты из feature.h)
    cli.request(37, feature_mask.to_bytes(4, "little"))
    after = cli.request(0x1009, version=2)
    ids_after = [after[1 + 9 * i] for i in range(after[0])]
    check(30 in ids_after and 31 in ids_after, f"включение SOFTSERIAL добавляет SOFTSERIAL1/2: {ids_after}")
    cli.request(37, (1 << 18).to_bytes(4, "little"))
    back = cli.request(0x1009, version=2)
    ids_back = [back[1 + 9 * i] for i in range(back[0])]
    check(30 not in ids_back, f"выключение фичи убирает порты: {ids_back}")
    cli.request(37, (1 << 6).to_bytes(4, "little"))  # вернуть как было для следующих проверок

    print("3. CLI поверх MSP (MSP2_CLI_COMMAND 0x3012) — используется для новых вкладок")
    out = cli_over_msp(cli, "version")
    check("Betaflight" in out and "4.5.4" in out, "версия получена через MSP CLI")
    out = cli_over_msp(cli, "get gyro_lpf1_static_hz")
    check("gyro_lpf1_static_hz = 250" in out, f"get вернул значение: {out.splitlines()[:1]}")
    check("Allowed range: 0 - 1000" in out, "get вернул Allowed range")

    # set и чтение обратно — так делает Configurator во вкладках на MSP2_CLI_SETTING
    cli.request(0x3010, b"gyro_lpf1_static_hz = 320", version=2)
    resp = cli.request(0x3010, b"gyro_lpf1_static_hz", version=2)
    check(resp.startswith(b"gyro_lpf1_static_hz = 320"), f"MSP2_CLI_SETTING: {resp.decode()}")

    info = cli.request(0x3011, b"gyro_lpf1_static_hz\x00\x00\x00", version=2)
    total = u16(info, 0)
    check(total == len(info) - 2,
          f"MSP2_CLI_SETTING_INFO: поле длины совпадает с телом ответа ({total} = {len(info) - 2})")
    body = info[2:].decode("latin-1")
    check("min=0" in body and "max=1000" in body, f"MSP2_CLI_SETTING_INFO: {body.replace(chr(10), ' ')}")

    bad = cli.request(0x3010, b"gyro_lpf1_static_hz = 999999", version=2)
    check(bad == b"", "некорректное значение => пустой ответ (FC отказал)")

    print("4. Пейджинг длинного вывода (dump)")
    out = cli_over_msp(cli, "dump")
    check("# master" in out and "set gyro_lpf1_static_hz = 320" in out, "dump: секция master присутствует")
    check(len(out) > 10000, f"dump вернул {len(out)} байт через пейджинг")

    print("5. «Сырой» CLI: неинтерактивный режим (как MSP.send_cli_command)")
    cli.close()  # Configurator держит одно соединение; освобождаем сокет
    cli2 = Client()
    cli2.send_raw(bytes([0x02]) + b"status\n" + bytes([0x03]))  # STX + команда + ETX
    data = b""
    deadline = time.time() + 3
    while time.time() < deadline and bytes([0x03]) not in data:
        chunk = cli2.read_some(0.5)
        if not chunk:
            break
        data += chunk
    check(data.startswith(bytes([0x02])), "ответ начинается с STX (flow control)")
    check(b"MCU F40X" in data, "вывод status получен")
    check(data.rstrip().endswith(bytes([0x03])), "поток завершён ETX")
    check([entry[1] for entry in fc.recent_cli[-1:]] == ["status"],
          f"команда из неинтерактивного CLI попала в журнал монитора: "
          f"{[e[1] for e in fc.recent_cli[-3:]]}")

    print("6. «Сырой» CLI: интерактивный режим (кнопка CLI в Configurator)")
    cli2.close()
    cli3 = Client()
    cli3.send_raw(b"#")
    time.sleep(0.3)
    data = cli3.read_some(1.0).decode("latin-1")
    check("Entering CLI Mode" in data and "CLI" in data, "получено приветствие CLI")
    check(data.endswith("# "), "получено приглашение '# '")

    cli3.send_raw(b"get serialrx_provider\n")
    time.sleep(0.3)
    data = cli3.read_some(1.0).decode("latin-1")
    check("serialrx_provider = CRSF" in data, "интерактивный get вернул CRSF")
    check(data.endswith("# "), "приглашение после команды")

    cli3.send_raw(b"bogus_command\n")
    time.sleep(0.3)
    data = cli3.read_some(1.0).decode("latin-1")
    check("UNKNOWN COMMAND" in data, f"неизвестная команда => {data.strip().splitlines()[0] if data.strip() else '—'}")
    journal = [entry[1] for entry in fc.recent_cli]
    check(journal.count("get serialrx_provider") == 1 and journal.count("bogus_command") == 1,
          f"журнал раздела «CLI-команды» пополнился ровно по разу на команду: {journal}")
    reply = next(entry[2] for entry in fc.recent_cli if entry[1] == "get serialrx_provider")
    check("serialrx_provider = CRSF" in reply,
          "в журнале виден и ответ команды — монитор показывает то же, что вы ввели в CLI")

    print("7. Ребут по 'save' и переподключение")
    cli3.send_raw(b"save\n")
    time.sleep(0.3)
    closed = cli3.expect_close(5.0)
    check(closed, "соединение разорвано (FC перезагрузился)")
    cli3.close()
    time.sleep(1.2)
    cli4 = None
    for _ in range(10):
        try:
            cli4 = Client(timeout=2.0)
            break
        except OSError:
            time.sleep(0.5)
    check(cli4 is not None, "порт снова принимает подключения после ребута")
    payload = cli4.request(1)
    check(payload[1] == 1 and payload[2] == 46, "после ребута FC отвечает на MSP_API_VERSION")
    # значение, сохранённое до ребута, должно остаться (после 'save')
    resp = cli4.request(0x3010, b"gyro_lpf1_static_hz", version=2)
    check(b"= 320" in resp, f"сохранённое значение живо после ребута: {resp.decode()}")

    print("8. MSP_SET_REBOOT (Save and Reboot из Configurator)")
    cli4.request(68, bytes([0]), version=1)
    closed = cli4.expect_close(5.0)
    check(closed, "соединение разорвано по MSP_SET_REBOOT")
    cli4.close()

    print("9. EEPROM: конфигурация переживает полный перезапуск процесса")
    import subprocess

    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    env = dict(os.environ)
    root = Path(__file__).resolve().parent.parent
    port = 15888

    def start_emulator(http=None):
        argv = [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}",
                "--eeprom", str(eeprom), "--quiet"]
        if http:
            argv += ["--http", str(http)]
        proc = subprocess.Popen(argv, cwd=root, env=env,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(40):
            try:
                return proc, Client(port=port, timeout=1.0)
            except OSError:
                time.sleep(0.25)
        raise RuntimeError("эмулятор не поднялся")

    proc1, c1 = start_emulator()
    check(not eeprom.exists(), "старт с пустым «флешем»: файла ещё нет")
    c1.request(0x3010, b"gyro_lpf1_static_hz = 640", version=2)   # set
    c1.request(250, b"", version=1)                               # MSP_EEPROM_WRITE (Save в Configurator)
    time.sleep(0.3)
    check(eeprom.exists(), f"файл «флеша» создан ({eeprom.stat().st_size} байт)")
    c1.request(68, bytes([0]), version=1)                          # Save and Reboot
    c1.expect_close(5.0)
    c1.close()
    time.sleep(0.5)
    proc1.terminate()
    proc1.wait(timeout=10)

    proc2, c2 = start_emulator()
    resp = c2.request(0x3010, b"gyro_lpf1_static_hz", version=2)
    check(b"= 640" in resp, f"после перезапуска значение на месте: {resp.decode()}")

    # UI-запись: вкладки пишут конфигурацию не через CLI, а через MSP_SET_*
    # (например, «Датчики» выключают акселерометр кодом 97). Проверяем, что это тоже сохраняется.
    # фичи из UI (вкладка «Конфигурация»): включаем LED_STRIP и SOFTSERIAL.
    # SOFTSERIAL на железе добавляет порты SOFTSERIAL1/2 (идентификаторы 30/31).
    # биты — ровно как в feature.h и в Configurator (Features.js): с пропусками
    feature_bits = {"SOFTSERIAL": 1 << 6, "GPS": 1 << 7, "LED_STRIP": 1 << 16, "OSD": 1 << 18}
    feature_mask = 0
    for bit in feature_bits.values():
        feature_mask |= bit
    c2.request(37, feature_mask.to_bytes(4, "little"), version=1)   # MSP_SET_FEATURE_CONFIG
    # вкладка «GPS» сохраняет свой конфиг кодом MSP_SET_GPS_CONFIG (223) — 6 байт
    c2.request(223, bytes([1, 3, 1, 0, 0, 1]), version=1)
    c2.request(97, bytes([1, 0, 1, 0]), version=1)      # acc -> NONE
    c2.request(62, bytes([10, 0, 30]), version=1)       # ARMING: auto_disarm_delay, small_angle
    # BEEPER: ровно 9 байт, как crunch в MSPHelper.js (маска u32 + тон u8 + условия u32)
    c2.request(185, (1).to_bytes(4, "little") + bytes([3]) + (8).to_bytes(4, "little"), version=1)
    c2.request(250, b"", version=1)                     # EEPROM_WRITE
    time.sleep(0.3)
    c2.request(68, bytes([0]), version=1)               # Save and Reboot
    c2.expect_close(5.0)
    c2.close()
    time.sleep(0.5)
    proc2.terminate()
    proc2.wait(timeout=10)

    proc2b, c2b = start_emulator()
    sensors = c2b.request(96, b"", version=1)
    check(list(sensors) == [1, 0, 1, 0], f"настройка датчиков (acc выключен) пережила перезапуск: {list(sensors)}")
    arming = c2b.request(61, b"", version=1)
    check(list(arming) == [10, 0, 30], f"ARMING_CONFIG из UI пережил перезапуск: {list(arming)}")
    gcfg2 = c2b.request(132, b"", version=1)
    check(list(gcfg2) == [1, 3, 1, 0, 0, 1],
          f"GPS_CONFIG из UI пережил перезапуск: {list(gcfg2)} (провайдер UBLOX, SBAS WAAS, Galileo)")
    mask = struct.unpack_from("<H", c2b.request(150, b"", version=1), 4)[0]
    check(not mask & 1, f"маска сенсоров учитывает выключенный акселерометр (0x{mask:02x})")
    cli_str = cli_over_msp(c2b, "get small_angle")
    check("= 30" in cli_str, f"значение видно и в CLI: {cli_str.strip()}")

    features = struct.unpack_from("<I", c2b.request(36, b"", version=1))[0]
    check(features & feature_bits["SOFTSERIAL"] and features & feature_bits["LED_STRIP"],
          f"фичи из UI пережили перезапуск (маска 0x{features:05x})")
    ports2 = c2b.request(0x1009, b"", version=2)
    port_ids = [ports2[1 + 9 * i] for i in range(ports2[0])]
    check(30 in port_ids and 31 in port_ids,
          f"SOFTSERIAL добавила порты SOFTSERIAL1/2: {port_ids}")
    cli_feat = cli_over_msp(c2b, "feature")
    check(cli_feat.startswith("Enabled:") and "SOFTSERIAL" in cli_feat and "LED_STRIP" in cli_feat,
          f"CLI 'feature' подтверждает: {cli_feat.strip()}")
    c2b.request(37, (feature_mask & ~feature_bits["SOFTSERIAL"]).to_bytes(4, "little"), version=1)
    c2b.request(250, b"", version=1)
    time.sleep(0.3)
    ports3 = c2b.request(0x1009, b"", version=2)
    port_ids3 = [ports3[1 + 9 * i] for i in range(ports3[0])]
    check(30 not in port_ids3, f"выключение SOFTSERIAL убирает порты: {port_ids3}")
    c2b.close()
    proc2b.terminate()
    proc2b.wait(timeout=10)

    # 'defaults' сбрасывает и записывает заводские значения во «флеш»
    proc2c, cli_after_defaults = start_emulator()
    cli_after_defaults.send_raw(b"#")
    time.sleep(0.3)
    cli_after_defaults.read_some(1.0)
    cli_after_defaults.send_raw(b"defaults\n")
    time.sleep(0.4)
    cli_after_defaults.expect_close(5.0)
    cli_after_defaults.close()
    time.sleep(0.5)
    proc2c.terminate()
    proc2c.wait(timeout=10)

    proc3, c3 = start_emulator()
    resp = c3.request(0x3010, b"gyro_lpf1_static_hz", version=2)
    check(b"= 250" in resp, f"после 'defaults' во «флеше» заводское значение: {resp.decode()}")
    c3.close()
    proc3.terminate()
    proc3.wait(timeout=10)

    # ------------------------------------------------------------------ вкладка «Конфигурация»
    print("10. Вкладка «Конфигурация»: длины ответов и значения (MSPHelper.js / msp.c 4.5.4)")
    proc4, c4 = start_emulator()
    api = c4.request(1)
    check(len(api) == 3, f"MSP_API_VERSION: {len(api)} байт")

    adv = c4.request(90)
    check(len(adv) == 20, f"MSP_ADVANCED_CONFIG: {len(adv)} байт (нужно 20 — раскладка msp.c)")
    check(u16(adv, 6) == 550, f"digitalIdleOffsetValue = {u16(adv, 6)} (550 = 5.5 %)")
    check(u16(adv, 4) == 480, f"motorPwmRate = {u16(adv, 4)} Гц")
    check(adv[19] == 90, f"DEBUG_COUNT = {adv[19]} (в 4.5.4 их 90)")
    check(adv[3] == 7, f"протокол моторов = {adv[3]} (7 = DSHOT600)")

    rx = c4.request(44)
    check(len(rx) == 38, f"MSP_RX_CONFIG: {len(rx)} байт (нужно 38)")
    check((u16(rx, 1), u16(rx, 3), u16(rx, 5)) == (2000, 1500, 1050),
          f"max_check/mid_rc/min_check = {u16(rx,1)}/{u16(rx,3)}/{u16(rx,5)}")
    check((u16(rx, 8), u16(rx, 10)) == (885, 2115),
          f"rx_min_usec/rx_max_usec = {u16(rx,8)}/{u16(rx,10)}")
    check(max(0, (u16(rx, 14) - 1000)) // 10 == 30, f"порог airmode = {max(0,(u16(rx,14)-1000))//10} %")

    arm = c4.request(61)
    check(len(arm) == 3, f"MSP_ARMING_CONFIG: {len(arm)} байт (нужно 3)")
    beeper = c4.request(184)
    check(len(beeper) == 9, f"MSP_BEEPER_CONFIG: {len(beeper)} байт (нужно 9)")
    feats = c4.request(36)
    check(len(feats) == 4, f"MSP_FEATURE_CONFIG: {len(feats)} байт (нужно 4)")
    sensors = c4.request(96)
    check(len(sensors) == 4, f"MSP_SENSOR_CONFIG: {len(sensors)} байт (нужно 4)")
    name = c4.request(10)
    check(name == b"EMU-QUAD", f"MSP_NAME = {name!r}")
    craft = c4.request(0x3006, bytes([2]), version=2)
    craft_name = get_text(craft, 1)[0]
    check(craft_name == "EMU-QUAD", f"MSP2_GET_TEXT (имя дрона) = {craft_name!r}")
    pilot = c4.request(0x3006, bytes([1]), version=2)
    pilot_name = get_text(pilot, 1)[0]
    check(pilot_name == "VIRTUAL PILOT", f"MSP2_GET_TEXT (имя пилота) = {pilot_name!r}")

    # запись: протокол DSHOT300, порог airmode 25 %, цикл PID 2, debug_mode 3
    req = bytearray(adv)
    req[1], req[3] = 2, 6
    req[18] = 3
    c4.request(91, bytes(req[:19]))
    rx_req = bytearray(rx)
    rx_req[14:16] = (25 * 10 + 1000).to_bytes(2, "little")
    c4.request(45, bytes(rx_req))
    c4.request(250)                                     # MSP_EEPROM_WRITE («Сохранить»)
    c4.request(68, bytes([0]))                          # MSP_SET_REBOOT («Сохранить и перезагрузить»)
    c4.expect_close(5.0)
    c4.close()
    proc4.terminate()
    proc4.wait(timeout=10)

    proc5, c5 = start_emulator()
    adv2, rx2 = c5.request(90), c5.request(44)
    check(adv2[1] == 2 and adv2[3] == 6 and adv2[18] == 3,
          f"после перезапуска: цикл PID = {adv2[1]}, протокол = {adv2[3]}, debug_mode = {adv2[18]}")
    check(max(0, (u16(rx2, 14) - 1000)) // 10 == 25,
          f"после перезапуска: порог airmode = {max(0,(u16(rx2,14)-1000))//10} %")
    cli_out = cli_over_msp(c5, "get pid_process_denom")
    check("pid_process_denom = 2" in cli_out, f"CLI видит то же значение: {cli_out.strip().splitlines()[0]}")
    check("motor_pwm_protocol = DSHOT300" in cli_over_msp(c5, "get motor_pwm_protocol"),
          "CLI: motor_pwm_protocol = DSHOT300")
    c5.close()
    proc5.terminate()
    proc5.wait(timeout=10)

    # ------------------------------------------------------------------ вкладка «Failsafe»
    print("14. Вкладка «Failsafe»: диапазон импульсов, «Этап 1», «Этап 2», сохранение")
    proc12, c12 = start_emulator()

    rx_cfg = c12.request(44)
    check((u16(rx_cfg, 8), u16(rx_cfg, 10)) == (885, 2115),
          f"диапазон импульсов: {u16(rx_cfg, 8)}…{u16(rx_cfg, 10)}")

    failsafe = c12.request(75)
    check(len(failsafe) == 8, f"MSP_FAILSAFE_CONFIG: {len(failsafe)} байт (нужно 8)")
    check(failsafe[0] == 15, f"«Длительность 1го этапа»: {failsafe[0] / 10:.1f} с (в прошивке 1.5, а не 1.0)")
    check(failsafe[1] == 10, f"время посадки: {failsafe[1] / 10:.1f} с")
    check(u16(failsafe, 2) == 1000 and failsafe[4] == 0 and u16(failsafe, 5) == 100 and failsafe[7] == 1,
          "газ 1000, тумблер «Этап 1», низкий газ 10 с, процедура «Падение»")

    rxfail = c12.request(77)
    check(len(rxfail) == 48, f"MSP_RXFAIL_CONFIG: {len(rxfail)} байт = {len(rxfail) // 3} каналов (CRSF)")
    modes = [rxfail[i * 3] for i in range(len(rxfail) // 3)]
    check(modes[:4] == [0, 0, 0, 0] and all(m == 1 for m in modes[4:]),
          "«Этап 1»: у стиков «Автоматически» (0), у AUX «Удержание» (1)")
    check(u16(rxfail, 3 * 3 + 1) == 875, "у газа значение 875 мкс (RX_MIN_USEC на сетке 25 мкс)")

    rc = c12.request(105)
    check(len(rc) == 32, f"MSP_RC: {len(rc) // 2} каналов (CRSF отдаёт 16)")
    check(list(c12.request(64)) == [0, 1, 3, 2, 4, 5, 6, 7], "карта каналов: заводская AETR1234 (8 байт)")

    # правки вкладки
    new_rx = bytearray(rx_cfg)
    new_rx[8:10] = (1000).to_bytes(2, "little")
    new_rx[10:12] = (2000).to_bytes(2, "little")
    c12.request(45, bytes(new_rx))
    c12.request(76, bytes([20, 30]) + (1200).to_bytes(2, "little") + bytes([2])
                + (50).to_bytes(2, "little") + bytes([0]))
    c12.request(78, bytes([4, 2]) + (1750).to_bytes(2, "little"))
    edited = c12.request(75)
    check((edited[0], edited[1], u16(edited, 2), edited[4], u16(edited, 5), edited[7]) == (20, 30, 1200, 2, 50, 0),
          "«Этап 2» принял правки: 2.0 с / 3.0 с / газ 1200 / «Этап 2» / 5 с / «Приземление»")
    check((c12.request(77)[4 * 3], u16(c12.request(77), 4 * 3 + 1)) == (2, 1750),
          "«Этап 1»: AUX 1 переведён в «Значение» 1750 мкс")
    check((u16(c12.request(44), 8), u16(c12.request(44), 10)) == (1000, 2000),
          "диапазон импульсов принял правки")

    c12.request(250)
    c12.request(68, b"\x00")
    time.sleep(1.0)
    c12.close()
    proc12.terminate()
    proc12.wait(timeout=10)

    proc13, c13 = start_emulator()
    saved = c13.request(75)
    check((saved[0], saved[1]) == (20, 30), "«Этап 2» пережил перезапуск эмулятора")
    check((c13.request(77)[4 * 3], u16(c13.request(77), 4 * 3 + 1)) == (2, 1750),
          "«Этап 1» пережил перезапуск эмулятора")
    check(u16(c13.request(44), 8) == 1000, "диапазон импульсов пережил перезапуск")
    check("failsafe_procedure = AUTO-LAND" in cli_over_msp(c13, "get failsafe_procedure"),
          "CLI: failsafe_procedure = AUTO-LAND")
    c13.close()
    proc13.terminate()
    proc13.wait(timeout=10)

    # ------------------------------------------------------- вкладка «Питание и батарея»
    print("13. Вкладка «Питание и батарея»: настройки, живые данные, калибровка, сохранение")
    proc10, c10 = start_emulator()

    battery = c10.request(32)
    check(len(battery) == 13, f"MSP_BATTERY_CONFIG: {len(battery)} байт (нужно 13)")
    check((u16(battery, 7), u16(battery, 9), u16(battery, 11)) == (330, 430, 350),
          f"напряжения на банку: {u16(battery, 7)}/{u16(battery, 9)}/{u16(battery, 11)} "
          f"(0.01 В — вкладка показывает 3.30/4.30/3.50, а не нули)")
    check(u16(battery, 3) == 1500, f"ёмкость: {u16(battery, 3)} мА·ч")
    check(battery[5] == 1 and battery[6] == 1, "датчики напряжения и тока: ADC")

    voltage_cfg = c10.request(56)
    check(len(voltage_cfg) == 7 and voltage_cfg[1] == 5 and voltage_cfg[2] == 10,
          f"MSP_VOLTAGE_METER_CONFIG: подкадр длиной {voltage_cfg[1]} для метра id {voltage_cfg[2]}")
    current_cfg = c10.request(40)
    check(current_cfg[0] == 2 and current_cfg[2] == 10 and current_cfg[9] == 80,
          "MSP_CURRENT_METER_CONFIG: ADC (id 10) и виртуальный (id 80)")

    voltage_meters = c10.request(128)
    check(list(voltage_meters) == [10, 168], f"MSP_VOLTAGE_METERS: 16.8 В ({list(voltage_meters)})")
    check(len(c10.request(129)) == 10, "MSP_CURRENT_METERS: два метра по 5 байт")

    state = c10.request(130)
    check(state[0] == 4 and 1670 <= u16(state, 9) <= 1680,
          f"состояние батареи: {state[0]} банки, {u16(state, 9) / 100:.2f} В "
          f"(чуть ниже 16.80 В: идёт расход электроники)")
    check(40 <= i16(state, 6) <= 70,
          f"в покое виден ток электроники: {i16(state, 6) / 100:.2f} А (FC, приёмник, VTX, LED)")
    check(u16(state, 4) <= 80,
          f"расход считается и в покое: {u16(state, 4)} мА·ч за первые секунды (электроника)")
    check(state[8] == 0, "статус батареи: 0 (OK)")

    c10.request(214, b"".join((1500).to_bytes(2, "little") for _ in range(4)) + b"\x00" * 8)
    time.sleep(0.5)
    running = c10.request(130)
    check(1200 < i16(running, 6) < 1500,
          f"тест моторов: ток {i16(running, 6) / 100:.2f} А (четыре ESC: холостой ток + 12 А на половине газа)")
    check(u16(running, 4) >= 1, "счётчик мА·ч тикает только под нагрузкой")
    check(u16(c10.request(110), 7) < 1680, "под нагрузкой напряжение просело (в MSP_ANALOG меньше 16.80 В)")
    time.sleep(1.2)
    check(i16(c10.request(130), 6) > 1000,
          "моторы держат газ, пока вкладка не пришлёт «Стоп» — как на железной плате (таймаута нет)")
    c10.request(214, b"".join((1000).to_bytes(2, "little") for _ in range(4)) + b"\x00" * 8)
    time.sleep(2.0)   # мотор выбегает: винт тормозится не мгновенно
    stop_current = i16(c10.request(130), 6) / 100
    check(40 <= i16(c10.request(130), 6) <= 70,
          f"после «Стоп» ток вернулся к току покоя: {stop_current:.2f} А (электроника, моторы встали)")
    back = u16(c10.request(110), 7)
    check(1650 <= back <= 1680,
          f"и просадки нет: напряжение покоя {back / 100:.2f} В (ниже 16.80 — за время теста "
          f"моторов часть заряда «сгорела», время эмулятора идёт ×10)")

    c10.request(33, bytes([36, 42, 38]) + (2200).to_bytes(2, "little") + bytes([1, 2])
                + (360).to_bytes(2, "little") + (420).to_bytes(2, "little") + (380).to_bytes(2, "little"))
    c10.request(57, bytes([10, 120, 10, 1]))
    c10.request(41, bytes([80]) + (450).to_bytes(2, "little") + (5).to_bytes(2, "little"))
    edited = c10.request(32)
    check((u16(edited, 7), u16(edited, 9), u16(edited, 11), u16(edited, 3)) == (360, 420, 380, 2200),
          "MSP_SET_BATTERY_CONFIG: правки вкладки приняты (3.60/4.20/3.80 В, 2200 мА·ч)")
    check(edited[6] == 2, f"выбранный датчик тока: {edited[6]} (2 = VIRTUAL)")
    check(c10.request(56)[4] == 120, "MSP_SET_VOLTAGE_METER_CONFIG: масштаб 120 сохранён")
    check(c10.request(40)[11:13] == (450).to_bytes(2, "little"), "MSP_SET_CURRENT_METER_CONFIG: масштаб 450 сохранён")

    c10.request(250)                      # «Сохранить»
    c10.request(68, b"\x00")              # «Сохранить и перезагрузить»
    time.sleep(0.8)
    c10.close()
    proc10.terminate()
    proc10.wait(timeout=10)

    proc11, c11 = start_emulator()
    saved = c11.request(32)
    check((u16(saved, 7), u16(saved, 9), u16(saved, 3)) == (360, 420, 2200),
          "настройки батареи пережили перезапуск эмулятора")
    check(u16(c11.request(130), 4) == 0, "счётчик мА·ч после перезагрузки обнулён")
    check("vbat_warning_cell_voltage = 380" in cli_over_msp(c11, "get vbat_warning_cell_voltage"),
          "CLI: get vbat_warning_cell_voltage = 380")
    check("current_meter = VIRTUAL" in cli_over_msp(c11, "get current_meter"),
          "CLI: current_meter = VIRTUAL")
    c11.close()
    proc11.terminate()
    proc11.wait(timeout=10)

    # ------------------------------------- живое питание: «виртуальный полёт» эмулятора
    print("13в. Виртуальный полёт: вкладка «Питание и батарея» живая (ток, расход, просадка)")
    import urllib.request

    http_sim = port + 1
    proc_sim, c_sim = start_emulator(http=http_sim)

    def page_get(path):
        with urllib.request.urlopen(f"http://127.0.0.1:{http_sim}{path}", timeout=5) as response:
            return response.read().decode("utf-8", "replace")

    def sim_json():
        return json.loads(page_get("/api/state"))["power"]

    def motor_values(client, count=4):
        payload = client.request(104)
        return [u16(payload, i * 2) for i in range(count)]

    idle = c_sim.request(130)
    check(40 <= i16(idle, 6) <= 70,
          f"на столе ток не ноль: {i16(idle, 6) / 100:.2f} А — электроника борта")
    check(motor_values(c_sim) == [1000] * 4, "при этом моторы стоят: MSP_MOTOR отдаёт 1000")
    check(sim_json()["sim"]["on"] is False, "виртуальный полёт по умолчанию выключен")

    page = page_get("/")
    check("Питание и батарея" in page and "/sim_flight/on" in page,
          "на мониторе есть карточка «Питание и батарея» и кнопка «включить виртуальный полёт»")

    cli_over_msp(c_sim, "sim_flight on")
    time.sleep(2.5)
    flying = c_sim.request(130)
    current = i16(flying, 6) / 100
    check(current > 3.0, f"в полёте ток вырос до {current:.2f} А (газ по сценарию, а не 0.00 А)")
    check(u16(flying, 9) < u16(idle, 9),
          f"напряжение просело под нагрузкой: {u16(flying, 9) / 100:.2f} В "
          f"против {u16(idle, 9) / 100:.2f} В на столе")
    check(u16(flying, 4) > 0, f"расход мА·ч пошёл: {u16(flying, 4)} мА·ч")
    gas = motor_values(c_sim)
    check(all(1000 < value < 2000 for value in gas),
          f"MSP_MOTOR: газ держит сценарий — слайдеры вкладки «Моторы» не нужны ({gas})")
    tele = c_sim.request(139)
    check(len(tele) == 1 + 13 * 4 and u32(tele, 1) > 1000,
          f"телеметрия ESC живая: обороты мотора 1 — {u32(tele, 1)}, "
          f"ток {u16(tele, 10) / 100:.2f} А, расход {u16(tele, 12)} мА·ч")

    before = c_sim.request(130)
    time.sleep(5.0)
    after = c_sim.request(130)
    grown = u16(after, 4) - u16(before, 4)
    check(grown >= 30, f"расход тикает на глазах: +{grown} мА·ч за 5 с")
    # Напряжение в каждый момент = покой минус просадка от текущего тока, поэтому смотрим
    # на спад за длинное окно: разряд за 5 с даёт ~0.1 В, а «дыхание» газа меньше.
    check(u16(after, 9) <= u16(before, 9) - 5,
          f"и напряжение падает по мере разряда: {u16(before, 9) / 100:.2f} → "
          f"{u16(after, 9) / 100:.2f} В")

    status = cli_over_msp(c_sim, "sim_flight status")
    check("state: ON - flight" in status and "mAh" in status,
          "CLI: 'sim_flight status' печатает состояние, ток и расход")
    check("emulator: virtual flight" in cli_over_msp(c_sim, "help sim_flight"),
          "CLI: 'help sim_flight' описывает команду")
    check("SCALE MUST BE 1..600" in cli_over_msp(c_sim, "sim_flight scale 0"),
          "CLI: 'sim_flight scale 0' — понятная ошибка вместо мусора")
    page = page_get("/")
    check("/sim_flight/off" in page and "полёт" in page,
          "пока идёт полёт, на мониторе кнопка «остановить полёт»")

    cli_over_msp(c_sim, "sim_flight scale 600")     # ускоряем: батарея «сгорит» за секунды
    packs_before = sim_json()["sim"]["packs"]
    deadline = time.time() + 30
    while time.time() < deadline and sim_json()["sim"]["packs"] == packs_before:
        time.sleep(0.5)
    sim_now = sim_json()["sim"]
    check(sim_now["packs"] > packs_before,
          f"разряженная батарея меняется сама: батарей заменено {sim_now['packs']} "
          f"(посадка → новая батарея)")
    check(sim_json()["mah"] < 1500, "после замены счётчик мА·ч начат с нуля")

    cli_over_msp(c_sim, "sim_flight scale 10")
    cli_over_msp(c_sim, "sim_flight off")
    time.sleep(3.0)
    stopped = c_sim.request(130)
    check(40 <= i16(stopped, 6) <= 70,
          f"после остановки ток снова только электроники: {i16(stopped, 6) / 100:.2f} А")
    check(motor_values(c_sim) == [1000] * 4, "моторы остановлены: MSP_MOTOR снова 1000")
    cli_over_msp(c_sim, "sim_flight reset")
    check(u16(c_sim.request(130), 4) <= 2, "CLI 'sim_flight reset' обнулил счётчик мА·ч")

    page_get("/sim_flight/on")
    time.sleep(1.0)
    check(sim_json()["sim"]["on"] is True,
          "кнопка монитора включает полёт (то же, что 'sim_flight on')")
    page_get("/sim_flight/reset")
    check(sim_json()["mah"] <= 5, "кнопка «сбросить расход» обнуляет счётчик")
    page_get("/sim_flight/off")
    time.sleep(0.5)
    check(sim_json()["sim"]["on"] is False, "кнопка «остановить полёт» выключает полёт")

    c_sim.close()
    proc_sim.terminate()
    proc_sim.wait(timeout=10)

    # ------------------------------------------------------------------ вкладка «Датчики»
    print("12. Вкладка «Датчики»: поза, живые данные, триммирование, выравнивание, калибровка")
    proc8, c8 = start_emulator()

    att = c8.request(108)
    check(len(att) == 6, f"MSP_ATTITUDE: {len(att)} байт (нужно 6)")
    check(i16(att, 0) == 0 and i16(att, 2) == 0, "дрон лежит на столе: крен и тангаж 0.0 (фигурка не вращается)")
    check(i16(att, 4) == 0, f"курс в ГРАДУСАХ (не в десятых долях): {i16(att, 4)}")

    imu = c8.request(102)
    check(len(imu) == 18, f"MSP_RAW_IMU: {len(imu)} байт (нужно 18 — 9 значений)")
    check(2040 <= i16(imu, 4) <= 2056, f"акселерометр: по Z 1 g = {i16(imu, 4)} (2048 в единицах прошивки)")

    trim = c8.request(240)
    check(len(trim) == 4 and i16(trim, 0) == 0 and i16(trim, 2) == 0,
          f"MSP_ACC_TRIM: триммирование отдаётся и нулевое: pitch {i16(trim, 0)}, roll {i16(trim, 2)}")
    c8.request(239, i16_bytes(24) + i16_bytes(-6))
    trim2 = c8.request(240)
    check((i16(trim2, 0), i16(trim2, 2)) == (24, -6), "MSP_SET_ACC_TRIM: правка триммирования принята")

    align = c8.request(126)
    check(len(align) == 7, f"MSP_SENSOR_ALIGNMENT: {len(align)} байт (нужно 7)")
    check(align[3] == 1, "флаг «есть данные о вращении гироскопа» выставлен (gyroDetectionFlags)")

    compass = c8.request(133)
    check(len(compass) == 2, f"MSP_COMPASS_CONFIG: {len(compass)} байт (нужно 2)")
    c8.request(224, u16_bytes(120))
    check(u16(c8.request(133)) == 120, "MSP_SET_COMPASS_CONFIG: склонение 12.0° принято")

    active = c8.request(0x300A, version=2)
    check(active[0] != 0 and active[1] != 0 and active[4] == 0,
          f"обнаруженные датчики: gyro {active[0]}, acc {active[1]}, дальномер {active[4]} (0 = нет на плате)")

    board = c8.request(4)
    check(int.from_bytes(board[-4:], "little") & 1 == 1,
          "MSP_BOARD_INFO: до калибровки выставлен бит «акселерометр не откалиброван»")
    c8.request(205)
    time.sleep(1.3)
    board2 = c8.request(4)
    check(int.from_bytes(board2[-4:], "little") & 1 == 0,
          "MSP_ACC_CALIBRATION: через секунду бит снят — вкладка видит завершение калибровки")

    c8.request(250)                      # «Сохранить»
    c8.request(68, b"\x00")              # «Сохранить и перезагрузить»
    time.sleep(0.8)
    c8.close()
    proc8.terminate()
    proc8.wait(timeout=10)

    proc9, c9 = start_emulator()
    trim3 = c9.request(240)
    check((i16(trim3, 0), i16(trim3, 2)) == (24, -6),
          f"триммирование пережило перезапуск: {i16(trim3, 0)}/{i16(trim3, 2)}")
    check(u16(c9.request(133)) == 120, "склонение пережило перезапуск: 12.0°")
    check(int.from_bytes(c9.request(4)[-4:], "little") & 1 == 0, "калибровка акселерометра осталась зачтённой")
    check("acc_trim_pitch = 24" in cli_over_msp(c9, "get acc_trim_pitch"),
          "CLI: get acc_trim_pitch = 24")
    c9.close()
    proc9.terminate()
    proc9.wait(timeout=10)

    # ---------------------------------------------------------- вкладка «ВИДЕОПЕРЕДАТЧИК»
    print("11. Вкладка «ВИДЕОПЕРЕДАТЧИК»: таблица VTX (SmartAudio на UART4)")
    proc6, c6 = start_emulator()

    def ports_vtx_banner(payload) -> bool:
        """Ровно то, что проверяет Configurator (usePortsState.js: vtxTableNotConfigured):
        плашка «таблица VTX не настроена» появляется при доступной, но пустой таблице."""
        available, bands, channels, levels = payload[11], payload[12], payload[13], payload[14]
        return bool(available) and (bands == 0 or channels == 0 or levels == 0)

    vtx = c6.request(88)
    check(len(vtx) == 15, f"MSP_VTX_CONFIG: {len(vtx)} байт (нужно 15)")
    check(vtx[0] == 3, f"тип VTX = {vtx[0]} (3 = TBS SmartAudio — на UART4 включена эта функция)")
    check(vtx[11] == 1, "таблица VTX доступна (сборка с USE_VTX_TABLE)")
    check((vtx[12], vtx[13], vtx[14]) == (5, 8, 4),
          f"заводская таблица заполнена: полос {vtx[12]}, каналов {vtx[13]}, "
          f"уровней мощности {vtx[14]} (как у настроенного квада)")
    check(not ports_vtx_banner(vtx),
          "жёлтой плашки «таблица VTX не настроена» на вкладке «Порты» НЕ будет "
          f"(полосы/каналы/уровни: {vtx[12]}/{vtx[13]}/{vtx[14]})")
    check(ports_vtx_banner(bytes([0] * 11 + [1, 0, 8, 4])),
          "проверка плашки верна: с пустой таблицей она бы появилась")
    # `defaults` (как и MSP_RESET_CONF) возвращает ту же заводскую таблицу — плашка не вернётся
    c6.request(89, bytes([0] * 11 + [0, 0, 0, 1]))     # вручную очищаем таблицу, как это делает человек
    check((c6.request(88)[12], c6.request(88)[13]) == (0, 0),
          "очистка таблицы во вкладке действительно опустошает её")
    cli_over_msp(c6, "defaults nosave")                # без ребута: проверяем состояние в памяти
    vtx_reset = c6.request(88)
    check((vtx_reset[12], vtx_reset[13], vtx_reset[14]) == (5, 8, 4)
          and not ports_vtx_banner(vtx_reset),
          "после 'defaults' таблица снова заводская — плашки «таблица VTX не настроена» нет")
    check((vtx[1], vtx[2], u16(vtx, 5)) == (1, 1, 5865),
          f"канал уже выбран: полоса {vtx[1]}, канал {vtx[2]}, {u16(vtx, 5)} МГц "
          "(BOSCAM A, канал 1)")

    status = c6.request(0x3004, b"", version=2)
    check(len(status) == 38, f"MSP2_GET_VTX_DEVICE_STATUS: {len(status)} байт (4 уровня мощности)")
    check(status[0] == 3 and status[1] == 1, f"устройство SmartAudio готово (тип {status[0]})")
    check(status[15] == 4 and (u16(status, 16), u16(status, 18)) == (14, 25)
          and (u16(status, 28), u16(status, 30)) == (29, 800),
          f"уровни передатчика: {status[15]} (первый {u16(status, 18)} мВт = 14 dBm, "
          f"последний {u16(status, 30)} мВт = 29 dBm)")
    check(status[16 + 4 * 4] == 5 and status[17 + 4 * 4] == 3,
          "паспорт SmartAudio: 5 байт, версия 3 (2.1)")

    # заполняем таблицу так же, как это делает интерфейс: размеры → уровни → банды
    cfg = bytearray(15)
    cfg[7], cfg[8] = 1, 3          # выбран банд 1 (RACEBAND), канал 3 — частота должна прийти из таблицы
    cfg[11], cfg[12], cfg[13], cfg[14] = 2, 8, 2, 1
    c6.request(89, bytes(cfg))
    c6.request(228, bytes([1]) + (25).to_bytes(2, "little") + bytes([2]) + b"25")
    c6.request(228, bytes([2]) + (800).to_bytes(2, "little") + bytes([3]) + b"800")
    for number, (name, letter, freqs) in enumerate((
        ("RACEBAND", "R", [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]),
        ("MYBAND", "X", [5900, 5910, 5920, 5930, 5940, 5950, 5960, 5970]),
    ), start=1):
        payload = (bytes([number, len(name)]) + name.encode() + letter.encode()
                   + bytes([1 if number == 1 else 0, 8])
                   + b"".join(f.to_bytes(2, "little") for f in freqs))
        c6.request(227, payload)

    band = c6.request(137, bytes([1]))
    check(band[0] == 1 and band[2:10] == b"RACEBAND", f"банд 1: {band[2:10].decode()}")
    check(band[10:11] == b"R" and band[12] == 8, f"буква {band[10:11].decode()}, каналов {band[12]}")
    check(u16(band, 13) == 5658 and u16(band, 27) == 5917,
          f"частоты: {u16(band, 13)} … {u16(band, 27)} МГц")
    power = c6.request(138, bytes([2]))
    check(power[0] == 2 and u16(power, 1) == 800 and power[4:7] == b"800",
          f"уровень 2: {u16(power, 1)} мВт, подпись {power[4:7].decode()}")
    vtx2 = c6.request(88)
    check(u16(vtx2, 5) == 5732, f"частота выбрана из таблицы: {u16(vtx2, 5)} МГц (RaceBand, канал 3)")
    status2 = c6.request(0x3004, b"", version=2)
    check(len(status2) == 30, f"статус с двумя уровнями: {len(status2)} байт (16 + 2×4 + 6)")
    check((u16(status2, 16), u16(status2, 18)) == (14, 25),
          f"уровень 1: {u16(status2, 16)} dBm = {u16(status2, 18)} мВт")

    out = cli_over_msp(c6, "vtxtable")
    check(("vtxtable bands 2" in out and "vtxtable band 1 RACEBAND R FACTORY 5658" in out.replace("  ", " ")) or
          "vtxtable band 1 RACEBAND R FACTORY  5658" in out,
          "CLI 'vtxtable' показывает таблицу")
    check("vtxtable powerlabels 25 800" in out, "CLI: подписи уровней")
    check("level 14 dBm, power 25 mW" in cli_over_msp(c6, "vtx_info"), "CLI 'vtx_info' показывает уровни")

    c6.request(250)                                     # «Сохранить»
    c6.request(68, bytes([0]))                          # «Сохранить и перезагрузить»
    c6.expect_close(5.0)
    c6.close()
    time.sleep(0.5)
    proc6.terminate()
    proc6.wait(timeout=10)

    proc7, c7 = start_emulator()
    vtx3 = c7.request(88)
    check((vtx3[12], vtx3[13], vtx3[14]) == (2, 8, 2),
          f"таблица пережила перезапуск: бандов {vtx3[12]}, каналов {vtx3[13]}, уровней {vtx3[14]}")
    check(u16(vtx3, 5) == 5732, f"частота сохранена: {u16(vtx3, 5)} МГц")
    band3 = c7.request(137, bytes([2]))
    check(band3[2:10].rstrip() == b"MYBAND" and u16(band3, 13) == 5900,
          f"свой банд на месте: {band3[2:10].decode().strip()} / {u16(band3, 13)} МГц")
    check("vtxtable band 2 MYBAND X CUSTOM" in cli_over_msp(c7, "vtxtable").replace("  ", " "),
          "CLI видит свою таблицу после перезапуска")
    c7.close()
    proc7.terminate()
    proc7.wait(timeout=10)

    # ------------------------------------------------------- вкладка «СЕРВОПРИВОДЫ»
    print("15. Вкладка «СЕРВОПРИВОДЫ»: build-опции (USE_SERVOS), сервоприводы, микшер, сохранение")
    proc12, c12 = start_emulator()

    build = c12.request(5)
    check(len(build) == 48, f"MSP_BUILD_INFO: {len(build)} байт (11 + 8 + 7 + опции + терминатор)")
    check(build[0:11].decode() == "Sep 24 2026", f"дата сборки: {build[0:11].decode()!r} (ровно 11 байт)")
    check(build[11:19].decode() == "12:00:00", f"время сборки: {build[11:19].decode()!r} (сдвига нет)")
    check(build[19:26].decode().rstrip() == "emu0bf", f"git-ревизия: {build[19:26].decode()!r}")

    codes, pos = [], 26
    while pos + 1 < len(build):
        code = u16(build, pos)
        pos += 2
        if code == 0:
            break
        codes.append(code)
    check(16420 in codes, "build options содержат USE_SERVOS (16420) — по нему Configurator показывает вкладку")
    check(16412 in codes,
          "build options содержат USE_GPS (16412) — иначе вкладки «GPS» в Configurator нет вовсе")
    check(codes == [4097, 8231, 16408, 16412, 16413, 16416, 16420, 16421, 16423, 16429],
          f"коды опций: {codes}")
    check(build[pos - 2 : pos] == b"\x00\x00" and pos == len(build),
          "список завершён нулевым u16 (MSPHelper.js читает коды до нуля)")
    check(len(codes) == 10, "после последней опции нет лишних байт (иначе вкладки поехали бы)")

    # Features.js фильтрует функции вкладки «Конфигурация» по тем же build-опциям
    # (opt.includes(feature.dependsOn)), а TELEMETRY возвращает для CRSF/GHST/FPORT/JETI.
    option_names = ["USE_SERIALRX_CRSF", "USE_DSHOT", "USE_DASHBOARD", "USE_LED_STRIP", "USE_OSD_SD",
                    "USE_SERVOS", "USE_VTX", "USE_SOFTSERIAL", "USE_RANGEFINDER"]
    for name, depends in (("SERVO_TILT", "SERVOS"), ("CHANNEL_FORWARDING", "SERVOS"),
                          ("LED_STRIP", "LED_STRIP"), ("DISPLAY", "DASHBOARD"), ("OSD", "OSD"),
                          ("SONAR", "RANGEFINDER")):
        check(any(depends in opt for opt in option_names),
              f"функция {name} остаётся на вкладке «Конфигурация» (build-опция с «{depends}» есть)")
    check(any("CRSF" in opt for opt in option_names),
          "TELEMETRY не пропадёт: Configurator возвращает её для CRSF (USE_TELEMETRY в списке нет)")

    servos = c12.request(120)
    check(len(servos) == 96 and len(servos) % 12 == 0,
          f"MSP_SERVO_CONFIGURATIONS: {len(servos)} байт = 8 × 12 (вкладка считает набор непустым)")
    check((u16(servos, 0), u16(servos, 2), u16(servos, 4)) == (1000, 2000, 1500),
          "заводские значения: min 1000, max 2000, середина 1500 (pgResetFn_servoParams)")
    check(servos[6] == 100 and servos[7] == 0xFF,
          f"скорость {servos[6]} и «нет перенаправления» (0xFF) — как в прошивке")
    check(int.from_bytes(servos[8:12], "little") == 0, "реверс источников по умолчанию пуст")

    mix = c12.request(241)
    check(len(mix) == 112, f"MSP_SERVO_MIX_RULES: {len(mix)} байт = 16 правил по 7")

    livedata = c12.request(103)
    check(len(livedata) == 16, f"MSP_SERVO: {len(livedata) // 2} живых значений")
    check(all(u16(livedata, i * 2) == 1500 for i in range(8)), "на столе все сервоприводы в середине")

    c12.request(212, bytes([2]) + u16_bytes(900) + u16_bytes(2100) + u16_bytes(1500)
                + bytes([50, 4]) + (1 << 4).to_bytes(4, "little"))
    edited = c12.request(120)
    check((u16(edited, 24), u16(edited, 26), u16(edited, 28)) == (900, 2100, 1500),
          "MSP_SET_SERVO_CONFIGURATION (212): границы Servo 3 приняты")
    check(edited[30] == 50 and edited[31] == 4, "скорость 50 и перенаправление с AUX 1 записаны")
    check(int.from_bytes(edited[32:36], "little") == 1 << 4, "реверс источника (бит 4 = RC roll) записан")
    check(u16(c12.request(120), 0) == 1000, "соседние сервоприводы не задеты")
    c12.request(212, bytes([9]) + bytes(12))
    check(len(c12.request(120)) == 96, "индекс >= 8 отклонён (как MSP_RESULT_ERROR в msp.c)")

    live2 = c12.request(103)
    check(u16(live2, 4) == 1000, f"Servo 3 отдаёт уровень AUX 1: {u16(live2, 4)} мкс — ровно как в MSP_RC")
    check(u16(live2, 0) == 1500, "остальные — середина")

    c12.request(250)                       # «Сохранить»
    c12.request(68, b"\x00")              # «Сохранить и перезагрузить»
    time.sleep(0.8)
    c12.close()
    proc12.terminate()
    proc12.wait(timeout=10)

    proc13, c13 = start_emulator()
    saved = c13.request(120)
    check((u16(saved, 24), u16(saved, 26), saved[30], saved[31]) == (900, 2100, 50, 4),
          "правки сервопривода пережили перезапуск эмулятора")
    check(16420 in [int.from_bytes(c13.request(5)[i : i + 2], "little")
                    for i in range(26, 44, 2)],
          "build options на месте после перезапуска (вкладка не пропадёт)")
    out = cli_over_msp(c13, "servo")
    check("servo 2 900 2100 1500 50 4" in out, "CLI: 'servo' печатает таблицу как прошивка")
    out = cli_over_msp(c13, "dump")
    check("# servo" in out and "# servo mixer" in out, "CLI: 'dump' содержит секции servo и servo mixer")
    check("smix reverse 2 4 r" in out, "CLI: 'dump' содержит направление сервопривода (бит 4)")
    check("###ERROR IN smix: INVALID NAME: customairplane" in cli_over_msp(c13, "smix load customairplane"),
          "CLI: 'smix load' с неизвестным микшером отвечает ошибкой, как прошивка")
    c13.close()
    proc13.terminate()
    proc13.wait(timeout=10)

    # ------------------------------------------------------ вкладка «PID настройки»
    print("16. Вкладка «PID настройки»: таблица, фильтры, ползунки, профили, сохранение")
    proc14, c14 = start_emulator()

    names = c14.request(117)
    check(names.decode("latin-1") == "ROLL;PITCH;YAW;LEVEL;MAG;",
          f"MSP_PIDNAMES: {names.decode('latin-1')} (без него вкладка падала при загрузке)")

    pids = c14.request(112)
    check(len(pids) == 15, f"MSP_PID: {len(pids)} байт = 5 осей × 3")
    check(tuple(pids[0:3]) == (45, 80, 40) and tuple(pids[3:6]) == (47, 84, 46)
          and tuple(pids[9:12]) == (50, 75, 75),
          f"заводские PID: roll {tuple(pids[0:3])}, pitch {tuple(pids[3:6])}, LEVEL {tuple(pids[9:12])}")

    advanced = c14.request(94)
    check(len(advanced) == 61, f"MSP_PID_ADVANCED: {len(advanced)} байт (раскладка msp.c 4.5.4)")
    check(advanced[17] == 60 and u16(advanced, 21) == 80,
          f"«Макс. угол наклона» {advanced[17]}°, антигравитация {u16(advanced, 21)}")
    check(advanced[42] == 37 and advanced[43] == 20 and advanced[52] == 15,
          f"D Max gain {advanced[42]}, advance {advanced[43]}, Feedforward boost {advanced[52]}")
    check(advanced[57] == 1 and advanced[58] == 65 and u16(advanced, 59) == 1350,
          f"TPA: режим {advanced[57]} (D), {advanced[58]}% с {u16(advanced, 59)}")

    rc = c14.request(111)
    check(len(rc) == 23, f"MSP_RC_TUNING: {len(rc)} байта")
    check(rc[2] == 67 and rc[6] == 50 and rc[22] == 3,
          f"рейты: srate {rc[2]}, throttle mid {rc[6]}, тип {rc[22]} (ACTUAL)")
    check(u16(rc, 16) == 1998 and u16(rc, 20) == 1998, "лимиты скорости по осям 1998°/с")

    filters = c14.request(92)
    check(len(filters) == 49, f"MSP_FILTER_CONFIG: {len(filters)} байт (порядок полей как в msp.c)")
    check(u16(filters, 1) == 75 and u16(filters, 3) == 100 and u16(filters, 39) == 300,
          f"D-term LPF1 {u16(filters, 1)}, yaw lowpass {u16(filters, 3)}, добротность динамического фильтра {u16(filters, 39)}")
    check(filters[48] == 3 and filters[47] == 5, f"динамических фильтров {filters[48]}, экспонента LPF {filters[47]}")

    deadband = c14.request(125)
    check(list(deadband) == [0, 0, 40, 50, 0], f"MSP_RC_DEADBAND: {list(deadband)} (alt-hold 40, 3D-дедбенд 50)")

    motor = c14.request(131)
    check(len(motor) == 10 and u16(motor, 0) == 1070 and u16(motor, 4) == 1000,
          f"MSP_MOTOR_CONFIG: min {u16(motor, 0)}, max {u16(motor, 2)}, min_command {u16(motor, 4)}")
    check(motor[8] == 1, "bidirectional DShot включён (иначе «Динамический Idle» недоступен)")

    sliders = c14.request(140)
    check(len(sliders) == 53, f"MSP_SIMPLIFIED_TUNING: {len(sliders)} байт = 17 + 18 + 18")
    check(list(sliders[0:9]) == [2, 100, 100, 100, 100, 100, 100, 100, 100],
          "ползунки: режим RPY, все множители 100% (simplified_pids_mode = RPY)")
    check(list(c14.request(145)) == [1, 1, 1], "MSP_VALIDATE_SIMPLIFIED_TUNING: ползунки совпадают с PID")
    check(len(c14.request(142, bytes(sliders[:15]))) == 18,
          "MSP_CALCULATE_SIMPLIFIED_PID: 3 оси × (P, I, D, D Max, Feedforward)")

    status_ex = c14.request(150)
    check(status_ex[13] == 4, f"MSP_STATUS_EX: профилей PID {status_ex[13]} (PID_PROFILE_COUNT = 4)")
    check((c14.request(0x3006, bytes([3]), version=2))[2:].decode("latin-1") == "Profile 1",
          "имя PID-профиля читается (MSP2_GET_TEXT)")

    c14.request(202, bytes([55, 90, 45, 57, 94, 51, 55, 90, 5, 50, 75, 75, 40, 0, 0]))
    check(tuple(c14.request(112)[0:3]) == (55, 90, 45), "MSP_SET_PID (202): правки таблицы приняты")

    edit_adv = bytearray(advanced)
    edit_adv[17] = 70
    edit_adv[21:23] = u16_bytes(120)
    c14.request(95, bytes(edit_adv))
    check(c14.request(94)[17] == 70, "MSP_SET_PID_ADVANCED (95): «макс. угол наклона» сохранён")

    edit_rc = bytearray(rc)
    edit_rc[2] = 82
    c14.request(204, bytes(edit_rc))
    check(c14.request(111)[2] == 82, "MSP_SET_RC_TUNING (204): srate сохранён")

    edit_filters = bytearray(filters)
    edit_filters[1:3] = u16_bytes(85)
    edit_filters[5:7] = u16_bytes(180)     # гироскопический нотч, ползунки его не трогают
    edit_filters[7:9] = u16_bytes(90)
    c14.request(93, bytes(edit_filters))
    check(u16(c14.request(92), 1) == 85 and u16(c14.request(92), 5) == 180,
          "MSP_SET_FILTER_CONFIG (93): D-term LPF1 85 Гц и гиро-нотч 180 Гц сохранены")

    edit_sliders = bytearray(sliders[:35])
    edit_sliders[1] = 120      # «Множитель PID» 120%
    edit_sliders[18] = 120     # «Множитель фильтров D-term» 120% (17 — флаг «Фильтры D-term»)
    c14.request(141, bytes(edit_sliders))
    check(u16(c14.request(92), 1) == 90 and u16(c14.request(92), 33) == 90,
          f"множитель фильтров D-term 120%: статический {u16(c14.request(92), 1)} Гц, "
          f"динамический {u16(c14.request(92), 33)}…{u16(c14.request(92), 35)} Гц")
    check(tuple(c14.request(112)[0:3]) == (54, 96, 48),
          f"ползунок 120% пересчитал PID: {tuple(c14.request(112)[0:3])} (45/80/40 × 1.2)")

    c14.request(0x3007, b"\x03\x09MyProfile", version=2)
    check((c14.request(0x3006, bytes([3]), version=2))[2:].decode("latin-1") == "MyProfile",
          "имя PID-профиля записано (MSP2_SET_TEXT)")

    c14.request(210, bytes([1]))
    check(tuple(c14.request(112)[0:3]) == (45, 80, 40), "MSP_SELECT_SETTING (210): профиль 2 — заводские значения")
    c14.request(183, bytes([0, 1, 0]))
    check(tuple(c14.request(112)[0:3]) == (54, 96, 48), "MSP_COPY_PROFILE (183): профиль 1 скопирован в профиль 2")
    c14.request(219)
    check(tuple(c14.request(112)[0:3]) == (45, 80, 40), "MSP_SET_RESET_CURR_PID (219): профиль сброшен к заводским")
    c14.request(210, bytes([0]))
    check(tuple(c14.request(112)[0:3]) == (54, 96, 48), "профиль 1 сохранил свои правки (значения живут в профилях)")

    out = cli_over_msp(c14, "get p_roll")
    check("p_roll = 54" in out, "CLI: 'get p_roll' совпадает с таблицей вкладки")
    out = cli_over_msp(c14, "get d_max_gain")
    check("d_max_gain = 37" in out, "CLI: advanced-настройки видны через 'get'")

    c14.request(250)                       # «Сохранить»
    c14.request(68, b"\x00")              # «Сохранить и перезагрузить»
    time.sleep(0.8)
    c14.close()
    proc14.terminate()
    proc14.wait(timeout=10)

    proc15, c15 = start_emulator()
    check(tuple(c15.request(112)[0:3]) == (54, 96, 48), "PID пережили перезапуск эмулятора")
    filters = c15.request(92)
    check(u16(filters, 5) == 180 and u16(filters, 7) == 90, "гироскопический нотч пережил перезапуск")
    check(u16(filters, 1) == 90, "D-term LPF1 пересчитан ползунком (75 × 1.2 = 90) и тоже сохранён")
    check(c15.request(111)[2] == 82, "рейты пережили перезапуск")
    check(c15.request(140)[1] == 120, "ползунок пережил перезапуск")
    check(len(c15.request(117)) == 25, "вкладка загружается и после перезапуска (117 отвечает)")
    c15.close()
    proc15.terminate()
    proc15.wait(timeout=10)

    # ------------------------------------------------------ вкладка «Приёмник»
    print("17. Вкладка «Приёмник»: каналы A/E/R/T, «Предпросмотр», карта каналов, сохранение")
    proc16, c16 = start_emulator()

    rc = c16.request(105)
    channels = [u16(rc, i * 2) for i in range(len(rc) // 2)]
    check(len(channels) == 16, f"MSP_RC: {len(channels)} каналов (CRSF отдаёт 16)")
    check(channels[:3] == [1500, 1500, 1500] and channels[3] == 1000,
          f"порядок A/E/R/T: стики {channels[:3]} в центре, газ {channels[3]} внизу")
    check(all(950 <= value <= 1050 for value in channels[4:]), "AUX 1…12 выключены (тумблеры на столе)")

    def preview_rate(rc_data, rate, rc_rate, expo, deadband, limit):
        """RateCurve.rcCommandRawToDegreesPerSecond() для ACTUAL — как в ReceiverTab.vue."""
        command = min(max(abs(rc_data - 1500) - deadband, 0), 500) / (500 - deadband)
        angular = command * rc_rate + max(0, rate - rc_rate) * abs(command) * (command**5 * expo + command * (1 - expo))
        return max(-limit, min(limit, angular))

    rates = [preview_rate(channels[i], (67 * 1000), (7 * 1000), 0.0, 0, 1998) for i in range(3)]
    check(all(abs(rate) < 0.5 for rate in rates),
          f"«Предпросмотр»: угловые скорости {[round(r, 1) for r in rates]} °/с — коптер стоит на месте")
    check(abs(preview_rate(1000, 67 * 1000, 7 * 1000, 0.0, 0, 1998)) > 1000,
          "контроль: с прежними данными (Рыск=1000) модель крутилась бы на ~1998 °/с")

    check(list(c16.request(50)) == [0], "MSP_RSSI_CONFIG: аналоговый RSSI выключен (канал 0)")
    rx_map = list(c16.request(64))
    check(rx_map == [0, 1, 3, 2, 4, 5, 6, 7], f"MSP_RX_MAP: {rx_map} = AETR1234")
    rc_tuning = c16.request(111)
    check(len(rc_tuning) == 23 and rc_tuning[22] == 3,
          f"MSP_RC_TUNING: {len(rc_tuning)} байта, рейты ACTUAL (лимит {u16(rc_tuning, 16)}°/с)")
    rx_config = c16.request(44)
    check(len(rx_config) == 38, f"MSP_RX_CONFIG: {len(rx_config)} байт (раскладка MSPHelper.js)")
    check(rx_config[0] == 9 and (u16(rx_config, 5), u16(rx_config, 3), u16(rx_config, 1)) == (1050, 1500, 2000),
          f"Serial RX CRSF, «Диапазон стиков» {u16(rx_config, 5)} / {u16(rx_config, 3)} / {u16(rx_config, 1)} мкс")
    # В «флеше» после раздела 14 лежат правки Failsafe — заводские 885/2115 проверяет
    # tools/check_receiver_tab.py на чистом «флеше».
    check(750 <= u16(rx_config, 8) < u16(rx_config, 10) <= 2250 and rx_config[31] == 1 and rx_config[30] == 30,
          f"RC-сглаживание ВКЛ, множитель 30 (Авто); диапазон импульсов "
          f"{u16(rx_config, 8)}…{u16(rx_config, 10)} мкс (правки раздела 14)")
    check(list(c16.request(125)) == [0, 0, 40, 50, 0], "MSP_RC_DEADBAND: RC 0 / Yaw 0 / 3D-газ 50 мкс")
    check(list(c16.request(42)) == [2, 0], "MSP_MIXER_CONFIG: QUADX")

    c16.request(65, bytes([3, 0, 1, 2, 4, 5, 6, 7]))
    c16.request(51, bytes([7]))
    c16.request(218, bytes([4, 8, 40]) + u16_bytes(60))
    edit_rx = bytearray(rx_config)
    edit_rx[0] = 2                                                  # SBUS
    edit_rx[5:7] = u16_bytes(1000)
    c16.request(45, bytes(edit_rx))
    check(list(c16.request(64)) == [3, 0, 1, 2, 4, 5, 6, 7], "MSP_SET_RX_MAP (65): карта TAER1234 принята")
    check(list(c16.request(50)) == [7], "MSP_SET_RSSI_CONFIG (51): «Канал RSSI» = 7")
    check(list(c16.request(125))[:2] == [4, 8], "MSP_SET_RC_DEADBAND (218): RC 4 / Yaw 8")
    check(c16.request(44)[0] == 2 and u16(c16.request(44), 5) == 1000,
          "MSP_SET_RX_CONFIG (45): SBUS, «мин. стик» 1000 мкс")

    c16.request(250)                       # «Сохранить»
    c16.request(68, b"\x00")              # «Сохранить и перезагрузить»
    time.sleep(0.8)
    c16.close()
    proc16.terminate()
    proc16.wait(timeout=10)

    proc17, c17 = start_emulator()
    check(list(c17.request(64)) == [3, 0, 1, 2, 4, 5, 6, 7], "карта каналов пережила перезапуск")
    check(list(c17.request(50)) == [7], "«Канал RSSI» пережил перезапуск")
    check(c17.request(44)[0] == 2, "Serial RX (SBUS) пережил перезапуск")
    channels_after = [u16(c17.request(105), i * 2) for i in range(4)]
    check(channels_after == [1500, 1500, 1500, 1000],
          f"каналы после перезапуска: {channels_after} — «Предпросмотр» по-прежнему стоит")

    out = cli_over_msp(c17, "get serialrx_provider")
    check("serialrx_provider = SBUS" in out, "CLI: 'get serialrx_provider' совпадает с вкладкой")
    out = cli_over_msp(c17, "get rssi_channel")
    check("rssi_channel = 7" in out, "CLI: 'get rssi_channel' совпадает с вкладкой")
    out = cli_over_msp(c17, "map")
    check("map TAER1234" in out, "CLI: 'map' показывает сохранённую карту")
    out = cli_over_msp(c17, "get deadband")
    check("deadband = 4" in out, "CLI: 'get deadband' совпадает с вкладкой")

    cli_over_msp(c17, "defaults")
    time.sleep(0.7)
    c17.close()
    proc17.terminate()
    proc17.wait(timeout=10)
    proc18, c18 = start_emulator()
    check(list(c18.request(64)) == [0, 1, 3, 2, 4, 5, 6, 7] and list(c18.request(50)) == [0],
          "CLI: 'defaults' вернул AETR1234 и выключенный RSSI")
    check(c18.request(44)[0] == 9, "CLI: 'defaults' вернул Serial RX CRSF")
    c18.close()
    proc18.terminate()
    proc18.wait(timeout=10)

    # ------------------------------------------ виртуальная аппаратура («пульт»)
    print("18. Виртуальная аппаратура (TX): каналы через /api/rc, страница /tx, сброс")
    import json as _json
    import urllib.error
    import urllib.request

    http_port = port + 1
    proc19 = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--http", str(http_port),
         "--eeprom", str(eeprom), "--quiet"],
        cwd=root, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    for _ in range(40):
        try:
            c19 = Client(port=port, timeout=1.0)
            break
        except OSError:
            time.sleep(0.25)
    else:
        raise RuntimeError("эмулятор с веб-сервером не поднялся")

    def tx_get(path="/api/rc"):
        with urllib.request.urlopen(f"http://127.0.0.1:{http_port}{path}", timeout=4) as response:
            return response.read()

    def tx_post(payload):
        request = urllib.request.Request(
            f"http://127.0.0.1:{http_port}/api/rc",
            data=_json.dumps(payload).encode(), headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=4) as response:
                return response.status, _json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, _json.loads(exc.read())

    def msp_channels():
        data = c19.request(105)
        return [u16(data, i * 2) for i in range(len(data) // 2)]

    state = _json.loads(tx_get())
    check(state["channels"] == [1500, 1500, 1500, 1000] + [1000] * 12,
          f"«аппаратура подключена, стики стоят»: {state['channels'][:4]} + 12 тумблеров выключены")
    check(state["names"][:4] == ["roll", "pitch", "yaw", "throttle"] and len(state["names"]) == 16,
          "каналы подписаны: roll/pitch/yaw/throttle + aux1…aux12")

    status, body = tx_post({"roll": 1700, "throttle": 1200})
    check(status == 200 and body["applied"] == 2, f"POST /api/rc: применено {body.get('applied')} канала")
    check(msp_channels()[:4] == [1700, 1500, 1500, 1200],
          f"MSP_RC отдаёт стики аппаратуры: {msp_channels()[:4]}")
    check(body["channels"][0] == 1700, "ответ API возвращает новые значения (страница /tx сразу видит их)")

    roll_rate = preview_rate(1700, 67 * 1000, 7 * 1000, 0.0, 0, 1998)
    yaw_rate = preview_rate(1500, 67 * 1000, 7 * 1000, 0.0, 0, 1998)
    check(abs(roll_rate) > 1000 and yaw_rate == 0,
          f"«Предпросмотр» наклоняется от стика: крен {roll_rate:.0f} °/с, рыск {yaw_rate:.0f} °/с")

    status, _ = tx_post({"aux1": 2000})
    check(status == 200 and msp_channels()[4] in (1999, 2000, 2001),
          f"тумблер AUX 1 включён: канал {msp_channels()[4]} мкс (режимы на вкладке «Режимы» сработают)")

    status, body = tx_post({"foo": 1})
    check(status == 400 and "не распознаны" in body["error"], f"неизвестный канал: HTTP {status} и понятная ошибка")

    # Отказ обязан приходить ДО изменений: иначе 400 оставлял бы часть запроса применённой
    source_before = _json.loads(tx_get())["source"]
    status, body = tx_post({"source": "смоук-400", "roll": 1600, "нет_такого_канала": 1500})
    check(status == 400 and "не распознаны" in body["error"] and body["state"]["channels"][0] != 1600,
          f"неизвестный канал в одном запросе с валидным полем: HTTP {status}, валидное поле не применено")
    check(_json.loads(tx_get())["source"] == source_before,
          f"после отказа источник остался прежним ({source_before})")
    status, body = tx_post({"roll": "не-число"})
    check(status == 400 and "числами" in body["error"],
          f"нечисловое значение канала: HTTP {status} и понятная ошибка (раньше соединение рвалось)")
    status, _ = tx_post({"roll": 1650})
    check(status == 200 and msp_channels()[0] == 1650,
          "после отказов сервер принимает запросы дальше и каналы пишутся")

    status, body = tx_post({"reset": True})
    check(status == 200 and body["channels"] == [1500, 1500, 1500, 1000] + [1000] * 12,
          "сброс: стики в центр, тумблеры выключены")

    page = tx_get("/tx").decode("utf-8")
    check('href="/aux"' in page and 'href="/msc"' in page and "class=nav" in page,
          "на странице /tx есть полоска кнопок: одним нажатием уйти на «Режимы» и «Чёрный ящик»")
    nav_page = tx_get("/nav").decode("utf-8")
    check(all(f'href="{href}"' in nav_page for href in ("/tx", "/aux", "/msc", "/")),
          "страница «Переходы» (/nav) ведёт на /tx, /aux, /msc и монитор")
    aux_page = tx_get("/aux").decode("utf-8")
    check("Режимы (AUX)" in aux_page and 'id="rows"' in aux_page and "permanentId" in aux_page,
          "страница /aux отдаётся: таблица слотов, справочник permanentId и автообновление")
    check('id="roll"' in page and 'id="throttle"' in page and "/api/rc" in page and "Стики в центр" in page,
          "страница /tx отдаётся: четыре стика, тумблеры AUX и кнопки управления")
    check("1.46" in tx_get("/api/state").decode("utf-8"), "/api/state тоже показывает каналы (раздел rc)")

    # «Выдернуть USB»: прошивка отдаёт значения «Этапа 1» и поднимает флаг RX_FAILSAFE
    status, body = tx_post({"connected": False, "source": "smoke"})
    check(status == 200 and body["failsafe"] and body["active_channels"] == 16,
          "аппаратура отключена: каналы на месте, но прошивка в failsafe")
    stage1 = msp_channels()
    check(stage1[:4] == [1500, 1500, 1500, 900],
          f"значения «Этапа 1» (rx.c): стики в центре, газ {stage1[3]} мкс (rx_min_usec по сетке 25 мкс)")
    flags = int.from_bytes(c19.request(150)[17:21], "little")
    check(flags & (1 << 2) != 0,
          f"флаг ARMING_DISABLED_RX_FAILSAFE поднят (0x{flags:08X}) — вкладка «Приёмник» покажет «потеря связи»")

    status, body = tx_post({"connected": True})
    check(status == 200 and not body["failsafe"] and body["channels"][:4] == [1500, 1500, 1500, 1000],
          "аппаратура подключена обратно: каналы снова живые")
    flags = int.from_bytes(c19.request(150)[17:21], "little")
    check(flags & (1 << 2) == 0, f"флаг потери связи снят (0x{flags:08X})")

    # «Порты»: без функции Serial RX прошивка не поднимает приёмник — MSP_RC пуст
    RX_SERIAL_MASK = 0x40
    ports_payload = c19.request(4105, version=2)
    port_count = ports_payload[0]
    stripped = bytearray(ports_payload)
    for i in range(port_count):
        if stripped[1 + i * 9] == 1:                      # UART2 — порт приёмника
            mask = int.from_bytes(stripped[1 + i * 9 + 1 : 1 + i * 9 + 5], "little") & ~RX_SERIAL_MASK
            stripped[1 + i * 9 + 1 : 1 + i * 9 + 5] = mask.to_bytes(4, "little")
    c19.request(4106, bytes(stripped), version=2)
    check(c19.request(105) == b"", "Serial RX выключен во вкладке «Порты»: MSP_RC пуст (приёмник не поднят)")
    check(_json.loads(tx_get())["serial_ok"] is False, "страница /tx пишет: «приёмник не настроен»")
    c19.request(4106, ports_payload, version=2)
    check(len(c19.request(105)) == 32, "Serial RX включён обратно — 16 каналов вернулись")

    # настоящая аппаратура: кодек CRSF (0x16) — тем же режимом читается COM-порт (tools/tx.py crsf)
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
    import tx as tx_tool
    raw_frame = tx_tool.build_crsf_channels_frame([988, 1500, 2012, 1000] + [1000] * 12)
    check(len(raw_frame) == 26 and raw_frame[0] == 0xC8 and raw_frame[2] == 0x16,
          f"кадр CRSF RC: {len(raw_frame)} байт, адрес 0xC8, тип 0x16")
    decoded = tx_tool.decode_crsf_rc_frame(raw_frame)
    check(decoded is not None and abs(decoded[0] - 988) <= 1 and abs(decoded[1] - 1500) <= 1
          and abs(decoded[2] - 2012) <= 1,
          f"декодер повторяет crsfReadRawRC(): 172/992/1811 -> {decoded[:3]} мкс")
    check(abs(tx_tool.crsf_raw_to_us(tx_tool.crsf_us_to_raw(1500)) - 1500) <= 1
          and tx_tool.crsf_us_to_raw(988) == 172,
          f"преобразование туда-обратно: 1500 мкс -> {tx_tool.crsf_us_to_raw(1500)} -> "
          f"{tx_tool.crsf_raw_to_us(tx_tool.crsf_us_to_raw(1500))} мкс; 988 мкс -> 172 (минимум CRSF)")
    spoiled = bytearray(raw_frame)
    spoiled[-1] ^= 0xFF
    check(tx_tool.decode_crsf_rc_frame(bytes(spoiled)) is None, "кадр с неверным CRC отбрасывается")

    # «стик в крайнем положении»: у виртуальной аппаратуры диапазон 1000…2000 мкс
    channels_from_crsf = tx_tool.decode_crsf_rc_frame(
        tx_tool.build_crsf_channels_frame([1100, 1500, 1900, 1200] + [1000] * 12))
    status, body = tx_post({"channels": channels_from_crsf, "source": "CRSF COM5"})
    check(status == 200 and msp_channels()[:4] == [1100, 1500, 1900, 1200] and body["source"] == "CRSF COM5",
          f"кадры CRSF попадают в каналы FC: {msp_channels()[:4]} (источник виден на странице /tx)")

    # стики — не конфигурация FC: во «флеш» они не попадают
    tx_post({"roll": 1800})
    c19.request(250)                       # MSP_EEPROM_WRITE
    c19.request(68, b"\x00")              # MSP_SET_REBOOT
    time.sleep(0.8)
    c19.close()
    proc19.terminate()
    proc19.wait(timeout=10)
    proc20, c20 = start_emulator()
    channels_after_restart = [u16(c20.request(105), i * 2) for i in range(4)]
    check(channels_after_restart == [1500, 1500, 1500, 1000],
          f"после перезапуска стики снова в центре ({channels_after_restart}) — «аппаратура» не пишется во «флеш»")
    c20.close()
    proc20.terminate()
    proc20.wait(timeout=10)

    # ------------------------------- старый «флеш» с устаревшим эхом (регрессия 25.09.2026)
    print("19. Старый «флеш»: эхо прежних сборок не глушит живые значения")
    import json as _json
    import tempfile as _tempfile

    stale_dir = Path(_tempfile.mkdtemp())
    stale_eeprom = stale_dir / "eeprom.json"
    stale_eeprom.write_text(_json.dumps({
        "version": 1,
        "settings": {},
        # эхо записанных payload'ов, как их оставляли прежние сборки
        "config_payloads": {
            "36": "00000000",                     # фичи: «всё выключено»
            "44": (bytes(38)).hex(),              # RX_CONFIG: нули
            "61": "000014",                       # arming: small_angle 20
            "140": bytes(53).hex(),               # слайдеры: нули
        },
        "features": ["RX_SERIAL"],                # в состоянии — только RX_SERIAL
        "serial_ports": [{"identifier": 1, "functions": ["RX_SERIAL"]}],
    }), encoding="utf-8")

    stale_port = 15889

    def start_stale():
        proc = subprocess.Popen(
            [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{stale_port}", "--eeprom", str(stale_eeprom), "--quiet"],
            cwd=root, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        for _ in range(40):
            try:
                client = Client(port=stale_port, timeout=1.0)
            except OSError as exc:
                if proc.poll() is not None:
                    raise RuntimeError(
                        f"эмулятор раздела 19 не поднялся: порт {stale_port} занят другой копией. "
                        f"Остановите её и запустите тест снова") from exc
                time.sleep(0.25)
                continue
            if proc.poll() is not None:
                raise RuntimeError(
                    f"на порту {stale_port} отвечает ЧУЖОЙ эмулятор (наша копия уже завершилась)")
            return proc, client
        raise RuntimeError("эмулятор не поднялся")

    proc21, c21 = start_stale()
    feats_stale = int.from_bytes(c21.request(36), "little")
    check(feats_stale == 1 << 3,
          f"фичи берутся из состояния, а не из эха: 0x{feats_stale:08X} (только RX_SERIAL)")
    rx_stale = c21.request(44)
    check(rx_stale[0] == 9 and len(rx_stale) == 38,
          f"RX_CONFIG тоже из состояния: provider = {rx_stale[0]} (CRSF), эхо-нули не влияют (длина {len(rx_stale)})")
    check(rx_stale[8:10] == (885).to_bytes(2, "little") and rx_stale[10:12] == (2115).to_bytes(2, "little"),
          "диапазон импульсов 885…2115 мкс читается из состояния, а не из нулевого эха")

    c21.request(37, (feats_stale | (1 << 6) | (1 << 5)).to_bytes(4, "little"))
    feats_now = int.from_bytes(c21.request(36), "little")
    check(feats_now & (1 << 6) and feats_now & (1 << 5),
          f"SOFTSERIAL и SERVO_TILT включаются и видны сразу: 0x{feats_now:08X} — «не сохранялись» из-за эха")
    ports_now = c21.request(4105, version=2)
    port_ids = [ports_now[1 + i * 9] for i in range(ports_now[0])]
    check(30 in port_ids and 31 in port_ids, f"вкладка «Порты»: {port_ids} (SOFTSERIAL1/2 появились)")

    c21.request(250)                       # Save
    c21.request(68, b"\x00")              # Save and Reboot
    time.sleep(0.8)
    c21.close()
    proc21.terminate()
    proc21.wait(timeout=10)
    proc22, c22 = start_stale()
    feats_after = int.from_bytes(c22.request(36), "little")
    check(feats_after & (1 << 6) and feats_after & (1 << 5),
          f"после перезапуска фичи на месте: 0x{feats_after:08X}")
    ports_after = c22.request(4105, version=2)
    port_ids_after = [ports_after[1 + i * 9] for i in range(ports_after[0])]
    check(30 in port_ids_after and 31 in port_ids_after, f"порты SOFTSERIAL1/2 пережили перезапуск: {port_ids_after}")
    saved_state = _json.loads(stale_eeprom.read_text())
    check("36" not in (saved_state.get("config_payloads") or {}) and
          "44" not in (saved_state.get("config_payloads") or {}),
          f"устаревшее эхо убрано из «флеша»: остались {sorted(saved_state.get('config_payloads') or {})}")
    # сквозные (не моделируемые) блоки по-прежнему хранятся эхом
    c22.request(185, (1).to_bytes(4, "little") + bytes([9]) + (0x02).to_bytes(4, "little"))
    c22.request(250)
    c22.close()
    proc22.terminate()
    proc22.wait(timeout=10)
    proc23, c23 = start_stale()
    beeper_after = c23.request(184)
    check(len(beeper_after) > 4 and beeper_after[4] == 9,
          f"сквозная запись (BEEPER_CONFIG) работает как раньше: тон = {beeper_after[4] if len(beeper_after) > 4 else '—'}")
    c23.close()
    proc23.terminate()
    proc23.wait(timeout=10)

    print("20. Живая аппаратура (Zorro и др.): биндинг, SBUS и приёмник в COM-порту")
    import json as _json20
    import tempfile as _tempfile20

    live_dir = Path(_tempfile20.mkdtemp())
    live_eeprom = live_dir / "eeprom.json"
    live_port = 15890
    live_http = live_port + 1

    def start_live():
        proc = subprocess.Popen(
            [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{live_port}",
             "--http", str(live_http), "--eeprom", str(live_eeprom), "--quiet"],
            cwd=root, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        for _ in range(40):
            try:
                client = Client(port=live_port, timeout=1.0)
            except OSError as exc:
                if proc.poll() is not None:
                    raise RuntimeError(
                        f"эмулятор раздела 20 не поднялся: порт {live_port} занят другой копией. "
                        f"Остановите её (Ctrl+C в её окне) и запустите тест снова") from exc
                time.sleep(0.25)
                continue
            if proc.poll() is not None:
                raise RuntimeError(
                    f"на порту {live_port} отвечает ЧУЖОЙ эмулятор (наша копия уже завершилась). "
                    f"Остановите лишние копии и запустите тест снова")
            return proc, client
        proc.terminate()
        raise RuntimeError("эмулятор раздела 20 не поднялся")

    def live_get(path="/api/rc"):
        with urllib.request.urlopen(f"http://127.0.0.1:{live_http}{path}", timeout=4) as response:
            return _json20.loads(response.read())

    def live_post(payload):
        request = urllib.request.Request(
            f"http://127.0.0.1:{live_http}/api/rc",
            data=_json20.dumps(payload).encode(), headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=4) as response:
                return response.status, _json20.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, _json20.loads(exc.read())

    proc20, c20 = start_live()
    state20 = live_get()
    check(state20["bind"]["state"] == "bound" and state20["bind"]["ready"] and state20["serial_ok"],
          f"после запуска аппаратура «связана»: {state20['bind']['text']}")

    def stage1_channels():
        data = c20.request(105)
        return [u16(data, i * 2) for i in range(len(data) // 2)]

    status, body = live_post({"bind": False, "source": "smoke"})
    check(status == 200 and body["bind"]["state"] == "none" and not body["connected"],
          f"«Развязать»: бинда нет, кадров нет ({body['bind']['text']})")
    check(stage1_channels()[:4] == [1500, 1500, 1500, 900],
          f"без бинда приёмник отдаёт «Этап 1»: {stage1_channels()[:4]} — как при потере связи")
    flags20 = int.from_bytes(c20.request(150)[17:21], "little")
    check(flags20 & (1 << 2) != 0,
          f"флаг RX_FAILSAFE поднят и без бинда (0x{flags20:08X}) — вкладка «Приёмник» предупредит")

    status, body = live_post({"bind": True, "bind_seconds": 1.5})
    check(status == 200 and body["bind"]["state"] == "binding" and not body["connected"],
          f"«Связать»: идёт поиск приёмника ({body['bind']['seconds_left']} с), кадров пока нет")
    check(stage1_channels()[:4] == [1500, 1500, 1500, 900],
          "пока идёт биндинг, на вкладке «Приёмник» тоже «Этап 1» — как у настоящего приёмника")
    time.sleep(1.8)
    after_bind = live_get()
    check(after_bind["bind"]["state"] == "bound" and after_bind["connected"],
          f"биндинг завершился сам: {after_bind['bind']['text']}")
    flags20 = int.from_bytes(c20.request(150)[17:21], "little")
    check(flags20 & (1 << 2) == 0, f"флаг потери связи снят после бинда (0x{flags20:08X})")

    live_post({"roll": 1750, "source": "smoke"})
    check(stage1_channels()[:4] == [1750, 1500, 1500, 1000],
          f"каналы живые: крен {stage1_channels()[0]} мкс — стики снова управляют")
    live_post({"bind": True, "bind_seconds": 0, "reset": True})
    check(live_get()["bind"]["state"] == "bound" and stage1_channels()[:4] == [1500, 1500, 1500, 1000],
          "мгновенный бинд (bind_seconds: 0) и стики в центр")

    # --- у кого управление: физическая аппаратура или эта страница (кнопки /tx) ---
    state20 = live_get()
    check(state20["control"]["mode"] == "virtual" and state20["control"]["virtual"],
          f"по умолчанию управляет эта страница: {state20['control']['text']}")
    check(state20["aux_types"]["aux1"] == "switch" and state20["aux_types"]["aux3"] == "switch3"
          and state20["aux_types"]["aux4"] == "switch3" and state20["aux_types"]["aux7"] == "knob"
          and state20["aux_types"]["aux10"] == "knob",
          "типы AUX из прошивки: AUX 3/4 — трёхпозиционные, AUX 7/10 — крутилки, остальные — тумблеры")

    status, body = live_post({"control": "physical", "source": "smoke"})
    check(status == 200 and body["control"]["physical"] and not body["connected"]
          and body["bind"]["state"] == "none",
          "«Подключить аппаратуру»: управление у физической, ползунки страницы гаснут, связи нет")
    check(stage1_channels()[:4] == [1500, 1500, 1500, 900] and
          (int.from_bytes(c20.request(150)[17:21], "little") & (1 << 2)) != 0,
          "до биндинга каналы не активны: «Этап 1» и флаг RX_FAILSAFE (как и требует приёмник)")
    status, body = live_post({"bind": True, "bind_seconds": 0})
    check(status == 200 and body["bind"]["state"] == "bound" and body["connected"],
          "«Связать (Bind)» после подключения аппаратуры: каналы активировались")
    # управление у аппаратуры — значит её тумблеры и крутилки приходят с флагом physical
    live_post({"aux3": 1500, "aux7": 1730, "physical": True, "source": "пульт: AUX"})
    check(stage1_channels()[6] == 1500 and stage1_channels()[10] == 1730,
          f"трёхпозиционный AUX 3 (середина {stage1_channels()[6]}) и крутилка AUX 7 "
          f"({stage1_channels()[10]} мкс) работают как каналы")

    status, body = live_post({"control": "virtual", "source": "smoke"})
    check(status == 200 and body["control"]["virtual"] and not body["connected"]
          and body["bind"]["state"] == "none",
          "«Отключить (выдернуть USB)»: управление вернулось странице, связи нет — нужен бинд")
    check(stage1_channels()[:4] == [1500, 1500, 1500, 900],
          "после отключения аппаратуры каналы снова «Этап 1» (полосы стоят)")
    live_post({"bind": True, "bind_seconds": 0, "reset": True})
    check(live_get()["bind"]["state"] == "bound" and stage1_channels()[:4] == [1500, 1500, 1500, 1000],
          "«Связать» возвращает живые каналы виртуальной аппаратуры")

    # --- «чей ход — того и каналы»: данные физической аппаратуры не принимаются,
    #     пока управление у страницы /tx (иначе два источника давали «срывы») ---
    live_post({"control": "physical", "bind": True, "bind_seconds": 0})
    status, body = live_post({"channels": {"roll": 1900}, "physical": True, "source": "пульт"})
    check(status == 200 and body["channels"][0] == 1900,
          f"физическая аппаратура пишет каналы, когда управление у неё (крен {body['channels'][0]})")
    status, body = live_post({"channels": {"roll": 1400}, "source": "страница /tx"})
    check(status == 409 and "аппаратуры" in body.get("error", "") and body["state"]["channels"][0] == 1900,
          f"пока каналами владеет аппаратура, страница и скрипты не пишут (HTTP {status}), "
          f"крен остался {body['state']['channels'][0]}")
    live_post({"control": "virtual", "source": "smoke"})
    status, body = live_post({"channels": {"roll": 1000}, "physical": True, "source": "пульт"})
    check(status == 409 and "виртуальной аппаратуры" in body.get("error", "")
          and body["state"]["channels"][0] == 1900,
          f"после «Отключить (выдернуть USB)» данные пульта отклоняются (HTTP {status}), "
          f"канал не изменился: {body['state']['channels'][0]}")
    status, body = live_post({"channels": {"roll": 1200}, "source": "страница /tx"})
    check(status == 200 and body["channels"][0] == 1200,
          "а сама страница /tx каналы пишет — управление у неё")

    # --- раскладка осей пульта видна в /api/rc (её показывает страница /tx) ---
    live_post({"control": "physical", "bind": True, "bind_seconds": 0,
               "rc_map": {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 4,
                          "invert": ["throttle"], "device": "Zorro"}})
    state20 = live_get()
    check(state20["map"].get("roll") == 0 and state20["map"].get("yaw") == 4
          and state20["map"].get("invert") == ["throttle"],
          f"раскладка осей пульта отдаётся в /api/rc: {state20['map']}")

    control_cli = subprocess.run([sys.executable, str(root / "tools" / "tx.py"), "control", "physical",
                                  "--http", str(live_http)],
                                 cwd=root, env=env, capture_output=True, text=True, timeout=60)
    check(control_cli.returncode == 0 and "физической" in control_cli.stdout
          and live_get()["control"]["physical"],
          "python tools/tx.py control physical — передача управления из консоли")
    live_post({"control": "virtual", "reset": True, "bind": True, "bind_seconds": 0})

    # один «перехват» управления вторым источником — это норма, тревоги быть не должно.
    # Ждём, пока «утихнут» источники предыдущих проверок (окно наблюдения — 5 с).
    time.sleep(5.2)
    live_post({"channels": {"roll": 1500}, "source": "USB-пульт: Zorro [PID 111]"})
    live_post({"channels": {"roll": 1500}, "source": "USB-пульт: Zorro [PID 222]"})
    check(not live_get().get("multi_source"),
          "простая передача управления другому источнику тревоги не вызывает")
    # а вот когда два окна инструмента пишут по очереди (A, B, A, B…) — каналы «прыгают»
    for pid in (111, 222, 111, 222, 111):
        live_post({"channels": {"roll": 1500}, "source": f"USB-пульт: Zorro [PID {pid}]"})
    live_multi = live_get()
    check(live_multi.get("multi_source") and live_multi.get("source_changes", 0) >= 3
          and live_multi.get("updated_ago") is not None,
          f"эмулятор видит, что каналы пишут два окна инструмента по очереди "
          f"({live_multi.get('source_changes')} переключений источника)")

    live_post({"layout_note": "канал ГАЗ назначен на ось 2 (Z), а в покое эта ось в ЦЕНТРЕ — "
                              "раскладка осей не та", "source": "smoke"})
    check("раскладка осей не та" in (live_get().get("layout_note") or ""),
          "предупреждение о раскладке доходит до /api/rc — его показывает страница /tx")

    page_tx = urllib.request.urlopen(f"http://127.0.0.1:{live_http}/tx", timeout=4).read().decode("utf-8")
    # скрипт страницы должен быть без синтаксических ошибок (иначе страница «пустая»:
    # вставленный по невнимательности «#» в JS ломал её целиком — проверяем Node, если он есть)
    script = page_tx.split("<script>", 1)[1].rsplit("</script>", 1)[0]
    script_path = Path(tempfile.gettempdir()) / "bfemu_tx_page_check.js"
    script_path.write_text(script, encoding="utf-8")
    node = shutil.which("node")
    if node:
        node_run = subprocess.run([node, "--check", str(script_path)],
                                  capture_output=True, text=True, timeout=60)
        check(node_run.returncode == 0,
              f"скрипт страницы /tx проходит проверку синтаксиса Node ({node_run.stderr.strip()[:80] or 'чисто'})")
    script_path.unlink(missing_ok=True)

    # тот же скрипт запускаем в Node с заглушкой браузера: так видны ошибки, которых
    # не видит простая проверка синтаксиса (например, обращение к переменной не в своей области)
    if node:
        stub = """
const element = () => new Proxy({style: {}, classList: {add() {}, remove() {}}, dataset: {}},
  {get: (t, k) => (k in t ? t[k] : (k === "appendChild" || k === "addEventListener" ? () => {} : "")),
   set: (t, k, v) => (t[k] = v, true)});
global.document = {getElementById: element, querySelectorAll: () => [], addEventListener: () => {},
                   createElement: element, body: element()};
global.window = {addEventListener: () => {}};
global.setInterval = () => 1;
const state = {
  channels: [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000],
  connected: true, serial_ok: true, serial_port: "UART2", source: "test",
  control: {mode: "physical", physical: true, virtual: false, text: "physical"},
  map: {roll: 0, pitch: 1, throttle: 3, yaw: 5, invert: ["throttle"], device: "Zorro"},
  layout_note: "layout", bind: {state: "bound", text: "bound"}};
global.fetch = async () => ({ok: true, json: async () => state});
"""
        runtime = """
(async () => { await sync(); render(); stateText("t"); await apply({roll: 1600}, "t");
  console.log("OK"); })().catch(e => { console.error("ОШИБКА:", e && e.message); process.exit(3); });
"""
        run_path = Path(tempfile.gettempdir()) / "bfemu_tx_page_run.js"
        run_path.write_text(stub + script + runtime, encoding="utf-8")
        run = subprocess.run([node, str(run_path)], capture_output=True, text=True, timeout=60)
        check(run.returncode == 0 and "OK" in run.stdout,
              "скрипт страницы /tx по-настоящему проигрывается (синхронизация, отрисовка, "
              f"движение ползунка): {(run.stderr.strip().splitlines() or [''])[-1][:90] or 'без ошибок'}")
        run_path.unlink(missing_ok=True)

    # --- раздел «CLI-команды» монитора: команда из вкладки CLI должна быть видна,
    #     и скрипт самой страницы должен по-настоящему проигрываться (а не падать,
    #     например, на опечатке вида .strip() вместо .trim())
    print("17в. Монитор: раздел «CLI-команды» наполняется из CLI Configurator")
    live_cli_sock = socket.create_connection(("127.0.0.1", live_port), timeout=4)
    live_cli_sock.settimeout(0.7)
    live_cli_sock.sendall(b"#")
    time.sleep(0.3)
    try:
        live_cli_sock.recv(4096)
    except TimeoutError:
        pass
    live_cli_sock.sendall(b"get serialrx_provider\r")
    time.sleep(0.4)
    try:
        live_cli_sock.recv(8192)
    except TimeoutError:
        pass
    live_cli_sock.close()
    time.sleep(0.3)
    live_journal = json.loads(
        urllib.request.urlopen(f"http://127.0.0.1:{live_http}/api/state", timeout=4).read()
    )["cli"]
    check(any(entry[1] == "get serialrx_provider" for entry in live_journal),
          f"команда из CLI Configurator попала в журнал монитора: "
          f"{[e[1] for e in live_journal[-3:]]}")

    node = shutil.which("node")            # пригодится для прогона скрипта монитора
    monitor_page = urllib.request.urlopen(f"http://127.0.0.1:{live_http}/", timeout=4).read().decode("utf-8")
    check("cli-body" in monitor_page and "get serialrx_provider" in monitor_page
          and "обновляется само" in monitor_page,
          "монитор показывает команду в карточке «CLI-команды» (и сам обновляется)")
    monitor_script = monitor_page.split("<script>", 1)[1].rsplit("</script>", 1)[0]
    if node:
        monitor_run = Path(tempfile.gettempdir()) / "bfemu_monitor_run.js"
        monitor_run.write_text("""
const store = {};
global.document = {getElementById: (id) => (store[id] = store[id] || {dataset: {}, innerHTML: ""})};
global.fetch = async () => ({json: async () => ({
  cli: [[1700000000, "status", "MCU F40X"], [1700000001, "get serialrx_provider", "serialrx_provider = CRSF"]],
  msp: [[1700000002, 0x3012, "MSP2_CLI_COMMAND", 4, 483]]})});
global.setInterval = () => 1;
""" + monitor_script + """
(async () => { await new Promise(r => setTimeout(r, 60));
  const cli = store["cli-body"].innerHTML, msp = store["msp-body"].innerHTML;
  console.log(cli.includes("status") && cli.includes("CRSF") && msp.includes("0x3012") ? "OK" : "ПУСТО");
})().catch(e => { console.error("ОШИБКА:", e && e.message); process.exit(3); });
""", encoding="utf-8")
        monitor_probe = subprocess.run([node, str(monitor_run)], capture_output=True, text=True, timeout=60)
        check(monitor_probe.returncode == 0 and "OK" in monitor_probe.stdout,
              "скрипт монитора по-настоящему проигрывается: карточки CLI и MSP заполняются "
              f"({(monitor_probe.stderr.strip().splitlines() or [''])[-1][:70] or 'без ошибок'})")
        monitor_run.unlink(missing_ok=True)

    check("Связать (Bind)" in page_tx and "Развязать" in page_tx
          and "run_tx_joystick.bat" in page_tx and "pygame" not in page_tx,
          "страница /tx: кнопки «Связать (Bind)»/«Развязать» и подсказка про USB Joystick — "
          "без упоминаний библиотек")
    check("Подключить аппаратуру" in page_tx and "Отключить (выдернуть USB)" in page_tx
          and "control_hint" in page_tx and 'id="aux_card"' in page_tx
          and "buildAux" in page_tx and "switch3" in page_tx and "крутилка" in page_tx
          and 'id="layout_note"' in page_tx and 'id="sources_note"' in page_tx
          and "Каналы пишут несколько источников" in page_tx
          and "раскладка осей пульта пока не приходила" in page_tx
          and "раскладка осей пульта пока не приходила" in page_tx,
          "страница /tx: кнопки подключения/отключения аппаратуры и разные органы AUX "
          "(тумблеры, трёхпозиционные, крутилки)")

    # --- команды CLI прошивки 4.5.4, которых раньше не было (отвечали UNKNOWN COMMAND).
    # Раздел идёт на своём эмуляторе со своим «флешем»: он переключает профили, крутит
    # моторы и стирает накопитель, а живой стенд из 17в должен остаться нетронутым.
    print("17г. Команды CLI прошивки 4.5.4: профили, rxfail, rxrange, adjrange, моторы, DShot")
    cli454_dir = Path(tempfile.mkdtemp(prefix="bfemu_cli454_"))
    cli454_eeprom = cli454_dir / "eeprom.json"
    cli454_port = 15892      # свободный: 15888/15890 — стенды, 15889 — раздел 19
    # Если порт уже занят (забытая копия эмулятора с прежним кодом), тест молча работал бы
    # со СТАРЫМ экземпляром — проверяем порт заранее и просим остановить его.
    try:
        _probe = socket.create_connection(("127.0.0.1", cli454_port), timeout=0.5)
        _probe.close()
        raise RuntimeError(f"порт {cli454_port} занят другой копией эмулятора: "
                           f"остановите её (killemu.py) и запустите тест снова")
    except OSError:
        pass
    proc454 = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{cli454_port}",
         "--eeprom", str(cli454_eeprom), "--quiet"],
        cwd=root, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    for _ in range(40):
        try:
            c454 = Client(port=cli454_port, timeout=1.0)
            break
        except OSError as exc:
            if proc454.poll() is not None:
                raise RuntimeError(
                    f"эмулятор раздела 17г не поднялся: порт {cli454_port} занят другой копией. "
                    f"Остановите её и запустите тест снова") from exc
            time.sleep(0.25)
    else:
        raise RuntimeError("эмулятор раздела 17г не поднялся")

    # 1) ни одна команда из таблицы прошивки 4.5.4 (cli.c cmdTable[]) не отвечает UNKNOWN
    firmware_commands = [
        "adjrange", "beacon", "bind_rx", "board_name", "color", "dshot_telemetry_info",
        "dshotprog", "escprog", "flash_erase", "flash_read", "flash_scan", "flash_write",
        "gyroregisters", "led", "manufacturer_id", "mmix", "mode_color", "motor",
        "play_sound", "profile", "rateprofile", "rc_smoothing_info", "rxfail", "rxrange",
        "sd_info", "serialpassthrough", "signature", "simplified_tuning",
        # 'msc' проверяется отдельно (последним): он уводит борт в режим ЗУ, и MSP умолкает.
    ]
    unknown = [name for name in firmware_commands
               if "UNKNOWN COMMAND" in cli_over_msp(c454, name)]
    check(not unknown, f"все {len(firmware_commands)} команд из таблицы прошивки 4.5.4 "
                       f"распознаются (не распознаны: {unknown})")

    # 2) профили PID и рейтов — как cliProfile()/cliRateProfile()
    check(cli_over_msp(c454, "profile").strip() == "profile 0", "CLI: 'profile' печатает 'profile 0'")
    check(cli_over_msp(c454, "profile 2").strip() == "profile 2", "CLI: 'profile 2' переключает профиль")
    check("PROFILE OUTSIDE OF [0..3]" in cli_over_msp(c454, "profile 9"),
          "CLI: 'profile 9' отвечает 'PROFILE OUTSIDE OF [0..3]' — как cliProfile()")
    check(cli_over_msp(c454, "rateprofile 1").strip() == "rateprofile 1",
          "CLI: 'rateprofile 1' переключает профиль рейтов")
    check("RATE PROFILE OUTSIDE OF [0..3]" in cli_over_msp(c454, "rateprofile 4"),
          "CLI: 'rateprofile 4' отвечает 'RATE PROFILE OUTSIDE OF [0..3]'")

    # 2б) профиль батареи — команда из МАСТЕРА BF (после 4.5.4): в 4.5.4 профилей нет,
    # поведение и текст ошибки — по cliBatteryProfile() мастера
    check(cli_over_msp(c454, "battery_profile").strip() == "battery_profile 0",
          "CLI: 'battery_profile' печатает активный профиль батареи ('battery_profile 0')")
    check(cli_over_msp(c454, "battery_profile 2").strip() == "battery_profile 2"
          and cli_over_msp(c454, "battery_profile").strip() == "battery_profile 2",
          "CLI: 'battery_profile 2' переключает профиль батареи")
    check("BATTERY PROFILE OUTSIDE OF [0..2]" in cli_over_msp(c454, "battery_profile 9"),
          "CLI: 'battery_profile 9' отвечает 'BATTERY PROFILE OUTSIDE OF [0..2]' — как мастер")
    check("battery_profile - change battery profile" in cli_over_msp(c454, "help battery_profile"),
          "CLI: 'help battery_profile' печатает описание команды мастера")

    # 3) rxfail: 18 каналов, режимы a/h/s, значение только для SET
    rxfail_dump = cli_over_msp(c454, "rxfail").strip().splitlines()
    check(len(rxfail_dump) == 18 and rxfail_dump[0] == "rxfail 0 a" and rxfail_dump[3] == "rxfail 3 a",
          f"CLI: 'rxfail' печатает 18 каналов, стик — AUTO ({rxfail_dump[0]!r})")
    check(cli_over_msp(c454, "rxfail 5").strip() == "rxfail 5 h",
          "CLI: AUX-канал по умолчанию HOLD ('rxfail 5 h')")
    check(cli_over_msp(c454, "rxfail 3 s 1000").strip() == "rxfail 3 s 1000",
          "CLI: 'rxfail 3 s 1000' задаёт значение (сетка 25 мкс)")
    check(cli_over_msp(c454, "rxfail 3").strip() == "rxfail 3 s 1000",
          "CLI: значение канала читается обратно")
    check("INVALID ARGUMENT COUNT" in cli_over_msp(c454, "rxfail 5 s"),
          "CLI: 'rxfail 5 s' без значения — INVALID ARGUMENT COUNT, как cliRxFailsafe()")
    check("PARSING FAILED" in cli_over_msp(c454, "rxfail 5 a"),
          "CLI: у AUX-каналов нет AUTO ('rxfail 5 a' → PARSING FAILED)")

    # 4) rxrange и adjrange
    check(cli_over_msp(c454, "rxrange 0 1100 1900").strip() == "rxrange 0 1100 1900",
          "CLI: 'rxrange 0 1100 1900' правит диапазон канала")
    check(cli_over_msp(c454, "rxrange").strip().splitlines()[0] == "rxrange 0 1100 1900",
          "CLI: 'rxrange' печатает 4 строки, первая — правленая")
    check(cli_over_msp(c454, "rxrange reset").strip() == ""
          and cli_over_msp(c454, "rxrange").strip().splitlines()[0] == "rxrange 0 1000 2000",
          "CLI: 'rxrange reset' возвращает заводские 1000…2000")
    check("range min/max NOT BETWEEN 750 AND 2250" in cli_over_msp(c454, "rxrange 0 500 2000"),
          "CLI: вне границ rx.h (750…2250) — ошибка диапазона, как в cliRxRange()")
    adj = cli_over_msp(c454, "adjrange 9 0 2 1400 1600 12 3").strip()
    check(adj == "adjrange 9 0 2 1400 1600 12 3 0 0",
          f"CLI: 'adjrange' пишет слот и печатает 'adjrange 9 0 2 1400 1600 12 3 0 0' ({adj!r})")
    adj_dump = cli_over_msp(c454, "adjrange").strip().splitlines()
    check(len(adj_dump) == ADJUSTMENT_RANGE_COUNT and adj_dump[9].startswith("adjrange 9 0 2 1400 1600 12"),
          f"CLI: 'adjrange' печатает все {ADJUSTMENT_RANGE_COUNT} слотов")

    # 5) моторы, микшер и DShot
    check(cli_over_msp(c454, "motor 0 1500").strip().splitlines() == ["Using output 0.", "motor 0: 1500"],
          "CLI: 'motor 0 1500' печатает 'Using output 0.' и значение")
    motor_now = [u16(c454.request(104), i * 2) for i in range(4)]
    check(motor_now[0] == 1500 and motor_now[1] == 1000,
          f"CLI: выставленный мотор виден во вкладке «Моторы» (MSP_MOTOR: {motor_now})")
    check("VALUE NOT BETWEEN 1000 AND 2000" in cli_over_msp(c454, "motor 0 3000"),
          "CLI: выход за 1000…2000 — ошибка, как cliMotor()")
    cli_over_msp(c454, "motor 255 1000")
    check(cli_over_msp(c454, "mmix 0 1 -1 1 -1").strip() == "mmix 0 1.000 -1.000 1.000 -1.000",
          "CLI: 'mmix' задаёт свой микшер (ftoa из printMotorMix: три знака после запятой)")
    check(cli_over_msp(c454, "mmix reset").strip() == "" and cli_over_msp(c454, "mmix").strip() == "",
          "CLI: 'mmix reset' очищает таблицу, пустая таблица ничего не печатает")
    check("Command Sent: 1" in cli_over_msp(c454, "dshotprog 0 1 2"),
          "CLI: 'dshotprog' отправляет команды ESC и печатает 'Command Sent: N'")
    dshot_info = cli_over_msp(c454, "dshot_telemetry_info")
    check("Dshot reads: 0" in dshot_info and "Motor    Type   eRPM" in dshot_info,
          "CLI: 'dshot_telemetry_info' печатает статистику и шапку таблицы моторов")

    # 6) пищалка и DShot-бикон: формат processBeeperCommand()
    # Заводские значения — по pg/beeper.c 4.5.4: beeper_off_flags = 0 (поля нет в
    # reset-шаблоне), dshotBeaconOffFlags = DSHOT_BEACON_ALLOWED_MODES (RX_LOST и RX_SET).
    check(cli_over_msp(c454, "beeper").strip() == "Disabled:  none",
          "CLI: 'beeper' по умолчанию — 'Disabled:  none' (beeper_off_flags = 0)")
    check(cli_over_msp(c454, "beacon").strip() == "Disabled:  RX_LOST  RX_SET",
          "CLI: 'beacon' по умолчанию — RX_LOST и RX_SET выключены (DEFAULT_DSHOT_BEACON_OFF_FLAGS)")
    check(cli_over_msp(c454, "beacon -RX_LOST").strip() == "Disabled RX_LOST",
          "CLI: 'beacon -RX_LOST' выключает пищалку по потере связи")
    check(cli_over_msp(c454, "beacon GYRO_CALIBRATED").strip().endswith("INVALID NAME: GYRO_CALIBRATED"),
          "CLI: 'beacon' принимает только RX_LOST/RX_SET — остальные имена не проходят")
    cli_over_msp(c454, "beacon RX_SET")          # включили RX_SET — остался только RX_LOST
    beacon_flags = int.from_bytes(c454.request(184)[5:9], "little")
    check(beacon_flags == 1 << 1,
          f"CLI: флаги DShot-пищалки доехали до MSP_BEEPER_CONFIG (0x{beacon_flags:08x})")
    check("Playing sound 0: GYRO_CALIBRATED" in cli_over_msp(c454, "play_sound 0"),
          "CLI: 'play_sound 0' печатает 'Playing sound 0: GYRO_CALIBRATED'")

    # 6б) `vtx` — cliVtx(): 10 слотов переключения каналов по AUX
    vtx_out = cli_over_msp(c454, "vtx").strip().splitlines()
    check(len(vtx_out) == 10 and vtx_out[0] == "vtx 0 0 0 0 0 900 900"
          and vtx_out[9] == "vtx 9 0 0 0 0 900 900",
          "CLI: 'vtx' без аргументов — 10 нулевых слотов, как printVtx()")
    check(cli_over_msp(c454, "vtx 0 4 1 2 1 1500 1700").strip() == "vtx 0 4 1 2 1 1500 1700",
          "CLI: 'vtx <слот> <aux> <band> <channel> <power> <нач> <кон>' пишет и печатает слот")
    check(cli_over_msp(c454, "vtx 1 4").strip().endswith("INVALID ARGUMENT COUNT"),
          "CLI: меньше шести валидных аргументов — слот обнулён и INVALID ARGUMENT COUNT")
    check(cli_over_msp(c454, "vtx 12 0 0 0 0 900 900").strip().endswith("INDEX NOT BETWEEN 0 AND 9"),
          "CLI: индекс вне 0…9 — ошибка диапазона индекса")
    check("vtx 0 4 1 2 1 1500 1700" in cli_over_msp(c454, "diff all"),
          "CLI: 'diff all' показывает настроенный слот `vtx`")

    # 7) плата, подпись, накопители
    check(cli_over_msp(c454, "board_name").strip() == "board_name VIRTUALFC"
          and "CANNOT BE CHANGED" in cli_over_msp(c454, "board_name OTHER"),
          "CLI: 'board_name' читает имя и не даёт переименовать плату")
    check(cli_over_msp(c454, "manufacturer_id").strip() == "manufacturer_id BFR",
          "CLI: 'manufacturer_id' печатает 'manufacturer_id BFR'")
    signature_line = cli_over_msp(c454, "signature").strip().split()
    check(len(signature_line) == 2 and signature_line[0] == "signature" and len(signature_line[1]) == 64,
          "CLI: 'signature' печатает 64 hex-символа (2 × SIGNATURE_LENGTH)")
    check("INVALID LENGTH" in cli_over_msp(c454, "signature 00ff"),
          "CLI: короткая подпись — 'INVALID LENGTH: 4 (EXPECTED: 64)'")
    check(cli_over_msp(c454, "sd_info").strip() == "SD card: Not configured",
          "CLI: 'sd_info' отвечает 'SD card: Not configured' (SDCARD_MODE_NONE)")
    check(cli_over_msp(c454, "flash_scan").strip().splitlines() == ["Verifying", "Success"],
          "CLI: 'flash_scan' печатает 'Verifying' и 'Success'")
    check(cli_over_msp(c454, "flash_write 64 hello").strip() == "Wrote 6 bytes at 64.",
          "CLI: 'flash_write' пишет текст (счётчик включает ведущий пробел, как strchr в cli.c)")
    check("hello" in cli_over_msp(c454, "flash_read 64 8"),
          "CLI: 'flash_read' читает записанное обратно")

    # 8) сглаживание, упрощённая настройка и эмуляторные ограничения
    smoothing = cli_over_msp(c454, "rc_smoothing_info")
    check("# RC Smoothing Type: FILTER" in smoothing and "(auto)" in smoothing
          and "Detected Rx frequency:" in smoothing,
          "CLI: 'rc_smoothing_info' печатает тип сглаживания и срезы с пометкой (auto)")
    check(cli_over_msp(c454, "simplified_tuning disable").strip() == "Disabled simplified tuning."
          and "OFF" in cli_over_msp(c454, "get simplified_pids_mode"),
          "CLI: 'simplified_tuning disable' снимает ползунки (simplified_pids_mode = OFF)")
    check(cli_over_msp(c454, "simplified_tuning apply").strip() == "Applied simplified tuning.",
          "CLI: 'simplified_tuning apply' пересчитывает профили")
    check("NOT SUPPORTED IN THIS EMULATOR" in cli_over_msp(c454, "gyroregisters")
          and "NOT SUPPORTED IN THIS EMULATOR" in cli_over_msp(c454, "serialpassthrough 1 57600")
          and "Error starting ESC connection" in cli_over_msp(c454, "escprog sk 0"),
          "CLI: команды без железа отвечают ошибкой эмулятора, а не UNKNOWN COMMAND")

    # 9) help — как cliHelp(): имя, « - » описание, аргументы второй строкой
    help_out = cli_over_msp(c454, "help profile")
    check("profile - change profile" in help_out and "\t[<index>]" in help_out,
          "CLI: 'help profile' печатает описание и аргументы, как cliHelp()")
    check("NO MATCHES FOR" in cli_over_msp(c454, "help нетакойкоманды"),
          "CLI: 'help <несуществующее>' отвечает 'NO MATCHES FOR'")

    # 9б) resource — как cliResource() 4.5.4: раньше на голое «resource» эмулятор отвечал
    # «INVALID ARGUMENT COUNT», а по прошивке он должен печатать таблицу ресурсов.
    res_dump = cli_over_msp(c454, "resource")
    check("resource MOTOR 1 A00" in res_dump and "resource BEEPER 1 C13" in res_dump
          and "resource SERIAL_RX 1 A10" in res_dump,
          "CLI: 'resource' печатает назначенные ресурсы строками 'resource <имя> <индекс> <пин>'")
    check("NONE" not in res_dump and "resource MOTOR 5" not in res_dump,
          "CLI: 'resource' скрывает свободные слоты — printResource(HIDE_UNUSED)")
    res_show = cli_over_msp(c454, "resource show")
    check("Currently active IO resource assignments:" in res_show and "(reboot to update)" in res_show
          and "--------------------" in res_show,
          "CLI: 'resource show' печатает заголовок и 20 тире, как cliResource()")
    check("A00: MOTOR 1" in res_show and "C13: BEEPER" in res_show and "A10: SERIAL_RX 1" in res_show,
          "CLI: 'resource show' печатает «пин: владелец [индекс]»")
    res_all = cli_over_msp(c454, "resource show all")
    check("Currently active Timers:" in res_all and "Currently active DMA:" in res_all
          and "TIM2:" in res_all and "    CH1 : MOTOR 1" in res_all,
          "CLI: 'resource show all' добавляет таблицы таймеров и DMA (showTimers()/showDma())")
    check("DMA2 Stream 5: SERIAL_RX 1" in res_all and "DMA1 Stream 7: FREE" in res_all,
          "CLI: DMA показывает владельцев потоков, свободные потоки — FREE")
    check("timer A00 AF1" in cli_over_msp(c454, "timer")
          and "dma ADC 1 0" in cli_over_msp(c454, "dma")
          and "# ADC 1: DMA2 Stream 4 Channel 0" in cli_over_msp(c454, "dma"),
          "CLI: 'timer' и 'dma' печатают дампы прошивки (задание + комментарий с потоком)")
    check("Resource is set to B10" in cli_over_msp(c454, "resource LED 1 B10")
          and "resource LED 1 B10" in cli_over_msp(c454, "resource"),
          "CLI: 'resource LED 1 B10' назначает пин, он сразу виден в таблице")
    check("Resource is freed" in cli_over_msp(c454, "resource LED 1 none")
          and "resource LED 1" not in cli_over_msp(c454, "resource"),
          "CLI: 'resource LED 1 none' освобождает пин ('Resource is freed')")
    check("NOTE: B07 already assigned to I2C_SDA 1." in cli_over_msp(c454, "resource LED 1 B07"),
          "CLI: назначение занятого пина печатает NOTE — как resourceCheck()")
    cli_over_msp(c454, "resource LED 1 C14")
    duplex = cli_over_msp(c454, "resource SERIAL_TX 2 A09")
    check("NOTE: A09 already assigned to SERIAL_TX 1. SERIAL_TX 1 disabled." in duplex,
          "CLI: дубль у того же владельца освобождается — «SERIAL_TX 1 disabled»")
    check("NOTE: A09 already assigned to SERIAL_TX 2. SERIAL_TX 2 disabled."
          in cli_over_msp(c454, "resource SERIAL_TX 1 A09"),
          "CLI: возврат пина обратно освобождает второй слот (resourceCheck() по кругу)")
    cli_over_msp(c454, "resource SERIAL_TX 2 D05")
    res_back = cli_over_msp(c454, "resource")
    check("resource SERIAL_TX 1 A09" in res_back and "resource SERIAL_TX 2 D05" in res_back,
          "CLI: после возврата оба слота SERIAL_TX снова на своих пинах")
    check("INVALID RESOURCE NAME: 'NOSUCH'" in cli_over_msp(c454, "resource NOSUCH 1 A00"),
          "CLI: неизвестное имя ресурса — 'INVALID RESOURCE NAME: …'")
    led_strip = cli_over_msp(c454, "resource LED_STRIP 1 B06")
    dump_after = cli_over_msp(c454, "resource")
    check("NOTE: B06 already assigned to I2C_SCL 1." in led_strip
          and "resource LED_STRIP 1 B06" in dump_after and "resource LED 1 C14" in dump_after,
          "CLI: 'LED_STRIP' правит свой слот, не задевая 'LED' (совпадение по полному имени)")
    cli_over_msp(c454, "resource LED_STRIP 1 B08")   # вернуть пин LED_STRIP на место
    check("INDEX NOT BETWEEN 1 AND 8" in cli_over_msp(c454, "resource MOTOR 9 A00"),
          "CLI: индекс вне 1…8 — 'INDEX NOT BETWEEN 1 AND 8'")
    check("PARSING FAILED" in cli_over_msp(c454, "resource MOTOR 1 Z99")
          and "PARSING FAILED" in cli_over_msp(c454, "resource MOTOR 1"),
          "CLI: несуществующий пин или пропущенный аргумент — 'PARSING FAILED'")
    check("# resources" not in cli_over_msp(c454, "diff"),
          "CLI: 'diff' без изменений ресурсов не печатает секцию resources")
    cli_over_msp(c454, "resource LED 1 B10")     # отклонение от заводских пинов
    diff_out = cli_over_msp(c454, "diff")
    check("# resources" in diff_out and "resource LED 1 B10" in diff_out
          and "resource LED 1 C14" not in diff_out,
          "CLI: 'diff' печатает изменённый ресурс, заводской пин в дамп не попадает")
    cli_over_msp(c454, "resource LED 1 C14")     # вернуть заводской пин
    check("Resource is set to C13" in cli_over_msp(c454, "resource BEEPER C13"),
          "CLI: у одиночного ресурса индекс можно не писать (atoi('C13') = 0), как в прошивке")

    # 9в) timer / dma — чтение заданий и новые секции в `dump` (printTimer/printDmaopt)
    check("timer A00 AF1" in cli_over_msp(c454, "timer")
          and "# pin A00: TIM2 CH1 (AF1)" in cli_over_msp(c454, "timer"),
          "CLI: 'timer' печатает задание и расшифровку канала — printTimerDetails()")
    check("# AF1: TIM2 CH1" in cli_over_msp(c454, "timer A00 list")
          and "# timer A00: no change: AF1" in cli_over_msp(c454, "timer A00 AF1"),
          "CLI: 'timer <пин> list' и 'timer <пин> AF<n>' отвечают как cliTimer()")
    check("INVALID ALTERNATE FUNCTION FOR A00: 'AF7'" in cli_over_msp(c454, "timer A00 AF7")
          and "NOT IMPLEMENTED YET" in cli_over_msp(c454, "timer list")
          and "timer C13 NONE" in cli_over_msp(c454, "timer C13"),
          "CLI: у пина один таймер — чужой AF отвергается; 'timer list' — NOT IMPLEMENTED YET")
    dma_out = cli_over_msp(c454, "dma")
    check("dma UART_TX 1 0" in dma_out and "# UART_TX 1: DMA2 Stream 7 Channel 4" in dma_out
          and "dma SPI_TX 1 0" in dma_out and "NONE" not in dma_out,
          "CLI: 'dma' печатает задания прошивки парами строк и скрывает пустые слоты")
    check("dma ADC 2 NONE" in cli_over_msp(c454, "dma ADC 2")
          and "BAD DEVICE: BADDEV" in cli_over_msp(c454, "dma BADDEV 1 0")
          and "BAD INDEX: '9'" in cli_over_msp(c454, "dma ADC 9")
          and "dma pin A00 NONE" in cli_over_msp(c454, "dma pin A00"),
          "CLI: 'dma <устр> <инд>' читает задание, мусор — BAD DEVICE / BAD INDEX")
    dump_out = cli_over_msp(c454, "dump")
    check("# resources" in dump_out and "resource MOTOR 5 NONE" in dump_out
          and "# timer" in dump_out and "# pin B08: TIM4 CH3 (AF2)" in dump_out
          and "# dma" in dump_out and "dma UART_RX 3 NONE" in dump_out
          and "dma pin A00 NONE" in dump_out,
          "CLI: 'dump' содержит секции resources/timer/dma — как printConfig() 4.5.4")

    cli_over_msp(c454, "resource LED 1 B10")     # назначение должно попасть во «флеш»
    c454.request(250)                            # MSP_EEPROM_WRITE — «Сохранить»
    c454.request(68, b"\x00")                    # MSP_SET_REBOOT
    time.sleep(0.8)
    c454.close()
    c454 = None
    for _ in range(40):
        try:
            c454 = Client(port=cli454_port, timeout=1.0)
            break
        except OSError:
            time.sleep(0.25)
    check(c454 is not None, "эмулятор снова принимает подключения после перезагрузки")
    check("resource LED 1 B10" in cli_over_msp(c454, "resource"),
          "назначение пина пережило Save и перезагрузку — ресурсы живут во «флеше»")
    cli_over_msp(c454, "resource LED 1 C14")

    # 10) стирание «флеша» и режим ЗУ — последними: они меняют состояние эмулятора
    erase_out = cli_over_msp(c454, "flash_erase")
    check("Erasing, please wait ... " in erase_out and "Done." in erase_out,
          "CLI: 'flash_erase' стирает накопитель и печатает 'Erasing…' и 'Done.'")
    msc_out = cli_over_msp(c454, "msc")
    check("Restarting in mass storage mode" in msc_out and "Rebooting" in msc_out,
          "CLI: 'msc' печатает '# Restarting in mass storage mode' и 'Rebooting'")
    c454.close()
    proc454.terminate()
    proc454.wait(timeout=10)

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
    import tx as tx20

    # --- калибровка осей: поиск оси по отклонению стика (без железа) ---
    check(tx20.best_axis([0.0, 0.0, 0.0, 0.0], [0.1, 0.9, 0.0, 0.0], set()) == (1, False),
          "калибровка: отклонённый вправо стик находит свою ось (ось 1, без зеркалирования)")
    check(tx20.best_axis([0.0, 0.0, 0.0, 0.0], [-0.95, 0.0, 0.0, 0.0], set()) == (0, True),
          "калибровка: перевёрнутая ось помечается зеркальной (газ/рыск вверх-вправо = 2000)")
    check(tx20.best_axis([0.0, 0.0], [0.05, -0.06], set()) is None
          and tx20.best_axis([0.0, 0.0, 0.0], [0.9, 0.0, 0.0], {0}) is None,
          "калибровка: слабые движения и уже занятые оси не путаются со стиком")
    check(tx20.invert_names("pitch, yaw") == ["pitch", "yaw"], "зеркальные каналы разбираются из строки")
    check(tx20.layout_warnings({"roll": 0, "pitch": 1, "throttle": 2, "yaw": 5},
                               [0.0, 0.0, 0.0, -1.0, -1.0, 0.0], ("X", "Y", "Z", "R", "U", "V")) == [],
          "газ с трещоткой в центре — это норма, предупреждения нет (та самая ложная тревога)")
    check(any("Рыск" in note and "ЦЕНТРЕ" in note for note in
              tx20.layout_warnings({"roll": 0, "pitch": 1, "throttle": 2, "yaw": 3},
                                   [0.0, 0.0, 0.1, -1.0], ("X", "Y", "Z", "R"))),
          "стик крена/тангажа/рыска на упоре — вот это настоящий признак неверной раскладки")
    check(any("два канала" in note for note in
              tx20.layout_warnings({"roll": 0, "pitch": 0, "throttle": 2, "yaw": 3},
                                   [0.0, 0.0, 0.0, -1.0], ("X", "Y", "Z", "R"))),
          "если двум каналам отдана одна ось — инструмент говорит и об этом")
    axes20 = tx20.describe_axes([0.01, -0.02, 0.0, -1.0], {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 3},
                                ("X", "Y", "Z", "R"))
    check(any("упор внизу/влево" in line for line in axes20)
          and any("→ Газ" in line for line in axes20)
          and not any("газ" in line.lower() and "[!]" in line for line in axes20),
          "запуск инструмента печатает, где покоится каждая ось (про газ — без придирок)")
    check("Газ — ось 3 (R)" in tx20.sticks_table({"roll": 0, "pitch": 1, "throttle": 3, "yaw": 5},
                                                 [0.0, 0.0, 0.0, -1.0], (), ("X", "Y", "Z", "R")),
          f"в консоль печатается расшифровка осей: {tx20.sticks_table({'throttle': 3}, [-1.0], (), ('R',))}")

    # --- полная калибровка на «виртуальном пульте»: раскладка не AETR, газ перевёрнут ---
    class ScriptedRadio:
        """Пульт, который «отклоняет» стики по очереди: новый шаг виден по паузе в опросе."""
        AXIS_NAMES = ("X", "Y", "Z", "R", "U", "V")
        # порядок шагов инструмента: крен, тангаж, рыск, газ (газ — «прокачкой» вверх-вниз)
        PLAN = ((0, +1.0), (1, +1.0), (5, +1.0), (3, -1.0))

        def __init__(self):
            self.step, self.last = 0, 0.0
            self.step_since = time.time() + 1.0               # стики стоят, пока идёт «замер покоя»
            self.ticks = 0

        def read(self, device):
            now = time.time()
            if self.last and now - self.last > 0.3:           # пауза между шагами калибровки
                self.step = min(self.step + 1, len(self.PLAN) - 1)
                self.step_since, self.ticks = now, 0
            self.last = now
            axes = [0.0] * 6
            axis, sign = self.PLAN[self.step]
            if now - self.step_since > 0.35:                  # человек реагирует не мгновенно
                if self.step == 3:                            # газ: прокачиваем вверх-вниз, кончаем внизу
                    self.ticks += 1
                    axes[axis] = -1.0 if self.ticks < 20 and self.ticks % 8 < 4 else (+1.0 if self.ticks < 20 else -1.0)
                else:
                    axes[axis] = sign
            return axes, [], []

    radio20 = ScriptedRadio()
    real_input = builtins.input
    try:
        builtins.input = lambda *a, **k: ""
        mapping20, invert20 = tx20.detect_mapping(radio20, {"name": "виртуальный пульт"})
        check(mapping20 == {"roll": 0, "pitch": 1, "throttle": 3, "yaw": 5}
              and invert20 == ["throttle"],
              f"калибровка --detect сама находит раскладку не-AETR: {mapping20}, "
              f"зеркалятся {invert20} (человек только отклоняет стики, кнопок жать не надо)")
        saved20 = json.loads(Path(tx20.MAP_FILE).read_text(encoding="utf-8"))
        check(saved20["map"] == {"roll": 0, "pitch": 1, "throttle": 3, "yaw": 5}
              and saved20["invert"] == ["throttle"],
              "найденная раскладка сохранена в tx_map.json (её подхватит следующий запуск)")
        built20 = tx20.build_payload(mapping20, [0.0, 0.0, 0.0, -1.0, 0.0, 1.0], [], [])
        mirrored20 = tx20._invert_values([built20.get(n, 1500) for n in tx20.NAMES], "throttle")
        check(built20["yaw"] == 2000 and built20["aux1"] == 1500
              and mirrored20[tx20.NAMES.index("throttle")] == 2000,
              "с найденной раскладкой каждый стик идёт в свой канал, а не в AUX: "
              f"рыск {built20['yaw']}, свободная ось в AUX1 {built20['aux1']}, "
              f"газ после зеркалирования {mirrored20[tx20.NAMES.index('throttle')]}")
    finally:
        builtins.input = real_input
        Path(tx20.MAP_FILE).unlink(missing_ok=True)

    # незаконченная калибровка не должна сохраняться и портить каналы
    partial = Path(tx20.MAP_FILE)
    backup = partial.read_text(encoding="utf-8") if partial.exists() else None
    try:
        partial.write_text('{"map": {"roll": 0, "pitch": 1}, "invert": []}', encoding="utf-8")
        mapping20, invert20 = tx20.parse_map("auto")
        check(mapping20 == dict(tx20.DEFAULT_MAP) and invert20 == [],
              "неполная сохранённая раскладка игнорируется (берётся AETR, а не «половина» карты)")
    finally:
        if backup is None:
            partial.unlink(missing_ok=True)
        else:
            partial.write_text(backup, encoding="utf-8")

    check(tx20.ControlRefused.__name__ == "ControlRefused",
          "в инструменте есть отдельная реакция на «данные отклонены» (HTTP 409)")

    # --- «каналы «прыгают» из-за второго окна инструмента»: замок и сверка с FC ---
    lock_path = Path(tx20.LOCK_FILE)
    lock_backup = lock_path.read_text(encoding="utf-8") if lock_path.exists() else None
    try:
        lock_path.unlink(missing_ok=True)
        tx20.lock_take()
        check(tx20.lock_read().get("alive") and tx20.lock_read().get("pid") == os.getpid(),
              "инструмент отмечает себя в tx_lock.json как ведущий каналы")
        stopped = None
        try:
            tx20.lock_take()
        except SystemExit as exc:
            stopped = str(exc)
        dead_lock = json.loads(lock_path.read_text(encoding="utf-8"))
        dead_lock.update({"pid": 999999, "heartbeat": time.time()})
        lock_path.write_text(json.dumps(dead_lock), encoding="utf-8")
        check(not tx20.lock_read()["alive"],
              "если прежнее окно закрыто (процесса нет), его «замок» не мешает новому запуску")
        lock_path.write_text(json.dumps({"pid": os.getpid(), "heartbeat": time.time()}), encoding="utf-8")
        check(tx20.lock_read()["alive"], "живой процесс с недавним откликом считается хозяином каналов")
        check(stopped is not None and "другой запуск" in stopped,
              "второе окно инструмента останавливается с подсказкой, а не воюет за каналы")
        tx20.lock_beat()
        tx20.lock_release()
        check(not lock_path.exists(), "после выхода инструмент освобождает tx_lock.json")
    finally:
        if lock_backup is None:
            lock_path.unlink(missing_ok=True)
        else:
            lock_path.write_text(lock_backup, encoding="utf-8")

    sent20 = {"roll": 1500, "pitch": 1500, "yaw": 1500, "throttle": 1000, "aux1": 1000, "aux2": 1000}
    foreign20 = {"channels": [1500, 1500, 1000, 1000, 999, 1500] + [1000] * 10}
    check(tx20.compare_with_fc(sent20, foreign20) == [("Рыск", 1500, 1000), ("AUX2", 1000, 1500)],
          "сверка с FC видит, что каналы переписал кто-то ещё (тот самый «рыск в AUX2»)")
    check(tx20.compare_with_fc(sent20, {"channels": [1500, 1500, 1500, 1000, 1001, 1000] + [1000] * 10}) == [],
          "когда в FC лежит ровно посланное — расхождений нет (у AUX 1 допуск на ±2 мкс)")
    check("[PID " in tx20.my_source("Zorro"),
          f"источник подписан PID: {tx20.my_source('Zorro')}")

    import bfemu as _bfemu
    check(tx20.TOOL_BUILD == _bfemu.__build__,
          f"инструмент печатает ту же дату сборки, что эмулятор ({tx20.TOOL_BUILD}) — "
          "по ней сразу видно старое окно в другом терминале")
    check("сборка" in tx20.build_note() and tx20.TOOL_BUILD in tx20.build_note(),
          f"при запуске инструмент печатает свою сборку: {tx20.build_note()}")

    note_foreign = tx20.foreign_note({"source": "USB-пульт: Zorro [PID 30072]", "updated_ago": 0.01})
    check("другой источник" in note_foreign and "PID 30072" in note_foreign
          and "Ctrl+C" in note_foreign,
          "инструмент предупреждает, если каналы только что писал другой источник (старое окно)")
    check(tx20.foreign_note({"source": "USB-пульт: Zorro [PID 30072]", "updated_ago": 30.0}) == ""
          and tx20.foreign_note({}) == "",
          "если чужой источник давно молчит (или его не было) — предупреждения нет")

    # --- «закрой чужие окна»: находим PID по подписи и даём готовую команду ---
    two = ["USB-пульт: Драйвер PC-джойстика Microsoft [PID 20376]",
           "USB-пульт: Драйвер PC-джойстика Microsoft [PID 28888]"]
    check(tx20.pid_of_source(two[1]) == 28888 and tx20.pid_of_source("виртуальная аппаратура") is None,
          "номер процесса читается из подписи источника (и отсутствует там, где его нет)")
    check(tx20.other_tool_pids({"sources": two}, own=20376) == [28888],
          "чужой запуск инструмента находится по PID (наш собственный не трогаем)")
    check(tx20.kill_command(28888).endswith("28888") and
          ("taskkill" in tx20.kill_command(28888)) == (os.name == "nt"),
          f"готовая команда закрытия для этой системы: {tx20.kill_command(28888)}")
    mark = tx20.multi_source_note(two, own=20376)
    check("это окно" in mark and "ЧУЖОЕ окно" in mark and "28888" in mark,
          "тревога помечает, какое окно наше, а какое чужое — его и надо закрыть")
    check("close_other_tools" in inspect.getsource(tx20.joystick),
          "ключ --kill-others действительно закрывает чужие окна инструмента")

    note_multi = tx20.multi_source_note(["USB-пульт: Zorro [PID 111]", "USB-пульт: Zorro [PID 222]"])
    check("несколько источников" in note_multi and "Ctrl+C" in note_multi
          and "рыск уводит AUX" in note_multi and "сборка" in note_multi,
          "текст тревоги «пишут несколько источников» объясняет, что делать (Ctrl+C в лишнем окне)")
    check(tx20.multi_source_note(["USB-пульт: Zorro [PID 111]"]) == ""
          and tx20.multi_source_note([]) == "",
          "один источник тревоги не вызывает (обычная передача управления)")
    note_mis = tx20.mismatch_note([("Рыск", 1500, 1000), ("AUX2", 1000, 1500)],
                                  "USB-пульт: Zorro [PID 4411]", 0.2)
    check("Рыск: послали 1500, в FC 1000" in note_mis and "AUX2: послали 1000, в FC 1500" in note_mis
          and "PID 4411" in note_mis,
          "текст тревоги «отдаёт не то, что посылаем» называет каналы и текущий источник")
    check(tx20.mismatch_note([]) == "", "когда всё сходится, тревоги нет")

    # --- калибровка AUX: тумблеры и крутилки ---
    check([tx20.aux_value({"type": "switch2", "levels": [-1.0, 1.0]}, v)
           for v in (-1.0, -0.6, 0.6, 1.0)] == [1000, 1000, 2000, 2000],
          "тумблер на 2 положения даёт 1000 или 2000 (щелчок вниз/вверх)")
    check([tx20.aux_value({"type": "switch3", "levels": [-1.0, 0.0, 1.0]}, v)
           for v in (-1.0, -0.3, 0.0, 0.3, 1.0)] == [1000, 1500, 1500, 1500, 2000],
          "тумблер на 3 положения проходит 1000 / 1500 / 2000 (середина — 1500)")
    check([tx20.aux_value({"type": "knob", "min": -1.0, "max": 1.0}, v)
           for v in (-1.0, -0.5, 0.0, 0.5, 1.0)] == [1000, 1250, 1500, 1750, 2000],
          "крутилка идёт линейно от 1000 до 2000")
    check(tx20.aux_value({"type": "switch2", "levels": [0.0, 1.0]}, 0.9) == 2000
          and tx20.aux_value({"type": "knob", "min": 0.0, "max": 1.0}, 0.0) == 1000,
          "размах у разных пультов учитывается (положения берутся измеренные, а не -1…1)")

    check(tx20.pick_aux_channel("switch2", []) == 1
          and tx20.pick_aux_channel("switch3", [1]) == 2
          and tx20.pick_aux_channel("knob", [1, 2]) == 3,
          "каналы выдаются по порядку: первый тумблер → AUX1 (CH5), второй → AUX2 (CH6)…")
    parsed_aux = tx20.parse_aux_map("3=switch2,4=switch3:4,5=knob")
    check([(item["axis"], item["type"], item["channel"]) for item in parsed_aux]
          == [(3, "switch2", 1), (4, "switch3", 4), (5, "knob", 2)],
          "ручная раскладка --aux-map разбирается (номер канала можно задать: 4=switch3:4)")

    # классификация: тумблер, трёхпозиционный тумблер и крутилка
    check(tx20.classify_aux_sample([1.0] * 20 + [-1.0] * 20, rest=-1.0)[0] == "switch2"
          and tx20.classify_aux_sample([-1.0] * 15 + [0.0] * 15 + [1.0] * 15, rest=-1.0)[0] == "switch3"
          and tx20.classify_aux_sample([i / 40 for i in range(-40, 40)], rest=-1.0)[0] == "knob",
          "по поведению оси видно, что это: тумблер на 2 положения, на 3 положения или крутилка")

    # разнос раскладки AUX по файлу
    aux_backup = Path(tx20.MAP_FILE).read_text(encoding="utf-8") if Path(tx20.MAP_FILE).exists() else None
    try:
        aux_two = [{"axis": 3, "type": "switch2", "channel": 1, "levels": [-1.0, 1.0]},
                   {"axis": 4, "type": "knob", "channel": 2, "min": -1.0, "max": 1.0}]
        tx20.save_map({"roll": 0, "pitch": 1, "yaw": 5, "throttle": 2}, ["throttle"], "Zorro",
                      aux=aux_two)
        check(tx20.load_aux() == aux_two,
              "раскладка AUX сохраняется в tx_map.json и переживает перезапуск инструмента")
        saved20b = json.loads(Path(tx20.MAP_FILE).read_text(encoding="utf-8"))
        check(saved20b["map"] == {"roll": 0, "pitch": 1, "yaw": 5, "throttle": 2}
              and saved20b["aux"] == aux_two,
              "раскладка стиков и AUX живут в одном файле (калибровки не мешают друг другу)")
        payload_aux = tx20.build_payload({"roll": 0, "pitch": 1, "yaw": 5, "throttle": 2},
                                         [0.0, 0.0, -1.0, 1.0, 0.0, 0.0], [], [], aux_map=aux_two)
        check(payload_aux.get("aux1") == 2000 and payload_aux.get("aux2") == 1500,
              f"каналы AUX ведут тумблер и крутилка: AUX1={payload_aux.get('aux1')} (щёлкнут вверх), "
              f"AUX2={payload_aux.get('aux2')} (крутилка в середине)")
        check(payload_aux.get("roll") == 1500 and payload_aux.get("throttle") == 1000
              and len([k for k in payload_aux if k.startswith("aux")]) == 2,
              "стики при этом на своих каналах, лишних AUX не появляется")
    finally:
        if aux_backup is None:
            Path(tx20.MAP_FILE).unlink(missing_ok=True)
        else:
            Path(tx20.MAP_FILE).write_text(aux_backup, encoding="utf-8")

    # живая калибровка AUX на «пульте» с двумя тумблерами
    class ScriptedAux:
        AXIS_NAMES = ("X", "Y", "Z", "R", "U", "V")

        def __init__(self):
            self.t0 = None

        def read(self, device):
            if self.t0 is None:
                self.t0 = time.time()
            t = time.time() - self.t0
            axes = [0.0, 0.0, -1.0, -1.0, -1.0, 0.0]
            if 0.9 <= t < 2.2:                     # тумблер на 2 положения
                axes[3] = 1.0 if int((t - 0.9) / 0.5) % 2 == 0 else -1.0
            if t >= 2.4:                           # тумблер на 3 положения
                step = int((t - 2.4) / 0.6)
                axes[4] = [-1.0, 0.0, 1.0, 0.0][min(step, 3)]
            return axes, [], []

    found_aux = tx20.detect_aux(ScriptedAux(), {"name": "тестовый пульт"},
                                {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 5},
                                watch=1.3, idle=1.5, quiet=True)
    check([(item["axis"], item["type"], item["channel"]) for item in found_aux]
          == [(3, "switch2", 1), (4, "switch3", 2)],
          f"калибровка AUX сама находит оба тумблера и их тип: {tx20.format_aux(found_aux)}")
    check("тумблер (2 положения)" in tx20.format_aux(found_aux)
          and "ось 3" in tx20.format_aux(found_aux),
          "в раскладке видно и тип, и ось (по этим строкам страница /tx показывает, что откуда)")

    class SilentAux:
        AXIS_NAMES = ("X", "Y", "Z", "R", "U", "V")

        def read(self, device):
            return [0.0, 0.0, -1.0, -1.0, -1.0, 0.0], [], []

    silent = tx20.detect_aux(SilentAux(), {"name": "молчащий пульт"},
                             {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 5},
                             watch=0.6, idle=0.7, quiet=True)
    notes_silent = " ".join(tx20.silent_axes_note([0.0, 0.0, -1.0, -1.0, -1.0, 0.0], [], {0, 1, 2, 5},
                                                  [], ("X", "Y", "Z", "R", "U", "V")))
    check(silent == [] and "MIXES" in notes_silent and "ось 3" in notes_silent,
          "если тумблеры не шевелятся, инструмент прямо говорит про микшер пульта (CH5…CH8)")


    sbus_frame = tx20.build_sbus_frame([988, 1500, 2012, 1000, 2000] + [1500] * 11)
    check(len(sbus_frame) == 25 and sbus_frame[0] == 0x0F and sbus_frame[-1] == 0x00,
          f"кадр SBUS: {len(sbus_frame)} байт, заголовок 0x0F, хвост 0x00")
    decoded_sbus = tx20.decode_sbus_frame(sbus_frame)
    check(decoded_sbus is not None and decoded_sbus[0][:5] == [988, 1500, 2012, 1000, 2000],
          f"SBUS-кодек: кадр -> каналы {decoded_sbus[0][:5] if decoded_sbus else '—'} (172..1811 -> 988..2012 мкс)")
    check(tx20.decode_sbus_frame(sbus_frame[:-1] + b"\x01") is None,
          "битый кадр SBUS отбрасывается (проверяется хвост 0x00)")

    decoder_sbus = tx20.WireDecoder("auto")
    check(decoder_sbus.feed(b"\x55" + tx20.build_sbus_frame([1500] * 16, failsafe=True))[0]["failsafe"] is True,
          "кадр SBUS с флагом failsafe -> «потеря связи» (аппаратуру выключили)")
    decoder_crsf = tx20.WireDecoder("auto")
    events_crsf = decoder_crsf.feed(b"\x00" + tx20.build_crsf_channels_frame([1100, 1500, 1900, 1200] + [1500] * 12))
    check(events_crsf and events_crsf[0]["protocol"] == "crsf" and decoder_crsf.protocol == "crsf",
          f"разбор потока из COM-порта: мусор + кадр CRSF -> {events_crsf[0]['channels'][:4] if events_crsf else '—'}")
    decoder_sbus2 = tx20.WireDecoder("auto")
    events_sbus = decoder_sbus2.feed(b"\x11\x22" + tx20.build_sbus_frame([1000, 1600, 1400, 1100] + [1000] * 12) + b"\x33")
    check(events_sbus and events_sbus[0]["protocol"] == "sbus" and decoder_sbus2.protocol == "sbus",
          f"тот же разбор определяет SBUS: каналы {events_sbus[0]['channels'][:4] if events_sbus else '—'}")

    # живой пульт: два способа чтения, встроенный режим Windows работает без библиотек
    joystick_help = subprocess.run([sys.executable, str(root / "tools" / "tx.py"), "joystick", "--help"],
                                   cwd=root, env=env, capture_output=True, text=True, timeout=60)
    check("--map" in joystick_help.stdout and "--invert" in joystick_help.stdout
          and "--backend" not in joystick_help.stdout,
          "режим joystick: ключи --map/--invert есть, выбора «чем читать» нет — способ один, "
          "встроенный (в Windows)")
    no_radio = subprocess.run([sys.executable, str(root / "tools" / "tx.py"), "axes"],
                              cwd=root, env=env, capture_output=True, text=True, timeout=60)
    no_radio_text = no_radio.stdout + no_radio.stderr
    check(no_radio.returncode != 0 and "Windows" in no_radio_text
          and "USB Joystick" in no_radio_text and "pip install" not in no_radio_text,
          "если пульт не читается, подсказка объясняет причину и не предлагает ничего ставить")

    # pygame в поставке быть не должно: сборка из исходников на новых Python «зависает»
    leftovers = []
    for path in list((root / "tools").glob("*.py")) + list((root / "bfemu").glob("*.py")) \
            + [root / "README.md", root / "WINDOWS.md"] + list((root / "docs").glob("*.md")) \
            + list(root.glob("*.bat")):
        text = path.read_text(encoding="utf-8", errors="replace")
        if "pygame" in text:
            leftovers.append(path.name)
    check(not leftovers,
          f"pygame убран из поставки целиком: упоминаний нет ({leftovers if leftovers else 'чисто'})")
    # .bat-файлы на Windows должны запускаться двойным щелчком: только ASCII и переводы строк CRLF
    bat_bad = []
    for path in sorted(root.glob("*.bat")):
        data = path.read_bytes()
        why = []
        if any(b > 127 for b in data):
            why.append("есть кириллица")
        if data.count(b"\n") != data.count(b"\r\n"):
            why.append("есть строки без CR")
        if why:
            bat_bad.append(f"{path.name}: {', '.join(why)}")
    check(not bat_bad,
          f"командные файлы .bat: только ASCII и CRLF ({bat_bad if bat_bad else 'все чистые'})")

    check(tx20.winmm_normalize(0, 0, 65535) == -1.0
          and abs(tx20.winmm_normalize(32767, 0, 65535)) < 0.0001
          and tx20.winmm_normalize(65535, 0, 65535) == 1.0,
          "встроенный режим: ось Windows 0…65535 -> -1…1, центр 32767 -> 0 (стик по центру — 1500 мкс)")
    built = tx20.build_payload({"roll": 0, "pitch": 1, "throttle": 2, "yaw": 3},
                               [-0.5, 0.0, -1.0, 1.0, 0.0, 1.0], [True, False], [(1, 0)])
    check(built["roll"] == 1250 and built["pitch"] == 1500 and built["throttle"] == 1000
          and built["yaw"] == 2000,
          f"каналы из показаний пульта: {dict(list(built.items())[:4])} (стики по --map)")
    check(built["aux1"] == 1500 and built["aux2"] == 2000
          and built["aux3"] == 2000 and built["aux4"] == 1000,
          f"свободные оси и кнопки уходят в AUX (нажата — 2000): aux1={built['aux1']}, aux3={built['aux3']}")
    check(built["aux5"] == 2000 and built["aux6"] == 1000 and built["aux6"] != 1500,
          f"«шляпа» (POV) уходит в AUX: aux5={built['aux5']} (вправо), aux6={built['aux6']} (центр — выкл)")

    selftest_run = subprocess.run([sys.executable, str(root / "tools" / "tx.py"), "selftest"],
                                  cwd=root, env=env, capture_output=True, text=True, timeout=120)
    check(selftest_run.returncode == 0 and "ВСЕ САМОПРОВЕРКИ ПРОЙДЕНЫ" in selftest_run.stdout,
          "python tools/tx.py selftest (CRSF + SBUS + разбор потока) — зелёный")

    bind_cli = subprocess.run([sys.executable, str(root / "tools" / "tx.py"), "bind",
                               "--http", str(live_http), "--bind-seconds", "0"],
                              cwd=root, env=env, capture_output=True, text=True, timeout=120)
    check(bind_cli.returncode == 0 and "связана" in bind_cli.stdout,
          "python tools/tx.py bind — «связать» из консоли работает")
    no_port = subprocess.run([sys.executable, str(root / "tools" / "tx.py"), "wire"],
                             cwd=root, env=env, capture_output=True, text=True, timeout=120)
    check(no_port.returncode != 0 and "укажите порт" in (no_port.stdout + no_port.stderr),
          "python tools/tx.py wire без --port: понятная подсказка, а не трассировка")

    # бинд — состояние «в эфире», а не конфигурация: во «флеш» не пишется, после ребута снова «связана»
    live_post({"bind": False})
    c20.request(250)
    c20.request(68, b"\x00")
    time.sleep(0.8)
    c20.close()
    proc20.terminate()
    proc20.wait(timeout=10)
    check("rx_bind_state" not in live_eeprom.read_text(encoding="utf-8"),
          "биндинг не попал в «флеш» (это состояние эфира, как и положения стиков)")
    proc20b, c20b = start_live()
    live_state = _json20.loads(urllib.request.urlopen(f"http://127.0.0.1:{live_http}/api/rc", timeout=4).read())
    check(live_state["bind"]["state"] == "bound" and live_state["channels"][:4] == [1500, 1500, 1500, 1000],
          "после перезагрузки эмулятора аппаратура снова «связана», стики в центре")
    c20b.close()
    proc20b.terminate()
    proc20b.wait(timeout=10)

    # ---------------------------------------------------------- вкладка «OSD»
    print("\n12. Вкладка «OSD»: элементы, статистика, таймеры, предупреждения")

    def decode_osd(payload: bytes) -> dict:
        """Разбор ровно так, как это делает Configurator (osd.js: OSD.msp.decode)."""
        view, pos = payload, 0
        out = {"flags": view[pos]}
        pos += 1
        if out["flags"] and len(view) > 1:
            out["video_system"] = view[pos]
            pos += 1
            out["units"] = view[pos]
            pos += 1
            out["rssi"] = view[pos]
            pos += 1
            out["cap"] = u16(view, pos)
            pos += 2
            pos += 1                                   # устаревшее поле таймера
            count = view[pos]
            pos += 1
            out["alt"] = u16(view, pos)
            pos += 2
            out["items"] = [u16(view, pos + 2 * i) for i in range(count)]
            pos += 2 * count
            stats_count = view[pos]
            pos += 1
            out["stats"] = [view[pos + i] == 1 for i in range(stats_count)]
            pos += stats_count
            timers_count = view[pos]
            pos += 1
            out["timers"] = [u16(view, pos + 2 * i) for i in range(timers_count)]
            pos += 2 * timers_count
            pos += 2                                   # младшее слово предупреждений
            warnings_count = view[pos]
            pos += 1
            out["warnings_flags"] = u32(view, pos)
            pos += 4
            out["warnings"] = [bool(out["warnings_flags"] & (1 << i)) for i in range(warnings_count)]
            out["profiles"] = view[pos]
            out["profile_selected"] = view[pos + 1] - 1
            pos += 2
            out["overlay"] = view[pos]
            pos += 1
            out["camera_frame"] = (view[pos], view[pos + 1])
            pos += 2
            out["link_quality"] = u16(view, pos)
            pos += 2
        out["read"], out["total"] = pos, len(view)
        return out

    proc8, c8 = start_emulator()
    osd = decode_osd(c8.request(84))
    check(osd["flags"] != 0,
          f"OSD: флаги непустые (0x{osd['flags']:02x}) — с нулевыми флагами вкладка была пустой")
    check(osd["flags"] & 0x31 == 0x31,
          "OSD: функция включена (бит 0), аппаратный MAX7456 (бит 4) и устройство найдено (бит 5)")
    check(osd["read"] == osd["total"],
          f"OSD: ответ разобран до конца ({osd['read']} из {osd['total']} байт)")
    check(len(osd["items"]) == OSD_ITEM_COUNT and len(osd["stats"]) == OSD_STAT_COUNT
          and len(osd["timers"]) == OSD_TIMER_COUNT and len(osd["warnings"]) == OSD_WARNING_COUNT,
          f"OSD: счётчики как в прошивке 4.5.4 — элементов {len(osd['items'])}, "
          f"статистики {len(osd['stats'])}, таймеров {len(osd['timers'])}, "
          f"предупреждений {len(osd['warnings'])}")
    check(osd["profiles"] == OSD_PROFILE_COUNT and osd["profile_selected"] == 0,
          f"OSD: профилей {osd['profiles']}, выбран первый")
    check(osd["units"] == 1 and osd["rssi"] == 20 and osd["cap"] == 2200 and osd["alt"] == 100,
          "OSD: заводские значения — метрическая система, RSSI 20 %, ёмкость 2200, высота 100")
    check(len([value for value in osd["items"] if value & OSD_VISIBLE]) >= 4,
          "OSD: несколько элементов видно — превью вкладки заполнено")
    check(osd["camera_frame"] == (24, 11) and osd["link_quality"] == 80,
          "OSD: кадр камеры 24×11 и порог качества связи 80 % (API 1.46)")
    canvas = c8.request(189)
    check(canvas[:2] == b"\x1e\x10", f"OSD: полотно {canvas[0]}×{canvas[1]} (30×16, как SD-видео)")

    moved = 6 | (3 << 5) | OSD_VISIBLE
    c8.request(85, bytes([15]) + moved.to_bytes(2, "little"))
    flags = 0x41FFF
    c8.request(85, bytes([0xFF, 2, 2, 25]) + (2500).to_bytes(2, "little")
               + (0).to_bytes(2, "little") + (120).to_bytes(2, "little")
               + (flags & 0xFFFF).to_bytes(2, "little") + flags.to_bytes(4, "little")
               + bytes([2, 2, 24, 11]) + (70).to_bytes(2, "little"))
    c8.request(85, bytes([0xFE, 1]) + (300).to_bytes(2, "little"))
    c8.request(85, bytes([5]) + (0).to_bytes(2, "little") + bytes([0]))
    after = decode_osd(c8.request(84))
    check(after["items"][15] == moved, "OSD: перетаскивание элемента сохраняется в FC")
    check(after["units"] == 2 and after["cap"] == 2500 and after["link_quality"] == 70
          and after["profile_selected"] == 1,
          "OSD: общие настройки (единицы, тревоги, профиль, LQ) применяются")
    check(after["timers"][1] == 300 and after["stats"][5] is False,
          "OSD: таймер и галочки статистики записываются")
    c8.request(250)                                    # «Сохранить» → MSP_EEPROM_WRITE
    c8.close()
    time.sleep(0.5)
    proc8.terminate()
    proc8.wait(timeout=10)
    proc8, c8 = start_emulator()
    kept = decode_osd(c8.request(84))
    check(kept["items"][15] == moved and kept["cap"] == 2500 and kept["timers"][1] == 300,
          "OSD: настройки пережили перезапуск эмулятора (лежат во «флеше»)")
    c8.close()
    proc8.terminate()
    proc8.wait(timeout=10)

    # ------------------------------------- вкладка «Корректирование в полёте»
    print("\n13. Вкладка «Корректирование в полёте»: слоты, диапазоны, запись и сохранение")

    def decode_adjustments(payload: bytes) -> list:
        """Разбор ровно как MSPHelper.ts: байта-счётчика нет, слотов = длина ответа / 6."""
        out, pos = [], 0
        for _ in range(len(payload) // 6):
            out.append({
                "aux": payload[pos + 1],
                "start": 900 + payload[pos + 2] * 25,
                "end": 900 + payload[pos + 3] * 25,
                "function": payload[pos + 4],
                "switch": payload[pos + 5],
            })
            pos += 6
        return out

    # Секция идёт после CLI-команды 'defaults' из предыдущих проверок: она стирает таблицу
    # корректировок (так и должно быть — заводская пустая). Для проверки «вкладка не пустая»
    # берём чистый эмулятор без «флеша»: он стартует как настроенный квад.
    eeprom.unlink(missing_ok=True)
    proc9, c9 = start_emulator()
    payload = c9.request(52)                          # MSP_ADJUSTMENT_RANGES
    ranges = decode_adjustments(payload)
    check(len(payload) == ADJUSTMENT_RANGE_COUNT * 6 and len(ranges) == ADJUSTMENT_RANGE_COUNT,
          f"Корректировки: ответ 52 — {len(ranges)} слотов × 6 байт (ровно то, что разбирает вкладка)")
    check(len(c9.request(116)) > 20 and len(c9.request(119)) >= 8,
          "Корректировки: MSP_BOXNAMES/MSP_BOXIDS приходят — таблица режимов не пустая")
    active = [r for r in ranges if r["start"] != r["end"]]
    check(len(active) >= 2,
          f"Корректировки: включённых слотов {len(active)} из {len(ranges)} — вкладка не пустая")
    check(active[0]["aux"] == 0 and active[0]["start"] == 1300 and active[0]["end"] == 1700
          and active[0]["function"] == 1,
          "Корректировки: слот #1 — AUX1 1300…1700, функция RC Rate")
    check(any(r["function"] == 12 for r in active) and any(r["function"] == 24 for r in active),
          "Корректировки: есть слоты «выбор профиля рейтов» (12) и «сила Horizon» (24)")
    check(all(900 <= r["start"] <= 2100 and 900 <= r["end"] <= 2100 for r in ranges)
          and all(r["function"] <= ADJUSTMENT_FUNCTION_COUNT for r in ranges),
          "Корректировки: значения слотов в пределах 900…2100 и допустимых функций")

    # запись ровно теми кадрами, что шлёт вкладка (MSPHelper.sendAdjustmentRanges)
    c9.request(53, bytes([4, 0, 6, 26, 38, 1, 6]))     # слот #5: AUX7 1550…1850 → RC Rate, через AUX7
    c9.request(53, bytes([0, 0, 0, 0, 0, 0, 0]))       # слот #1 выключаем (900…900)
    after = decode_adjustments(c9.request(52))
    check(after[4]["aux"] == 6 and after[4]["start"] == 1550 and after[4]["end"] == 1850
          and after[4]["function"] == 1,
          "Корректировки: новая настройка записана и видна в ответе (слот #5 = AUX7 1550…1850)")
    check(after[0]["start"] == after[0]["end"],
          "Корректировки: выключенный слот отдаётся как 900…900 — вкладка снимет галочку")
    check(len(after) == ADJUSTMENT_RANGE_COUNT,
          "Корректировки: число слотов после записи не поехало")
    c9.request(250)                                    # «Сохранить» → MSP_EEPROM_WRITE
    c9.close()
    time.sleep(0.5)
    proc9.terminate()
    proc9.wait(timeout=10)
    proc9, c9 = start_emulator()
    kept = decode_adjustments(c9.request(52))
    check(kept[4]["aux"] == 6 and kept[4]["start"] == 1550 and kept[0]["start"] == kept[0]["end"],
          "Корректировки: слоты пережили перезапуск эмулятора (лежат во «флеше»)")
    check(len(kept) == ADJUSTMENT_RANGE_COUNT,
          "Корректировки: после перезапуска вкладка снова получает полный ответ")
    c9.close()
    proc9.terminate()
    proc9.wait(timeout=10)

    # ------------------------------------------------------- вкладка «Чёрный ящик»
    print("\n14. Вкладка «Чёрный ящик»: накопитель, чтение лога, настройки записи")

    def bb_summary(payload: bytes) -> dict:
        """Разбор MSP_DATAFLASH_SUMMARY ровно как MSPHelper.ts."""
        flags = payload[0]
        return {
            "ready": bool(flags & 1), "supported": bool(flags & 2),
            "sectors": u32(payload, 1), "size": u32(payload, 5), "used": u32(payload, 9),
        }

    def bb_pull(client, used: int) -> bytes:
        """Цикл из useDataflashPull.js: блоки по 4096 байт до конца накопителя."""
        out, address = bytearray(), 0
        while address < used:
            reply = client.request(71, address.to_bytes(4, "little")
                                   + (4096).to_bytes(2, "little") + b"\x01")
            if len(reply) < 7:
                break
            assert u32(reply, 0) == address, "ответ не про запрошенный адрес"
            assert reply[6] == 0, "вкладка не умеет сжатие"
            chunk = reply[7:7 + u16(reply, 4)]
            if not chunk:
                break
            out.extend(chunk)
            address += len(chunk)
        return bytes(out)

    eeprom.unlink(missing_ok=True)
    monitor = socket.socket()
    monitor.bind(("127.0.0.1", 0))
    bb_http = monitor.getsockname()[1]
    monitor.close()
    proc10, c10 = start_emulator(http=bb_http)
    summary = bb_summary(c10.request(70))
    check(summary["supported"] and summary["ready"],
          "Чёрный ящик: MSP 70 отдаёт supported+ready — вкладка видит накопитель")
    check(summary["sectors"] == 4096 and summary["size"] == 16 * 1024 * 1024,
          f"Чёрный ящик: {summary['sectors']} секторов, {summary['size']} Б (W25Q128 16 МиБ)")
    check(summary["used"] > 0,
          f"Чёрный ящик: занято {summary['used']} Б — эмулятор наполнил его тестовыми пролётами")
    blob = bb_pull(c10, summary["used"])
    marker = b"H Product:Blackbox flight data recorder"
    check(len(blob) == summary["used"], f"Чёрный ящик: лог выкачан целиком ({len(blob)} Б)")
    starts = [i for i in range(len(blob)) if blob.startswith(marker, i)]
    check(len(starts) == 6, f"Чёрный ящик: в накопителе {len(starts)} логов разных типов полёта")
    text = blob[:starts[1] if len(starts) > 1 else len(blob)].decode("latin-1", "replace")
    check("P interval:2" in text and "looptime:1000" in text,
          "Чёрный ящик: заголовок лога как у прошивки (P interval 2 → 500 Гц)")
    check(len(text.split("Field I name:")[1].splitlines()[0].split(",")) == 38,
          "Чёрный ящик: в логе 38 основных полей (Field I name)")

    config = c10.request(80)                                   # MSP_BLACKBOX_CONFIG
    check(len(config) == 11 and config[0] & 1,
          f"Чёрный ящик: MSP 80 — {len(config)} байт, чёрный ящик поддерживается")
    check(config[1] == 1, f"Чёрный ящик: устройство записи {config[1]} (1 = SPI flash)")
    check(u16(config, 4) * (1 << config[6]) == 32 and config[3] == 1 << config[6],
          f"Чёрный ящик: rateDenom {config[3]} и P ratio {u16(config, 4)} согласованы с I interval 32")
    # Частоту записи не меняем: смена запускает пересборку «флеша» в фоне (это отдельная
    # проверка в tools/check_blackbox.py), а нам здесь важны принятие записи и сохранение.
    c10.request(81, bytes([1, 1, 2]) + (2).to_bytes(2, "little") + bytes([1]) + (0b101).to_bytes(4, "little"))
    after = c10.request(80)
    check(after[6] == 1 and u32(after, 7) == 0b101,
          "Чёрный ящик: MSP_SET_BLACKBOX_CONFIG принят вкладкой (частота и маска полей)")
    cli_text = cli_over_msp(c10, "blackbox")
    check("Blackbox (dataflash)" in cli_text and "used" in cli_text,
          "Чёрный ящик: CLI 'blackbox' показывает сводку по накопителю")
    check("motor_fault" in cli_text and "arm_resonance" in cli_text,
          "Чёрный ящик: в сводке есть отказ двигателя и резонанс луча")
    c10.request(72)                                            # MSP_DATAFLASH_ERASE
    check(bb_summary(c10.request(70))["used"] == 0,
          "Чёрный ящик: MSP 72 стирает накопитель (вкладка «Стереть»)")
    cli_over_msp(c10, "blackbox refill")
    refilled = 0
    for _ in range(40):                                        # наполнение идёт в фоне
        time.sleep(1.0)
        refilled = bb_summary(c10.request(70))["used"]
        if refilled > 0:
            break
    check(refilled > 0, f"Чёрный ящик: CLI 'blackbox refill' наполняет накопитель заново ({refilled} Б)")

    # Кнопка «Активировать режим запоминающего устройства»: перезагрузка в режим ЗУ (MSP 68).
    # Настоящий борт после этого молчит по MSP, а «флеш» виден файлами — у эмулятора их
    # отдаёт страница /msc: по файлу на пролёт плюс общий дамп.
    import urllib.request as _url
    reply = c10.request(68, bytes([2]))
    check(reply[:1] == b"\x02" and len(reply) >= 2 and reply[1] == 1,
          f"Чёрный ящик: кнопка «режим ЗУ» принята (режим {reply[0] if reply else '—'}, "
          f"готовность {reply[1] if len(reply) > 1 else '—'})")
    time.sleep(1.0)
    silent = False
    try:
        probe = Client(port=port, timeout=1.0)
        try:
            probe.request(70)
        except (TimeoutError, OSError):
            silent = True
        probe.close()
    except OSError:
        silent = True
    check(silent, "Чёрный ящик: в режиме ЗУ MSP молчит — как настоящий борт")
    page = _url.urlopen(f"http://127.0.0.1:{bb_http}/msc", timeout=5).read().decode()
    check(all(f"LOG{n:05d}.BBL" in page for n in range(1, 7)) and "BLACKBOX_ALL.BBL" in page,
          "Чёрный ящик: страница режима ЗУ отдаёт по файлу на пролёт (LOG00001…06.BBL)")
    first = _url.urlopen(f"http://127.0.0.1:{bb_http}/msc/LOG00004.BBL", timeout=10).read()
    check(first.startswith(marker) and len(first) > 1000,
          f"Чёрный ящик: отдельный файл пролёта скачивается ({len(first)} Б)")
    _url.urlopen(f"http://127.0.0.1:{bb_http}/msc/exit", timeout=5).read()
    time.sleep(1.0)
    c10 = Client(port=port, timeout=3.0)
    check(bb_summary(c10.request(70))["used"] == refilled,
          "Чёрный ящик: после выхода из режима ЗУ MSP отвечает снова, данные на месте")
    c10.request(81, bytes([1, 1, 2]) + (2).to_bytes(2, "little") + bytes([1]) + (0).to_bytes(4, "little"))
    c10.close()
    proc10.terminate()
    proc10.wait(timeout=10)
    server.stop()
    print()
    if failures:
        print(f"ПРОВАЛЕНО тестов: {len(failures)}")
        for f in failures:
            print("  -", f)
        return 1
    print("ВСЕ ТЕСТЫ ПРОЙДЕНЫ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
