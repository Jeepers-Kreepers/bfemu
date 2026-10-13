#!/usr/bin/env python3
"""Проверка вкладки «Питание и батарея»: настройки, живые данные, калибровка, сохранение.

Скрипт повторяет то, что делает вкладка Configurator 2026.6.2 (PowerTab.vue / usePower.js):

  loadData():        MSP_STATUS_EX, MSP_VOLTAGE_METERS(128), MSP_CURRENT_METERS(129),
                     MSP_CURRENT_METER_CONFIG(40), MSP_VOLTAGE_METER_CONFIG(56),
                     MSP_BATTERY_STATE(130), MSP_BATTERY_CONFIG(32);
  поллинг:           128 / 129 / 130 раз в секунду;
  Save:              MSP_SET_BATTERY_CONFIG(33) -> MSP_SET_VOLTAGE_METER_CONFIG(57)
                     на каждый конфиг -> MSP_SET_CURRENT_METER_CONFIG(41) -> MSP_EEPROM_WRITE(250);
  «Сохранить и перезагрузить»: то же + MSP_SET_REBOOT(68).

Проверяется: значения из прошивки (3.30/4.30/3.50 В на банку, ёмкость 1500 мАч),
живые данные (16.80 В, 4 банки, ток электроники в покое, ток при тесте моторов),
«виртуальный полёт» (ток, расход мА·ч и просадка меняются сами; разряженная батарея
садится и меняется), калибровка, сохранение всего этого в «флеш» и полный перезапуск
процесса эмулятора, а также CLI-имена.

Запуск:  python3 tools/check_power_tab.py      (Windows: python tools\\check_power_tab.py)
"""
from __future__ import annotations

import re
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

PORT = 16061
FAILS: list[str] = []

VOLTAGE_METER_ID_BATTERY_1 = 10
CURRENT_METER_ID_BATTERY_1 = 10
CURRENT_METER_ID_VIRTUAL_1 = 80
SOURCE_NAMES = {0: "NONE", 1: "ADC", 2: "ESC"}
CURRENT_SOURCE_NAMES = {0: "NONE", 1: "ADC", 2: "VIRTUAL", 3: "ESC", 4: "MSP"}
STATE_NAMES = {0: "OK", 1: "ПРЕДУПРЕЖДЕНИЕ", 2: "КРИТИЧНО", 3: "БАТАРЕЯ НЕ ПОДКЛЮЧЕНА"}


def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5 : i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:
            size = buf[i + 6] | (buf[i + 7] << 8)
            if i + 8 + size <= len(buf):
                return buf[i + 8 : i + 8 + size]
    return None


class Fc:
    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.sock.settimeout(0.4)

    def call(self, code: int, payload: bytes = b"", v2: bool = False) -> bytes:
        self.sock.sendall(encode_v2(code, payload) if v2 else encode_v1(code, payload))
        buf, deadline = b"", time.time() + 2.5
        while time.time() < deadline:
            try:
                buf += self.sock.recv(65536)
            except socket.timeout:
                pass
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.03)
        return b""

    def cli(self, command: str, window: float = 2.5) -> str:
        out, offset, deadline = b"", 0, time.time() + window
        while time.time() < deadline:
            payload = command.encode() if offset == 0 else command.encode() + b"\x00" + offset.to_bytes(2, "little")
            resp = self.call(0x3012, payload, v2=True)
            if len(resp) < 3:
                break
            out += resp[3:]
            offset += len(resp) - 3
            if offset >= int.from_bytes(resp[0:2], "little") or len(resp) == 3:
                break
        return out.decode("latin-1", "replace").replace("\r\n", "\n")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def start(port: int, eeprom: Path):
    proc = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    for _ in range(60):
        try:
            return proc, Fc(port)
        except OSError:
            time.sleep(0.25)
    raise RuntimeError("эмулятор не поднялся")


def stop(proc):
    proc.terminate()
    proc.wait(timeout=10)


def check(ok: bool, what: str):
    print(f"   {'[OK ]' if ok else '[НЕТ]'} {what}")
    if not ok:
        FAILS.append(what)


def u16(b, off=0):
    return int.from_bytes(b[off : off + 2], "little")


def i16(b, off=0):
    return int.from_bytes(b[off : off + 2], "little", signed=True)


def read_battery_config(fc: Fc) -> dict:
    """Разбор ответа так, как это делает MSPHelper.js (vbat* — уже в вольтах)."""
    data = fc.call(32)
    return {
        "raw_len": len(data),
        "vbatmincellvoltage": u16(data, 7) / 100 if len(data) >= 13 else u16(data, 0) / 10,
        "vbatmaxcellvoltage": u16(data, 9) / 100 if len(data) >= 13 else u16(data, 1) / 10,
        "vbatwarningcellvoltage": u16(data, 11) / 100 if len(data) >= 13 else u16(data, 2) / 10,
        "capacity": u16(data, 3),
        "voltageMeterSource": data[5] if len(data) > 5 else 0,
        "currentMeterSource": data[6] if len(data) > 6 else 0,
    }


def read_state(fc: Fc) -> dict:
    data = fc.call(130)
    return {
        "cellCount": data[0],
        "capacity": u16(data, 1),
        "voltage": u16(data, 9) / 100,
        "mAhDrawn": u16(data, 4),
        "amperage": i16(data, 6) / 100,
        "batteryState": data[8],
    }


def read_voltage_config(fc: Fc) -> dict:
    data = fc.call(56)
    if len(data) < 7 or data[1] != 5:
        return {}
    return {
        "id": data[2], "sensorType": data[3], "vbatscale": data[4],
        "vbatresdivval": data[5], "vbatresdivmultiplier": data[6],
    }


def read_current_configs(fc: Fc) -> list:
    data = fc.call(40)
    count = data[0]
    out, pos = [], 1
    for _ in range(count):
        length = data[pos]
        if length != 6:
            pos += 1 + length
            continue
        out.append({"id": data[pos + 1], "sensorType": data[pos + 2],
                    "scale": i16(data, pos + 3), "offset": i16(data, pos + 5)})
        pos += 1 + length
    return out


def main() -> int:
    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    proc, fc = start(PORT, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «ПИТАНИЕ И БАТАРЕЯ»: что эмулятор отдаёт")
    print("=" * 78)
    config = read_battery_config(fc)
    state = read_state(fc)
    voltage_cfg = read_voltage_config(fc)
    current_cfgs = read_current_configs(fc)
    voltage_meters = fc.call(128)
    current_meters = fc.call(129)

    print(f"   Напряжение на банку: мин {config['vbatmincellvoltage']:.2f} В, "
          f"макс {config['vbatmaxcellvoltage']:.2f} В, предупреждение {config['vbatwarningcellvoltage']:.2f} В")
    print(f"   Ёмкость (mAh): {config['capacity']}")
    print(f"   Датчики: напряжения {SOURCE_NAMES.get(config['voltageMeterSource'], '?')}, "
          f"тока {CURRENT_SOURCE_NAMES.get(config['currentMeterSource'], '?')}")
    print(f"   Калибровка напряжения: id {voltage_cfg.get('id')}, масштаб {voltage_cfg.get('vbatscale')}, "
          f"делитель {voltage_cfg.get('vbatresdivval')}, множитель {voltage_cfg.get('vbatresdivmultiplier')}")
    for cfg in current_cfgs:
        print(f"   Калибровка тока: id {cfg['id']}, тип {cfg['sensorType']}, "
              f"масштаб {cfg['scale']}, смещение {cfg['offset']}")
    print(f"   Датчики напряжения (128): {list(voltage_meters)}")
    print(f"   Датчики тока (129): {list(current_meters)}")
    print(f"   Состояние: банок {state['cellCount']}, {state['voltage']:.2f} В, {state['mAhDrawn']} мА·ч, "
          f"{state['amperage']:.2f} А, статус {STATE_NAMES.get(state['batteryState'], '?')}")

    check(config["raw_len"] == 13, f"MSP_BATTERY_CONFIG: {config['raw_len']} байт (нужно 13)")
    check(abs(config["vbatmincellvoltage"] - 3.30) < 0.001,
          f"минимальное напряжение на банку: {config['vbatmincellvoltage']:.2f} В (а не 0)")
    check(abs(config["vbatmaxcellvoltage"] - 4.30) < 0.001,
          f"максимальное напряжение на ячейку: {config['vbatmaxcellvoltage']:.2f} В")
    check(abs(config["vbatwarningcellvoltage"] - 3.50) < 0.001,
          f"предупреждение: {config['vbatwarningcellvoltage']:.2f} В")
    check(config["capacity"] == 1500, f"ёмкость: {config['capacity']} мА·ч")
    check(config["voltageMeterSource"] == 1 and config["currentMeterSource"] == 1,
          "датчики напряжения и тока: ADC (как на плате с делителями)")

    check(voltage_cfg.get("id") == VOLTAGE_METER_ID_BATTERY_1 and voltage_cfg.get("sensorType") == 0,
          "MSP_VOLTAGE_METER_CONFIG: подкадр длиной 5, id 10 (метр батареи)")
    check(len(current_cfgs) == 2 and current_cfgs[0]["id"] == CURRENT_METER_ID_BATTERY_1
          and current_cfgs[1]["id"] == CURRENT_METER_ID_VIRTUAL_1,
          "MSP_CURRENT_METER_CONFIG: два подкадра — ADC (id 10) и виртуальный (id 80)")

    check(len(voltage_meters) == 2 and voltage_meters[0] == VOLTAGE_METER_ID_BATTERY_1
          and abs(voltage_meters[1] / 10 - 16.8) < 0.05,
          f"MSP_VOLTAGE_METERS: {voltage_meters[1] / 10:.1f} В (на столе 4S ≈ 16.8 В)")
    check(len(current_meters) == 10, "MSP_CURRENT_METERS: два метра по 5 байт")

    check(state["cellCount"] == 4, f"банок: {state['cellCount']} (напряжение делится на макс. на банку)")
    check(abs(state["voltage"] - 16.8) < 0.05, f"напряжение: {state['voltage']:.2f} В")
    check(0.4 <= state["amperage"] <= 0.7,
          f"в покое ток не ноль: {state['amperage']:.2f} А — электроника борта (FC, приёмник, VTX, LED)")
    check(state["mAhDrawn"] <= 80,
          f"расход считается и в покое: {state['mAhDrawn']} мА·ч за первые секунды")
    check(state["batteryState"] == 0, "статус батареи: OK (напряжение выше предупреждения)")

    print("\n-- Ток появляется только когда крутятся моторы (тест на вкладке «Моторы») --")
    payload = b"".join((1500).to_bytes(2, "little") for _ in range(4)) + b"\x00" * 8
    fc.call(214, payload)
    time.sleep(0.5)
    running = read_state(fc)
    print(f"   4 мотора на 1500: {running['amperage']:.2f} А, {running['mAhDrawn']} мА·ч")
    check(12.0 < running["amperage"] < 15.0,
          f"ток при газе 1500: {running['amperage']:.2f} А (четыре ESC: холостой ток + 12 А на половине газа)")
    check(running["mAhDrawn"] >= 1, "счётчик мА·ч растёт только под нагрузкой")
    time.sleep(1.2)
    holding = read_state(fc)
    check(holding["amperage"] > 10.0 and holding["mAhDrawn"] > running["mAhDrawn"],
          "моторы держат газ без авто-стопа (как на железной плате) — ток и счётчик продолжают расти")
    fc.call(214, b"".join((1000).to_bytes(2, "little") for _ in range(4)) + b"\x00" * 8)
    time.sleep(2.0)
    stopped = read_state(fc)
    check(0.4 <= stopped["amperage"] <= 0.7 and stopped["mAhDrawn"] >= holding["mAhDrawn"],
          f"после «Стоп» моторы выбежали, ток вернулся к току электроники "
          f"({stopped['amperage']:.2f} А), счётчик не откатился")

    print("\n-- Виртуальный полёт: ток, расход и напряжение меняются сами --")
    # То же, что делают кнопка «включить виртуальный полёт» на веб-мониторе и команда CLI.

    def packs_replaced() -> int:
        match = re.search(r"packs replaced: (\d+)", fc.cli("sim_flight status"))
        return int(match.group(1)) if match else 0

    def motor_outputs() -> list:
        payload = fc.call(104)
        return [int.from_bytes(payload[i * 2:i * 2 + 2], "little") for i in range(len(payload) // 2)]

    def esc_telemetry() -> dict:
        payload = fc.call(139)
        base = 1
        return {
            "rpm": int.from_bytes(payload[base:base + 4], "little"),
            "temp_c": payload[base + 6],
            "voltage_v": int.from_bytes(payload[base + 7:base + 9], "little") / 100,
            "current_a": int.from_bytes(payload[base + 9:base + 11], "little") / 100,
            "mah": int.from_bytes(payload[base + 11:base + 13], "little"),
        }

    check("state: ON - flight" in fc.cli("sim_flight on"), "CLI 'sim_flight on' включает полёт")
    time.sleep(2.5)
    flying = read_state(fc)
    outputs = motor_outputs()
    esc = esc_telemetry()
    print(f"   В полёте: {flying['amperage']:.2f} А, {flying['mAhDrawn']} мА·ч, {flying['voltage']:.2f} В; "
          f"моторы {outputs}, ESC: {esc['rpm']} об, {esc['current_a']:.2f} А, {esc['temp_c']} °C")
    check(flying["amperage"] > 3.0, f"ток в полёте: {flying['amperage']:.2f} А (вкладка больше не мёртвая)")
    check(flying["voltage"] < 16.75, f"напряжение просело под нагрузкой: {flying['voltage']:.2f} В")
    check(all(1000 < value < 2000 for value in outputs[:4]),
          f"MSP_MOTOR: газ держит сценарий полёта ({outputs[:4]})")
    check(esc["rpm"] > 1000 and esc["current_a"] > 0.5,
          f"MSP_MOTOR_TELEMETRY: обороты {esc['rpm']}, ток {esc['current_a']:.2f} А — вкладка «Моторы» живая")

    def voltage_peak(window: float = 2.0) -> float:
        """Самое высокое напряжение за окно: момент, когда газ минимален, а значит
        просадка почти нулевая — то есть «напряжение покоя». Так динамику разряда
        видно надёжнее, чем по одному мгновенному замеру."""
        peak = 0.0
        deadline = time.time() + window
        while time.time() < deadline:
            peak = max(peak, read_state(fc)["voltage"])
            time.sleep(0.2)
        return peak

    first = read_state(fc)
    peak_before = voltage_peak()
    time.sleep(6.0)
    second = read_state(fc)
    peak_after = voltage_peak()
    check(second["mAhDrawn"] - first["mAhDrawn"] >= 30,
          f"расход тикает на глазах: +{second['mAhDrawn'] - first['mAhDrawn']} мА·ч за 6 с")
    check(peak_after < peak_before - 0.05,
          f"напряжение падает по мере разряда: {peak_before:.2f} → {peak_after:.2f} В "
          f"(пики покоя, без влияния газа)")

    fc.cli("sim_flight scale 600")     # ускоряем время: батарея «кончится» за секунды
    before_packs = packs_replaced()
    deadline = time.time() + 30
    while time.time() < deadline and packs_replaced() == before_packs:
        time.sleep(0.5)
    after_packs = packs_replaced()
    print(f"   Замен батареи: {after_packs}, расход после замены {read_state(fc)['mAhDrawn']} мА·ч")
    check(after_packs > before_packs,
          f"разряженная батарея меняется сама: посадка и новая батарея (замен {after_packs})")
    check(read_state(fc)["mAhDrawn"] < 1500, "счётчик мА·ч на новой батарее начинается с нуля")

    fc.cli("sim_flight scale 10")
    fc.cli("sim_flight off")
    time.sleep(3.0)
    off_state = read_state(fc)
    check(0.4 <= off_state["amperage"] <= 0.7 and motor_outputs()[:4] == [1000] * 4,
          f"после 'sim_flight off' моторы стоят (1000), ток — только электроника "
          f"({off_state['amperage']:.2f} А)")
    fc.cli("sim_flight reset")
    check(read_state(fc)["mAhDrawn"] <= 2, "CLI 'sim_flight reset' обнуляет счётчик мА·ч")

    print("\n-- Правки во вкладке: правка напряжений, ёмкости, источников и калибровки --")
    # 3.60 / 4.20 / 3.80 В, ёмкость 2200, датчики: напряжение ADC, ток VIRTUAL
    fc.call(33, bytes([36, 42, 38]) + (2200).to_bytes(2, "little") + bytes([1, 2])
            + (360).to_bytes(2, "little") + (420).to_bytes(2, "little") + (380).to_bytes(2, "little"))
    fc.call(57, bytes([VOLTAGE_METER_ID_BATTERY_1, 120, 10, 1]))
    fc.call(41, bytes([CURRENT_METER_ID_VIRTUAL_1]) + (450).to_bytes(2, "little")
            + (5).to_bytes(2, "little", signed=True))

    config = read_battery_config(fc)
    check(abs(config["vbatmincellvoltage"] - 3.60) < 0.001 and config["capacity"] == 2200
          and config["currentMeterSource"] == 2,
          f"прочитано обратно: {config['vbatmincellvoltage']:.2f} В, {config['capacity']} мА·ч, "
          f"датчик тока {CURRENT_SOURCE_NAMES.get(config['currentMeterSource'])}")
    check(read_voltage_config(fc).get("vbatscale") == 120, "масштаб напряжения сохранён (120)")
    virtual = [c for c in read_current_configs(fc) if c["id"] == CURRENT_METER_ID_VIRTUAL_1]
    check(virtual and virtual[0]["scale"] == 450 and virtual[0]["offset"] == 5,
          "масштаб виртуального датчика тока сохранён (450/5)")

    print("\n-- «Сохранить» и «Сохранить и перезагрузить» (как вкладка: SET_* -> EEPROM_WRITE) --")
    fc.call(250)
    fc.call(68, b"\x00")
    time.sleep(1.0)
    fc.close()
    stop(proc)

    print("\n" + "=" * 78)
    print("ПОЛНЫЙ ПЕРЕЗАПУСК ЭМУЛЯТОРА")
    print("=" * 78)
    proc, fc2 = start(PORT, eeprom)
    config = read_battery_config(fc2)
    state = read_state(fc2)
    check(abs(config["vbatmincellvoltage"] - 3.60) < 0.001
          and abs(config["vbatmaxcellvoltage"] - 4.20) < 0.001
          and abs(config["vbatwarningcellvoltage"] - 3.80) < 0.001,
          f"напряжения на банку пережили перезапуск: {config['vbatmincellvoltage']:.2f}/"
          f"{config['vbatmaxcellvoltage']:.2f}/{config['vbatwarningcellvoltage']:.2f} В")
    check(config["capacity"] == 2200, f"ёмкость: {config['capacity']} мА·ч")
    check(config["currentMeterSource"] == 2, "выбранный датчик тока (VIRTUAL) на месте")
    check(read_voltage_config(fc2).get("vbatscale") == 120, "калибровка напряжения на месте")
    check(state["mAhDrawn"] <= 20 and 0.4 <= state["amperage"] <= 0.7,
          f"после перезагрузки счётчик начат с нуля, а борд ест ток электроники "
          f"({state['mAhDrawn']} мА·ч, {state['amperage']:.2f} А)")

    for command, expected in (
        ("get vbat_min_cell_voltage", "vbat_min_cell_voltage = 360"),
        ("get vbat_max_cell_voltage", "vbat_max_cell_voltage = 420"),
        ("get vbat_warning_cell_voltage", "vbat_warning_cell_voltage = 380"),
        ("get bat_capacity", "bat_capacity = 2200"),
        ("get battery_meter", "battery_meter = ADC"),
        ("get current_meter", "current_meter = VIRTUAL"),
        ("get vbat_scale", "vbat_scale = 120"),
        ("get ibatv_scale", "ibatv_scale = 450"),
    ):
        out = fc2.cli(command)
        check(expected in out, f"CLI '{command}' → {'найдено' if expected in out else 'НЕТ: ' + out.strip()[:40]!r}")

    print("\n-- Батарея «отключена»: датчик напряжения переводим в NONE --")
    fc2.cli("set battery_meter = NONE")
    fc2.cli("save")
    time.sleep(0.6)
    fc2.close()
    stop(proc)
    proc, fc3 = start(PORT, eeprom)
    state = read_state(fc3)
    print(f"   Состояние: банок {state['cellCount']}, {state['voltage']:.2f} В, "
          f"статус {STATE_NAMES.get(state['batteryState'], '?')}")
    check(state["cellCount"] == 0 and state["voltage"] == 0.0 and state["batteryState"] == 3,
          "без датчика напряжения вкладка показывает «батарея не подключена», а не выдуманные 16.8 В")
    fc3.close()
    stop(proc)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for item in FAILS:
            print("  -", item)
        return 1
    print("ВКЛАДКА «ПИТАНИЕ И БАТАРЕЯ» РАБОТАЕТ: значения из прошивки, живые данные, всё сохраняется")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
