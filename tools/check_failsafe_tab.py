#!/usr/bin/env python3
"""Проверка вкладки «Failsafe»: диапазон импульсов, «Этап 1», «Этап 2», сохранение.

Скрипт повторяет то, что делает вкладка Configurator 2026.6.2 (FailsafeTab.vue):

  loadConfig(): MSP_RX_CONFIG(44), MSP_FAILSAFE_CONFIG(75), MSP_GPS_RESCUE(135),
                MSP_RXFAIL_CONFIG(77), MSP_FEATURE_CONFIG(37), MSP_BOXNAMES,
                MSP_BOXIDS, MSP_RC(105), MSP_RSSI_CONFIG(50), MSP_MODE_RANGES;
  Save:         MSP_SET_RX_CONFIG(45) -> MSP_SET_FAILSAFE_CONFIG(76) ->
                MSP_SET_RXFAIL_CONFIG(78) на каждый канал ->
                MSP_SET_FEATURE_CONFIG(37) -> MSP_SET_GPS_RESCUE(225) ->
                MSP_EEPROM_WRITE(250) -> MSP_SET_REBOOT(68).

Ожидаемые значения — из прошивки Betaflight 4.5.4:
  rx_min_usec / rx_max_usec              885 / 2115 (target/common_defaults_post.h)
  failsafe_delay 15 (1.5 с), off_delay 10 (1.0 с), throttle 1000,
  switch_mode 0 («Этап 1»), throttle_low_delay 100 (10 с), procedure 1 (DROP) — failsafe.c
  «Этап 1»: AUTO у стиков (у газа 875 мкс), HOLD у AUX — rx.c + сетка 25 мкс
  карта каналов: AETR1234 -> [0, 1, 3, 2, 4, 5, 6, 7] — pg/rx.c
  каналов: 16 (CRSF_MAX_CHANNEL)

Запуск:  python3 tools/check_failsafe_tab.py      (Windows: python tools\\check_failsafe_tab.py)
"""
from __future__ import annotations

import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

PORT = 16071
FAILS: list[str] = []
MODE_NAMES = {0: "АВТО", 1: "УДЕРЖАНИЕ", 2: "ЗНАЧЕНИЕ"}
SWITCH_NAMES = {0: "Этап 1", 1: "KILL", 2: "Этап 2"}
PROCEDURE_NAMES = {0: "ПРИЗЕМЛЕНИЕ", 1: "ПАДЕНИЕ", 2: "GPS RESCUE"}
CHANNEL_NAMES = ["Крен", "Тангаж", "Рыск.", "Газ"] + [f"AUX {i}" for i in range(1, 13)]


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
            time.sleep(0.02)
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


def read_failsafe(fc: Fc) -> dict:
    data = fc.call(75)
    return {"raw": data, "delay": data[0], "offDelay": data[1], "throttle": u16(data, 2),
            "switchMode": data[4], "lowDelay": u16(data, 5), "procedure": data[7]}


def read_rxfail(fc: Fc) -> list:
    data = fc.call(77)
    return [{"mode": data[i * 3], "value": u16(data, i * 3 + 1)} for i in range(len(data) // 3)]


def read_rx_config(fc: Fc) -> dict:
    data = fc.call(44)
    return {"raw": data, "rx_min_usec": u16(data, 8), "rx_max_usec": u16(data, 10)}


def main() -> int:
    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    proc, fc = start(PORT, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «FAILSAFE»: что эмулятор отдаёт (дрон на столе, приёмник CRSF)")
    print("=" * 78)
    rx = read_rx_config(fc)
    fs = read_failsafe(fc)
    rxfail = read_rxfail(fc)
    rc = fc.call(105)
    rx_map = list(fc.call(64))
    rssi = fc.call(50)
    gps_rescue = fc.call(135)

    print(f"   Диапазон импульсов: {rx['rx_min_usec']} … {rx['rx_max_usec']} мкс")
    print(f"   Failsafe тумблер: {SWITCH_NAMES.get(fs['switchMode'], '?')}")
    print(f"   Этап 2: задержка {fs['delay'] / 10:.1f} с, лимит низкого газа {fs['lowDelay'] / 10:.0f} с, "
          f"процедура {PROCEDURE_NAMES.get(fs['procedure'], '?')}, газ при посадке {fs['throttle']}")
    print(f"   Этап 1: каналов {len(rxfail)}")
    for index, channel in enumerate(rxfail[:6]):
        extra = f", значение {channel['value']} мкс" if channel["mode"] == 2 else ""
        print(f"      {CHANNEL_NAMES[index]:8} {MODE_NAMES.get(channel['mode'], '?')}{extra}")
    print(f"      … всего {len(rxfail)} каналов")
    print(f"   Каналы MSP_RC: {len(rc) // 2} | карта: {rx_map} (AETR1234) | RSSI-канал: {rssi[0] if rssi else '—'}")
    print(f"   GPS Rescue (135): {len(gps_rescue)} байт")

    check(len(rx["raw"]) == 38, f"MSP_RX_CONFIG: {len(rx['raw'])} байт (нужно 38)")
    check((rx["rx_min_usec"], rx["rx_max_usec"]) == (885, 2115),
          f"диапазон импульсов: {rx['rx_min_usec']}…{rx['rx_max_usec']} (заводские значения прошивки)")
    check(len(fs["raw"]) == 8, f"MSP_FAILSAFE_CONFIG: {len(fs['raw'])} байт (нужно 8)")
    check(fs["delay"] == 15, f"«Длительность 1го этапа»: {fs['delay'] / 10:.1f} с (в прошивке 1.5 с, а не 1.0)")
    check(fs["offDelay"] == 10, f"время посадки: {fs['offDelay'] / 10:.1f} с (заводское 1.0 с)")
    check(fs["throttle"] == 1000, f"газ при посадке: {fs['throttle']}")
    check(fs["switchMode"] == 0, "тумблер failsafe: «Этап 1»")
    check(fs["lowDelay"] == 100, f"«Лимит длительности низкого уровня газа»: {fs['lowDelay'] / 10:.0f} с")
    check(fs["procedure"] == 1, "процедура: «Падение» (1 = DROP)")

    check(len(rxfail) == 16, f"«Этап 1»: {len(rxfail)} каналов (CRSF отдаёт 16, а не 6)")
    check([c["mode"] for c in rxfail[:4]] == [0, 0, 0, 0],
          "у стиков режим «Автоматически» (0), а не «Удержание» — как pgResetFn в rx.c")
    check(all(c["mode"] == 1 for c in rxfail[4:]), "у AUX-каналов режим «Удержание» (1)")
    check(rxfail[3]["value"] == 875, f"у газа значение {rxfail[3]['value']} мкс (RX_MIN_USEC на сетке 25 мкс)")
    check(rxfail[0]["value"] == 1500, "у стиков значение 1500 мкс (среднее)")

    check(len(rc) == 32, f"MSP_RC: {len(rc)} байт = {len(rc) // 2} каналов (CRSF_MAX_CHANNEL)")
    check(rx_map == [0, 1, 3, 2, 4, 5, 6, 7], f"карта каналов: {rx_map} — заводская AETR1234, 8 байт")
    check(len(fc.call(116)) > 0 and len(fc.call(119)) > 0, "BOXNES/BOXIDS: список режимов приходит")
    check(rssi == b"\x00", f"MSP_RSSI_CONFIG: канал {rssi[0] if rssi else '—'} (0 = выключен)")
    check(len(gps_rescue) == 26, f"MSP_GPS_RESCUE: {len(gps_rescue)} байт")

    # ------------------------------------------------------------------ правки
    print("\n" + "=" * 78)
    print("ПРАВИМ ДИАПАЗОН ИМПУЛЬСОВ, ЭТАП 1 И ЭТАП 2 (как вкладка перед «Сохранить»)")
    print("=" * 78)

    new_rx = bytearray(rx["raw"])
    new_rx[8:10] = (1000).to_bytes(2, "little")
    new_rx[10:12] = (2000).to_bytes(2, "little")
    fc.call(45, bytes(new_rx))
    after = read_rx_config(fc)
    check((after["rx_min_usec"], after["rx_max_usec"]) == (1000, 2000),
          f"диапазон импульсов сохранён: {after['rx_min_usec']}…{after['rx_max_usec']}")

    fc.call(76, bytes([20, 30]) + (1200).to_bytes(2, "little") + bytes([2])
            + (50).to_bytes(2, "little") + bytes([0]))
    fs2 = read_failsafe(fc)
    check((fs2["delay"], fs2["offDelay"], fs2["throttle"], fs2["switchMode"], fs2["lowDelay"], fs2["procedure"])
          == (20, 30, 1200, 2, 50, 0),
          f"«Этап 2»: {fs2['delay'] / 10:.1f} с / посадка {fs2['offDelay'] / 10:.1f} с / газ {fs2['throttle']} / "
          f"{SWITCH_NAMES[fs2['switchMode']]} / {fs2['lowDelay'] / 10:.0f} с / {PROCEDURE_NAMES[fs2['procedure']]}")

    fc.call(78, bytes([4, 2]) + (1750).to_bytes(2, "little"))     # AUX 1: «Значение» 1750
    fc.call(78, bytes([0, 1]) + (1500).to_bytes(2, "little"))     # Крен: «Удержание»
    fc.call(78, bytes([3, 2]) + (1000).to_bytes(2, "little"))     # Газ: «Значение» 1000
    rxfail2 = read_rxfail(fc)
    check((rxfail2[4]["mode"], rxfail2[4]["value"]) == (2, 1750), "AUX 1: «Значение» 1750 мкс")
    check(rxfail2[0]["mode"] == 1, "Крен: «Удержание»")
    check((rxfail2[3]["mode"], rxfail2[3]["value"]) == (2, 1000), "Газ: «Значение» 1000 мкс")
    fc.call(78, bytes([4, 2]) + (1487).to_bytes(2, "little"))     # округление до сетки 25 мкс
    check(read_rxfail(fc)[4]["value"] == 1475,
          f"значение округляется как в прошивке: 1487 -> {read_rxfail(fc)[4]['value']} мкс")

    print("\n-- «Сохранить» + «Сохранить и перезагрузить» (SET_* -> EEPROM_WRITE -> ребут) --")
    fc.call(250)
    fc.call(68, b"\x00")
    time.sleep(1.0)
    fc.close()
    stop(proc)

    print("\n" + "=" * 78)
    print("ПОЛНЫЙ ПЕРЕЗАПУСК ЭМУЛЯТОРА")
    print("=" * 78)
    proc, fc2 = start(PORT, eeprom)
    rx3 = read_rx_config(fc2)
    fs3 = read_failsafe(fc2)
    rxfail3 = read_rxfail(fc2)
    check((rx3["rx_min_usec"], rx3["rx_max_usec"]) == (1000, 2000),
          f"диапазон импульсов пережил перезапуск: {rx3['rx_min_usec']}…{rx3['rx_max_usec']}")
    check((fs3["delay"], fs3["offDelay"], fs3["procedure"]) == (20, 30, 0),
          f"«Этап 2» пережил перезапуск: {fs3['delay'] / 10:.1f} с / {fs3['offDelay'] / 10:.1f} с / "
          f"{PROCEDURE_NAMES[fs3['procedure']]}")
    check((rxfail3[4]["mode"], rxfail3[4]["value"]) == (2, 1475),
          f"«Этап 1» пережил перезапуск: AUX 1 = {MODE_NAMES[rxfail3[4]['mode']]} {rxfail3[4]['value']}")
    check(rxfail3[0]["mode"] == 1 and rxfail3[3]["value"] == 1000, "правки стиков и газа на месте")

    for command, expected in (
        ("get rx_min_usec", "rx_min_usec = 1000"),
        ("get rx_max_usec", "rx_max_usec = 2000"),
        ("get failsafe_delay", "failsafe_delay = 20"),
        ("get failsafe_off_delay", "failsafe_off_delay = 30"),
        ("get failsafe_throttle", "failsafe_throttle = 1200"),
        ("get failsafe_switch_mode", "failsafe_switch_mode = STAGE2"),
        ("get failsafe_throttle_low_delay", "failsafe_throttle_low_delay = 50"),
        ("get failsafe_procedure", "failsafe_procedure = AUTO-LAND"),
        ("get failsafe_stick_threshold", "failsafe_stick_threshold = 30"),
    ):
        out = fc2.cli(command)
        check(expected in out, f"CLI '{command}' → {'найдено' if expected in out else 'НЕТ: ' + out.strip()[:40]!r}")

    print("\n-- 'defaults': заводские значения возвращаются (в том числе «Этап 1») --")
    fc2.cli("defaults")
    time.sleep(0.7)
    fc2.close()
    stop(proc)
    proc, fc3 = start(PORT, eeprom)
    fs4 = read_failsafe(fc3)
    rxfail4 = read_rxfail(fc3)
    rx4 = read_rx_config(fc3)
    check((fs4["delay"], fs4["offDelay"], fs4["procedure"]) == (15, 10, 1),
          f"«Этап 2» вернулся к заводскому: {fs4['delay'] / 10:.1f} с / {fs4['offDelay'] / 10:.1f} с / "
          f"{PROCEDURE_NAMES[fs4['procedure']]}")
    check([c["mode"] for c in rxfail4[:4]] == [0, 0, 0, 0] and all(c["mode"] == 1 for c in rxfail4[4:]),
          "«Этап 1» вернулся к заводскому: стики АВТО, AUX УДЕРЖАНИЕ")
    check((rx4["rx_min_usec"], rx4["rx_max_usec"]) == (885, 2115),
          f"диапазон импульсов: {rx4['rx_min_usec']}…{rx4['rx_max_usec']}")
    fc3.close()
    stop(proc)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for item in FAILS:
            print("  -", item)
        return 1
    print("ВКЛАДКА «FAILSAFE» РАБОТАЕТ: значения из прошивки, правки сохраняются и переживают перезапуск")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
