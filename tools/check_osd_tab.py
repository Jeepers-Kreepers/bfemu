#!/usr/bin/env python3
"""Проверка вкладки «OSD»: элементы, статистика, таймеры, предупреждения, запись и сохранение.

Проверка разбирает ответ MSP_OSD_CONFIG тем же порядком полей, что и Configurator
(src/components/tabs/osd/osd.js: OSD.msp.decode), поэтому «зелёный» прогон означает,
что вкладка получит полный набор данных, а не пустую страницу.
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

from bfemu.fc import (  # noqa: E402
    OSD_ITEM_COUNT, OSD_PROFILE_COUNT, OSD_STAT_COUNT, OSD_TIMER_COUNT, OSD_VISIBLE,
    OSD_WARNING_COUNT,
)

PORT_DEFAULT = 15761
FAILED = []


def check(ok: bool, text: str) -> bool:
    print(f"   [{'OK ' if ok else 'FAIL'}] {text}")
    if not ok:
        FAILED.append(text)
    return ok


def u16(data: bytes, pos: int) -> int:
    return int.from_bytes(data[pos:pos + 2], "little")


def u32(data: bytes, pos: int) -> int:
    return int.from_bytes(data[pos:pos + 4], "little")


def decode_osd(payload: bytes) -> dict:
    """Разбор ответа так, как это делает Configurator (osd.js, OSD.msp.decode)."""
    view = payload
    pos = 0
    result = {"flags": view[pos]}
    pos += 1
    if result["flags"] and len(view) > 1:
        result["video_system"] = view[pos]
        pos += 1
        result["units"] = view[pos]
        pos += 1
        result["alarms"] = {"rssi": view[pos]}
        pos += 1
        result["alarms"]["cap"] = u16(view, pos)
        pos += 2
        pos += 1                                   # устаревшее поле таймера
        items_count = view[pos]
        pos += 1
        result["alarms"]["alt"] = u16(view, pos)
        pos += 2
        result["items_count"] = items_count
        result["items"] = [u16(view, pos + 2 * i) for i in range(items_count)]
        pos += 2 * items_count
        stats_count = view[pos]
        pos += 1
        result["stats"] = [view[pos + i] == 1 for i in range(stats_count)]
        pos += stats_count
        timers_count = view[pos]
        pos += 1
        result["timers"] = [u16(view, pos + 2 * i) for i in range(timers_count)]
        pos += 2 * timers_count
        pos += 2                                   # младшее слово предупреждений
        warnings_count = view[pos]
        pos += 1
        result["warnings_flags"] = u32(view, pos)
        pos += 4
        result["warnings"] = [bool(result["warnings_flags"] & (1 << i)) for i in range(warnings_count)]
        result["profiles"] = view[pos]
        result["profile_selected"] = view[pos + 1] - 1
        pos += 2
        result["overlay_radio_mode"] = view[pos]
        pos += 1
        result["camera_frame"] = (view[pos], view[pos + 1])
        pos += 2
        result["alarms"]["link_quality"] = u16(view, pos)
        pos += 2
    result["bytes_read"] = pos
    result["bytes_total"] = len(view)
    return result


def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:                 # $M> (v1)
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5 : i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:                 # $X> (v2)
            size = buf[i + 6] | (buf[i + 7] << 8)
            if i + 8 + size <= len(buf):
                return buf[i + 8 : i + 8 + size]
    return None


class Fc:
    """Такой же клиент MSP, как в остальных проверках (encode_v1/encode_v2 из bfemu.msp)."""

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
                break
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.03)
        return b""

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def start_emulator(port: int, eeprom: Path):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT)
    proc = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"],
        cwd=ROOT, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    for _ in range(40):
        try:
            return proc, Fc(port)
        except OSError:
            time.sleep(0.25)
    raise RuntimeError("эмулятор не поднялся")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp", default=None, help="адрес эмулятора, напр. 127.0.0.1:15761")
    parser.add_argument("--port", type=int, default=None,
                        help="порт эмулятора; по умолчанию берётся свободный (чтобы проверка "
                             "не зацепила оставшийся от прошлого прогона эмулятор)")
    args = parser.parse_args()
    if args.port or args.tcp:
        port = args.port or int(str(args.tcp).split(":")[-1])
    else:
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

    eeprom = Path("/tmp/check_osd_eeprom.json")
    eeprom.unlink(missing_ok=True)
    proc, fc = start_emulator(port, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «OSD»: элементы, статистика, таймеры, предупреждения")
    print("=" * 78)

    payload = fc.call(84)
    data = decode_osd(payload)
    print(f"   ответ MSP_OSD_CONFIG: {len(payload)} байт, разобрано {data['bytes_read']}")
    check(data["flags"] != 0, f"флаги OSD непустые (0x{data['flags']:02x}) — вкладка не будет пустой")
    check(data["flags"] & (1 << 0), "флаг «функция OSD включена» (бит 0)")
    check(data["flags"] & (1 << 4), "флаг «аппаратный OSD MAX7456» (бит 4)")
    check(data["flags"] & (1 << 5), "флаг «устройство найдено» (бит 5) — шрифты и превью доступны")
    check(data["video_system"] in (0, 1, 2), f"видеосистема: {data['video_system']} (0 = AUTO)")
    check(data["units"] == 1, f"единицы измерения: {data['units']} (1 = метрическая)")
    check(data["items_count"] == OSD_ITEM_COUNT,
          f"элементов в ответе: {data['items_count']} (как OSD_ITEM_COUNT в прошивке)")
    check(len(data["items"]) == OSD_ITEM_COUNT, "места всех элементов прочитаны")
    check(data["stats_count"] if "stats_count" in data else len(data["stats"]) == OSD_STAT_COUNT,
          f"статистика: {len(data['stats'])} записей (OSD_STAT_COUNT)")
    check(len(data["timers"]) == OSD_TIMER_COUNT, f"таймеров: {len(data['timers'])} (OSD_TIMER_COUNT)")
    check(len(data["warnings"]) == OSD_WARNING_COUNT,
          f"предупреждений: {len(data['warnings'])} (OSD_WARNING_COUNT)")
    check(data["profiles"] == OSD_PROFILE_COUNT, f"профилей OSD: {data['profiles']}")
    check(data["profile_selected"] == 0, "выбран первый профиль (в ответе — с нуля)")
    check(data["alarms"]["rssi"] == 20 and data["alarms"]["cap"] == 2200
          and data["alarms"]["alt"] == 100 and data["alarms"]["link_quality"] == 80,
          "тревоги: RSSI 20 %, ёмкость 2200 мА·ч, высота 100 м, LQ 80 %")
    check(data["camera_frame"] == (24, 11), f"кадр камеры: {data['camera_frame'][0]}×{data['camera_frame'][1]}")
    check(data["bytes_read"] == data["bytes_total"],
          "ответ разобран до конца — «лишних» байт, на которых падает разбор, нет")

    visible = [(index, value) for index, value in enumerate(data["items"]) if value]
    print("\n-- что стоит на экране (профиль 1) --")
    from bfemu.fc import OSD_ITEMS
    for index, value in visible:
        x, y = value & 0x1F, (value >> 5) & 0x1F
        seen = bool(value & OSD_VISIBLE)
        print(f"   {index:2d} {OSD_ITEMS[index]:22s} x={x:2d} y={y:2d} видно: {seen}")
    check(len(visible) >= 4, f"на экране есть элементы ({len(visible)} шт.) — превью вкладки не пустое")

    canvas = fc.call(189)
    check(len(canvas) == 2, f"MSP_OSD_CANVAS: {len(canvas)} байта")
    print(f"   полотно OSD: {canvas[0]}×{canvas[1]} (столбцов × строк)")
    check(canvas[0] == 30 and canvas[1] == 16, "полотно как у SD-видео (30×16)")

    # --- запись: как это делает вкладка (перетаскивание элемента, галочки, тревоги) ---
    print("\n-- правим то же, что правит вкладка --")
    # ровно те кадры, что шлёт вкладка (stores/osd.ts: encodeLayout / encodeOther / encodeTimer)
    new_pos = 6 | (3 << 5) | OSD_VISIBLE          # ALTITUDE переезжает на x=6, y=3, профиль 1
    fc.call(85, bytes([15]) + new_pos.to_bytes(2, "little"))          # encodeLayout
    warnings_flags = 0x41FFF
    fc.call(85, bytes([0xFF, 2, 2, 25])                               # encodeOther: -1, видео, единицы,
            + (2500).to_bytes(2, "little")                            # rssi, cap u16,
            + (0).to_bytes(2, "little") + (120).to_bytes(2, "little")  # 0 u16 (устаревшее), alt u16,
            + (warnings_flags & 0xFFFF).to_bytes(2, "little")         # предупреждения u16,
            + warnings_flags.to_bytes(4, "little")                    # предупреждения u32,
            + bytes([2, 2, 24, 11])                                   # профиль 2, overlay, кадр камеры
            + (70).to_bytes(2, "little"))                             # порог LQ
    fc.call(85, bytes([0xFE, 1]) + (300).to_bytes(2, "little"))       # encodeTimer: -2, индекс 1, значение
    fc.call(85, bytes([5]) + (0).to_bytes(2, "little") + bytes([0]))  # статистика: индекс 5, выкл, экран 0
    after = decode_osd(fc.call(84))
    check(after["items"][15] == new_pos, "перетаскивание элемента сохранилось (новое место в ответе)")
    check(after["units"] == 2 and after["alarms"]["cap"] == 2500 and after["alarms"]["link_quality"] == 70,
          f"общие настройки приняты: единицы {after['units']}, ёмкость {after['alarms']['cap']}, LQ {after['alarms']['link_quality']}")
    check(after["profile_selected"] == 1, "выбран второй профиль OSD")
    check(after["timers"][1] == 300, f"таймер 2 = {after['timers'][1]}")
    check(after["stats"][5] is False, "галочка статистики снимается")

    # --- сохранение: как кнопка «Сохранить» во вкладке ---
    print("\n-- сохранение и перезапуск эмулятора --")
    fc.call(250)                                   # MSP_EEPROM_WRITE
    fc.close()
    proc.terminate()
    proc.wait(timeout=10)
    proc, fc = start_emulator(port, eeprom)
    saved = decode_osd(fc.call(84))
    check(saved["items"][15] == new_pos and saved["alarms"]["cap"] == 2500
          and saved["timers"][1] == 300 and saved["profile_selected"] == 1,
          "настройки OSD пережили перезапуск эмулятора")
    check(saved["items_count"] == OSD_ITEM_COUNT and saved["flags"] != 0,
          "после перезапуска вкладка снова получает полный ответ")

    fc.close()
    proc.terminate()
    proc.wait(timeout=10)
    eeprom.unlink(missing_ok=True)

    print()
    if FAILED:
        print(f"ПРОВАЛЕНО проверок: {len(FAILED)}")
        for item in FAILED:
            print("  -", item)
        return 1
    print("ВКЛАДКА «OSD» РАБОТАЕТ: элементы, статистика, таймеры и предупреждения читаются, "
          "правки сохраняются и переживают перезапуск")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
