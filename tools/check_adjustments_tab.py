#!/usr/bin/env python3
"""Проверка вкладки «Корректирование в полёте» (Adjustments).

Вкладка получает данные четырьмя запросами (composables/adjustments/useAdjustmentsData.js):
MSP_BOXNAMES (116), MSP_ADJUSTMENT_RANGES (52), MSP_BOXIDS (119) и MSP_RC (105). Слоты она
разбирает ровно как MSPHelper.ts: байта-счётчика в ответе нет, число слотов = длина ответа / 6
(при API >= 1.48 на слот приходится 10 байт: добавляются центр и масштаб).

Поэтому «зелёный» прогон означает, что вкладка увидит заполненную таблицу, а не пустую
страницу: она показывает только включённые слоты (начало диапазона != конец).
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
    ADJUSTMENT_FUNCTION_COUNT, ADJUSTMENT_RANGE_COUNT,
)

# Заводские шаги диапазона: импульс = 900 + шаг * 25 (rc_modes.h: channelRange_t).
STEP_MIN, STEP_MAX = 0, 48
# Какие функции корректировки прошивка выполняет «выбором» (MSPHelper: SELECT_MODE_FUNCTIONS).
SELECT_MODE_FUNCTIONS = {12, 24, 25, 29, 30, 31, 32, 33}

FAILED = []


def check(ok: bool, text: str) -> bool:
    print(f"   [{'OK ' if ok else 'FAIL'}] {text}")
    if not ok:
        FAILED.append(text)
    return ok


def _frame(buf: bytes):
    """Кадр ответа. Умеет «джамбо» MSP v1 (длина 255 → настоящая длина u16, msp.ts) —
    так прошивка шлёт длинные ответы, например MSP_BOXNAMES с 28 режимами."""
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:                 # $M> (v1)
            size = buf[i + 3]
            head = i + 5
            if size >= 255:                                          # джамбо-кадр
                if i + 7 > len(buf):
                    return None
                size = buf[i + 5] | (buf[i + 6] << 8)
                head += 2
            if head + size <= len(buf):
                return buf[head : head + size]
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
            except TimeoutError:
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


def decode_adjustments(payload: bytes, bytes_per_item: int = 6) -> list:
    """Разбор ровно как MSPHelper.ts: слотов = длина ответа / размер записи."""
    out, pos = [], 0
    for _ in range(len(payload) // bytes_per_item):
        item = {
            "slot_index": payload[pos],
            "aux": payload[pos + 1],
            "start": 900 + payload[pos + 2] * 25,
            "end": 900 + payload[pos + 3] * 25,
            "function": payload[pos + 4],
            "switch": payload[pos + 5],
            "center": 0,
            "scale": 0,
        }
        if bytes_per_item == 10:
            item["center"] = payload[pos + 6] | (payload[pos + 7] << 8)
            item["scale"] = payload[pos + 8] | (payload[pos + 9] << 8)
        out.append(item)
        pos += bytes_per_item
    return out


def mode_of(item: dict) -> str:
    """Режим, который нарисует вкладка (useAdjustmentsData.getAdjustmentMode)."""
    if item["function"] in SELECT_MODE_FUNCTIONS:
        return "выбор"
    return "абсолютный" if item["center"] > 0 else "пошаговый"


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
    parser.add_argument("--tcp", default=None, help="адрес эмулятора, напр. 127.0.0.1:15762")
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

    eeprom = Path("/tmp/check_adjustments_eeprom.json")
    eeprom.unlink(missing_ok=True)
    proc, fc = start_emulator(port, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «КОРРЕКТИРОВАНИЕ В ПОЛЁТЕ»: слоты, диапазоны, запись и сохранение")
    print("=" * 78)

    # --- то, что вкладка запрашивает первой (useAdjustmentsData.loadMSPData) ---
    boxnames = fc.call(116)
    boxids = fc.call(119)
    names = [n for n in boxnames.decode(errors="replace").split(";") if n]
    check(len(names) >= 8 and len(boxids) == len(names),
          f"MSP_BOXNAMES/MSP_BOXIDS: {len(names)} режимов и столько же идентификаторов")
    rc = fc.call(105)
    channels = len(rc) // 2
    check(channels == 16, f"MSP_RC: {channels} каналов — вкладка предложит {max(0, channels - 4)} AUX")

    payload = fc.call(52)
    ranges = decode_adjustments(payload)
    print(f"   ответ MSP_ADJUSTMENT_RANGES: {len(payload)} байт = {len(ranges)} слотов × 6 байт")
    check(len(payload) == ADJUSTMENT_RANGE_COUNT * 6,
          f"слотов в ответе: {len(ranges)} (MAX_ADJUSTMENT_RANGE_COUNT), без байта-счётчика")
    check(len(ranges) == ADJUSTMENT_RANGE_COUNT, "все слоты прочитаны — таблица вкладки не пустая")
    check(all(0 <= r["function"] <= ADJUSTMENT_FUNCTION_COUNT for r in ranges),
          f"номера функций в допустимых пределах (1…{ADJUSTMENT_FUNCTION_COUNT})")
    check(all(900 + r["start"] - 900 <= 2100 and r["end"] <= 2100 for r in ranges),
          "импульсы диапазонов в пределах 900…2100")

    active = [r for r in ranges if r["start"] != r["end"]]
    print(f"\n-- включённые слоты ({len(active)} из {len(ranges)}) — ровно их и рисует вкладка --")
    for index, item in enumerate(active, 1):
        print(f"   слот #{index}: AUX{item['aux'] + 1} {item['start']}…{item['end']} импульсов → "
              f"функция {item['function']} ({mode_of(item)}), через AUX{item['switch'] + 1}")
    check(len(active) >= 2,
          f"включённых слотов: {len(active)} — вкладка показывает таблицу, а не пустую страницу")
    check(any(r["start"] != r["end"] and r["end"] > 1500 for r in active),
          "есть слот, срабатывающий в верхнем положении тумблера (диапазон выше 1500)")

    # --- запись: ровно те кадры, что шлёт вкладка (MSPHelper.sendAdjustmentRanges) ---
    print("\n-- правим то же, что правит вкладка --")
    # слот 5 (индекс 4): AUX7 (индекс 6) 1550…1850 → RC Rate (1), через AUX7
    fc.call(53, bytes([4, 0, 6, 26, 38, 1, 6]))
    # слот 1 выключаем: диапазон 900…900 — так вкладка «гасит» слот
    fc.call(53, bytes([0, 0, 0, 0, 0, 0, 0]))
    after = decode_adjustments(fc.call(52))
    check(after[4]["aux"] == 6 and after[4]["start"] == 1550 and after[4]["end"] == 1850
          and after[4]["function"] == 1,
          f"новая корректировка записана: слот #5 = AUX7 {after[4]['start']}…{after[4]['end']}, "
          f"функция {after[4]['function']}")
    check(after[0]["start"] == after[0]["end"],
          "слот #1 выключен (диапазон 900…900) — вкладка снимет галочку")
    check(after[4]["switch"] == 6, "канал-переключатель (колонка «через канал») сохранён")
    check(len(after) == ADJUSTMENT_RANGE_COUNT,
          "число слотов не поехало после записи — разбор вкладки не сломается")

    # --- сохранение: как кнопка «Сохранить» во вкладке ---
    print("\n-- сохранение и перезапуск эмулятора --")
    fc.call(250)                                   # MSP_EEPROM_WRITE
    fc.close()
    proc.terminate()
    proc.wait(timeout=10)
    proc, fc = start_emulator(port, eeprom)
    saved = decode_adjustments(fc.call(52))
    check(saved[4]["aux"] == 6 and saved[4]["start"] == 1550 and saved[4]["end"] == 1850,
          "правка слота пережила перезапуск эмулятора")
    check(saved[0]["start"] == saved[0]["end"], "выключенный слот остался выключенным")
    check(len(saved) == ADJUSTMENT_RANGE_COUNT and len(fc.call(52)) == ADJUSTMENT_RANGE_COUNT * 6,
          "после перезапуска вкладка снова получает полный ответ (30 слотов)")

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
    print("ВКЛАДКА «КОРРЕКТИРОВАНИЕ В ПОЛЁТЕ» РАБОТАЕТ: слоты читаются, правки сохраняются "
          "и переживают перезапуск")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
