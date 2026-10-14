#!/usr/bin/env python3
"""Виртуальная аппаратура из консоли: двигать каналы виртуального FC.

Работает через ту же HTTP-ручку, что и страница <http://127.0.0.1:8088/tx>,
поэтому эмулятор должен быть запущен (обычно `run_windows.bat`).

Примеры:

    python tools/tx.py                                  # показать каналы
    python tools/tx.py roll=1700 yaw=1600 aux1=2000     # выставить каналы
    python tools/tx.py 1500 1500 1500 1000              # позиции 0…3 (A/E/R/T)
    python tools/tx.py center                           # стики в центр, тумблеры выкл
    python tools/tx.py demo                             # 6 секунд покачивания
    python tools/tx.py disconnect                       # «выдернуть USB» у аппаратуры
    python tools/tx.py connect                          # «воткнуть USB» обратно
    python tools/tx.py bind                             # «связать» (биндинг: 2 с поиска -> связана)
    python tools/tx.py unbind                           # «развязать» (кадров нет — «Этап 1»)
    python tools/tx.py joystick                         # вести каналы настоящим USB-пультом (Zorro и др.)
    python tools/tx.py axes                             # показать оси USB-пульта (подобрать --map)
    python tools/tx.py wire --port COM5                 # читать настоящий приёмник из COM-порта
    python tools/tx.py --http 8188 roll=1600            # если монитор на другом порту

Что происходит дальше: Configurator опрашивает MSP_RC каждые 33 мс, поэтому полосы на вкладке
«Приёмник» и модель в «Предпросмотре» реагируют сразу. Состояние стиков во «флеш» не пишется —
после перезапуска эмулятора аппаратура снова «в центре».

Каналы активны только тогда, когда во вкладке «Порты» на каком-нибудь UART включён **Serial RX**
(по умолчанию — UART2). Если Serial RX выключить, прошивка не поднимает приёмник, и MSP_RC
отдаёт пустой ответ — на вкладке «Приёмник» полосы становятся пустыми.

Режимы с настоящей аппаратурой (если она есть):

    python tools/tx.py joystick                         # пульт как USB-джойстик (без библиотек)
    python tools/tx.py joystick --detect                # калибровка: сам найдёт ось каждого стика
    python tools/tx.py axes                             # какие оси у пульта и что на них сейчас
    python tools/tx.py wire --port COM5                 # приёмник в USB-UART переходнике (pyserial)
    python tools/tx.py crsf --port COM5 --baud 420000   # то же, но только CRSF

**Zorro (EdgeTX/OpenTX) — самый простой путь.** На пульте: MODEL SETUP → USB Joystick, подключить
шнуром к компьютеру и запустить `python tools\tx.py joystick`. Никакие библиотеки не нужны:
пульт читается системным интерфейсом Windows (то же, что видят «Игровые устройства», `joy.cpl`).
Стики пульта начинают двигать каналы — как будто пульт связан с приёмником FC. Если стики
двигают не свои каналы (например, рыск попадает в AUX), запустите калибровку
`python tools\tx.py joystick --detect`: инструмент попросит отклонить по одному стику и сам
определит, какая ось какому стику соответствует и не перевёрнута ли она; раскладка сохранится
в `tx_map.json` и будет использоваться дальше автоматически. Вручную:
`--map roll=0,pitch=1,throttle=2,yaw=3 --invert pitch`.

Управление каналами всегда у одного хозяина: пока на странице /tx нажато «Подключить
аппаратуру», данные пульта принимаются; после «Отключить (выдернуть USB)» они отклоняются
(HTTP 409) и скрипт ждёт, пока управление вернут, — иначе пульт и ползунки писали бы каналы
одновременно, и получались бы «срывы».

**Приёмник в COM-порту.** Если аппаратура уже связана со своим приёмником (или вы хотите видеть
настоящий бинд), а приёмник подключён к USB-UART переходнику: `python tools\tx.py wire --port COM5`
— эмулятор сам определит CRSF (420000 бод) или SBUS и начнёт отдавать каналы Configurator.
Как только пошли кадры, эмулятор считает, что **бинд состоялся**; если кадры пропали на секунду —
«потеря связи» и «Этап 1» на вкладке «Приёмник»; для SBUS кадр с флагом failsafe тоже даёт «Этап 1».

Оба режима ведут каналы непрерывно: Configurator опрашивает MSP_RC каждые 33 мс, поэтому
полосы на вкладке «Приёмник» и модель в «Предпросмотре» реагируют сразу.

Установка зависимостей:
    python -m pip install pyserial      # только для режимов crsf/wire (приёмник в COM-порту)

Режиму joystick библиотеки не нужны: пульт читается встроенным способом — системным интерфейсом
Windows winmm, ровно как в «Игровых устройствах» (`Win+R` → `joy.cpl`). Ограничение интерфейса:
до 6 осей и 32 кнопок на устройство (у пульта их меньше), плюс читается «шляпа» (POV).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

NAMES = ["roll", "pitch", "yaw", "throttle"] + [f"aux{i}" for i in range(1, 13)]
LABELS = ["Крен", "Тангаж", "Рыск", "Газ"] + [f"AUX{i}" for i in range(1, 13)]


class ControlRefused(RuntimeError):
    """Эмулятор отклонил данные прибора: управление каналами сейчас у страницы /tx."""


def api(port: int, payload=None):
    url = f"http://127.0.0.1:{port}/api/rc"
    data = None if payload is None else json.dumps(payload).encode()
    request = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=4) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", "replace")
        if exc.code == 409:              # данные отклонены: управление у виртуальной аппаратуры
            raise ControlRefused(text) from exc
        raise SystemExit(f"эмулятор ответил ошибкой: {text}") from exc
    except (urllib.error.URLError, OSError) as exc:
        raise SystemExit(
            f"не вижу эмулятор на http://127.0.0.1:{port} — запустите run_windows.bat ({exc})"
        ) from exc


def show(state: dict, note: str = ""):
    if note:
        print(note)
    bind = state.get("bind") or {}
    if bind:
        mark = {"bound": "связана", "binding": "ИДЁТ БИНД", "none": "НЕ связана"}.get(bind.get("state"), "?")
        print(f"   биндинг: {mark} — {bind.get('text', '')}")
    for index, (name, label) in enumerate(zip(NAMES, LABELS, strict=False)):
        value = state["channels"][index]
        mark = ""
        if index < 4:
            mark = "центр" if value == 1500 else ("внизу" if value <= 1050 else "отклонён")
        else:
            mark = "ВКЛ" if value >= 1500 else "выкл"
        print(f"   {index:>2} {name:<9} {label:<7} {value:>5} мкс  {mark}")


def parse_assignments(tokens: list) -> dict:
    """roll=1700 aux1=2000 либо просто четыре числа (A/E/R/T)."""
    if tokens and all(t.replace(".", "").isdigit() for t in tokens):
        return {"channels": [int(t) for t in tokens]}
    mapping = {}
    for token in tokens:
        if "=" not in token:
            raise SystemExit(f"не понял {token!r}: нужен вид имя=значение, напр. roll=1700")
        name, _, value = token.partition("=")
        name = name.strip().lower()
        if name not in NAMES and not name.isdigit():
            raise SystemExit(f"неизвестный канал {name!r}. Известные: {', '.join(NAMES)}")
        try:
            mapping[name] = int(value)
        except ValueError as exc:
            raise SystemExit(f"значение {value!r} — не число") from exc
    return {"channels": mapping}


def demo(port: int, seconds: float = 6.0, step: float = 0.06):
    import math
    print(f"демо {seconds:.0f} с: покачиваю стики — смотрите «Предпросмотр» в Configurator")
    start = time.time()
    while True:
        t = time.time() - start
        if t > seconds:
            break
        api(port, {"source": "tools/tx.py (демо)", "channels": {
            "roll": 1500 + int(400 * math.sin(t * 1.6)),
            "pitch": 1500 + int(400 * math.sin(t * 1.1 + 1.0)),
            "yaw": 1500 + int(300 * math.sin(t * 0.7 + 2.0)),
            "throttle": 1300 + int(300 * math.sin(t * 0.9)),
            "aux1": 2000 if int(t * 1.25) % 2 else 1000,
        }})
        time.sleep(step)
    api(port, {"reset": True, "source": "tools/tx.py (демо)"})
    show(api(port), "демо закончилось, стики в центре:")


# ----------------------------------------------------------------- CRSF-кадры (настоящая аппаратура)
CRSF_ADDRESS_FC = 0xC8                 # адрес полётного контроллера
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_SCALE_LEGACY = 0.62477120195241   # crsf.h: CRSF_RC_CHANNEL_SCALE_LEGACY
CRSF_OFFSET_LEGACY = 881               # crsf.c: (channelScale * value) + 881


def crsf_us_to_raw(microseconds: float) -> int:
    """Микросекунды → «единицы CRSF» (172…1811): обратное к crsfReadRawRC()."""
    raw = (microseconds - CRSF_OFFSET_LEGACY) / CRSF_SCALE_LEGACY
    return max(172, min(1811, int(round(raw))))


def crsf_raw_to_us(raw: int) -> int:
    """«Единицы CRSF» → микросекунды: точь-в-точь crsfReadRawRC() из crsf.c."""
    return int(round(CRSF_SCALE_LEGACY * raw + CRSF_OFFSET_LEGACY))


def crsf_crc8(data: bytes, poly: int = 0xD5) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_crsf_channels_frame(microseconds: list) -> bytes:
    """Кадр CRSF 0x16 (RC channels packed) из 16 микросекунд — для тестов и синтетики."""
    values = [crsf_us_to_raw(us) for us in list(microseconds)[:16]]
    values += [crsf_us_to_raw(1000)] * (16 - len(values))
    packed = 0
    for index, value in enumerate(values):
        packed |= (value & 0x7FF) << (11 * index)
    payload = packed.to_bytes(22, "little")
    body = bytes([CRSF_FRAMETYPE_RC_CHANNELS_PACKED]) + payload
    return bytes([CRSF_ADDRESS_FC, len(body) + 1]) + body + bytes([crsf_crc8(body)])


def decode_crsf_rc_frame(frame: bytes):
    """Разбирает один кадр CRSF. Возвращает список из 16 микросекунд или None.

    Структура: [адрес][длина][тип][payload][crc8]; длина считает тип, payload и CRC.
    Проверяем адрес полётного контроллера (0xC8), тип 0x16 и CRC8 (полином 0xD5).
    """
    if len(frame) < 5 or frame[0] != CRSF_ADDRESS_FC:
        return None
    length = frame[1]
    if length < 4 or len(frame) < length + 2:
        return None
    body = frame[2 : 2 + length - 1]                    # тип + payload
    crc = frame[2 + length - 1]
    if frame[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED or crc != crsf_crc8(body):
        return None
    packed = int.from_bytes(body[1 : 1 + 22], "little")
    return [max(885, min(2115, crsf_raw_to_us((packed >> (11 * i)) & 0x7FF))) for i in range(16)]


def crsf_selftest() -> int:
    """Проверка кодека без железа: кадр → микросекунды."""
    sample = [1000, 1500, 2000, 1000] + [1000] * 12
    frame = build_crsf_channels_frame(sample)
    decoded = decode_crsf_rc_frame(frame)
    ok = decoded is not None and abs(decoded[0] - 1000) <= 1 and abs(decoded[1] - 1500) <= 1 \
        and abs(decoded[2] - 2000) <= 1
    print(f"кадр {len(frame)} байт: {frame.hex(' ')}")
    print(f"декодировано: {decoded[:4] if decoded else None} — {'OK' if ok else 'ОШИБКА'}")
    broken = bytearray(frame)
    broken[-1] ^= 0xFF
    print(f"кадр с испорченным CRC: {decode_crsf_rc_frame(bytes(broken))} (должно быть None)")
    return 0 if ok and decode_crsf_rc_frame(bytes(broken)) is None else 1


SBUS_HEADER = 0x0F                     # sbus.c: SBUS_FRAME_HEADER
SBUS_FOOTER = 0x00                     # sbus.c: SBUS_FRAME_FOOTER
SBUS_FRAME_LENGTH = 25                 # 1 байт заголовок + 22 байта каналов + флаги + хвост
SBUS_SCALE = 0.625                     # ширина бита канала (5/8), как в sbus.c
SBUS_OFFSET = 881                      # минимальный импульс, как в прошивке
SBUS_FLAG_FRAME_LOST = 0x08
SBUS_FLAG_FAILSAFE = 0x04
SBUS_VALUE_MASK = 0x07FF


def sbus_us_to_raw(microseconds: float) -> int:
    """мкс -> 11-битное значение SBUS (172 -> 988 мкс, 1811 -> 2012 мкс)."""
    return int(round((float(microseconds) - SBUS_OFFSET) / SBUS_SCALE))


def sbus_raw_to_us(raw: int) -> int:
    """11-битное значение SBUS -> мкс."""
    return int(round(int(raw) * SBUS_SCALE + SBUS_OFFSET))


def build_sbus_frame(microseconds: list, failsafe: bool = False, frame_lost: bool = False) -> bytes:
    """Собрать кадр SBUS (25 байт) — как его шлёт приёмник в линию к FC."""
    bits = 0
    for index, value in enumerate(list(microseconds)[:16]):
        bits |= (sbus_us_to_raw(value) & SBUS_VALUE_MASK) << (11 * index)
    flags = (SBUS_FLAG_FAILSAFE if failsafe else 0) | (SBUS_FLAG_FRAME_LOST if frame_lost else 0)
    return bytes([SBUS_HEADER]) + bits.to_bytes(22, "little") + bytes([flags, SBUS_FOOTER])


def decode_sbus_frame(frame: bytes):
    """Кадр SBUS -> (каналы в мкс, флаг failsafe) либо None, если кадр битый."""
    if len(frame) != SBUS_FRAME_LENGTH or frame[0] != SBUS_HEADER or frame[-1] != SBUS_FOOTER:
        return None
    bits = int.from_bytes(frame[1:23], "little")
    channels = [sbus_raw_to_us((bits >> (11 * index)) & SBUS_VALUE_MASK) for index in range(16)]
    return channels, bool(frame[23] & SBUS_FLAG_FAILSAFE)


class WireDecoder:
    """Разбор потока байтов из COM-порта (настоящий приёмник) на каналы.

    Понимает два протокола, которые отдают каналы «как есть» в линию:

      * CRSF — кадры 0x16 от приёмника к FC (420000 бод);
      * SBUS — кадры 25 байт с заголовком 0x0F (100000 бод, нужен инвертирующий переходник).

    `feed()` возвращает список событий: {"channels": [...16 значений...], "failsafe": bool,
    "protocol": "crsf"/"sbus"}. Кадр с флагом failsafe (SBUS) даёт событие с failsafe=True —
    это «аппаратура выключена/пропала», на вкладке «Приёмник» будет «Этап 1».

    Протокол `auto`: по первому целому кадру определяется протокол и дальше слушается только он
    (в эфире одновременно CRSF и SBUS не бывает). Если известно заранее — `--protocol crsf|sbus`.
    """

    def __init__(self, protocol: str = "auto"):
        self.protocol = (protocol or "auto").lower()
        self.buffer = b""

    def feed(self, data: bytes) -> list:
        """Скормить очередную порцию байтов из порта, получить события с каналами."""
        self.buffer += bytes(data or b"")
        if self.protocol == "auto":
            # Пробуем оба варианта на одном и том же буфере и «залипаем» на том протоколе,
            # по которому прошёл первый целый кадр: смешивать CRSF и SBUS в одном потоке
            # нельзя — сканер одного съедал бы байты другого.
            for candidate in ("crsf", "sbus"):
                saved = self.buffer
                events = getattr(self, "_scan_" + candidate)()
                remainder = self.buffer
                self.buffer = saved
                if events:
                    self.protocol = candidate
                    self.buffer = remainder
                    return events
            self.buffer = self.buffer[-64:]      # ещё не поняли протокол — храним хвост
            return []
        events = self._scan_crsf() if self.protocol == "crsf" else self._scan_sbus()
        if len(self.buffer) > 512:
            self.buffer = self.buffer[-256:]
        return events

    def _scan_crsf(self) -> list:
        events = []
        while True:
            start = self.buffer.find(bytes([CRSF_ADDRESS_FC]))
            if start < 0:
                self.buffer = self.buffer[-1:] if self.buffer else b""
                break
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 2:
                break
            need = self.buffer[1] + 2
            if len(self.buffer) < need:
                break
            frame, self.buffer = self.buffer[:need], self.buffer[need:]
            channels = decode_crsf_rc_frame(frame)
            if channels:
                events.append({"channels": channels, "failsafe": False, "protocol": "crsf"})
        return events

    def _scan_sbus(self) -> list:
        events = []
        while True:
            start = self.buffer.find(bytes([SBUS_HEADER]))
            if start < 0:
                self.buffer = b""
                break
            self.buffer = self.buffer[start:]
            if len(self.buffer) < SBUS_FRAME_LENGTH:
                break
            frame, rest = self.buffer[:SBUS_FRAME_LENGTH], self.buffer[SBUS_FRAME_LENGTH:]
            decoded = decode_sbus_frame(frame)
            if decoded is None:
                self.buffer = self.buffer[1:]          # ложный заголовок — ищем дальше
                continue
            channels, failsafe = decoded
            events.append({"channels": channels, "failsafe": failsafe, "protocol": "sbus"})
            self.buffer = rest
        return events


def sbus_selftest() -> int:
    """Проверка SBUS-кодека без железа: кадр -> байты -> кадр."""
    print("самопроверка SBUS:")
    frame = build_sbus_frame([988, 1500, 2012, 1000, 2000, 1500, 1500, 1500,
                              1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])
    print(f"   кадр {len(frame)} Б: {frame[:6].hex(' ')} … {frame[-3:].hex(' ')}")
    decoded = decode_sbus_frame(frame)
    assert decoded is not None, "кадр не декодировался"
    channels, failsafe = decoded
    print(f"   декодировано: {channels[:5]} …, failsafe={failsafe}")
    assert channels[:5] == [988, 1500, 2012, 1000, 2000], channels[:5]
    assert not failsafe
    bad = decode_sbus_frame(frame[:-1] + b"\x01")
    print(f"   битый хвост кадра -> {bad}")
    assert bad is None
    fs = decode_sbus_frame(build_sbus_frame([1500] * 16, failsafe=True))
    assert fs is not None and fs[1] is True, fs
    print("   кадр с флагом failsafe распознан")

    decoder = WireDecoder("sbus")
    assert decoder.feed(frame[:7]) == [], "неполный кадр не должен давать события"
    events = decoder.feed(frame[7:])
    assert len(events) == 1 and events[0]["channels"][:4] == [988, 1500, 2012, 1000], events
    print("   поток байтов: неполный кадр — тишина, целый — каналы")
    print("SBUS: ВСЁ ВЕРНО\n")
    return 0


def selftest() -> int:
    """Все самопроверки, которые не требуют железа: CRSF, SBUS, разбор потока."""
    crsf_selftest()
    sbus_selftest()
    print("самопроверка разбора потока (WireDecoder):")
    decoder = WireDecoder("auto")
    events = decoder.feed(b"\x00\xff" + build_crsf_channels_frame([1100, 1500, 1900, 1200] + [1500] * 12))
    assert [e["protocol"] for e in events] == ["crsf"] and events[0]["channels"][:4] == [1100, 1500, 1900, 1200], events
    print(f"   поток CRSF с мусором впереди: {len(events)} кадр(а), авто-определение -> crsf")
    assert decoder.protocol == "crsf" and decoder.feed(b"\x00") == [], "после определения протокол не меняется"

    decoder = WireDecoder("auto")
    events = decoder.feed(b"\x55\x55" + build_sbus_frame([1000, 1600, 1400, 1100] + [1000] * 12) + b"\x99")
    assert [e["protocol"] for e in events] == ["sbus"] and events[0]["channels"][:4] == [1000, 1600, 1400, 1100], events
    print(f"   поток SBUS с мусором вокруг: {len(events)} кадр, авто-определение -> sbus")
    assert decoder.protocol == "sbus"
    print("разбор потока: ВСЁ ВЕРНО")

    print("ВСЕ САМОПРОВЕРКИ ПРОЙДЕНЫ")
    return 0


def crsf(port: int, serial_name: str, baud: int, rate: float):
    """Читает кадры CRSF из COM-порта и ведёт каналы виртуального FC (см. wire())."""
    return wire(port, serial_name, baud, "crsf", rate)


def _invert_values(values: list, invert: str) -> list:
    """Зеркалить выбранные каналы вокруг 1500 (`--invert roll,pitch`)."""
    names = {name.strip().lower() for name in (invert or "").replace(" ", "").split(",") if name.strip()}
    indexes = {NAMES.index(name) for name in names if name in NAMES}
    if not indexes:
        return list(values)
    return [3000 - value if index in indexes else value for index, value in enumerate(values)]


def wire(port: int, serial_name: str, baud: int, protocol: str = "auto", rate: float = 100.0,
         invert: str = ""):
    """Читать НАСТОЯЩИЙ приёмник из COM-порта: каналы идут прямо в виртуальный FC.

    Пока идут кадры, эмулятор считает, что аппаратура связана (бинд состоялся) и «в эфире».
    Пропали кадры на секунду (или пришёл SBUS-кадр с флагом failsafe) — «Этап 1» и
    предупреждение о потерянной связи на вкладке «Приёмник».
    """
    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "нужен пакет pyserial, чтобы читать приёмник из COM-порта:\n"
            "    python -m pip install pyserial"
        ) from exc
    print(f"открываю {serial_name} @ {baud} бод, протокол {protocol} ...")
    try:
        link = serial.Serial(serial_name, baud, timeout=0.1)
    except Exception as exc:
        raise SystemExit(f"не удалось открыть {serial_name}: {exc}") from exc

    lock_take()                              # два писателя одновременно = «прыгающие» каналы
    source = f"приёмник {serial_name} [{my_source().split('[', 1)[-1]}"
    print(build_note())
    started_note = foreign_note(api(port, {"control": "physical", "source": source}))
    if started_note:
        print(started_note)
    print("управление передано физической аппаратуре (страница /tx на это время гаснут)")
    print("читаю кадры приёмника — Ctrl+C для выхода")
    print("   приёмник обычно отдаёт CRSF 420000 или SBUS 100000 (для SBUS нужен инвертор);")
    print("   как только пойдут кадры, аппаратура считается связанной (биндинг выполнен).")

    decoder = WireDecoder(protocol)
    frames = 0
    last_frame = 0.0
    last_log = time.time()
    last_beat = 0.0
    lost = False
    refused = False
    step = 1.0 / max(rate, 1)
    try:
        while True:
            events = decoder.feed(link.read(256))
            now = time.time()
            for event in events:
                frames += 1
                if event["failsafe"]:
                    if not lost:
                        lost = True
                        api(port, {"connected": False, "source": f"RX {serial_name}: failsafe-кадр"})
                        print("   приёмник отдал кадр failsafe — «Этап 1»")
                    continue
                channels = _invert_values(event["channels"], invert)
                payload = {"channels": channels, "connected": True, "physical": True,
                           "source": f"RX {serial_name} ({event['protocol'].upper()})"}
                if lost or last_frame == 0.0:
                    payload["bind"] = True          # кадры пошли — значит, связались
                    payload["bind_seconds"] = 0
                try:
                    api(port, payload)
                except ControlRefused:
                    if not refused:
                        refused = True
                        print("   управление каналами сейчас у страницы /tx — данные приёмника не "
                              "принимаются: нажмите «Подключить аппаратуру» на странице")
                    break
                if refused:
                    refused = False
                    print("   управление снова у физической аппаратуры — продолжаю")
                last_frame = now
                if lost:
                    lost = False
                    print("   кадры снова идут — связь восстановлена")
            if frames and not lost and now - last_frame > 1.0:
                lost = True
                api(port, {"connected": False, "source": f"RX {serial_name}: кадров нет"})
                print("   кадров нет больше секунды — потеря связи, «Этап 1»")
            if now - last_beat > 1.0:
                last_beat = now
                lock_beat()
            if now - last_log > 2.0 and frames:
                last_log = now
                state = api(port)
                sticks = " ".join(f"{label}={value}" for label, value in
                                  zip(LABELS[:4], state["channels"][:4], strict=False))
                print(f"   кадров {frames}, {sticks}  (биндинг: {state['bind']['text']})")
            if rate:
                time.sleep(step)
    except KeyboardInterrupt:
        print("\nостановлено")
    finally:
        link.close()
        lock_release()
        try:
            # аппаратуру «выключаем»: управление возвращается странице /tx, каналов нет («Этап 1»)
            api(port, {"connected": False, "control": "virtual",
                       "source": f"RX {serial_name}: чтение остановлено, управление у страницы /tx"})
            print("управление возвращено странице /tx — нажмите там «Связать», чтобы каналы снова пошли")
        except SystemExit:
            pass                            # эмулятор уже остановлен — это не ошибка


def winmm_normalize(value, minimum, maximum) -> float:
    """Абсолютное значение оси Windows (0…65535) -> -1…1. Центр (32767) даёт 0."""
    span = float(maximum) - float(minimum)
    if span <= 0:
        return 0.0
    return max(-1.0, min(1.0, (2.0 * (float(value) - float(minimum)) / span) - 1.0))


def build_payload(mapping: dict, axes: list, buttons: list, hats: list = (),
                  aux_map=()) -> dict:
    """Каналы из показаний пульта: стики по `--map`, тумблеры/крутилки — по раскладке AUX.

    Оси приходят в -1…1 (у обоих способов чтения), поэтому мкс = 1000 + (значение + 1) * 500.
    Оси, назначенные тумблерам и крутилкам калибровкой (`--detect-aux`), ведут свои AUX-каналы:
    тумблер переключает 1000/1500/2000, крутилка — 1000…2000 линейно. Остальные свободные оси,
    кнопки и «шляпа» идут в оставшиеся AUX-каналы (в покое 1000, чтобы режимы не срабатывали).
    """
    payload = {}
    used = set(mapping.values())
    for name, axis in mapping.items():
        if 0 <= axis < len(axes):
            payload[name] = int(round(1000 + (axes[axis] + 1) * 500))
    taken_channels = set()
    for item in aux_map or []:
        axis, channel = int(item.get("axis", -1)), int(item.get("channel", 0))
        if not 1 <= channel <= AUX_MAX or axis in used or not 0 <= axis < len(axes):
            continue
        payload[f"aux{channel}"] = aux_value(item, axes[axis])
        taken_channels.add(channel)
        used.add(axis)
    free = [a for a in range(len(axes)) if a not in used]
    slots = [n for n in range(1, AUX_MAX + 1) if n not in taken_channels]
    for slot, axis in zip(slots, free, strict=False):
        payload[f"aux{slot}"] = int(round(1000 + (axes[axis] + 1) * 500))
    base = max(taken_channels | {int(k[3:]) for k in payload if k.startswith("aux")} or {0}) + 1
    for i, pressed in enumerate(buttons):
        name = f"aux{base + i}"
        if name in NAMES:
            payload[name] = 2000 if pressed else 1000
    # «Шляпа» (POV): положительное направление — 2000 («включено»), центр и отрицательное — 1000.
    # Так в покое канал выключен и назначенные на AUX режимы не срабатывают сами.
    slot = base + len(buttons)
    for hat in hats:
        for value in tuple(hat)[:2]:
            name = f"aux{slot}"
            if name in NAMES:
                payload[name] = 2000 if value > 0 else 1000
            slot += 1
    return payload


class WinMMJoystick:
    """USB-пульт в Windows без сторонних библиотек (winmm: joyGetPosEx / joyGetDevCapsW).

    Работает через тот же интерфейс, что и «Игровые устройства» Windows (`joy.cpl`), поэтому
    ничего доустанавливать не нужно: аппаратура в режиме USB Joystick видна сразу. Ограничения
    Windows-интерфейса: до 6 осей и до 32 кнопок на устройство — для пульта (4 стика + тумблеры)
    с запасом хватает.
    """

    label = "winmm (встроенный режим Windows, без библиотек)"
    AXIS_NAMES = ("X", "Y", "Z", "R", "U", "V")

    def __init__(self):
        import ctypes
        if not sys.platform.startswith("win"):
            raise RuntimeError("чтение USB-пульта встроенным способом доступно только в Windows "
                               f"(сейчас {sys.platform})")
        from ctypes import wintypes

        class JOYINFOEX(ctypes.Structure):
            _fields_ = [("dwSize", wintypes.DWORD), ("dwFlags", wintypes.DWORD),
                        ("dwXpos", wintypes.DWORD), ("dwYpos", wintypes.DWORD),
                        ("dwZpos", wintypes.DWORD), ("dwRpos", wintypes.DWORD),
                        ("dwUpos", wintypes.DWORD), ("dwVpos", wintypes.DWORD),
                        ("dwButtons", wintypes.DWORD), ("dwButtonNumber", wintypes.DWORD),
                        ("dwPOV", wintypes.DWORD),
                        ("dwReserved1", wintypes.DWORD), ("dwReserved2", wintypes.DWORD)]

        class JOYCAPSW(ctypes.Structure):
            _fields_ = [("wMid", wintypes.WORD), ("wPid", wintypes.WORD),
                        ("szPname", ctypes.c_wchar * 32),
                        ("wXmin", wintypes.UINT), ("wXmax", wintypes.UINT),
                        ("wYmin", wintypes.UINT), ("wYmax", wintypes.UINT),
                        ("wZmin", wintypes.UINT), ("wZmax", wintypes.UINT),
                        ("wNumButtons", wintypes.UINT),
                        ("wPeriodMin", wintypes.UINT), ("wPeriodMax", wintypes.UINT),
                        ("wRmin", wintypes.UINT), ("wRmax", wintypes.UINT),
                        ("wUmin", wintypes.UINT), ("wUmax", wintypes.UINT),
                        ("wVmin", wintypes.UINT), ("wVmax", wintypes.UINT),
                        ("wCaps", wintypes.UINT), ("wMaxAxes", wintypes.UINT),
                        ("wNumAxes", wintypes.UINT), ("wMaxButtons", wintypes.UINT),
                        ("szRegKey", ctypes.c_wchar * 32),
                        ("szOEMVxD", ctypes.c_wchar * 260)]

        self._ctypes = ctypes
        self._JOYINFOEX, self._JOYCAPSW = JOYINFOEX, JOYCAPSW
        self._winmm = ctypes.windll.winmm
        self._count = int(self._winmm.joyGetNumDevs())
        self._caps = {}

    def describe(self) -> str:
        return f"winmm, устройств в системе: {self._count} (библиотеки не нужны)"

    def _caps_for(self, index):
        if index not in self._caps:
            caps = self._JOYCAPSW()
            rc = self._winmm.joyGetDevCapsW(index, self._ctypes.byref(caps),
                                            self._ctypes.sizeof(caps))
            self._caps[index] = caps if rc == 0 else None
        return self._caps[index]

    def devices(self) -> list:
        found = []
        for index in range(min(self._count, 16)):
            caps = self._caps_for(index)
            if caps is None:
                continue
            found.append({"index": index, "name": caps.szPname.strip() or f"джойстик {index}",
                          "axes": int(caps.wNumAxes), "buttons": int(caps.wNumButtons)})
        return found

    def read(self, device: dict):
        index = device["index"]
        info = self._JOYINFOEX()
        info.dwSize = self._ctypes.sizeof(info)
        info.dwFlags = 0xFF                                  # JOY_RETURNALL
        if self._winmm.joyGetPosEx(index, self._ctypes.byref(info)) != 0:
            raise RuntimeError(f"джойстик {index} не отвечает — выдернут USB?")
        caps = self._caps_for(index)
        positions = (info.dwXpos, info.dwYpos, info.dwZpos, info.dwRpos, info.dwUpos, info.dwVpos)
        ranges = ((caps.wXmin, caps.wXmax), (caps.wYmin, caps.wYmax), (caps.wZmin, caps.wZmax),
                  (caps.wRmin, caps.wRmax), (caps.wUmin, caps.wUmax), (caps.wVmin, caps.wVmax))
        axes = [winmm_normalize(positions[i], ranges[i][0], ranges[i][1])
                for i in range(min(len(self.AXIS_NAMES), int(caps.wNumAxes)))]
        buttons = [bool(info.dwButtons & (1 << i)) for i in range(min(32, int(caps.wNumButtons)))]
        hats = []
        if info.dwPOV != 0xFFFF:                             # «шляпа»: 0 — вверх, 9000 — вправо
            angle = math.radians(info.dwPOV / 100.0)
            hats.append((round(math.sin(angle)), round(math.cos(angle))))
        return axes, buttons, hats


TOOL_BUILD = "2026-10-08-c"           # сверяется с bfemu.__build__ в smoke-тесте
MAP_FILE = Path(__file__).resolve().parent.parent / "tx_map.json"
LOCK_FILE = Path(__file__).resolve().parent.parent / "tx_lock.json"   # кто сейчас ведёт каналы
LOCK_STALE = 6.0                    # если отклик старше — считаем, что прежний запуск уже закрыт


def pid_alive(pid) -> bool:
    """Жив ли процесс с таким номером (аккуратно, без «убийственных» сигналов в Windows)."""
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return True
    if pid <= 0:
        return True
    if os.name == "nt":
        # os.kill(pid, 0) в Windows завершает процесс — для проверки он не годится
        import ctypes
        PROCESS_QUERY_LIMITED_INFORMATION, STILL_ACTIVE = 0x1000, 259
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
        if not handle:
            return False
        code = ctypes.c_ulong()
        ok = kernel32.GetExitCodeProcess(handle, ctypes.byref(code))
        kernel32.CloseHandle(handle)
        return bool(ok) and code.value == STILL_ACTIVE
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def lock_read() -> dict:
    """Данные о текущем писателе каналов из tx_lock.json (свежие — только если отклик новый)."""
    try:
        data = json.loads(LOCK_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    age = time.time() - float(data.get("heartbeat") or 0)
    data["age"] = age
    data["pid_alive"] = pid_alive(data.get("pid")) if data.get("pid") else True
    # инструмент жив, если и откликается недавно, и процесс ещё существует
    data["alive"] = age <= LOCK_STALE and data["pid_alive"]
    return data


def lock_take(force: bool = False) -> dict:
    """Занять место писателя каналов. Если его уже занял другой запуск — остановиться с подсказкой.

    Без этого легко получить «прыгающие» каналы: старое окно инструмента продолжает вести каналы
    со своей (прежней) раскладкой осей, а новое — со своей. Снаружи это выглядит как «рыск уезжает
    в AUX, коптер крутится».
    """
    current = lock_read()
    if current.get("alive") and not force:
        raise SystemExit(
            f"каналы уже ведёт другой запуск инструмента (PID {current.get('pid', '?')}, "
            f"отклик {current['age']:.1f} с назад).\n"
            "Скорее всего, осталось открытым прежнее окно с 'python tools\\tx.py joystick' или\n"
            "run_tx_joystick.bat. Нажмите в нём Ctrl+C (или закройте окно) и запустите этот заново.\n"
            "Если прежний запуск точно закрыт — подождите пару секунд или добавьте --force.")
    data = {"pid": os.getpid(), "started": time.strftime("%H:%M:%S"),
            "heartbeat": time.time(), "when": time.strftime("%Y-%m-%d %H:%M")}
    LOCK_FILE.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return data


def lock_beat() -> None:
    """Отметить, что инструмент жив (раз в секунду)."""
    try:
        data = json.loads(LOCK_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        data = {}
    data.update({"pid": os.getpid(), "heartbeat": time.time()})
    try:
        LOCK_FILE.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    except OSError:
        pass


def lock_release() -> None:
    """Освободить место писателя каналов."""
    try:
        data = json.loads(LOCK_FILE.read_text(encoding="utf-8"))
        if int(data.get("pid") or 0) == os.getpid():
            LOCK_FILE.unlink(missing_ok=True)
    except (OSError, ValueError):
        LOCK_FILE.unlink(missing_ok=True)


def my_source(device_name: str = "") -> str:
    """Подпись источника: с PID, чтобы эмулятор мог отличить два окна инструмента."""
    return f"USB-пульт: {device_name} [PID {os.getpid()}]" if device_name \
        else f"tools/tx.py [PID {os.getpid()}]"


def build_note() -> str:
    """Строка «какая это сборка» — по ней видно старое окно в другом терминале."""
    return f"инструмент tools/tx.py, сборка {TOOL_BUILD}"


def foreign_note(state: dict) -> str:
    """Предупреждение, если каналы прямо сейчас писал кто-то другой (пустая строка — если нет)."""
    ago = (state or {}).get("updated_ago")
    source = (state or {}).get("source") or ""
    if ago is None or ago > 3.0 or not source:
        return ""
    return ("   [!] каналы только что писал другой источник: " + source +
            f" (обновлено {ago} с назад)."
            "\n       Если это ваше прежнее окно инструмента — нажмите там Ctrl+C:"
            "\n       пока работают оба, каналы будут «прыгать».")


def pid_of_source(name: str):
    """Номер процесса из подписи источника («… [PID 20376]») или None."""
    text = str(name or "")
    marker = "[PID "
    pos = text.find(marker)
    if pos < 0:
        return None
    digits = ""
    for ch in text[pos + len(marker):]:
        if ch.isdigit():
            digits += ch
        else:
            break
    return int(digits) if digits else None


def other_tool_pids(state: dict, own: int | None = None) -> list:
    """Номера ДРУГИХ запусков инструмента, которые сейчас пишут каналы (кроме нашего)."""
    own = os.getpid() if own is None else own
    pids = []
    for name in (state or {}).get("sources") or []:
        pid = pid_of_source(name)
        if pid and pid != own and pid not in pids:
            pids.append(pid)
    return pids


def kill_command(pid: int) -> str:
    """Готовая команда закрытия процесса для этой системы."""
    return f"taskkill /PID {pid} /F" if os.name == "nt" else f"kill -9 {pid}"


def close_other_tools(port: int) -> list:
    """Закрыть чужие окна инструмента (те, что сейчас пишут каналы). Возвращает [(PID, получилось)]."""
    pids = other_tool_pids(api(port))
    result = []
    for pid in pids:
        cmd = (["taskkill", "/PID", str(pid), "/F"] if os.name == "nt"
               else ["kill", "-9", str(pid)])
        try:
            done = subprocess.run(cmd, capture_output=True, text=True)
            result.append((pid, done.returncode == 0))
        except OSError:
            result.append((pid, False))
    return result


def multi_source_note(sources, own: int | None = None) -> str:
    """Текст тревоги «каналы пишут несколько источников» (или пустая строка, если всё хорошо)."""
    names = [str(n) for n in (sources or []) if str(n).strip()]
    if len(names) < 2:
        return ""
    own = os.getpid() if own is None else own
    others = []
    lines = ["   [!] каналы в эмуляторе пишут несколько источников — это окна инструмента:"]
    for name in names:
        pid = pid_of_source(name)
        if pid == own:
            lines.append(f"       • {name}   ← это окно (здесь напечатана эта строка)")
        else:
            lines.append(f"       • {name}   ← ЧУЖОЕ окно")
            if pid:
                others.append(pid)
    lines.append(f"       Сверить легко: {build_note()} — она печатается при запуске.")
    if others:
        lines.append("       Закройте чужое окно: Ctrl+C в нём, либо выполните: " +
                     "; ".join(kill_command(pid) for pid in others))
        lines.append("       (или перезапустите инструмент с ключом --kill-others — он закроет чужие"
                     " окна сам)")
    else:
        lines.append("       Закройте все окна, кроме одного: в лишнем нажмите Ctrl+C.")
    lines.append("       Пока оба работают, каналы «прыгают»: «рыск уводит AUX»,"
                 " «коптер постоянно крутится».")
    return "\n".join(lines)


def mismatch_note(mismatch, source: str = "", updated_ago=None) -> str:
    """Текст тревоги «эмулятор отдаёт не то, что мы посылаем» (или пустая строка)."""
    if not mismatch:
        return ""
    shown = "; ".join(f"{label}: послали {ours}, в FC {theirs}" for label, ours, theirs in mismatch[:4])
    ago = "—" if updated_ago is None else f"{updated_ago} с назад"
    line = (f"   [!] эмулятор отдаёт не то, что мы посылаем ({shown})."
            f"\n       источник сейчас: {source or '—'}, обновлено {ago}.")
    pid = pid_of_source(source)
    if pid and pid != os.getpid():
        line += (f"\n       Это ЧУЖОЕ окно инструмента (PID {pid}) — оно переписывает каналы."
                 f"\n       Закройте его: Ctrl+C в нём, либо выполните: {kill_command(pid)}")
    elif not pid:
        line += "\n       Проверьте, не работает ли ещё одно окно инструмента (Ctrl+C в нём)."
    return line


def compare_with_fc(sent: dict, state: dict, invert=()) -> list:
    """Что мы послали против того, что лежит в FC: список расхождений (канал, послали, в FC).

    Расхождение — верный признак, что в эмулятор пишет кто-то ещё (второе окно инструмента):
    сами мы отправляем каналы десятки раз в секунду, и они тут же перезаписываются чужими.
    """
    channels = (state or {}).get("channels") or []
    rows = []
    for name, value in sent.items():
        if name not in NAMES or not isinstance(value, int):
            continue
        index = NAMES.index(name)
        if index >= len(channels):
            continue
        ours, theirs = int(value), int(channels[index])
        limit = 4 if index >= 4 else 2          # у AUX 1 эмулятор добавляет ±2 мкс «живости»
        if abs(ours - theirs) > limit:
            rows.append((LABELS[index], ours, theirs))
    return rows
DEFAULT_MAP = {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 3}


def load_map() -> dict:
    """Сохранённая раскладка осей (`tx_map.json`, её пишет калибровка `--detect`)."""
    try:
        saved = json.loads(MAP_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    mapping = {name: int(value) for name, value in (saved.get("map") or {}).items()
               if name in NAMES and str(value).lstrip("-").isdigit()}
    invert = [name for name in (saved.get("invert") or []) if name in NAMES]
    if mapping:
        mapping["invert"] = invert
    return mapping


def load_aux() -> list:
    """Раскладка AUX из tx_map.json: какие оси ведут тумблеры и крутилки и на какие каналы."""
    try:
        saved = json.loads(MAP_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    result = []
    for item in saved.get("aux") or []:
        if not isinstance(item, dict):
            continue
        kind = str(item.get("type") or "")
        axis, channel = item.get("axis"), item.get("channel")
        if kind not in AUX_TYPES or not str(axis).lstrip("-").isdigit() \
                or not str(channel).lstrip("-").isdigit():
            continue
        clean = {"axis": int(axis), "type": kind, "channel": int(channel)}
        if kind == "knob":
            clean["min"] = float(item.get("min", -1.0))
            clean["max"] = float(item.get("max", 1.0))
        else:
            levels = [float(level) for level in (item.get("levels") or []) if str(level).lstrip("-.").replace(".", "", 1).isdigit()]
            clean["levels"] = levels or ([-1.0, 1.0] if kind == "switch2" else [-1.0, 0.0, 1.0])
        result.append(clean)
    return result


def save_map(mapping: dict, invert, device: str = "", aux=None) -> dict:
    """Сохранить раскладку стиков и AUX, чтобы не повторять калибровку при следующем запуске.

    `aux=None` — оставить прежнюю раскладку AUX (её пишет отдельный шаг калибровки тумблеров).
    """
    previous = load_aux() if aux is None else []
    data = {
        "map": {name: axis for name, axis in mapping.items() if name in NAMES},
        "invert": [name for name in invert_names(invert)],
        "device": device,
        "when": time.strftime("%Y-%m-%d %H:%M"),
        "aux": [dict(item) for item in (previous if aux is None else aux)],
    }
    MAP_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return data


def invert_names(invert) -> list:
    """Нормализовать список зеркальных каналов: строка «pitch,yaw» или список."""
    if isinstance(invert, str):
        return [name.strip().lower() for name in invert.replace(" ", "").split(",") if name.strip()]
    return [str(name).strip().lower() for name in (invert or []) if str(name).strip()]


def format_map(mapping: dict, invert=()) -> str:
    parts = [f"{LABELS[NAMES.index(name)]} — ось {axis}" for name, axis in mapping.items() if name in NAMES]
    names = invert_names(invert)
    text = ", ".join(parts)
    if names:
        text += " (зеркалятся: " + ", ".join(LABELS[NAMES.index(n)] for n in names if n in NAMES) + ")"
    return text


def parse_map(text: str) -> tuple:
    """Раскладка из ключа `--map`: «auto» (файл или AETR) либо «roll=0,pitch=1,…»."""
    text = (text or "").strip()
    if text in ("", "auto"):
        saved = load_map()
        if saved:
            invert = saved.pop("invert", [])
            if all(name in saved for name in ("roll", "pitch", "throttle", "yaw")):
                print(f"использую сохранённую раскладку осей ({MAP_FILE.name}): {format_map(saved, invert)}")
                return saved, invert
            print(f"[!] в {MAP_FILE.name} раскладка неполная ({format_map(saved, invert)}) — "
                  f"беру AETR. Откалибруйте заново: python tools\\tx.py joystick --detect")
        print("раскладка по умолчанию AETR: roll=0, pitch=1, throttle=2, yaw=3")
        print("если стики двигают не свои каналы — запустите: python tools\\tx.py joystick --detect")
        return dict(DEFAULT_MAP), []
    mapping = {}
    for pair in text.split(","):
        name, _, axis = pair.partition("=")
        name = name.strip()
        if name in NAMES and axis.strip().lstrip("-").isdigit():
            mapping[name] = int(axis)
    if not mapping:
        raise SystemExit("не понял --map, пример: --map roll=0,pitch=1,throttle=2,yaw=3 "
                         "[--invert pitch], либо --map auto")
    return mapping, []


def best_axis(baseline: list, sample: list, used) -> tuple:
    """По отклонению стика найти ось: (номер оси, зеркалить ли) либо None, если не двигали.

    `baseline` — положения осей в покое, `sample` — при отклонённом стике. Берём ось с
    наибольшим отклонением; если значение ушло в минус — канал надо зеркалить (вверх/вправо
    должно быть 2000 мкс).
    """
    best, delta = None, 0.0
    for index, (rest, moved) in enumerate(zip(baseline, sample, strict=False)):
        if index in used:
            continue
        if abs(moved - rest) > delta:
            best, delta = index, abs(moved - rest)
    if best is None or delta < 0.25:            # стик не отклоняли
        return None
    return best, (sample[best] - baseline[best] < 0)


# ---------------------------------------------------------------- AUX (тумблеры и крутилки)
AUX_TYPES = ("switch2", "switch3", "knob")
AUX_TYPE_LABELS = {"switch2": "тумблер (2 положения)", "switch3": "тумблер (3 положения)",
                   "knob": "крутилка"}
AUX_MAX = 12                        # AUX1…AUX12, столько каналов отдаёт прошивка
# Каналы выдаются по порядку обнаружения: первый найденный тумблер → AUX1, второй → AUX2 и так
# далее. Так совпадает с микшером пульта (CH5 → AUX1, CH6 → AUX2 …), а поменять всегда можно
# ключом --aux-map. Нужен именно порядок: какой канал у тумблера в Configurator, видно по полосе,
# которая двигается при переключении.
AUX_AXIS_MOTION = 0.45              # насколько должна сдвинуться ось, чтобы счесть это движением
AUX_AXIS_SETTLE = 0.25              # столько значение должно простоять, чтобы стать «положением»
AUX_LEVEL_GAP = 0.25                # положения ближе этого считаются одним и тем же


def aux_value(item: dict, value: float) -> int:
    """Значение AUX-канала (мкс) из положения оси: тумблер — по положениям, крутилка — линейно.

    Тумблер на 2 положения даёт 1000/2000, на 3 — 1000/1500/2000 (по числу найденных положений),
    крутилка — 1000…2000 линейно по откалиброванному ходу. Положения берутся измеренные, поэтому
    размах у разных пультов (-1…1, 0…1, -1…0) роли не играет.
    """
    kind = item.get("type")
    if kind == "knob":
        low, high = float(item.get("min", -1.0)), float(item.get("max", 1.0))
        if high - low < 0.2:                     # ход не откалиброван — считаем серединой
            return 1500
        ratio = min(1.0, max(0.0, (float(value) - low) / (high - low)))
        return int(round(1000 + 1000 * ratio))
    levels = sorted(float(level) for level in (item.get("levels") or []))
    if len(levels) < 2:
        return 1000
    steps = [1000 + int(round(1000 * index / (len(levels) - 1))) for index in range(len(levels))]
    nearest = min(range(len(levels)), key=lambda index: abs(float(value) - levels[index]))
    return steps[nearest]


def format_aux(aux) -> str:
    """«AUX1 ← тумблер (2 положения) — ось 3, AUX3 ← тумблер (3 положения) — ось 4»."""
    parts = []
    for item in aux or []:
        kind = AUX_TYPE_LABELS.get(item.get("type"), str(item.get("type")))
        parts.append(f"AUX{item['channel']} ← {kind} — ось {item['axis']}")
    return ", ".join(parts)


def format_aux_note(aux) -> str:
    """Короткая строка про AUX для консоли и для состояния на странице /tx."""
    return format_aux(aux)


def parse_aux_map(text) -> list:
    """Раскладка AUX из ключа `--aux-map`: auto | none | «3=switch2,4=switch3:4,5=knob:7»."""
    if isinstance(text, list):                    # уже разобранный список (например, из tx_map.json)
        return [dict(item) for item in text]
    text = (text or "").strip()
    if text in ("", "auto"):
        return load_aux()
    if text in ("none", "-"):
        return []
    result = []
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        spec, _, tail = chunk.partition("=")
        axis = spec.strip()
        if not axis.lstrip("-").isdigit():
            raise SystemExit("не понял --aux-map: нужен вид «ось=тип[:канал]», "
                             "например --aux-map 3=switch2,4=switch3:4")
        kind, _, channel = tail.strip().partition(":")
        kind = kind.strip().lower()
        if kind not in AUX_TYPES:
            raise SystemExit(f"тип «{kind}» не знаю: можно {', '.join(AUX_TYPES)}")
        item = {"axis": int(axis), "type": kind, "channel": int(channel) if channel.strip().isdigit()
                else pick_aux_channel(kind, [it["channel"] for it in result])}
        if kind == "knob":
            item["min"], item["max"] = -1.0, 1.0
        elif kind == "switch2":
            item["levels"] = [-1.0, 1.0]
        else:
            item["levels"] = [-1.0, 0.0, 1.0]
        result.append(item)
    return result


def pick_aux_channel(kind: str, taken) -> int:
    """Первый свободный AUX-канал: первый тумблер ложится на AUX1 (CH5), второй — на AUX2 (CH6)…

    Тип канала (`kind`) на выбор не влияет: номер канала у тумблера задаёт микшер пульта, а нам
    важно повторить привычный порядок. Сменить всегда можно: `--aux-map 3=switch2:3`.
    """
    taken = {int(n) for n in taken or []}
    for channel in range(1, AUX_MAX + 1):
        if channel not in taken:
            return channel
    raise SystemExit("все AUX1…AUX12 уже заняты — освободите канал или проверьте --aux-map")


STICK_LIMIT = 0.60                  # стик крена/тангажа/рыска в покое обязан быть в центре
DEAD_LIMIT = 0.9                    # ось, застывшая на упоре, похожа на тумблер или мёртвую ось


def layout_warnings(mapping: dict, axes: list, axis_names=()) -> list:
    """Подсказки по раскладке — только то, что действительно однозначно.

    Стики крена, тангажа и рыска сами возвращаются в центр: если такой канал в покое стоит на
    упоре, значит ему отдана чужая ось (например, рыску досталась ось газа или тумблера).
    Про газ предупреждений нет: у него бывает трещотка, и ручка в покое стоит где угодно —
    «центр у газа» ни о чём не говорит.
    """
    warnings = []
    seen = {}
    for name in ("roll", "pitch", "yaw", "throttle"):
        axis = mapping.get(name)
        if axis is None or not (0 <= axis < len(axes)):
            continue
        if axis in seen:
            warnings.append(f"на одну ось {axis} назначены два канала ({LABELS[NAMES.index(seen[axis])]} "
                            f"и {LABELS[NAMES.index(name)]}) — калибровка нужна заново: "
                            f"python tools\\tx.py joystick --detect")
            continue
        seen[axis] = name
        if name == "throttle":
            continue
        if abs(axes[axis]) > STICK_LIMIT:
            label = f"ось {axis}" + (f" ({axis_names[axis]})" if axis < len(axis_names) else "")
            warnings.append(
                f"канал {LABELS[NAMES.index(name)]} назначен на {label}, а стик крена/тангажа/рыска\n"
                f"   в покое обязан быть в ЦЕНТРЕ, а эта ось стоит на упоре ({axes[axis]:+.2f}).\n"
                "   Похоже, каналу отдана чужая ось (газ или тумблер). Исправьте калибровкой:\n"
                "       python tools\\tx.py joystick --detect")
    return warnings


def describe_axes(axes: list, mapping: dict, axis_names=(), invert=()) -> list:
    """Что видно по каждой оси пульта: покоится в центре или на упоре и какому каналу отдана.

    Печатается при запуске `joystick` — по этим строкам сразу видно, та ли раскладка
    (у газа ручка внизу, у стиков — центр).
    """
    used_by = {axis: name for name, axis in mapping.items() if name in NAMES}
    mirrored = invert_names(invert)
    lines = []
    for index, value in enumerate(axes):
        label = f"ось {index}" + (f" ({axis_names[index]})" if index < len(axis_names) else "")
        where = ("упор внизу/влево" if value <= -0.95 else "упор вверху/вправо" if value >= 0.95
                 else "центр" if abs(value) <= 0.05 else "середина")
        who = used_by.get(index)
        if who:
            label += f" → {LABELS[NAMES.index(who)]}" + (" [зеркалится]" if who in mirrored else "")
            if who != "throttle" and abs(value) > STICK_LIMIT:
                where += "   [!] стик крена/тангажа/рыска в покое должен быть в центре: --detect"
        lines.append(f"   {label}: {value:+.2f} ({where})")
    return lines


def sticks_table(mapping: dict, axes: list, invert=(), axis_names=()) -> str:
    """«Крен — ось 0 (X) …»: расшифровка номеров осей, чтобы сверять со списком `axes`."""
    names = axis_names
    mirrored = invert_names(invert)
    parts = []
    for name in ("roll", "pitch", "throttle", "yaw"):
        if name not in mapping:
            continue
        axis = mapping[name]
        label = f"{LABELS[NAMES.index(name)]} — ось {axis}"
        if axis < len(names):
            label += f" ({names[axis]})"
        if name in mirrored:
            label += " [зеркалится]"
        value = f"{axes[axis]:+.2f}" if 0 <= axis < len(axes) else "?"
        parts.append(f"{label}, сейчас {value}")
    return "; ".join(parts)


DETECT_STEPS = (
    ("roll",  "стик КРЕН — обычно ПРАВЫЙ стик вправо-влево: отклоните ВПРАВО до упора"),
    ("pitch", "стик ТАНГАЖ — обычно ПРАВЫЙ стик вверх-вниз: отклоните ВВЕРХ до упора"),
    ("yaw",   "стик РЫСК — обычно ЛЕВЫЙ стик влево-вправо: отклоните ВПРАВО до упора"),
    ("throttle", "стик ГАЗ — обычно ЛЕВЫЙ стик вверх-вниз: подвигайте его вверх-вниз "
                 "(до упоров, можно несколько раз) и ОСТАВЬТЕ ВВЕРХУ"),
)


def _wait_for_stick(radio, device, baseline: list, used, motion: float = 0.35,
                    hold: float = 0.5, timeout: float = 30.0) -> tuple:
    """Ждать, пока отклонят стик: сама находит ось с наибольшим отклонением (и зеркалирование).

    Человеку не нужно попадать в момент измерения: измеряем, пока не увидим устойчивое
    отклонение больше `motion` дольше `hold` секунд. Возвращает (ось, зеркалить ли) либо None.
    """
    started = time.time()
    candidate, since = None, 0.0
    print("      …жду отклонения стика (до упора)", flush=True)
    while time.time() - started < timeout:
        axes_values, _buttons, _hats = radio.read(device)
        deltas = {index: value - baseline[index] for index, value in enumerate(axes_values)
                  if index not in used and index < len(baseline)}
        if deltas:
            axis = max(deltas, key=lambda i: abs(deltas[i]))
            if abs(deltas[axis]) >= motion:
                if candidate != axis:
                    candidate, since = axis, time.time()
                elif time.time() - since >= hold:
                    return axis, deltas[axis] < 0
            else:
                candidate, since = None, 0.0
        time.sleep(0.04)
    return None


def _wait_for_throttle(radio, device, used, need_range: float = 0.8,
                       settle: float = 0.7, timeout: float = 30.0):
    """Ждать, пока подвигают газ вверх-вниз, и найти его ось по размаху движения.

    У газа бывает трещотка и ручка стоит где угодно, поэтому ось ищем не по «отклонению от
    покоя», а по размаху: подвигайте газ до упоров. Направление определяем по последнему
    положению — ручку просят оставить ВВЕРХУ, значит на этой оси должно быть «плюс».
    """
    started, lows, highs = time.time(), None, None
    settled, settled_since = None, None
    print("      …жду движения газа (вверх-вниз)", flush=True)
    while time.time() - started < timeout:
        axes_values, _buttons, _hats = radio.read(device)
        lows = list(axes_values) if lows is None else [min(a, b) for a, b in zip(lows, axes_values, strict=False)]
        highs = list(axes_values) if highs is None else [max(a, b) for a, b in zip(highs, axes_values, strict=False)]
        free = [i for i in range(len(axes_values)) if i not in used]
        if free:
            axis = max(free, key=lambda i: highs[i] - lows[i])
            if highs[axis] - lows[axis] >= need_range:        # по ручке прошли почти целиком
                value = axes_values[axis]
                if settled is None or abs(value - settled) > 0.08:
                    settled, settled_since = value, time.time()
                elif time.time() - settled_since >= settle:   # ручку оставили и держат
                    return axis, value < 0
        time.sleep(0.04)
    return None


def _sample_mean(radio, device, seconds: float) -> list:
    """Средние положения осей за `seconds` секунд."""
    started, sums, count = time.time(), None, 0
    while time.time() - started < seconds:
        axes_values, _buttons, _hats = radio.read(device)
        sums = axes_values if sums is None else [a + b for a, b in zip(sums, axes_values, strict=False)]
        count += 1
        time.sleep(0.05)
    return [value / max(count, 1) for value in (sums or [])]


def stable_levels(samples, gap: float = AUX_LEVEL_GAP, min_run: int = 4) -> list:
    """Устойчивые положения оси из потока измерений: у тумблера их 2–3, у крутилки — много."""
    levels, run = [], []
    for value in [*list(samples), None]:
        if value is not None and (not run or abs(value - run[-1]) <= gap):
            run.append(value)
            continue
        if len(run) >= min_run:
            mean = sum(run) / len(run)
            if not any(abs(mean - level) <= gap for level in levels):
                levels.append(mean)
        run = [] if value is None else [value]
    return sorted(levels)


def classify_aux_sample(samples, rest=None, level_gap: float = AUX_LEVEL_GAP) -> tuple:
    """Тумблер или крутилка? Возвращает (тип, данные раскладки).

    Тумблер за время наблюдения стоит в двух-трёх положениях, поэтому почти все измерения
    попадают рядом с ними. У крутилки положение меняется плавно, и таких «кучек» не выходит.
    """
    values = [float(v) for v in samples]
    if not values:
        return "switch2", {"levels": [-1.0, 1.0]}
    levels = stable_levels(values)
    if rest is not None and not any(abs(float(rest) - level) <= level_gap for level in levels):
        levels.append(float(rest))               # покоившееся положение — тоже положение тумблера
        levels.sort()
    near = sum(1 for value in values
               if any(abs(value - level) <= level_gap for level in levels)) / len(values)
    if len(levels) >= 4 or near < 0.7:
        return "knob", {"min": round(min(values), 3), "max": round(max(values), 3)}
    if len(levels) <= 2:
        return "switch2", {"levels": [round(level, 3) for level in levels]}
    return "switch3", {"levels": [round(level, 3) for level in levels]}


def _wait_for_aux_axis(radio, device, baseline: list, exclude, motion: float = AUX_AXIS_MOTION,
                       hold: float = 0.3, timeout: float = 8.0, watch: float = 7.0):
    """Ждать движения тумблера/крутилки: вернуть (ось, измерения за время наблюдения) либо None.

    Ось ищем по отклонению от покоя среди осей, не занятых стиками и уже найденными тумблерами.
    Затем наблюдаем эту ось `watch` секунд — человеку этого хватает, чтобы перевести тумблер
    во все положения или крутануть крутилку до упоров.
    """
    exclude = set(exclude)
    started, candidate, since = time.time(), None, 0.0
    while time.time() - started < timeout:
        axes_values, _buttons, _hats = radio.read(device)
        deltas = {index: abs(axes_values[index] - baseline[index])
                  for index in range(min(len(axes_values), len(baseline)))
                  if index not in exclude}
        axis = max(deltas, key=lambda i: deltas[i]) if deltas else None
        if axis is not None and deltas[axis] >= motion:
            if candidate != axis:
                candidate, since = axis, time.time()
            elif time.time() - since >= hold:
                samples = []
                watch_started = time.time()
                while time.time() - watch_started < watch:
                    values, _b, _h = radio.read(device)
                    if axis < len(values):
                        samples.append(values[axis])
                    time.sleep(0.03)
                return axis, samples
        else:
            candidate, since = None, 0.0
        time.sleep(0.03)
    return None


def _knob_range(radio, device, axis: int, seconds: float = 5.0) -> tuple:
    """Найти ход крутилки: минимальное и максимальное положение за `seconds` секунд."""
    lows = highs = None
    started = time.time()
    while time.time() - started < seconds:
        values, _b, _h = radio.read(device)
        if axis < len(values):
            value = values[axis]
            lows = value if lows is None else min(lows, value)
            highs = value if highs is None else max(highs, value)
        time.sleep(0.03)
    if lows is None:
        return -1.0, 1.0
    if highs - lows < 0.5:                       # до упоров не докрутили — берём весь ход оси
        return -1.0, 1.0
    return round(lows, 3), round(highs, 3)


def aux_axis_hints(baseline: list, used, axis_names=()) -> list:
    """Подсказки про свободные оси: какие в покое стоят на упоре (похоже на тумблеры)."""
    lines = []
    for index, value in enumerate(baseline):
        if index in set(used):
            continue
        label = f"ось {index}" + (f" ({axis_names[index]})" if index < len(axis_names) else "")
        if abs(value) >= 0.9:
            lines.append(f"   • {label} в покое на упоре ({value:+.2f}) — похоже, это тумблер "
                         f"(или ось, которую пульт не отдаёт)")
    return lines


def silent_axes_note(baseline: list, moved, used, found_axes, axis_names=()) -> list:
    """Что сказать про оси, которые так и не шевельнулись: чаще всего тумблер не назначен в микшере."""
    lines = []
    for index, value in enumerate(baseline):
        if index in set(used) or index in set(found_axes) or index in set(moved):
            continue
        label = f"ось {index}" + (f" ({axis_names[index]})" if index < len(axis_names) else "")
        lines.append(
            f"   [!] {label} за всю калибровку не шевельнулась (в покое {value:+.2f}).\n"
            "       В режиме USB Joystick пульт отдаёт по USB только те каналы, что назначены\n"
            "       в микшере модели (обычно CH5…CH8). Проверьте на пульте: MODEL SETUP → MIXES —\n"
            "       тумблер/крутилка должны быть назначены на канал, и тогда он появится здесь осью.")
    return lines


def detect_aux(radio, device, mapping: dict, watch: float = 7.0, idle: float = 8.0,
               max_aux: int = 8, quiet: bool = False) -> list:
    """Калибровка тумблеров и крутилок: сама находит оси, тип и назначает им AUX-каналы.

    Ничего нажимать в такт не нужно: переведите тумблер (или покрутите крутилку) — инструмент
    увидит движение, понаблюдает за осью и сам решит, что это: тумблер на 2 положения, тумблер
    на 3 положения или крутилка. Найденное сохраняется в tx_map.json и переживает перезапуск.
    """
    used = {int(axis) for axis in mapping.values() if str(axis).lstrip("-").isdigit()}
    axis_names = tuple(getattr(radio, "AXIS_NAMES", ()) or ())
    if not quiet:
        print("\nКалибровка AUX: тумблеры и крутилки. Нажимать ничего не надо.")
        print("   • переведите тумблер во ВСЕ его положения (щёлк-щёлк) — для трёхпозиционного")
        print("     пройдите все три;")
        print("   • крутилку покрутите до упоров в обе стороны;")
        print(f"   • когда переключать больше нечего — просто подождите {idle:.0f} секунд, я закончу.")
    baseline = _sample_mean(radio, device, 0.8)
    hints = aux_axis_hints(baseline, used, axis_names)
    if hints and not quiet:
        print("   что видно в покое:")
        for line in hints:
            print(line)
    found, moved_axes = [], []
    for step in range(1, max_aux + 1):
        if not quiet:
            print(f"\n   Шаг {step}: переключите ЛЮБОЙ ещё не найденный тумблер или крутилку"
                  f" (жду {idle:.0f} с)", flush=True)
        seen = _wait_for_aux_axis(radio, device, baseline,
                                  exclude=used | {item["axis"] for item in found} | set(moved_axes),
                                  timeout=idle, watch=watch)
        if seen is None:
            break
        axis, samples = seen
        moved_axes.append(axis)
        kind, extra = classify_aux_sample(samples, baseline[axis] if axis < len(baseline) else None)
        if kind == "knob":
            low, high = min(samples), max(samples)
            if high - low < 1.2 and not quiet:
                print("      похоже на крутилку — покрутите её до упоров в обе стороны (5 с)…")
                low, high = _knob_range(radio, device, axis, seconds=5.0)
            extra = {"min": round(low, 3), "max": round(high, 3)}   # ход: крутилка линейна
        item = {"axis": axis, "type": kind,
                "channel": pick_aux_channel(kind, [other["channel"] for other in found])}
        item.update(extra)
        found.append(item)
        if not quiet:
            label = f"ось {axis}" + (f" ({axis_names[axis]})" if axis < len(axis_names) else "")
            detail = (f"ход {extra['min']:+.2f}…{extra['max']:+.2f}" if kind == "knob"
                      else "положения " + " / ".join(f"{aux_value(item, level)}" for level in extra["levels"]))
            channel_hint = f" (на пульте это обычно CH{axis + 1})"
            print(f"      нашёл: {label} → AUX{item['channel']}, "
                  f"{AUX_TYPE_LABELS[kind]}, {detail}{channel_hint}")
    if found:
        if not quiet:
            print(f"\n   найдено {len(found)}: " + format_aux(found))
            if any(item["type"] == "switch2" for item in found):
                print("   если у тумблера на самом деле три положения — повторите калибровку AUX,",
                      "пройдя все три (или задайте вручную: --aux-map 3=switch3)")
    silence = silent_axes_note(baseline, moved_axes, used, [item["axis"] for item in found], axis_names)
    if not quiet and silence:
        print("\nчто осталось непонятным:")
        for line in silence:
            print(line)
    if not quiet:
        free_channels = [n for n in range(1, AUX_MAX + 1) if n not in [item["channel"] for item in found]]
        if free_channels:
            print(f"   AUX-каналы {free_channels[:6]} останутся в покое 1000 — их можно назначить "
                  "вручную: --aux-map 5=switch2,6=knob")
            print("   если в микшере пульта тумблер стоит на другом канале, поправьте номер: "
                  "--aux-map 3=switch2:3 (ось 3 → AUX3)")
    return found


def detect_mapping(radio, device, seconds: float = 1.5) -> tuple:
    """Калибровка стиков: ждёт движения каждого стика и сама находит ось и зеркалирование.

    Ничего нажимать в такт не нужно: одно движение стиком — один шаг. Раскладка сохраняется
    в `tx_map.json` только если найдены ВСЕ четыре стика (иначе файл не пишется, а старая
    «половинчатая» раскладка не могла бы испортить каналы).
    """
    print("Калибровка стиков. Всего 4 шага: двигайте по одному стику, как попросят.")
    print("После каждого шага инструмент сам перейдёт к следующему — нажимать ничего не надо.")
    print("Отпустите стики в центр и не трогайте их.")
    input("   когда готовы — нажмите Enter… ")
    baseline = _sample_mean(radio, device, 0.8)
    mapping, invert, used = {}, [], set()
    for name, prompt in DETECT_STEPS:
        label = LABELS[NAMES.index(name)]
        print(f"\n   Шаг {len(mapping) + 1}/4 — {label}: {prompt}")
        if name == "throttle":
            found = _wait_for_throttle(radio, device, used)
        else:
            found = _wait_for_stick(radio, device, baseline, used)
        if found is None:
            print(f"      [!] движения стика {label} не увидел (30 с). Шаг НЕ засчитан.")
            continue
        axis, inverted = found
        mapping[name] = axis
        used.add(axis)
        if inverted:
            invert.append(name)
        print(f"      определил: ось {axis}" + (" (зеркалится)" if inverted else ""))
        if name != "throttle":
            print("      отпустите стик в центр")
            time.sleep(0.6)
    missing = [LABELS[NAMES.index(n)] for n, _ in DETECT_STEPS if n not in mapping]
    if missing:
        raise SystemExit(
            f"калибровка не закончена: не определены {', '.join(missing)}.\n"
            "Раскладка НЕ сохранена (иначе каналы поехали бы вразнобой).\n"
            "Запустите ещё раз и отклоняйте стик до упора после подсказки:\n"
            "    python tools\\tx.py joystick --detect")
    save_map(mapping, invert, device.get("name", ""))
    print(f"\nраскладка сохранена в {MAP_FILE}: {format_map(mapping, invert)}")
    axis_names = tuple(getattr(radio, "AXIS_NAMES", ()) or ())
    print("\nкак это выглядит сейчас (стики отпущены):")
    try:
        rest_axes, _b, _h = radio.read(device)
    except RuntimeError:
        rest_axes = baseline
    for line in describe_axes(rest_axes, mapping, axis_names, invert):
        print(line)
    for name in ("roll", "pitch", "yaw"):
        if name in mapping:
            print(f"   проверка: {LABELS[NAMES.index(name)]} в покое = "
                  f"{rest_axes[mapping[name]]:+.2f} (должно быть около 0 — стик в центре)")
    free = [index for index in range(len(rest_axes)) if index not in used]
    if free:
        print("   свободные оси " + ", ".join(str(i) for i in free) +
              " уходят в AUX-каналы по порядку (в покое там 1000). Если ваши тумблеры на пульте\n"
              "   двигают эти оси — они появятся на соответствующем AUX.")
    return mapping, invert


def open_backend():
    """Открыть чтение USB-пульта: встроенный способ Windows (winmm), без сторонних библиотек."""
    try:
        return WinMMJoystick()
    except RuntimeError as exc:
        raise SystemExit(
            f"не получилось открыть USB-пульт: {exc}\n\n"
            "Что делать:\n"
            "   • читать пульт можно только в Windows и только встроенным способом — ставки на\n"
            "     библиотеки в этой сборке нет;\n"
            "   • проверьте, что пульт в режиме USB Joystick (EdgeTX/OpenTX: MODEL SETUP → USB Joystick);\n"
            "   • если пульта нет вообще, каналы можно вести страницей http://127.0.0.1:8088/tx\n"
            "     или консолью: python tools\\tx.py roll=1700 yaw=1600 aux1=2000"
        ) from exc


def _describe_devices(radio, found: list):
    print(f"способ чтения: {radio.describe()}")
    print(f"найдено устройств: {len(found)}")
    for device in found:
        print(f"   [{device['index']}] {device['name']} — осей {device['axes']}, "
              f"кнопок {device['buttons']}")


def axes(index: int = 0, seconds: float = 5.0):
    """Показать USB-пульты и их оси — чтобы подобрать `--map` для режима joystick."""
    radio = open_backend()
    found = radio.devices()
    if not found:
        raise SystemExit(
            "USB-пульт не найден. Включите на аппаратуре режим USB Joystick (EdgeTX/OpenTX:\n"
            "MODEL SETUP -> USB Joystick) и подключите её шнуром; в Windows она должна появиться\n"
            "в «Игровые устройства» (Win+R -> joy.cpl)."
        )
    _describe_devices(radio, found)
    if index >= len(found):
        raise SystemExit(f"устройства с номером {index} нет (всего {len(found)})")
    device = found[index]
    print(f"\nдвигайте стики пульта [{index}] {device['name']} — показываю положения {seconds:.0f} с:")
    started = time.time()
    try:
        while time.time() - started < seconds:
            axes_values, buttons, hats = radio.read(device)
            shown = " | ".join(f"ось {i}: {value:+.2f} ({1000 + (value + 1) * 500:.0f} мкс)"
                               for i, value in enumerate(axes_values))
            pressed = [i for i, state in enumerate(buttons) if state]
            extra = ("  кнопки: " + str(pressed)) if pressed else ""
            if hats and tuple(hats[0]) != (0, 0):
                extra += f"  шляпа: {tuple(hats[0])}"
            print("   " + shown + extra)
            time.sleep(0.15)
    except KeyboardInterrupt:
        print()
    print("\nподсказка: назначить оси каналам можно так:")
    print("   python tools/tx.py joystick --map roll=0,pitch=1,throttle=2,yaw=3 [--invert pitch]")
    return 0


def joystick(port: int, index: int, rate: float, mapping: dict, invert="", detect: bool = False,
             force: bool = False, kill_others: bool = False, aux_map=None,
             detect_aux_run: bool = False, no_aux: bool = False, aux_only: bool = False):
    """Вести каналы стиками настоящего USB-пульта (встроенное чтение в Windows, без библиотек)."""
    radio = open_backend()
    found = radio.devices()
    if not found:
        raise SystemExit(
            "USB-пульт не найден. Проверьте, что аппаратура подключена по USB и включён режим\n"
            "USB Joystick (EdgeTX/OpenTX: MODEL SETUP → USB Joystick); в Windows она должна\n"
            "появиться в «Игровые устройства» (Win+R → joy.cpl)."
        )
    _describe_devices(radio, found)
    if index >= len(found):
        raise SystemExit(f"устройства с номером {index} нет (всего {len(found)})")

    device = found[index]
    lock_take(force)                      # два окна инструмента одновременно — источник «прыгающих» каналов

    if kill_others:
        closed = close_other_tools(port)
        for pid, done in closed:
            print(f"   закрываю чужое окно инструмента: PID {pid} — "
                  + ("закрыто" if done else "не получилось, закройте его вручную (Ctrl+C)"))
        if not closed:
            print("   чужих окон инструмента не видно — каналы веду я один")

    if detect:
        mapping, invert = detect_mapping(radio, device)
    if aux_only:
        # калибровка только тумблеров и крутилок (раскладка стиков берётся сохранённая)
        found_aux = detect_aux(radio, device, mapping)
        save_map(mapping, invert, device["name"], aux=found_aux)
        print(f"\nраскладка AUX сохранена в {MAP_FILE}: "
              + (format_aux(found_aux) if found_aux else "ничего не найдено"))
        if not found_aux:
            print("   если тумблеры есть, но их не видно: назначьте их на каналы в микшере пульта")
            print("   (MODEL SETUP → MIXES, обычно CH5…CH8) — по USB пульт отдаёт только их.")
        return 0
    if detect and not no_aux:
        found_aux = detect_aux(radio, device, mapping)
        save_map(mapping, invert, device["name"], aux=found_aux)
        aux_map = found_aux
    if aux_map is None:
        aux_map = load_aux()
    aux_map = list(aux_map or [])
    names_invert = invert_names(invert)
    axis_names = tuple(getattr(radio, "AXIS_NAMES", ()) or ())

    try:
        rest_axes, _b, _h = radio.read(device)
        notes = layout_warnings(mapping, rest_axes, axis_names)
    except RuntimeError:
        rest_axes, notes = [], []
    source = my_source(device["name"])
    print(build_note())
    before = api(port, {"control": "physical", "rc_map": {**mapping, "invert": names_invert,
                                                          "device": device["name"]},
                        "layout_note": " ".join(notes),
                        "aux_map": [format_aux([item]) for item in aux_map],
                        "source": source})
    started_note = foreign_note(before)
    if started_note:
        print(started_note)
    print(f"веду каналы пультом [{index}] {device['name']} — Ctrl+C для выхода")
    print("   управление у физической аппаратуры: ползунки страницы /tx отключены")
    print("   раскладка осей:", format_map(mapping, names_invert))
    if aux_map:
        print("   AUX-каналы:", format_aux(aux_map))
    else:
        print("   AUX-каналы: тумблеры и крутилки не откалиброваны — "
              "python tools\\tx.py joystick --detect-aux")
    if rest_axes:
        print("   оси пульта в покое (по ним видно, та ли раскладка):")
        for line in describe_axes(rest_axes, mapping, axis_names, names_invert):
            print(line)
        print("   каналы:", sticks_table(mapping, rest_axes, names_invert, axis_names))
    for note in notes:
        print("   [!] " + note)
    print("   двигайте стики — на вкладке «Приёмник» поедут полосы, коптер наклонится.")
    print("   если стики двигают не свои каналы: Ctrl+C, затем python tools\\tx.py joystick --detect")
    step = 1.0 / max(rate, 1)
    last_log = 0.0
    refused = False
    warned_layout = ""
    now_layout_check = 0.0
    last_beat = last_check = time.time()
    warned_mismatch = warned_multi = 0.0
    stuck: dict = {}
    warned = set()
    try:
        while True:
            axes_values, buttons, hats = radio.read(device)

            # ось «стика», застывшая на упоре больше 2 с, — скорее всего это тумблер, а не стик.
            # Газ не проверяем: у него трещотка, и ручка как раз стоит на упоре — это норма.
            for name, axis in mapping.items():
                if name == "throttle":
                    stuck.pop(name, None)
                    continue
                if axis < len(axes_values) and abs(abs(axes_values[axis]) - 1.0) < 0.02:
                    stuck[name] = stuck.get(name, 0.0) + step
                    if stuck[name] > 2.0 and name not in warned:
                        warned.add(name)
                        print(f"   [!] ось {axis} ({LABELS[NAMES.index(name)]}) всё время на упоре — "
                              f"похоже, это тумблер, а не стик. Проверьте раскладку: --detect")
                else:
                    stuck[name] = 0.0

            # раскладка «не та» видна по покоящейся оси газа: напоминаем об этом каждые 5 с
            if now_layout_check <= time.time():
                now_layout_check = time.time() + 5.0
                notes = layout_warnings(mapping, axes_values, axis_names)
                if notes:
                    if notes[0] != warned_layout:
                        warned_layout = notes[0]
                        print("\n   [!] " + notes[0] + "\n")
                        try:                          # и на страницу /tx, чтобы было видно
                            api(port, {"layout_note": notes[0], "source": "tools/tx.py (раскладка)"})
                        except (ControlRefused, SystemExit):
                            pass
                    else:
                        print("   [!] раскладка по-прежнему не та: " + notes[0].splitlines()[0])

            payload = build_payload(mapping, axes_values, buttons, hats, aux_map)
            if names_invert:
                mirrored = _invert_values([payload.get(name, 1500) for name in NAMES],
                                          ",".join(names_invert))
                payload = {name: mirrored[NAMES.index(name)] for name in payload}
            try:
                state = api(port, {"channels": payload, "bind": True, "bind_seconds": 0,
                                   "physical": True, "source": source})
                if refused:
                    refused = False
                    print("   управление снова у физической аппаратуры — продолжаю")
            except ControlRefused:
                if not refused:
                    refused = True
                    print("\n   управление каналами сейчас у страницы /tx («Отключить (выдернуть USB)»).\n"
                          "   Данные пульта не принимаются: нажмите на странице «Подключить аппаратуру»,\n"
                          "   а затем «Связать (Bind)». Скрипт продолжает ждать.")
                time.sleep(0.5)
                continue
            now = time.time()
            if now - last_beat > 1.0:                     # отмечаемся, что инструмент жив
                last_beat = now
                lock_beat()
            if now - last_check > 2.0:
                # сверяем: в эмуляторе должно лежать ровно то, что мы послали. Если нет —
                # туда пишет кто-то ещё (чаще всего — второе, давно запущенное окно инструмента)
                last_check = now
                try:
                    live = api(port)
                except (ControlRefused, SystemExit):
                    live = None
                if live:
                    if live.get("multi_source") and now - warned_multi > 15.0:
                        warned_multi = now
                        print("\n" + multi_source_note(live.get("sources")) + "\n")
                        if kill_others:
                            # включён --kill-others: убираем чужие окна, не дожидаясь реакции человека
                            for pid, done in close_other_tools(port):
                                print(f"   закрываю чужое окно инструмента: PID {pid} — "
                                      + ("закрыто" if done
                                         else "не получилось, закройте его вручную (Ctrl+C)"))
                    mismatch = compare_with_fc(payload, live, names_invert)
                    if mismatch and now - warned_mismatch > 10.0:
                        warned_mismatch = now
                        print("\n" + mismatch_note(mismatch, live.get("source"),
                                                   live.get("updated_ago")) + "\n")
            if now - last_log > 3.0:
                last_log = now
                sticks = " ".join(f"{LABELS[NAMES.index(name)]}={payload[name]:>4}"
                                  for name in mapping if name in NAMES and name in payload)
                auxiliaries = " ".join(f"AUX{int(key[3:])}={value:>4}"
                                       for key, value in sorted(
                                           ((k, v) for k, v in payload.items() if k.startswith("aux")),
                                           key=lambda kv: int(kv[0][3:]))[:4])
                print(f"   {sticks}  {auxiliaries}  (биндинг: {state['bind']['text']})")
            time.sleep(step)
    except KeyboardInterrupt:
        print("\nостановлено, стики возвращены в центр")
        show(api(port, {"reset": True, "control": "virtual", "source": "tools/tx.py"}),
             "итог (управление снова у страницы /tx — нажмите там «Связать»):")
    finally:
        lock_release()


def main() -> int:
    parser = argparse.ArgumentParser(description="Виртуальная аппаратура bfemu (каналы MSP_RC)")
    parser.add_argument("commands", nargs="*",
                        help="roll=1700 … | center | demo | connect | disconnect | control | bind | "
                             "unbind | joystick | axes | wire | crsf | ports | crsf-selftest | "
                             "sbus-selftest | selftest")
    parser.add_argument("--http", type=int, default=8088, help="порт веб-монитора эмулятора (по умолчанию 8088)")
    parser.add_argument("--seconds", type=float, default=6.0, help="длительность демо, с")
    parser.add_argument("--index", type=int, default=0, help="номер USB-джойстика для режима joystick")
    parser.add_argument("--rate", type=float, default=50.0, help="частота опроса USB-пульта, Гц")
    parser.add_argument("--port", default=None, help="COM-порт аппаратуры для режима crsf, напр. COM5")
    parser.add_argument("--baud", type=int, default=None,
                        help="скорость порта приёмника (по умолчанию: CRSF 420000, SBUS 100000)")
    parser.add_argument("--protocol", default="auto", choices=("auto", "crsf", "sbus"),
                        help="протокол приёмника в COM-порту (по умолчанию auto — CRSF и SBUS)")
    parser.add_argument("--invert", default="", help="зеркалить каналы, напр. --invert pitch,yaw")
    parser.add_argument("--bind-seconds", type=float, default=2.0,
                        help="сколько «искать приёмник» при команде bind (по умолчанию 2 с)")
    parser.add_argument("--map", default="auto",
                        help="какие оси пульта каким каналам соответствуют: auto (сохранённая "
                             "калибровка или AETR) либо roll=0,pitch=1,throttle=2,yaw=3")
    parser.add_argument("--kill-others", action="store_true",
                        help="закрыть чужие окна инструмента, которые сейчас пишут каналы "
                             "(в них и «прыгают» каналы, и уезжает Рыск в AUX)")
    parser.add_argument("--force", action="store_true",
                        help="начать вести каналы даже если другой запуск инструмента ещё работает "
                             "(по умолчанию такой запуск останавливается с подсказкой)")
    parser.add_argument("--detect-aux", dest="detect_aux_only", action="store_true",
                        help="калибровка ТОЛЬКО тумблеров и крутилок: находит их оси и назначает "
                             "AUX-каналы (раскладка стиков берётся сохранённая или AETR)")
    parser.add_argument("--no-aux", dest="no_aux", action="store_true",
                        help="с --detect не калибровать тумблеры и крутилки (только стики)")
    parser.add_argument("--aux-map", default="auto",
                        help="какие оси каким AUX-каналам: auto (сохранённая калибровка), none "
                             "(сбросить) либо 3=switch2,4=switch3:4,5=knob:7 "
                             "(типы: switch2, switch3, knob)")
    parser.add_argument("--detect", action="store_true",
                        help="калибровка стиков: подсказывает по стику и сама находит ось "
                             "(раскладка сохраняется в tx_map.json)")
    args = parser.parse_args()

    tokens = args.commands
    if not tokens:
        state = api(args.http)
        show(state, "текущие каналы виртуальной аппаратуры:")
        print(f"   аппаратура: {'подключена' if state['connected'] else 'ОТКЛЮЧЕНА'}"
              f" | приёмник: {state['serial_port'] or 'не настроен (нужен Serial RX во вкладке «Порты»)'}"
              f" | активных каналов: {state['active_channels']}")
        return 0

    command = tokens[0].lower()
    if command == "center":
        show(api(args.http, {"reset": True, "source": "tools/tx.py"}), "стики в центр, тумблеры выключены:")
        return 0
    if command in ("connect", "plug"):
        state = api(args.http, {"connected": True, "source": "tools/tx.py"})
        show(state, "аппаратура подключена (USB), каналы в эфире:")
        return 0
    if command in ("control", "owner"):
        if len(tokens) < 2 or tokens[1].lower() not in ("virtual", "physical"):
            raise SystemExit("укажите, кому управление: python tools/tx.py control virtual|physical")
        who = tokens[1].lower()
        state = api(args.http, {"control": who, "source": "tools/tx.py"})
        show(state, ("управление у физической аппаратуры (кнопки страницы /tx отключены):"
                     if who == "physical" else
                     "управление снова у страницы /tx — «Связать», чтобы каналы пошли:"))
        return 0
    if command in ("bind", "pair"):
        state = api(args.http, {"bind": True, "bind_seconds": args.bind_seconds,
                                "source": "tools/tx.py"})
        show(state, f"биндинг: аппаратура ищет приёмник (до {args.bind_seconds:g} с)…"
                    if state["bind"]["state"] == "binding" else "биндинг: аппаратура связана:")
        return 0
    if command in ("unbind", "unpair"):
        state = api(args.http, {"bind": False, "source": "tools/tx.py"})
        show(state, "развязано: приёмник кадров не получает —")
        return 0
    if command in ("disconnect", "unplug"):
        state = api(args.http, {"connected": False, "source": "tools/tx.py"})
        show(state, "USB выдернут: прошивка отдаёт значения «Этапа 1» —")
        return 0
    if command == "demo":
        demo(args.http, args.seconds)
        return 0
    if command == "crsf-selftest":
        return crsf_selftest()
    if command == "sbus-selftest":
        return sbus_selftest()
    if command == "selftest":
        return selftest()
    if command in ("crsf", "wire"):
        if not args.port:
            raise SystemExit("укажите порт: python tools/tx.py wire --port COM5 [--protocol auto]")
        protocol = "crsf" if command == "crsf" else args.protocol
        baud = args.baud if args.baud else (420000 if protocol == "crsf" else 100000
                                            if protocol == "sbus" else 420000)
        wire(args.http, args.port, baud, protocol, args.rate, args.invert)
        return 0
    if command == "axes":
        return axes(args.index)
    if command == "ports":
        try:
            from serial.tools import list_ports
        except ImportError as exc:
            raise SystemExit("нужен пакет pyserial: python -m pip install pyserial") from exc
        found = list(list_ports.comports())
        if not found:
            print("последовательных портов не найдено")
        for item in found:
            print(f"   {item.device}: {item.description}")
        return 0
    if command == "joystick":
        mapping, saved_invert = parse_map(args.map)
        invert = args.invert or ",".join(saved_invert)
        aux_only = bool(args.detect_aux_only and not args.detect)
        aux_map = None if (aux_only or args.detect) else parse_aux_map(args.aux_map)
        joystick(args.http, args.index, args.rate, mapping, invert, args.detect, args.force,
                 args.kill_others, aux_map=aux_map, no_aux=args.no_aux, aux_only=aux_only)
        return 0

    payload = parse_assignments(tokens)
    payload["source"] = "tools/tx.py"
    show(api(args.http, payload), "каналы выставлены:")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
