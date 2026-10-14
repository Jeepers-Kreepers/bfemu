"""Чёрный ящик Betaflight: кодирование логов и «флеш»-накопитель.

Здесь ровно то, что прошивка пишет в dataflash, — теми же байтами и тем же порядком:

* кодировщики — порт ``blackbox_encoding.c``: UnsignedVB, SignedVB (зигзаг),
  TAG2_3S32, TAG8_4S16, TAG8_8SVB;
* кадры — как ``writeIntraframe`` / ``writeInterframe`` / ``writeSlowFrame`` /
  ``blackboxLogEvent`` в ``blackbox.c`` (I — ключевой кадр, P — дельта к нему,
  S — медленные поля режимов, E — события);
* заголовок — те же строки ``H …`` (включая ``H Field I/P/S …``), те же
  предсказатели и то же поведение P-кадров (время — вторая разница, PID —
  разница с прошлым кадром, гироскоп/моторы — с полусуммой двух прошлых кадров).

Проверка — официальным декодером ``betaflight/blackbox-tools`` (``blackbox_decode``):
0 негодных кадров, 0 пропущенных итераций; скорость потока сходится с ``H looptime``.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # только для подсказок типов: bb_scenarios импортирует blackbox, не наоборот
    from .bb_scenarios import Flight

# ------------------------------------------------------------------ параметры потока
LOOPTIME_US = 1000          # цикл PID: 1 кГц («H looptime:1000»)
LOG_EVERY = 2               # пишем каждый второй цикл → 500 Гц («H P interval:2»)
I_INTERVAL = 32             # кадр I каждые 32 цикла (≈32 мс, как blackboxIInterval)
P_RATIO = I_INTERVAL // LOG_EVERY       # «H P ratio:16»
SLOW_INTERVAL = I_INTERVAL * 8          # кадр S не реже раза в ~0,25 с
RATE_HZ = LOG_EVERY * 1_000_000 // LOOPTIME_US

MINTHROTTLE = 1070          # motorOutput низ (minthrottle)
MAXTHROTTLE = 2000
MOTOR_IDLE = 1070           # getMotorOutputLow() — предсказатель MINMOTOR
ACC_1G = 2048
MOTOR_POLES = 14
VBATREF = 1680              # 16,80 В в единицах 0,01 В — «H vbatref»


# ------------------------------------------------------------------ кодировщики
def unsigned_vb(value: int) -> bytes:
    """7 бит на байт, старший бит — «дальше есть ещё» (blackboxWriteUnsignedVB)."""
    value &= 0xFFFFFFFF
    out = bytearray()
    while value > 127:
        out.append((value & 0xFF) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def signed_vb(value: int) -> bytes:
    """Зигзаг + переменный байт (blackboxWriteSignedVB)."""
    return unsigned_vb((value << 1) ^ (value >> 31) if value < 0 else (value << 1))


def tag2_3s32(values) -> bytes:
    """Три числа: 2, 4, 6 или 32 бита каждое — порт blackboxWriteTag2_3S32.

    Раскладки: ``ss11 2233`` (2 бита), ``ss00 1111 2222 3333`` (4 бита),
    ``ss11 1111 0022 2222 0033 3333`` (6 бит), ``sstt tttt`` + числа по 1…4 байта.
    """
    BITS_2, BITS_4, BITS_6, BITS_32 = 0, 1, 2, 3
    values = [int(value) for value in values[:3]]
    selector = BITS_2
    for value in values:
        if value >= 32 or value < -32:
            selector = BITS_32
            break
        if value >= 8 or value < -8:
            selector = max(selector, BITS_6)
        elif value >= 2 or value < -2:
            selector = max(selector, BITS_4)
    out = bytearray()
    if selector == BITS_2:
        out.append((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03))
    elif selector == BITS_4:
        out.append((selector << 6) | (values[0] & 0x0F))
        out.append(((values[1] & 0x0F) << 4) | (values[2] & 0x0F))
    elif selector == BITS_6:
        out.append((selector << 6) | (values[0] & 0x3F))
        out.append(values[1] & 0xFF)
        out.append(values[2] & 0xFF)
    else:
        selector2 = 0
        for value in reversed(values):                 # первый элемент — в младших битах
            selector2 <<= 2
            if -128 <= value < 128:
                selector2 |= 0
            elif -32768 <= value < 32768:
                selector2 |= 1
            elif -8388608 <= value < 8388608:
                selector2 |= 2
            else:
                selector2 |= 3
        out.append((selector << 6) | selector2)
        for index, value in enumerate(values):
            width = ((selector2 >> (2 * index)) & 0x03) + 1
            raw = value & ((1 << (8 * width)) - 1)
            out.extend(raw.to_bytes(width, "little"))
    return bytes(out)


def tag8_4s16(values) -> bytes:
    """Четыре числа по 0/4/8/16 бит (blackboxWriteTag8_4S16)."""
    values = [int(v) for v in values]
    selector = 0
    for index, value in enumerate(values):
        if value == 0:
            mode = 0
        elif -8 <= value <= 7:
            mode = 1
        elif -128 <= value <= 127:
            mode = 2
        else:
            mode = 3
        selector |= mode << (index * 2)
    out = bytearray([selector])
    nibble = False
    buffer = 0
    for index, value in enumerate(values):
        mode = (selector >> (index * 2)) & 0x03
        if mode == 0:
            continue
        if mode == 1:
            if not nibble:
                buffer = (value & 0x0F) << 4
                nibble = True
            else:
                out.append(buffer | (value & 0x0F))
                nibble = False
        elif mode == 2:
            if not nibble:
                out.append(value & 0xFF)
            else:
                out.append(buffer | ((value >> 4) & 0x0F))
                buffer = (value & 0x0F) << 4
        else:
            if not nibble:
                out.append((value >> 8) & 0xFF)
                out.append(value & 0xFF)
            else:
                out.append(buffer | ((value >> 12) & 0x0F))
                out.append((value >> 4) & 0xFF)
                buffer = (value & 0x0F) << 4
    if nibble:
        out.append(buffer)
    return bytes(out)


def tag8_8svb(values) -> bytes:
    """До восьми чисел: битовая карта «не ноль» + знаковые VB (blackboxWriteTag8_8SVB)."""
    values = [int(v) for v in values]
    if len(values) == 1:
        return signed_vb(values[0])
    header = 0
    for index, value in enumerate(values):
        if value != 0:
            header |= 1 << index
    out = bytearray([header])
    for value in values:
        if value != 0:
            out.extend(signed_vb(value))
    return bytes(out)


def _div2(value: int) -> int:
    """Деление на 2 с усечением к нулю — как целочисленное деление в C."""
    return value // 2 if value >= 0 else -((-value) // 2)


# ------------------------------------------------------------------ таблица полей
PRED_0, PRED_PREVIOUS, PRED_STRAIGHT_LINE, PRED_AVERAGE_2 = 0, 1, 2, 3
PRED_MINTHROTTLE, PRED_MOTOR_0, PRED_INC = 4, 5, 6
PRED_VBATREF, PRED_MINMOTOR = 9, 11

ENC_SIGNED_VB, ENC_UNSIGNED_VB, ENC_NEG_14BIT = 0, 1, 3
ENC_TAG8_8SVB, ENC_TAG2_3S32, ENC_TAG8_4S16, ENC_NULL = 6, 7, 8, 9


def _field(name, key, index, signed, i_pred, i_enc, p_pred, p_enc):
    return {"name": name, "key": key, "index": index, "signed": signed,
            "i_pred": i_pred, "i_enc": i_enc, "p_pred": p_pred, "p_enc": p_enc}


def _main_fields():
    """Поля кадров I/P — порядок и правила как в blackboxMainFields (без GPS/магнитометра).

    ``signed`` — то, что уходит в заголовок («H Field I signed»), как колонка signed
    в прошивке. Тип VB в кадре I выбирается кодировкой (i_enc), как в writeIntraframe.
    """
    fields = [
        _field("loopIteration", "loop_iteration", None, False, PRED_0, ENC_UNSIGNED_VB, PRED_INC, ENC_NULL),
        _field("time", "time", None, False, PRED_0, ENC_UNSIGNED_VB, PRED_STRAIGHT_LINE, ENC_SIGNED_VB),
    ]
    for key, i_enc, p_enc, signed in (("axis_p", ENC_SIGNED_VB, ENC_SIGNED_VB, True),
                                      ("axis_i", ENC_SIGNED_VB, ENC_TAG2_3S32, True),
                                      ("axis_d", ENC_SIGNED_VB, ENC_SIGNED_VB, True),
                                      ("axis_f", ENC_SIGNED_VB, ENC_SIGNED_VB, True)):
        prefix = {"axis_p": "axisP", "axis_i": "axisI", "axis_d": "axisD", "axis_f": "axisF"}[key]
        for axis in range(3):
            fields.append(_field(f"{prefix}[{axis}]", key, axis, signed, PRED_0, i_enc, PRED_PREVIOUS, p_enc))
    for axis in range(4):
        fields.append(_field(f"rcCommand[{axis}]", "rc_command", axis, axis != 3, PRED_0,
                             ENC_UNSIGNED_VB if axis == 3 else ENC_SIGNED_VB,
                             PRED_PREVIOUS, ENC_TAG8_4S16))
    fields.append(_field("vbatLatest", "vbat", None, False, PRED_VBATREF, ENC_NEG_14BIT,
                         PRED_PREVIOUS, ENC_TAG8_8SVB))
    fields.append(_field("amperageLatest", "amperage", None, True, PRED_0, ENC_SIGNED_VB,
                         PRED_PREVIOUS, ENC_TAG8_8SVB))
    fields.append(_field("rssi", "rssi", None, False, PRED_0, ENC_UNSIGNED_VB,
                         PRED_PREVIOUS, ENC_TAG8_8SVB))
    for key, prefix in (("gyro_adc", "gyroADC"), ("gyro_unfilt", "gyroUnfilt"), ("acc_smooth", "accSmooth")):
        for axis in range(3):
            fields.append(_field(f"{prefix}[{axis}]", key, axis, True, PRED_0, ENC_SIGNED_VB,
                                 PRED_AVERAGE_2, ENC_SIGNED_VB))
    for axis in range(4):                      # motor[0] считается от minthrottle, остальные — от motor[0]
        fields.append(_field(f"motor[{axis}]", "motor", axis, False,
                             PRED_MINMOTOR if axis == 0 else PRED_MOTOR_0,
                             ENC_UNSIGNED_VB if axis == 0 else ENC_SIGNED_VB,
                             PRED_AVERAGE_2, ENC_SIGNED_VB))
    for axis in range(4):
        fields.append(_field(f"eRPM[{axis}]", "erpm", axis, False, PRED_0, ENC_UNSIGNED_VB,
                             PRED_PREVIOUS, ENC_SIGNED_VB))
    return fields


MAIN_FIELDS = _main_fields()

SLOW_FIELDS = [
    _field("flightModeFlags", "flight_mode_flags", None, False, PRED_0, ENC_UNSIGNED_VB, 0, 0),
    _field("stateFlags", "state_flags", None, False, PRED_0, ENC_UNSIGNED_VB, 0, 0),
    _field("failsafePhase", "failsafe_phase", None, False, PRED_0, ENC_TAG2_3S32, 0, 0),
    _field("rxSignalReceived", "rx_signal_received", None, False, PRED_0, ENC_TAG2_3S32, 0, 0),
    _field("rxFlightChannelsValid", "rx_channels_valid", None, False, PRED_0, ENC_TAG2_3S32, 0, 0),
]

EVENT_FLIGHTMODE = 30
EVENT_LOG_END = 255
ARM_FLAG = 1                                   # BOXARM — младший бит rcModeActivationMask


def _value(sample, field):
    key, index = field["key"], field["index"]
    return sample[key][index] if index is not None else sample[key]


# ------------------------------------------------------------------ кадры
def _write_intraframe(sample, sysinfo) -> bytes:
    """Кадр I: полные значения. Порт writeIntraframe() (без GPS/магнитометра/отладки)."""
    out = bytearray(b"I")
    for field in MAIN_FIELDS:
        if field["i_enc"] == ENC_NULL:
            continue
        value = _value(sample, field)
        if field["i_enc"] == ENC_NEG_14BIT:                       # напряжение: от vbatref вниз
            out.extend(unsigned_vb((sysinfo["vbatref"] - value) & 0x3FFF))
            continue
        if field["i_pred"] == PRED_MINMOTOR:
            value -= sysinfo["minthrottle"]
        elif field["i_pred"] == PRED_MOTOR_0:
            value -= sample["motor"][0]
        out.extend(signed_vb(value) if field["i_enc"] == ENC_SIGNED_VB else unsigned_vb(value))
    return bytes(out)


def _write_interframe(sample, previous, previous2, sysinfo) -> bytes:
    """Кадр P: дельты к предсказаниям. Порт writeInterframe().

    Время пишем ОТДЕЛЬНО (вторая разница, предсказатель STRAIGHT_LINE) — в общем цикле
    поле time пропускаем, иначе оно уйдёт в поток дважды и декодер разъедется.
    """
    out = bytearray(b"P")
    out.extend(signed_vb(sample["time"] - 2 * previous["time"] + previous2["time"]))

    def delta(field, average=False):
        current = _value(sample, field)
        left = _value(previous, field)
        if average:
            return current - _div2(left + _value(previous2, field))
        return current - left

    position = 0
    while position < len(MAIN_FIELDS):
        field = MAIN_FIELDS[position]
        encoding = field["p_enc"]
        if encoding == ENC_NULL or field["key"] in ("time", "loop_iteration"):
            position += 1
            continue
        if encoding in (ENC_TAG2_3S32, ENC_TAG8_4S16, ENC_TAG8_8SVB):
            group_size = {ENC_TAG2_3S32: 3, ENC_TAG8_4S16: 4, ENC_TAG8_8SVB: 8}[encoding]
            group = []
            while (position + len(group) < len(MAIN_FIELDS)
                   and MAIN_FIELDS[position + len(group)]["p_enc"] == encoding
                   and len(group) < group_size):
                group.append(MAIN_FIELDS[position + len(group)])
            values = [delta(item) for item in group]
            if encoding == ENC_TAG2_3S32:
                out.extend(tag2_3s32(values))
            elif encoding == ENC_TAG8_4S16:
                out.extend(tag8_4s16(values))
            else:
                out.extend(tag8_8svb(values))
            position += len(group)
            continue
        out.extend(signed_vb(delta(field, average=(field["p_pred"] == PRED_AVERAGE_2))))
        position += 1
    return bytes(out)


def _write_slow_frame(sample) -> bytes:
    """Кадр S: медленные поля режимов (writeSlowFrame)."""
    out = bytearray(b"S")
    out.extend(unsigned_vb(sample["flight_mode_flags"]))
    out.extend(unsigned_vb(sample["state_flags"]))
    out.extend(tag2_3s32([sample["failsafe_phase"], sample["rx_signal_received"],
                          sample["rx_channels_valid"]]))
    return bytes(out)


def _event_frame(event: int, payload: bytes = b"") -> bytes:
    """Кадр E: событие (blackboxLogEvent)."""
    return bytes([ord("E"), event & 0xFF]) + payload


def _header(flight, sysinfo) -> bytes:
    """Заголовок лога — те же «H …» строки, что пишет прошивка перед первым кадром."""
    names = ",".join(field["name"] for field in MAIN_FIELDS)
    signed = ",".join("1" if field["signed"] else "0" for field in MAIN_FIELDS)
    i_pred = ",".join(str(field["i_pred"]) for field in MAIN_FIELDS)
    i_enc = ",".join(str(field["i_enc"]) for field in MAIN_FIELDS)
    p_pred = ",".join(str(field["p_pred"]) for field in MAIN_FIELDS)
    p_enc = ",".join(str(field["p_enc"]) for field in MAIN_FIELDS)
    slow_names = ",".join(field["name"] for field in SLOW_FIELDS)
    slow_signed = ",".join("1" if field["signed"] else "0" for field in SLOW_FIELDS)
    slow_pred = ",".join(str(field["i_pred"]) for field in SLOW_FIELDS)
    slow_enc = ",".join(str(field["i_enc"]) for field in SLOW_FIELDS)
    lines = [
        ("Product", "Blackbox flight data recorder by Nicholas Sherlock"),
        ("Data version", "2"),
        ("Firmware type", "Betaflight"),
        # Формат как в blackbox.c: «Betaflight <версия> (<ревизия>) <плата>». Без слова
        # «Betaflight» в начале декодер не узнаёт версию и врёт с масштабом напряжения.
        ("Firmware revision", f"Betaflight {sysinfo['firmware']} ({sysinfo['build']}) {sysinfo['board']}"),
        ("Firmware date", sysinfo["build_date"]),
        ("Board information", f"{sysinfo['manufacturer']} {sysinfo['board']}"),
        ("Log start datetime", flight.start.strftime("%Y-%m-%dT%H:%M:%S")),
        ("Craft name", flight.craft_name),
        ("I interval", str(sysinfo.get("i_interval", I_INTERVAL))),
        ("P interval", str(sysinfo.get("log_every", LOG_EVERY))),
        ("P ratio", str(sysinfo.get("p_ratio", P_RATIO))),
        ("minthrottle", str(sysinfo["minthrottle"])),
        ("maxthrottle", str(sysinfo["maxthrottle"])),
        ("gyro_scale", "0x3f800000"),
        ("motorOutput", f"{sysinfo['minthrottle']},{sysinfo['maxthrottle']}"),
        ("acc_1G", str(sysinfo["acc_1g"])),
        ("vbat_scale", str(sysinfo["vbat_scale"])),
        ("vbatcellvoltage", "330,350,430"),
        ("vbatref", str(sysinfo["vbatref"])),
        ("currentSensor", f"{sysinfo.get('current_offset', 0)},{sysinfo.get('current_scale', 400)}"),
        ("looptime", str(LOOPTIME_US)),
        ("gyro_sync_denom", "1"),
        ("pid_process_denom", "1"),
        ("motor_poles", str(sysinfo.get("motor_poles", MOTOR_POLES))),
        ("rollPID", sysinfo.get("pid_roll", "45,80,40")),
        ("pitchPID", sysinfo.get("pid_pitch", "47,84,46")),
        ("yawPID", sysinfo.get("pid_yaw", "45,80,0")),
        ("rc_rates", sysinfo.get("rc_rates", "70,70,70")),
        ("deadband", sysinfo.get("deadband", "0")),
        ("yaw_deadband", sysinfo.get("yaw_deadband", "0")),
        ("Field I name", names),
        ("Field I signed", signed),
        ("Field I predictor", i_pred),
        ("Field I encoding", i_enc),
        ("Field P predictor", p_pred),
        ("Field P encoding", p_enc),
        ("Field S name", slow_names),
        ("Field S signed", slow_signed),
        ("Field S predictor", slow_pred),
        ("Field S encoding", slow_enc),
    ]
    return "".join(f"H {key}:{value}\n" for key, value in lines).encode("latin-1")


def encode_flight(flight: Flight, sysinfo: dict) -> bytes:
    """Собирает один лог: заголовок, кадры I/P/S/E и «End of log».

    Модель считает цикл PID с шагом ``LOOPTIME_US`` (1 мс), а в «флеш» пишется каждый
    ``LOG_EVERY``-й цикл — отсюда «H P interval:2» и шаг времени 2 мс (500 Гц), как у
    настоящего борта с sample_rate 1/2. События и смена режимов, случившиеся между
    записанными кадрами, прилипают к ближайшему следующему кадру.
    """
    log_every = int(sysinfo.get("log_every", LOG_EVERY) or 1)
    i_interval = int(sysinfo.get("i_interval", I_INTERVAL) or LOG_EVERY)
    slow_every = i_interval * 8
    out = bytearray(_header(flight, sysinfo))
    samples = flight.samples
    previous = previous2 = None
    pending_events: list = []
    slow_changed = False
    logged = 0
    out.extend(_write_slow_frame(samples[0]))
    for sample in samples:
        pending_events.extend(sample.get("events", ()))
        slow_changed = slow_changed or bool(sample.get("slow_changed"))
        if sample["loop_iteration"] % log_every != 0:
            continue
        is_intra = previous is None or sample["loop_iteration"] % i_interval == 0
        if is_intra:
            out.extend(_write_intraframe(sample, sysinfo))
        else:
            out.extend(_write_interframe(sample, previous, previous2, sysinfo))
        if logged == 0 or slow_changed or logged % slow_every == 0:
            out.extend(_write_slow_frame(sample))
        for event, payload in pending_events:
            out.extend(_event_frame(event, payload))
        pending_events.clear()
        slow_changed = False
        logged += 1
        if is_intra:
            previous2 = previous = sample
        else:
            previous2, previous = previous, sample
    for event, payload in pending_events:
        out.extend(_event_frame(event, payload))
    out.extend(_event_frame(EVENT_LOG_END, b"End of log\x00"))
    return bytes(out)


def flight_mode_event(flags: int, last_flags: int) -> tuple:
    """Событие «сменились режимы полёта» (FLIGHTMODE=30) — как пишет blackboxLogEvent."""
    return (EVENT_FLIGHTMODE, unsigned_vb(flags) + unsigned_vb(last_flags))


# ------------------------------------------------------------------ «флеш» (dataflash)
class BlackboxFlash:
    """Кускок «флеш»-памяти, куда прошивка пишет логи (W25Q128: 16 МиБ, сектор 4 КиБ)."""

    CAPACITY = 16 * 1024 * 1024
    SECTOR_SIZE = 4096

    def __init__(self, data: bytes = b""):
        self.data = bytes(data)

    @property
    def sectors(self) -> int:
        return self.CAPACITY // self.SECTOR_SIZE

    @property
    def size(self) -> int:
        return self.CAPACITY

    @property
    def used(self) -> int:
        return len(self.data)

    @property
    def flags(self) -> int:
        """MSP_DATAFLASH_SUMMARY: бит0 — готов, бит1 — поддерживается."""
        return 0b11

    def erase(self) -> None:
        self.data = b""

    def read(self, address: int, length: int) -> bytes:
        if address >= len(self.data):
            return b""
        return self.data[address:address + length]

    def write(self, address: int, chunk: bytes) -> None:
        data = bytearray(self.data)
        if len(data) < address:
            data.extend(b"\xff" * (address - len(data)))
        data[address:address + len(chunk)] = chunk
        self.data = bytes(data)

    def save(self, path: Path) -> None:
        """Запись на диск через временный файл: половинчатый «флеш» на диск не попадёт."""
        path = Path(path)
        tmp = path.with_name(path.name + ".part")
        tmp.write_bytes(self.data)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: Path) -> BlackboxFlash:
        path = Path(path)
        return cls(path.read_bytes() if path.exists() else b"")


def scan_logs(data: bytes) -> list:
    """Разбор «флеша»: где начинается каждый лог и что о нём известно (для вкладки)."""
    marker = b"H Product:Blackbox flight data recorder"
    starts = []
    position = data.find(marker)
    while position >= 0:
        starts.append(position)
        position = data.find(marker, position + len(marker))
    logs = []
    for number, start in enumerate(starts, 1):
        end = starts[number] if number < len(starts) else len(data)
        chunk = data[start:end]
        logs.append({
            "index": number,
            "offset": start,
            "size": len(chunk),
            "start_datetime": _header_value(chunk, "Log start datetime"),
            "craft": _header_value(chunk, "Craft name"),
            "i_interval": _header_value(chunk, "I interval"),
            "p_interval": _header_value(chunk, "P interval"),
            "looptime": _header_value(chunk, "looptime"),
            "duration": _log_duration(chunk),
            "frames": _count_frames(chunk),
        })
    return logs


def _header_value(chunk: bytes, key: str) -> str:
    for line in chunk[:8192].decode("latin-1").splitlines():
        if line.startswith(f"H {key}:"):
            return line.split(":", 1)[1].strip()
    return ""


def _count_frames(chunk: bytes) -> int:
    """Оценка числа кадров: считаем маркеры кадров после заголовка."""
    end = chunk.find(b"H Field S encoding:")
    end = chunk.find(b"\n", end) + 1
    body = chunk[end:]
    return sum(1 for byte in body if byte in (ord("I"), ord("P")))


def _log_duration(chunk: bytes) -> float:
    """Длительность лога: по заголовку о интервалах и числу кадров."""
    try:
        looptime = int(_header_value(chunk, "looptime") or LOOPTIME_US)
        p_interval = int(_header_value(chunk, "P interval") or LOG_EVERY)
    except ValueError:
        looptime, p_interval = LOOPTIME_US, LOG_EVERY
    frames = _count_frames(chunk)
    if not frames:
        return 0.0
    return frames * p_interval * looptime / 1_000_000
