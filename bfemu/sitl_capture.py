"""Запись фактических MSP-ответов Betaflight SITL без модели полёта или расчёта PID.

Для работы требуется запущенная прошивка SITL. Данные датчиков и моторов берутся
только из её ответов MSP; внешняя среда должна самостоятельно подавать входы.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

from .msp import FROM_FC, MspParser, encode_v1

MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_PID_ADVANCED = 94
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_PID = 112
# Порядок и единицы полей — из MSP-обработчика Betaflight 4.5.4.
STREAM_COMMANDS = (MSP_RAW_IMU, MSP_MOTOR, MSP_RC, MSP_ATTITUDE, MSP_STATUS)
COMMAND_NAMES = {
    MSP_FC_VARIANT: "MSP_FC_VARIANT", MSP_FC_VERSION: "MSP_FC_VERSION", MSP_BOARD_INFO: "MSP_BOARD_INFO",
    MSP_PID_ADVANCED: "MSP_PID_ADVANCED", MSP_PID: "MSP_PID", MSP_STATUS: "MSP_STATUS",
    MSP_RAW_IMU: "MSP_RAW_IMU", MSP_MOTOR: "MSP_MOTOR", MSP_RC: "MSP_RC", MSP_ATTITUDE: "MSP_ATTITUDE",
}


class CaptureError(Exception):
    """Ответ SITL недоступен или не соответствует протоколу."""


class SitlMspClient:
    """Строго последовательные MSP-запросы к отдельному SITL на localhost."""

    def __init__(self, port=5761, timeout=3.0):
        self.connection = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.connection.settimeout(timeout)
        self.parser = MspParser()

    def close(self):
        self.connection.close()

    def request(self, code: int) -> bytes:
        self.connection.sendall(encode_v1(code))
        while True:
            chunk = self.connection.recv(4096)
            if not chunk:
                raise CaptureError("Соединение с SITL закрыто до получения ответа MSP.")
            self.parser.feed(chunk)
            for reply in self.parser:
                if reply.code != code or reply.direction != FROM_FC:
                    raise CaptureError(f"Неожиданный ответ MSP при запросе {code}: код {reply.code}, направление {reply.direction}.")
                return reply.payload

    def identify(self) -> dict:
        variant = self.request(MSP_FC_VARIANT)
        version = self.request(MSP_FC_VERSION)
        board = self.request(MSP_BOARD_INFO)
        if variant != b"BTFL" or version != bytes((4, 5, 4)) or board[:4] != b"SITL":
            raise CaptureError("Ответивший контроллер не сообщает идентификатор Betaflight 4.5.4 / SITL; запись отменена.")
        return {"variant": "BTFL", "version": "4.5.4", "board": "SITL"}


def decode_payload(code: int, payload: bytes) -> dict:
    """Расшифровать только документированные поля, сохраняя исходный ответ отдельно."""
    if code == MSP_RAW_IMU:
        if len(payload) != 18:
            raise CaptureError(f"MSP_RAW_IMU: ожидалось 18 байт, получено {len(payload)}.")
        values = struct.unpack("<9h", payload)
        # В SITL 4.5.4 virtual gyro имеет scale=2000/32768; MSP_RAW_IMU
        # передаёт gyroRateDps(), делящую фильтрованный сигнал на этот scale.
        gyro = list(values[3:6])
        return {"acc_adc": list(values[:3]), "gyro_raw": gyro,
                "gyro_dps": [value * (2000.0 / 32768.0) for value in gyro], "mag_adc": list(values[6:])}
    if code == MSP_MOTOR:
        if len(payload) != 16:
            raise CaptureError(f"MSP_MOTOR: ожидалось 16 байт, получено {len(payload)}.")
        return {"motor_output_msp": list(struct.unpack("<8H", payload))}
    if code == MSP_RC:
        if len(payload) % 2 or len(payload) > 36:
            raise CaptureError(f"MSP_RC: некорректная длина ответа {len(payload)}.")
        return {"channels_msp": list(struct.unpack(f"<{len(payload) // 2}H", payload))}
    if code == MSP_ATTITUDE:
        if len(payload) != 6:
            raise CaptureError(f"MSP_ATTITUDE: ожидалось 6 байт, получено {len(payload)}.")
        roll, pitch, yaw = struct.unpack("<hhh", payload)
        return {"roll_decideg": roll, "pitch_decideg": pitch, "yaw_deg": yaw}
    # MSP_STATUS и снимки PID также сохраняются целиком, без выдуманной интерпретации.
    return {}


def _record(stream, record):
    stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
    stream.flush()


def validate_timing(duration: float, interval: float) -> None:
    if not math.isfinite(duration) or not 0 < duration <= 3600:
        raise CaptureError("Продолжительность должна быть больше 0 и не больше 3600 секунд.")
    if not math.isfinite(interval) or not 0.02 <= interval <= 10:
        raise CaptureError("Интервал опроса должен быть от 0,02 до 10 секунд.")


def capture(client: SitlMspClient, stream, duration: float, interval: float, *, identity=None) -> int:
    validate_timing(duration, interval)
    identity = client.identify() if identity is None else identity
    _record(stream, {"type": "identity", **identity, "time_basis": "host_monotonic_after_response",
                     "note": "MSP-показания снимаются последовательно, не одновременно; PID-составляющие MSP не содержит."})
    start = time.monotonic()
    count = 0

    def collect(code):
        nonlocal count
        payload = client.request(code)
        elapsed = round(time.monotonic() - start, 6)
        _record(stream, {"type": "msp", "received_s": elapsed, "command": COMMAND_NAMES[code],
                         "payload_hex": payload.hex(), "decoded": decode_payload(code, payload)})
        count += 1

    # Снимок профиля сделан до потока: он не подтверждает неизменность настроек позже.
    for code in (MSP_PID, MSP_PID_ADVANCED):
        collect(code)
    end = start + duration
    next_sample = time.monotonic()
    while time.monotonic() < end:
        for code in STREAM_COMMANDS:
            if time.monotonic() >= end:
                break
            collect(code)
        next_sample += interval
        remaining = min(next_sample, end) - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        elif next_sample < time.monotonic() - interval:
            # Не выдаём отставание последовательных MSP-запросов за равномерные отсчёты.
            next_sample = time.monotonic()
    return count


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="новый файл JSON Lines; существующий не перезаписывается")
    parser.add_argument("--duration", type=float, default=10, help="длительность записи, секунд")
    parser.add_argument("--interval", type=float, default=0.1, help="интервал начала циклов MSP-опроса, секунд")
    parser.add_argument("--port", type=int, default=5761, help="локальный TCP-порт MSP SITL")
    args = parser.parse_args(argv)
    client = None
    try:
        if not 1 <= args.port <= 65535:
            raise CaptureError("Недопустимый TCP-порт.")
        validate_timing(args.duration, args.interval)
        # Существующий файл не перезаписывается, в том числе при гонке после этой проверки.
        if args.output.exists():
            raise CaptureError(f"Файл уже существует: {args.output}")
        client = SitlMspClient(port=args.port)
        identity = client.identify()
        with args.output.open("x", encoding="utf-8") as stream:
            count = capture(client, stream, args.duration, args.interval, identity=identity)
        print(f"Записано MSP-ответов: {count}. Файл: {args.output}")
        return 0
    except (OSError, CaptureError) as exc:
        print(f"SITL capture: {exc}", file=sys.stderr)
        return 1
    finally:
        if client:
            client.close()


if __name__ == "__main__":
    sys.exit(main())
