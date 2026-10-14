"""
Кодек протокола MSP (MultiWii Serial Protocol) — v1 и v2.

Кадры (то, что Configurator пишет в порт и получает обратно):

  v1: '$' 'M' '<' <len:u8> <code:u8> <payload...> <crc:u8>          crc = XOR(len, code, payload)
  v2: '$' 'X' '<' <flag:u8> <code:u16le> <len:u16le> <payload...> <crc:u8>
                                              crc = CRC8-DVB-S2 по полям flag..payload

Прошивка отвечает тем же протоколом (v1 на v1-запрос, v2 на v2-запрос), поэтому
эмулятор обязан помнить, в каком протоколе пришёл запрос.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterator

BEGIN = 0x24  # '$'
PROTO_V1 = ord("M")
PROTO_V2 = ord("X")
TO_V1 = ord("<")      # запрос от Configurator к FC
ERROR_REPLY = ord("!")  # ответ-ошибка (как в msp_serial.c)
TO_V2 = ord("<")
FROM_FC = ord(">")

# CLI-поток поверх UART (используется, когда порт в режиме CLI, см. cli.py)
STX = 0x02  # начало вывода CLI
ETX = 0x03  # конец вывода CLI
LF = 0x0A
CR = 0x0D

_CRC8_DVB_S2_TABLE = None


def _crc8_dvb_s2_table():
    global _CRC8_DVB_S2_TABLE
    if _CRC8_DVB_S2_TABLE is None:
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
            table.append(crc)
        _CRC8_DVB_S2_TABLE = table
    return _CRC8_DVB_S2_TABLE


def crc8_dvb_s2(data: bytes, crc: int = 0) -> int:
    table = _crc8_dvb_s2_table()
    for b in data:
        crc = table[crc ^ b]
    return crc


def crc_xor(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


@dataclass
class Request:
    version: int  # 1 или 2
    code: int
    payload: bytes
    flag: int = 0


JUMBO_FRAME_SIZE_LIMIT = 255   # msp_serial.c: больше — «джамбо»-кадр с длиной u16


def encode_v1(code: int, payload: bytes = b"", direction: int = TO_V1) -> bytes:
    """Кадр MSP v1. Для ответов FC direction = FROM_FC ('>'), как в прошивке.

    Длинные ответы (от 255 байт — например, чтение «флеша» чёрного ящика) прошивка
    отправляет «джамбо»-кадром: в поле длины 255, сразу за кодом — настоящая длина u16
    (mspSerialEncode в msp_serial.c). Контрольная сумма покрывает и эти байты.
    """
    length = len(payload)
    if length >= JUMBO_FRAME_SIZE_LIMIT:
        body = bytes([JUMBO_FRAME_SIZE_LIMIT, code & 0xFF]) + length.to_bytes(2, "little") + bytes(payload)
    else:
        body = bytes([length & 0xFF, code & 0xFF]) + bytes(payload)
    return bytes([BEGIN, PROTO_V1, direction]) + body + bytes([crc_xor(body)])


def encode_v2(code: int, payload: bytes = b"", flag: int = 0, direction: int = FROM_FC) -> bytes:
    header = bytes([flag & 0xFF, code & 0xFF, (code >> 8) & 0xFF, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    crc = crc8_dvb_s2(header + bytes(payload))
    return bytes([BEGIN, PROTO_V2, direction]) + header + bytes(payload) + bytes([crc])


class MspParser:
    """Потоковый парсер запросов. Кормите feed(), получайте Request'ы."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)

    def __iter__(self) -> Iterator[Request]:
        while True:
            req = self._next()
            if req is None:
                return
            yield req

    def _next(self) -> Request | None:
        while True:
            req = self._parse_one()
            if req is Request or isinstance(req, Request):
                return req
            if req is None:
                return None
            # req == "retry": кадр отброшен, пробуем следующий

    def _parse_one(self):
        buf = self._buf
        # синхронизация по '$'
        while buf and buf[0] != BEGIN:
            del buf[0]
        if len(buf) < 3:
            return None
        proto = buf[1]
        if proto == PROTO_V1:
            if len(buf) < 5:
                return None
            length = buf[3]
            if length == JUMBO_FRAME_SIZE_LIMIT:
                # «Джамбо»-кадр: за кодом идёт настоящая длина u16
                if len(buf) < 7:
                    return None
                length = buf[5] | (buf[6] << 8)
                total = 7 + length + 1
                if len(buf) < total:
                    return None
                frame = bytes(buf[:total])
                code = frame[4]
                payload = frame[7 : 7 + length]
                crc = frame[total - 1]
                del buf[:total]
                if crc_xor(frame[3 : 7 + length]) != crc:
                    return "retry"
                return Request(version=1, code=code, payload=payload)
            total = 5 + length + 1
            if len(buf) < total:
                return None
            frame = bytes(buf[:total])
            code = frame[4]
            payload = frame[5 : 5 + length]
            crc = frame[total - 1]
            del buf[:total]
            if crc_xor(frame[3 : 5 + length]) != crc:
                return "retry"  # кадр битый — прошивка его молча отбрасывает
            return Request(version=1, code=code, payload=payload)
        if proto == PROTO_V2:
            if len(buf) < 8:
                return None
            flag = buf[3]
            code = buf[4] | (buf[5] << 8)
            length = buf[6] | (buf[7] << 8)
            total = 8 + length + 1
            if len(buf) < total:
                return None
            frame = bytes(buf[:total])
            payload = frame[8 : 8 + length]
            crc = frame[total - 1]
            del buf[:total]
            if crc8_dvb_s2(frame[3 : 8 + length]) != crc:
                return "retry"
            return Request(version=2, code=code, payload=payload, flag=flag)
        del buf[0]
        return "retry"


class Writer:
    """Помощник для сборки payload'ов ответов (little-endian, как в прошивке)."""

    def __init__(self):
        self.buf = bytearray()

    def u8(self, v):
        self.buf += bytes([int(v) & 0xFF])
        return self

    def i8(self, v):
        self.buf += (int(v) & 0xFF).to_bytes(1, "little")
        return self

    def u16(self, v):
        self.buf += (int(v) & 0xFFFF).to_bytes(2, "little")
        return self

    def i16(self, v):
        self.buf += (int(v) & 0xFFFF).to_bytes(2, "little", signed=False)
        return self

    def u32(self, v):
        self.buf += (int(v) & 0xFFFFFFFF).to_bytes(4, "little")
        return self

    def i32(self, v):
        self.buf += (int(v) & 0xFFFFFFFF).to_bytes(4, "little")
        return self

    def text(self, s: str):  # C-строка (с завершающим NUL)
        self.buf += s.encode("latin-1", "replace") + b"\x00"
        return self

    def ltext(self, s: str):
        """Строка «длина + символы» — так прошивка пишет имена в MSP_BOARD_INFO
        и MSP2_GET_TEXT (см. sbufWriteU8(dst, strlen(...)) в msp.c)."""
        data = s.encode("latin-1", "replace")[:255]
        self.buf += bytes([len(data)]) + data
        return self

    def raw(self, b: bytes):
        self.buf += b
        return self

    def bytes(self) -> bytes:
        return bytes(self.buf)


class Reader:
    """Чтение входящих payload'ов в стиле прошивки."""

    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def u8(self):
        v = self.data[self.pos] if self.pos < len(self.data) else 0
        self.pos += 1
        return v

    def i8(self):
        v = self.u8()
        return v - 256 if v > 127 else v

    def u16(self):
        lo = self.u8()
        hi = self.u8()
        return lo | (hi << 8)

    def i16(self):
        v = self.u16()
        return v - 65536 if v > 32767 else v

    def u32(self):
        v = self.u16() | (self.u16() << 16)
        return v

    def i32(self):
        v = self.u32()
        return v - 4294967296 if v > 2147483647 else v

    def rest(self) -> bytes:
        v = self.data[self.pos :]
        self.pos = len(self.data)
        return v

    @property
    def remaining(self) -> int:
        """Сколько байт запроса ещё не прочитано (sbufBytesRemaining в прошивке)."""
        return max(0, len(self.data) - self.pos)
