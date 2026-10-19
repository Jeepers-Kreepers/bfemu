"""
Кодек протокола MSP (MultiWii Serial Protocol) — v1 и v2.

Кадры (то, что INAV Configurator пишет в порт и получает обратно):

  v1: '$' 'M' '<' <len:u8> <code:u8> <payload...> <crc:u8>          crc = XOR(len, code, payload)
  v2: '$' 'X' '<' <flag:u8> <code:u16le> <len:u16le> <payload...> <crc:u8>
                                              crc = CRC8-DVB-S2 по полям flag..payload

Прошивка отвечает тем же протоколом (v1 на v1-запрос, v2 на v2-запрос). Ответ-ошибка
использует направление '!' вместо '>' (msp_serial.c). Длинные ответы (от 255 байт)
прошивка шлёт «джамбо»-кадром: в поле длины 255, сразу за кодом — настоящая длина u16.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterator

BEGIN = 0x24  # '$'
PROTO_V1 = ord("M")
PROTO_V2 = ord("X")
TO_FC = ord("<")      # запрос от Configurator к FC
FROM_FC = ord(">")    # ответ FC
ERROR_REPLY = ord("!")  # ответ-ошибка (как в msp_serial.c)

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
    direction: int = TO_FC  # '<' запрос, '>' ответ, '!' ответ-ошибка


JUMBO_FRAME_SIZE_LIMIT = 255


def encode_v1(code: int, payload: bytes = b"", direction: int = TO_FC) -> bytes:
    """Кадр MSP v1. Для ответов FC direction = FROM_FC ('>') или ERROR_REPLY ('!')."""
    length = len(payload)
    if length >= JUMBO_FRAME_SIZE_LIMIT:
        body = bytes([JUMBO_FRAME_SIZE_LIMIT, code]) + length.to_bytes(2, "little") + payload
    else:
        body = bytes([length, code]) + payload
    return bytes([BEGIN, PROTO_V1, direction]) + body + bytes([crc_xor(body)])


def encode_v2(code: int, payload: bytes = b"", direction: int = TO_FC, flag: int = 0) -> bytes:
    """Кадр MSP v2. Для ответов FC direction = FROM_FC ('>') или ERROR_REPLY ('!')."""
    head = bytes([flag]) + int(code).to_bytes(2, "little") + len(payload).to_bytes(2, "little")
    body = head + payload
    return bytes([BEGIN, PROTO_V2, direction]) + body + bytes([crc8_dvb_s2(body)])


def encode(req: Request, payload: bytes = b"", error: bool = False) -> bytes:
    """Ответ на запрос: тем же протоколом, что и запрос."""
    direction = ERROR_REPLY if error else FROM_FC
    if req.version == 2:
        return encode_v2(req.code, payload, direction, req.flag)
    return encode_v1(req.code, payload, direction)


class Parser:
    """Потоковый парсер входящих MSP-кадров с ресинхронизацией."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterator[Request]:
        self._buf.extend(data)
        while True:
            req = self._extract()
            if req is None:
                return
            yield req

    def _extract(self) -> Request | None:
        buf = self._buf
        # найти начало кадра (принимаем и запросы '<', и ответы '>' / '!')
        while True:
            if len(buf) < 3:
                return None
            if buf[0] != BEGIN:
                del buf[0]
                continue
            if buf[1] not in (PROTO_V1, PROTO_V2):
                del buf[0]
                continue
            if buf[2] not in (TO_FC, FROM_FC, ERROR_REPLY):
                del buf[0]
                continue
            break

        direction = buf[2]

        if buf[1] == PROTO_V1:
            if len(buf) < 5:
                return None
            length = buf[3]
            if length == JUMBO_FRAME_SIZE_LIMIT:
                if len(buf) < 7:
                    return None
                length = int.from_bytes(buf[5:7], "little")
                header = 7
                code = buf[4]
            else:
                header = 5
                code = buf[4]
            total = header + length + 1
            if len(buf) < total:
                return None
            body = bytes(buf[3:total - 1])
            crc = buf[total - 1]
            if crc_xor(body) != crc:
                del buf[0]
                return self._extract()
            payload = bytes(buf[header:total - 1])
            del buf[:total]
            return Request(version=1, code=code, payload=payload, direction=direction)

        # v2
        if len(buf) < 8:
            return None
        flag = buf[3]
        code = int.from_bytes(buf[4:6], "little")
        length = int.from_bytes(buf[6:8], "little")
        total = 8 + length + 1
        if len(buf) < total:
            return None
        body = bytes(buf[3:total - 1])
        crc = buf[total - 1]
        if crc8_dvb_s2(body) != crc:
            del buf[0]
            return self._extract()
        payload = bytes(buf[8:total - 1])
        del buf[:total]
        return Request(version=2, code=code, payload=payload, flag=flag,
                       direction=direction)
