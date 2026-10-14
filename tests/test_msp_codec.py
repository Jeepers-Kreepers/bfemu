"""Кодек MSP: кадры v1/v2, контрольные суммы, поведение потокового разбора.

Это тот слой, на котором ломается связь с Configurator: ошибка в CRC или в длине
проявляется как «FC не отвечает», и искать её по интерфейсу очень дорого.
"""
from __future__ import annotations

import pytest

from bfemu.msp import FROM_FC, MspParser, crc8_dvb_s2, crc_xor, encode_v1, encode_v2


def frame_v1(code: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), code]) + payload
    return b"$M<" + body + bytes([crc_xor(body)])


def frame_v2(code: int, payload: bytes = b"") -> bytes:
    body = bytes([0]) + code.to_bytes(2, "little") + len(payload).to_bytes(2, "little") + payload
    return b"$X<" + body + bytes([crc8_dvb_s2(body)])


def parse_all(data: bytes) -> list:
    parser = MspParser()
    parser.feed(data)
    return list(parser)


def test_v1_round_trip():
    requests = parse_all(frame_v1(101, b"\x01\x02"))
    assert len(requests) == 1
    assert (requests[0].version, requests[0].code, requests[0].payload) == (1, 101, b"\x01\x02")


def test_v2_round_trip():
    requests = parse_all(frame_v2(0x3012, b"version"))
    assert len(requests) == 1
    assert (requests[0].version, requests[0].code, requests[0].payload) == (2, 0x3012, b"version")


def test_jumbo_frame_v1():
    """Длина 255 в v1 означает «настоящая длина дальше, двумя байтами» (jumbo)."""
    payload = bytes(range(256)) * 2                      # 512 байт — больше, чем влезает в u8
    body = bytes([255, 7]) + len(payload).to_bytes(2, "little") + payload
    data = b"$M<" + body + bytes([crc_xor(body)])
    requests = parse_all(data)
    assert len(requests) == 1
    assert requests[0].code == 7
    assert requests[0].payload == payload


def test_resync_after_garbage():
    """Мусор в потоке не должен «съедать» следующий нормальный кадр."""
    requests = parse_all(b"\x00\xff\x13hello" + frame_v1(1))
    assert [r.code for r in requests] == [1]


def test_bad_crc_is_dropped_silently():
    """Прошивка молча отбрасывает кадр с битой CRC — и продолжает разбирать поток."""
    bad = bytearray(frame_v1(1))
    bad[-1] ^= 0xFF
    requests = parse_all(bytes(bad) + frame_v1(2))
    assert [r.code for r in requests] == [2]


def test_incomplete_frame_waits_for_rest():
    parser = MspParser()
    whole = frame_v1(101, b"\x07\x08")
    parser.feed(whole[:-1])
    assert list(parser) == []
    parser.feed(whole[-1:])
    assert [r.code for r in parser] == [101]


@pytest.mark.parametrize("code,payload", [(1, b""), (101, b"\x01" * 32), (255, bytes(range(64)))])
def test_encode_v1_is_parseable(code, payload):
    """Ответ FC, разобранный тем же парсером, должен дать исходные код и данные."""
    data = encode_v1(code, payload, direction=FROM_FC).replace(b"$M>", b"$M<", 1)
    requests = parse_all(data)
    assert [(r.code, r.payload) for r in requests] == [(code, payload)]


@pytest.mark.parametrize("code", [0x1001, 0x3011, 0x3012])
def test_encode_v2_is_parseable(code):
    data = encode_v2(code, b"abc", direction=FROM_FC).replace(b"$X>", b"$X<", 1)
    requests = parse_all(data)
    assert [(r.code, r.payload) for r in requests] == [(code, b"abc")]


def test_crc_values_match_firmware():
    """Контрольные константы: XOR — из mspSerialEncode(), CRC8 DVB-S2 — из crc8_dvb_s2()."""
    assert crc_xor(b"\x00\x01") == 1
    assert crc8_dvb_s2(b"") == 0
    assert crc8_dvb_s2(b"\x00") == 0
    assert crc8_dvb_s2(b"\x01") == 0xD5
