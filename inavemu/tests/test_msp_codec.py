"""Кодек MSP: кадры v1/v2, джамбо, ресинхронизация, ответ-ошибка."""
from inavemu import msp


def test_v1_roundtrip():
    raw = msp.encode_v1(101, b"\x01\x02")
    parsed = list(msp.Parser().feed(raw))
    assert len(parsed) == 1
    assert parsed[0].version == 1
    assert parsed[0].code == 101
    assert parsed[0].payload == b"\x01\x02"


def test_v2_roundtrip():
    raw = msp.encode_v2(0x2000, b"\x00" * 22)
    parsed = list(msp.Parser().feed(raw))
    assert len(parsed) == 1
    assert parsed[0].version == 2
    assert parsed[0].code == 0x2000
    assert parsed[0].payload == b"\x00" * 22


def test_v2_big_code():
    raw = msp.encode_v2(0x1007, b"name\x00")
    parsed = list(msp.Parser().feed(raw))
    assert parsed[0].code == 0x1007


def test_jumbo_v1():
    payload = bytes(range(256)) * 2
    raw = msp.encode_v1(71, payload)
    parsed = list(msp.Parser().feed(raw))
    assert parsed[0].payload == payload


def test_resync_on_garbage():
    good = msp.encode_v2(1, b"")
    stream = b"\x00\x13garbage" + good + b"\xff"
    parsed = list(msp.Parser().feed(stream))
    assert len(parsed) == 1
    assert parsed[0].code == 1


def test_byte_by_byte():
    raw = msp.encode_v2(2, b"INAV")
    parser = msp.Parser()
    got = []
    for i in range(len(raw)):
        got.extend(parser.feed(raw[i:i + 1]))
    assert len(got) == 1
    assert got[0].payload == b"INAV"


def test_error_reply():
    req = msp.Request(version=2, code=0x9999, payload=b"")
    frame = msp.encode(req, b"", error=True)
    assert frame[2] == msp.ERROR_REPLY
    ok = msp.encode(req, b"\x2a")
    assert ok[2] == msp.FROM_FC


def test_crc_helpers():
    assert msp.crc_xor(b"\x01\x02\x03") == 0
    assert isinstance(msp.crc8_dvb_s2(b"abc"), int)
