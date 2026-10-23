"""Пульт: управление каналами RC (слайдеры/кнопки + аппаратура по USB/Gamepad).

Проверяет POST/GET /api/rc, запись в rc_channels (видна через MSP_RC — полосы
вкладки Receiver в Configurator), валидацию и страницу /rc.
"""
import json
import struct
import urllib.error
import urllib.request

import pytest

from inavemu import msp
from inavemu.fc import InavFC, MSPError
from inavemu.server import FCServer, MonitorServer


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


def msp_rc(fc):
    raw = ask(fc, 105)  # MSP_RC
    return list(struct.unpack_from("<16H", raw))


@pytest.fixture()
def web(fc):
    srv = FCServer(fc, host="127.0.0.1", port=0)
    monitor = MonitorServer(srv, host="127.0.0.1", port=0)
    monitor.start()
    base = f"http://127.0.0.1:{monitor.port}"
    yield base
    monitor.stop()


def _get(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()


def _post(url, payload):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode())


# ---------------- модель: set_rc_channels ----------------

def test_set_rc_channels_writes(fc):
    vals = [1000, 1200, 1300, 1400, 1600, 1700, 1800, 1900]
    assert fc.set_rc_channels(vals) == 8
    assert fc.rc_channels[:8] == vals
    assert fc.rc_channels[8:] == [1500] * 8, "каналы 9..16 не трогаем"


def test_set_rc_channels_throttle_default_low(fc):
    assert fc.rc_channels[2] == 1000, "CH3 (Throttle) — газ вниз по умолчанию"


def test_set_rc_channels_partial(fc):
    fc.set_rc_channels([1111, 2222, 999])
    assert fc.rc_channels[:4] == [1111, 2222, 999, 1500]


def test_set_rc_channels_rejects_bad(fc):
    for bad in ([], [1500] * 17, [True], [1.5], ["1500"], [1500.0],
                [1500, None], [299], [2301], [-100]):
        with pytest.raises(MSPError):
            fc.set_rc_channels(bad)


def test_set_rc_channels_visible_via_msp_rc(fc):
    vals = [1234, 1456, 1678, 1789, 1890, 1901, 2002, 2100]
    fc.set_rc_channels(vals)
    chans = msp_rc(fc)
    assert chans[:8] == vals
    assert chans[2] == 1678, "Configurator видит газ в полосе CH3"


# ---------------- HTTP: /api/rc ----------------

def test_api_rc_get(web, fc):
    code, body = _get(web + "/api/rc")
    assert code == 200
    js = json.loads(body)
    assert len(js["channels"]) == 16
    assert js["channels"][2] == 1000


def test_api_rc_post_roundtrip(web, fc):
    vals = [1000, 1200, 1300, 1400, 1600, 1700, 1800, 1900]
    code, js = _post(web + "/api/rc", {"channels": vals})
    assert code == 200
    assert js["channels"][:8] == vals
    code, body = _get(web + "/api/rc")
    assert json.loads(body)["channels"][:8] == vals
    assert msp_rc(fc)[:8] == vals


def test_api_rc_post_16_channels(web, fc):
    vals = [875 + i * 25 for i in range(16)]
    code, js = _post(web + "/api/rc", {"channels": vals})
    assert code == 200
    assert js["channels"] == vals


def test_api_rc_post_errors(web):
    for payload in ({}, {"channels": 5}, {"channels": []}, {"channels": [1500] * 17},
                    {"channels": [True] * 8}, {"channels": ["x"] * 8},
                    {"channels": [299] * 8}, {"channels": [2400] * 8},
                    {"channels": [1500, 1500, 3.5, 1500, 1500, 1500, 1500, 1500]}):
        code, js = _post(web + "/api/rc", payload)
        assert code == 400, payload
        assert "error" in js


def test_api_rc_in_state(web):
    code, body = _get(web + "/api/state")
    assert code == 200
    state = json.loads(body)
    assert len(state["rc"]) == 16


# ---------------- страница /rc ----------------

def test_rc_page_renders(web):
    code, body = _get(web + "/rc")
    assert code == 200
    assert "Пульт" in body
    for token in ("sl0", "num7", "gps7", "gp_preset", "Gamepad",
                  "Radiomaster Zorro", "rc_neutral", "id=\"gp_on\""):
        assert token in body, token
    code, body = _get(web + "/")
    assert 'href="/rc"' in body, "«Пульт» есть в общей навигации"


def test_rc_page_has_no_cdn(web):
    code, body = _get(web + "/rc")
    assert code == 200
    for token in ("http://", "https://", "cdn", "googleapis"):
        assert token not in body, f"внешний ресурс: {token}"
