"""Веб-монитор: экранирование, защита от межсайтовых запросов, формат /api/state.

Монитор поднимается прямо в процессе теста на свободном порту — без подпроцессов,
поэтому весь модуль укладывается в пару секунд.
"""
from __future__ import annotations

import json
import socket
import threading
import urllib.error
import urllib.request

import pytest

from bfemu.server import HttpMonitor


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


@pytest.fixture
def monitor(fc):
    http = HttpMonitor(fc, free_port(), strict_port=True, host="127.0.0.1")
    http.prepare()
    thread = threading.Thread(target=http.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{http.port}"
    http.stop()
    thread.join(timeout=5)


def get(url: str, headers: dict | None = None):
    request = urllib.request.Request(url, headers=headers or {})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def post(url: str, payload: dict, headers: dict | None = None):
    request = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json", **(headers or {})})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def test_default_bind_is_localhost(fc):
    """Монитор не должен быть виден всей сети, пока об этом не попросили явно."""
    assert HttpMonitor(fc, 0).host == "127.0.0.1"


def test_craft_name_is_escaped(fc, monitor):
    """Имя дрона задаётся из CLI/Configurator — в разметку оно обязано попадать экранированным."""
    fc.cli_execute("set craft_name = <img src=x onerror=alert(1)>")
    fc.cli_execute("version")                       # строка уедет в журнал CLI на странице
    status, body = get(monitor + "/")
    page = body.decode()
    assert status == 200
    assert "<img src=x onerror" not in page
    assert "&lt;img src=x onerror" in page


def test_cross_site_requests_are_rejected(monitor):
    assert get(monitor + "/sim_flight/on", {"Sec-Fetch-Site": "cross-site"})[0] == 403
    assert post(monitor + "/api/rc", {"roll": 1600}, {"Origin": "http://evil.example"})[0] == 403


def test_same_origin_and_console_requests_work(monitor):
    assert get(monitor + "/sim_flight/on", {"Sec-Fetch-Site": "same-origin"})[0] == 200
    assert get(monitor + "/sim_flight/off")[0] == 200          # консольный клиент без заголовков
    assert post(monitor + "/api/rc", {"roll": 1600})[0] == 200
    channels = json.loads(get(monitor + "/api/rc")[1])["channels"]
    roll = channels["roll"] if isinstance(channels, dict) else channels[0]
    assert roll == 1600


def test_rejected_rc_request_changes_nothing(fc, monitor):
    """Отказ (400) обязан приходить до изменений: раньше поля запроса уже применялись.

    Повторное ревью: `/api/rc` возвращал 400 на неизвестный канал, но `source` из того же
    запроса успевал записаться. Здесь проверяем и ответ, и состояние после него.
    """
    before = json.loads(get(monitor + "/api/rc")[1])
    status, body = post(monitor + "/api/rc",
                        {"source": "тест-400", "roll": 1600, "нет_такого_канала": 1500})
    after = json.loads(get(monitor + "/api/rc")[1])
    assert status == 400 and "не распознаны" in json.loads(body)["error"]
    assert after["source"] == before["source"], "источник не должен записаться при отказе"
    assert after["channels"] == before["channels"], "каналы не должны измениться при отказе"


def test_non_numeric_channel_value_is_rejected(fc, monitor):
    """Нечисловое значение раньше роняло обработчик: соединение рвалось без ответа."""
    before = json.loads(get(monitor + "/api/rc")[1])["channels"]
    status, body = post(monitor + "/api/rc", {"roll": "не-число"})
    assert status == 400 and "числами" in json.loads(body)["error"]
    assert json.loads(get(monitor + "/api/rc")[1])["channels"] == before
    assert post(monitor + "/api/rc", {"roll": 1600})[0] == 200, "сервер продолжает работать"


def test_infinite_channel_value_is_rejected(monitor):
    """JSON принимает Infinity: int(inf) в set_rc_channel падал уже после записи каналов."""
    status, body = post(monitor + "/api/rc", {"roll": float("inf")})
    assert status == 400 and "числами" in json.loads(body)["error"]


def test_internal_error_is_answered_with_json(fc, monitor, monkeypatch):
    """Если запись каналов всё же сорвалась, клиент получает ответ, а не обрыв связи."""
    def boom(_payload):
        raise RuntimeError("поломка ради проверки")

    monkeypatch.setattr(fc, "set_rc_channels", boom)
    status, body = post(monitor + "/api/rc", {"roll": 1600})
    assert status == 500 and "поломка ради проверки" in json.loads(body)["error"]


def test_api_state_has_expected_sections(monitor):
    status, body = get(monitor + "/api/state")
    state = json.loads(body)
    assert status == 200
    for key in ("fc", "rc", "settings", "features", "ports", "blackbox", "gps",
                "motors", "power", "modes", "failsafe", "cli", "msp", "counters"):
        assert key in state, f"в /api/state нет раздела {key}"
    assert state["settings"] == 683


def test_unknown_blackbox_file_is_404(monitor):
    assert get(monitor + "/msc/NOPE.BBL")[0] == 404
