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

from bfemu.fc import MSP_DATAFLASH_ERASE
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


# --- повторное ревью, часть 4: кривой ввод больше не рвёт соединение -----------------------
BAD_RC_FIELDS = [
    ({"bind_seconds": "несколько"}, "bind_seconds"),
    ({"bind_seconds": -1}, "bind_seconds"),
    ({"bind_seconds": True}, "bind_seconds"),
    ({"aux_map": 5}, "aux_map"),
    ({"rc_map": "нет"}, "rc_map"),
    ({"layout_note": 3}, "layout_note"),
    ({"source": 7}, "source"),
    ({"reset": "нет"}, "reset"),
    ({"connected": "да"}, "connected"),
    ({"control": "нет-такого"}, "control"),
]


@pytest.mark.parametrize("payload,field", BAD_RC_FIELDS)
def test_bad_field_type_is_answered_with_json_400(fc, monitor, payload, field):
    """Раньше обработчик падал на `float("несколько")`/`list(5)`: клиент получал обрыв связи.

    `/api/rc` — единственная точка записи для страницы /tx и инструмента tools/tx.py, поэтому
    отказ обязан приходить ответом: 400, текст про поле и состояние, ничего не изменив.
    """
    before = json.loads(get(monitor + "/api/rc")[1])
    status, body = post(monitor + "/api/rc", payload)
    answer = json.loads(body)
    assert status == 400, f"{payload} принят как верный"
    assert field in answer["error"] and "state" in answer
    after = json.loads(get(monitor + "/api/rc")[1])
    assert after["channels"] == before["channels"], "каналы не должны измениться при отказе"
    assert get(monitor + "/api/state")[0] == 200, "монитор продолжает работать"


def test_non_numeric_dashboard_path_is_answered_with_page(fc, monitor):
    """`/failsafe/procedure/abc` роняло обработчик `int(action.split('/')[1])` — обрыв без ответа."""
    status, body = get(monitor + "/failsafe/procedure/abc")
    page = body.decode()
    assert status == 400 and "не выполнен" in page
    assert get(monitor + "/")[0] == 200, "страница отвечает после отказа"


def test_msc_page_says_no_flights_after_erase(fc, monitor):
    """Шаблон /msc показывал «-1 пролётов»: служебный дамп считался пролётом (часть 4)."""
    assert "пролётов: -1" not in get(monitor + "/msc")[1].decode()
    fc.handle(MSP_DATAFLASH_ERASE, b"", 1)
    page = get(monitor + "/msc")[1].decode()
    assert "пролётов нет" in page and "пролётов: -1" not in page


def test_pid_trainer_bidirectional_msp_and_profiles(fc, monitor):
    from bfemu.fc import MSP_PID, MSP_SET_PID

    url = monitor + "/api/pid-trainer"
    status, body = get(url + "?axis=roll&scenario=step")
    state = json.loads(body)
    assert status == 200 and state["profile"] == 0
    assert len(state["points"]) == 301
    payload = {"axis": "roll", "profile": state["profile"], "expected": state["values"],
               "values": {"p": 75, "i": 90, "d": 50}}
    assert post(url, payload)[0] == 200
    assert list(fc.handle(MSP_PID, b"", 1)[:3]) == [75, 90, 50]
    before_graph = state["points"]
    updated = json.loads(get(url)[1])
    assert updated["points"] != before_graph
    raw = bytearray(fc.handle(MSP_PID, b"", 1))
    raw[:3] = bytes([30, 40, 20])
    fc.handle(MSP_SET_PID, bytes(raw), 1)
    assert json.loads(get(url)[1])["values"] == {"p": 30, "i": 40, "d": 20}
    # Старая форма не перезапишет изменения из Configurator.
    assert post(url, payload)[0] == 409
    fc.select_profile(1)
    assert post(url, payload)[0] == 409
    fc.select_profile(0)
    assert list(fc.handle(MSP_PID, b"", 1)[:3]) == [30, 40, 20]
    fc.save_to_eeprom()
    from bfemu.fc import VirtualFC
    restored = VirtualFC(eeprom_path=fc.eeprom_path)
    assert list(restored.handle(MSP_PID, b"", 1)[:3]) == [30, 40, 20]


def test_pid_trainer_validation_and_page(fc, monitor):
    url = monitor + "/api/pid-trainer"
    status, page = get(monitor + "/pid-trainer")
    assert status == 200 and "Почувствуйте PID" in page.decode()
    assert b'href="/pid-trainer"' in get(monitor + "/")[1]
    assert get(url + "?axis=bogus")[0] == 400
    assert get(url + "?scenario=bogus")[0] == 400
    current = json.loads(get(url)[1])
    payload = {"axis": "roll", "profile": current["profile"], "expected": current["values"],
               "values": {"p": 80, "i": 80, "d": 80}}
    for invalid in (-1, 251, True, 1.5, "20", None):
        payload["values"]["d"] = invalid
        assert post(url, payload)[0] == 400
        assert json.loads(get(url)[1])["values"] == current["values"]
    assert post(url, payload, {"Origin": "http://evil.example"})[0] == 403
