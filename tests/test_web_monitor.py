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
    assert len(state["points"]) == len(state["reference_points"]) == 301
    assert state["reference_points"][76] == [0.76, 190.04]
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
    assert "Учебная модель, не полётный симулятор" not in page.decode()
    assert "Не переносите результаты на реальный дрон без проверки" not in page.decode()
    assert "Идеальный отклик · учебный ориентир" in page.decode()
    assert "без перерегулирования и влияния возмущений" in page.decode()
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


def test_pid_trainer_full_settings_and_sliders(fc, monitor):
    from bfemu.fc import MSP_PID, MSP_PID_ADVANCED, MSP_SIMPLIFIED_TUNING

    url = monitor + "/api/pid-trainer"
    full_url = url + "/settings"
    state = json.loads(get(url)[1])
    assert sum(len(group["fields"]) for group in state["groups"]) == 53
    before_filters = {name: fc.settings[name] for name in ("dterm_lpf1_static_hz", "gyro_lpf1_static_hz")}
    request = {"profile": state["profile"], "expected": state["settings"],
               "changes": {"tpa_rate": 42, "f_roll": 250, "d_min_roll": 23, "angle_p_gain": 70}}
    assert post(full_url, request)[0] == 200
    advanced = fc.handle(MSP_PID_ADVANCED, b"", 1)
    assert advanced[58] == 42 and int.from_bytes(advanced[32:34], "little") == 250
    assert advanced[39] == 23 and fc.handle(MSP_PID, b"", 1)[9] == 70
    # Не теряем изменения из MSP или другой вкладки и не допускаем частичных записей.
    assert post(full_url, request)[0] == 409
    state = json.loads(get(url)[1])
    expected = state["settings"]
    assert post(full_url, {"profile": state["profile"], "expected": expected,
                           "changes": {"tpa_rate": 25, "f_roll": 1001}})[0] == 400
    assert fc.settings["tpa_rate"] == 42
    assert post(full_url, {"profile": state["profile"], "expected": expected,
                           "changes": {"tpa_rate": 25, "simplified_pi_gain": 120}})[0] == 400
    sliders = {"profile": state["profile"], "expected": expected,
               "changes": {"simplified_pids_mode": 1, "simplified_pi_gain": 150}}
    yaw_before = [fc.settings[f"{prefix}_yaw"] for prefix in "pidf"]
    assert post(full_url, sliders)[0] == 200
    assert fc.settings["p_roll"] == 67 and fc.settings["i_roll"] == 120
    assert [fc.settings[f"{prefix}_yaw"] for prefix in "pidf"] == yaw_before
    assert fc.handle(MSP_SIMPLIFIED_TUNING, b"", 1)[5] == 150
    assert {name: fc.settings[name] for name in before_filters} == before_filters
    state = json.loads(get(url)[1])
    sliders = {"profile": state["profile"], "expected": state["settings"],
               "changes": {"simplified_pids_mode": 0, "simplified_pi_gain": 50}}
    assert post(full_url, sliders)[0] == 200
    assert fc.settings["p_roll"] == 67  # OFF не пересчитывает PID
    fc.select_profile(1)
    assert post(full_url, sliders)[0] == 409


def test_pid_trainer_extended_settings_reject_cross_origin(fc, monitor):
    state = json.loads(get(monitor + "/api/pid-trainer")[1])
    body = {"profile": state["profile"], "expected": state["settings"], "changes": {"tpa_rate": 0}}
    assert post(monitor + "/api/pid-trainer/settings", body, {"Origin": "http://evil.example"})[0] == 403
    assert fc.settings["tpa_rate"] == state["settings"]["tpa_rate"]


def test_pid_analysis_api_is_stateless_and_validated(fc, monitor):
    url = monitor + "/api/pid-trainer"
    state = json.loads(get(url)[1])
    before = state["settings"]
    after = {**before, "p_roll": 70, "tpa_rate": 10}
    request = {"axis": "roll", "scenario": "step", "before": before, "after": after}
    status, body = post(url + "/analysis", request)
    result = json.loads(body)
    assert status == 200 and len(result["changes"]) == 2
    assert result["max_gyro_delta"] > 0 and fc.settings["p_roll"] == before["p_roll"]
    for broken in ({**request, "axis": "bad"}, {**request, "after": {}},
                   {**request, "after": {**after, "p_roll": True}},
                   {**request, "scenario": "bad"}):
        assert post(url + "/analysis", broken)[0] == 400
    assert post(url + "/analysis", request, {"Origin": "http://evil.example"})[0] == 403
    assert "Анализ действий" in get(monitor + "/pid-trainer")[1].decode()


def test_pid_trainer_reports_first_zero_error_time(fc, monitor):
    url = monitor + "/api/pid-trainer"
    state = json.loads(get(url + "?axis=roll&scenario=step")[1])
    assert state["metrics"]["first_zero_error_s"] == 2.16
    assert state["points"][256][1] == state["points"][256][2]
    assert all(point[1] != point[2] for point in state["points"][40:256])
    payload = {"axis": "roll", "profile": state["profile"], "expected": state["values"],
               "values": {"p": 0, "i": 0, "d": 0}}
    assert post(url, payload)[0] == 200
    assert json.loads(get(url)[1])["metrics"]["first_zero_error_s"] is None
    page = get(monitor + "/pid-trainer")[1].decode()
    assert 'id="zero-error-time"' in page
    assert "Первое совпадение с заданием" in page
    assert "После первого совпадения ошибка может появиться снова" in page


def test_pid_trainer_has_separate_resultant_signal_graph(monitor):
    page = get(monitor + "/pid-trainer")[1].decode()
    assert page.index('id="term-p"') < page.index('id="term-i"') < page.index('id="term-d"') < page.index('id="term-f"') < page.index('id="result-graph"')
    assert "вертикальные шкалы подбираются независимо" in page
    assert 'aria-label="График результирующего управляющего сигнала выбранной оси"' in page
    assert "сумма составляющих P, I, D и Feedforward после ограничения" in page
    assert "Вертикальный масштаб автоматически подстраивается под сигнал" in page


def test_pid_trainer_undo_restores_snapshot_and_rejects_external_changes(fc, monitor):
    from bfemu.fc import MSP_PID, MSP_SET_PID

    url = monitor + "/api/pid-trainer"
    original = json.loads(get(url + "?axis=roll&scenario=step")[1])
    pitch = json.loads(get(url + "?axis=pitch&scenario=step")[1])["values"]
    new_values = {"p": 75, "i": 92, "d": 55}
    status, body = post(url, {"axis": "roll", "profile": original["profile"],
                              "expected": original["values"], "values": new_values})
    assert status == 200
    applied = json.loads(body)
    reverse = {"axis": applied["axis"], "profile": applied["profile"],
               "expected": applied["values"], "values": original["values"]}
    assert post(url, reverse)[0] == 200
    restored = json.loads(get(url + "?axis=roll&scenario=step")[1])
    assert restored["values"] == original["values"] and restored["points"] == original["points"]
    assert json.loads(get(url + "?axis=pitch&scenario=step")[1])["values"] == pitch

    assert post(url, {"axis": "roll", "profile": original["profile"],
                      "expected": original["values"], "values": new_values})[0] == 200
    raw = bytearray(fc.handle(MSP_PID, b"", 1))
    raw[0] = 101
    fc.handle(MSP_SET_PID, bytes(raw), 1)
    assert post(url, reverse)[0] == 409
    assert json.loads(get(url + "?axis=roll")[1])["values"]["p"] == 101
    fc.select_profile(1)
    assert post(url, reverse)[0] == 409


def test_pid_trainer_graph_follows_manually_selected_pid_profile(fc, monitor):
    url = monitor + "/api/pid-trainer?axis=roll&scenario=step"
    first = json.loads(get(url)[1])
    fc.select_profile(1)
    fc._store_setting("p_roll", first["values"]["p"] + 25)
    fc._save_active_pid_profile()
    second = json.loads(get(url)[1])
    assert second["profile"] == 1
    assert second["values"]["p"] == first["values"]["p"] + 25
    assert second["points"] != first["points"]
    fc.select_profile(0)
    restored = json.loads(get(url)[1])
    assert restored["profile"] == 0
    assert restored["values"] == first["values"]
    assert restored["points"] == first["points"]


def test_sitl_data_page_offers_recorded_raw_msp_not_generated_graphs(monitor):
    status, data = get(monitor + "/sitl-data")
    page = data.decode()
    assert status == 200 and "Данные Betaflight SITL" in page
    assert "файл обрабатывается в браузере" in page.lower()
    assert "MSP_RAW_IMU" in page and "MSP_MOTOR" in page
    assert "в этих MSP-ответах не передаются" in page
    assert "JSON.parse" in page and "payload_hex" in page


def test_aircraft_page_is_reachable_and_does_not_claim_sitl_integration(monitor):
    status, data = get(monitor + "/aircraft")
    page = data.decode()
    assert status == 200
    assert "Параметры аппарата" in page
    assert 'href="/aircraft"' in get(monitor + "/pid-trainer")[1].decode()
    assert 'class="on" href="/aircraft"' in page
    assert "пока не передаётся в Betaflight или симулятор" in page
    assert 'name="mass_g"' in page and 'name="inertia_xx"' in page
    assert 'name="motor_4_output"' in page and 'name="sensor_notes"' in page
    assert 'name="motor_1_model" type="text"' in page
    assert "localStorage" in page
