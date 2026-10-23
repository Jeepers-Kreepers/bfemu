"""Веб-интерфейс и виртуальное оборудование (/equipment)."""
import json
import struct
import urllib.error
import urllib.request

import pytest

from inavemu import msp
from inavemu.fc import EQ_KEYS, InavFC, MSPError
from inavemu.server import FCServer, MonitorServer


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


# ---------------- модель оборудования ----------------

def test_equipment_all_on_by_default(fc):
    eq = fc.get_equipment()
    assert set(eq) == set(EQ_KEYS)
    assert all(eq.values()), "по умолчанию стенд оснащён полностью"
    st = ask(fc, 151)  # MSP_SENSOR_STATUS
    assert list(st) == [1, 1, 1, 1, 1, 1, 1, 1, 1], \
        "все индикаторы (Gyro..Speed) горят"


def test_equipment_toggle_reflects_in_sensor_status(fc):
    fc.set_equipment("mag", False)
    fc.set_equipment("gps", False)
    st = ask(fc, 151)
    # healthy, gyro, acc, mag, baro, gps, sonar, pitot, opflow
    assert list(st) == [1, 1, 1, 0, 1, 0, 1, 1, 1]
    fc.set_equipment("mag", True)
    assert ask(fc, 151)[3] == 1


def test_equipment_sync_with_sensor_config(fc):
    # выключение магнитометра гасит его и в MSP_SENSOR_CONFIG (кнопки Calibration)
    fc.set_equipment("mag", False)
    sc = ask(fc, 96)
    assert sc[2] == 0
    fc.set_equipment("mag", True)
    sc = ask(fc, 96)
    assert sc[2] == 2  # вернулся «чип» HMC5883
    # и наоборот: SET_SENSOR_CONFIG выключает оборудование
    assert ask(fc, 97, bytes([2, 3, 0, 4, 3, 2])) == b""
    assert fc.get_equipment()["mag"] is False
    assert ask(fc, 151)[3] == 0


def test_equipment_unknown_name_is_error(fc):
    with pytest.raises(MSPError):
        fc.set_equipment("warp_drive", True)


def test_equipment_telemetry_follows(fc):
    fc.set_equipment("gps", False)
    gps = ask(fc, 106)  # MSP_RAW_GPS
    assert gps[0] == 0 and gps[1] == 0, "GPS выключен — нет фикса"
    fc.set_equipment("gps", True)
    gps = ask(fc, 106)
    assert gps[0] == 1 and gps[1] > 0
    fc.set_equipment("sonar", False)
    assert ask(fc, 58) == b"\x00" * 4
    fc.set_equipment("sonar", True)
    assert struct.unpack("<i", ask(fc, 58))[0] > 0
    fc.set_equipment("pitot", False)
    assert ask(fc, 0x2009) == b"\x00" * 4
    fc.set_equipment("pitot", True)
    assert struct.unpack("<i", ask(fc, 0x2009))[0] > 0


def test_equipment_persists(fc, tmp_path):
    fc.eeprom_path = tmp_path / "e.json"
    fc.set_equipment("gyro", False)
    fc.set_equipment("gps", False)
    fc.save_eeprom()
    fc2 = InavFC(eeprom_path=fc.eeprom_path)
    assert fc2.get_equipment()["gyro"] is False
    assert fc2.get_equipment()["gps"] is False


# ---------------- веб-интерфейс ----------------

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


def test_web_pages(web):
    code, body = _get(web + "/")
    assert code == 200 and "Монитор" in body and "Оборудование" in body
    code, body = _get(web + "/equipment")
    assert code == 200
    for title in ("Гироскоп", "Магнитометр", "Оптический поток", "Сонар"):
        assert title in body
    code, _ = _get(web + "/nope")
    assert code == 404


def test_web_state_contains_equipment(web):
    code, body = _get(web + "/api/state")
    assert code == 200
    st = json.loads(body)
    assert all(st["equipment"].values())


def test_web_toggle_equipment(web, fc):
    code, body = _post(web + "/api/equipment", {"mag": False, "sonar": False})
    assert code == 200
    assert body["equipment"]["mag"] is False
    assert body["equipment"]["sonar"] is False
    assert body["equipment"]["gyro"] is True
    assert fc.get_equipment()["mag"] is False
    st = ask(fc, 151)
    assert list(st)[3] == 0 and list(st)[6] == 0
    # частичное обновление не трогает остальное
    code, body = _post(web + "/api/equipment", {"mag": True})
    assert body["equipment"]["mag"] is True
    assert body["equipment"]["sonar"] is False


def test_web_bad_payload_is_400(web):
    code, body = _post(web + "/api/equipment", {"warp_drive": True})
    assert code == 400 and "error" in body
    code, body = _post(web + "/api/equipment", {"mag": "yes"})
    assert code == 400
    code, body = _post(web + "/api/equipment", {})
    assert code == 400


# ---------------- моторы и сервы ----------------

def test_outputs_counts_drive_mixer_rules(fc):
    # по умолчанию 4 мотора, серв нет
    assert fc.motors_connected == 4 and fc.servos_connected == 0
    mm = ask(fc, 0x1005)   # MSP2_COMMON_MOTOR_MIXER
    used = [struct.unpack_from("<H", mm, i)[0] == 3000
            for i in range(0, 96, 8)]
    assert sum(used) == 4
    fc.set_motors_connected(7)
    fc.set_servos_connected(3)
    mm = ask(fc, 0x1005)
    assert sum(struct.unpack_from("<H", mm, i)[0] == 3000
               for i in range(0, 96, 8)) == 7
    sm = ask(fc, 0x2020)   # MSP2_INAV_SERVO_MIXER
    targets = [sm[i] for i in range(0, 36 * 6, 6)
               if struct.unpack_from("<H", sm, i + 2)[0] != 0]
    assert targets == [0, 1, 2]
    # профиль 2 — пустые правила в кодировке прошивки (2000), а не нули
    assert mm[96:104] == struct.pack("<HHHH", 2000, 2000, 2000, 2000)


def test_outputs_ranges_are_validated(fc):
    import pytest as _pytest
    with _pytest.raises(MSPError):
        fc.set_motors_connected(0)
    with _pytest.raises(MSPError):
        fc.set_motors_connected(13)
    with _pytest.raises(MSPError):
        fc.set_servos_connected(-1)
    with _pytest.raises(MSPError):
        fc.set_servos_connected(19)


def test_motor_values_and_set_motor_echo(fc):
    assert len(ask(fc, 104)) == 16          # MSP_MOTOR: 8×u16
    assert len(ask(fc, 103)) == 36          # MSP_SERVO: 18×u16
    payload = struct.pack("<8H", 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900)
    assert ask(fc, 214, payload) == b""     # MSP_SET_MOTOR
    assert ask(fc, 104) == payload          # значения эхом идут в MSP_MOTOR
    with pytest.raises(MSPError):
        ask(fc, 214, b"\x01")


def test_outputs_persist(fc, tmp_path):
    fc.eeprom_path = tmp_path / "e.json"
    fc.set_motors_connected(6)
    fc.set_servos_connected(2)
    fc.save_eeprom()
    fc2 = InavFC(eeprom_path=fc.eeprom_path)
    assert fc2.motors_connected == 6
    assert fc2.servos_connected == 2


def test_web_outputs_page_and_api(web, fc):
    code, body = _get(web + "/outputs")
    assert code == 200
    for title in ("Моторы", "Сервоприводы", "M12", "S18"):
        assert title in body
    code, body = _get(web + "/api/state")
    st = json.loads(body)
    assert st["outputs"]["motors"] == 4
    code, body = _post(web + "/api/outputs", {"motors": 8, "servos": 5})
    assert code == 200
    assert body["outputs"]["motors"] == 8
    assert body["outputs"]["servos"] == 5
    assert fc.motors_connected == 8 and fc.servos_connected == 5
    # частичное обновление
    code, body = _post(web + "/api/outputs", {"servos": 0})
    assert body["outputs"]["motors"] == 8
    assert body["outputs"]["servos"] == 0


def test_web_outputs_bad_payload_is_400(web):
    code, body = _post(web + "/api/outputs", {"motors": 13})
    assert code == 400
    code, body = _post(web + "/api/outputs", {"motors": True})
    assert code == 400
    code, body = _post(web + "/api/outputs", {"warp": 1})
    assert code == 400
    code, body = _post(web + "/api/outputs", {})
    assert code == 400
    assert "error" in body
