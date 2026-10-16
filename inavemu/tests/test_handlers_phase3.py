"""Обработчики фазы 3: миссии/WP, safehome, fw-approach, geozone,
логические условия, programming PID, failsafe, blackbox, dataflash.

Форматы кадров сверены с fc_msp.c прошивки INAV 9.1.0 и с запросами
inav-configurator 9.1.1 (mission_control.js, programming.js,
onboard_logging.js, failsafe.js).
"""
import struct

import pytest

from inavemu import msp
from inavemu.fc import (
    MSP2_BLACKBOX_CONFIG, MSP2_INAV_FW_APPROACH, MSP2_INAV_GEOZONE,
    MSP2_INAV_GEOZONE_VERTEX, MSP2_INAV_GVAR_STATUS,
    MSP2_INAV_LOGIC_CONDITIONS, MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED,
    MSP2_INAV_LOGIC_CONDITIONS_SINGLE, MSP2_INAV_LOGIC_CONDITIONS_STATUS,
    MSP2_INAV_PROGRAMMING_PID, MSP2_INAV_PROGRAMMING_PID_STATUS,
    MSP2_INAV_SAFEHOME, MSP2_INAV_SET_FW_APPROACH, MSP2_INAV_SET_GEOZONE,
    MSP2_INAV_SET_GEOZONE_VERTICE, MSP2_INAV_SET_LOGIC_CONDITIONS,
    MSP2_INAV_SET_PROGRAMMING_PID, MSP2_INAV_SET_SAFEHOME,
    MSP2_SET_BLACKBOX_CONFIG,
    MSP_BLACKBOX_CONFIG, MSP_DATAFLASH_ERASE, MSP_DATAFLASH_READ,
    MSP_DATAFLASH_SUMMARY, MSP_FAILSAFE_CONFIG, MSP_SDCARD_SUMMARY,
    MSP_SET_FAILSAFE_CONFIG, MSP_SET_WP, MSP_WP, MSP_WP_GETINFO,
    MSP_WP_MISSION_LOAD, MSP_WP_MISSION_SAVE,
    MSPError, InavFC,
)


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


def wp_payload(no, action, lat, lon, alt, p1, p2, p3, flag):
    return (bytes([no, action])
            + struct.pack("<iiI", lat, lon, alt)
            + struct.pack("<HHH", p1, p2, p3) + bytes([flag]))


# ---------------------------------------------------------------- миссии

def test_wp_upload_and_getinfo(fc):
    assert ask(fc, MSP_WP_GETINFO) == bytes([0, 120, 0, 0])
    ask(fc, MSP_SET_WP, wp_payload(1, 1, 55755830, 37617380, 5000, 0, 0, 0, 0))
    ask(fc, MSP_SET_WP, wp_payload(2, 1, 55756000, 37618000, 7000, 0, 0, 0, 0))
    ask(fc, MSP_SET_WP, wp_payload(3, 4, 0, 0, 0, 0, 0, 0, 0xA5))
    assert ask(fc, MSP_WP_GETINFO) == bytes([0, 120, 1, 3])

    # ответ MSP_WP зеркалит запрос: [no][action][lat][lon][alt][p1..p3][flag]
    got = ask(fc, MSP_WP, bytes([2]))
    assert got[0] == 2 and got[1] == 1
    assert struct.unpack_from("<iiI", got, 2) == (55756000, 37618000, 7000)
    assert got[20] == 0


def test_wp_sequential_upload_only(fc):
    # загрузка «не по порядку» молча игнорируется (как в прошивке)
    ask(fc, MSP_SET_WP, wp_payload(3, 1, 1, 1, 1, 0, 0, 0, 0))
    assert ask(fc, MSP_WP_GETINFO) == bytes([0, 120, 0, 0])
    # №1 начинает миссию заново
    ask(fc, MSP_SET_WP, wp_payload(1, 1, 5, 6, 7, 0, 0, 0, 0xA5))
    ask(fc, MSP_SET_WP, wp_payload(1, 1, 8, 9, 10, 0, 0, 0, 0xA5))
    assert ask(fc, MSP_WP_GETINFO) == bytes([0, 120, 1, 1])
    assert struct.unpack_from("<iiI", ask(fc, MSP_WP, bytes([1])), 2) == (8, 9, 10)


def test_wp_jump_p1_index_roundtrip(fc):
    # JUMP: наружу p1 = индекс+1, внутрь — индекс
    ask(fc, MSP_SET_WP, wp_payload(1, 6, 0, 0, 0, 3, 0, 0, 0))  # JUMP к №3
    got = ask(fc, MSP_WP, bytes([1]))
    assert struct.unpack_from("<H", got, 14)[0] == 3
    assert fc.waypoints[0]["p1"] == 2


def test_wp_default_reply(fc):
    # пустая миссия: action=RTH(4), flag=LAST(0xA5) — как getWaypoint()
    got = ask(fc, MSP_WP, bytes([5]))
    assert got == wp_payload(5, 4, 0, 0, 0, 0, 0, 0, 0xA5)


def test_mission_save_load(fc):
    with pytest.raises(MSPError):
        ask(fc, MSP_WP_MISSION_SAVE, bytes([0]))  # миссия не готова
    ask(fc, MSP_SET_WP, wp_payload(1, 1, 11, 22, 33, 0, 0, 0, 0xA5))
    assert ask(fc, MSP_WP_MISSION_SAVE, bytes([0])) == b""
    # RAM «сбрасывается» — миссия восстанавливается из NVM
    fc.waypoints = []
    fc.waypoint_list_valid = False
    assert ask(fc, MSP_WP_MISSION_LOAD, bytes([0])) == b""
    assert ask(fc, MSP_WP_GETINFO) == bytes([0, 120, 1, 1])
    with pytest.raises(MSPError):
        ask(fc, MSP_WP_MISSION_LOAD, bytes([0, 0]))  # payload != 1 байт


def test_mission_load_empty_nv(fc):
    with pytest.raises(MSPError):
        ask(fc, MSP_WP_MISSION_LOAD, bytes([0]))


# ---------------------------------------------------------------- failsafe

def test_failsafe_config_roundtrip(fc):
    raw = ask(fc, MSP_FAILSAFE_CONFIG)
    assert len(raw) == 20
    new = struct.pack("<BBH B H BB hhh HH B", 10, 2, 1200, 0, 30, 2, 5,
                      -10, 12, 34, 100, 50, 1)
    assert ask(fc, MSP_SET_FAILSAFE_CONFIG, new) == b""
    assert ask(fc, MSP_FAILSAFE_CONFIG) == new
    assert fc.store.get("failsafe_delay") == 10
    assert fc.store.get("failsafe_procedure") == 2
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_FAILSAFE_CONFIG, new[:19])


# ---------------------------------------------------------------- safehome

def test_safehome_roundtrip(fc):
    for i in range(8):
        got = ask(fc, MSP2_INAV_SAFEHOME, bytes([i]))
        assert got == bytes([i, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    payload = bytes([2, 1]) + struct.pack("<ii", 55755830, 37617380)
    assert ask(fc, MSP2_INAV_SET_SAFEHOME, payload) == b""
    assert ask(fc, MSP2_INAV_SAFEHOME, bytes([2])) == payload
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SAFEHOME, bytes([8]))
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_SAFEHOME, payload[:9])


# ---------------------------------------------------------------- fw-approach

def test_fw_approach_roundtrip(fc):
    got = ask(fc, MSP2_INAV_FW_APPROACH, bytes([0]))
    assert len(got) == 15
    payload = (bytes([3]) + struct.pack("<ii", 5000, 1000) + bytes([1])
               + struct.pack("<hh", 180, -90) + bytes([1]))
    assert ask(fc, MSP2_INAV_SET_FW_APPROACH, payload) == b""
    assert ask(fc, MSP2_INAV_FW_APPROACH, bytes([3])) == payload
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_FW_APPROACH, bytes([17]))


# ---------------------------------------------------------------- geozone

def test_geozone_polygon(fc):
    # SET_GEOZONE = 14 байт; сбрасывает вершины зоны
    zone = (bytes([5]) + bytes([1, 1])       # INCLUSIVE, POLYGON
            + struct.pack("<ii", 0, 10000) + bytes([0, 0]) + bytes([3]))
    assert ask(fc, MSP2_INAV_SET_GEOZONE, zone) == b""
    assert ask(fc, MSP2_INAV_GEOZONE, bytes([5])) == zone
    for i, (lat, lon) in enumerate([(1, 2), (3, 4), (5, 6)]):
        payload = bytes([5, i]) + struct.pack("<ii", lat, lon)
        assert ask(fc, MSP2_INAV_SET_GEOZONE_VERTICE, payload) == b""
        assert ask(fc, MSP2_INAV_GEOZONE_VERTEX, bytes([5, i])) == payload
    # повторный SET_GEOZONE чистит вершины; vertexCount — конфиг-поле
    ask(fc, MSP2_INAV_SET_GEOZONE, zone)
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_GEOZONE_VERTEX, bytes([5, 0]))
    assert ask(fc, MSP2_INAV_GEOZONE, bytes([5]))[13] == 3


def test_geozone_circular_radius(fc):
    zone = (bytes([1]) + bytes([0, 0])       # EXCLUSIVE, CIRCULAR
            + struct.pack("<ii", 0, 0) + bytes([0, 2]) + bytes([2]))
    ask(fc, MSP2_INAV_SET_GEOZONE, zone)
    # круговая: 10 байт + 4 байта радиуса (хранится в вершине №+1)
    payload = (bytes([1, 0]) + struct.pack("<ii", 55755830, 37617380)
               + struct.pack("<I", 250))
    assert ask(fc, MSP2_INAV_SET_GEOZONE_VERTICE, payload) == b""
    assert ask(fc, MSP2_INAV_GEOZONE_VERTEX, bytes([1, 0])) == payload
    assert ask(fc, MSP2_INAV_GEOZONE, bytes([1]))[13] == 2


def test_geozone_bounds(fc):
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_GEOZONE, bytes([63]))
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_GEOZONE_VERTICE,
            bytes([0, 126]) + struct.pack("<ii", 0, 0))


# ------------------------------------------------------- логические условия

def test_logic_conditions(fc):
    default = ask(fc, MSP2_INAV_LOGIC_CONDITIONS_SINGLE, bytes([0]))
    assert default == bytes([0, 255]) + bytes(12)  # activatorId=-1 (255)
    # SET: [idx] + 14 байт
    rec = (bytes([1, 3, 2, 4, 5]) + struct.pack("<i", 100)
           + bytes([6]) + struct.pack("<i", -7) + bytes([1]))
    assert ask(fc, MSP2_INAV_SET_LOGIC_CONDITIONS, rec) == b""
    assert ask(fc, MSP2_INAV_LOGIC_CONDITIONS_SINGLE, bytes([1])) == rec[1:]
    # маска «настроено»: бит 1 выставлен
    mask = ask(fc, MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED)
    assert struct.unpack("<II", mask) == (0x2, 0)
    assert len(ask(fc, MSP2_INAV_LOGIC_CONDITIONS_STATUS)) == 64 * 4
    # устаревшая пачечная команда — ошибка, как в прошивке
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_LOGIC_CONDITIONS)
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_LOGIC_CONDITIONS_SINGLE, bytes([64]))


# --------------------------------------------------------- programming PID

def test_programming_pid(fc):
    assert ask(fc, MSP2_INAV_PROGRAMMING_PID) == bytes(4 * 19)
    rec = (bytes([0, 1, 2]) + struct.pack("<i", 42) + bytes([3])
           + struct.pack("<i", -1) + struct.pack("<HHHH", 10, 20, 30, 40))
    assert ask(fc, MSP2_INAV_SET_PROGRAMMING_PID, rec) == b""
    assert ask(fc, MSP2_INAV_PROGRAMMING_PID)[:19] == rec[1:]
    assert len(ask(fc, MSP2_INAV_PROGRAMMING_PID_STATUS)) == 4 * 4
    assert len(ask(fc, MSP2_INAV_GVAR_STATUS)) == 8 * 4


# ------------------------------------------------------------- blackbox

def test_blackbox_configs(fc):
    # v1: «API no longer supported» — 4 нуля
    assert ask(fc, MSP_BLACKBOX_CONFIG) == bytes(4)
    # v2: [supported][device][rate_num][rate_denom][includeFlags]
    v2 = ask(fc, MSP2_BLACKBOX_CONFIG)
    assert v2 == bytes([1, 0]) + struct.pack("<HHI", 1, 32, 0)
    new = bytes([2]) + struct.pack("<HHI", 1, 16, 0x0F)
    assert ask(fc, MSP2_SET_BLACKBOX_CONFIG, new) == b""
    assert ask(fc, MSP2_BLACKBOX_CONFIG) == bytes([1]) + new
    with pytest.raises(MSPError):
        ask(fc, MSP2_SET_BLACKBOX_CONFIG, new[:8])
    # v1 SET прошивкой не поддерживается
    with pytest.raises(MSPError):
        ask(fc, 81, new[:4])


# ------------------------------------------------------------- dataflash

def test_dataflash(fc):
    summary = ask(fc, MSP_DATAFLASH_SUMMARY)
    assert len(summary) == 13
    assert summary[0] == 1  # ready
    got = ask(fc, MSP_DATAFLASH_READ, struct.pack("<IH", 0, 64))
    assert got == struct.pack("<I", 0) + bytes(64)
    assert ask(fc, MSP_DATAFLASH_ERASE) == b""
    assert ask(fc, MSP_SDCARD_SUMMARY) == bytes(11)


# ------------------------------------------- цепочки как у Configurator

def test_configurator_chains(fc):
    # getWaypointsFromFC: GETINFO + WP[1..count] (миссия из 2 WP)
    for i, flag in ((1, 0), (2, 0xA5)):
        ask(fc, MSP_SET_WP, wp_payload(i, 1, i * 10, i * 20, i * 30, 0, 0, 0, flag))
    info = ask(fc, MSP_WP_GETINFO)
    for no in range(1, info[3] + 1):
        assert ask(fc, MSP_WP, bytes([no]))[0] == no
    # loadSafehomes: 8 запросов подряд — все отвечают
    for i in range(8):
        assert len(ask(fc, MSP2_INAV_SAFEHOME, bytes([i]))) == 10
    # loadFwApproach: 17 запросов
    for i in range(17):
        assert len(ask(fc, MSP2_INAV_FW_APPROACH, bytes([i]))) == 15
    # loadGeozones: 63 зоны + финальный запрос id=63 (прошивка отвечает ошибкой)
    for i in range(63):
        assert ask(fc, MSP2_INAV_GEOZONE, bytes([i]))[0] == i
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_GEOZONE, bytes([63]))
