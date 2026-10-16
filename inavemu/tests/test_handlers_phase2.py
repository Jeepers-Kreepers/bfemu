"""Обработчики фазы 2: PID, ez-tune, датчики, GPS, OSD, миссии."""
import struct

import pytest

from inavemu import msp
from inavemu.fc import (
    MSP2_ADSB_VEHICLE_LIST, MSP2_INAV_CUSTOM_OSD_ELEMENTS,
    MSP2_INAV_EZ_TUNE, MSP2_INAV_EZ_TUNE_SET, MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED,
    MSP2_INAV_OSD_ALARMS, MSP2_INAV_OSD_LAYOUTS, MSP2_INAV_OSD_PREFERENCES,
    MSP2_INAV_OSD_SET_ALARMS, MSP2_INAV_OSD_SET_LAYOUT_ITEM, MSP2_INAV_OSD_SET_PREFERENCES,
    MSP2_INAV_RATE_DYNAMICS, MSP2_INAV_SERVO_MIXER, MSP2_INAV_SET_RATE_DYNAMICS,
    MSP2_PID, MSP2_SET_PID, MSP_GPSSTATISTICS, MSP_SENSOR_CONFIG,
    MSP_SET_RESET_CURR_PID, MSP_SET_WP, MSP_WP, MSP_WP_GETINFO,
    MSPError, InavFC,
    OSD_ITEM_COUNT, OSD_LAYOUT_COUNT,
)


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


def test_pid_banks(fc):
    pids = ask(fc, MSP2_PID)
    assert len(pids) == 11 * 4
    assert pids[0] == 40  # P для ROLL
    new = bytes([10, 20, 30, 40]) * 11
    ask(fc, MSP2_SET_PID, new)
    assert ask(fc, MSP2_PID)[:4] == bytes([10, 20, 30, 40])


def test_reset_curr_pid_ack(fc):
    assert ask(fc, MSP_SET_RESET_CURR_PID) == b""


def test_rate_dynamics(fc):
    rd = ask(fc, MSP2_INAV_RATE_DYNAMICS)
    assert len(rd) == 6
    ask(fc, MSP2_INAV_SET_RATE_DYNAMICS, bytes([9] * 6))
    assert ask(fc, MSP2_INAV_RATE_DYNAMICS) == bytes([9] * 6)


def test_ez_tune(fc):
    ez = ask(fc, MSP2_INAV_EZ_TUNE)
    assert len(ez) == 11
    payload = bytes([1]) + struct.pack("<H", 200) + bytes(range(8))
    ask(fc, MSP2_INAV_EZ_TUNE_SET, payload)
    ez = ask(fc, MSP2_INAV_EZ_TUNE)
    assert ez[0] == 1
    assert struct.unpack_from("<H", ez, 1)[0] == 200


def test_sensor_config(fc):
    sc = ask(fc, MSP_SENSOR_CONFIG)
    assert len(sc) == 6


def test_servo_mixer_empty(fc):
    assert ask(fc, MSP2_INAV_SERVO_MIXER) == b""


def test_adsb_empty(fc):
    adsb = ask(fc, MSP2_ADSB_VEHICLE_LIST)
    assert len(adsb) == 10
    assert adsb[0] == 0  # нет объектов


def test_gps_statistics(fc):
    st = ask(fc, MSP_GPSSTATISTICS)
    assert len(st) == 21  # 2+4+4+4+2+2+2+1


def test_osd_layout_counts(fc):
    head = ask(fc, MSP2_INAV_OSD_LAYOUTS)
    assert head == bytes([OSD_LAYOUT_COUNT, OSD_ITEM_COUNT])


def test_osd_layout_items(fc):
    layout = ask(fc, MSP2_INAV_OSD_LAYOUTS, bytes([0]))
    assert len(layout) == OSD_ITEM_COUNT * 2
    # запись одного элемента
    pos = 0x2000 | (5 << 6) | 7
    ask(fc, MSP2_INAV_OSD_SET_LAYOUT_ITEM, bytes([0, 3]) + struct.pack("<H", pos))
    back = ask(fc, MSP2_INAV_OSD_LAYOUTS, bytes([0, 3, 0]))
    assert struct.unpack("<H", back)[0] == pos


def test_osd_layout_bad_index(fc):
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_OSD_LAYOUTS, bytes([OSD_LAYOUT_COUNT]))


def test_osd_alarms_roundtrip(fc):
    alarms = ask(fc, MSP2_INAV_OSD_ALARMS)
    assert len(alarms) == 28
    changed = bytearray(alarms)
    struct.pack_into("<H", changed, 1, 45)  # fly_minutes
    ask(fc, MSP2_INAV_OSD_SET_ALARMS, bytes(changed))
    assert struct.unpack_from("<H", ask(fc, MSP2_INAV_OSD_ALARMS), 1)[0] == 45


def test_osd_preferences_roundtrip(fc):
    prefs = ask(fc, MSP2_INAV_OSD_PREFERENCES)
    assert len(prefs) == 10
    changed = bytearray(prefs)
    changed[0] = 3  # video_system
    ask(fc, MSP2_INAV_OSD_SET_PREFERENCES, bytes(changed))
    assert ask(fc, MSP2_INAV_OSD_PREFERENCES)[0] == 3


def test_custom_osd_and_logic_mask(fc):
    assert ask(fc, MSP2_INAV_CUSTOM_OSD_ELEMENTS) == b""
    assert ask(fc, MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED) == b"\x00" * 8


def test_waypoint_stub(fc):
    info = ask(fc, MSP_WP_GETINFO)
    assert len(info) == 4
    wp = ask(fc, MSP_WP, bytes([1]))
    assert len(wp) == 21  # u8+u8+i32+i32+i32+i16+i16+i16+u8
    assert ask(fc, MSP_SET_WP, wp) == b""
