"""Обработчики MSP: рукопожатие, вкладки, SET-команды, ответ-ошибка.

Длины и порядок полей сверены с декодером js/msp/MSPHelper.js (Configurator 9.1.1).
"""
import struct

import pytest

from inavemu import msp
from inavemu.fc import (
    MSP2_CF_SERIAL_CONFIG, MSP2_COMMON_SETTING, MSP2_COMMON_SETTING_INFO,
    MSP2_INAV_ANALOG, MSP2_INAV_BATTERY_CONFIG, MSP2_INAV_MISC, MSP2_INAV_MIXER,
    MSP2_INAV_RATE_PROFILE, MSP2_INAV_STATUS, MSP_ACTIVEBOXES, MSP_ADVANCED_CONFIG,
    MSP_API_VERSION, MSP_BOARD_INFO, MSP_BOARD_ALIGNMENT, MSP_BOXIDS,
    MSP_BUILD_INFO, MSP_CURRENT_METER_CONFIG, MSP_DATAFLASH_SUMMARY, MSP_EEPROM_WRITE,
    MSP_FC_VARIANT, MSP_FC_VERSION, MSP_FEATURE, MSP_MODE_RANGES, MSP_NAME,
    MSP_RC, MSP_RC_DEADBAND, MSP_RX_CONFIG, MSP_RX_MAP, MSP_SENSOR_ALIGNMENT,
    MSP_SENSOR_STATUS, MSP_SET_FEATURE, MSP_SET_MODE_RANGE, MSP_SET_REBOOT,
    MSP_UID, MSP_VTX_CONFIG, MSPError, InavFC,
)


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b"", version=2):
    req = msp.Request(version=version, code=code, payload=payload)
    return fc.handle(req)


# ---------------- рукопожатие ----------------

def test_handshake_identity(fc):
    api = ask(fc, MSP_API_VERSION)
    assert api[0] == 0 and api[1] == 2 and api[2] == 5  # MSP API 2.5
    assert ask(fc, MSP_FC_VARIANT) == b"INAV"
    assert ask(fc, MSP_FC_VERSION) == bytes([9, 1, 0])
    build = ask(fc, MSP_BUILD_INFO)
    assert len(build) >= 19  # 11 + 8 [+ 8 git]
    board = ask(fc, MSP_BOARD_INFO)
    assert board[:4] == b"SITL"
    assert board[6] == 2  # osdUsed
    assert board[7] & 1  # VCP
    uid = ask(fc, MSP_UID)
    assert len(uid) == 12


def test_board_info_target_name(fc):
    fc.target = "MATEKF405SE"
    board = ask(fc, MSP_BOARD_INFO)
    tlen = board[8]
    assert board[9:9 + tlen] == b"MATEKF405SE"


def test_fc_version_flag():
    fc = InavFC(fc_version="9.0.1")
    assert ask(fc, MSP_FC_VERSION) == bytes([9, 0, 1])


def test_box_ids(fc):
    boxes = ask(fc, MSP_BOXIDS)
    assert len(boxes) == len(fc.store.boxes) == 55
    assert boxes[0] == 0  # ARM permanent_id


def test_dataflash_summary(fc):
    d = ask(fc, MSP_DATAFLASH_SUMMARY)
    assert len(d) == 13
    assert d[0] & 1  # ready


# ---------------- периодика и датчики ----------------

def test_inav_status_layout(fc):
    st = ask(fc, MSP2_INAV_STATUS)
    assert len(st) == 22  # 2+2+2+2+1+4+8+1
    cycle = struct.unpack_from("<H", st, 0)[0]
    assert cycle == 1000
    assert st[21] == 0  # mixer profile


def test_sensor_status(fc):
    assert len(ask(fc, MSP_SENSOR_STATUS)) == 9


def test_active_boxes(fc):
    assert len(ask(fc, MSP_ACTIVEBOXES)) == 8


def test_analog_layout(fc):
    a = ask(fc, MSP2_INAV_ANALOG)
    assert len(a) == 24
    cells = (a[0] >> 4) & 0x0F
    assert cells == 3
    voltage = struct.unpack_from("<H", a, 1)[0]
    assert 1000 < voltage < 1300  # ~12.6 В


def test_raw_imu_attitude_altitude(fc):
    from inavemu.fc import MSP_ALTITUDE, MSP_ATTITUDE, MSP_RAW_IMU
    imu = ask(fc, MSP_RAW_IMU)
    assert len(imu) == 18
    att = ask(fc, MSP_ATTITUDE)
    assert len(att) == 6
    alt = ask(fc, MSP_ALTITUDE)
    assert len(alt) == 10  # i32 + u16 + i32


# ---------------- вкладки ----------------

def test_misc_layout(fc):
    misc = ask(fc, MSP2_INAV_MISC)
    assert len(misc) == 41  # 2+2+2+2+2 +1+1+1+1 +2 +2 +1+1 +2+2+2+2 +4+4+4 +1
    midrc = struct.unpack_from("<h", misc, 0)[0]
    assert midrc == 1500


def test_misc_set_roundtrip(fc):
    from inavemu.fc import MSP2_INAV_SET_MISC
    misc = bytearray(ask(fc, MSP2_INAV_MISC))
    struct.pack_into("<h", misc, 0, 1490)  # midrc
    struct.pack_into("<H", misc, 10, 777)  # vbatscale
    ask(fc, MSP2_INAV_SET_MISC, bytes(misc))
    back = ask(fc, MSP2_INAV_MISC)
    assert struct.unpack_from("<h", back, 0)[0] == 1490
    assert struct.unpack_from("<H", back, 10)[0] == 777


def test_battery_config_layout(fc):
    bat = ask(fc, MSP2_INAV_BATTERY_CONFIG)
    assert len(bat) == 29


def test_mixer_layout(fc):
    mix = ask(fc, MSP2_INAV_MIXER)
    assert len(mix) == 9
    # прошивка отдаёт константы ёмкости generic-таргета, не фактические счётчики
    assert mix[7] == 12  # MAX_SUPPORTED_MOTORS
    assert mix[8] == 18  # MAX_SUPPORTED_SERVOS


def test_serial_config_layout(fc):
    ports = ask(fc, MSP2_CF_SERIAL_CONFIG)
    assert len(ports) == len(fc.serial_ports) * 9
    ident = ports[0]
    assert ident == 0  # UART1
    funcs = struct.unpack_from("<I", ports, 1)[0]
    assert funcs & 1  # MSP


def test_rx_config_layout(fc):
    rx = ask(fc, MSP_RX_CONFIG)
    assert len(rx) == 24
    assert struct.unpack_from("<H", rx, 3)[0] == 1500  # midrc


def test_rate_profile_layout(fc):
    rp = ask(fc, MSP2_INAV_RATE_PROFILE)
    assert len(rp) == 15


def test_mode_ranges(fc):
    mr = ask(fc, MSP_MODE_RANGES)
    assert len(mr) == 40 * 4


def test_rc_map_and_deadband(fc):
    assert ask(fc, MSP_RX_MAP) == bytes([0, 1, 2, 3, 4, 5, 6, 7])
    db = ask(fc, MSP_RC_DEADBAND)
    assert len(db) == 5


def test_alignment_and_advanced(fc):
    assert len(ask(fc, MSP_SENSOR_ALIGNMENT)) == 4
    assert len(ask(fc, MSP_ADVANCED_CONFIG)) == 9
    assert len(ask(fc, MSP_BOARD_ALIGNMENT)) == 6
    assert len(ask(fc, MSP_CURRENT_METER_CONFIG)) == 7
    assert len(ask(fc, MSP_VTX_CONFIG)) >= 7


def test_rc_channels(fc):
    rc = ask(fc, MSP_RC)
    assert len(rc) == 16 * 2
    assert struct.unpack_from("<H", rc, 0)[0] == 1500


# ---------------- настройки ----------------

def test_setting_info_and_get(fc):
    info = ask(fc, MSP2_COMMON_SETTING_INFO, b"applied_defaults\x00")
    assert info.startswith(b"applied_defaults\x00")
    val = ask(fc, MSP2_COMMON_SETTING, b"applied_defaults\x00")
    assert val == bytes([0])
    # по индексу: имя + NUL, pg u16, type u8, section u8, mode u8, min i32, max u32, index u16
    name_len = len(b"applied_defaults\x00")
    idx_off = name_len + 2 + 1 + 1 + 1 + 4 + 4
    idx = struct.unpack_from("<H", info, idx_off)[0]
    val2 = ask(fc, MSP2_COMMON_SETTING, bytes([0]) + struct.pack("<H", idx))
    assert val2 == val


def test_setting_unknown(fc):
    with pytest.raises(MSPError):
        ask(fc, MSP2_COMMON_SETTING_INFO, b"nope\x00")


# ---------------- SET и служебные ----------------

def test_set_feature_and_save(fc, tmp_path):
    fc.eeprom_path = tmp_path / "eeprom.json"
    ask(fc, MSP_SET_FEATURE, struct.pack("<I", 0b1000))
    assert fc.features == 0b1000
    ask(fc, MSP_EEPROM_WRITE)
    fc2 = InavFC(eeprom_path=fc.eeprom_path)
    assert fc2.features == 0b1000


def test_set_mode_range(fc):
    ask(fc, MSP_SET_MODE_RANGE, bytes([3, 1, 40, 80]))
    mr = fc.mode_ranges[3]
    assert mr["aux"] == 1
    assert mr["start"] == 900 + 40 * 25
    assert mr["end"] == 900 + 80 * 25


def test_reboot_flag(fc, tmp_path):
    fc.eeprom_path = tmp_path / "e.json"
    ask(fc, MSP_SET_REBOOT)
    assert fc.reboot_requested


def test_unknown_code_is_error(fc):
    with pytest.raises(MSPError):
        ask(fc, 0x7777)
