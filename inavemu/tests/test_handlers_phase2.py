"""Обработчики фазы 2: PID, ez-tune, датчики, GPS, OSD, миссии."""
import struct

import pytest

from inavemu import msp
from inavemu.fc import (
    MSP2_ADSB_VEHICLE_LIST, MSP2_COMMON_MOTOR_MIXER, MSP2_COMMON_SET_MOTOR_MIXER,
    MSP2_INAV_CUSTOM_OSD_ELEMENT, MSP2_INAV_CUSTOM_OSD_ELEMENTS,
    MSP2_INAV_EZ_TUNE, MSP2_INAV_EZ_TUNE_SET, MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED,
    MSP2_INAV_MIXER,
    MSP2_INAV_OSD_ALARMS, MSP2_INAV_OSD_LAYOUTS, MSP2_INAV_OSD_PREFERENCES,
    MSP2_INAV_OSD_SET_ALARMS, MSP2_INAV_OSD_SET_LAYOUT_ITEM, MSP2_INAV_OSD_SET_PREFERENCES,
    MSP2_INAV_RATE_DYNAMICS, MSP2_INAV_SELECT_BATTERY_PROFILE, MSP2_INAV_SERVO_MIXER,
    MSP2_INAV_SET_MIXER, MSP2_INAV_SET_RATE_DYNAMICS, MSP2_INAV_SET_SERVO_MIXER,
    MSP2_PID, MSP2_SET_PID, MSP_GPSSTATISTICS, MSP_SENSOR_CONFIG,
    MSP_SET_SENSOR_CONFIG,
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
    # стенд оснащён: кнопки калибровки магнитометра/опфлоу не блокируются
    assert sc[0] != 0 and sc[1] != 0 and sc[2] != 0
    assert sc[3] != 0 and sc[4] != 0 and sc[5] != 0
    # GET читает настройки *_hardware (автодетекция выписала чипы)
    assert sc[0] == 2      # ACC_MPU6000
    assert sc[1] == 3      # BARO_MS5611
    assert sc[2] == 2      # MAG_HMC5883
    # SET пишет сквозняком в настройки
    ask(fc, MSP_SET_SENSOR_CONFIG, bytes([5, 6, 7, 2, 3, 1]))
    assert ask(fc, MSP_SENSOR_CONFIG) == bytes([5, 6, 7, 2, 3, 1])
    assert fc.store.get("mag_hardware") == 7


def test_set_mixer_format(fc):
    # crunch конфигуратора: 5×u8 + u16 + 2 заглушки; прошивка требует ровно 9 байт
    p = struct.pack("<bBBbbhBB", -1, 0, 1, 2, 1, 14, 0, 0)
    assert ask(fc, MSP2_INAV_SET_MIXER, p) == b""
    assert fc.mixer["yaw_motor_direction"] == -1
    assert fc.mixer["platform_type"] == 2
    assert fc.mixer["has_flaps"] == 1
    assert fc.mixer["applied_mixer_preset"] == 14
    mix = ask(fc, MSP2_INAV_MIXER)
    assert mix[7] == 12 and mix[8] == 18  # ёмкости не «съедаются» заглушками SET
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_MIXER, p[:8])  # не 9 байт


def test_servo_mixer_empty(fc):
    # прошивка всегда отдаёт оба mixer-профиля: 2 × MAX_SERVO_RULES строк по 6 байт
    data = ask(fc, MSP2_INAV_SERVO_MIXER)
    assert data == b"\x00" * (6 * 36 * 2)


def test_servo_mixer_roundtrip(fc):
    row = bytes([3, 2, 0x10, 0x27, 5, 7])  # target, input, rate=10000, speed, condition
    assert ask(fc, MSP2_INAV_SET_SERVO_MIXER, bytes([1]) + row) == b""  # idx=1, 7 байт
    data = ask(fc, MSP2_INAV_SERVO_MIXER)
    assert data[6:12] == row  # слот 1
    assert data[0:6] == b"\x00" * 6  # слот 0 пуст
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_SERVO_MIXER, b"\x00" * 6)  # не 7 байт
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_SERVO_MIXER, bytes([36]) + row)  # idx >= 36


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


def test_osd_default_layout(fc):
    """Заводская раскладка прошивки: layout 0 из pgResetFn_osdLayoutsConfig,
    раскладки 1..3 пустые (всё скрыто) — как на новом FC."""
    l0 = ask(fc, MSP2_INAV_OSD_LAYOUTS, bytes([0]))
    vals = struct.unpack(f"<{OSD_ITEM_COUNT}H", l0)
    visible = [i for i, v in enumerate(vals) if v & 0x2000]
    assert visible == [0, 1, 7, 9, 11, 12, 14, 15, 28, 30]
    assert vals[0] == 0x2000 | (0 << 6) | 23  # OSD_RSSI_VALUE: POS(23,0)|VISIBLE
    assert vals[1] == 0x2000 | (0 << 6) | 12  # OSD_MAIN_BATT_VOLTAGE: POS(12,0)|VISIBLE
    for layout in (1, 2, 3):
        data = ask(fc, MSP2_INAV_OSD_LAYOUTS, bytes([layout]))
        assert data == b"\x00" * (OSD_ITEM_COUNT * 2)


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
    # не пустой payload: пустой ответ заставляет конфигуратор пропустить
    # колбэки (return из processData) и повесить цепочку загрузки OSD
    assert ask(fc, MSP2_INAV_CUSTOM_OSD_ELEMENTS) == bytes([0, 0, 0])
    element = ask(fc, MSP2_INAV_CUSTOM_OSD_ELEMENT, bytes([0]))
    assert len(element) == 3  # visibility: u8 + u16
    assert ask(fc, MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED) == b"\x00" * 8


def test_select_battery_profile_and_motor_mixer(fc):
    assert ask(fc, MSP2_INAV_SELECT_BATTERY_PROFILE, bytes([2])) == b""
    assert fc.battery_profile == 2
    fc.set_motors_connected(1)  # один мотор, остальные правила пустые
    row = bytes([0]) + struct.pack("<HHHH", 3000, 1900, 2100, 2050)  # idx + 4×u16
    assert ask(fc, MSP2_COMMON_SET_MOTOR_MIXER, row) == b""
    mm = ask(fc, MSP2_COMMON_MOTOR_MIXER)
    # GET: оба профиля × 12 строк по 8 байт (без индекса);
    # пустые правила = 0.0 в кодировке прошивки (2000), а не нули
    empty = struct.pack("<HHHH", 2000, 2000, 2000, 2000)
    assert mm == row[1:9] + empty * 11 + empty * 12
    with pytest.raises(MSPError):
        ask(fc, MSP2_COMMON_SET_MOTOR_MIXER, b"\x00" * 8)  # не 9 байт
    with pytest.raises(MSPError):
        ask(fc, MSP2_COMMON_SET_MOTOR_MIXER, bytes([12]) + row[1:9])  # idx >= 12


def test_waypoint_stub(fc):
    info = ask(fc, MSP_WP_GETINFO)
    assert len(info) == 4
    wp = ask(fc, MSP_WP, bytes([1]))
    assert len(wp) == 21  # u8+u8+i32+i32+i32+i16+i16+i16+u8
    assert ask(fc, MSP_SET_WP, wp) == b""
