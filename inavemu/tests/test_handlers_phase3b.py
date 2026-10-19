"""Обработчики фазы 3b: остатки вкладок Configurator — LED strip, фильтры,
PID advanced, навигация (poshold/RTH/FW), 3D, калибровки, servos, выходы,
timer modes, PG list, raw RC/GPS, RC tuning.

Форматы сверены с fc_msp.c INAV 9.1.0 (все SET — строгая длина, как в
прошивке).
"""
import struct

import pytest

from inavemu import msp
from inavemu.fc import (
    MSP2_COMMON_PG_LIST, MSP2_INAV_GPS_UBLOX_COMMAND, MSP2_INAV_LED_STRIP_CONFIG_EX,
    MSP2_INAV_MC_BRAKING, MSP2_INAV_OPFLOW_CALIBRATION,
    MSP2_INAV_OUTPUT_MAPPING, MSP2_INAV_OUTPUT_MAPPING_EXT2,
    MSP2_INAV_SELECT_MIXER_PROFILE, MSP2_INAV_SERVO_CONFIG,
    MSP2_INAV_SET_LED_STRIP_CONFIG_EX, MSP2_INAV_SET_MC_BRAKING,
    MSP2_INAV_SET_SERVO_CONFIG, MSP2_INAV_SET_TIMER_OUTPUT_MODE,
    MSP2_INAV_TIMER_OUTPUT_MODE,
    MSP_CALIBRATION_DATA, MSP_FILTER_CONFIG, MSP_FW_CONFIG, MSP_LED_COLORS,
    MSP_LED_STRIP_CONFIG, MSP_LED_STRIP_MODECOLOR, MSP_NAV_POSHOLD,
    MSP_OSD_CHAR_WRITE, MSP_PID_ADVANCED, MSP_RTH_AND_LAND_CONFIG,
    MSP_SET_3D, MSP_SET_CALIBRATION_DATA, MSP_SET_FILTER_CONFIG,
    MSP_SET_FW_CONFIG, MSP_SET_LED_COLORS, MSP_SET_LED_STRIP_CONFIG,
    MSP_SET_LED_STRIP_MODECOLOR, MSP_SET_NAV_POSHOLD, MSP_SET_PID_ADVANCED,
    MSP_SET_RAW_GPS, MSP_SET_RAW_RC, MSP_SET_RC_TUNING,
    MSP_SET_RTH_AND_LAND_CONFIG, MSP_3D,
    MSPError, InavFC,
)


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


def test_3d_roundtrip(fc):
    assert ask(fc, MSP_3D) == bytes(6)
    payload = struct.pack("<HHH", 50, 60, 1500)
    assert ask(fc, MSP_SET_3D, payload) == b""
    assert ask(fc, MSP_3D) == payload


def test_filter_config_roundtrip(fc):
    raw = ask(fc, MSP_FILTER_CONFIG)
    assert len(raw) == 23
    new = (bytes([33]) + struct.pack("<HHHHHHHHHHH", 200, 150, 0, 1, 0, 1,
                                     0, 1, 40, 30, 0))
    assert ask(fc, MSP_SET_FILTER_CONFIG, new) == b""
    assert ask(fc, MSP_FILTER_CONFIG) == new
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_FILTER_CONFIG, new[:22])


def test_pid_advanced_roundtrip(fc):
    raw = ask(fc, MSP_PID_ADVANCED)
    assert len(raw) == 17
    new = bytes(13) + struct.pack("<HH", 15, 25)
    assert ask(fc, MSP_SET_PID_ADVANCED, new) == b""
    assert ask(fc, MSP_PID_ADVANCED) == new


def test_nav_poshold_roundtrip(fc):
    raw = ask(fc, MSP_NAV_POSHOLD)
    assert len(raw) == 13
    new = struct.pack("<BHHHHBBH", 1, 300, 250, 400, 200, 35, 1, 1450)
    assert ask(fc, MSP_SET_NAV_POSHOLD, new) == b""
    assert ask(fc, MSP_NAV_POSHOLD) == new


def test_rth_and_land_roundtrip(fc):
    raw = ask(fc, MSP_RTH_AND_LAND_CONFIG)
    assert len(raw) == 21
    new = struct.pack("<HBBBBBHHHHHHH", 250, 1, 1, 1, 1, 1,
                      550, 1200, 200, 100, 500, 2000, 550)
    assert ask(fc, MSP_SET_RTH_AND_LAND_CONFIG, new) == b""
    assert ask(fc, MSP_RTH_AND_LAND_CONFIG) == new


def test_fw_config_roundtrip(fc):
    raw = ask(fc, MSP_FW_CONFIG)
    assert len(raw) == 12
    new = struct.pack("<HHHBBBBH", 1600, 1300, 1900, 40, 25, 15, 12, 2500)
    assert ask(fc, MSP_SET_FW_CONFIG, new) == b""
    assert ask(fc, MSP_FW_CONFIG) == new


def test_calibration_roundtrip(fc):
    raw = ask(fc, MSP_CALIBRATION_DATA)
    assert len(raw) == 27, "JS читает magGain до байта 26 — нужно 27 байт"
    assert raw[0] == 0x3F  # все оси откалиброваны
    # SET: 6×u16 acc + 3×u16 mag + u16 opflow + 3×u16 magGain — 26 байт
    body = (struct.pack("<HHHHHHHHH", 1, 2, 3, 400, 500, 600,
                        -10 & 0xFFFF, 20, 30)
            + struct.pack("<H", 512) + struct.pack("<HHH", 1100, 1200, 1300))
    assert len(body) == 26
    assert ask(fc, MSP_SET_CALIBRATION_DATA, body) == b""
    assert ask(fc, MSP_CALIBRATION_DATA) == bytes([0x3F]) + body
    # как в JS: чтение по фиксированным смещениям до байта 25
    reply = ask(fc, MSP_CALIBRATION_DATA)
    struct.unpack_from("<hhhhhhhhhhh", reply, 1)  # 11×i16 не падает
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_CALIBRATION_DATA, body[:17])


def test_led_strip_roundtrip(fc):
    # цвета: 16 × 4
    colors = b"".join(struct.pack("<HBB", i * 10, 200, 250) for i in range(16))
    assert ask(fc, MSP_SET_LED_COLORS, colors) == b""
    assert ask(fc, MSP_LED_COLORS) == colors
    # расширенный конфиг: 128 × 5 байт
    assert len(ask(fc, MSP2_INAV_LED_STRIP_CONFIG_EX)) == 128 * 5
    led = bytes([1, 2, 0x10, 0x02, 7])
    assert ask(fc, MSP2_INAV_SET_LED_STRIP_CONFIG_EX, bytes([3]) + led) == b""
    ex = ask(fc, MSP2_INAV_LED_STRIP_CONFIG_EX)
    assert ex[3 * 5:4 * 5] == led
    # легаси: [idx][u32]
    legacy = struct.pack("<I", 0x0A0B0C0D)
    assert ask(fc, MSP_SET_LED_STRIP_CONFIG, bytes([4]) + legacy) == b""
    assert ask(fc, MSP_LED_STRIP_CONFIG)[16:20] == legacy
    # mode colors: 51 × 3
    assert len(ask(fc, MSP_LED_STRIP_MODECOLOR)) == 51 * 3
    assert ask(fc, MSP_SET_LED_STRIP_MODECOLOR, bytes([0, 1, 5])) == b""
    assert ask(fc, MSP_LED_STRIP_MODECOLOR)[5] == 5  # запись (0,1): цвет в байте 5
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_LED_COLORS, colors[:63])


def test_osd_char_write(fc):
    assert ask(fc, MSP_OSD_CHAR_WRITE, bytes(55)) == b""
    with pytest.raises(MSPError):
        ask(fc, MSP_OSD_CHAR_WRITE, bytes(54))


def test_servo_config_roundtrip(fc):
    raw = ask(fc, MSP2_INAV_SERVO_CONFIG)
    assert len(raw) == 18 * 7
    payload = struct.pack("<HHHB", 1100, 1900, 1550, 90)
    assert ask(fc, MSP2_INAV_SET_SERVO_CONFIG, bytes([2]) + payload) == b""
    got = ask(fc, MSP2_INAV_SERVO_CONFIG)
    assert got[14:21] == payload
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_SERVO_CONFIG, bytes([18]) + payload)


def test_output_mapping(fc):
    ext = ask(fc, MSP2_INAV_OUTPUT_MAPPING_EXT2)
    assert len(ext) == 8 * 6
    assert ask(fc, MSP2_INAV_OUTPUT_MAPPING) == bytes(
        [flags & 0xFF for _, flags, _ in fc.output_mapping])


def test_mc_braking_roundtrip(fc):
    raw = ask(fc, MSP2_INAV_MC_BRAKING)
    assert len(raw) == 14
    new = struct.pack("<HHHBHHHB", 200, 100, 500, 5, 300, 400, 150, 20)
    assert ask(fc, MSP2_INAV_SET_MC_BRAKING, new) == b""
    assert ask(fc, MSP2_INAV_MC_BRAKING) == new


def test_timer_output_mode(fc):
    pairs = ask(fc, MSP2_INAV_TIMER_OUTPUT_MODE)
    assert len(pairs) == 8 * 2
    assert ask(fc, MSP2_INAV_TIMER_OUTPUT_MODE, bytes([2])) == bytes([2, 0])
    assert ask(fc, MSP2_INAV_SET_TIMER_OUTPUT_MODE, bytes([2, 3])) == b""
    assert ask(fc, MSP2_INAV_TIMER_OUTPUT_MODE, bytes([2])) == bytes([2, 3])
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_TIMER_OUTPUT_MODE, bytes([8, 0]))


def test_select_mixer_profile(fc):
    assert ask(fc, MSP2_INAV_SELECT_MIXER_PROFILE, bytes([1])) == b""
    assert fc.mixer_profile == 1


def test_gps_ublox_command(fc):
    # без UBLOX — ошибка; с UBLOX и >= 8 байт — ок
    fc.store.set("gps_provider", 1)  # MSP — не UBLOX
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_GPS_UBLOX_COMMAND, bytes(8))
    fc.store.set("gps_provider", 0)  # UBLOX
    assert ask(fc, MSP2_INAV_GPS_UBLOX_COMMAND, bytes(8)) == b""
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_GPS_UBLOX_COMMAND, bytes(7))


def test_pg_list(fc):
    out = ask(fc, MSP2_COMMON_PG_LIST)
    assert len(out) % 6 == 0 and len(out) > 0
    first_pg = int.from_bytes(out[:2], "little")
    # запрос одного PG: [first u16] -> last = first
    one = ask(fc, MSP2_COMMON_PG_LIST, struct.pack("<H", first_pg))
    assert one[:2] == struct.pack("<H", first_pg)
    assert len(one) == 6


def test_raw_rc_and_gps(fc):
    frame = struct.pack("<HHH", 1200, 1500, 1800)
    assert ask(fc, MSP_SET_RAW_RC, frame) == b""
    assert fc.rc_channels[:3] == [1200, 1500, 1800]
    gps = bytes([2, 12]) + struct.pack("<IIHH", 55755830, 37617380, 160, 250)
    assert len(gps) == 14
    assert ask(fc, MSP_SET_RAW_GPS, gps) == b""
    assert fc.gps["fix"] == 2 and fc.gps["sats"] == 12
    assert fc.gps["lat"] == 55755830


def test_rc_tuning_set(fc):
    payload = bytes([100, 20, 30, 40, 50, 60, 70, 80]) + struct.pack("<H", 1200)
    assert ask(fc, MSP_SET_RC_TUNING, payload) == b""
    assert fc.rate_profile["roll_rate"] == 300   # ×10 как в GET
    assert fc.rate_profile["pa_breakpoint"] == 1200
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_RC_TUNING, payload[:9])
