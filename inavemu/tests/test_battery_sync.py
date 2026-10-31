"""Синхронизация батарейного блока MISC/BATTERY_CONFIG с базой настроек.

Как и в прошивке (fc_msp.c MSP2_INAV_MISC/MSP2_INAV_SET_MISC), провод
MSP2_INAV_MISC/BATTERY_CONFIG и CLI-настройки (bat_voltage_src, vbat_*,
battery_capacity_*, failsafe_throttle, mag_declination, min_command,
current_meter_*) — одно и то же: CLI `set` переключает поля вкладки
Configuration, а сохранение вкладки меняет значения CLI.

Единицы провода сверены с MSPHelper.js Configurator 9.1.1:
vbat*cellvoltage — сотые вольта (425 = 4.25 В), mag_declination — десятые
градуса (настройка — сотые).
"""
import struct

import pytest

from inavemu.cli import CliSession
from inavemu.fc import (
    MSP2_INAV_ANALOG, MSP2_INAV_BATTERY_CONFIG, MSP2_INAV_MISC,
    MSP2_INAV_SET_BATTERY_CONFIG, MSP2_INAV_SET_MISC,
    MSP_EEPROM_WRITE, MSPError, InavFC,
)
from inavemu import msp

# смещения полей в MSP2_INAV_MISC (41 байт)
MIS = {
    "mincommand": 6, "failsafe_throttle": 8, "mag_declination": 14,
    "vbatscale": 16, "voltage_source": 18, "battery_cells": 19,
    "vbatdetectcellvoltage": 20, "vbatmincellvoltage": 22,
    "vbatmaxcellvoltage": 24, "vbatwarningcellvoltage": 26,
    "battery_capacity": 28, "battery_capacity_warning": 32,
    "battery_capacity_critical": 36, "battery_capacity_unit": 40,
}
# смещения полей в MSP2_INAV_BATTERY_CONFIG (29 байт)
BAT = {
    "vbatscale": 0, "voltage_source": 2, "battery_cells": 3,
    "vbatdetectcellvoltage": 4, "vbatmincellvoltage": 6,
    "vbatmaxcellvoltage": 8, "vbatwarningcellvoltage": 10,
    "current_offset": 12, "current_scale": 14,
    "capacity": 16, "capacity_warning": 20,
    "capacity_critical": 24, "capacity_unit": 28,
}


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b"", version=2):
    return fc.handle(msp.Request(version=version, code=code, payload=payload))


def u16(frame, offset):
    return struct.unpack_from("<H", frame, offset)[0]


def u8(frame, offset):
    return frame[offset]


def patch(frame, offset, fmt, value):
    struct.pack_into(fmt, frame, offset, value)


# ---------------- дефолты: вкладка и CLI показывают одно и то же ----------------

def test_misc_battery_block_matches_store_defaults(fc):
    misc = ask(fc, MSP2_INAV_MISC)
    st = fc.store
    assert u8(misc, MIS["voltage_source"]) == st.get("bat_voltage_src") == 0
    assert u16(misc, MIS["vbatscale"]) == st.get("vbat_scale") == 1100
    assert u8(misc, MIS["battery_cells"]) == st.get("bat_cells") == 0
    assert u16(misc, MIS["vbatdetectcellvoltage"]) == 425
    assert u16(misc, MIS["vbatmincellvoltage"]) == 330
    assert u16(misc, MIS["vbatmaxcellvoltage"]) == 420
    assert u16(misc, MIS["vbatwarningcellvoltage"]) == 350
    assert u16(misc, MIS["failsafe_throttle"]) == st.get("failsafe_throttle") == 1000
    assert u16(misc, MIS["mincommand"]) == st.get("min_command") == 1000
    assert struct.unpack_from("<h", misc, MIS["mag_declination"])[0] == 0


def test_battery_config_matches_store_defaults(fc):
    bat = ask(fc, MSP2_INAV_BATTERY_CONFIG)
    assert u16(bat, BAT["vbatscale"]) == 1100
    assert u8(bat, BAT["voltage_source"]) == 0
    assert u16(bat, BAT["current_scale"]) == fc.store.get("current_meter_scale") == 400
    assert u16(bat, BAT["current_offset"]) == 0


def test_cli_get_shows_battery_defaults(fc):
    cli = CliSession(fc)
    cli.enter()
    text = cli.feed(b"get vbat_scale\r\n").decode()
    assert "vbat_scale = 1100" in text
    text = cli.feed(b"get bat_voltage_src\r\n").decode()
    assert "bat_voltage_src = RAW" in text


# ---------------- приём: CLI set → вкладка (MSP2_INAV_MISC) ----------------

def test_cli_set_bat_voltage_src_switches_misc(fc):
    # «set bat_voltage_src = SAG_COMP» должен переключить Voltage source
    cli = CliSession(fc)
    cli.enter()
    out = cli.feed(b"set bat_voltage_src = SAG_COMP\r\n").decode()
    assert "bat_voltage_src set to SAG_COMP" in out
    assert u8(ask(fc, MSP2_INAV_MISC), MIS["voltage_source"]) == 1
    assert u8(ask(fc, MSP2_INAV_BATTERY_CONFIG), BAT["voltage_source"]) == 1


def test_cli_set_battery_block_switches_misc(fc):
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set vbat_scale = 1234\r\n")
    cli.feed(b"set bat_cells = 4\r\n")
    cli.feed(b"set vbat_cell_detect_voltage = 430\r\n")
    cli.feed(b"set vbat_min_cell_voltage = 320\r\n")
    cli.feed(b"set battery_capacity = 3300\r\n")
    cli.feed(b"set battery_capacity_unit = MWH\r\n")
    misc = ask(fc, MSP2_INAV_MISC)
    assert u16(misc, MIS["vbatscale"]) == 1234
    assert u8(misc, MIS["battery_cells"]) == 4
    # сотые вольта без умножения на 100: 430 = 4.30 В, а не 43000
    assert u16(misc, MIS["vbatdetectcellvoltage"]) == 430
    assert u16(misc, MIS["vbatmincellvoltage"]) == 320
    assert struct.unpack_from("<I", misc, MIS["battery_capacity"])[0] == 3300
    assert u8(misc, MIS["battery_capacity_unit"]) == 1


def test_cli_set_failsafe_throttle_and_min_command(fc):
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set failsafe_throttle = 1600\r\n")
    cli.feed(b"set min_command = 1050\r\n")
    misc = ask(fc, MSP2_INAV_MISC)
    assert u16(misc, MIS["failsafe_throttle"]) == 1600
    assert u16(misc, MIS["mincommand"]) == 1050


# ---------------- обратно: сохранение вкладки → CLI ----------------

def test_set_misc_updates_cli_store(fc):
    misc = bytearray(ask(fc, MSP2_INAV_MISC))
    patch(misc, MIS["voltage_source"], "<B", 1)
    patch(misc, MIS["battery_cells"], "<B", 5)
    patch(misc, MIS["vbatdetectcellvoltage"], "<H", 425)
    assert ask(fc, MSP2_INAV_SET_MISC, bytes(misc)) == b""
    assert fc.store.get("bat_voltage_src") == 1
    assert fc.store.get("bat_cells") == 5
    assert fc.store.get("vbat_cell_detect_voltage") == 425
    cli = CliSession(fc)
    cli.enter()
    assert "bat_voltage_src = SAG_COMP" in cli.feed(b"get bat_voltage_src\r\n").decode()
    assert "bat_cells = 5" in cli.feed(b"get bat_cells\r\n").decode()


def test_set_misc_full_battery_block_roundtrip(fc):
    misc = bytearray(ask(fc, MSP2_INAV_MISC))
    values = {
        "vbatscale": ("<H", 1200), "voltage_source": ("<B", 1),
        "battery_cells": ("<B", 6), "vbatdetectcellvoltage": ("<H", 425),
        "vbatmincellvoltage": ("<H", 330), "vbatmaxcellvoltage": ("<H", 420),
        "vbatwarningcellvoltage": ("<H", 350), "battery_capacity": ("<I", 5000),
        "battery_capacity_warning": ("<I", 2500),
        "battery_capacity_critical": ("<I", 900),
        "battery_capacity_unit": ("<B", 1),
    }
    for key, (fmt, value) in values.items():
        patch(misc, MIS[key], fmt, value)
    ask(fc, MSP2_INAV_SET_MISC, bytes(misc))
    st = fc.store
    assert st.get("vbat_scale") == 1200
    assert st.get("bat_voltage_src") == 1
    assert st.get("bat_cells") == 6
    assert st.get("vbat_cell_detect_voltage") == 425
    assert st.get("vbat_min_cell_voltage") == 330
    assert st.get("vbat_max_cell_voltage") == 420
    assert st.get("vbat_warning_cell_voltage") == 350
    assert st.get("battery_capacity") == 5000
    assert st.get("battery_capacity_warning") == 2500
    assert st.get("battery_capacity_critical") == 900
    assert st.get("battery_capacity_unit") == 1
    # провод читает то же самое из базы
    back = ask(fc, MSP2_INAV_MISC)
    for key, (fmt, value) in values.items():
        assert struct.unpack_from(fmt, back, MIS[key])[0] == value


def test_set_battery_config_updates_store(fc):
    bat = bytearray(ask(fc, MSP2_INAV_BATTERY_CONFIG))
    patch(bat, BAT["voltage_source"], "<B", 1)
    patch(bat, BAT["current_offset"], "<H", 0xFFFF)  # -1 как int16
    patch(bat, BAT["current_scale"], "<H", 400)
    patch(bat, BAT["capacity"], "<I", 2200)
    assert ask(fc, MSP2_INAV_SET_BATTERY_CONFIG, bytes(bat)) == b""
    assert fc.store.get("bat_voltage_src") == 1
    assert fc.store.get("current_meter_offset") == -1
    assert fc.store.get("current_meter_scale") == 400
    assert fc.store.get("battery_capacity") == 2200
    # провод вернёт те же 16 бит
    assert u16(ask(fc, MSP2_INAV_BATTERY_CONFIG), BAT["current_offset"]) == 0xFFFF


def test_cli_set_current_meter_flows_to_battery_config(fc):
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set current_meter_scale = 550\r\n")
    cli.feed(b"set current_meter_offset = -3\r\n")
    bat = ask(fc, MSP2_INAV_BATTERY_CONFIG)
    assert u16(bat, BAT["current_scale"]) == 550
    assert struct.unpack_from("<h", bat, BAT["current_offset"])[0] == -3


# ---------------- mag_declination: провод в десятых градуса ----------------

def test_mag_declination_scaling(fc):
    assert fc.store.set("mag_declination", 1234)  # сотые градуса
    misc = ask(fc, MSP2_INAV_MISC)
    assert struct.unpack_from("<h", misc, MIS["mag_declination"])[0] == 123
    frame = bytearray(ask(fc, MSP2_INAV_MISC))
    patch(frame, MIS["mag_declination"], "<h", -55)
    ask(fc, MSP2_INAV_SET_MISC, bytes(frame))
    assert fc.store.get("mag_declination") == -550


# ---------------- валидация как в прошивке ----------------

def test_set_misc_invalid_voltage_source_errors(fc):
    misc = bytearray(ask(fc, MSP2_INAV_MISC))
    patch(misc, MIS["voltage_source"], "<B", 2)
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_MISC, bytes(misc))
    assert fc.store.get("bat_voltage_src") == 0  # сброшен как в прошивке


def test_set_misc_invalid_capacity_unit_errors(fc):
    misc = bytearray(ask(fc, MSP2_INAV_MISC))
    patch(misc, MIS["battery_capacity_unit"], "<B", 2)
    with pytest.raises(MSPError):
        ask(fc, MSP2_INAV_SET_MISC, bytes(misc))
    assert fc.store.get("battery_capacity_unit") == 0


# ---------------- MSP2_INAV_ANALOG: автоопределение ячеек ----------------

def test_analog_cell_autodetect(fc):
    analog = ask(fc, MSP2_INAV_ANALOG)
    # 12.60 В / 4.25 В + 1 = 3 ячейки (bat_cells = 0 — автоопределение)
    assert (analog[0] >> 4) & 0x0F == 3
    fc.store.set("bat_cells", 4)
    analog = ask(fc, MSP2_INAV_ANALOG)
    assert (analog[0] >> 4) & 0x0F == 4


# ---------------- eeprom: база настроек — источник истины ----------------

def test_eeprom_preserves_cli_and_tab_values(tmp_path):
    path = tmp_path / "eeprom.json"
    fc = InavFC(eeprom_path=path)
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set bat_voltage_src = SAG_COMP\r\n")
    ask(fc, MSP_EEPROM_WRITE)

    fc2 = InavFC(eeprom_path=path)
    assert fc2.store.get("bat_voltage_src") == 1
    assert u8(ask(fc2, MSP2_INAV_MISC), MIS["voltage_source"]) == 1
