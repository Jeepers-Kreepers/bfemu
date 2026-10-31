"""Урок 6: команды aux и adjrange — вывод всех слотов и разбор, как в cli.c INAV 9.1.0.

Прошивка печатает ВСЕ слоты (пустые тоже: 'aux 3 0 0 900 900'), диапазоны в
микросекундах; слот режима привязан к permanentId (поле mode), слот корректировки
несёт adjustmentIndex. При неполных аргументах слот обнуляется (как в cliAux()).
"""
import json

import pytest

from inavemu import msp
from inavemu.cli import CliSession
from inavemu.fc import (
    ADJUSTMENT_RANGE_COUNT,
    InavFC,
    MODE_ACTIVATION_CONDITION_COUNT,
    MSP_ADJUSTMENT_RANGES,
    MSPError,
    MSP_MODE_RANGES,
    MSP_SET_ADJUSTMENT_RANGE,
    MSP_SET_MODE_RANGE,
)


@pytest.fixture()
def fc():
    return InavFC()


@pytest.fixture()
def cli(fc):
    return CliSession(fc)


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


# ---------------- вывод: все слоты, как в прошивке ----------------

def test_aux_lists_all_slots(cli):
    lines = cli.feed(b"aux\r\n").decode().splitlines()
    slots = [ln for ln in lines if ln.startswith("aux ")]
    assert len(slots) == MODE_ACTIVATION_CONDITION_COUNT
    assert slots[0] == "aux 0 0 0 1700 2100", "сток: ARM на AUX1"
    assert slots[5] == "aux 5 0 0 900 900", "пустой слот тоже печатается"


def test_adjrange_lists_all_slots(cli):
    lines = cli.feed(b"adjrange\r\n").decode().splitlines()
    slots = [ln for ln in lines if ln.startswith("adjrange ")]
    assert len(slots) == ADJUSTMENT_RANGE_COUNT
    assert slots[0] == "adjrange 0 0 0 900 900 0 0"
    assert slots[3] == "adjrange 3 0 0 900 900 0 0"


# ---------------- aux: запись и вывод ----------------

def test_aux_set_shows_in_listing(cli):
    cli.feed(b"aux 0 0 0 900 2100\r\n")
    assert "aux 0 0 0 900 2100" in cli.feed(b"aux\r\n").decode()


def test_aux_set_mode_binding(cli, fc):
    cli.feed(b"aux 1 2 1 1300 1700\r\n")  # permanentId 2 = HORIZON
    assert "aux 1 2 1 1300 1700" in cli.feed(b"aux\r\n").decode()
    assert fc.mode_ranges[1]["mode"] == 2


def test_aux_set_snaps_to_25us_grid(cli):
    cli.feed(b"aux 2 0 0 912 2099\r\n")
    assert "aux 2 0 0 900 2075" in cli.feed(b"aux\r\n").decode()


def test_aux_incomplete_clears_slot(cli):
    cli.feed(b"aux 0 0 0 900 2100\r\n")
    cli.feed(b"aux 0 0 0\r\n")
    assert "aux 0 0 0 900 900" in cli.feed(b"aux\r\n").decode()


def test_aux_unknown_permanent_id_clears(cli):
    cli.feed(b"aux 0 99 0 900 2100\r\n")
    assert "aux 0 0 0 900 900" in cli.feed(b"aux\r\n").decode()


def test_aux_errors(cli):
    assert b"### ERROR: index must be between 0 and 39" in \
        cli.feed(b"aux 40 0 0 900 2100\r\n")
    assert b"### ERROR: Parse error" in cli.feed(b"aux 0 x y\r\n")


# ---------------- adjrange: запись и вывод ----------------

def test_adjrange_set_shows_in_listing(cli, fc):
    cli.feed(b"adjrange 0 1 2 1300 1700 3 4\r\n")
    assert "adjrange 0 1 2 1300 1700 3 4" in cli.feed(b"adjrange\r\n").decode()
    assert fc.adjustment_ranges[0] == {"slot": 0, "adjustmentIndex": 1, "aux": 2,
                                       "start": 1300, "end": 1700,
                                       "function": 3, "switch": 4}


def test_adjrange_incomplete_clears_and_errors(cli):
    cli.feed(b"adjrange 0 1 2 1300 1700 3 4\r\n")
    assert b"### ERROR: Parse error" in cli.feed(b"adjrange 0 1 2\r\n")
    assert "adjrange 0 0 0 900 900 0 0" in cli.feed(b"adjrange\r\n").decode()


def test_adjrange_errors(cli):
    assert b"### ERROR: index must be between 0 and 19" in \
        cli.feed(b"adjrange 20 0 0 900 2100 0 0\r\n")
    assert b"### ERROR: Parse error" in cli.feed(b"adjrange 0 x y\r\n")


# ---------------- MSP: слоты с mode/adjustmentIndex видны вкладкам ----------------

def test_msp_mode_ranges_wire(fc):
    fc.mode_ranges[0].update({"mode": 2, "aux": 1, "start": 900, "end": 2100})
    raw = ask(fc, MSP_MODE_RANGES)
    assert len(raw) == 40 * 4
    assert list(raw[:4]) == [2, 1, 0, 48], "permanentId, aux, startStep, endStep"


def test_msp_set_mode_range_wire(fc):
    ask(fc, MSP_SET_MODE_RANGE, bytes([3, 1, 7, 40, 80]))
    mr = fc.mode_ranges[3]
    assert (mr["mode"], mr["aux"]) == (1, 7)
    assert (mr["start"], mr["end"]) == (900 + 40 * 25, 900 + 80 * 25)
    raw = ask(fc, MSP_MODE_RANGES)
    assert list(raw[12:16]) == [1, 7, 40, 80]


def test_msp_set_mode_range_errors(fc):
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_MODE_RANGE, bytes([0, 0, 0, 0]))        # 4 байта
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_MODE_RANGE, bytes([99, 0, 0, 0, 0]))    # индекс
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_MODE_RANGE, bytes([0, 250, 0, 0, 0]))   # permanentId


def test_msp_adjustment_ranges_wire(fc):
    fc.adjustment_ranges[1].update({"adjustmentIndex": 2, "aux": 3,
                                    "start": 1300, "end": 1700,
                                    "function": 5, "switch": 6})
    raw = ask(fc, MSP_ADJUSTMENT_RANGES)
    assert len(raw) == 20 * 6
    assert list(raw[6:12]) == [2, 3, 16, 32, 5, 6], \
        "adjustmentIndex, aux, шаги, функция, switch"


def test_msp_set_adjustment_range_wire(fc):
    ask(fc, MSP_SET_ADJUSTMENT_RANGE, bytes([2, 1, 4, 10, 20, 7, 8]))
    ar = fc.adjustment_ranges[2]
    assert (ar["adjustmentIndex"], ar["aux"], ar["start"], ar["end"],
            ar["function"], ar["switch"]) == (1, 4, 900 + 10 * 25, 900 + 20 * 25, 7, 8)
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_ADJUSTMENT_RANGE, bytes([2, 1, 4, 10, 20, 7]))     # 6 байт
    with pytest.raises(MSPError):
        ask(fc, MSP_SET_ADJUSTMENT_RANGE, bytes([2, 9, 4, 10, 20, 7, 8]))  # adjIndex


# ---------------- dump/diff: секции # aux и # adjrange ----------------

def test_dump_sections(cli):
    cli.feed(b"aux 0 0 0 900 2100\r\n")
    text = cli.feed(b"dump\r\n").decode()
    lines = [ln for ln in text.split("# aux")[1].splitlines() if ln.startswith("aux ")]
    assert len(lines) == 40, "dump печатает все слоты"
    assert lines[0] == "aux 0 0 0 900 2100"
    assert "# adjrange" in text
    adj = [ln for ln in text.split("# adjrange")[1].splitlines()
           if ln.startswith("adjrange ")]
    assert len(adj) == 20


def test_diff_sections_show_only_changed(cli):
    cli.feed(b"aux 0 0 0 900 2100\r\n")
    text = cli.feed(b"diff\r\n").decode()
    lines = [ln for ln in text.split("# aux")[1].splitlines() if ln.startswith("aux ")]
    assert lines == ["aux 0 0 0 900 2100"]
    adj = [ln for ln in text.split("# adjrange")[1].splitlines()
           if ln.startswith("adjrange ")]
    assert adj == [], "пустые слоты корректировок в diff не печатаются"


# ---------------- стоковая конфигурация Modes (заливка производителя) ----------------

STOCK = [
    {"mode": 0, "aux": 0, "start": 1700, "end": 2100},   # ARM — AUX1
    {"mode": 1, "aux": 1, "start": 1300, "end": 1700},   # ANGLE — AUX2
    {"mode": 2, "aux": 1, "start": 900, "end": 1300},    # HORIZON — AUX2
    {"mode": 3, "aux": 2, "start": 1300, "end": 1700},   # NAV ALTHOLD — AUX3
    {"mode": 10, "aux": 2, "start": 1700, "end": 2100},  # NAV RTH — AUX3
]


def test_boot_stock_mode_ranges(fc):
    for i, item in enumerate(STOCK):
        slot = fc.mode_ranges[i]
        assert (slot["mode"], slot["aux"], slot["start"], slot["end"]) == \
            (item["mode"], item["aux"], item["start"], item["end"]), item
    assert fc.mode_ranges[5]["start"] == 900, "слоты дальше пустые"
    raw = ask(fc, MSP_MODE_RANGES)
    assert list(raw[0:4]) == [0, 0, 32, 48], "ARM виден в MSP сразу"
    assert list(raw[16:20]) == [10, 2, 32, 48], "NAV RTH в 5-м слоте"


def test_cli_shows_stock_modes(cli):
    text = cli.feed(b"aux\r\n").decode()
    assert "aux 0 0 0 1700 2100" in text
    assert "aux 1 1 1 1300 1700" in text
    assert "aux 4 10 2 1700 2100" in text


def test_defaults_restores_stock_modes(cli, fc):
    cli.feed(b"aux 0 0 0 900 2100\r\n")
    cli.feed(b"aux 9 1 0 900 2100\r\n")
    cli.feed(b"defaults noreboot\r\n")
    for i, item in enumerate(STOCK):
        slot = fc.mode_ranges[i]
        assert (slot["mode"], slot["aux"], slot["start"], slot["end"]) == \
            (item["mode"], item["aux"], item["start"], item["end"])
    assert fc.mode_ranges[9]["start"] == 900, "лишние слоты убраны"


# ---------------- старый/новый eeprom и сток ----------------

def test_old_eeprom_all_empty_keeps_stock(tmp_path):
    path = tmp_path / "eeprom.json"
    path.write_text(json.dumps({
        "mode_ranges": [{"id": i, "aux": 0, "start": 900, "end": 900}
                        for i in range(40)],
    }))
    fc2 = InavFC(eeprom_path=path)
    assert fc2.mode_ranges[0]["end"] == 2100, "старый пустой флеш → сток"
    assert fc2.mode_ranges[4]["mode"] == 10


def test_old_eeprom_configured_slots_kept(tmp_path):
    path = tmp_path / "eeprom.json"
    saved = [{"id": i, "aux": 0, "start": 900, "end": 900} for i in range(40)]
    saved[5] = {"id": 5, "aux": 3, "start": 1000, "end": 1500}
    path.write_text(json.dumps({"mode_ranges": saved}))
    fc2 = InavFC(eeprom_path=path)
    assert (fc2.mode_ranges[5]["aux"], fc2.mode_ranges[5]["start"],
            fc2.mode_ranges[5]["end"]) == (3, 1000, 1500), "настройки юзера сохранены"
    assert fc2.mode_ranges[0]["start"] == 1700, "остальные слоты — сток"


def test_new_eeprom_all_empty_keeps_stock(tmp_path):
    path = tmp_path / "eeprom.json"
    saved = [{"id": i, "mode": 0, "aux": 0, "start": 900, "end": 900}
             for i in range(40)]
    path.write_text(json.dumps({"mode_ranges": saved}))
    fc2 = InavFC(eeprom_path=path)
    assert fc2.mode_ranges[0]["start"] == 1700, \
        "пустые слоты не воспроизводятся — стенд оснащён стоком"
    assert fc2.mode_ranges[4]["mode"] == 10


def test_new_eeprom_user_slot_wins_over_stock(tmp_path):
    path = tmp_path / "eeprom.json"
    saved = [{"id": i, "mode": 0, "aux": 0, "start": 900, "end": 900}
             for i in range(40)]
    saved[2] = {"id": 2, "mode": 5, "aux": 3, "start": 1100, "end": 1600}
    path.write_text(json.dumps({"mode_ranges": saved}))
    fc2 = InavFC(eeprom_path=path)
    assert (fc2.mode_ranges[2]["mode"], fc2.mode_ranges[2]["aux"],
            fc2.mode_ranges[2]["start"], fc2.mode_ranges[2]["end"]) == \
        (5, 3, 1100, 1600), "настроенный слот юзера сохраняется"
    assert fc2.mode_ranges[0]["start"] == 1700, "остальные слоты — сток"


def test_old_eeprom_slots_load(tmp_path):
    path = tmp_path / "eeprom.json"
    path.write_text(json.dumps({
        "adjustment_ranges": [{"slot": i, "aux": 0, "start": 900, "end": 900,
                               "function": 0, "switch": 0} for i in range(20)],
    }))
    fc2 = InavFC(eeprom_path=path)
    assert fc2.adjustment_ranges[5]["adjustmentIndex"] == 0
