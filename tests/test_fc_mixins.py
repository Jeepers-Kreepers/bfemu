"""Границы примесей VirtualFC и сохранение единого MSP/CLI API."""
from __future__ import annotations

from collections import Counter

from bfemu import fc as public
from bfemu import fc_constants as constants
from bfemu.fc import VirtualFC


def test_mixins_do_not_shadow_each_other():
    """Новый метод не должен незаметно затереть метод соседнего домена в MRO."""
    owners = Counter(name for base in VirtualFC.__bases__ for name in vars(base)
                     if not name.startswith("__"))
    assert all(count == 1 for count in owners.values())
    assert len(VirtualFC.__bases__) >= 10
    assert VirtualFC._handlers.__module__ == "bfemu.fc"
    assert VirtualFC._h_set_rc_tuning.__module__ == "bfemu.fc_tuning"
    assert VirtualFC._h_set_osd_canvas.__module__ == "bfemu.fc_video"
    assert VirtualFC._h_dataflash_read.__module__ == "bfemu.fc_blackbox"


def test_public_msp_constants_are_reexported():
    for name in ("MSP_STATUS", "MSP_SET_OSD_CANVAS", "MSP_MODE_RANGES", "MSP_DATAFLASH_READ",
                 "MSP2_CLI_COMMAND", "DATA_PATH", "state_based_read_codes"):
        assert getattr(public, name) is getattr(constants, name)


def test_cross_domain_dispatch_and_persistence(fc):
    """MSP -> OSD/режимы -> EEPROM -> новый экземпляр; CLI и телеметрия по-прежнему доступны."""
    assert fc.handle(public.MSP_SET_OSD_CANVAS, bytes((28, 17)), 1) == b""
    assert fc.handle(public.MSP_OSD_CANVAS, b"", 1) == bytes((28, 17))
    assert fc.set_mode_range(0, 0, 0, 1700, 2100)
    fc.cli_execute("set craft_name = MIXINTEST")
    fc.save_to_eeprom()

    restarted = VirtualFC(eeprom_path=fc.eeprom_path, log=lambda *_: None)
    assert restarted.handle(public.MSP_OSD_CANVAS, b"", 1) == bytes((28, 17))
    assert restarted.mode_ranges[0]["start"] == 1700
    assert "MIXINTEST" in restarted.cli_execute("get craft_name")
    assert len(restarted.handle(public.MSP_STATUS, b"", 1)) == 13


def test_rc_api_keeps_state_on_wrong_types(fc):
    """Кривой тип поля от страницы /api/rc раньше ронял обработчик (обрыв соединения).

    Теперь значение просто остаётся прежним: раскладка AUX и карта осей — украшение
    страницы /tx, из-за ошибки в них терять состояние борта нельзя (часть 4 ревью).
    """
    fc.set_rc_aux_map(["AUX1 ← тумблер (2 положения) — ось 3"])
    fc.set_rc_aux_map(5)
    assert fc.rc_aux_map == ["AUX1 ← тумблер (2 положения) — ось 3"]
    assert fc.set_rc_aux_map("AUX2 ← крутилка") == ["AUX2 ← крутилка"], "строка — тоже раскладка"

    fc.set_rc_map_info({"roll": 1, "device": "TX16S"})
    saved = dict(fc.rc_map_info)
    assert saved["roll"] == 1 and saved["device"] == "TX16S"
    fc.set_rc_map_info("нет")                       # не объект
    fc.set_rc_map_info({"roll": "не-число"})        # объект, но ось не разобрать
    assert fc.rc_map_info == saved
