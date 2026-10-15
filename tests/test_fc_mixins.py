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
