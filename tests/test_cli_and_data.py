"""CLI-движок, таблица настроек и «флеш».

Главная мысль: эмулятор обещает «все команды прошивки 4.5.4» и «683 реальные настройки».
Эти обещания здесь и проверяются — по скопированным исходникам Betaflight из `docs/`.
"""
from __future__ import annotations

import json
import re

from bfemu.cli import CliEngine

BF_SOURCES = "docs/betaflight-4.5.4"


def firmware_commands(repo_root) -> set[str]:
    source = (repo_root / BF_SOURCES / "cli.c").read_text(encoding="utf-8", errors="replace")
    return set(re.findall(r'CLI_COMMAND_DEF\(\s*"([^"]+)"', source))


def test_all_firmware_cli_commands_are_implemented(repo_root):
    missing = sorted(firmware_commands(repo_root) - set(CliEngine.COMMANDS))
    assert missing == [], f"нет команд прошивки: {missing}"


def test_settings_table_covers_firmware_value_table(repo_root):
    """Все настройки из valueTable[] прошивки должны быть в bf_data.json."""
    source = (repo_root / BF_SOURCES / "cli_settings.c").read_text(encoding="utf-8", errors="replace")
    table = re.search(r"const clivalue_t valueTable\[\]\s*=\s*\{(.*?)\n\};", source, re.S)
    firmware = set(re.findall(r'\{\s*"([A-Za-z_0-9]+)"\s*,', table.group(1)))

    data = json.loads((repo_root / "bfemu" / "data" / "bf_data.json").read_text(encoding="utf-8"))
    ours = {item["name"] for item in data["settings"]}
    assert firmware - ours == set(), f"нет настроек: {sorted(firmware - ours)[:10]}"
    assert len(ours) >= len(firmware)


def test_feature_bits_match_firmware_header(repo_root):
    """FEATURE_* нумеруются с пропусками: сдвиг на единицу включает чужую фичу."""
    header = (repo_root / BF_SOURCES / "feature.h").read_text(encoding="utf-8", errors="replace")
    firmware = {name: int(bit) for name, bit in re.findall(r"FEATURE_([A-Z_0-9]+)\s*=\s*1\s*<<\s*(\d+)", header)}
    data = json.loads((repo_root / "bfemu" / "data" / "bf_data.json").read_text(encoding="utf-8"))
    ours = data["feature_bits"]
    for name, bit in ours.items():
        if name in firmware:
            assert firmware[name] == bit, f"фича {name}: бит {bit}, а в прошивке {firmware[name]}"
    assert ours.get("SOFTSERIAL") == 6
    assert ours.get("GPS") == 7


def test_get_and_set_round_trip(fc):
    assert "250" in fc.cli_execute("get gyro_lpf1_static_hz")
    fc.cli_execute("set gyro_lpf1_static_hz = 320")
    assert "320" in fc.cli_execute("get gyro_lpf1_static_hz")


def test_set_rejects_value_out_of_range(fc):
    before = fc.cli_execute("get gyro_lpf1_static_hz")
    answer = fc.cli_execute("set gyro_lpf1_static_hz = 999999")
    assert "Invalid" in answer or "ERROR" in answer.upper()
    assert fc.cli_execute("get gyro_lpf1_static_hz") == before


def test_string_setting_prints_as_text(fc):
    fc.cli_execute("set craft_name = TESTQUAD")
    assert "craft_name = TESTQUAD" in fc.cli_execute("get craft_name")


def test_unknown_command_answers_like_firmware(fc):
    """cli.c 4.5.4: в неинтерактивном режиме — `ERR_CMD_NA: <строка>`,

    а в интерактивном (с эхом) — `###ERROR IN input: UNKNOWN COMMAND, TRY 'HELP'###`.
    """
    assert fc.cli_execute("no_such_command").startswith("ERR_CMD_NA: no_such_command")

    engine = CliEngine(fc)
    engine.echo = True
    assert "UNKNOWN COMMAND, TRY 'HELP'" in engine.execute_line("no_such_command")


def test_softserial_feature_adds_ports(fc):
    """Как на железе: софт-порты появляются только при включённой фиче."""
    before = [port["identifier"] for port in fc.serial_ports]
    assert 30 not in before and 31 not in before
    fc.cli_execute("feature SOFTSERIAL")
    after = [port["identifier"] for port in fc.serial_ports]
    assert 30 in after and 31 in after
    fc.cli_execute("feature -SOFTSERIAL")
    assert 30 not in [port["identifier"] for port in fc.serial_ports]


def test_eeprom_survives_restart(fc, tmp_path):
    """Сохранённое в «флеш» переживает перезапуск, несохранённое — нет."""
    from bfemu.fc import VirtualFC

    fc.cli_execute("set craft_name = SAVED")
    fc.save_to_eeprom()
    fc.cli_execute("set craft_name = ONLY-IN-RAM")

    restarted = VirtualFC(eeprom_path=fc.eeprom_path, log=lambda *a, **k: None)
    assert "craft_name = SAVED" in restarted.cli_execute("get craft_name")


def test_defaults_wipe_returns_factory_values(fc):
    fc.cli_execute("set craft_name = SOMETHING")
    fc.save_to_eeprom()
    fc.wipe_eeprom()
    from bfemu.fc import VirtualFC

    restarted = VirtualFC(eeprom_path=fc.eeprom_path, log=lambda *a, **k: None)
    assert "craft_name = EMU-QUAD" in restarted.cli_execute("get craft_name")


def test_dump_contains_main_blocks(fc):
    dump = fc.cli_execute("dump")
    for block in ("# version", "# feature", "# master"):
        assert block in dump, f"в dump нет блока {block}"
