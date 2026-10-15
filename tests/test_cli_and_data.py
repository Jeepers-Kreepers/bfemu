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


def test_bat_files_are_crlf_and_ascii(repo_root):
    """Командный файл с LF или кириллицей закрывает окно cmd.exe раньше, чем его прочтут.

    Этот же инвариант проверяет сквозной тест, но здесь он стоит десятки миллисекунд —
    на Windows ошибка видна сразу, а не через полторы минуты.
    """
    beaten = []
    for path in sorted(repo_root.glob("*.bat")):
        data = path.read_bytes()
        if any(byte > 127 for byte in data):
            beaten.append(f"{path.name}: не-ASCII байты")
        if data.count(b"\n") != data.count(b"\r\n"):
            beaten.append(f"{path.name}: строки без CR")
    assert not beaten, "; ".join(beaten)


def test_ci_workflow_forces_utf8_output(repo_root):
    """Сценарий CI обязан переводить вывод в UTF-8.

    Без этого шаг на windows-latest падает с UnicodeEncodeError на первом же русском
    слове: Python пишет stdout в кодировке системной кодовой страницы (cp1252).
    Проверка дешёвая, а ошибка воспроизводится только в облаке — ловим локально.
    """
    workflow = (repo_root / "ci" / "github-actions-ci.yml").read_text(encoding="utf-8")
    assert "PYTHONUTF8" in workflow and "PYTHONIOENCODING" in workflow


def test_ci_workflow_calls_only_existing_checks(repo_root):
    """Рабочий процесс вызывает только те проверки, которые есть в `ci/checks.py`.

    Опечатка в имени проверки иначе всплыла бы только в облаке: шаг упал бы с
    «invalid choice» через минуту после начала прогона.
    """
    import re
    import sys

    sys.path.insert(0, str(repo_root / "ci"))
    try:
        import checks
    finally:
        sys.path.pop(0)

    workflow = (repo_root / "ci" / "github-actions-ci.yml").read_text(encoding="utf-8")
    called = {
        name
        for call in re.findall(r"python ci/checks\.py ([a-z ]+)", workflow)
        for name in call.split()
    }
    assert called, "в сценарии нет ни одного вызова ci/checks.py"
    unknown = called - set(checks.CHECKS) - {"all"}
    assert not unknown, f"сценарий зовёт несуществующие проверки: {sorted(unknown)}"
    assert set(checks.ORDER) == set(checks.CHECKS), "ORDER и CHECKS разошлись"


def test_status_shows_gps_only_with_the_feature(fc):
    """Раньше `status` печатал «GPS: connected» даже без фичи GPS.

    В cli.c строка строится под featureIsEnabled(FEATURE_GPS), иначе «NOT ENABLED» — иначе
    человек видел приёмник, которого прошивка не использует (повторное ревью, часть 4).
    """
    assert "GPS: connected" in fc.cli_execute("status"), "по умолчанию GPS включён фичей"
    fc.cli_execute("feature -GPS")
    status = fc.cli_execute("status")
    assert "GPS: NOT ENABLED" in status and "GPS: connected" not in status


def test_status_prints_port_and_msp_provider_like_firmware(fc):
    """У GPS по MSP нет UART — прошивка печатает «GPS: connected, MSP, …» (cli.c)."""
    fc.cli_execute("set gps_provider = 2")
    gps_line = next(line for line in fc.cli_execute("status").splitlines()
                    if line.startswith("GPS: "))
    assert gps_line.startswith("GPS: connected, MSP,"), gps_line
    fc.cli_execute("set gps_provider = 1")
    gps_line = next(line for line in fc.cli_execute("status").splitlines()
                    if line.startswith("GPS: "))
    assert "UART" in gps_line and "set to" in gps_line, gps_line
