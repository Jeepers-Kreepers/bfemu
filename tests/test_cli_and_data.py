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
    # Значение списка задаётся именем: cliSet() сверяет его через strcasecmp, номер не принимает.
    fc.cli_execute("set gps_provider = MSP")
    gps_line = next(line for line in fc.cli_execute("status").splitlines()
                    if line.startswith("GPS: "))
    assert gps_line.startswith("GPS: connected, MSP,"), gps_line
    fc.cli_execute("set gps_provider = UBLOX")
    gps_line = next(line for line in fc.cli_execute("status").splitlines()
                    if line.startswith("GPS: "))
    assert "UART" in gps_line and "set to" in gps_line, gps_line


# ---------------------------------------------------------------------------------------
# Повторное ревью, часть 5: вывод CLI должен совпадать с cli.c 4.5.4
# (формат ошибок, `set`, `get` без аргументов, строки профилей)

def test_error_lines_end_with_hashes(fc):
    """cliPrintErrorVa() печатает «###ERROR IN <команда>: <текст>###».

    Раньше закрывающих «###» не было. По ним Configurator отличает отказ от обычного
    вывода (в его тестах ответ FC выглядит как «###ERROR IN cli: UNKNOWN COMMAND###»).
    """
    answer = fc.cli_execute("get nosuchsetting")
    assert answer.strip() == "###ERROR IN get: INVALID NAME: nosuchsetting###", answer


def test_set_confirms_the_new_value(fc):
    """cliSet(): успешная запись печатается как «<имя> set to <значение>»."""
    assert fc.cli_execute("set gyro_lpf1_static_hz = 320").strip() == "gyro_lpf1_static_hz set to 320"


def test_set_accepts_name_in_any_case(fc):
    """cliGetSettingIndex(): strncasecmp — регистр имени не важен, но имя целиком."""
    assert "gyro_lpf1_static_hz set to 330" in fc.cli_execute("set GYRO_LPF1_STATIC_HZ = 330")
    # имя должно совпасть целиком: не «префикс», а именно строка настройки
    assert fc.cli_execute("set gyro_lpf1 = 330").strip() == "###ERROR IN set: INVALID NAME: gyro_lpf1 = 330###"


def test_set_rejects_number_for_a_lookup_value(fc):
    """Значение списка задаётся именем: cliSet() сверяет его с таблицей через strcasecmp."""
    before = fc.settings["gyro_lpf1_type"]
    answer = fc.cli_execute("set gyro_lpf1_type = 1")
    assert "###ERROR IN set: INVALID VALUE###" in answer, answer
    assert "Allowed values: PT1, BIQUAD, PT2, PT3" in answer, answer
    assert fc.settings["gyro_lpf1_type"] == before
    assert "gyro_lpf1_type set to BIQUAD" in fc.cli_execute("set gyro_lpf1_type = biquad")


def test_set_failure_prints_the_range(fc):
    """После «INVALID VALUE» прошивка печатает cliPrintVarRange(): границы или имена."""
    answer = fc.cli_execute("set gyro_calib_duration = 10")
    assert answer.splitlines()[1] == "Allowed range: 50 - 3000", answer
    answer = fc.cli_execute("set craft_name = A_TOO_LONG_CRAFT_NAME")
    assert "STRING MUST BE 1-16 CHARACTERS OR '-' FOR EMPTY###" in answer, answer
    assert "String length: 1 - 16" in answer, answer


def test_set_parses_numbers_like_atoi(fc):
    """cliSet() использует atoi()/strtoul(..., 10): «250abc» — это 250, «0x10» — это 0."""
    fc.cli_execute("set gyro_lpf1_static_hz = 250abc")
    assert fc.settings["gyro_lpf1_static_hz"] == 250
    fc.cli_execute("set gyro_lpf1_static_hz = 0x10")
    assert fc.settings["gyro_lpf1_static_hz"] == 0


def test_set_star_appends_limits_to_numeric_settings(fc):
    """cliSet() с «*» зовёт cliPrintVar(..., full=true): у числовых настроек — «min max»."""
    lines = fc.cli_execute("set *").splitlines()
    assert lines[0] == "Current settings: ", lines[0]
    assert "gyro_lpf1_static_hz = 250 0 1000" in lines
    assert "gyro_lpf1_type = PT1" in lines, "у списка границы не печатаются"
    assert len(lines) == len(fc.settings_list) + 1


def test_get_without_arguments_lists_the_whole_table(fc):
    """cliGet(): пустая подстрока есть в любом имени, поэтому `get` печатает все настройки."""
    lines = fc.cli_execute("get").splitlines()
    assert len(lines) > 2 * len(fc.settings_list), "на каждую настройку минимум две строки"
    assert "gyro_lpf1_static_hz = 250" in lines


def test_get_prints_profile_number_for_profile_settings(fc):
    """cliGet() для PROFILE_VALUE зовёт cliProfile(), для рейтов — cliRateProfile()."""
    rate = fc.cli_execute("get roll_rc_rate").splitlines()
    assert rate[1] == "rateprofile 0", rate
    profile = fc.cli_execute("get profile_name").splitlines()
    assert "profile 0" in profile, profile
    fc.cli_execute("profile 2")
    assert "profile 2" in fc.cli_execute("get profile_name").splitlines()
