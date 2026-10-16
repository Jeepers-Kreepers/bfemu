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


def test_set_without_equals_reads_the_setting(fc):
    """cliSet(): ветка «без =» зовёт cliGet() — `set <имя>` это чтение, а не отказ.

    Прошивка (cli.c 4.5.4, конец cliSet): `} else { cliGet(cmdName, cmdline); }`. Эмулятор
    отвечал на это `INVALID NAME`, из-за чего Configurator и скрипты, читавшие настройку
    через `set`, видели ошибку вместо значения.
    """
    assert fc.cli_execute("set gyro_lpf1_static_hz") == fc.cli_execute("get gyro_lpf1_static_hz")
    # подстрока, как в strcasestr(): совпадений может быть несколько
    many = fc.cli_execute("set gyro_lpf1")
    assert many.startswith("gyro_lpf1_type =") and "gyro_lpf1_dyn_min_hz =" in many, many


def test_set_without_equals_reports_the_set_command(fc):
    """В ветке чтения cmdName остаётся «set» — так и печатается имя команды в ошибке."""
    assert fc.cli_execute("set nosuchthing").strip() == "###ERROR IN set: INVALID NAME: nosuchthing###"
    assert fc.cli_execute("get nosuchthing").strip() == "###ERROR IN get: INVALID NAME: nosuchthing###"


def test_set_without_arguments_prints_the_whole_table(fc):
    """len == 0 в cliSet() — «Current settings: » и таблица (без границ: они только у `set *`)."""
    answer = fc.cli_execute("set")
    lines = answer.splitlines()
    assert lines[0] == "Current settings: ", repr(lines[0])
    assert lines[1].startswith(fc.settings_list[0]["name"] + " = "), lines[1]
    assert len(lines) == len(fc.settings_list) + 1, len(lines)
    full = fc.cli_execute("set *").splitlines()
    assert len(full) == len(fc.settings_list) + 1
    assert " 0 1000" in full[3] or any(line.endswith(" 0 1000") for line in full), full[:5]


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


# ---------------------------------------------------------------------------------------
# Повторное ревью, часть 5: заводские значения из reset-шаблонов прошивки
# ---------------------------------------------------------------------------------------

# Значения, которые `tools/gen_data.py` раньше не находил (reset-функции через
# RESET_CONFIG/RESET_CONFIG_2), а эмулятор выводил из своих «эвристик». Все — из
# pgResetFn_*/PG_RESET_TEMPLATE 4.5.4; в скобках — место в исходниках прошивки.
FIRMWARE_DEFAULTS = {
    # flight/pid.c:123 resetPidProfile()
    "iterm_windup": 85, "iterm_limit": 400, "pidsum_limit": 500, "pidsum_limit_yaw": 400,
    "dyn_idle_p_gain": 50, "dyn_idle_i_gain": 50, "dyn_idle_d_gain": 50,
    "dyn_idle_max_increase": 150, "dyn_idle_start_increase": 50,
    "acro_trainer_lookahead_ms": 50, "acro_trainer_gain": 75, "angle_earth_ref": 100,
    "horizon_limit_degrees": 135, "horizon_delay_ms": 500, "abs_control_limit": 90,
    "abs_control_error_limit": 20, "abs_control_cutoff": 11, "launch_control_gain": 40,
    "launch_trigger_throttle_percent": 20, "launch_trigger_allow_reset": 1,
    "throttle_boost_cutoff": 15, "pid_at_min_throttle": 1, "anti_gravity_p_gain": 100,
    "anti_gravity_cutoff_hz": 5, "angle_feedforward_smoothing_ms": 80,
    # flight/imu.c:127 imuConfig
    "acc_lpf_hz": 25, "small_angle": 25,
    # fc/controlrate_profile.c:45, flight/servos.c:90
    "tpa_low_rate": 20, "tpa_low_breakpoint": 1050, "ez_landing_threshold": 25,
    "ez_landing_limit": 15, "ez_landing_speed": 50,
    # pg/rx.c:80 pgResetFn_rxConfig (RESET_CONFIG_2)
    "max_aux_channels": 14, "rssi_scale": 100, "rssi_src_frame_lpf_period": 30,
    "rssi_smoothing": 125, "rc_smoothing_auto_factor_throttle": 30, "srxl2_unit_id": 1,
    "srxl2_baud_fast": 1, "serialrx_halfduplex": 1, "spektrum_sat_bind_autoreset": 1,
    "max_check": 1900, "mid_rc": 1500, "min_check": 1050, "airmode_start_throttle_percent": 25,
    # flight/crash_recovery.c
    "crash_dthreshold": 50, "crash_gthreshold": 400, "crash_setpoint_threshold": 350,
    "crash_time": 500, "crash_recovery_angle": 10, "crash_recovery_rate": 100,
    "crash_limit_yaw": 200,
    # PG-настройки под #if/#else: ветка стоковой цели без RACE_PRO, не F411, без
    # BEEPER_INVERTED (imu.c:119-121, pg/scheduler.h, pg/beeper_dev.c, pg/sdcard.c,
    # pg/rx_spi_cc2500.c, drivers/stm32/platform_mcu.h)
    "cpu_overclock": 0, "scheduler_relax_rx": 25, "scheduler_relax_osd": 25,
    "beeper_od": 1, "beeper_inversion": 0, "sdcard_detect_inverted": 0,
    "cc2500_spi_chip_detect": 1,
}

# Настройки, которые эмулятор намеренно держит не заводскими (виртуальное железо).
DOCUMENTED_DEVIATIONS = {
    "gps_ublox_use_galileo", "gps_set_home_point_once",   # уходят в MSP_GPS_CONFIG
    "battery_meter", "current_meter",                     # ADC-датчики виртуальной платы
    "bat_capacity",                                       # ёмкость для симуляции разряда
    "blackbox_device",                                    # виртуальный накопитель
}


def load_data(repo_root) -> dict:
    return json.loads((repo_root / "bfemu" / "data" / "bf_data.json").read_text())


def load_overrides(repo_root) -> dict:
    return json.loads((repo_root / "bfemu" / "data" / "fc_overrides.json").read_text())


def test_factory_values_come_from_reset_templates(repo_root):
    """Заводские значения — из pgResetFn_*/PG_RESET_TEMPLATE, а не из догадок.

    Генератор не разбирал reset-функции через RESET_CONFIG/RESET_CONFIG_2 (rxConfig и др.)
    и брал первую ветку `#if`, поэтому 363 настройки оставались без значения, а часть
    (small_angle, cpu_overclock, beeper_od…) получала не ту ветку. Здесь они сверяются
    со значениями прошивки 4.5.4 — по именам из `docs/betaflight-4.5.4`.
    """
    data = {s["name"]: s for s in load_data(repo_root)["settings"]}
    wrong = {n: data[n]["default"] for n, want in FIRMWARE_DEFAULTS.items()
             if data[n]["default"] != want}
    assert wrong == {}, f"расходится с прошивкой: {wrong}"
    missing = [n for n in FIRMWARE_DEFAULTS if data[n]["default"] is None]
    assert missing == [], f"нет заводского значения: {missing}"


def test_most_settings_have_a_factory_value(repo_root):
    """После разбора reset-шаблонов заводских значений должно быть заметно больше 320."""
    settings = load_data(repo_root)["settings"]
    known = [s for s in settings if s["default"] is not None]
    assert len(known) >= 450, f"заводских значений всего {len(known)}/{len(settings)}"


def test_overrides_only_touch_existing_and_documented_settings(repo_root):
    """fc_overrides.json — только существующие настройки и только документированные отклонения.

    В файле было 95 ключей: 25 на имена, которых в 4.5.4 нет вовсе (`midrc`, `maxcheck`,
    `minthrottle`… — это имена полей структур, а не настройки CLI), ещё 40 дублировали
    заводские значения из bf_data.json, а три (`max_check` 2000, `airmode_start_throttle_percent`
    30, `blackbox_sample_rate` 1/2) расходились с прошивкой. Теперь файл описывает только
    то, что нельзя вывести из исходников.
    """
    data = {s["name"]: s for s in load_data(repo_root)["settings"]}
    overrides = load_overrides(repo_root)

    assert set(overrides) - set(data) == set(), "ключи без такой настройки"
    for name, value in overrides.items():
        if name in DOCUMENTED_DEVIATIONS or data[name]["default"] is None:
            continue
        expected = value
        values = data[name].get("values") or []
        if isinstance(value, str) and value in values:
            expected = values.index(value)
        assert data[name]["default"] == expected, (
            f"{name}: override {value!r} не совпадает с заводским "
            f"{data[name]['default']!r} и не объявлен в DOCUMENTED_DEVIATIONS")


def test_lookup_tables_have_no_header_names(repo_root):
    """В таблицах имён не должно быть имён включённых файлов.

    `batteryStateStrings` объявлена одной строкой, а разбор ждал `};` на следующих строках
    и затягивал в значения имена последующих `#include` («platform.h», «common/maths.h»…).
    """
    tables = load_data(repo_root)["lookup_tables"]
    junk = {name: values for name, values in tables.items()
            if any(v.endswith(".h") for v in values)}
    assert junk == {}, f"в таблицах имена файлов: {junk}"
    assert tables["batteryStateStrings"] == ["OK", "WARNING", "CRITICAL", "NOT PRESENT", "INIT"]


def test_defaults_reset_uses_the_factory_values(fc):
    """`defaults` возвращает именно заводские значения прошивки."""
    fc.cli_execute("set small_angle = 45")
    fc.cli_execute("set dyn_idle_p_gain = 20")
    fc.reset_to_defaults()
    assert fc.settings["small_angle"] == 25, "imuConfig.small_angle = DEFAULT_SMALL_ANGLE (25)"
    assert fc.settings["dyn_idle_p_gain"] == 50, "resetPidProfile(): dyn_idle_p_gain = 50"
    assert fc.settings["max_check"] == 1900, "rxConfig.maxcheck = 1900"
    assert fc.settings["cpu_overclock"] == 0, "DEFAULT_CPU_OVERCLOCK = 0 для F40X"
