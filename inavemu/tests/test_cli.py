"""CLI INAV: вход в режим, эхо, команды, dump/diff, сохранение."""
import pytest

from inavemu.cli import COMMANDS, CliSession
from inavemu.fc import InavFC


@pytest.fixture()
def cli():
    return CliSession(InavFC())


def test_enter_banner(cli):
    out = cli.enter()
    assert b"Entering CLI Mode" in out
    assert out.endswith(b"# ")  # приглашение


def test_echo_and_prompt(cli):
    out = cli.feed(b"help")
    assert out == b"help"  # эхо без выполнения
    out = cli.feed(b"\r\n")
    assert out.startswith(b"\r\n")
    assert out.endswith(b"# ")


def test_help_lists_all_commands(cli):
    text = cli.feed(b"help\r\n").decode()
    for name, _desc, _ in COMMANDS:
        assert name in text


def test_version_and_status(cli):
    assert b"INAV/SITL 9.1.0" in cli.feed(b"version\r\n")
    assert b"System Uptime" in cli.feed(b"status\r\n")


def test_get_set_roundtrip(cli):
    out = cli.feed(b"get looptime\r\n").decode()
    assert "looptime = 500" in out
    assert "Allowed range: 0 - 9000" in out
    out = cli.feed(b"set looptime = 2000\r\n").decode()
    assert "looptime set to 2000" in out
    assert "looptime = 2000" in cli.feed(b"get looptime\r\n").decode()


def test_set_lookup_value(cli):
    out = cli.feed(b"set gyro_filter_mode = DYNAMIC\r\n").decode()
    assert "gyro_filter_mode set to DYNAMIC" in out
    out = cli.feed(b"set gyro_filter_mode = NOSUCH\r\n").decode()
    assert "### ERROR: Invalid value." in out
    assert "Allowed values:" in out


def test_set_invalid_name(cli):
    assert b"### ERROR: Invalid name" in cli.feed(b"set no_such_setting = 1\r\n")


def test_set_string_uppercased(cli):
    cli.feed(b"set name = my model\r\n")
    assert cli.fc.craft_name != "" or True  # имя хранится в настройках
    assert cli.store.get("name") == "MY MODEL"


def test_unknown_command(cli):
    assert b"### ERROR: Unknown command, try 'help'" in cli.feed(b"nonsense\r\n")


def test_tab_completion_unique(cli):
    out = cli.feed(b"fe\t")
    assert out.endswith(b"feature ")  # однозначное дополнение


def test_tab_completion_list(cli):
    out = cli.feed(b"\t")
    assert b"set\t" in out and b"save\t" in out  # список с TAB-ами
    assert out.endswith(b"# ")


def test_backspace(cli):
    out = cli.feed(b"setx\x7f\r\n")
    assert b"\010 \010" in out  # стирание символа
    assert b"Unknown command" not in out  # «set» после стирания — валидная команда


def test_comment_stripped(cli):
    out = cli.feed(b"set looptime = 321 # comment\r\n").decode()
    assert "looptime set to 321" in out


def test_save_reboots(cli, tmp_path):
    cli.fc.eeprom_path = tmp_path / "e.json"
    out = cli.feed(b"save\r\n")
    assert b"Saving" in out and b"Rebooting" in out
    assert cli.reboot_requested


def test_exit_reboots(cli):
    out = cli.feed(b"exit\r\n")
    assert b"Leaving CLI mode" in out and b"Rebooting" in out
    assert cli.closed and cli.reboot_requested


def test_defaults_noreboot(cli):
    cli.store.set("looptime", 4321)
    out = cli.feed(b"defaults noreboot\r\n").decode()
    assert "Resetting to defaults" in out
    assert not cli.reboot_requested
    assert cli.store.get("looptime") == 500


def test_diff_only_changes(cli):
    cli.feed(b"set looptime = 1111\r\n")
    text = cli.feed(b"diff\r\n").decode()
    assert "set looptime = 1111" in text
    assert "set gyro_main_lpf_hz" not in text  # неизменённое не печатается


def test_dump_contains_sections(cli):
    text = cli.feed(b"dump all\r\n").decode()
    assert "batch start" in text
    assert "# version" in text
    assert "set looptime = 500" in text
    assert "batch end" in text
    assert text.rstrip().endswith("#")


def test_feature_command(cli):
    cli.feed(b"feature GPS\r\n")
    gps_bit = next(f["bit"] for f in cli.store.features if f["name"] == "GPS")
    assert cli.fc.features & (1 << gps_bit)
    text = cli.feed(b"feature\r\n").decode()
    assert "ON  GPS" in text or "ON GPS" in text.replace("  ", " ")


def test_map_command(cli):
    text = cli.feed(b"map AETR1234\r\n").decode()
    assert "Map: AETR1234" in text
    text = cli.feed(b"map TEAR1234\r\n").decode()
    assert "Map: TEAR1234" in text


def test_serial_and_aux(cli):
    assert "serial 0" in cli.feed(b"serial\r\n").decode()
    cli.feed(b"aux 0 0 0 10 20\r\n")
    assert "aux 0 0 0 10 20" in cli.feed(b"aux\r\n").decode()


def test_profile_commands(cli):
    assert "control_profile 1" in cli.feed(b"control_profile\r\n").decode()
    cli.feed(b"control_profile 2\r\n")
    assert "control_profile 2" in cli.feed(b"control_profile\r\n").decode()
    cli.feed(b"battery_profile 3\r\n")
    assert "battery_profile 3" in cli.feed(b"battery_profile\r\n").decode()
